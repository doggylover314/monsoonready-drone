#!/usr/bin/env python3
"""Talk to a SiK telemetry radio over its own serial port.

    ./python tools/sik.py probe                    # sweep bauds, report both ends
    ./python tools/sik.py probe --port /dev/ttyUSB0
    ./python tools/sik.py factory                  # AT&F + AT&W + ATZ, local radio
    ./python tools/sik.py set S2 64                # one register, then AT&W + ATZ
    ./python tools/sik.py sweep-air                # hunt the far radio's AIR_SPEED

Parameters are addressed by register number (S0, S1, S2, ...). Run `probe`
first to dump ATI5 with the register mapping; never guess at a number.

A radio-only tool, kept separate from parameters.py, bench.py and mavlink_link.py.

Command mode: the radio watches for `+++` between two silences of at least a
second each. ATI and ATI5 run locally; the RT forms run on the far radio, but
only over an established link, so they cannot rescue a mismatched pair.
Firmware flashes need the bootloader over a direct serial connection; QGC
handles firmware, this tool does not.

A firmware upgrade resets the radio to the new image's defaults, so the two
ends can end up disagreeing on SERIAL_SPEED, AIR_SPEED or NETID. A mismatch
kills the link and, without checking the LEDs, looks exactly like a dead
radio.
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing: ./pip install pyserial")

# Ordered by likelihood: 57600 is the SiK default, 115200 shows up on
# reconfigured host links.
# Kept identical to sik_config.py: see the note there.
BAUDS = [57600, 115200, 38400, 19200, 9600, 230400, 250000]

# AIR_SPEED values the firmware accepts, in kbps. Both ends must agree exactly.
AIR_SPEEDS = [2, 4, 8, 16, 19, 24, 32, 48, 64, 96, 128, 192, 250]

GUARD_S = 1.2          # silence either side of +++, spec minimum is 1.0


def find_port():
    """Find the SiK radio's USB-serial bridge. Prints the candidates so it is never confused with the ESP32."""
    cands = [p for p in list_ports.comports()
             if 'USB' in p.device or 'ACM' in p.device]
    if not cands:
        sys.exit("no USB serial port found. Is the radio plugged in?")
    if len(cands) > 1:
        print("more than one USB serial port present:")
        for p in cands:
            print(f"  {p.device}  {p.description}")
        sys.exit("pick one with --port")
    print(f"using {cands[0].device} ({cands[0].description})")
    return cands[0].device


def command_mode(ser):
    """Enter command mode, or confirm the radio is already there.

    A radio already in command mode does not answer `+++` with OK, it gives
    no response at all, so check with ATI first to catch that case.
    """
    ser.reset_input_buffer()
    ser.write(b'\r\nATI\r\n')
    ser.flush()
    time.sleep(0.5)
    if b'SiK' in ser.read(ser.in_waiting or 128):
        return True
    ser.reset_input_buffer()
    time.sleep(GUARD_S)
    ser.write(b'+++')
    ser.flush()
    time.sleep(GUARD_S)
    return b'OK' in ser.read(ser.in_waiting or 64)


def leave_command_mode(ser):
    """Exit command mode with ATO; without it the radio stays in command mode and carries no MAVLink."""
    try:
        ser.write(b'ATO\r\n')
        ser.flush()
        time.sleep(0.3)
    except serial.SerialException:
        pass
    ser.close()


def at(ser, cmd, wait=0.6):
    ser.reset_input_buffer()
    ser.write(cmd.encode() + b'\r\n')
    ser.flush()
    time.sleep(wait)
    return ser.read(ser.in_waiting or 4096).decode('ascii', 'replace')


def open_at(port, baud, quiet=False):
    """Open the port at one baud and try to reach command mode. The caller closes it."""
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as exc:
        # Do not silence "busy" errors: the root cause differs from a radio
        # that is simply not answering. Hiding it would send the search
        # toward a dead radio when the real problem is the port already
        # being held open.
        busy = 'busy' in str(exc).lower()
        print(f"  {baud:>6}: cannot open ({exc})")
        if busy:
            print("       ^ something else has this port open. Close QGC, or "
                  "any other program holding the radio, and re-run.")
        return None
    if command_mode(ser):
        if not quiet:
            print(f"  {baud:>6}: radio answers here")
        return ser
    if not quiet:
        print(f"  {baud:>6}: no answer")
    ser.close()
    return None


def probe(port):
    print(f"\nsweeping bauds on {port} (about {GUARD_S * 2 + 0.5:.1f}s each)")
    ser = None
    for baud in BAUDS:
        ser = open_at(port, baud)
        if ser:
            break
    if not ser:
        print("\nNO ANSWER AT ANY BAUD. The USB bridge enumerates but the "
              "radio behind it is not running firmware that talks. Most "
              "likely it is still sitting in the bootloader after the flash, "
              "so power-cycle it (unplug and replug) and probe again. If it "
              "still says nothing, re-run the QGC firmware upgrade: the "
              "bootloader is reachable even when the application is not.")
        return 1

    print("\nLOCAL RADIO")
    print(at(ser, 'ATI').strip())
    local = at(ser, 'ATI5', wait=1.0)
    print(local.strip())

    print("\nREMOTE RADIO (through the link)")
    # A remote dump over the air link with ECC on is slow: 4s timeout per
    # attempt, 3 retries.
    remote = ''
    for _ in range(3):
        remote += at(ser, 'RTI5', wait=4.0) or ''
        if 'S0' in remote or 'S1' in remote:
            break
    if 'S0' in remote or 'S1' in remote:
        print(remote.strip())
        print("\nBOTH ENDS ANSWERED, so the link is up and the radios agree "
              "on NETID and AIR_SPEED. Compare the two dumps above: "
              "SERIAL_SPEED may differ without breaking the link, everything "
              "else must match.")
    else:
        print("  no answer from the far radio.")
        print("\nCHECK THE GREEN LED BEFORE BELIEVING THAT. Solid green means "
              "the radio HAS a link and this tool simply did not get a remote "
              "dump back, which is a different and much smaller problem: "
              "remote AT commands are slow, and they are slower again with "
              "ECC on. If it is solid, ignore this section and just point "
              "QGC at the radio, because a working MAVLink link is the actual "
              "goal and RTI5 is only a proxy for it.")
        print("\nBlinking green on BOTH ends is what a real mismatch looks "
              "like. NETID, AIR_SPEED and ECC must be identical, as must "
              "MIN_FREQ, MAX_FREQ and NUM_CHANNELS; SERIAL_SPEED may differ. "
              "Register numbers are in the dump above. After a firmware "
              "upgrade the flashed end sits on the new firmware's defaults "
              "while the other end keeps whatever it always had.")
    leave_command_mode(ser)
    return 0


def read_reg(ser, n):
    """One register's value, or None: ATSn? echoes just the number."""
    for line in at(ser, f'ATS{n}?').splitlines():
        line = line.strip()
        if line.isdigit():
            return int(line)
    return None


def sweep_air(port):
    """Sweep the local radio's AIR_SPEED until the remote radio answers.

    No confirmation prompt, since the setting is reversible; restores the
    original value on failure.
    """
    print("\nfinding a baud that reaches the local radio")
    ser = None
    for baud in BAUDS:
        ser = open_at(port, baud, quiet=True)
        if ser:
            print(f"  local radio answers at {baud}")
            break
    if not ser:
        sys.exit("local radio does not answer at any baud; run probe first")
    baud = ser.baudrate

    original = read_reg(ser, 2)
    print(f"  AIR_SPEED starts at {original}; it goes back there if nothing "
          f"links")

    for sp in AIR_SPEEDS:
        at(ser, f'ATS2={sp}')
        at(ser, 'AT&W')
        at(ser, 'ATZ', wait=2.0)
        ser.close()
        # The SiK pair hops its channel sequence to resync; sleep 3s to let
        # the link form before querying it.
        time.sleep(3.0)
        ser = open_at(port, baud, quiet=True)
        if not ser:
            print(f"  AIR_SPEED {sp:>3}: lost the local radio, stopping")
            return 1
        remote = at(ser, 'RTI', wait=2.0) or ''
        if 'SiK' not in remote:
            remote += at(ser, 'RTI', wait=2.0) or ''
        hit = 'SiK' in remote
        print(f"  AIR_SPEED {sp:>3}: {'LINK UP' if hit else 'nothing'}")
        if hit:
            print(f"\nFOUND IT. Both radios are on AIR_SPEED {sp}. Leaving it "
                  f"here. Change the far end with the RT form first if you "
                  f"want a different value on both.")
            leave_command_mode(ser)
            return 0

    if original is not None:
        at(ser, f'ATS2={original}')
        at(ser, 'AT&W')
        at(ser, 'ATZ', wait=2.0)
        print(f"\nput AIR_SPEED back to {original}")
    ser.close()
    print("\nNO AIR_SPEED LINKED, so AIR_SPEED is not the difference and the "
          "search has to move off this end. In rough order of likelihood:\n"
          "  1. The far radio has no power, or not enough. Look at its LEDs: "
          "no light at all means no power, a blinking green means it is "
          "powered and searching, solid green means linked. That one look "
          "settles more than any sweep.\n"
          "  2. NETID differs. It cannot be swept blind, the space is 0-499.\n"
          "  3. MIN_FREQ, MAX_FREQ or NUM_CHANNELS differ. These must match "
          "as exactly as NETID does and a firmware change can move them.\n"
          "Fixing 2 and 3 without a link means reaching the far radio "
          "directly, over the Pixhawk's serial passthrough or on a bench "
          "adapter, and setting both ends to the same values.")
    return 1


def write_one(port, name, value):
    ser = None
    for baud in BAUDS:
        ser = open_at(port, baud, quiet=True)
        if ser:
            break
    if not ser:
        sys.exit("radio does not answer at any baud; run probe first")
    before = at(ser, 'ATI5', wait=1.0)
    print(f"setting {name} to {value}")
    print(at(ser, f'ATS{name}={value}' if name.isdigit()
             else f'AT{name}={value}').strip())
    print(at(ser, 'AT&W').strip())
    at(ser, 'ATZ', wait=2.0)
    ser.close()   # ATZ already rebooted it out of command mode
    print("\nwritten and rebooted. Before:")
    print(before.strip())
    print("\nre-run probe to read it back. A write you have not read back is "
          "not a write you know happened.")
    return 0


def factory(port):
    ser = None
    for baud in BAUDS:
        ser = open_at(port, baud, quiet=True)
        if ser:
            break
    if not ser:
        sys.exit("radio does not answer at any baud; run probe first")
    print("\nAT&F resets THIS radio to firmware defaults. The far radio keeps "
          "whatever it has, so if they were paired on custom settings this "
          "will break the pair until the far end is reset too.")
    if input("type 'factory' to continue: ").strip().lower() != 'factory':
        leave_command_mode(ser)
        sys.exit("aborted, nothing written")
    print(at(ser, 'AT&F').strip())
    print(at(ser, 'AT&W').strip())
    at(ser, 'ATZ', wait=2.0)
    ser.close()   # ATZ already rebooted it out of command mode
    print("reset and rebooted. Run probe to see the defaults it landed on.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['probe', 'factory', 'set', 'sweep-air'])
    ap.add_argument('name', nargs='?')
    ap.add_argument('value', nargs='?')
    ap.add_argument('--port', default=None,
                    help='serial device; omit to auto-pick when exactly one '
                         'USB serial port is present')
    args = ap.parse_args()

    port = args.port or find_port()
    if args.cmd == 'probe':
        return probe(port)
    if args.cmd == 'factory':
        return factory(port)
    if args.cmd == 'sweep-air':
        return sweep_air(port)
    if not args.name or args.value is None:
        sys.exit("set needs a name and a value, e.g. set S2 64")
    return write_one(port, args.name, args.value)


if __name__ == '__main__':
    sys.exit(main())
