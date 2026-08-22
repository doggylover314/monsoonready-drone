#!/usr/bin/env python3
"""Talk to a SiK telemetry radio over its own serial port.

    ./python tools/sik.py probe                    # sweep bauds, report both ends
    ./python tools/sik.py probe --port /dev/ttyUSB0
    ./python tools/sik.py factory                  # AT&F + AT&W + ATZ, local radio
    ./python tools/sik.py set S2 64                # one register, then AT&W + ATZ
    ./python tools/sik.py sweep-air                # hunt the far radio's AIR_SPEED

Parameters are addressed by REGISTER NUMBER (S0, S1, S2 ...), not by name.
`probe` dumps ATI5, which prints each register's number and its name side by
side, so read the mapping off your own radio rather than off a table
somewhere: the numbering has not been verified from SiK source here and it is
not worth guessing at.

WHY THIS EXISTS (2026-08-22): QGC flashed the GROUND radio and then stopped
detecting it. A SiK firmware upgrade RESETS the radio's parameters to the new
firmware's defaults, and defaults have changed across SiK versions, so a
one-sided flash can leave the two ends on different SERIAL_SPEED, AIR_SPEED or
NETID. Healthy radios that disagree on any of those simply never link, which
looks identical to a dead radio from the ground station.

This is a radio tool only. It never touches the autopilot, so it does not
belong in parameters.py (params), bench.py (probes) or mavlink_link.py (the
MAVLink link library).

HOW SiK COMMAND MODE WORKS, and why the timing below is not arbitrary: the
radio watches its serial stream for `+++` surrounded by at least one second of
silence on both sides. Send it too soon after other traffic, or follow it with
a newline, and the radio passes it through as data instead of entering command
mode. Once in, `ATI` returns the version, `ATI5` dumps every parameter, and
the `RT` forms of the same commands run on the REMOTE radio through the link.
RT commands only work when the link is already up, which is exactly why they
cannot rescue a mismatched pair.

FIRMWARE CANNOT BE LOADED FROM HERE. SiK firmware goes in through the
bootloader over a direct local serial connection, which is what QGC does. No
AT command uploads firmware and neither does this tool.
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing: ./pip install pyserial")

# Ordered by how likely each is on this project's radios. 57600 is the SiK
# default and what QGC tries first; 115200 shows up on radios someone has
# reconfigured for a faster host link.
BAUDS = [57600, 115200, 38400, 19200, 9600, 230400]

# AIR_SPEED values the firmware accepts, in kbps. Both ends must agree exactly.
AIR_SPEEDS = [2, 4, 8, 16, 19, 24, 32, 48, 64, 96, 128, 192, 250]

GUARD_S = 1.2          # silence either side of +++, spec minimum is 1.0


def find_port():
    """A SiK ground radio shows up as a USB-serial bridge. So does an ESP32,
    so print what was found rather than silently picking."""
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
    """Get the radio into AT command mode, or confirm it is already there.

    ASK BEFORE KNOCKING. A radio ALREADY in command mode does not answer `+++`
    with OK, it answers with nothing, so a tool that only knows how to knock
    reports a healthy radio as dead. That is a real bug this file shipped with
    on 2026-08-22: `probe` succeeded, left the radio in command mode, and the
    very next `sweep-air` said "does not answer at any baud". So try a plain
    ATI first and treat a version string as proof we are already in.
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
    """ATO returns the radio to passing data. WITHOUT THIS THE RADIO STAYS IN
    COMMAND MODE AND CARRIES NO MAVLINK, so a diagnostic run would leave QGC
    with a link that enumerates and never talks. Shipped missing, same day."""
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
    """Open at one baud and try to reach command mode. Caller closes."""
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as exc:
        # NEVER quiet. "Device or resource busy" means QGC or another shell
        # holds the port, which is a completely different problem from a
        # silent radio, and hiding it behind --quiet sends the reader off
        # chasing a dead radio that is fine.
        busy = 'busy' in str(exc).lower()
        print(f"  {baud:>6}: cannot open ({exc})")
        if busy:
            print("       ^ something else has this port open. Close QGC, or "
                  "any other program holding the radio, and re-run.")
        return None
    if command_mode(ser):
        if not quiet:
            # This line is a SUCCESS, not a symptom. Command mode is where AT
            # commands are answered; the tool puts the radio back into data
            # mode on the way out. An earlier wording read like a fault report
            # and sent the user power-cycling a radio that was working.
            print(f"  {baud:>6}: radio answers here")
        return ser
    if not quiet:
        print(f"  {baud:>6}: no answer")
    ser.close()
    return None


def probe(port):
    print(f"\nsweeping bauds on {port} (about {GUARD_S * 2:.1f}s each)")
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
    # A remote dump is sixteen lines fetched over the AIR link, not over USB,
    # and with ECC on the air link carries about half its nominal rate. 1.5 s
    # was a guess and it was too short: on 2026-08-23 the ground radio showed
    # a SOLID green LED, meaning linked, while this same call reported "no
    # answer". Three attempts at 4 s, and the verdict below now defers to the
    # LED rather than pretending a silent RTI5 proves anything.
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
    """One register's value, or None. ATSn? echoes just the number."""
    for line in at(ser, f'ATS{n}?').splitlines():
        line = line.strip()
        if line.isdigit():
            return int(line)
    return None


def sweep_air(port):
    """Walk the local radio's AIR_SPEED until the remote answers.

    NO CONFIRMATION PROMPT (user, 2026-08-22: "why do you need a confirmation
    for the air-sweep? It is nothing dangerous at all"). He is right, it is a
    reversible radio setting. The prompt was friction pretending to be safety.
    What it should have done instead, and now does, is PUT THE SETTING BACK
    when the sweep finds nothing: the first version walked to the end of the
    list and abandoned the radio on AIR_SPEED 250, which is not where it
    started and not a value anything else on this project uses.
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
        # A SiK pair does not link the instant both ends boot: they hop a
        # shared channel sequence and have to find each other. Asking RTI too
        # early reports "nothing" for a pair that would have linked a second
        # later, which turns the whole sweep into a false negative.
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
