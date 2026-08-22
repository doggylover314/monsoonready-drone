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
    """Return True if the radio answered +++ with OK."""
    ser.reset_input_buffer()
    time.sleep(GUARD_S)
    ser.write(b'+++')
    ser.flush()
    time.sleep(GUARD_S)
    reply = ser.read(ser.in_waiting or 64)
    return b'OK' in reply


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
        if not quiet:
            print(f"  {baud:>6}: cannot open ({exc})")
        return None
    if command_mode(ser):
        if not quiet:
            print(f"  {baud:>6}: OK, in command mode")
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
    remote = at(ser, 'RTI5', wait=1.5)
    if 'S0' in remote or 'S1' in remote:
        print(remote.strip())
        print("\nBOTH ENDS ANSWERED, so the link is up and the radios agree "
              "on NETID and AIR_SPEED. Compare the two dumps above: "
              "SERIAL_SPEED may differ without breaking the link, everything "
              "else must match.")
    else:
        print("  no answer from the far radio.")
        print("\nTHE LINK IS DOWN. The local radio is healthy, so this is a "
              "settings mismatch, not a dead radio. NETID, AIR_SPEED and ECC "
              "must be identical on both ends; SERIAL_SPEED may differ. Their "
              "register numbers are in the ATI5 dump above. After a firmware "
              "upgrade the flashed end sits on the new firmware's defaults "
              "while the other end keeps whatever it always had, which is "
              "exactly how a healthy pair stops linking. Run `sweep-air` to "
              "hunt the far radio's AIR_SPEED from this side.")
    ser.close()
    return 0


def sweep_air(port):
    """Walk the local radio's AIR_SPEED until the remote answers."""
    print("\nfinding a baud that reaches the local radio")
    ser = None
    for baud in BAUDS:
        ser = open_at(port, baud, quiet=True)
        if ser:
            print(f"  local radio answers at {baud}")
            break
    if not ser:
        sys.exit("local radio does not answer at any baud; run probe first")

    print("\nwalking AIR_SPEED. Each step writes and reboots the LOCAL radio, "
          "so this changes its settings even if nothing is found.")
    if input("type 'sweep' to continue: ").strip().lower() != 'sweep':
        ser.close()
        sys.exit("aborted, nothing written")

    for sp in AIR_SPEEDS:
        at(ser, f'ATS2={sp}')
        at(ser, 'AT&W')
        at(ser, 'ATZ', wait=2.0)
        ser.close()
        time.sleep(1.0)
        ser = open_at(port, ser.baudrate, quiet=True)
        if not ser:
            print(f"  AIR_SPEED {sp:>3}: lost the local radio, stopping")
            return 1
        remote = at(ser, 'RTI', wait=1.5)
        hit = 'SiK' in remote or 'RADIO' in remote.upper()
        print(f"  AIR_SPEED {sp:>3}: {'LINK UP' if hit else 'nothing'}")
        if hit:
            print(f"\nFOUND IT. Both radios are on AIR_SPEED {sp}. Leave it "
                  f"here, or set the far end to match a value you prefer "
                  f"with RTS2 before changing the local one.")
            ser.close()
            return 0
    ser.close()
    print("\nno AIR_SPEED linked. NETID probably differs too; that cannot be "
          "swept blind because the search space is 0-499. The far radio's "
          "NETID has to be read with the two radios wired together, or reset "
          "to default on both ends.")
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
    ser.close()
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
        ser.close()
        sys.exit("aborted, nothing written")
    print(at(ser, 'AT&F').strip())
    print(at(ser, 'AT&W').strip())
    at(ser, 'ATZ', wait=2.0)
    ser.close()
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
