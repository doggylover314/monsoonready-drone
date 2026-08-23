#!/usr/bin/env python3
"""Read and set SiK radio parameters over AT commands. Local radio only.

    ./python tools/sik_config.py                       # find it, dump everything
    ./python tools/sik_config.py --set NETID=25
    ./python tools/sik_config.py --set MIN_FREQ=433050 --set MAX_FREQ=434790

WHY THIS EXISTS (2026-08-23): QGC flashed the ground radio's firmware, which
resets EVERY radio parameter to the firmware's defaults, and QGC then stopped
detecting the radio at all. Two of those defaults will do exactly that:

  SERIAL_SPEED  the rate the radio talks to the COMPUTER. If the flash moved
                it, nothing that opens the port at the old baud sees anything,
                which looks identical to a dead radio. This is why the tool
                SCANS bauds instead of trusting one.
  MIN_FREQ/MAX_FREQ  which band the radio transmits on. The generic hm_trp
                image supports 433 and 915, and a 433 radio carrying 915
                settings will never hear its partner however close you stand.

NOTHING HERE IS FROM MEMORY. The tool never hardcodes an S-register number: it
asks the radio for ATI5, which returns every parameter as "S3:NETID=25", and
looks names up in that. If a register moved between firmware versions, this
still works and a hardcoded table would have lied.

LOCAL RADIO ONLY, DELIBERATELY. The remote radio is reachable with RT commands
instead of AT, but only over a working link, and if the link worked you would
not be running this.

AT MODE IS TIMING-SENSITIVE, not a command: one second of silence, the three
characters +++, one second of silence. Traffic on the port defeats it, so if
the aircraft is powered and the link IS up, this will fail until you unplug
one end. That is the firmware's rule, not a limitation here.
"""

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing. This is the same venv the other tools use: "
             "pip install pyserial")

# SiK's own SERIAL_SPEED values, commonest first. A flash can land on any.
BAUDS = [57600, 115200, 38400, 9600, 19200, 250000]
GUARD_S = 1.1          # firmware wants >1s of silence either side of '+++'


def candidate_ports(explicit):
    if explicit:
        return [explicit]
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/tty.usbserial*'))
    # A Pixhawk is a ttyACM and is never a SiK. Excluded so a mistyped run
    # cannot dump '+++' into the flight controller's USB link.
    return ports


def talk(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write((cmd + '\r\n').encode())
    ser.flush()
    time.sleep(wait)
    return ser.read(ser.in_waiting or 1).decode(errors='replace')


def enter_at_mode(ser, tries=2):
    """Returns True if the radio answered. Costs ~2.5s per attempt.

    Two attempts, because this is genuinely flaky: the guard time is measured
    by the radio, so anything still in its receive buffer from a previous
    session restarts the clock. Observed 2026-08-23, the first invocation
    succeeded and the immediate second one did not, on the same radio."""
    for _ in range(tries):
        # A radio left in command mode by an earlier session answers AT and
        # ignores +++, so ask before shouting.
        if 'OK' in talk(ser, 'AT'):
            return True
        ser.reset_input_buffer()
        time.sleep(GUARD_S)
        ser.write(b'+++')
        ser.flush()
        time.sleep(GUARD_S)
        if 'OK' in ser.read(ser.in_waiting or 1).decode(errors='replace'):
            return True
    return False


def find_radio(ports, bauds):
    for port in ports:
        for baud in bauds:
            print(f"  trying {port} at {baud}...", flush=True)
            try:
                ser = serial.Serial(port, baud, timeout=0.5)
            except (OSError, serial.SerialException) as exc:
                print(f"    cannot open: {exc}")
                break
            try:
                if enter_at_mode(ser):
                    print(f"  RADIO ANSWERED on {port} at {baud}")
                    return ser, port, baud
            except (OSError, serial.SerialException) as exc:
                print(f"    {exc}")
            ser.close()
    return None, None, None


def read_params(ser):
    """{name: (register, value)} straight from the radio's own ATI5."""
    text = talk(ser, 'ATI5', wait=1.0)
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith('S') or ':' not in line or '=' not in line:
            continue
        reg, rest = line.split(':', 1)
        name, value = rest.split('=', 1)
        out[name.strip()] = (reg.strip(), value.strip())
    return out, text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=None,
                    help='serial device; omit to scan ttyUSB*')
    ap.add_argument('--baud', type=int, default=None,
                    help='omit to scan the SiK serial speeds')
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                    help='set a parameter BY NAME. Repeatable. Nothing is '
                         'written to EEPROM without --write.')
    ap.add_argument('--write', action='store_true',
                    help='persist with AT&W and reboot the radio with ATZ. '
                         'Without it, --set changes are live but forgotten '
                         'at the next power cycle, which is the safe default '
                         'for trying a value out.')
    args = ap.parse_args()

    ports = candidate_ports(args.port)
    if not ports:
        sys.exit("no /dev/ttyUSB* found. The radio is not enumerating at all, "
                 "which is a cable or power problem, not a settings problem.")
    bauds = [args.baud] if args.baud else BAUDS

    print("Looking for the radio. Nothing is written yet.")
    ser, port, baud = find_radio(ports, bauds)
    if ser is None:
        sys.exit(
            "\nNo radio answered on any port at any speed.\n"
            "That is NOT a settings problem: at least one baud would have "
            "replied.\nCheck power, the USB cable, and that nothing else "
            "(QGC, a python tool) is holding the port open.\n"
            "If the aircraft is powered and the link is actually UP, MAVLink "
            "traffic\ndefeats the +++ sequence; unplug the air end and retry.")

    params, raw = read_params(ser)
    print(f"\n{talk(ser, 'ATI').strip()}")
    print(f"\n--- {port} @ {baud} ---")
    if not params:
        print("ATI5 returned nothing parseable. Raw reply:")
        print(raw)
        ser.close()
        return 1
    for name, (reg, value) in params.items():
        print(f"  {reg:>4}  {name:<14} {value}")

    band = params.get('MIN_FREQ', (None, ''))[1]
    if band.startswith('9'):
        print("\n  MIN_FREQ IS IN THE 900 BAND. If this is a 433 MHz radio, "
              "that alone\n  explains a dead link, and no amount of standing "
              "closer will fix it.\n  433 settings are MIN_FREQ=433050 "
              "MAX_FREQ=434790.")

    rc = 0
    for item in args.set:
        if '=' not in item:
            print(f"\n  SKIPPED '{item}': expected NAME=VALUE")
            rc = 1
            continue
        name, value = item.split('=', 1)
        name = name.strip().upper()
        if name not in params:
            print(f"\n  SKIPPED {name}: this radio has no such parameter. "
                  f"It reported: {', '.join(sorted(params))}")
            rc = 1
            continue
        reg = params[name][0]
        reply = talk(ser, f'AT{reg}={value.strip()}')
        ok = 'OK' in reply
        print(f"\n  {name} ({reg}) = {value.strip()}  ->  "
              f"{'OK' if ok else reply.strip() or 'no reply'}")
        rc |= 0 if ok else 1

    rebooted = False
    if args.set and args.write:
        print(f"  AT&W -> {talk(ser, 'AT&W').strip()}")
        print(f"  ATZ  -> {talk(ser, 'ATZ').strip()}  (radio rebooting)")
        rebooted = True
    elif args.set:
        print("\n  NOT SAVED. These are live but the next power cycle forgets "
              "them.\n  Re-run with --write once you know the values are "
              "right.")

    # ALWAYS hand the radio back in data mode. A radio left in command mode
    # carries no telemetry and looks exactly like a dead link, so forgetting
    # this turns a diagnostic tool into the fault it was meant to find.
    if not rebooted:
        talk(ser, 'ATO')
    ser.close()
    return rc


if __name__ == '__main__':
    sys.exit(main())
