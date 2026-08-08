#!/usr/bin/env python3
"""Small bench/field probes against the flight controller, over USB or the
SiK radio. Replaces the throwaway pymavlink one-liners with something that
gets the link details right in one place.

    training/.venv/bin/python tools/bench.py mode          # watch mode switch
    training/.venv/bin/python tools/bench.py battery       # V/A as the FC sees them
    training/.venv/bin/python tools/bench.py failsafe      # watch STATUSTEXT
    training/.venv/bin/python tools/bench.py gps           # fix / sats / HDOP
    training/.venv/bin/python tools/bench.py rng           # downward rangefinder
    training/.venv/bin/python tools/bench.py getparam PRX1_TYPE
    training/.venv/bin/python tools/bench.py setparam RNGFND1_GNDCLR 0.14

PORT AND BAUD ARE WORKED OUT FOR YOU when only one serial device is present:
a ttyUSB is assumed to be the SiK radio (57600), a ttyACM the Pixhawk's USB
(115200). With several plugged in it refuses to guess; name one with --conn.

Two things this gets right that a hand-typed one-liner does not:

  TARGETING. wait_heartbeat() returns the FIRST heartbeat from anyone, and
  this bus has had a second MAVLink talker on it (the ESP32, compid 195).
  Locking onto that leaves target_system 0 = broadcast, and commands then go
  to nobody in particular with no ack. wait_autopilot() waits for the real
  autopilot. See tools/wiring_check.py for the full story.

  LINK RATE. A SiK link is far slower than its 57600 serial port suggests.
  Requesting the usual stream rates saturates it and the losses look like
  faults. Stream requests here are 2 Hz on a low-baud link.
"""

import argparse
import sys
import time

from pymavlink import mavutil

# Same directory, and this is how the fix stays in one place.
from wiring_check import resolve_link, wait_autopilot

DOWN = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270


def connect(args):
    args.conn, args.baud = resolve_link(args.conn, args.baud)
    print(f"connecting {args.conn} at {args.baud} ...")
    m = mavutil.mavlink_connection(args.conn, baud=args.baud,
                                   source_system=250)
    if not wait_autopilot(m):
        sys.exit("no autopilot heartbeat (is the radio paired and the "
                 "aircraft powered?)")
    print(f"autopilot is system {m.target_system} component "
          f"{m.target_component}")
    return m


def streams(m, baud):
    rate = 2 if baud <= 57600 else 4
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                   rate, 1)


def cmd_mode(m, args):
    print("flip the mode switch through every position "
          f"({args.seconds:.0f}s) ...")
    last = None
    end = time.time() + args.seconds
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb is None or hb.get_srcComponent() != \
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            continue
        mode = mavutil.mode_string_v10(hb)
        if mode != last:
            print(f"  {mode}")
            last = mode


def cmd_battery(m, args):
    streams(m, args.baud)
    print("compare these with the multimeter at the pack:")
    end = time.time() + args.seconds
    while time.time() < end:
        s = m.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if s:
            print(f"  FC says {s.voltage_battery / 1000.0:6.2f} V  "
                  f"{s.current_battery / 100.0:6.2f} A")


def cmd_failsafe(m, args):
    streams(m, args.baud)
    print(f"switch the TRANSMITTER OFF now; watching messages "
          f"({args.seconds:.0f}s) ...")
    end = time.time() + args.seconds
    while time.time() < end:
        s = m.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
        if s:
            print(f"  {s.text}")


def cmd_gps(m, args):
    streams(m, args.baud)
    print("waiting for 10+ sats and HDOP < 1.5 (CRASH LESSONS rule) ...")
    t0 = time.time()
    end = t0 + args.seconds
    while time.time() < end:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if g is None:
            continue
        hdop = g.eph / 100.0
        ok = g.satellites_visible >= 10 and 0 < hdop < 1.5
        print(f"  {time.time() - t0:5.0f}s  fix {g.fix_type}  "
              f"sats {g.satellites_visible:2d}  hdop {hdop:5.2f}"
              f"{'   READY' if ok else ''}")


def cmd_rng(m, args):
    streams(m, args.baud)
    print("downward rangefinder; over water watch for dropouts, which are "
          "the whole point of the TF-Luna bench (TODO 6):")
    end = time.time() + args.seconds
    misses = 0
    while time.time() < end:
        d = m.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=2)
        if d is None:
            misses += 1
            print(f"  --- no reading ({misses}) ---")
            continue
        if d.orientation == DOWN:
            print(f"  {d.current_distance / 100.0:6.2f} m")


def cmd_getparam(m, args):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  args.name.encode(), -1)
    p = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
    print(f"  {args.name} = {p.param_value if p else 'NO REPLY'}")


def cmd_setparam(m, args):
    m.mav.param_set_send(m.target_system, m.target_component,
                         args.name.encode(), float(args.value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    p = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
    got = p.param_value if p else None
    print(f"  {args.name} = {got if got is not None else 'NO REPLY'}")
    if got is None:
        print("  no echo: over a radio link a lost packet looks exactly like "
              "a refused write, so re-read it with getparam before believing "
              "either.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['mode', 'battery', 'failsafe', 'gps',
                                    'rng', 'getparam', 'setparam'])
    ap.add_argument('name', nargs='?')
    ap.add_argument('value', nargs='?')
    ap.add_argument('--conn', default=None,
                    help='serial device; omit to auto-pick when '
                         'exactly one is present')
    ap.add_argument('--baud', type=int, default=None,
                    help='omit to follow the port type: 57600 '
                         'for a SiK radio, 115200 for USB')
    ap.add_argument('--seconds', type=float, default=60.0)
    args = ap.parse_args()

    if args.cmd in ('getparam', 'setparam') and not args.name:
        sys.exit(f"{args.cmd} needs a parameter name")
    if args.cmd == 'setparam' and args.value is None:
        sys.exit("setparam needs a value")

    m = connect(args)
    {'mode': cmd_mode, 'battery': cmd_battery, 'failsafe': cmd_failsafe,
     'gps': cmd_gps, 'rng': cmd_rng, 'getparam': cmd_getparam,
     'setparam': cmd_setparam}[args.cmd](m, args)


if __name__ == '__main__':
    main()
