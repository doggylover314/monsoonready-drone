#!/usr/bin/env python3
"""Small bench/field probes against the flight controller, over USB or the
SiK radio. Replaces the throwaway pymavlink one-liners with something that
gets the link details right in one place.

    ./python tools/bench.py mode          # watch mode switch
    ./python tools/bench.py battery       # V/A as the FC sees them
    ./python tools/bench.py failsafe      # watch STATUSTEXT
    ./python tools/bench.py gps           # fix / sats / HDOP
    ./python tools/bench.py rng           # downward rangefinder
    ./python tools/bench.py getparam PRX1_TYPE
    ./python tools/bench.py setparam RNGFND1_GNDCLR 0.14

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
import struct
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
        # fix_type must be 3D: a 2D fix has no usable altitude and ArduPilot
        # will not navigate on it, yet it can show 10+ sats and a good HDOP.
        ok = (g.fix_type >= mavutil.mavlink.GPS_FIX_TYPE_3D_FIX
              and g.satellites_visible >= 10 and 0 < hdop < 1.5)
        print(f"  {time.time() - t0:5.0f}s  fix {g.fix_type}  "
              f"sats {g.satellites_visible:2d}  hdop {hdop:5.2f}"
              f"{'   READY' if ok else ''}")


def cmd_rng(m, args):
    streams(m, args.baud)
    # NOT for the over-water bench any more: Raghav recorded 2026-08-09 that
    # the TF-Luna over water DOES NOT WORK and never will, so TODO 6 is closed
    # by verdict and descend-BESIDE is the only route. This probe is now just
    # "is the downward rangefinder healthy and continuous over ground".
    print("downward rangefinder over GROUND (the over-water question is "
          "settled: it does not work, descend-beside only):")
    end = time.time() + args.seconds
    last_down = time.time()
    misses = 0
    while time.time() < end:
        d = m.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=0.5)
        # Time the gap between DOWNWARD frames specifically. Keying on
        # recv_match's timeout hid every dropout whenever the upward sensor
        # was streaming, since its frames reset the clock.
        if time.time() - last_down > 1.0:
            misses += 1
            print(f"  --- NO DOWNWARD READING for "
                  f"{time.time() - last_down:.1f}s  (dropout {misses}) ---")
            last_down = time.time()
        if d is None or d.orientation != DOWN:
            continue
        last_down = time.time()
        lo, hi = d.min_distance / 100.0, d.max_distance / 100.0
        val = d.current_distance / 100.0
        flag = '' if lo <= val <= hi else f'   OUT OF RANGE ({lo:.2f}-{hi:.2f})'
        print(f"  {val:6.2f} m{flag}")


def await_param(m, name, timeout=10.0):
    """The PARAM_VALUE for THIS name, discarding others.

    ArduPilot broadcasts PARAM_VALUE whenever any parameter changes, and a
    second GCS on the same link produces more, so taking "the next one" and
    labelling it with the name you asked for can report a completely
    different parameter's value. push_params.py already does this correctly.
    """
    end = time.time() + timeout
    while time.time() < end:
        p = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if p is None:
            continue
        got = p.param_id if isinstance(p.param_id, str) else p.param_id.decode()
        if got.strip('\x00') == name:
            return p
    return None


def cmd_getparam(m, args):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  args.name.encode(), -1)
    p = await_param(m, args.name)
    print(f"  {args.name} = {p.param_value if p else 'NO REPLY'}")


def cmd_setparam(m, args):
    want = float(args.value)
    m.mav.param_set_send(m.target_system, m.target_component,
                         args.name.encode(), want,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    p = await_param(m, args.name)
    got = p.param_value if p else None
    print(f"  {args.name} = {got if got is not None else 'NO REPLY'}")
    if got is not None and struct.unpack('f', struct.pack('f', got))[0] != \
            struct.unpack('f', struct.pack('f', want))[0]:
        print(f"  REFUSED OR CLAMPED: asked {want}, board stored {got}. The "
              f"write did NOT take effect as typed.")
        sys.exit(1)
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
