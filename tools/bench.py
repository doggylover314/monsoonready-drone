#!/usr/bin/env python3
"""Bench probes for the flight controller: mode, battery, failsafe, GPS, rangefinder, nodes.

    ./python tools/bench.py mode          # watch mode switch
    ./python tools/bench.py battery       # V/A as the FC sees them
    ./python tools/bench.py failsafe      # watch STATUSTEXT
    ./python tools/bench.py gps           # fix / sats / HDOP
    ./python tools/bench.py rng           # downward rangefinder
    ./python tools/bench.py nodes         # who else is on the MAVLink bus

Parameter work belongs to tools/parameters.py; this file is sensor probing only.

Port and baud auto-detect when a single device is present: ttyUSB is a SiK
radio at 57600, ttyACM is the Pixhawk over USB at 115200. Pass --conn when
more than one is plugged in.

wait_autopilot() waits specifically for the Pixhawk's heartbeat rather than
the first one seen, since the ESP32 can answer too, at compid 195. SiK links
saturate at standard stream rates, so probes request only 2 Hz, low enough
that dropped packets do not masquerade as a real fault.
"""

import argparse
import os
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mavlink_link import connect, request_streams

DOWN = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270


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


# Components this project expects to see, so an unexpected one stands out
# rather than blending into a list of numbers.
KNOWN_COMPONENTS = {
    1:   'autopilot (the Pixhawk itself)',
    191: 'UNO Q (only heartbeats while a MavIO process runs: mission, '
         'test_everything, test_mission_link)',
    195: 'ESP32 obstacle ring',
    250: 'this script',
}


def cmd_nodes(m, args):
    """List MAVLink nodes on the bus by system and component.

    A heartbeat from component 191 (the UNO Q) confirms its transmit path to
    the Pixhawk works. Listens unfiltered, since an unknown node is exactly
    the interesting case.
    """
    request_streams(m, args.baud)
    print(f"listening {args.seconds:.0f}s for heartbeats from anyone ...")
    print("(a node is only listed once it has actually been heard)\n")
    seen = {}
    end = time.time() + args.seconds
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb is None:
            continue
        key = (hb.get_srcSystem(), hb.get_srcComponent())
        rec = seen.setdefault(key, {'n': 0, 'type': hb.type,
                                    'autopilot': hb.autopilot})
        rec['n'] += 1
        if rec['n'] == 1:
            name = KNOWN_COMPONENTS.get(key[1], 'UNKNOWN component')
            try:
                tname = mavutil.mavlink.enums['MAV_TYPE'][hb.type].name
            except KeyError:
                tname = f'MAV_TYPE {hb.type}'
            print(f"  sys {key[0]:3d} comp {key[1]:3d}  {tname:<28} {name}")

    print(f"\n{len(seen)} node(s) heard in {args.seconds:.0f}s:")
    for (sysid, compid), rec in sorted(seen.items()):
        print(f"  sys {sysid:3d} comp {compid:3d}  {rec['n']:4d} heartbeats  "
              f"{KNOWN_COMPONENTS.get(compid, 'UNKNOWN')}")
    if not any(c == 191 for _, c in seen):
        # Component 191 only sends heartbeats while a MavIO process is
        # running on the UNO Q; an idle board is silent by design, so its
        # absence here does not by itself mean the link is broken.
        print("\n  no component 191 heard. THIS ALONE DOES NOT MEAN THE LINK "
              "IS BROKEN.\n"
              "  Nothing on the UNO Q heartbeats unless a MavIO process is "
              "running\n"
              "  (the mission, test_mission_link, or test_everything). Start "
              "one on the\n"
              "  board and re-run this. The real proof of the TX direction is "
              "that\n"
              "  test_mission_link gets its SET_MESSAGE_INTERVAL ACKed.")
    else:
        print("\n  COMPONENT 191 PRESENT: the UNO Q's transmit path to the "
              "Pixhawk WORKS (USB via the hub since 2026-08-16).")


def cmd_battery(m, args):
    request_streams(m, args.baud)
    print("compare these with the multimeter at the pack:")
    end = time.time() + args.seconds
    while time.time() < end:
        s = m.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if s:
            print(f"  FC says {s.voltage_battery / 1000.0:6.2f} V  "
                  f"{s.current_battery / 100.0:6.2f} A")


def cmd_failsafe(m, args):
    request_streams(m, args.baud)
    print(f"switch the TRANSMITTER OFF now; watching messages "
          f"({args.seconds:.0f}s) ...")
    end = time.time() + args.seconds
    while time.time() < end:
        s = m.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
        if s:
            print(f"  {s.text}")


def cmd_gps(m, args):
    request_streams(m, args.baud)
    print("waiting for 10+ sats and HDOP < 1.5 (CRASH LESSONS rule) ...")
    t0 = time.time()
    end = t0 + args.seconds
    while time.time() < end:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if g is None:
            continue
        hdop = g.eph / 100.0
        # A 3D fix is required: a 2D fix has no altitude and will not
        # navigate, however good the satellite count or HDOP look.
        ok = (g.fix_type >= mavutil.mavlink.GPS_FIX_TYPE_3D_FIX
              and g.satellites_visible >= 10 and 0 < hdop < 1.5)
        print(f"  {time.time() - t0:5.0f}s  fix {g.fix_type}  "
              f"sats {g.satellites_visible:2d}  hdop {hdop:5.2f}"
              f"{'   READY' if ok else ''}")


def cmd_rng(m, args):
    request_streams(m, args.baud)
    # The TF-Luna does not work over water, so this probe only covers the
    # rangefinder over ground.
    print("downward rangefinder over GROUND (the over-water question is "
          "settled: it does not work, descend-beside only):")
    end = time.time() + args.seconds
    last_down = time.time()
    misses = 0
    while time.time() < end:
        d = m.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=0.5)
        # last_down tracks gaps between DOWNWARD frames specifically: any
        # DISTANCE_SENSOR message satisfies recv_match's own timeout, so
        # relying on that alone would hide a dropout in just the downward one.
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



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['mode', 'battery', 'failsafe', 'gps',
                                    'rng', 'nodes'])
    ap.add_argument('--conn', default=None,
                    help='serial device; omit to auto-pick when '
                         'exactly one is present')
    ap.add_argument('--baud', type=int, default=None,
                    help='omit to follow the port type: 57600 '
                         'for a SiK radio, 115200 for USB')
    ap.add_argument('--seconds', type=float, default=60.0)
    args = ap.parse_args()

    m, _, args.baud = connect(args.conn, args.baud)
    {'mode': cmd_mode, 'battery': cmd_battery, 'failsafe': cmd_failsafe,
     'gps': cmd_gps, 'rng': cmd_rng, 'nodes': cmd_nodes}[args.cmd](m, args)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # Probes run until --seconds expires. Ctrl-C ends one early; that is
        # normal, not an error, so no traceback here.
        print("\nstopped.")
        sys.exit(0)
