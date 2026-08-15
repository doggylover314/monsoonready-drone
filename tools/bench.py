#!/usr/bin/env python3
"""Small bench/field probes against the flight controller, over USB or the
SiK radio. Replaces the throwaway pymavlink one-liners with something that
gets the link details right in one place.

    ./python tools/bench.py mode          # watch mode switch
    ./python tools/bench.py battery       # V/A as the FC sees them
    ./python tools/bench.py failsafe      # watch STATUSTEXT
    ./python tools/bench.py gps           # fix / sats / HDOP
    ./python tools/bench.py rng           # downward rangefinder
    ./python tools/bench.py nodes         # who else is on the MAVLink bus

PARAMETERS ARE NOT HERE. getparam/setparam moved to
tools/parameters.py on 2026-08-10: reading and writing the board's
configuration is a different job from probing whether a sensor works,
and keeping them together meant two copies of the write-and-verify
logic, only one of which checked for a clamped value.

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
import os
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Shared link plumbing. Lives in its own library so this file does not
# have to import a 623-line PASS/FAIL test just to open a serial port.
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
    191: 'UNO Q  <-- TODO 7: this appearing IS the D0/D1 pass',
    195: 'ESP32 obstacle ring (parked, may be unplugged)',
    250: 'this script',
}


def cmd_nodes(m, args):
    """Every MAVLink talker on the bus, by system and component.

    THE POINT OF THIS IS TODO 7. The UNO Q sits sealed inside the airframe with
    no readable screen, so the probe sketch answers by TRANSMITTING a heartbeat
    as component 191 and letting the Pixhawk relay its existence to here. A row
    for comp 191 is the pass; its absence is the fail. The ESP32 at 195 already
    proves the channel works.

    Heartbeats are ~1 Hz per node, so give it a few seconds before believing an
    absence. This listens without filtering by target, unlike everything else
    in this file, precisely because the unknown node is the interesting one.
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
        # NOT a link failure on its own, and saying so cost real time at the
        # farm on 2026-08-15. Nothing on the UNO Q heartbeats unless a python
        # process using MavIO is RUNNING: the old probe sketch did it from
        # firmware, sketch_mav_shovel only forwards bytes. So an idle board
        # is silent by design.
        print("\n  no component 191 heard. THIS ALONE DOES NOT MEAN THE LINK "
              "IS BROKEN.\n"
              "  Nothing on the UNO Q heartbeats unless a MavIO process is "
              "running (the\n"
              "  shovel sketch only forwards bytes; the old probe sketch used "
              "to beat by\n"
              "  itself). Start the pump plus test_mission_link (or the "
              "mission) on the\n"
              "  board and re-run this. The real proof of the TX direction is "
              "that\n"
              "  test_mission_link gets its SET_MESSAGE_INTERVAL ACKed.")
    else:
        print("\n  COMPONENT 191 PRESENT: the UNO Q's transmit path to the "
              "Pixhawk WORKS. D0/D1 -> SERIAL5 is proven in the TX direction.")


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
        # fix_type must be 3D: a 2D fix has no usable altitude and ArduPilot
        # will not navigate on it, yet it can show 10+ sats and a good HDOP.
        ok = (g.fix_type >= mavutil.mavlink.GPS_FIX_TYPE_3D_FIX
              and g.satellites_visible >= 10 and 0 < hdop < 1.5)
        print(f"  {time.time() - t0:5.0f}s  fix {g.fix_type}  "
              f"sats {g.satellites_visible:2d}  hdop {hdop:5.2f}"
              f"{'   READY' if ok else ''}")


def cmd_rng(m, args):
    request_streams(m, args.baud)
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
        # Ctrl-C is the NORMAL way to end these: every probe runs until
        # --seconds expires and you usually have what you need before then.
        # Letting it raise printed a pymavlink traceback that looks exactly
        # like a tool crash, on a run that in fact worked perfectly.
        print("\nstopped.")
        sys.exit(0)
