#!/usr/bin/env python3
"""Interactive servo positioner. Nudges the gate step by step and reports the
pulse width, for setting the closed and open positions by eye.

    ./python tools/servo_jog.py                 # ch9, starts at 1500us
    ./python tools/servo_jog.py --start 1600
    ./python tools/servo_jog.py --channel 9 --step 5

Mark the two positions with `c` and `o`. On exit it prints them.

Pulse width is the commanded quantity. The angle readout beside it is derived,
(us - reference) / US_PER_DEG, since an MG90 has no position feedback. Higher
pulse width swings the horn clockwise at the spline.

Sends whatever is typed, no limits and no warnings (user, 2026-08-12).
SERVO<channel>_FUNCTION must be 0 or the autopilot acks and moves nothing.
Leaves the gate wherever it ends up rather than closing it.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'uno_q'))

from pymavlink import mavutil                                    # noqa: E402
from mavlink_link import connect, send_and_ack                   # noqa: E402
from dropper import PixhawkServoDropper as Gate                  # noqa: E402

HELP = """
commands:
  +  /  -    move one step (--step, default 5 deg) one way or the other
  +15  -7.5  move that many DEGREES, sign chooses the direction
  +30us      move that many MICROSECONDS, when you want to think in pulse width
  =1600      go straight to an absolute pulse width
  c          mark HERE as the CLOSED position
  o          mark HERE as the OPEN position
  z          re-zero the angle readout here (the pulse width is unaffected)
  h          this help
  q          quit and print the summary
"""


def parse_move(s, step_deg, us_per_deg):
    """A typed command -> ('abs'|'rel', microseconds), or (None, reason)."""
    s = s.strip().lower().replace(' ', '')
    if not s:
        return None, 'empty'
    if s.startswith('='):
        try:
            return 'abs', int(round(float(s[1:])))
        except ValueError:
            return None, f"{s[1:]!r} is not a number of microseconds"
    if s in ('+', '-'):
        s += str(step_deg)
    if s[0] not in '+-':
        return None, "moves need a sign: +5 to go one way, -5 the other"
    in_us = s.endswith('us')
    body = s[:-2] if in_us else s
    try:
        amount = float(body)
    except ValueError:
        return None, f"{body!r} is not a number"
    return 'rel', int(round(amount if in_us else amount * us_per_deg))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', type=int, default=9,
                    help='servo output; 9 = AUX OUT 1, as the gate is wired')
    ap.add_argument('--start', type=int, default=1500,
                    help='pulse width to move to first (default 1500)')
    ap.add_argument('--step', type=float, default=5.0,
                    help='degrees moved by a bare + or - (default 5)')
    ap.add_argument('--us-per-deg', type=float, default=Gate.US_PER_DEG,
                    help='microseconds per degree, angle readout only. '
                         'Default is dropper.py US_PER_DEG')
    ap.add_argument('--conn', default=None)
    ap.add_argument('--baud', type=int, default=None)
    args = ap.parse_args()

    if args.us_per_deg <= 0:
        sys.exit("--us-per-deg must be positive")

    m, _, args.baud = connect(args.conn, args.baud)
    ack_timeout = 5.0 if args.baud > 57600 else 12.0    # radio links are slower

    print(f"\ngate on ch{args.channel}, {args.us_per_deg:g} us/deg readout")
    print(HELP)

    pos = None          # last pulse width the board accepted
    ref = args.start    # angle readout zero
    marks = {}          # 'closed'/'open' -> us

    def goto(us, why):
        """Command a pulse width. Returns True only on an accepted ack."""
        nonlocal pos
        res = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                           args.channel, us, timeout=ack_timeout)
        if res != 'MAV_RESULT_ACCEPTED':
            print(f"  {us}us -> {res}   (nothing moved)")
            if res == 'NO ACK':
                print("  No ack: check the link, and close QGC if it is "
                      "connected, since it holds the port.")
            else:
                print(f"  Refused. Check SERVO{args.channel}_FUNCTION=0.")
            return False
        pos = us
        deg = (us - ref) / args.us_per_deg
        way = '' if deg == 0 else (
            ' clockwise' if deg > 0 else ' counter-clockwise')
        print(f"  {us}us   {deg:+.1f} deg{way} from the reference   ({why})")
        return True

    if not goto(args.start, 'start'):
        sys.exit("could not command the starting position")
    print("  ^ starting position. Everything below is relative to it.\n")

    while True:
        try:
            raw = input(f"[{pos}us] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw:
            continue
        low = raw.lower()
        if low in ('q', 'quit', 'exit'):
            break
        if low in ('h', '?', 'help'):
            print(HELP)
            continue
        if low == 'z':
            ref = pos
            print(f"  angle re-zeroed at {pos}us, readout only")
            continue
        if low in ('c', 'closed', 'o', 'open'):
            name = 'closed' if low in ('c', 'closed') else 'open'
            marks[name] = pos
            print(f"  marked {name.upper()} = {pos}us")
            continue

        kind, val = parse_move(raw, args.step, args.us_per_deg)
        if kind is None:
            print(f"  {val}. Type h for the command list.")
            continue
        goto(val if kind == 'abs' else pos + val, raw)

    # Summary: report where the gate ended up and whatever got marked.
    print(f"\nGATE LEFT AT {pos}us, not closed.")

    if 'closed' in marks and 'open' in marks:
        c, o = marks['closed'], marks['open']
        travel = abs(o - c) / args.us_per_deg
        way = 'counter-clockwise' if o < c else 'clockwise'
        print(f"\nMARKED:  closed {c}us   open {o}us")
        print(f"  travel {abs(o - c)}us, about {travel:.0f} deg {way} to open")
        print(f"\nPut these in uno_q/dropper.py:")
        print(f"  DEFAULT_CLOSED_US = {c}    DEFAULT_OPEN_US = {o}")
        print("  run_mission.py, wiring_check.py and flow_test.py read them "
              "from there.")
    elif marks:
        got = ', '.join(f"{k} {v}us" for k, v in marks.items())
        print(f"\nMARKED: {got}. Re-run and mark both positions.")
    else:
        print("\nNothing marked. Re-run, press c at closed and o at open.")


if __name__ == '__main__':
    main()
