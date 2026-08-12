#!/usr/bin/env python3
"""Interactive servo positioner: nudge the gate by an amount you choose, in a
direction you choose, and read off the pulse width when it looks right.

    ./python tools/servo_jog.py                 # ch9, starts at 1500us
    ./python tools/servo_jog.py --start 1600    # start from the old closed value
    ./python tools/servo_jog.py --channel 9 --step 5

WHAT THIS IS FOR: the servo's orientation on the gate has changed, so the
recorded closed/open pulse widths no longer mean what they meant. This walks
the servo one nudge at a time so you can watch the gate and mark the two
positions by eye. Mark them with `c` and `o`; on exit it prints the values and
which files to put them in.

THE ANGLE IS DERIVED, NOT MEASURED. An MG90 has no position feedback and the
Pixhawk cannot read a servo's actual angle, so nothing here knows where the
horn really is. The PULSE WIDTH is the real number: it is what this commands,
what dropper.py stores, and what the servo acts on. The angle beside it is
arithmetic, (us - reference) / US_PER_DEG, with US_PER_DEG = 10.0 from the
2026-08-10 bench observation that 900us swung the horn 90 degrees on this
servo. If a commanded 90 degrees does not look like 90 degrees, pass
--us-per-deg rather than inventing new pulse numbers to compensate.

DIRECTION: increasing the pulse width swung the horn clockwise at the spline
(1000us -> 1900us = 90 deg clockwise, 2026-08-10). Whether clockwise now opens
or closes the gate is what the new mounting decides, and what you are about to
establish by eye.

THIS TOOL IMPOSES NO LIMITS AND PRINTS NO WARNINGS (user, 2026-08-12). It
sends whatever you type. ArduPilot may still clamp a command to the output's
own SERVO<n>_MIN/_MAX, in which case the gate stops moving while the printed
number keeps going; that is a clamp, not a mechanical stop. The flight code's
guard in mavlink_io.py is separate and untouched, so if a new travel needs
pulses outside it, widening that is its own deliberate change.

SERVO<channel>_FUNCTION must be 0, or ArduPilot acks ACCEPTED and moves
nothing.

It does not close the gate on exit, deliberately: you are here to leave the
gate somewhere and look at it. It prints the final position instead.
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
    """A typed command -> ('abs'|'rel', microseconds), or (None, reason).

    Returns a delta in MICROSECONDS for relative moves, because microseconds
    are what actually get commanded; degrees are only ever a way of typing.
    """
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
                    help='microseconds per degree for the ANGLE READOUT only. '
                         'Default is dropper.py US_PER_DEG, measured on this '
                         'servo. Change it only if a commanded 90 degrees does '
                         'not look like 90 degrees')
    ap.add_argument('--conn', default=None)
    ap.add_argument('--baud', type=int, default=None)
    args = ap.parse_args()

    if args.us_per_deg <= 0:
        sys.exit("--us-per-deg must be positive")

    m, _, args.baud = connect(args.conn, args.baud)
    # A radio link drops packets and adds latency, so an ack that arrives
    # comfortably over USB can miss a 5s window and print a false failure on a
    # command that worked. Same patience the other tools use.
    ack_timeout = 5.0 if args.baud > 57600 else 12.0

    print(f"\ngate on ch{args.channel}, {args.us_per_deg:g} us/deg readout "
          f"(derived from the pulse width, not measured).")
    print(HELP)

    pos = None          # last pulse width the board ACCEPTED
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
                print(f"  No ack at all: check the link, and that QGC is "
                      f"closed since it owns the port while connected.")
            else:
                print(f"  The autopilot refused it. Check "
                      f"SERVO{args.channel}_FUNCTION=0.")
            return False
        pos = us
        deg = (us - ref) / args.us_per_deg
        way = '' if deg == 0 else (
            ' clockwise' if deg > 0 else ' counter-clockwise')
        print(f"  {us}us   {deg:+.1f} deg{way} from the reference   ({why})")
        return True

    if not goto(args.start, 'start'):
        sys.exit("could not command the starting position, so there is "
                 "nothing to jog from. Fix the above before continuing.")
    print("  ^ LOOK AT THE GATE. Everything below is relative to this.\n")

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
            print(f"  angle re-zeroed at {pos}us. The pulse width did not "
                  f"change; only the readout did.")
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

    # --- summary --------------------------------------------------------------
    print(f"\nGATE LEFT AT {pos}us. It was NOT closed automatically, because "
          f"this tool has no way of knowing which end closed means any more.")

    if 'closed' in marks and 'open' in marks:
        c, o = marks['closed'], marks['open']
        travel = abs(o - c) / args.us_per_deg
        way = 'counter-clockwise' if o < c else 'clockwise'
        print(f"\nMARKED:  closed {c}us   open {o}us")
        print(f"  travel {abs(o - c)}us = about {travel:.0f} deg {way} to open "
              f"(derived from {args.us_per_deg:g} us/deg, so trust the "
              f"microseconds and treat the degrees as a description)")
        print(f"\nThese two numbers belong in TWO places:")
        print(f"  uno_q/dropper.py   DEFAULT_CLOSED_US = {c}"
              f"    DEFAULT_OPEN_US = {o}")
        print(f"  uno_q/run_mission.py  --servo-closed-us / --servo-open-us "
              f"still hard-code the OLD 1000/1900 pair instead of reading "
              f"dropper.py, so the flight runner would ignore the values above. "
              f"Live bug, recorded in PROJECT_STATE 2026-08-12; fix it in the "
              f"same change.")
        print(f"  (tools/wiring_check.py and tools/flow_test.py already read "
              f"dropper.py, so they follow automatically.)")
    elif marks:
        got = ', '.join(f"{k} {v}us" for k, v in marks.items())
        print(f"\nMARKED: {got}. The other one was never marked, so run this "
              f"again and mark both: a single position cannot say which way "
              f"the gate travels.")
    else:
        print("\nNothing marked. Re-run and press c at the closed position and "
              "o at the open one.")


if __name__ == '__main__':
    main()
