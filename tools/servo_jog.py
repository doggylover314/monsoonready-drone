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
exactly which files to put them in.

THE ANGLE IS DERIVED, NOT MEASURED, and it matters that you know which:

  * An MG90 has NO position feedback, and the Pixhawk has no way to read a
    servo's actual angle. Nothing in this system can measure where the horn is.
  * The PULSE WIDTH is the real number. It is what this tool commands, what
    dropper.py stores, and what the servo acts on.
  * The angle shown beside it is arithmetic: (us - reference) / US_PER_DEG,
    with US_PER_DEG = 10.0 from the 2026-08-10 bench observation that a 900us
    change swung the horn 90 degrees ON THIS SERVO. That is a calibration
    ratio, not a datasheet figure. It survived the remount because it is a
    property of the servo, not of how the horn sits on the gate; what the
    remount changed is where zero is, which is why the reference is just
    "wherever you started" and can be re-zeroed with `z` at any time.
  * If a commanded 90 degrees does not look like 90 degrees on the gate, the
    ratio is wrong for this unit: pass --us-per-deg and say so, rather than
    inventing new pulse numbers to compensate.

DIRECTION: increasing the pulse width swung the horn CLOCKWISE on the bench
(1000us -> 1900us = 90 deg clockwise, 2026-08-10). That is at the spline.
Whether clockwise now opens or closes the gate is precisely what the new
mounting decides, and precisely what you are about to establish by eye.

THIS TOOL REFUSES NOTHING (user, 2026-08-12, after the 800-2200us guard
stopped a jog at 2200). It warns and sends anyway, because the whole point is
to find the travel limits of a mounting nobody has characterised yet, and a
tool that blocks at the old numbers cannot discover the new ones. TWO THINGS
THE REMOVAL DOES NOT CHANGE, and they are the ones that matter:

  * THE FLIGHT CODE'S GUARD IS UNTOUCHED. mavlink_io.PWM_MIN_US/PWM_MAX_US is
    still 800-2200 and still checked at construction in dropper.py and at send
    time in MavIO.set_servo. This tool does not go through either path: it
    sends MAV_CMD_DO_SET_SERVO directly. So if the new travel needs pulses
    outside 800-2200, those two constants have to be widened DELIBERATELY, as
    a separate decision, and this tool's freedom is not that decision.
  * ARDUPILOT MAY CLAMP, AND A CLAMP IS INVISIBLE HERE. DO_SET_SERVO is
    subject to the output's own SERVO<n>_MIN/_MAX, so a command outside that
    window can be accepted and then clamped, leaving the servo somewhere other
    than the number printed on screen. That would silently corrupt exactly the
    measurement you are here to take. So this reads SERVO<n>_MIN/_MAX off the
    board at startup and warns whenever a command leaves that window. WHEN YOU
    SEE THAT WARNING, BELIEVE THE GATE, NOT THE NUMBER: if the gate stopped
    moving while the printed pulse kept climbing, you have found the clamp, not
    the mechanical stop.

BEFORE YOU RUN IT:
  * HOPPER EMPTY. This drives the gate to positions nobody has verified yet,
    and half of them may be open.
  * Props off. Nothing here arms anything or touches a motor.
  * SERVO<channel>_FUNCTION must be 0, or ArduPilot acks ACCEPTED and moves
    nothing. An ACCEPTED with no movement means that parameter, the signal
    wire, or servo power (XY-3606 / USB-A buck), in that order of likelihood.
  * Near a mechanical stop, use SMALL steps and do not park the servo stalled
    against one. A stalled servo draws its stall current continuously and
    metal gears transmit that to whatever stops it. IF IT BUZZES OR HUMS AND
    STOPS MOVING, BACK OFF IMMEDIATELY: that is a stall, and it is the one way
    this tool can damage the hardware now that it refuses nothing.

IT DOES NOT CLOSE THE GATE ON EXIT, deliberately: you are here to leave the
gate somewhere and look at it. It prints the final position loudly instead.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'uno_q'))

from pymavlink import mavutil                                    # noqa: E402
from mavlink_link import connect, send_and_ack                   # noqa: E402
from parameters import await_param                               # noqa: E402
from mavlink_io import PWM_MIN_US, PWM_MAX_US                    # noqa: E402
from dropper import PixhawkServoDropper as Gate                  # noqa: E402

# A typo catcher, NOT a limit and NOT a spec. Hobby servos are conventionally
# commanded within roughly this band; no MG90 datasheet was consulted for it,
# so it only ever produces a warning. The real limits are the gate's mechanical
# stops (found by eye) and SERVO<n>_MIN/_MAX (read off the board below).
SANE_MIN_US = 500
SANE_MAX_US = 2500

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


def read_param(m, name, attempts=3, timeout=4.0):
    """One parameter off the board, or None. Read-only.

    Asks more than once for the same reason parameters.py does: a dropped
    request is indistinguishable from a parameter that does not exist, and
    treating one as the other sends you debugging the wrong thing.
    """
    for _ in range(attempts):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        p = await_param(m, name, timeout=timeout)
        if p is not None:
            return p.param_value
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', type=int, default=9,
                    help='servo output; 9 = AUX OUT 1, as the gate is wired')
    ap.add_argument('--start', type=int, default=1500,
                    help='pulse width to move to first. Default 1500 (mid '
                         'travel) because the gate position for an unknown '
                         'mounting is unknown, and mid travel is the least '
                         'likely to be hard against a stop')
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

    # The board's own limits on this output. READ, never written: a clamped
    # command is accepted and then silently altered, which would leave the
    # servo somewhere other than the number on screen and corrupt exactly the
    # calibration this tool exists to take.
    smin = read_param(m, f'SERVO{args.channel}_MIN')
    smax = read_param(m, f'SERVO{args.channel}_MAX')

    print(f"\ngate on ch{args.channel}. HOPPER EMPTY, props off.")
    print(f"angle readout uses {args.us_per_deg:g} us/deg and is DERIVED from "
          f"the pulse width, not measured: the servo reports nothing back.")
    if smin is not None and smax is not None:
        print(f"board limits SERVO{args.channel}_MIN/_MAX = {smin:.0f}-"
              f"{smax:.0f}us. Outside that window ArduPilot may clamp, so a "
              f"gate that STOPS MOVING while the printed number keeps climbing "
              f"has hit the clamp, not a mechanical stop.")
    else:
        print(f"could NOT read SERVO{args.channel}_MIN/_MAX, so the clamp "
              f"warning is unavailable this run. A gate that stops moving while "
              f"the number climbs may be clamped rather than stopped.")
    print(f"nothing is refused here (user, 2026-08-12). The flight code's "
          f"{PWM_MIN_US}-{PWM_MAX_US}us guard in mavlink_io.py is UNTOUCHED "
          f"and still governs dropper.py, so widening it for a new travel is a "
          f"separate and deliberate decision.")
    print(HELP)

    pos = None          # last pulse width the board ACCEPTED
    ref = args.start    # angle readout zero
    marks = {}          # 'closed'/'open' -> us

    def goto(us, why):
        """Command a pulse width. Returns True only on an accepted ack."""
        nonlocal pos
        # WARN, NEVER REFUSE. Finding the travel of an uncharacterised mounting
        # is the job, and a tool that blocks at the old numbers cannot discover
        # the new ones.
        if smin is not None and smax is not None and not smin <= us <= smax:
            print(f"  WARNING: {us}us is outside SERVO{args.channel}_MIN/_MAX "
                  f"({smin:.0f}-{smax:.0f}us). ArduPilot may clamp it, in which "
                  f"case the servo is NOT where the line below says. Confirm by "
                  f"eye before marking this position.")
        if not SANE_MIN_US <= us <= SANE_MAX_US:
            print(f"  WARNING: {us}us is outside the conventional "
                  f"{SANE_MIN_US}-{SANE_MAX_US}us servo command band. That band "
                  f"is a typo catcher, not an MG90 spec (no datasheet was "
                  f"consulted), but a real typo lands here. If the servo buzzes "
                  f"and stops moving, it is STALLED against a stop: back off.")
        res = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                           args.channel, us, timeout=ack_timeout)
        if res != 'MAV_RESULT_ACCEPTED':
            print(f"  {us}us -> {res}   (nothing moved)")
            if res == 'NO ACK':
                print(f"  No ack at all: check the link, and that QGC is "
                      f"closed since it owns the port while connected.")
            else:
                print(f"  The autopilot refused the command. Check "
                      f"SERVO{args.channel}_FUNCTION=0 and "
                      f"SERVO{args.channel}_MIN/_MAX brackets {us}.")
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
        target = val if kind == 'abs' else pos + val
        move_deg = abs(target - pos) / args.us_per_deg
        if move_deg > 45:
            print(f"  NOTE: that is a {move_deg:.0f} degree move in one go. "
                  f"Near a mechanical stop, small steps are how you avoid "
                  f"driving the gate into one at full speed.")
        goto(target, raw)

    # --- summary --------------------------------------------------------------
    print(f"\nGATE LEFT AT {pos}us. It was NOT closed automatically, because "
          f"this tool has no way of knowing which end closed means any more. "
          f"Check it by eye before loading the hopper.")

    if 'closed' in marks and 'open' in marks:
        c, o = marks['closed'], marks['open']
        travel = abs(o - c) / args.us_per_deg
        way = 'counter-clockwise' if o < c else 'clockwise'
        print(f"\nMARKED:  closed {c}us   open {o}us")
        print(f"  travel {abs(o - c)}us = about {travel:.0f} deg {way} to open "
              f"(derived from {args.us_per_deg:g} us/deg, so trust the "
              f"microseconds and treat the degrees as a description)")
        outside = [f"{v}us" for v in (c, o)
                   if not PWM_MIN_US <= v <= PWM_MAX_US]
        if outside:
            print(f"  NOTE: {', '.join(outside)} falls outside the flight "
                  f"code's {PWM_MIN_US}-{PWM_MAX_US}us guard, so dropper.py "
                  f"would REFUSE to construct with these values. Widening "
                  f"mavlink_io.PWM_MIN_US/PWM_MAX_US is then part of the same "
                  f"change, and it is a real decision: that guard is what "
                  f"catches a typo before it reaches a servo in flight.")
        print(f"\nTELL THE ASSISTANT THESE TWO NUMBERS. They belong in TWO "
              f"places and only one of them is currently right:")
        print(f"  uno_q/dropper.py   DEFAULT_CLOSED_US = {c}"
              f"    DEFAULT_OPEN_US = {o}")
        print(f"  uno_q/run_mission.py  --servo-closed-us / --servo-open-us "
              f"still hard-code the OLD 1000/1900 pair instead of reading "
              f"dropper.py, so the flight runner would ignore the values above "
              f"entirely. That is a live bug, recorded in PROJECT_STATE "
              f"2026-08-12, and it must be fixed in the same change.")
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
