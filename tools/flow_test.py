#!/usr/bin/env python3
"""Bench: measure the hopper's granule flow rate, grams per second.

    ./python tools/flow_test.py                 # default 0.5 s and 2.0 s
    ./python tools/flow_test.py --dwells 0.5 1 2 3 --repeats 2

NOT NEEDED TO FLY. This exists because the dropper now varies the DOSE by
holding the gate open longer for a bigger puddle, and until this is measured
those seconds are proportional to nothing: the mission can say "1.4 s" but
nobody can say how many grams that is, which is what a report or a judge will
ask.

WHY MORE THAN ONE DWELL: the useful question is not the average rate, it is
whether flow is LINEAR in time. A gate that takes 200 ms to swing open and
settle delivers far less than half of a 1 s dose in 0.5 s, so a single
measurement extrapolates wrongly at exactly the short doses small puddles get.
Two dwells reveal the offset; three or more let you see the shape.

Aircraft on the bench, hopper loaded, a tray or paper under the gate. Nothing
here arms anything or touches a motor: it only commands the servo channel.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'uno_q'))

from pymavlink import mavutil                                    # noqa: E402
from mavlink_link import connect, send_and_ack                   # noqa: E402
from dropper import PixhawkServoDropper as Gate                  # noqa: E402


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        sys.exit("\nno console input; run this in a terminal")


def cycle(m, ch, open_us, closed_us, dwell, ack_timeout):
    """Open the gate for `dwell` seconds, then close it. Returns True if both
    commands were accepted, because a refused open with an accepted close
    would otherwise read as a zero-gram result rather than a failed test."""
    t0 = time.time()
    a = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, ch, open_us,
                     timeout=ack_timeout)
    time.sleep(dwell)
    b = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, ch, closed_us,
                     timeout=ack_timeout)
    actual = time.time() - t0
    ok = a == 'MAV_RESULT_ACCEPTED' and b == 'MAV_RESULT_ACCEPTED'
    if not ok:
        print(f"    gate commands: open {a}, close {b}")
    # The commanded dwell and the real one differ by the ack round trips,
    # which over a radio link is not negligible. Report what actually
    # happened, and divide by that, not by the number you asked for.
    return ok, actual


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dwells', type=float, nargs='+', default=[0.5, 2.0],
                    help='gate-open times to measure, seconds')
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--channel', type=int, default=9)
    ap.add_argument('--open-us', type=int, default=Gate.DEFAULT_OPEN_US)
    ap.add_argument('--closed-us', type=int, default=Gate.DEFAULT_CLOSED_US)
    ap.add_argument('--conn', default=None)
    ap.add_argument('--baud', type=int, default=None)
    args = ap.parse_args()

    m, _, args.baud = connect(args.conn, args.baud)
    ack_timeout = 5.0 if args.baud > 57600 else 12.0
    print(f"gate on ch{args.channel}: {args.closed_us}us closed -> "
          f"{args.open_us}us open")
    print("hopper loaded, tray under the gate, and weigh the TRAY EMPTY first "
          "so you can subtract it.\n")

    results = []
    for dwell in args.dwells:
        for r in range(args.repeats):
            label = f"{dwell:g}s" + (f" #{r + 1}" if args.repeats > 1 else "")
            ask(f"  [{label}] empty the tray, then press Enter to open ")
            ok, actual = cycle(m, args.channel, args.open_us, args.closed_us,
                               dwell, ack_timeout)
            if not ok:
                print("    SKIPPED: the gate did not cycle cleanly")
                continue
            g = ask(f"    gate was open {actual:.2f}s. Grams caught? ")
            try:
                grams = float(g)
            except ValueError:
                print("    not a number, skipping this one")
                continue
            results.append((dwell, actual, grams))
            print(f"    {grams:.1f} g in {actual:.2f}s = "
                  f"{grams / actual:.1f} g/s\n")

    if not results:
        sys.exit("no measurements taken")

    print("\n=== RESULTS ===")
    print(f"{'asked':>7}{'actual':>8}{'grams':>8}{'g/s':>8}")
    for dwell, actual, grams in results:
        print(f"{dwell:>7.2f}{actual:>8.2f}{grams:>8.1f}{grams / actual:>8.1f}")

    # Straight-line fit through the (time, grams) points. The INTERCEPT is the
    # interesting number: markedly negative means the gate wastes time opening
    # before anything flows, so short doses under-deliver and the mission's
    # dose_s_min needs raising above that threshold.
    if len(results) >= 2:
        xs = [a for _, a, _ in results]
        ys = [g for _, _, g in results]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        denom = sum((x - mx) ** 2 for x in xs)
        if denom > 0:
            slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
            intercept = my - slope * mx
            print(f"\nfit: grams = {slope:.1f} * seconds + {intercept:+.1f}")
            print(f"  flow rate       : {slope:.1f} g/s")
            if intercept < -0.05 * slope:
                lost = -intercept / slope
                print(f"  gate opening lag: about {lost:.2f}s of the dwell "
                      f"delivers nothing, so raise MissionConfig dose_s_min "
                      f"above it or short doses will under-deliver")
            else:
                print("  no meaningful opening lag: flow is close to linear "
                      "in time, so dose seconds scale honestly")
    print("\nPut the g/s figure in PROJECT_STATE, and set "
          "MissionConfig.dose_s_per_m2 from the grams you actually want per "
          "square metre of water.")


if __name__ == '__main__':
    main()
