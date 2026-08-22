#!/usr/bin/env python3
"""Bench: measure the hopper's granule flow rate, grams per second.

    ./python tools/flow_test.py                 # default 0.5 s and 2.0 s
    ./python tools/flow_test.py --dwells 0.5 1 2 3 --repeats 2
    ./python tools/flow_test.py --cycles 20     # 20 opens, ONE weighing
    ./python tools/flow_test.py --grain-mg 0.6  # enter grain COUNT, not grams

WHEN THE SCALE CANNOT SEE ONE DOSE (2026-08-22, user: "grains are falling but
they are too few for the weighing machine"). A 1 s dose can be a few tenths of
a gram, under the resolution of a kitchen scale. Two ways out, and --cycles is
the better one:

  --cycles N   fire the gate N times into the SAME tray, weigh once, divide by
               N. A 0.3 g dose becomes 6 g at N=20, which any scale reads, and
               averaging N shots also removes the shot-to-shot scatter that a
               single dose has. Needs no extra constant, so nothing new can be
               wrong. USE THIS unless you have a reason not to.
  --grain-mg X enter a GRAIN COUNT at the prompt instead of grams; the tool
               multiplies by X mg. Honest only for counts small enough to
               actually count: a 1 mm mustard seed is about 0.6 mg, so one
               gram is ~1700 seeds. Use it for tiny doses, or to calibrate the
               per-grain mass in the first place (count 200, weigh them, X =
               milligrams / 200).

The two combine: --cycles 20 --grain-mg 0.6 counts the grains from 20 shots.

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

# BTI FIELD DOSE, for turning a measured g/s into a dose in seconds.
# Sources, both label rates, looked up 2026-08-22:
#   VectoBac G (Valent BioSciences): 2.5-20 lb/acre, the lower 2.5-10 band for
#     1st-2nd instar larvae and 10-20 for 3rd-4th instars or polluted water.
#     1 acre = 4046.86 m2, so 2.5 lb/acre = 0.28 g/m2 and 20 lb/acre = 2.24.
#   Summit Mosquito Bits: 1 tsp per 25 sq ft = 1 tsp per 2.32 m2. At an
#     ASSUMED 2.5 g per teaspoon of corn-cob granule that is ~1.1 g/m2.
# THE TEASPOON-TO-GRAMS STEP IS AN ASSUMPTION, not a label figure, so the
# Bits number is corroboration for the VectoBac range and not independent
# evidence. Use the range, not the midpoint, when anything is at stake.
# NONE OF THIS TRANSFERS TO THE SURROGATE: salt and sooji are stand-ins with
# their own densities, so a dwell calibrated on salt delivers the right
# SECONDS and the wrong GRAMS of anything else. Recalibrate for real Bti.
BTI_G_PER_M2_LOW = 0.28
BTI_G_PER_M2_TYPICAL = 1.1
BTI_G_PER_M2_HIGH = 2.24


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


def run_cycles(m, ch, open_us, closed_us, dwell, ack_timeout, cycles):
    """Fire the gate `cycles` times into the same tray. Returns
    (ok, total_open_seconds). Aborts the whole measurement on the first bad
    cycle rather than pressing on: a partial run weighed as a full one reports
    a low flow rate, which is the same wrong answer as a bridged hopper and
    would be indistinguishable from it afterwards."""
    total = 0.0
    for i in range(cycles):
        ok, actual = cycle(m, ch, open_us, closed_us, dwell, ack_timeout)
        if not ok:
            print(f"    cycle {i + 1} of {cycles} did not complete")
            return False, total
        total += actual
        if cycles > 1:
            # Let the servo settle and the column re-pack between shots, so
            # shot N+1 starts from the same state as shot 1.
            time.sleep(0.4)
    return True, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dwells', type=float, nargs='+', default=[0.5, 2.0],
                    help='gate-open times to measure, seconds')
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--channel', type=int, default=9)
    ap.add_argument('--open-us', type=int, default=Gate.DEFAULT_OPEN_US)
    ap.add_argument('--closed-us', type=int, default=Gate.DEFAULT_CLOSED_US)
    ap.add_argument('--cycles', type=int, default=1,
                    help='gate opens per weighing. Use 10-20 when one dose is '
                         'below the scale resolution; the total is divided by '
                         'this to give the per-dose figure.')
    ap.add_argument('--grain-mg', type=float, default=None,
                    help='milligrams per grain. If given, the prompt asks for '
                         'a GRAIN COUNT instead of grams.')
    ap.add_argument('--conn', default=None)
    ap.add_argument('--baud', type=int, default=None)
    args = ap.parse_args()
    if args.cycles < 1:
        ap.error('--cycles must be at least 1')
    if args.grain_mg is not None and args.grain_mg <= 0:
        ap.error('--grain-mg must be positive')

    m, _, args.baud = connect(args.conn, args.baud)
    ack_timeout = 5.0 if args.baud > 57600 else 12.0
    print(f"gate on ch{args.channel}: {args.closed_us}us closed -> "
          f"{args.open_us}us open")
    if args.cycles > 1:
        print(f"{args.cycles} opens per weighing; do NOT empty the tray "
              f"between them, only between dwells.")
    if args.grain_mg:
        print(f"entering GRAIN COUNTS, at {args.grain_mg:g} mg per grain")
    print("hopper loaded, tray under the gate, and weigh the TRAY EMPTY first "
          "so you can subtract it.\n")

    results = []
    for dwell in args.dwells:
        for r in range(args.repeats):
            label = f"{dwell:g}s" + (f" #{r + 1}" if args.repeats > 1 else "")
            ask(f"  [{label}] empty the tray, then press Enter to open ")
            ok, total_open = run_cycles(m, args.channel, args.open_us,
                                        args.closed_us, dwell, ack_timeout,
                                        args.cycles)
            if not ok:
                print("    SKIPPED: the gate did not cycle cleanly")
                continue
            unit = 'Grains' if args.grain_mg else 'Grams'
            shots = (f" over {args.cycles} opens" if args.cycles > 1 else "")
            raw = ask(f"    gate was open {total_open:.2f}s total{shots}. "
                      f"{unit} caught? ")
            try:
                caught = float(raw)
            except ValueError:
                print("    not a number, skipping this one")
                continue
            if caught <= 0:
                print("    nothing caught: the hopper is not flowing, and a "
                      "zero is not a rate. Not recorded.")
                continue
            total_g = caught * args.grain_mg / 1000.0 if args.grain_mg \
                else caught
            # Per-cycle, so every downstream number (the fit, the g/s, the
            # dose) is about ONE dose, which is what the mission commands.
            grams = total_g / args.cycles
            actual = total_open / args.cycles
            results.append((dwell, actual, grams))
            if args.cycles > 1:
                print(f"    {total_g:.2f} g total / {args.cycles} = "
                      f"{grams:.3f} g per dose")
            print(f"    {grams:.3f} g in {actual:.2f}s = "
                  f"{grams / actual:.2f} g/s\n")

    if not results:
        sys.exit("no measurements taken")

    print("\n=== RESULTS (per dose) ===")
    print(f"{'asked':>7}{'actual':>8}{'grams':>9}{'g/s':>8}")
    for dwell, actual, grams in results:
        print(f"{dwell:>7.2f}{actual:>8.2f}{grams:>9.3f}{grams / actual:>8.2f}")

    # Straight-line fit through the (time, grams) points. The INTERCEPT is the
    # interesting number: markedly negative means the gate wastes time opening
    # before anything flows, so short doses under-deliver and the mission's
    # dose_s_min needs raising above that threshold.
    slope = None            # stays None unless the fit below actually runs
    if len(results) >= 2:
        xs = [a for _, a, _ in results]
        ys = [g for _, _, g in results]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        denom = sum((x - mx) ** 2 for x in xs)
        if denom > 0:
            slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
            intercept = my - slope * mx
            print(f"\nfit: grams = {slope:.2f} * seconds + {intercept:+.2f}")
            print(f"  flow rate       : {slope:.2f} g/s")
            if intercept < -0.05 * slope:
                lost = -intercept / slope
                print(f"  gate opening lag: about {lost:.2f}s of the dwell "
                      f"delivers nothing, so raise MissionConfig dose_s_min "
                      f"above it or short doses will under-deliver")
            else:
                print("  no meaningful opening lag: flow is close to linear "
                      "in time, so dose seconds scale honestly")
    # What the rate MEANS, in the only units a dosing decision is made in.
    # Uses the straight mean rather than the fit's slope when there is only
    # one point, and says which it used, because a judge asking "how many
    # grams" deserves to know whether that came from one measurement.
    rate = None
    if slope is not None and slope > 0:
        rate, basis = slope, 'fitted slope'
    elif results:
        rate = sum(g / a for _, a, g in results) / len(results)
        basis = 'mean of %d measurement(s)' % len(results)
    if rate and rate > 0:
        print(f"\n=== DOSE, at {rate:.2f} g/s ({basis}) ===")
        print("Bti label rates: VectoBac G 2.5-20 lb/acre = "
              f"{BTI_G_PER_M2_LOW}-{BTI_G_PER_M2_HIGH} g/m2, "
              f"about {BTI_G_PER_M2_TYPICAL} g/m2 mid-label.")
        print(f"{'puddle':>9}{'grams':>9}{'seconds':>9}")
        for area in (1, 2, 5, 10, 20):
            grams = area * BTI_G_PER_M2_TYPICAL
            print(f"{area:>7} m2{grams:>9.1f}{grams / rate:>9.2f}")
        print("READ THIS BEFORE QUOTING ANY OF IT: the seconds are real, the "
              "grams are not. They were measured with a SURROGATE (salt, "
              "sooji, seeds), whose density and grain size are not Bti's, so "
              "the same dwell delivers a different mass of the real product. "
              "Re-run this with actual Bti granules before any number here "
              "goes in front of a judge.")
        smallest = BTI_G_PER_M2_TYPICAL / rate
        if smallest < 0.3:
            print(f"\nNOTE: a 1 m2 puddle is only {smallest:.2f}s of gate "
                  f"time, which is close to the servo's own travel time. At "
                  f"that scale the dose is set by how fast the gate moves, "
                  f"not by the dwell, so either shrink the hole or accept a "
                  f"floor dose and set MissionConfig.dose_s_min to it.")
    print("\nPut the g/s figure in PROJECT_STATE with the MATERIAL beside it, "
          "and set MissionConfig.dose_s_per_m2 from the grams you actually "
          "want per square metre of water.")


if __name__ == '__main__':
    main()
