#!/usr/bin/env python3
"""Bench: measure the hopper's granule flow rate, grams per second.

    ./python tools/flow_test.py                 # default 0.5 s and 2.0 s
    ./python tools/flow_test.py --dwells 0.5 1 2 3 --repeats 2
    ./python tools/flow_test.py --cycles 20     # 20 opens, one weighing
    ./python tools/flow_test.py --grain-mg 0.6  # enter grain count, not grams

Converts the mission's gate-open seconds into grams, which is what a dose is
specified in.

Two options for doses below the scale's resolution:
  --cycles N    fire N times into the same tray, weigh once, divide by N.
                Also averages out shot-to-shot scatter.
  --grain-mg X  enter a grain count instead of grams. A 1 mm mustard seed is
                about 0.6 mg. Calibrate X by counting 200 and weighing them.

Several dwells rather than one, because the question is whether flow is linear
in time: a gate takes time to swing open, so short doses can under-deliver.

Aircraft on the bench, hopper loaded, tray under the gate. Commands the servo
channel only, nothing arms and no motor turns.
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

# Bti label rates, VectoBac G (Valent BioSciences) 2.5-20 lb/acre over
# 4046.86 m2/acre. Low band is 1st-2nd instar larvae, high band 3rd-4th or
# polluted water. Surrogate granules give the right seconds, not the right
# grams: recalibrate on real Bti.
BTI_G_PER_M2_LOW = 0.28
BTI_G_PER_M2_TYPICAL = 1.1
BTI_G_PER_M2_HIGH = 2.24


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        sys.exit("\nno console input; run this in a terminal")


def cycle(m, ch, open_us, closed_us, dwell, ack_timeout):
    """Open the gate for `dwell` seconds, then close. True if both acked."""
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
    # Measured, not commanded: ack round trips add to the open time.
    return ok, actual


def run_cycles(m, ch, open_us, closed_us, dwell, ack_timeout, cycles):
    """Fire the gate `cycles` times into one tray -> (ok, total_open_seconds).

    Aborts on the first bad cycle, since a partial run weighed as a full one
    reads as a low flow rate.
    """
    total = 0.0
    for i in range(cycles):
        ok, actual = cycle(m, ch, open_us, closed_us, dwell, ack_timeout)
        if not ok:
            print(f"    cycle {i + 1} of {cycles} did not complete")
            return False, total
        total += actual
        if cycles > 1:
            time.sleep(0.4)     # servo settles, granule column re-packs
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
                    help='gate opens per weighing, 10-20 for small doses')
    ap.add_argument('--grain-mg', type=float, default=None,
                    help='milligrams per grain; prompts for a count not grams')
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
        print(f"{args.cycles} opens per weighing. Empty the tray between "
              f"dwells only, not between opens.")
    if args.grain_mg:
        print(f"entering GRAIN COUNTS, at {args.grain_mg:g} mg per grain")
    print("hopper loaded, tray under the gate, tray weighed empty first.\n")

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
                print("    nothing caught, hopper not flowing. Not recorded.")
                continue
            total_g = caught * args.grain_mg / 1000.0 if args.grain_mg \
                else caught
            # Per cycle, so every figure below describes ONE dose.
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

    # Supply check first, since a hopper running dry mid-dwell fakes the same
    # positive intercept as a fixed slug. The discriminator is g/s falling as
    # dwell rises.
    starved = False
    if len(results) >= 2:
        by_dwell = sorted(results, key=lambda r: r[1])
        first = by_dwell[0][2] / by_dwell[0][1]
        last = by_dwell[-1][2] / by_dwell[-1][1]
        if first > 0 and last < 0.7 * first:
            starved = True
            print(f"\n*** SUPPLY-LIMITED: refill the hopper and re-run ***")
            print(f"  g/s fell {100 * (1 - last / first):.0f}%, "
                  f"{first:.2f} at the shortest dwell to {last:.2f} at the "
                  f"longest. Only a hopper running dry does that.")
            print(f"  Best available figure is the shortest dwell, "
                  f"{first:.2f} g/s.")

    slope = None            # stays None if the fit does not run
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
                      f"delivers nothing. Raise MissionConfig dose_s_min "
                      f"above it.")
            elif intercept > 0.05 * slope and starved:
                print(f"  intercept {intercept:+.2f} g is the supply decay "
                      f"above, not a slug. Refill and re-run.")
            elif intercept > 0.05 * slope:
                # Granules already past the gate fall at gate-crack, so flow
                # is linear but does not pass through the origin.
                print(f"  fixed slug per open: about {intercept:.2f} g falls "
                      f"regardless of dwell. Minimum dose; only a smaller "
                      f"aperture reduces it.")
                print(f"  dwell for a target mass is "
                      f"(grams - {intercept:.2f}) / {slope:.2f}")
            else:
                print("  intercept near zero: dose seconds scale "
                      "proportionally")
    # Fitted slope when available, plain mean otherwise; the basis is printed.
    rate = None
    offset = 0.0            # grams at zero dwell, the fixed slug
    if slope is not None and slope > 0:
        rate, basis = slope, 'fitted slope'
        offset = intercept
    elif results:
        rate = sum(g / a for _, a, g in results) / len(results)
        basis = 'mean of %d measurement(s)' % len(results)
    if rate and rate > 0 and starved:
        print("\n=== DOSE: NOT COMPUTED ===")
        print("The hopper ran dry mid-measurement. Refill and re-run.")
    elif rate and rate > 0:
        print(f"\n=== DOSE, at {rate:.2f} g/s ({basis}) ===")
        print("Bti label rates: VectoBac G 2.5-20 lb/acre = "
              f"{BTI_G_PER_M2_LOW}-{BTI_G_PER_M2_HIGH} g/m2, "
              f"about {BTI_G_PER_M2_TYPICAL} g/m2 mid-label.")
        if offset > 0:
            print(f"Dwell solves grams = {rate:.2f}*s + {offset:.2f}. Any "
                  f"puddle wanting under {offset:.2f} g shows as 0.00s.")
        print(f"{'puddle':>9}{'grams':>9}{'seconds':>9}")
        for area in (1, 2, 5, 10, 20):
            grams = area * BTI_G_PER_M2_TYPICAL
            secs = (grams - offset) / rate
            flag = ''
            if secs <= 0:
                secs, flag = 0.0, '  <-- slug alone overshoots'
            print(f"{area:>7} m2{grams:>9.1f}{secs:>9.2f}{flag}")
        print("Measured on a surrogate granule, so the seconds carry over and "
              "the grams do not. Re-run on real Bti to quote a mass.")
        smallest = max(0.0, (BTI_G_PER_M2_TYPICAL - offset) / rate)
        if smallest < 0.3:
            print(f"\nNOTE: a 1 m2 puddle is {smallest:.2f}s of gate time, "
                  f"near the servo's own travel time. Shrink the aperture, or "
                  f"set MissionConfig.dose_s_min to this floor.")
    print("\nRecord the g/s figure with the material beside it, and set "
          "MissionConfig.dose_s_per_m2 from the target grams per square metre.")


if __name__ == '__main__':
    main()
