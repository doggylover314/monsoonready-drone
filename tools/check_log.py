#!/usr/bin/env python3
"""First-look analysis of an ArduPilot .bin log. Run this after EVERY flight,
before drawing any conclusion from memory of what the aircraft looked like.

    training/.venv/bin/python tools/check_log.py ~/logs/00000012.BIN

Reports, in the order they can stop the project:

  VIBRATION   VibeX/Y/Z while the motors are actually running, plus the
              accelerometer clipping counters. VibeZ median < 15 is THE gate
              before AltHold or Loiter (PROJECT_STATE FLIGHT GATES): this
              airframe previously ran a median of 20.6 with peaks over 60,
              and vibration-corrupted altitude is what destroyed it once.
  CLIPPING    Clip0/1/2 counting up means the accelerometer is saturating.
              Any clipping at hover invalidates the vibration numbers and
              the EKF's opinion of altitude with them.
  GPS         worst sats / worst HDOP seen while flying. The C3 crash was
              armed seconds after power-on at HDOP 65-99.
  HOVER       learned hover throttle. Above ~0.5 at full payload means the
              thrust margin is too thin (PROJECT_STATE micro-facts).
  ERRORS      ERR subsystem events and the autopilot's own messages, which
              is where failsafes and EKF complaints appear.

WHY THE THROTTLE FILTER: vibration on the bench is near zero, so including
the pre-arm and post-land parts of the log drags the median down and can
turn a failing aircraft into a passing number. Only samples taken while the
last-seen throttle was above --min-throttle are counted, and the count of
included samples is printed so a nearly-empty sample set is obvious.
"""

import argparse
import statistics
import sys

from pymavlink import mavutil

VIBE_GATE = 15.0          # VibeZ median, PROJECT_STATE flight gate


def pct(values, p):
    """Percentile without numpy (this runs anywhere pymavlink does)."""
    if not values:
        return float('nan')
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logfile')
    ap.add_argument('--min-throttle', type=float, default=0.2,
                    help='only count vibration while throttle-out exceeds '
                         'this (0-1). Default 0.2 = motors clearly running.')
    args = ap.parse_args()

    try:
        m = mavutil.mavlink_connection(args.logfile)
    except Exception as exc:                              # noqa: BLE001
        sys.exit(f"cannot open {args.logfile}: {exc}")

    vx, vy, vz = [], [], []
    clip = [0, 0, 0]
    thr = 0.0
    thr_max = 0.0
    sats_min, hdop_max = None, None
    errs, msgs = [], []
    hover_learned = None
    volt_min = None

    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()
        if t == 'CTUN':
            thr = getattr(msg, 'ThO', thr)
            thr_max = max(thr_max, thr)
        elif t == 'VIBE':
            if thr >= args.min_throttle:
                vx.append(msg.VibeX)
                vy.append(msg.VibeY)
                vz.append(msg.VibeZ)
                clip[0] = max(clip[0], getattr(msg, 'Clip0', 0))
                clip[1] = max(clip[1], getattr(msg, 'Clip1', 0))
                clip[2] = max(clip[2], getattr(msg, 'Clip2', 0))
        elif t == 'GPS':
            if thr >= args.min_throttle:
                ns = getattr(msg, 'NSats', None)
                hd = getattr(msg, 'HDop', None)
                if ns is not None:
                    sats_min = ns if sats_min is None else min(sats_min, ns)
                if hd is not None:
                    hdop_max = hd if hdop_max is None else max(hdop_max, hd)
        elif t == 'ERR':
            errs.append((msg.Subsys, msg.ECode))
        elif t == 'MSG':
            msgs.append(msg.Message)
        elif t == 'PARM':
            if msg.Name == 'MOT_THST_HOVER':
                hover_learned = msg.Value
        elif t == 'BAT':
            v = getattr(msg, 'Volt', None)
            if v and thr >= args.min_throttle:
                volt_min = v if volt_min is None else min(volt_min, v)

    print(f"\n=== {args.logfile} ===")
    if not vz:
        print(f"NO vibration samples above throttle {args.min_throttle}. "
              f"Highest throttle seen was {thr_max:.2f}. Either the aircraft "
              f"never spun up in this log, or it is a bench log.")
        return

    print(f"\nVIBRATION  ({len(vz)} samples while throttle > "
          f"{args.min_throttle})")
    for name, v in (('VibeX', vx), ('VibeY', vy), ('VibeZ', vz)):
        print(f"  {name}  median {statistics.median(v):5.1f}   "
              f"95th {pct(v, 95):5.1f}   max {max(v):5.1f}")
    zmed = statistics.median(vz)
    gate = zmed < VIBE_GATE
    print(f"  GATE: VibeZ median {zmed:.1f} vs < {VIBE_GATE:.0f}  ->  "
          f"{'PASS' if gate else 'FAIL, do not progress to AltHold/Loiter'}")

    print(f"\nCLIPPING   Clip0 {clip[0]}  Clip1 {clip[1]}  Clip2 {clip[2]}")
    if any(clip):
        print("  ANY clipping invalidates the vibration figures above and the "
              "EKF's altitude with them. Fix the mounting before believing a "
              "passing median.")

    print(f"\nGPS        worst sats {sats_min}   worst HDOP "
          f"{hdop_max if hdop_max is None else round(hdop_max, 2)}")
    if sats_min is not None and sats_min < 10:
        print("  BELOW the 10-sat rule from CRASH LESSONS.")
    if hdop_max is not None and hdop_max > 1.5:
        print("  ABOVE the HDOP 1.5 rule from CRASH LESSONS.")

    if hover_learned is not None:
        print(f"\nHOVER      MOT_THST_HOVER {hover_learned:.3f}")
        if hover_learned > 0.5:
            print("  Above ~0.5: thrust margin is thin, trim payload.")
    if volt_min is not None:
        print(f"\nBATTERY    lowest in-flight {volt_min:.2f} V "
              f"(BATT_LOW_VOLT 10.8 / CRT 10.2)")

    print(f"\nERRORS     {len(errs)} ERR events")
    for sub, code in errs[:20]:
        print(f"  subsys {sub} code {code}")
    interesting = [s for s in msgs
                   if any(k in s for k in ('EKF', 'Vibration', 'Failsafe',
                                           'failsafe', 'Error', 'PreArm',
                                           'Glitch', 'Bad'))]
    if interesting:
        print(f"\nMESSAGES   {len(interesting)} of note")
        for s in interesting[:25]:
            print(f"  {s}")


if __name__ == '__main__':
    main()
