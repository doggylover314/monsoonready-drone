#!/usr/bin/env python3
"""First-look analysis of an ArduPilot .bin log. Run this after EVERY flight,
before drawing any conclusion from memory of what the aircraft looked like.

    ./python tools/check_log.py ~/logs/00000037.BIN

EXIT CODE IS THE VERDICT: 0 = analysed and cleared, 1 = a gate failed, 2 = the
log could not be judged. Chaining `check_log.py x.BIN && next-step` is therefore
safe; before 2026-08-08 it was not, because every path exited 0.

Reports, in the order they can stop the project:

  VIBRATION   VibeX/Y/Z PER IMU INSTANCE while the motors are running. VibeZ
              median < 15 on EVERY instance is THE gate before AltHold or
              Loiter. Per instance matters: the EKF flies on one IMU, and
              pooling instances lets a bad one hide behind good ones.
  CLIPPING    the in-window RISE in each IMU's clip counter. The counter is
              cumulative since boot, so the raw value includes bench handling.
  GPS         worst sats/HDOP BEFORE the motors spun (the arming window, which
              is the one the C3 crash was about) and again while flying.
  COMPASS     magnetic field strength per instance, split BEFORE the motors
              spin and while flying. The split is the diagnosis: high at rest
              means the calibration is stale or something magnetic sits near
              the compass, while low at rest and high under throttle means
              current in nearby wiring. Recalibrating only fixes the first.
  HOVER       learned hover throttle from CTUN.ThH. Not from PARM: PARM only
              carries the value at log start, and hover learning never writes
              a PARM record, so reading it there always reports the old value.
  BATTERY     lowest voltage and total consumed. The consumed figure is worth
              comparing against what your charger puts back in: that is the
              only independent check on the current sensor's calibration.
  ERRORS      ERR events decoded to subsystem names, plus autopilot messages.

WHY THE THROTTLE FILTER, AND WHERE IT DOES NOT APPLY: vibration on the bench is
near zero, so including pre-arm samples drags the median down and can turn a
failing aircraft into a passing number. It is applied to VIBE only. GPS quality
is deliberately NOT filtered, because arming happens at zero throttle and the
arming window is exactly what the GPS rule exists to police.
"""

import argparse
import math
import statistics
import sys

from pymavlink import mavutil

VIBE_GATE = 15.0          # VibeZ median, PROJECT_STATE flight gate
MIN_SATS = 10             # CRASH LESSONS rule
MAX_HDOP = 1.5
MAX_HOVER_THR = 0.5       # above this the thrust margin is too thin
# Milligauss. NOT a universal constant: ArduPilot's arming check compares the
# measured field against the field expected at YOUR location, and 875 is the
# ceiling this aircraft itself printed at the farm ("Check mag field (1038,
# max 875)", logs 43 and 45). Somewhere with a different earth field the
# board will quote a different number, so treat this as the site figure the
# aircraft gave us rather than a rule from the wiki.
MAG_FIELD_GATE = 875.0

# ArduPilot LogErrorSubsystem, the ones worth naming. Unlisted codes print raw.
ERR_SUBSYS = {
    2: 'RADIO', 3: 'COMPASS', 5: 'FAILSAFE_RADIO', 6: 'FAILSAFE_BATT',
    7: 'FAILSAFE_GPS', 8: 'FAILSAFE_GCS', 9: 'FAILSAFE_FENCE', 10: 'FLIGHT_MODE',
    11: 'GPS', 12: 'CRASH_CHECK', 13: 'FLIP', 15: 'PARACHUTE',
    16: 'EKFCHECK', 17: 'FAILSAFE_EKFINAV', 18: 'BARO', 19: 'CPU',
    21: 'ADSB', 22: 'TERRAIN', 23: 'NAVIGATION', 24: 'FAILSAFE_TERRAIN',
    25: 'EKF_PRIMARY', 26: 'THRUST_LOSS_CHECK', 27: 'FAILSAFE_SENSORS',
    28: 'FAILSAFE_LEAK', 29: 'PILOT_INPUT', 30: 'FAILSAFE_VIBE',
}
# Subsystems whose presence should fail the run outright.
ERR_FATAL = {12, 16, 17, 26, 30}

# Substrings worth surfacing from MSG. 'Crash' and 'Thrust' are here because
# "Crash: Disarming" and "Potential Thrust Loss (n)" are how ArduPilot reports
# the prop/motor failure mode this airframe has already suffered, and neither
# contains any of the generic keywords.
MSG_KEYS = ('EKF', 'Vibration', 'Failsafe', 'failsafe', 'Error', 'PreArm',
            'Glitch', 'Bad', 'Crash', 'Thrust', 'Motor', 'Yaw', 'Compass',
            'Baro', 'Internal')


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
                    help='only count VIBRATION while throttle-out exceeds '
                         'this (0-1). Default 0.2 = motors clearly running.')
    args = ap.parse_args()

    try:
        m = mavutil.mavlink_connection(args.logfile)
    except Exception as exc:                              # noqa: BLE001
        sys.exit(f"cannot open {args.logfile}: {exc}")

    vibe = {}                 # imu -> [VibeZ ...]        (throttle-gated)
    vibe_xy = {}              # imu -> ([VibeX], [VibeY])
    clip_first, clip_last = {}, {}
    thr, thr_max = 0.0, 0.0
    flight_s = 0.0
    last_ctun_t = None
    ctun_seen = False
    hover = []
    gps_pre, gps_fly = [], []   # (sats, hdop) before motors / while flying
    mag_pre, mag_fly = {}, {}   # instance -> [|B| mG] before motors / flying
    errs, msgs = [], []
    volt_min, consumed, energy = None, None, None
    peak_amp = 0.0
    # Battery-monitor calibration AS FLOWN. The board's current settings are not
    # evidence about an old flight: these are the only trustworthy source for
    # what scaling produced the amps in THIS log.
    #
    # TIMESTAMPED, AND THAT IS THE WHOLE POINT. A log records a parameter at
    # boot AND every time it changes, so a value written while the aircraft sat
    # on the ground AFTER landing appears LATER in the file than the one that
    # actually flew. Taking the last record burned this project once already:
    # log 39 was read as having flown at BATT_AMP_PERVLT=17 when 17 was set at
    # t=758s, 131 seconds after the motors stopped, and the flight really ran
    # at 90.6866. That single mistake produced a confident and completely wrong
    # "the current does not respond to the parameter" conclusion. Resolve every
    # parameter to its value AT THE MOMENT THE MOTORS STARTED.
    batt_parms = []          # (time_s, name, value), in log order
    first_fly_t = None       # time the motors first passed the throttle gate

    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()
        flying = thr >= args.min_throttle

        if t == 'CTUN':
            ctun_seen = True
            # Sum the time the motors were actually running, so endurance is
            # measured rather than guessed. A log can hold many arm/disarm
            # cycles, and CurrTot accumulates across all of them.
            ts = getattr(msg, 'TimeUS', 0) / 1e6
            if thr >= args.min_throttle and first_fly_t is None:
                first_fly_t = ts
            if thr >= args.min_throttle and last_ctun_t is not None:
                gap = ts - last_ctun_t
                if 0 < gap < 1.0:      # ignore gaps across a disarm
                    flight_s += gap
            last_ctun_t = ts
            thr = getattr(msg, 'ThO', thr)
            thr_max = max(thr_max, thr)
            if thr >= args.min_throttle:
                th = getattr(msg, 'ThH', None)
                if th is not None:
                    hover.append(th)
        elif t == 'VIBE':
            if flying:
                imu = getattr(msg, 'IMU', 0)
                vibe.setdefault(imu, []).append(msg.VibeZ)
                xy = vibe_xy.setdefault(imu, ([], []))
                xy[0].append(msg.VibeX)
                xy[1].append(msg.VibeY)
                # Cumulative since boot: record first and last inside the
                # window so the report can show the rise, not the lifetime.
                c = getattr(msg, 'Clip', None)
                if c is not None:
                    clip_first.setdefault(imu, c)
                    clip_last[imu] = c
        elif t == 'GPS':
            ns, hd = getattr(msg, 'NSats', None), getattr(msg, 'HDop', None)
            if ns is not None and hd is not None:
                (gps_fly if flying else gps_pre).append((ns, hd))
        elif t == 'MAG':
            # |B| from the RAW field, which is what the arming check tests.
            # Offsets are logged separately and must not be subtracted here.
            x = getattr(msg, 'MagX', None)
            y = getattr(msg, 'MagY', None)
            z = getattr(msg, 'MagZ', None)
            if None not in (x, y, z):
                (mag_fly if flying else mag_pre).setdefault(
                    getattr(msg, 'I', 0), []).append(math.sqrt(x*x + y*y + z*z))
        elif t == 'ERR':
            errs.append((msg.Subsys, msg.ECode))
        elif t == 'MSG':
            msgs.append(msg.Message)
        elif t == 'BAT':
            v = getattr(msg, 'Volt', None)
            if v is not None and flying:
                volt_min = v if volt_min is None else min(volt_min, v)
            peak_amp = max(peak_amp, getattr(msg, 'Curr', 0.0) or 0.0)
            consumed = getattr(msg, 'CurrTot', consumed)
            energy = getattr(msg, 'EnrgTot', energy)
        elif t == 'PARM':
            name = getattr(msg, 'Name', '')
            if name in ('BATT_AMP_PERVLT', 'BATT_AMP_OFFSET', 'BATT_VOLT_MULT',
                        'BATT_CURR_PIN', 'BATT_CAPACITY', 'BATT_MONITOR'):
                batt_parms.append((getattr(msg, 'TimeUS', 0) / 1e6, name,
                                   getattr(msg, 'Value', None)))

    print(f"\n=== {args.logfile} ===")

    # --- can this log be judged at all? -------------------------------------
    if not vibe:
        if not ctun_seen:
            print("CANNOT EVALUATE: no CTUN messages in this log, so throttle "
                  "is unknown and vibration cannot be gated. Check LOG_BITMASK "
                  "(the ATTITUDE_MED/CTUN bit) rather than assuming the "
                  "aircraft never flew.")
            sys.exit(2)
        print(f"NO vibration samples above throttle {args.min_throttle}. "
              f"Highest throttle seen was {thr_max:.2f}, so the motors never "
              f"spun up: this is a bench log, not a flight.")
        sys.exit(2)

    failed = []

    # --- vibration, per IMU instance ----------------------------------------
    print(f"\nVIBRATION  (throttle > {args.min_throttle}, "
          f"{len(vibe)} IMU instance(s))")
    worst = 0.0
    for imu in sorted(vibe):
        vz = vibe[imu]
        vx, vy = vibe_xy[imu]
        med = statistics.median(vz)
        worst = max(worst, med)
        flag = '' if med < VIBE_GATE else '   <-- OVER THE GATE'
        print(f"  IMU{imu}  n={len(vz):5d}  VibeZ median {med:5.1f}  "
              f"95th {pct(vz, 95):5.1f}  max {max(vz):5.1f}{flag}")
        print(f"          VibeX median {statistics.median(vx):5.1f}   "
              f"VibeY median {statistics.median(vy):5.1f}")
    if len(vibe) == 1:
        print("  (only one IMU instance logs VIBE on this board)")
    gate_ok = worst < VIBE_GATE
    print(f"  GATE: worst per-IMU VibeZ median {worst:.1f} vs < "
          f"{VIBE_GATE:.0f}  ->  "
          f"{'PASS' if gate_ok else 'FAIL, do not progress to AltHold/Loiter'}")
    if not gate_ok:
        failed.append('vibration gate')

    # --- clipping ------------------------------------------------------------
    rises = {i: clip_last[i] - clip_first[i] for i in clip_last}
    print("\nCLIPPING   (rise while the motors were running)")
    for imu in sorted(rises):
        print(f"  IMU{imu}  +{rises[imu]}")
    if any(rises.values()):
        print("  ANY clipping invalidates the vibration figures above and the "
              "EKF's altitude with them. Fix the mounting before believing a "
              "passing median.")
        failed.append('accelerometer clipping')

    # --- GPS, unfiltered by throttle ----------------------------------------
    print("\nGPS")
    for label, data in (('before motors (the arming window)', gps_pre),
                        ('while flying', gps_fly)):
        if not data:
            print(f"  {label}: NO SAMPLES")
            continue
        ws = min(s for s, _ in data)
        wh = max(h for _, h in data)
        note = ''
        if ws < MIN_SATS:
            note += f'  <-- below the {MIN_SATS}-sat rule'
        if wh > MAX_HDOP:
            note += f'  <-- above the HDOP {MAX_HDOP} rule'
        print(f"  {label}: worst {ws} sats, worst HDOP {wh:.2f}{note}")
        if note and 'arming' in label:
            failed.append('GPS quality at arming')
    if not gps_pre and not gps_fly:
        print("  NO GPS DATA AT ALL in this log: GPS-dependent modes cannot "
              "be cleared from it.")

    # --- compass field strength ----------------------------------------------
    # Finding 8 of the 2026-08-17 scan: the farm logs read 1038 and 1058
    # against a stated max of 875 and the CAUSE was never established. The
    # before/after-throttle split below is what separates the two candidates,
    # because a recalibration is wasted effort against a current-driven field.
    print(f"\nCOMPASS    |B| in mG, gate {MAG_FIELD_GATE:.0f} (this site's "
          f"figure, from the aircraft's own arming message)")
    if not mag_pre and not mag_fly:
        print("  NO MAG messages in this log, so the compass cannot be judged "
              "from it. LOG_BITMASK's COMPASS bit is the thing to check.")
    for imu in sorted(set(mag_pre) | set(mag_fly)):
        for label, data in (('at rest ', mag_pre.get(imu, [])),
                            ('flying  ', mag_fly.get(imu, []))):
            if not data:
                print(f"  MAG{imu} {label}: no samples")
                continue
            med = statistics.median(data)
            flag = '' if med <= MAG_FIELD_GATE else '   <-- OVER THE GATE'
            print(f"  MAG{imu} {label}: median {med:6.0f}  95th "
                  f"{pct(data, 95):6.0f}  max {max(data):6.0f}{flag}")

    rest_bad = [i for i, d in mag_pre.items()
                if d and statistics.median(d) > MAG_FIELD_GATE]
    fly_only_bad = [i for i, d in mag_fly.items()
                    if d and statistics.median(d) > MAG_FIELD_GATE
                    and i not in rest_bad]
    if rest_bad:
        print(f"  MAG{','.join(map(str, rest_bad))} is already over the gate "
              f"with the motors stopped, so this is calibration or something "
              f"magnetic parked near the compass, not motor current. "
              f"Recalibrate outdoors, away from metal, and re-read this line.")
        failed.append('compass field at rest')
    if fly_only_bad:
        print(f"  MAG{','.join(map(str, fly_only_bad))} is fine at rest and "
              f"over the gate under throttle: that is current in wiring near "
              f"the compass, and no amount of recalibration fixes it. Route "
              f"the battery leads away from the compass instead.")
        failed.append('compass field under throttle')

    # --- hover throttle, learned ---------------------------------------------
    if hover:
        med_h, fin_h = statistics.median(hover), hover[-1]
        print(f"\nHOVER      learned CTUN.ThH median {med_h:.3f}, "
              f"final {fin_h:.3f}")
        if fin_h > MAX_HOVER_THR:
            print(f"  Above {MAX_HOVER_THR}: thrust margin is thin, trim "
                  f"payload.")
            failed.append('hover throttle')

    # --- battery -------------------------------------------------------------
    # Resolve each parameter to the value in force when the motors started, and
    # keep anything written later aside so it can be called out rather than
    # silently believed. With no flight in the log, the last value is all there
    # is and is the honest answer.
    batt_cal, batt_late = {}, {}
    for pt, name, val in batt_parms:
        if first_fly_t is None or pt <= first_fly_t:
            batt_cal[name] = val
        elif batt_cal.get(name) != val:
            batt_late[name] = val

    if volt_min is not None or consumed is not None:
        print("\nBATTERY")
        if volt_min is not None:
            print(f"  lowest in-flight {volt_min:.2f} V "
                  f"(BATT_LOW_VOLT 10.8 / CRT 10.2)")
            if volt_min == 0.0:
                print("  0.00 V means the monitor is unconfigured or its "
                      "sense line is dead, not that the pack is flat.")
        if peak_amp:
            print(f"  peak current {peak_amp:.0f} A")
        if batt_cal:
            print("  monitor calibration AS FLOWN: "
                  + ", ".join(f"{k}={v:g}" for k, v in sorted(batt_cal.items())))
        if batt_late:
            print("  CHANGED AFTER THE MOTORS STOPPED, so it did NOT affect "
                  "this flight's amps: "
                  + ", ".join(f"{k}->{v:g}" for k, v in sorted(batt_late.items()))
                  + ". The board carries these values NOW; the line above is "
                    "what produced the numbers below.")
            # NO CEILING CHECK HERE ANY MORE. A previous version computed a
            # "maximum readable current" as 3.3 V x BATT_AMP_PERVLT and called
            # anything above it IMPOSSIBLE. Log 39 flew with PERVLT=17 and
            # logged 143 A, which that rule declared impossible while the
            # aircraft was demonstrably in the air, so the 3.3 V assumption was
            # simply wrong and the check was a false alarm. Guessing at the
            # board's analog scaling is how that happened; this file will not
            # do it again without a datasheet.
            #
            # This check needs no hardware assumption at all: an aircraft
            # cannot draw more out of a pack than the pack holds. When counted
            # consumption passes BATT_CAPACITY the remaining-percentage hits
            # zero and ArduPilot fires the battery failsafe on a number rather
            # than on the cells, which is exactly how a flight gets cut short
            # with a healthy pack still aboard.
            cap = batt_cal.get('BATT_CAPACITY')
            if cap and consumed and consumed > cap:
                print(f"  PHANTOM CAPACITY CUTOFF: counted {consumed:.0f} mAh "
                      f"out of a {cap:.0f} mAh pack. Nothing can discharge a "
                      f"pack past its own capacity, so the current scaling is "
                      f"wrong and any battery failsafe in this log fired on "
                      f"arithmetic, not on the cells. Measure the pack's "
                      f"resting cell voltages: that is the real state of "
                      f"charge.")
        if flight_s:
            print(f"  motors running for {flight_s:.0f}s total across this "
                  f"log ({flight_s / 60:.1f} min)")
        if consumed:
            print(f"  consumed {consumed:.0f} mAh"
                  + (f" ({energy:.1f} Wh)" if energy else ""))
            if flight_s > 30:
                rate = consumed / (flight_s / 60.0)
                print(f"  burn rate {rate:.0f} mAh/min -> a full 8000 mAh "
                      f"pack lasts about {8000 / rate:.1f} min at this load, "
                      f"or {8000 * 0.8 / rate:.1f} min to a sensible 20% "
                      f"reserve")
            print(f"  CROSS-CHECK the current sensor: fly ONE pack from full, "
                  f"then compare what the charger puts back in against the "
                  f"consumed figure above. Note CurrTot accumulates over the "
                  f"WHOLE log, so a log with many arm/disarm cycles is not "
                  f"one flight's worth.")

    # --- errors and messages --------------------------------------------------
    print(f"\nERRORS     {len(errs)} ERR events")
    for sub, code in errs[:25]:
        name = ERR_SUBSYS.get(sub, f'subsys {sub}')
        fatal = '  <-- SAFETY CRITICAL' if sub in ERR_FATAL else ''
        print(f"  {name} (subsys {sub}) code {code}{fatal}")
    if any(sub in ERR_FATAL for sub, _ in errs):
        failed.append('safety-critical ERR event')

    interesting = [s for s in msgs if any(k in s for k in MSG_KEYS)]
    print(f"\nMESSAGES   {len(interesting)} of note, of {len(msgs)} total")
    for s in dict.fromkeys(interesting):
        print(f"  {s}")

    # --- verdict --------------------------------------------------------------
    if failed:
        print(f"\nVERDICT: FAIL ({', '.join(dict.fromkeys(failed))})")
        sys.exit(1)
    print("\nVERDICT: PASS, this log clears the gates it can judge")
    sys.exit(0)


if __name__ == '__main__':
    main()
