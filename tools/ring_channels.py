#!/usr/bin/env python3
"""Name the dead ring channels, and bench-check one sensor at a time.

    ~/venv/bin/python tools/ring_channels.py                  # 10 s survey
    ~/venv/bin/python tools/ring_channels.py --seconds 30
    ~/venv/bin/python tools/ring_channels.py --sensor 3 --truth 50
    ~/venv/bin/python tools/ring_channels.py --sensor all --truth 50

TWO MODES, because "is the ring wired up" and "is this sensor any good" are
different questions:

  SURVEY (default) answers WHICH CHANNELS ARE ALIVE, in one pass, with nothing
  set up in front of the drone. It needs no ground truth and no hands.

  --sensor answers IS THIS SENSOR RELIABLE AND ACCURATE. You put a target at a
  distance you have measured with a tape, and it reports how often the sensor
  actually saw it, how much the reading jittered, and how far it sat from the
  tape. A survey cannot answer that, because a survey has nothing to compare
  against: a sensor stuck at a plausible-looking 80 cm passes a survey.

WHY THIS EXISTS: "prx ring 4/6" says how many channels are alive but not
WHICH, and test_everything's "bearing bins that see an object" omits a healthy
channel that simply has nothing in front of it. The only thing that names them
is the ESP32's own debug log over USB, and the ESP32's USB is not always
reachable. It does not need to be: the answer is already on the wire.

HOW IT TELLS DEAD FROM CLEAR (config.h, this build):
  * a channel whose sensor failed reports SENSOR_MM_ERROR -> SECTOR_NO_DATA,
    which goes out as 65535 = "unknown". ArduPilot treats unknown as NO
    INFORMATION, never as clear, which is why an absent sector is safe but
    also why it is invisible in any "what can we see" summary.
  * a channel that is alive with nothing in range sends RANGE_MAX_CM + 1,
    a real number. Alive and clear is therefore distinguishable from dead.
  * a channel with an object sends its distance in cm.

CHANNEL 6, the UPWARD sensor, is not in OBSTACLE_DISTANCE at all: it travels
as its own DISTANCE_SENSOR with orientation 24 (up), which the Pixhawk sees as
RNGFND2. It is reported here anyway, because "which channels are alive" is the
question and ch6 is a channel. Orientation 25 on the same message is the
downward TF-Luna and is deliberately ignored.

So: 65535 = DEAD, anything else = ALIVE. That is the whole trick.

Sampling over several seconds rather than reading one message is deliberate:
the fault this chases is intermittent (loose contact / sagging 3V3 per
config.h), so a channel that flickers matters as much as one that is flat
dead, and only a rate can show that.

Read-only: it sends nothing to the aircraft and never arms anything.
"""

import argparse
import os
import statistics
import sys
import time
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mavlink_link import connect                                 # noqa: E402

NUM_RING = 6
UP_CH = 6                   # not a real OBSTACLE_DISTANCE slot; see docstring
INCREMENT_DEG = 60
NO_DATA = 65535
RANGE_MAX_CM = 200          # config.h; "clear" arrives as RANGE_MAX_CM + 1
UP_ORIENT = 24              # MAV_SENSOR_ROTATION_PITCH_90; 25 is the TF-Luna
FIRMWARE_HZ = 10.0          # config.h TX_RATE_HZ, this build

# PASS/FAIL GATES for --sensor. These are OUR numbers for a 2 m ring on an
# aircraft flown at <= 2 m/s, NOT VL53L0X datasheet figures, and they are not
# claimed to be: if a number ever has to be defended, read the datasheet.
# The reasoning: at 2 m/s the aircraft covers 20 cm per 100 ms sample, so a
# sensor that drops 5% of samples still gives roughly a reading every 20 cm,
# and jitter under 5 cm is small against the distances avoidance acts on.
DROPOUT_GATE_PCT = 5.0
NOISE_GATE_CM = 5.0
BIAS_GATE_CM = 5.0          # absolute floor, so a 20 cm target is not judged
BIAS_GATE_FRAC = 0.10       # by percentage alone
DETECT_GATE_PCT = 90.0      # with a target in front, this fraction must SEE it


def collect(m, seconds, show=True):
    """Listen for `seconds` and return (elapsed, msg count, raw, up).

    `raw[ch]` is every value that channel sent, in order, sentinels included,
    because the ORDER is what separates a sensor that flickers evenly from one
    that vanishes for a second at a time.
    """
    raw = {ch: [] for ch in range(NUM_RING)}
    up = []
    total = 0
    if show:
        print(f"\nlistening {seconds:g}s for OBSTACLE_DISTANCE ...")
    t0 = time.monotonic()
    deadline = t0 + seconds
    while time.monotonic() < deadline:
        msg = m.recv_match(type=['OBSTACLE_DISTANCE', 'DISTANCE_SENSOR'],
                           blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'DISTANCE_SENSOR':
            if msg.orientation == UP_ORIENT:
                up.append(msg.current_distance)
            continue
        total += 1
        for ch in range(NUM_RING):
            raw[ch].append(msg.distances[ch])
    return time.monotonic() - t0, total, raw, up


def stats(vals, elapsed, sentinels=True):
    """Everything derivable from one channel's raw sample list.

    `sentinels=False` for the up sensor, whose DISTANCE_SENSOR encoding of a
    failed reading this tool has NOT verified, so nothing is assumed about it.
    """
    reported = [v for v in vals if v != NO_DATA] if sentinels else list(vals)
    ranged = [v for v in reported if v <= RANGE_MAX_CM] if sentinels else reported
    gap = run = 0
    for v in vals:
        run = run + 1 if (sentinels and v == NO_DATA) else 0
        gap = max(gap, run)
    d = {'n': len(vals), 'reported': len(reported), 'ranged': ranged,
         'clear': len(reported) - len(ranged), 'gap': gap,
         'hz': len(reported) / elapsed if elapsed else 0.0}
    if ranged:
        d['lo'], d['hi'] = min(ranged), max(ranged)
        d['mean'] = statistics.fmean(ranged)
        d['median'] = statistics.median(ranged)
        d['sd'] = statistics.pstdev(ranged) if len(ranged) > 1 else 0.0
    return d


def verdict(d, truth):
    """One word plus the reason it was chosen. Order matters: a channel that
    is not reporting cannot also be judged noisy, and a sensor that is not
    seeing the target cannot be judged on the accuracy of readings it did
    not take."""
    if not d['n']:
        return 'NO DATA', 'no messages arrived at all'
    if not d['reported']:
        return 'DEAD', 'never reported a reading'
    drop = 100.0 * (d['n'] - d['reported']) / d['n']
    if drop > DROPOUT_GATE_PCT:
        return 'FLAKY', (f"dropped {drop:.1f}% of samples "
                         f"(gate {DROPOUT_GATE_PCT:g}%), longest gap {d['gap']}")
    if truth is not None:
        seen = 100.0 * len(d['ranged']) / d['reported']
        if seen < DETECT_GATE_PCT:
            return 'MISSING TARGET', (
                f"reported 'clear' on {d['clear']} of {d['reported']} samples "
                f"with a target at {truth:g} cm: it is alive but not seeing it")
    if not d['ranged']:
        return 'clear', 'alive, nothing within range the whole run'
    if d['sd'] > NOISE_GATE_CM:
        return 'NOISY', (f"jitter {d['sd']:.1f} cm (gate {NOISE_GATE_CM:g} cm) "
                         f"on a target that was not moving")
    if truth is not None:
        bias = d['mean'] - truth
        gate = max(BIAS_GATE_CM, BIAS_GATE_FRAC * truth)
        if abs(bias) > gate:
            return 'BIASED', (f"reads {bias:+.1f} cm off the tape "
                              f"(gate {gate:.1f} cm)")
    return 'ok', ''


def report(label, d, truth, verb):
    v, why = verb
    print(f"\n{label}")
    if d['n']:
        drop = 100.0 * (d['n'] - d['reported']) / d['n']
        print(f"  reported   {d['reported']} of {d['n']} samples "
              f"({100 - drop:.1f}%), {d['hz']:.1f} Hz, longest gap "
              f"{d['gap']} sample(s)")
        if d['clear']:
            print(f"  in range   {len(d['ranged'])}, "
                  f"'clear' (nothing within {RANGE_MAX_CM} cm) {d['clear']}")
    if d['ranged']:
        print(f"  distance   {d['lo']}-{d['hi']} cm, mean {d['mean']:.1f}, "
              f"median {d['median']:.0f}, jitter (sd) {d['sd']:.1f} cm")
        if truth is not None:
            bias = d['mean'] - truth
            worst = max(abs(x - truth) for x in d['ranged'])
            pct = 100.0 * bias / truth if truth else 0.0
            print(f"  vs tape    {truth:g} cm: bias {bias:+.1f} cm "
                  f"({pct:+.1f}%), worst single reading {worst:.0f} cm off")
    print(f"  VERDICT    {v}" + (f"  ({why})" if why else ""))


def sensor_mode(m, args):
    """Deep-check one sensor, or walk them all with a target you move."""
    if args.sensor == 'all':
        wanted = list(range(NUM_RING)) + [UP_CH]
    elif args.sensor == 'up':
        wanted = [UP_CH]
    else:
        wanted = [int(args.sensor)]

    if args.truth is None:
        print("\nNO --truth GIVEN. Reliability and jitter still mean something "
              "without it, accuracy does not: nothing here can tell a sensor "
              "reading 80 cm correctly from one stuck at 80 cm. Measure the "
              "target with a tape and pass --truth <cm> to get that.")

    walk = len(wanted) > 1
    for ch in wanted:
        if ch == UP_CH:
            label, where = 'up sensor (RNGFND2)', 'ABOVE the drone'
        else:
            label, where = (f"ch{ch}", f"in front of ch{ch}, "
                            f"{ch * INCREMENT_DEG} deg clockwise from the nose")
        if walk and not args.no_prompt:
            target = (f"{args.truth:g} cm" if args.truth is not None
                      else 'a measured distance')
            try:
                ans = input(f"\nPut the target {target} {where}, "
                            f"then Enter (s = skip, q = quit): ").strip().lower()
            except EOFError:
                ans = ''
            if ans.startswith('q'):
                break
            if ans.startswith('s'):
                continue

        elapsed, total, raw, up = collect(m, args.seconds)
        if not total and ch != UP_CH:
            print(silence_help())
            return 1
        if ch == UP_CH:
            # Its no-data encoding is unverified, so every value is taken at
            # face value and the caveat is printed rather than guessed around.
            d = stats(up, elapsed, sentinels=False)
            if not up:
                print(f"\n{label}\n  VERDICT    SILENT  (no DISTANCE_SENSOR "
                      f"orientation {UP_ORIENT} in {elapsed:.0f}s)")
                continue
            report(label, d, args.truth, verdict(d, args.truth))
            print("  NOTE       the up sensor's DISTANCE_SENSOR encoding of a "
                  "FAILED reading is not verified by this tool, so a stuck "
                  "value here is not distinguishable from a real one except "
                  "by moving the target and re-running.")
            continue

        link_hz = total / elapsed if elapsed else 0.0
        print(f"\nlink: {total} messages in {elapsed:.1f}s = {link_hz:.1f} Hz "
              f"(firmware sends {FIRMWARE_HZ:g} Hz; a lower number here is the "
              f"LINK losing packets, which hits every channel equally and is "
              f"not a sensor fault)")
        d = stats(raw[ch], elapsed)
        report(label + f" ({ch * INCREMENT_DEG} deg)", d, args.truth,
               verdict(d, args.truth))
        if args.truth is not None and d['ranged']:
            print("  CROSS-CHECK the wiring map while the target is there: "
                  "if a DIFFERENT channel is the one that moved, "
                  "SECTOR_FOR_CHANNEL in config.h is wrong, not the sensor.")
            others = [c for c in range(NUM_RING) if c != ch]
            moved = [c for c in others
                     if stats(raw[c], elapsed).get('ranged')]
            if moved:
                print(f"  channels also reading a distance right now: {moved} "
                      f"(expected only if something else is near them)")
    return 0


def silence_help():
    return (
        "no OBSTACLE_DISTANCE at all. The ESP32 sits on TELEM1 (SERIAL1), "
        "not TELEM2, which is the SiK radio. Three things can produce "
        "this silence and they are told apart in one step: run "
        "`tools/bench.py nodes --seconds 20`. Component 195 present = the "
        "ESP32 is alive and talking, so the fault is in what the Pixhawk "
        "does with its packets. Component 195 absent = the ESP32 is "
        "unpowered, not booted, or its TELEM1 wiring is disturbed, which "
        "is the same class of fault as every other silence this project "
        "has had.")


def survey(m, args):
    elapsed, total, raw, up = collect(m, args.seconds)
    alive, lo, hi, clear_n = defaultdict(int), {}, {}, defaultdict(int)
    # RANGE, not just the minimum (user, 2026-08-18): one number cannot tell a
    # sensor pinned at a fixed distance from one that is tracking the world.
    # 5-5 cm across a whole run is a stuck or blocked sensor; 39-125 cm is a
    # sensor doing its job. Clear readings (RANGE_MAX_CM + 1) are counted
    # separately so a real object never hides inside the "clear" sentinel.
    for ch in range(NUM_RING):
        d = stats(raw[ch], elapsed)
        alive[ch] = d['reported']
        clear_n[ch] = d['clear']
        if d['ranged']:
            lo[ch], hi[ch] = d['lo'], d['hi']

    if not total:
        sys.exit(silence_help())

    print(f"{total} messages\n")
    print("bearings below are what the FIRMWARE CLAIMS (config.h "
          "SECTOR_FOR_CHANNEL is the identity map, SENSOR_ANGLE_OFFSET_DEG 0), "
          "NOT something this tool can verify. To check it for real: put one "
          "object about 50 cm from ONE sensor, re-run, and see which channel "
          "moves. If the wrong one moves, the wiring order and the map "
          "disagree and SECTOR_FOR_CHANNEL is what to fix.\n")
    print(f"{'ch':>2}  {'bearing':>7}  {'alive':>12}  what it saw")
    dead, flaky = [], []
    for ch in range(NUM_RING):
        n = alive[ch]
        pct = 100.0 * n / total
        bearing = ch * INCREMENT_DEG
        if n == 0:
            verd, note = 'DEAD', 'never reported a reading'
            dead.append(ch)
        elif pct < 95:
            verd, note = 'FLAKY', f'dropped out {total - n} of {total}'
            flaky.append(ch)
        else:
            verd = 'alive'
            if ch in lo:
                span = hi[ch] - lo[ch]
                note = (f"{lo[ch]}-{hi[ch]} cm"
                        + (f", SPAN {span} cm" if span else ", NEVER CHANGED")
                        + (f", clear on {clear_n[ch]}" if clear_n[ch] else ""))
            else:
                note = 'clear throughout (nothing within range)'
        print(f"{ch:>2}  {bearing:>5} deg  {verd:>12}  {note} ({pct:.0f}%)")

    if up:
        print(f"{6:>2}     up      {'alive':>12}  {up[-1]} cm ({len(up)} msgs)")
    else:
        print(f"{6:>2}     up      {'SILENT':>12}  no DISTANCE_SENSOR "
              f"orientation {UP_ORIENT} (this is RNGFND2 on the Pixhawk)")

    print()
    if dead:
        live = [c for c in range(NUM_RING) if alive[c]]
        print(f"DEAD: channel(s) {dead} at bearing(s) "
              f"{[c * INCREMENT_DEG for c in dead]} deg clockwise from the nose.")
        if live or up:
            # The live channels ARE the control experiment. They sit on the
            # same common 3V3/GND and the same mux as the dead ones, so if they
            # report, a sagging rail or a broken bus cannot be the explanation.
            # Saying so matters: this project has twice chased a shared-supply
            # theory that the evidence already ruled out.
            print(f"  Channels {live}{' plus the up sensor' if up else ''} "
                  f"report on the SAME shared 3V3/GND and the same mux, so the "
                  f"rail and the bus are EXONERATED by this run. A fault that "
                  f"spares them is LOCAL to the dead channel: its own sensor, "
                  f"its own 4-wire bundle, or its own mux channel.")
            print("  DECISIVE TEST: move the dead channel's sensor to a mux "
                  "channel known to work, and re-run. Reports there = the "
                  "sensor is fine and the fault is that mux channel or its "
                  "wiring. Still dead = that sensor is bad, replace it.")
        else:
            print("  NOTHING is reporting, so this is not a per-channel fault: "
                  "suspect the ESP32, the shared 3V3/GND, or the mux itself "
                  "before touching any individual sensor.")
    if flaky:
        print(f"FLAKY: channel(s) {flaky} came and went, which is the same "
              f"loose-contact signature as DEAD but caught earlier.")
    if not dead and not flaky:
        print("All six channels reported throughout. Nothing to chase here.")
    print("\nTo judge ONE sensor rather than name the dead ones, put a target "
          "at a tape-measured distance and run "
          "`tools/ring_channels.py --sensor <n> --truth <cm>`.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--sensor', default=None,
                    help="0-5, 'up', or 'all': deep-check that sensor against "
                         "a target instead of surveying the whole ring")
    ap.add_argument('--truth', type=float, default=None,
                    help='tape-measured distance to the target, in cm')
    ap.add_argument('--no-prompt', action='store_true',
                    help='with --sensor all, do not wait between sensors')
    ap.add_argument('--conn', default=None)
    ap.add_argument('--baud', type=int, default=None)
    args = ap.parse_args()

    if args.sensor is not None and args.sensor not in ('all', 'up'):
        try:
            ch = int(args.sensor)
        except ValueError:
            sys.exit(f"--sensor takes 0-{NUM_RING - 1}, 'up', or 'all'")
        if not 0 <= ch < NUM_RING:
            sys.exit(f"--sensor takes 0-{NUM_RING - 1}, 'up', or 'all'")

    m, _, _ = connect(args.conn, args.baud)
    if args.sensor is None:
        return survey(m, args)
    return sensor_mode(m, args)


if __name__ == '__main__':
    sys.exit(main())
