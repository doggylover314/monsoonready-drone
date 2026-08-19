#!/usr/bin/env python3
"""Name the dead ring channels, using only the Pixhawk link.

    ~/venv/bin/python tools/ring_channels.py            # 10 s sample
    ~/venv/bin/python tools/ring_channels.py --seconds 30

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
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mavlink_link import connect                                 # noqa: E402

NUM_RING = 6
INCREMENT_DEG = 60
NO_DATA = 65535
RANGE_MAX_CM = 200          # config.h; "clear" arrives as RANGE_MAX_CM + 1
UP_ORIENT = 24              # MAV_SENSOR_ROTATION_PITCH_90; 25 is the TF-Luna


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--conn', default=None)
    ap.add_argument('--baud', type=int, default=None)
    args = ap.parse_args()

    import time
    m, _, _ = connect(args.conn, args.baud)

    alive = defaultdict(int)     # channel -> samples that were not 65535
    total = 0
    near = {}                    # channel -> closest cm seen
    up_n = 0                     # DISTANCE_SENSOR orientation 24 seen
    up_cm = None                 # last upward reading
    print(f"\nlistening {args.seconds:g}s for OBSTACLE_DISTANCE ...")

    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        msg = m.recv_match(type=['OBSTACLE_DISTANCE', 'DISTANCE_SENSOR'],
                           blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'DISTANCE_SENSOR':
            if msg.orientation == UP_ORIENT:
                up_n += 1
                up_cm = msg.current_distance
            continue
        total += 1
        for ch in range(NUM_RING):
            cm = msg.distances[ch]
            if cm != NO_DATA:
                alive[ch] += 1
                if cm <= RANGE_MAX_CM:
                    near[ch] = min(near.get(ch, cm), cm)

    if not total:
        sys.exit("no OBSTACLE_DISTANCE at all: the ESP32 is silent, or its "
                 "TELEM2 link is down. This tool cannot see individual "
                 "channels until the ring is transmitting something.")

    print(f"{total} messages\n")
    print(f"{'ch':>2}  {'bearing':>7}  {'alive':>12}  what it saw")
    dead, flaky = [], []
    for ch in range(NUM_RING):
        n = alive[ch]
        pct = 100.0 * n / total
        bearing = ch * INCREMENT_DEG
        if n == 0:
            verdict, note = 'DEAD', 'never reported a reading'
            dead.append(ch)
        elif pct < 95:
            verdict, note = 'FLAKY', f'dropped out {total - n} of {total}'
            flaky.append(ch)
        else:
            verdict = 'alive'
            note = (f"closest {near[ch]} cm" if ch in near
                    else 'clear (nothing in range)')
        print(f"{ch:>2}  {bearing:>5} deg  {verdict:>12}  {note} ({pct:.0f}%)")

    if up_n:
        print(f"{6:>2}     up      {'alive':>12}  {up_cm} cm ({up_n} msgs)")
    else:
        print(f"{6:>2}     up      {'SILENT':>12}  no DISTANCE_SENSOR "
              f"orientation {UP_ORIENT} (this is RNGFND2 on the Pixhawk)")

    print()
    if dead:
        live = [c for c in range(NUM_RING) if alive[c]]
        print(f"DEAD: channel(s) {dead} at bearing(s) "
              f"{[c * INCREMENT_DEG for c in dead]} deg clockwise from the nose.")
        if live or up_n:
            # The live channels ARE the control experiment. They sit on the
            # same common 3V3/GND and the same mux as the dead ones, so if they
            # report, a sagging rail or a broken bus cannot be the explanation.
            # Saying so matters: this project has twice chased a shared-supply
            # theory that the evidence already ruled out.
            print(f"  Channels {live}{' plus the up sensor' if up_n else ''} "
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


if __name__ == '__main__':
    main()
