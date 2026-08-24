#!/usr/bin/env python3
"""Level-horizon (AHRS trim) calibration over the board's USB link.

    ~/python ~/monsoonready-drone/tools/level_cal.py

Why this exists (2026-08-22): QGC's bulk parameter download fails over the
degraded SiK radio, so the "Level Horizon" button in QGC never becomes
usable. But levelling is not a parameter download, it is a single command,
MAV_CMD_PREFLIGHT_CALIBRATION with param5=2, which ArduPilot answers by
computing the current tilt and folding it into AHRS_TRIM_X/Y (verified against
ArduPilot's GCS handler and MAVProxy's `ahrstrim`, which sends 0,0,0,0,2,0,0).
It travels over the board's reliable Pixhawk USB link, so the SiK problem
never touches it.

What it fixes: logs 47-50 (2026-08-21) showed the aircraft resting at a
standing -1 to -2 deg pitch in every session. The roll controller integrated
that error while the throttle sat part-way up on the ground, then applied the
whole wound-up correction the instant the aircraft got light, which is the
tilt Raghav felt on the attempted take-off. Levelling zeroes the mount's
share of that standing error; the rest is procedural (brisk continuous
throttle to lift-off, no dwelling at partial throttle).

The one rule that matters: the frame must be physically level and still
while this runs, checked with a bubble or phone level on the booms, not on
the table. The command bakes whatever tilt it sees straight into the trim,
so running it on a tilted frame makes the aircraft worse, not better. Run it
a single time; the before/after trim it prints is the proof it worked.

No reboot needed, AHRS_TRIM is used live. Disarmed only: ArduPilot refuses
calibration while armed, and this checks first so the refusal comes with a
clearer message.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pymavlink import mavutil                                    # noqa: E402
from mavlink_link import connect, send_and_ack, drain_statustext  # noqa: E402
from parameters import await_param                              # noqa: E402

TRIMS = ('AHRS_TRIM_X', 'AHRS_TRIM_Y')      # roll, pitch, in radians
CONSTRAIN_DEG = 10.0                          # ArduPilot's own trim ceiling


def read_trim(m, name):
    for _ in range(3):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        p = await_param(m, name, timeout=4.0)
        if p is not None:
            return p.param_value
    return None


def is_armed(m, timeout=3.0):
    """The autopilot's armed bit, or None if no autopilot heartbeat arrives.

    Filtered to component 1. The ESP32 obstacle module heartbeats as
    component 195 on the same bus and ArduPilot forwards it; an
    onboard-controller heartbeat never sets the armed bit, so accepting
    any heartbeat read "disarmed" on an armed aircraft. Same component-195
    race as the 2026-08-15 mode-map bug.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            continue
        if hb.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            continue
        return bool(hb.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


def show(label, x, y):
    def d(v):
        return "  ??  " if v is None else f"{math.degrees(v):+5.2f} deg"
    print(f"  {label:7}  roll(X) {d(x)}   pitch(Y) {d(y)}")


def main():
    m, _, _ = connect(None, None)

    armed = is_armed(m)
    if armed is None:
        sys.exit("no heartbeat, so arm state is unknown; not calibrating "
                 "blind. Check the link and retry.")
    if armed:
        sys.exit("the aircraft is ARMED. Disarm before levelling; a trim "
                 "written under motion is meaningless.")

    bx, by = read_trim(m, TRIMS[0]), read_trim(m, TRIMS[1])
    print("\ncurrent trim (what the mount offset looks like now):")
    show("before", bx, by)
    if bx is None or by is None:
        sys.exit("could not read the trim back, so I cannot prove a change. "
                 "Not sending the command blind.")

    print("\nFRAME must be LEVEL (bubble on the booms) and STILL. This writes "
          "whatever tilt it sees into the trim.")
    try:
        if input("type 'level' to confirm and calibrate: ").strip() != 'level':
            sys.exit("not confirmed; nothing sent.")
    except EOFError:
        sys.exit("\nno console input; run this in a terminal.")

    r = send_and_ack(m, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                     0, 0, 0, 0, 2, 0, 0, timeout=10.0)
    for t in drain_statustext(m):
        print(f"    FC: {t}")
    if r != 'MAV_RESULT_ACCEPTED':
        sys.exit(f"calibration was not accepted: {r}. Nothing changed.")

    ax, ay = read_trim(m, TRIMS[0]), read_trim(m, TRIMS[1])
    print("\ntrim after levelling:")
    show("after", ax, ay)

    if ax is None or ay is None:
        sys.exit("command was accepted but the read-back failed; verify "
                 "AHRS_TRIM_X/Y with parameters.py get before trusting it.")

    moved = abs(ax - bx) > math.radians(0.05) or abs(ay - by) > math.radians(0.05)
    if not moved:
        print("\nTrim did not move. Either it was already level (fine, you are "
              "done) or the command did not take. If the aircraft rested tilted "
              "in the logs, expect a change; if you see none, tell me.")
    else:
        print(f"\nDone. Trim moved by roll {math.degrees(ax - bx):+.2f} deg, "
              f"pitch {math.degrees(ay - by):+.2f} deg. No reboot needed.")
    if max(abs(math.degrees(ax)), abs(math.degrees(ay))) > CONSTRAIN_DEG - 0.1:
        print(f"WARNING: a trim near {CONSTRAIN_DEG} deg means the frame was "
              f"far from level when you ran this, or the FC is mounted badly. "
              f"Re-level the frame and run once more; do not fly on this.")


if __name__ == '__main__':
    main()
