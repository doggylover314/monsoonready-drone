# Handover prompt

Paste the block below into a fresh chat to resume. Everything under it is a
pointer, not a copy: **PROJECT_STATE.md is the source of truth** and this file
must never restate its detail, or the two will disagree and the wrong one will
be believed. Keep this short. If it grows past a screen, the surplus belongs in
PROJECT_STATE.md.

Rewrite the LIVE THREAD section whenever the live thread changes. A stale
handover is worse than none, because it reads as current.

---

```
Read CLAUDE.md and PROJECT_STATE.md completely before doing anything, and
PRIVATE.md if present. git pull --rebase first.

Continuing MonsoonReady. Deadline 2026-08-15.

THE BATTERY/CURRENT-SENSOR THREAD IS CLOSED AND FULLY VERIFIED. Do not reopen
it. BATT_AMP_PERVLT 90.6866 was the module's "90 A" rating typed into QGC's
amps-per-volt field; the charger returned 2279 mAh against 8376 counted, a
3.68x over-count; corrected to 24, BATT_MONITOR back to 4, rebooted, and the
bench probe then read 12.62 V / 0.14 A at idle (props off), which also clears
BATT_AMP_OFFSET. Measured endurance 16.7 min on a full pack, 13.3 min to a 20%
reserve, hover 28.8 A. Airframe healthy: log 39 PASSES, VibeZ median 7.4, zero
clipping.

THE FOOTAGE-FREEZE DATE CONFLICT IS ALSO RESOLVED (2026-08-11): the "~Aug 10
freeze" was a self-imposed target, not the competition date, and is replaced by
"the last flyable window before Aug 15". By this project's own pre-committed
cutoff, the video is PLAN B: piloted flight plus a ground AI demo, because the
UNO Q<->Pixhawk link was not proven by the recorded Friday deadline.

OPEN WORK, none of it started, in the order that serves the video:
- Shoot the video: 5-10 min, ONE continuous unedited take, opens with a Google
  search for the date on screen (so the shoot needs internet), covers
  functionality + assembly + components, Drive link set to anyone-with-the-link.
- Servo gate still opens the wrong way. The fix is mechanical, not a number:
  reseat the horn 60 deg on the spline THEN swap the pulses in
  uno_q/dropper.py. Swapping pulses alone moves where the gate RESTS and can
  leave it open over a loaded hopper.
- Hopper flow rate never measured: `./python tools/flow_test.py`. Until it is,
  the variable-dose seconds are proportional to nothing.
- Camera FOV never measured (TODO 1).
- Phone hotspot to the UNO Q never set up, so no in-flight detection recording.
- UNO Q <-> Pixhawk D0/D1 link unproven (TODO 7); it blocks full autonomy and
  is out of scope for this video under the Plan B call above.

Ask before assuming anything else is still true.
```

---

## Why a fresh chat, and when

Long sessions get summarized, and a summary loses exactly the details that
matter here: which log, which parameter value, which claim was retracted. This
session retracted two of its own conclusions (a guessed 3.3 V ADC ceiling, and
an ESC current-rating argument). Those retractions are recorded in
PROJECT_STATE.md, so a fresh chat that reads the state file starts more
accurate than a long one running on a lossy summary.

Move when the current thread closes, not mid-diagnosis.
