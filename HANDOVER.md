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

Continuing MonsoonReady. Last session ended Monday 2026-08-10, ~22:00, after
the second real flight and a full day of battery-monitor debugging.

THE CURRENT-SENSOR THREAD IS CLOSED. Do not reopen it. BATT_AMP_PERVLT was
90.6866, which was the power module's "90 A" rating typed into QGC's
amps-per-volt field. The charger returned 2279 mAh against 8376 counted, a
3.68x over-count. Corrected to 24, verified by readback, and BATT_MONITOR
restored to 4 (it had been left at 0 = no sensing) with a reboot done.

WHAT THAT BOUGHT, all measured rather than modelled:
- Real hover current 28.8 A, real endurance 16.7 min on a full 8000 mAh pack,
  13.3 min to a 20% reserve. The 4.5 min "endurance" was the phantom mAh
  counter hitting BATT_CAPACITY, not the pack.
- The airframe is healthy: log 39 PASSES, VibeZ median 7.4, ZERO clipping,
  hover throttle 0.37-0.40, cells finished 3.95/3.94/3.97 balanced.

ONE VERIFICATION LEFT ON IT, ask me for the result before trusting the fix:
`./python tools/bench.py battery` with the pack connected and PROPS OFF.
Reading a parameter proves it is stored, not that the driver is running.
Expect ~pack voltage and under ~1 A; tens of amps at idle would mean a
BATT_AMP_OFFSET problem on top of the scale error already fixed.

OPEN WORK, none of it started, roughly in priority order:
- The competition video: 5-10 min, ONE continuous unedited take, opens with a
  Google search for the date on screen. PROJECT_STATE records a footage freeze
  around 2026-08-10, which has now PASSED. Resolve that date conflict before
  planning anything else.
- Servo gate still opens the wrong way. The fix is mechanical, not a number:
  reseat the horn 60 deg on the spline THEN swap the pulses in
  uno_q/dropper.py. Swapping pulses alone moves where the gate RESTS and can
  leave it open over a loaded hopper.
- Hopper flow rate never measured: `./python tools/flow_test.py`. Until it is,
  the variable-dose seconds are proportional to nothing.
- Camera FOV never measured (TODO 11).
- Phone hotspot to the UNO Q never set up, so no in-flight detection recording.
- UNO Q <-> Pixhawk D0/D1 link unproven, which is what blocks full autonomy.

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
