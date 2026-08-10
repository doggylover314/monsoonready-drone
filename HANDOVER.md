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

Continuing MonsoonReady. Last session ended Monday 2026-08-10, evening, right
after the second real flight.

LIVE THREAD: the battery current sensor reads roughly 3-4x high, and it is NOT
a calibration problem. See PROJECT_STATE.md, the 2026-08-10 LOG 39 entry, for
the evidence. One-line version: BATT_AMP_PERVLT changed 5.33x between logs 37
and 39 while the reported current changed 1.13x, so the reading does not
respond to the parameter and no value of it will fix this.

Consequences, all load-bearing:
- Consumed mAh, burn rate and every endurance figure from logs 37 and 39 are
  UNUSABLE. Real endurance is UNMEASURED. The 4.5 min flight was terminated by
  the phantom counter reaching BATT_CAPACITY 8000 (8376 counted), not by the
  pack, which finished at 3.95/3.94/3.97 V per cell.
- The airframe itself is FINE and this is the good news: log 39 is a PASS,
  VibeZ median 7.4, zero clipping, hover throttle 0.37-0.40.

NEXT ACTION, not yet done: zero-load current reading, props off, disarmed.
`./python tools/bench.py battery` - under ~1 A is healthy, tens of amps at zero
throttle means the sense path is faulty rather than miscalibrated.

ALSO AWAITING: the mAh the charger returned into the pack after that flight.
Expect 2000-2800 against 8376 counted. Ask me for it; it sizes the error.

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
