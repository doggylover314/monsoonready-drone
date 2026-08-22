# Handover prompt

Paste the block below into a fresh chat to resume. Everything under it is a
pointer, not a copy. PROJECT_STATE.md is the source of truth and this file must
never restate its detail, or the two will disagree and the wrong one will be
believed. Keep it short. If it grows past a screen, the surplus belongs in
PROJECT_STATE.md.

Rewrite the live thread below whenever the live thread changes. A stale handover
is worse than none, because it reads as current.

---

```
Read CLAUDE.md and PROJECT_STATE.md completely before doing anything, and
PRIVATE.md if present. Pull first (merge, never rebase).

Continuing MonsoonReady. SUBMISSION IS 2026-08-23 (user, primary source).
2026-08-15 was the shoot day, not the upload day.

CLOSED THREADS, do not reopen:
- Battery and current sensing. BATT_AMP_PERVLT 24, BATT_MONITOR 4, verified on
  the bench at 12.62 V / 0.14 A idle. Endurance 16.7 min full pack, 13.3 min to
  a 20% reserve.
- Vibration. It was the historic blocker and it is clear: recent logs read
  VibeZ median 7.4 to 9.0 against a gate of 15, zero clipping.
- The UNO Q to Pixhawk link. It runs over USB now, not D0/D1.
- Camera FOV. Measured 56.18 deg, baked into camera_geom as the default.
- The arming refusals. Root cause was two dead-but-fitted ring channels making
  the ESP32 block for a second at a time on retries, which stalled the MAVLink
  stream past ArduPilot's 500 ms proximity timeout. Unfitted in firmware, and
  ch6's separate fault was a high-resistance power joint, resoldered.

WHERE IT STANDS: the full autonomous loop has never flown. Two field days
failed, 2026-08-15 at the farm (camera stopped enumerating on the board) and
2026-08-21 (never got past prearm: HDOP, fence, proximity). Filming moved to the
field near the house.

OPEN:
- The flow rate is measured for mustard seeds but only from an under-filled
  hopper, about 4.2 to 5.3 g/s from the shortest dwells. A full-hopper run has
  never been taken and no dose figure should be quoted until it is.
- Dataset URLs and BibTeX for docs/03. Reyansh has the Universe pages.
- Compliance verification against current DGCA text, plus the Bti registration
  check. Raghav.
- All evidence capture, and the video.

Ask before assuming anything else is still true.
```

---

## Why a fresh chat, and when

Long sessions get summarized, and a summary loses exactly the details that
matter here: which log, which parameter value, which claim was retracted. This
project has had several conclusions retracted by its own assistant, including a
guessed ADC ceiling, an ESC current-rating argument, and a hopper slug that
turned out to be a starving hopper. Those retractions live in PROJECT_STATE.md,
so a fresh chat that reads the state file starts more accurate than a long one
running on a lossy summary.

Move when the current thread closes, not mid-diagnosis.
