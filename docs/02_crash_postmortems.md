# Crash Post-Mortems → S550 Airframe

Three crashes on the S550 that preceded the current F550. Each is recorded with
cause, evidence and the standing rule it produced. **Two of the three share a
root cause**, which is the most useful line in this document.

Sources: `Build Log.txt` (user-maintained) and the crash-lesson entries in
`PROJECT_STATE.md`. Figures marked approximate are approximate in the source.

---

## 1. Summary

| # | Trigger | Root cause | Damage | Rule produced |
|---|---------|------------|--------|---------------|
| 1 | Prop nut backed off | Nut handedness, no thread lock | Prop departed, aircraft tumbled | Handed nut caps + Loctite; prop check is preflight |
| 2 | AltHold in wind | Wrong mode for conditions | Drifted ~44 m into a tree, one arm snapped | Loiter by default; GPS-health gate before GPS modes |
| 3 | Vibration → false descent | Vibration corrupts the EKF altitude estimate | Climbed to ~47 m, disarmed deliberately; centre plate destroyed | **Vibration gate: median VibeZ < 15**; abort = mode change, not disarm |

---

## 2. Crash 1 → prop nut backed off

**Sequence.** A propeller nut unthreaded in flight. The resulting imbalance
produced heavy vibration, the altitude estimate degraded, the aircraft climbed
to approximately 12 m, and the propeller then departed the motor entirely and
the aircraft tumbled.

**Root cause.** Propeller nut **handedness**. On a multirotor half the motors
turn each way, so a nut correct on one motor progressively loosens on its
counter-rotating neighbour. Nothing held it once it began to walk.

**Contributing factor.** No thread-locking compound.

**Fixes applied.**

| Fix | Detail |
|-----|--------|
| Handedness verified | F550 uses handed nut caps: **black CW, silver CCW** |
| Thread lock | Blue Loctite on prop nuts |
| Procedure | Prop and nut condition is a standing preflight item |

**The lesson that outlived the fix.** The failure was mechanical, but the
aircraft became unflyable when the **altitude estimate** went bad, before the
propeller physically left. Vibration corrupts state estimation, and a drone
with a corrupted altitude estimate will fight the pilot. This recurs as crash 3.

---

## 3. Crash 2 → AltHold in wind

**Sequence.** Flown manually in AltHold. AltHold holds height but does nothing
about horizontal position. Wind carried the aircraft approximately 44 m into a
tree, snapping one arm.

**Root cause.** Pilot mode selection. Loiter, which holds position on GPS,
would have held station against the wind.

**Fixes applied.**

| Fix | Detail |
|-----|--------|
| Default mode | Loiter for anything other than deliberate manual practice |
| Switch layout | 3-position mode switch = Stabilize / AltHold / Loiter, so the useful modes need no menu diving |
| GPS gate | GPS modes only with **10+ satellites, HDOP < 1.5, no EKF complaints**, after a 2 to 5 minute settle |

**Note.** This was not a technical failure. It was a decision made in the air by
a pilot who did not yet have the habit. The fix is procedural, and procedures
only count when they are written before the flight.

---

## 4. Crash 3 → vibration, phantom fall, deliberate disarm

**Sequence.** Motor vibration corrupted the altitude estimate. The aircraft
concluded it was falling and climbed to counteract a descent that was not
happening, reaching approximately 47 m. Wind was carrying it toward
unrecoverable ground, and the pilot disarmed it in the air.

**Root cause.** **Vibration feeding the EKF.** The same mechanism as crash 1,
without a mechanical trigger: the airframe itself vibrated enough to poison the
state estimate.

**Contributing factors.**

- Armed within seconds of power-on, at **HDOP 65 to 99**, meaning the GPS
  solution was effectively meaningless at the moment of arming. The GPS
  hardware was healthy; it had not been given time to converge. Procedural, not
  a hardware fault.
- No vibration gate existed before flight.

**On the disarm.** The aircraft was drifting toward terrain from which it could
not be retrieved, and a powered aircraft arriving there uncontrolled was the
worse outcome. It is recorded as a deliberate act because describing it as a
loss of control would hide the reasoning.

### 4.1 Post-crash bench tests

Surface damage: S550 centre plate torn apart; one ESC had its wires pulled out.

| Item | Result |
|------|--------|
| Battery | **PASS** — no puffing, dents, heat, or cell voltage divergence |
| Motors (all 6) | **PASS** — hand and powered spin, no grinding or roughness |
| ESCs | **PASS**, including the resoldered one |
| GPS | **PASS** — 10 satellites, HDOP < 1.0 within 30 s of power-on |
| Pixhawk | **PASS** — all features and ports |
| Power module | **FAIL** — likely shorted in the crash. Replaced. |
| Buzzer | **FAIL**. Replaced. |
| Telemetry radio | **PASS** |
| RC receiver | **PASS** |

The GPS row is worth reading twice: HDOP < 1.0 in 30 seconds on the bench
confirms that arming at HDOP 99 was purely a matter of not waiting.

### 4.2 Fixes applied

| Fix | Detail |
|-----|--------|
| **Vibration gate** | Median VibeZ **< 15** in hover before any altitude- or position-holding mode. A gate, not a guideline. |
| Abort procedure | Abort = **flip to Stabilize**, not disarm. In-flight disarm is reserved for imminent person-strike or unrecoverable flyaway. |
| GPS discipline | As crash 2 |
| Rebuild countermeasures | Quality props only; rigid motor mounts; wiring service loops; FC on foam near CG; baro covered; GPS cable tied; harmonic notch configured; compass recalibrated |

### 4.3 Current status

Removing the rubber motor dampeners improved median vibration from
approximately 30 to approximately **20.6**. The gate is **15**.

**Vibration is not solved.** It is the open blocker on the project and the item
most likely to constrain the demonstration.

---

## 5. The common thread

Two of three crashes trace to one chain:

```
vibration → corrupted altitude estimate → flight controller acts on it
          → violent, unwanted behaviour
```

The third was a mode choice that position hold would have covered.

This is why the mission software treats altitude with visible suspicion. The
descent logic requires the rangefinder and the EKF altitude to **agree** before
it will drop, and aborts upward when they do not
(`01_project_writeup.md` §6.2). That design is not generic defensive
programming. It is this aircraft's own history written into code.
