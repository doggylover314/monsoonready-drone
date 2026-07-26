# Crash post-mortems

Three crashes, on the S550 airframe that preceded the current F550. Each one is
written the way an incident report should be: what happened, what actually
caused it, how we know, and what changed as a result. Two of the three share a
root cause, which is the most useful thing in this document.

Source: `Build Log.txt` (maintained by Raghav) and the crash-lesson entries in
`PROJECT_STATE.md`. Where a number is approximate it is marked as such.

---

## Crash 1: prop nut backed off

**What happened.** A propeller nut unthreaded itself in flight. The resulting
imbalance produced heavy vibration, the altitude estimate degraded, the
aircraft climbed to approximately 12m, and then the propeller departed the
motor entirely and the aircraft tumbled.

**Root cause.** Propeller nut handedness. On a multirotor, half the motors turn
one way and half the other; a nut that is correct on one motor will progressively
loosen on its counter-rotating neighbour. The nut was not held by anything once
it began to walk.

**Contributing factor.** No thread-locking compound.

**What changed.**
- Handedness verified on every motor. The F550 uses handed nut caps, black for
  clockwise and silver for counter-clockwise.
- Blue Loctite on prop nuts.
- Prop and nut condition became a standing preflight check item.

**What this crash taught us that outlived the fix.** The failure was mechanical,
but the aircraft became unflyable at the moment the *altitude estimate* went
bad, before the propeller physically left. Vibration corrupts state estimation,
and a drone with a corrupted altitude estimate will fight you. That lesson
recurs below.

---

## Crash 2: AltHold in wind

**What happened.** Flown manually in AltHold. AltHold holds height but does
nothing about horizontal position. Wind carried the aircraft approximately 44m
into a tree. One arm snapped.

**Root cause.** Pilot mode selection. AltHold was the wrong mode for the
conditions; Loiter, which holds position using GPS, would have held station
against the wind.

**What changed.**
- Loiter is the default mode for anything other than deliberate manual
  practice, conditions permitting.
- The three-position mode switch is configured Stabilize / AltHold / Loiter so
  that the useful modes are reachable without menu diving.
- A hard rule that GPS-dependent modes are only entered with 10+ satellites,
  HDOP below 1.5, no EKF complaints, and a 2 to 5 minute settle after power-on.

**Honest note.** This one was not a technical failure. It was a decision made
in the air by a pilot who did not yet have the habit. The fix is procedure, and
procedures only count if they are written down before the flight, which is part
of why the preflight checklist exists at all.

---

## Crash 3: vibration, phantom fall, deliberate disarm

**What happened.** Motor vibration corrupted the altitude estimate. The aircraft
concluded it was falling and climbed to counteract a descent that was not
happening, reaching approximately 47m. Wind was carrying it toward an area
where it could not be recovered. The pilot disarmed it deliberately in the air.

**Root cause.** Vibration feeding the EKF. This is the same mechanism as crash
1 but without a mechanical trigger: the airframe itself was vibrating enough to
poison the state estimate.

**Contributing factors.**
- Armed within seconds of power-on. HDOP was 65 to 99, meaning the GPS
  solution was effectively meaningless at the moment of arming. The GPS
  hardware was healthy; it simply had not been given time to converge. This
  was procedural, not a hardware fault.
- No vibration gate existed before flight.

**The disarm was a decision, not a loss of control.** The aircraft was drifting
toward terrain from which it could not be retrieved, and a powered aircraft
arriving there uncontrolled was the worse outcome. We record this as a
deliberate act because pretending it was an accident would hide the reasoning.

**Damage.** S550 centre plate torn apart. One ESC had its wires pulled out.
Motors, GPS, Pixhawk, battery, RC receiver and telemetry radio all undamaged on
inspection. Post-crash bench tests, all recorded in `Build Log.txt`:

| Item | Result |
|---|---|
| Battery | PASS: no puffing, dents, heat, or cell voltage divergence |
| Motors (all 6) | PASS: hand spin and powered spin, no grinding or roughness |
| ESCs | PASS, including the resoldered one |
| GPS | PASS: 10 satellites and HDOP below 1.0 within 30 seconds of power-on |
| Pixhawk | PASS: all features and ports |
| Power module | **FAIL**, likely shorted in the crash. Replaced. |
| Buzzer | **FAIL**. Replaced. |
| Telemetry radio | PASS |
| RC receiver | PASS |

That the GPS reached HDOP below 1.0 in 30 seconds on the bench is worth
noting: it confirms the arming-with-HDOP-99 problem was purely a matter of not
waiting.

**What changed.**
- **A hard vibration gate.** Median VibeZ must be below 15 in hover before any
  altitude-holding or position-holding mode is used. Not a guideline; a gate.
- **Abort is a mode change, not a disarm.** Flipping to Stabilize is the
  response to a misbehaving aircraft. Disarming in flight is reserved for an
  imminent strike on a person or an unrecoverable flyaway.
- **GPS discipline** as listed under crash 2.
- Rebuild countermeasures against vibration: quality propellers only, rigidly
  mounted motors, service loops on wiring, flight controller on foam near the
  centre of gravity, barometer covered, GPS cable tied down, harmonic notch
  filter configured, compass recalibrated.

**Where this stands today, stated plainly.** Removing the rubber motor
dampeners improved median vibration from approximately 30 to approximately
20.6. The gate is 15. **Vibration is not solved.** It is the open blocker on
this project and the item most likely to constrain what we can demonstrate. We
would rather show a judge a flight that met the gate than a flight that
happened.

---

## What the three crashes have in common

Two of the three trace to the same chain: vibration corrupts the altitude
estimate, the altitude estimate drives the flight controller, the flight
controller does something violent and unwanted. The third was a mode choice
that position hold would have covered.

This is why the mission software treats altitude with visible suspicion. The
descent logic requires the rangefinder and the EKF altitude to agree before it
will drop, and aborts upward when they do not
([01_project_writeup.md](01_project_writeup.md), section 6). That design is not
defensive programming in the abstract. It is this aircraft's own history
written into code.
