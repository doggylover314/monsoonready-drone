# Demo Video → Storyboard

Target length **~3 minutes**. Footage freeze **2026-08-10**, five days before
the deadline, so that editing never competes with flying.

The video is judged on presentation and is also the primary evidence for
functionality. Every shot therefore has a job: it proves one specific claim
made in `01_project_writeup.md`. Shots that look impressive but prove nothing
are marked cuttable.

---

## 1. Principles

| Principle | Reason |
|-----------|--------|
| **Show the onboard inference on screen** | A viewer cannot tell from outside whether detection happened on the UNO Q or on a laptop. The UNO Q's own view with the box drawn, and a visible inference time, is the only proof. |
| Real footage over animation | Diagrams only for what cannot be filmed, such as the serial architecture |
| Do not fake the loop | If automatic descent is not flight-ready, the descope version is shown and narrated as such. See §3. |
| Narration under 350 words | Far less than feels natural when writing to a 3-minute cut |

---

## 2. Shot list

| # | Time | Shot | Proves | Notes |
|---|------|------|--------|-------|
| 1 | 0:00–0:12 | Cold open: monsoon water standing on a rooftop or construction site, close, no narration | The problem is real and local | Shoot in actual conditions if the monsoon cooperates |
| 2 | 0:12–0:30 | Narration over the same footage: dengue, the larval stage, why ground crews miss sites | Problem framing | Three sentences |
| 3 | 0:30–0:45 | Aircraft on the ground, slow pan, then hopper and camera close-up | The hardware exists and is built | Granule tube and servo gate must be legible |
| 4 | 0:45–1:00 | Takeoff and survey pattern, wide | Aircraft flies | Ground shot, second camera if available |
| 5 | **1:00–1:25** | **UNO Q screen capture: still frame, bounding box, confidence, inference time in ms** | **Edge AI. The core claim.** | The most important 25 seconds in the video |
| 6 | 1:25–1:45 | Descent over target, from the ground, rangefinder height overlaid | Closed loop from detection to action | Height overlay reconstructed from the telemetry log in post |
| 7 | 1:45–1:55 | The drop, close-up if a second camera is available, granules landing in water | Mission completed | Salt, and the narration says so |
| 8 | 1:55–2:05 | Climb out, resume survey | A survey, not a stunt | |
| 9 | 2:05–2:20 | Landing, then the base-station page on a phone: heatmap, treated sites | Municipal work product | **Cuttable** if TODO 13 is unfinished |
| 10 | 2:20–2:40 | SITL dropout drill on screen, aborting upward, with one line about crash 3 | Engineering maturity | Differentiator: most demos hide their failures |
| 11 | 2:40–2:55 | Bench and build montage: training curves, obstacle ring, OLED status display | Depth of work | Fast cuts, music, no narration |
| 12 | 2:55–3:00 | Closing card: team names, AI-assistance line, dataset attribution | Honesty and CC BY 4.0 compliance | Required by licence |

---

## 3. Plan A and Plan B for shots 4 to 8

Plan B is the descope ladder rung 1 from `01_project_writeup.md` §8.

| Shot | Plan A (full auto) | Plan B (descope) |
|------|--------------------|------------------|
| 4 | Automatic survey mission in GUIDED | Manual Loiter flight over the survey area |
| 5 | **Identical** | **Identical** |
| 6 | Automatic descent commanded by the UNO Q | Pilot descends in Loiter; UNO Q confirms height and readiness |
| 7 | Drop commanded automatically at drop height | Drop commanded by the UNO Q on operator confirmation |
| 8 | Automatic climb and resume | Pilot climbs out |

Shot 5, the edge-AI claim, is unchanged in both plans. That is the point of the
descope: the detection and the drop decision remain onboard either way.

If Plan B is filmed, one plain narration sentence covers it, for example:

> "The detection and the drop decision run onboard. On this flight the aircraft
> is held in position by the pilot, because our vibration figures have not yet
> cleared the gate we set for automatic flight."

Three seconds, and it buys credibility for everything else in the video.

---

## 4. Capture procedure

| Item | Detail |
|------|--------|
| Cameras | Two minimum: one wide on a tripod, one handheld for close-ups |
| UNO Q screen capture | Running on **every** flight, not just the good one |
| Telemetry logging | On for every flight, so overlays can be reconstructed later |
| Drop shot | Shoot at least 4 takes. Hardest shot to get, easiest to miss. |
| Narration audio | Recorded indoors afterwards. Never use on-site audio for voice. |
| B-roll | More hardware footage than seems necessary; editing always wants more |
| Music | Never loud enough to drown the aircraft. The sound of a working hexacopter is evidence too. |

---

## 5. Cross-reference

Every claim the video makes is backed by an artefact in
`07_evidence_checklist.md`. A shot that cannot be matched to an evidence item
means either the claim is unsupported or the evidence list is incomplete.
