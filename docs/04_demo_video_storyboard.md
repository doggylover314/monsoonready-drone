# Demo video storyboard

Target length approximately 3 minutes. Footage freeze 2026-08-10, five days
before the 2026-08-15 deadline, so that editing is never competing with
flying.

The video is judged on presentation, but it is also the primary evidence for
functionality. Every shot below therefore has a job: it proves one specific
claim the write-up makes. Shots that look impressive but prove nothing are
noted as cuttable.

## Guiding decisions

**Show the onboard inference on screen.** The single most important thing to
communicate is that the model runs on the aircraft. A viewer cannot tell from
the outside whether detection happened on the UNO Q or on a laptop, so the
video has to show it: the UNO Q's own view with the bounding box drawn, and a
visible statement of the inference time.

**Real footage over animation.** Use a diagram only for things that cannot be
filmed, such as the serial architecture.

**Do not fake the loop.** If the fully automatic descent is not flight-ready,
show the descope version and say so in the narration. Judges see a great many
edited demos and the honest one is memorable. See the Plan B column.

## Shot list

| # | Time | Shot | Proves | Notes |
|---|---|---|---|---|
| 1 | 0:00-0:12 | Cold open: monsoon water standing on a rooftop or construction site, close, no narration | The problem is real and local | Shoot in actual conditions if the monsoon cooperates |
| 2 | 0:12-0:30 | Narration over the same footage: dengue, larval stage, why ground crews miss sites | Problem framing | Keep it to three sentences |
| 3 | 0:30-0:45 | The aircraft on the ground, slow pan, then the hopper and camera in close-up | The hardware exists and is built | Show the granule tube and servo gate clearly |
| 4 | 0:45-1:00 | Takeoff and survey pattern, wide | Aircraft flies | Ground shot plus, if available, a second camera |
| 5 | 1:00-1:25 | **Screen capture from the UNO Q**: still frame, bounding box, confidence, inference time in ms | **Edge AI, the core claim** | The most important 25 seconds in the video. Do not cut this short. |
| 6 | 1:25-1:45 | Descent over the target, from the ground, with the rangefinder height overlaid | Closed loop from detection to action | Height overlay can come from the telemetry log in post |
| 7 | 1:45-1:55 | The drop. Close-up if a second camera is available, plus granules landing in the water | The mission is completed | Salt, and the narration says so |
| 8 | 1:55-2:05 | Climb out and resume survey | The loop repeats, this is a survey not a stunt | |
| 9 | 2:05-2:20 | Landing, then the base-station page on a phone: heatmap, treated sites | Municipal work product | Cuttable if TODO 13 is not finished |
| 10 | 2:20-2:40 | Safety: the SITL dropout drill on screen, aborting upward, alongside one line about crash 3 | Engineering maturity | This shot is a differentiator. Most demos hide their failures. |
| 11 | 2:40-2:55 | Bench and build montage: training curves, the obstacle ring, the OLED status display | Depth of work | Fast cuts, music, no narration |
| 12 | 2:55-3:00 | Closing card: team names, AI-assistance disclosure line, dataset attribution | Honesty and licence compliance | Required by CC BY 4.0 |

## Plan A and Plan B for shots 4 to 8

| Shot | Plan A (full auto) | Plan B (descope, per the ladder) |
|---|---|---|
| 4 | Automatic survey mission in GUIDED | Manual Loiter flight over the survey area |
| 5 | Identical either way | Identical either way |
| 6 | Automatic descent commanded by the UNO Q | Pilot descends in Loiter, UNO Q confirms height and readiness |
| 7 | Drop commanded automatically at drop height | Drop commanded by the UNO Q on operator confirmation |
| 8 | Automatic climb and resume | Pilot climbs out |

If Plan B is what gets filmed, narration says so in one plain sentence, for
example: "The detection and the drop decision run onboard. On this flight the
aircraft is held in position by the pilot, because our vibration figures have
not yet cleared the gate we set for automatic flight." That sentence costs
three seconds and buys credibility for everything else in the video.

## Audio and narration

Write the script to the visuals, not the other way round. Total narration
should be around 300 to 350 words for three minutes, which is far less than
feels natural when writing. Cut ruthlessly.

Do not use music that drowns the aircraft. The sound of a hexacopter working
is evidence too.

## Capture checklist for shoot days

- Two cameras minimum, one wide on a tripod and one handheld for close-ups.
- Screen capture running on the UNO Q for every flight, not just the good one.
- Telemetry logging on for every flight, so overlays can be reconstructed later.
- Shoot the drop at least four times. It is the hardest shot to get right and
  the easiest to miss.
- Get clean audio of the narration indoors afterwards. Never use on-site audio
  for voice.
- Shoot more B-roll of the hardware than feels necessary. Editing always wants
  more.

## Cross-references

Every claim the video makes should be backed by something in
[07_evidence_checklist.md](07_evidence_checklist.md). If a shot cannot be
matched to an evidence item, either the claim is unsupported or the evidence
list is missing something.
