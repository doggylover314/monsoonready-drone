# Demo video plan

Five to ten minutes, one continuous take, no cuts. It has to open with today's
date on a screen, show the components, show the assembly, and show the thing
working. Publicly viewable afterwards.

That last part changes the planning completely. No edit means no fixing it
afterwards, and it rules out a whole category of things people put in demo
videos: overlays, a music bed, montage, telemetry graphics reconstructed from
the log. Whatever happens in front of the phone is the video.

`VIDEO_SCRIPT.md` in the repository root holds the line-by-line script, the
pre-roll checklist and the salvage rules. This file is the shorter version: the
running order, and what each part is there to prove.

The first attempt was 2026-08-15 at the farm. The camera stopped enumerating on
the board and no flight was filmed. The second field day was 2026-08-21 and the
aircraft never got past prearm. Filming moved to the field near the house,
which is close enough to try again cheaply.

## Running order

| Minutes | What is happening | What it proves |
|--|--|--|
| 0:00 to 0:30 | Google search for today's date on a laptop screen, held long enough to read | The date requirement. Get it clean, it is a disqualification criterion. |
| 0:30 to 2:15 | Slow walk around the airframe, close on each part as it is named | The hardware exists and was built rather than bought assembled |
| 2:15 to 3:00 | The wiring bay, power split, hand-soldered sensor hub, the hopper | Assembly is the team's own work |
| 3:00 to 4:00 | The detector running on screen, box drawn, timing visible | Edge AI. This is the core claim and the most important minute in the take. |
| 4:00 to 6:30 | The flight | Functionality |
| 6:30 to 8:00 | The tray with granules in the water, then the dashboard log | The loop closed, and it was recorded |

## The flight, two versions

Plan A is the full autonomous loop. Start the mission, hands off the sticks
except as safety pilot, and let it survey, detect, descend, drop and come home.

Plan B is flown by Reyansh with the detector recording and the drop commanded
manually, and it opens with a plain sentence saying so. Something like: the
detection you just watched runs on the drone, the command link that closes the
loop is not flight-ready today, so this one is me flying.

Three seconds, said once, at the start.

The onboard detection is identical either way, which is the whole point of
having a Plan B at all. The model stays on the aircraft whether or not the
aircraft is flying itself.

## What gets dropped on camera

Mustard seed. Not larvicide, and the narration says so out loud rather than
leaving it to a caption nobody reads. Bti is a real pesticide and dispensing it
on a student flight test would turn the demonstration into a pesticide
application.

Fine salt was the original test material. It bridged in the hopper and stopped
flowing, so it is out.

## Practical notes

One phone, held landscape, do-not-disturb on. A call kills the take.

Rehearse the walk without recording. Takes fail on forgetting a line and on
walking into frame, not on flying.

Watch the whole take back before anything is packed up. Audio audible, date
legible, drop visible. Then copy it to a laptop on the spot.

Never fake a result. An overclaim a judge catches is fatal, and a limitation
stated plainly scores.

Every claim the video makes should have an artefact behind it in
`07_evidence_checklist.md`. If a shot cannot be matched to one, either the
claim is unsupported or the evidence list has a hole in it.
