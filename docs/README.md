# MonsoonReady Dragonfly

An F550 hexacopter that finds standing water after monsoon rain and drops
granular Bti larvicide into it. The YOLO model runs on an Arduino UNO Q bolted
to the aircraft, so it locates in the air. No ground station, no cloud.

Arduino Physical AI Challenge India 2026, built by Reyansh and Raghav. Test
flights drop mustard seed, never larvicide.

This is the whole judged documentation set. It used to be eight files and they
kept drifting apart, so they are one file now. Everything in it comes from
`PROJECT_STATE.md`, `Build Log.txt` and the source. Where a number exists but
nobody has measured it, the text says so and leaves it, rather than filling the
gap with something plausible. Firmware throughout is ArduCopter 4.7.0,
Pixhawk1-bdshot, flashed 2026-07-25. Work items live in `PROJECT_STATE.md`,
because a document that tracks its own TODO list goes stale in a week.

## Why granules

Aedes aegypti lays eggs in small pools: flat roofs, building sites, blocked
drains, tarpaulins, an old tank nobody emptied. Larvae are stuck in the water
until they hatch, which makes them the easy thing to kill. Council crews
already do this on foot, slowly, and only where they can walk.

Granular Bti has one requirement, which is landing in the water. No tank, no
pump, no nozzle, nothing to drift downwind. What is left is a much smaller
problem: find the water, get over it, drop a measured amount.

## The loop

While the Pixhawk flies the survey rows, the UNO Q takes downward photos and
runs them through the model. Water in frame means the coordinates get locked
right then, at altitude. Then it repositions beside the water, comes down on
the TF-Luna, crosses over the middle, opens the gate, crosses back. Climb, next
row. Once it lands, the UNO Q stops being a mission computer and becomes a web
dashboard showing where it flew and what it treated.

Survey altitude is the one number that changes everything and it took a flight
to learn that. The first demo video was flown at 15 m. At that height a
half-metre puddle is 44 pixels across and the model finds it about one frame in
ten, which is not a detector so much as a lottery. Survey altitude is now 5 m,
where the same puddle is 132 pixels and lands somewhere around half the frames
at a confidence floor of 0.25. Lower costs coverage per row, so row spacing
came down with it, and the aircraft now stops for a second at each waypoint
instead of shooting on the move.

It descends beside the puddle rather than over it because the TF-Luna uses 850
nm infrared, and still water at that wavelength acts like a mirror. Dry ground
three metres to the side gives an honest height, and the aircraft holds that
height across the water.

## What we claim

The model finds standing water. That is the whole claim.

It cannot tell you the water has been there long enough to breed anything. From
survey altitude, this morning's puddle and a two-week-old breeding site are the
same handful of pixels. Stagnation has to come from repeat visits: fly again on
another day, see which pools are still there, then have someone confirm on the
ground. One flight produces candidates and nothing more. That gap between what
the system does and what the concept promises is the honest limit of the
project, and closing it needs a survey run across several days rather than more
code.

## Hardware

| Part | Why this one |
|------|--------------|
| F550 hexacopter | Carries the payload and survives losing a motor. Replaced the S550 we destroyed. |
| Pixhawk 2.4.8, ArduCopter 4.7.0 | Mature guided-mode MAVLink, and logs good enough to work out what went wrong. |
| 6x A2212 920KV, 1045 props, 45A BLHeli_32 ESCs | Motors swapped from EMAX MT2213 when we could not get matching props in India. ESCs rated well above what the motors pull. |
| 3S 8000mAh LiPo | One pack. It lives on the drone. |
| Arduino UNO Q, 4GB | Runs the detector. The whole point of the project. |
| Logitech B525, 720p | Already owned, and UVC works on the UNO Q today. That beat any spec gain from buying something. |
| TF-Luna, downward | Native ArduPilot support. Gives height and puddle size for the dose. |
| VL53L0X ring on a TCA9548A, read by an ESP32 | Cheapest proximity ring the flight controller understands without a firmware fork. |
| MG90 metal-gear servo | Opens a gate on a tube. Metal gears because a stripped nylon gear means no drop. |
| SH1106 OLED, SiK 433 MHz, FlySky FS-i6X | Prearm status without a laptop, ground monitoring, and arm and kill on their own switches. |

Serial layout on the Pixhawk: the UNO Q companion on USB speaking MAVLink2, the
ESP32 obstacle ring on TELEM1 at 115200 as component 195, the SiK radio on
TELEM2, the NEO-M8N GPS and compass on SERIAL3, and the TF-Luna on SERIAL4 at
115200. The ESP32 sends plain `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR`, so
ArduPilot's own avoidance reads them unmodified. The UNO Q commands the
aircraft as component 191.

Four of the six ring positions work, plus the upward one. Marking the other two
absent in firmware mattered more than it sounds. While they were marked
present, the ESP32 retried them every five seconds and each retry blocked its
loop for about a second, which was long enough for ArduPilot to call the
proximity sensor dead and refuse to arm on roughly a fifth of our attempts.

The ring reports distances and steers nothing. `OA_TYPE`, `AVOID_ENABLE` and
`GUID_OPTIONS` are all 0 as of 23 August, and that is deliberate. We had them
on for one flight and it failed: in daylight the VL53L0X sensors return phantom
obstacles at 0.3 to 0.8 m over open ground, so path planning fought a wall that
was not there and RTL never made it home. Those same phantoms explain months of
"PreArm: Proximity" refusals. Working out which mechanism applied where meant
reading ArduPilot source rather than trusting parameter names, because simple
avoidance only touches guided velocity targets, which is the descent, while
survey rows send position targets and need `GUID_OPTIONS` bit 6 and `OA_TYPE`
together. That work stands. The sensors have to stop lying before any of it
goes back on.

## The model

One class, `puddle`, and every source dataset gets collapsed to it. An image
left with no boxes stays in as a negative, which is free hard-negative data. We
deliberately drop `pool` and `water tank`: real breeding sites, but not things
this aircraft should drop into, and teaching the model to find them would only
mean writing code to ignore them later.

`yolo26n` at 640 px, trained on the RTX 3050. We tried `yolov8n` first. The
attention models from v12 onward are too slow on an A53, and cloud inference is
not an option over a building site with no signal.

| | Precision | Recall | mAP50 | mAP50-95 |
|--|--|--|--|--|
| Run 1, `yolov8n`, 11.7k images | 0.744 | 0.687 | 0.725 | 0.431 |
| Run 2, `yolo26n`, 21.7k images | 0.795 | 0.708 | 0.766 | 0.467 |

Both on the same v2 validation set, the only fair comparison. Run 1 originally
reported 0.789 mAP50 on its own easier v1 set and re-scoring it honestly
dropped it to 0.725, which is the number above. Run 2 wins everything with
fewer parameters.

On the UNO Q it runs at 489 ms a frame, about 2 fps, and the laptop and the
board give identical predictions on the same 24 images down to the confidence
value. The mission takes stills while hovering rather than processing video, so
one second a frame was the bar. We benchmarked the larger sizes on the actual
board instead of guessing: `yolo26s` at 1921 ms a frame and `yolo26m` at 4378
ms, against 489 for the nano. Augmentation includes 180 degree rotation and
vertical flip, because a photo taken straight down has no correct way up.

Recall matters more than precision here. A missed puddle is an untreated site
and a false positive costs a few grams, so 0.708 is the figure we would rather
improve. It is also measured against public datasets mostly not shot from 5 m
looking down, which makes it a proxy for the real task rather than a
measurement of it.

Three failure modes, all found by looking at run-1 predictions one image at a
time rather than by reading metrics. Sheet water, meaning a thin film with no
puddle-shaped outline. Glare off the surface. And close range, where the water
fills the frame. Our own photos from survey height answered the first two.
Close range changed the architecture instead, which is the next section.

## Mission logic

Each of these exists because something went wrong, or was going to.

The target gets locked at altitude. The first detection at survey altitude sets
the coordinates and the aircraft stops looking. Close-range frames are where
the model is least reliable, so re-detecting would hand steering to its worst
input.

Descents abort upward. That rule separates the cases rather than treating every
missing reading as a fault:

| Situation | What it means | What happens |
|--|--|--|
| No reading, still high | Out of range. Normal. | Keep descending |
| No reading, below 6 m | Should be seeing ground | Abort upward |
| Had a reading, lost it | Dropout | Abort upward |
| EKF below drop height, rangefinder never confirmed | The two disagree | Abort upward |
| Good reading at drop height | Confirmed | Drop |

A missed puddle costs nothing. A blind descent costs the aircraft. A TF-Luna is
good to about 8 m, so a descent starting above that is legitimately blind at
first and a naive "no reading means abort" rule would abort every time. At a 5
m survey the descent now starts below that 6 m threshold, so there is a three
second grace window after entering the descent before the never-acquired case
can fire. An abort clears the target and resumes the survey. It never retries
the same puddle.

Four more rules, and the reason each one is there:

The pilot always wins. Any mode change away from guided stands the mission
down, because it must never fight the sticks.

Takeoff, each survey leg and each approach have timeouts. ArduPilot does not
acknowledge guided position targets, so a refused destination looks exactly
like one the aircraft is still flying to, and without a timeout the mission
sits there until the battery failsafe.

Detections outside the geofence are thrown away. Our camera sees well beyond
the row it is flying, so water at the edge of frame can be outside the fence.
Flying there is either refused silently or breaches and triggers an RTL.

No latch above 1.5 HDOP or below 8 satellites. We measured the position
wandering ten metres on a bad day, which is wider than the puddle.

Separately, the mission tracks how long the detector has gone without a usable
frame and aborts past the limit. That one came from a real failure: at the farm
the camera stopped enumerating after preflight passed, and without the check
the aircraft would have flown a full blind survey and reported zero detections
with nothing explaining why.

## The dose

Bti label rates work out at 0.28 to 2.24 g/m², about 1.1 g/m² mid-label. Gate
dwell comes from that and the measured flow.

| Puddle | Bti at 1.1 g/m² | Gate open |
|--|--|--|
| 1 m² | 1.1 g | 0.23 s |
| 2 m² | 2.2 g | 0.46 s |
| 4 m² | 4.4 g | 0.92 s |
| 6 m² | 6.6 g | 1.38 s |

Treat that as a placeholder. The flow rate behind it, 4.8 g/s, is the midpoint
of
4.2 to 5.3 g/s measured at the two shortest dwells, and every one of those runs
came from an under-filled hopper that was starving by the end. A full hopper
has never been measured. Mustard seed was the material, and the gate passes a
volume per second, so grams depend on bulk density, for which we have no
verified figure. Seconds are honest for seed. Grams for Bti are arithmetic, not
measurement.

At 1 m² the dwell is close to the servo's own travel time, so the dose there is
set by how fast the gate moves rather than how long it stays open. Fine salt
was the first test material and it bridged, which turned out to be cohesion
between hundred-micron grains rather than the hole being too small. Mustard
seed at about 1 mm is the closer match to VectoBac G's corn-cob granule anyway.

The flight code does not currently agree with that table, and this is the
honest place to say so. `mission.py` sizes the dwell as area times
`dose_s_per_m2`, which is 0.4 s/m² clamped between 0.3 and 3.0 s, with 1.0 s
used when the area is unknown. Over a 1 m² puddle that gives 0.4 s where the
table says 0.23 s, nearly double. The table is the figure we intend, because it
is the only one with a measured flow rate behind it, and 0.4 predates that
measurement. Changing the constant is a flight-code change and it waits until
after filming.

## Simulation

`uno_q/sitl_test.py` runs two scenarios against ArduPilot SITL. Nominal: one
drop at drop height, survey finishes, RTL, no abort. Rangefinder dropout: the
reading is suppressed mid-descent and the run passes only if the aircraft
aborts upward, drops nothing, and still finishes the survey. Both pass. That
dropout drill is the only test of the abort rule that exists, and it is a
scripted drill rather than a flight.

## The three crashes

All three happened on the S550, the airframe before this one. Two share a root
cause, which is the part worth knowing.

| | Trigger | Damage | Rule it produced |
|--|--|--|--|
| 1 | Prop nut unthreaded in flight | Prop departed, aircraft tumbled | Handed nut caps, Loctite, prop check every preflight |
| 2 | AltHold in wind | Drifted about 44 m into a tree, one arm snapped | Loiter by default, GPS gate before any GPS mode |
| 3 | Vibration corrupted the altitude estimate | Climbed to about 47 m, disarmed in the air, centre plate destroyed | Median VibeZ under 15 in hover. Abort means Stabilize, not disarm. |

Crash 1 started with a nut coming off in flight. The imbalance shook the
aircraft hard enough that the altitude estimate degraded, it climbed to roughly
12 m, and then the propeller left the motor and it tumbled. The cause was nut
handedness: half the motors on a multirotor turn each way, so a nut that
tightens on one unscrews on its neighbour. No thread lock either. That lesson
outlived the fix, because the mechanical failure was only the trigger. What
made the aircraft unflyable was the altitude estimate going bad, and it went
bad before the prop physically left.

Crash 2 was flown manually in AltHold, which holds height and does nothing
about position. Wind took it 44 m into a tree. Loiter would have held station.
Not a technical failure, so the fix is procedural: Loiter unless the plan is
deliberate manual practice, and the mode switch laid out Stabilize, AltHold,
Loiter so nobody has to hunt. The GPS rule came out of this too, meaning ten
satellites or more, HDOP under 1.5, no EKF complaints, after two to five
minutes of settling.

Crash 3 is the one the mission code is built around. Our airframe shook enough
on its own to poison the state estimate. The aircraft decided it was falling
and climbed to fight a descent that was not happening, reaching about 47 m.
Wind was pushing it toward ground it could not be retrieved from, and Reyansh
disarmed it in the air. It had also been armed within seconds of power-on, at
HDOP 65 to 99. Nothing was wrong with the GPS, it just had not been given time;
bench-tested after the crash, the same unit reached 10 satellites and HDOP
under 1.0 in thirty seconds, which settles whose fault that was. The disarm is
recorded as a deliberate choice, because calling it a loss of control would
hide the reasoning. A powered aircraft arriving in that spot was the worse
outcome. Everything else on the bench passed: battery, all six motors, ESCs
including the resoldered one, GPS, Pixhawk, telemetry radio, receiver. Only the
power module and the buzzer failed, and both were replaced.

Vibration used to be the open blocker on this project and it is not any more.
Taking the rubber motor dampeners off moved the median from about 30 to about
20.6, and the rebuild onto the F550 did the rest. Recent flight logs read a
median VibeZ of 7.4 to 9.0 against a gate of 15, with zero clipping. Log 34,
the worst on record, sat at 46 with 5927 clip events, and that flight turned
out to have a damaged prop. We keep it deliberately, because a failing
vibration plot next to a passing one is better evidence than a passing one
alone.

Two of the three crashes go the same way: vibration corrupts the altitude
estimate, the flight controller acts on it, and the aircraft does something
violent that nobody asked for. Number two was a mode choice that position hold
would have covered. This is why the mission code treats altitude with
suspicion. A descent needs the rangefinder and the EKF altitude to agree before
it will drop, and it aborts upward when they do not, and that rule exists
because of crash 3 rather than because it seemed like good practice.

## Training data and licences

Every public dataset behind the water detector is CC BY 4.0, so attribution is
a licence obligation and this section is that attribution. `TBD` means a URL or
BibTeX block nobody has written down yet. Each is on the dataset's Roboflow
Universe page under "Cite this Project", which produces the exact citation.
Guessing at them would be worse than leaving them blank.

Dataset v1, 11,725 train and 3,069 val, used by training run 1:

| Set, as named in `training/exports/` | Images | Classes kept | Citation |
|---|---|---|---|
| mosquito v1 | 5,069 incl. splits | `puddle` only | TBD |
| puddle_Detect | ~4,930 | single class | TBD |
| hanyang puddle-detection | ~1,500 | single class | TBD |
| hanyang puddle | ~1,500 | single class | TBD |
| water | ~1,000 | single class | TBD |
| yinjia part 2 | TBD | single class | TBD |

Dataset v2, about 21,700 train and 4,500 val, merged 2026-07-25. Everything
above, plus:

| Set | Images | Classes kept | Citation |
|---|---|---|---|
| Thesis, mosquito-breeding-grounds-2 | 3,470 | `Temporary Water Sites` only, of 7 | `universe.roboflow.com/thesis-kjmym/mosquito-breeding-grounds-2`, exact block still to verify |
| Fumigation habitats | ~1,060 | puddle, probable-stagnant-water | TBD |
| Fumigation habitats2 | ~1,290 | puddle, probable-stagnant-water | TBD |
| Our own nadir photographs | TBD | `puddle` | Original work, this project |

Those first-party photographs matter more than their count suggests. They are
shot straight down from height, which is the geometry this mission actually
flies, and no public dataset provides it. They include deliberate negatives:
shadows, tarpaulins, wet ground with no pooling, rooftops, plastic sheeting.

One thing to disclose. Some v2 sources carry segmentation polygons rather than
boxes, 3,001 segments against 3,881 boxes on the validation split. Ultralytics
reads the boxes and drops the polygons, so training is unaffected, but the
label files are mixed and anyone re-using the merge should know.

We rejected more than we kept. `mosquito v4` is excluded and actively harmful,
because it has no puddle class at all, so merging it adds images whose water is
unlabelled and teaches the model that water is background; it lives in
`training/excluded/` rather than deleted so the decision stays visible.
`mosquito-breeding-grounds` v3, v4 and v5 are container-only spin-offs with the
same problem. `82myj` at 112 images and `Eds breeding-detection` at 556 mostly
containers were too small to matter, `Fumigation` vol3 to vol5 have no puddle
class, the insect close-up sets are photographs of mosquitoes rather than
water, and various aedes and MBG sets cover ground that better-labelled sets
already cover.

`training/merge_datasets.py` collapses everything into one single-class set. A
set with exactly one class is kept whole whatever that class is called, which
covers sets whose only class is `water` or a non-English word for puddle. A set
with several classes keeps the water-named classes and drops the other boxes.
An image whose boxes all got dropped stays as a negative with an empty label
file, which is free training data, because an image with tyres and bottles and
no water teaches the model what standing water is not. Roboflow `test` splits
get folded into our validation split, which rests on the real test set being
the drone.

Two rules govern the merge. No two versions of the same image pool, because
Roboflow versions are re-splits and re-augmentations of the same photographs,
and merging two lands the same image in train and val, inflating the score
while teaching nothing. And `pool` and `water tank` are dropped on purpose.
They are real breeding sites, so keeping them looks defensible, but they are
not drop targets. A swimming pool is permanent managed water and a granule drop
into one is useless and unwelcome. Training the model to fire on them would
produce confident detections the mission logic then has to suppress. This is
`KEEP_NAMES` in `merge_datasets.py`.

For the submission and the video credits, once the URLs are recorded:

> The water-detection model was trained on publicly available datasets from
> Roboflow Universe, each licensed CC BY 4.0 and listed with full attribution in
> the project documentation, together with original photographs taken by the
> project team.

## Where this sits legally, in India

Not legal advice, just the project's own position. Indian drone rules have
changed repeatedly since 2021 and the assistant that drafted this has knowledge
running to May 2026, so some of it could be out of date. Every claim below
carries a confidence level and the low-confidence ones need checking against
current DGCA and Ministry of Civil Aviation text. A judge from Arduino,
Qualcomm or Robu.in may know the current rules better than this document does.

MonsoonReady is a student prototype. It has flown only in controlled test
conditions and demonstration flights drop inert mustard seed, so no pesticide
is applied at any point. Digital Sky registration has not been completed, and
that is written here as an open gap rather than dressed up as compliance.

| Claim | Confidence | Detail |
|--|--|--|
| Weight categories | Reasonably high, verify | Under the Drone Rules 2021, aircraft are categorised by all-up weight: Nano to 250 g, Micro 250 g to 2 kg, Small 2 kg to 25 kg, larger above. |
| Our category | Reasonably high | An F550 with payload is over 2 kg, so Small. Meaningfully more regulated than Micro. |
| Registration route | High that it exists, lower on detail | Registration and a unique identification number through Digital Sky |
| Airspace | High that it exists, lower on detail | Green, yellow and red zones on the airspace map |
| Pilot certification | High that it exists, lower on detail | Through DGCA-approved training organisations, requirements varying by category |
| Aerial larvicide application | Low. Needs primary sources. | A standard operating procedure exists for drone-based pesticide application on the agriculture side, and pesticide registration runs through the CIB&RC under the Insecticides Act 1968. Whether public-health larviciding by drone is treated differently is not established here. |

The gaps, stated rather than papered over. Digital Sky registration is not
done: self-registration was blocked by the portal for this class of build and
two students found no route through it, so no exemption is claimed and no blame
is placed on the portal. Neither of us holds a remote pilot certificate. Flight
conditions have been controlled prototype testing, in open areas away from
people and property, within visual line of sight, at low altitude, and someone
still has to write down the actual test site accurately rather than favourably.
Larvicide registration is unchecked: nobody has confirmed whether the Bti
product we sourced is registered for this use in India, and a photograph of the
packaging covers the documentation while saying nothing about regulatory
status.

Lawful deployment would take aircraft registration and a UIN, remote pilot
certification for whoever is flying, operation inside permitted airspace with
authorisation where required, confirmation that the larvicide is registered for
the intended use along with whatever aerial-application procedure applies,
coordination with the municipal body responsible for vector control, and
insurance plus an operations manual appropriate to the category. Coordination
is the item that gets forgotten. Larviciding public spaces is a municipal job,
and a private drone dropping granules into public water without telling anyone
is a bad idea whatever the law says. This is meant to be used by a
vector-control programme, alongside ground crews rather than instead of them.

Demonstrations drop seed for the same reason. Granular Bti kills mosquito,
blackfly and midge larvae and is used widely in public health work, and
dropping it in a demo would turn a student flight test into a pesticide
application under a body of law this project is neither qualified for nor
licensed under. So the hopper carries mustard seed. Decided early, not
discovered late.

## The demo video

Five to ten minutes, one continuous take, no cuts. It opens with today's date
on a screen, shows the components, shows the assembly, and shows the thing
working, and it is publicly viewable afterwards. That last part changes the
planning completely. No edit means no fixing it later, and it rules out a whole
category of things people put in demo videos: overlays, a music bed, montage,
telemetry graphics rebuilt from the log. Whatever happens in front of the phone
is the video.

Our first attempt was 2026-08-15 at the farm, where the camera stopped
enumerating on the board. The second field day was 2026-08-21 and the aircraft
never got past prearm. Filming moved to the field near the house. The first
demo video that did get flown surveyed at 15 m, which is the altitude the
detection rate argument above is about, and every flight since has surveyed at
5 m.

Running order, and what each part is there to prove:

| Minutes | What happens | What it proves |
|--|--|--|
| 0:00 to 0:30 | Google search for today's date, held long enough to read | The date requirement. A disqualification criterion, so get it clean. |
| 0:30 to 2:15 | Slow walk around the airframe, close on each part as it is named | The hardware exists and was built, not bought assembled |
| 2:15 to 3:00 | Wiring bay, power split, hand-soldered sensor hub, the hopper | The assembly is ours |
| 3:00 to 4:00 | The detector running on screen, box drawn, timing visible | Edge AI. The core claim, and the most important minute. |
| 4:00 to 6:30 | The flight | Functionality |
| 6:30 to 8:00 | Granules in the water, then the dashboard log | The loop closed, and it was recorded |

There are two versions of the flight. Plan A is the full autonomous loop: start
the mission, hands off except as safety pilot, and let it survey, detect,
descend, drop and come home. Plan B is flown by Reyansh with the detector
recording and the drop commanded by hand, opening with a plain sentence saying
so. Onboard detection is identical either way, which is the whole point of
having a Plan B. Our model stays on the aircraft whether or not the aircraft is
flying itself.

What gets dropped is mustard seed, and the narration says so out loud rather
than leaving it to a caption nobody reads.

Never fake a result. An overclaim a judge catches is fatal, and a limitation
stated plainly scores.

## Evidence

Every claim above should have something behind it. Files go in
`docs/evidence/`, which so far holds only the two SITL transcripts. Use the
names below so the cross-references resolve, and downsize large image sets
before committing.

Model and training: `training_curve.png` for mAP against epoch, proving
training was real and converged. `training_terminal.png` with GPU and batch
size visible, proving it was trained locally on the RTX 3050 rather than in a
cloud notebook. `dataset_counts.txt`, the `merge_datasets.py` output.
`spotcheck_grid.png`, validation predictions including the failures, which is
the argument for dataset v2. `onnx_benchmark.txt`, already done at 489 ms
median and 2.05 fps. And `unoq_detection.png`, a UNO Q screen capture with box,
confidence and timing, which is the single most important artefact in the
project. `training/spotcheck/` and `training/spotcheck_aerial/` already hold
run-1 prediction images and should survive the retrain, because run 1 against
run 2 on identical images is itself evidence.

Bench: `hopper_flow_test.mp4`, seed through the tube and gate without bridging.
`hopper_dose.png`, a weighed dose, which needs re-running from a full hopper.
`tfluna_water_bench.png`, the TF-Luna over a water basin, which is what decided
descend-beside. `esp32_mavlink_inspector.png` showing `OBSTACLE_DISTANCE` and
`DISTANCE_SENSOR` at 10 Hz from component 195. `ring_channels.txt` from
`tools/ring_channels.py`. `oled_status.jpg`, `power_calibration.jpg`, and
`vibration_log.png`. Capture the vibration plot whether it passes or fails; log
34 is the failing one at median 46 with 5927 clip events and recent logs read
7.4 to 9.0, and both together beat the passing one alone.

Flight: `hover_test.mp4` for the unloaded hover, `gps_health.png` for
satellites and HDOP before arming, `loiter_flight.mp4` for position hold in
wind, `full_loop.mp4` for the judged loop, and `flight_log.bin`. Keep the
`.bin` from every flight, not just the good ones, because logs-first
troubleshooting is a standing rule here.

Build: `build_progress_*.jpg` for assembly, wiring, service loops and foam
mounting, `payload_layout.jpg` with weights, `weight_measurement.jpg` which
feeds the regulatory category question, `sensor_ring.jpg` showing the field of
view clear of legs and props, and `conformal_coating.jpg`.

Simulation: `sitl_happy_path.txt` and `sitl_dropout_drill.txt` are captured
already, and both regenerate with SITL running.

```bash
./python uno_q/sitl_test.py > docs/evidence/sitl_happy_path.txt
```

```bash
./python uno_q/sitl_test.py --drill dropout > docs/evidence/sitl_dropout_drill.txt
```

Documentation: `dataset_licences.png`, the Roboflow Universe pages showing CC
BY
4.0 and the citation blocks, which also fills the TBD rows above. And
`bti_product.jpg`, the larvicide packaging.

If time runs short, the order is `unoq_detection.png` for the edge-AI claim,
`full_loop.mp4` for functionality, `vibration_log.png` for engineering honesty,
`sitl_dropout_drill.txt` for safety design, and `spotcheck_grid.png` for honest
evaluation.

## How AI was used

We used Claude heavily. It wrote code, researched parts and datasets, and
drafted these documents. Here is the split, including the times it was wrong.

The short version, for the submission and the video:

> This project was built with substantial AI assistance. Claude wrote and
> reviewed code, researched component and dataset choices, and drafted the
> documentation. The architecture, the engineering decisions, all of the
> building, and all of the testing and debugging are ours. We understood
> everything we accepted, and we corrected it when it was wrong.

| Area | The assistant | Us |
|------|---------------|-----|
| Python in `tools/`, `training/`, `uno_q/` | Wrote it | Specified it, designed the state machine, reviewed, ran, debugged |
| ESP32 firmware | Wrote it | Specified and reviewed. It has never flashed the board. |
| Parts and datasets | Options and comparisons | Every actual choice |
| These documents | Drafted from the state file, the build log and the source | Corrected and cut |
| The idea | Nothing | Ours, including the point that granular larvicide means no spray system |
| The architecture | Nothing | Hexacopter, serial layout, detection onboard, target latching, abort upward, ESP32 speaking standard MAVLink instead of something custom |
| Building and flying it | Nothing | Assembly, soldering, wiring, calibration, every flight, all three crashes |
| Debugging | Nothing | It has never had the aircraft in front of it |

Four mistakes cost real time.

Parameter names. ArduPilot renames things between versions and the assistant
kept using old names from memory. `WPNAV_SPEED` does not exist on 4.7, it is
`WP_SPD`, and that one would have failed silently. Same with `RTL_ALT` to
`RTL_ALT_M`, `RNGFND1_GNDCLEAR` to `RNGFND1_GNDCLR`, and `ARMING_CHECK`, which
4.7 replaced with `ARMING_SKIPCHK` and inverted while it was at it, so the old
name reads as a dead link rather than an error. Our fix was a rule: check the
board or the source, never memory. It kept happening anyway.

Hardware this board cannot do. It planned a bidirectional-DShot RPM notch
filter before anyone checked whether the timer channels have DMA. They do not.
We moved to an in-flight FFT notch.

A rangefinder rule that would have aborted every descent. What it proposed
treated "no reading" as a fault, but the TF-Luna is good to about 8 m, so the
first part of any descent starting above that is blind and always will be.
Caught in review, and the rule now separates "never acquired" from "acquired
then lost".

Two theories that survived on plausibility alone. It decided our arming
failures came from the firmware's 201 cm "clear" value being read as no data,
and held that for three days. Reading the ArduPilot source killed it, because
the driver timestamps every message before checking any sector, and the real
cause was the ESP32 blocking for a second at a time while it retried two dead
sensors. Separately, it assumed for days that `AVOID_ENABLE=7` meant the
obstacle ring protected the whole flight, when the source shows avoidance is
applied only to guided velocity targets and the survey rows send position
targets. Both are the same failure: trusting a plausible model of the system
instead of going and looking.

It is good at "write this, check that" and it needs somebody who knows the
hardware sitting next to it.

Two people, two machines, one repository, an assistant on each side. Neither
assistant can see the other's session, so everything goes through
`PROJECT_STATE.md`: current state, a dated decision log nobody edits after the
fact, and a section saying who is working on what. Every change updates it and
every session starts by reading it. That turned out to be the whole trick.

## Where it stands

Working: the airframe, which clears its vibration gate; the ring; the
dashboard; parameter management; the detector on the board; the geofence and
route generation; and the whole mission loop in simulation.

Not done: the full autonomous loop has never flown. The aircraft has been armed
with the mission commanded, but between GPS quality, a fence drawn too tight
around the take-off spot and the proximity refusals above, there is no complete
automatic flight on video.

With more time we would fly the full loop on video, measure the dose from a
full hopper, and run a real survey across several days so the persistence claim
is demonstrated rather than described.

Reproducing it: one repository, and every decision sits in a dated,
author-tagged, append-only log in `PROJECT_STATE.md`, because two people on two
machines built this. Commands are in the top-level `README.md`. One thing worth
saying is that we do not use bulk parameter load from a ground station, because
it drops writes silently. `tools/parameters.py` acknowledges every one.
