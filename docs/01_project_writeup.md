# MonsoonReady: an autonomous larvicide drone for monsoon mosquito breeding sites

Arduino Physical AI Challenge India, 2026. Team of two (Raghav, Reyansh).

## 1. The problem

*Aedes aegypti*, the mosquito that carries dengue and chikungunya, breeds in
small bodies of standing water that appear everywhere after monsoon rain:
puddles on flat rooftops, water pooled on construction sites, blocked drains,
tarpaulins, unused tanks. The larval stage is where the mosquito is most
vulnerable and least mobile. Municipal control programmes rely on ground crews
walking sites and treating water by hand, which is slow, and which misses the
places a person cannot easily reach or does not know about, such as a flat
roof three buildings over.

The insight is not that drones can spray. Agricultural spray drones exist. It
is that larval control does not need spraying at all. Granular *Bti* only has
to **land in the water**. That removes the tank, the pump, the nozzle, the
spray drift, and most of the weight, and turns the problem into: find the
water, get above it, drop a measured amount of granules.

## 2. What the system does

One judged loop, all of it onboard:

1. The hexacopter flies a survey pattern at approximately 15m.
2. An **Arduino UNO Q** onboard the aircraft runs a YOLO water-detection model
   on downward-facing stills from a USB camera. No cloud, no ground station in
   the loop.
3. On a detection, the target's position is **latched** at survey altitude.
4. The aircraft moves over the target and descends, watching a downward
   rangefinder the whole way.
5. At drop height, an SG90 servo opens a gate on a granule tube and a measured
   dose falls into the water.
6. The aircraft climbs back to survey altitude and resumes the pattern.
7. On landing, the UNO Q switches role and becomes a base station, serving a
   heatmap and report of what was found and treated.

Demo flights drop **inert salt**, not Bti, so that nothing about the
demonstration is a pesticide application. See
[05_compliance_narrative.md](05_compliance_narrative.md).

## 3. What we claim, and what we do not

The model detects **standing-water candidates**. It cannot see stagnation,
because stagnation is a property of time, not of a single frame. A fresh
puddle and a two-week-old breeding site look identical from 15m.

That is a real limitation and we state it rather than paper over it. In the
system design, stagnation is established the way it actually can be:
persistence of the same candidate across repeated passes on different days,
plus operator confirmation. A single flight produces candidates; a survey
programme produces breeding sites.

We would rather be marked down for a narrow honest claim than caught making a
broad false one.

## 4. Hardware

Everything below is either flying hardware or bench hardware already in hand.

| Subsystem | Part | Why this one |
|---|---|---|
| Frame | F550 hexacopter (X) | Six motors give payload margin and survive a motor failure. Replaced an S550 destroyed in crash 3, for which centre plates are no longer available. |
| Flight controller | Pixhawk 2.4.8, ArduCopter 4.7.0 (Pixhawk1-bdshot) | Full ArduPilot support, mature guided-mode MAVLink interface, good logging for post-incident analysis. |
| Motors / props | 6x DJI A2212 920KV, DJI-style 1045 | Chosen after EMAX MT2213 propeller availability failed in India. |
| ESCs | 6x 45A BLHeli_32 | Rated far above the A2212's draw, so they never run hot. |
| Battery | 3S 8000mAh LiPo, XT60 | Survey endurance. |
| AI compute | **Arduino UNO Q, 4GB** | Runs the detector onboard. The Physical AI premise of the project. |
| Camera | Logitech B525, 720p UVC | Already owned; UVC works on the UNO Q today, which beat any spec advantage a new camera would have offered. |
| Height / descent | Benewake TF-Luna, serial, downward | Native ArduPilot support, gives true height above the water surface during descent and a size estimate for dosing. |
| Obstacle sensing | 7x VL53L0X on a TCA9548A mux, read by an ESP32, translated to MAVLink | Six in a 60 degree ring, one facing up. Cheapest way to get a proximity ring the flight controller understands natively. |
| Dispenser | SG90 servo gate on a tube | No tank, no pump, no nozzle. The cheapest actuator that can do the job. |
| Status display | 1.3in I2C OLED | Prearm pass/fail, satellites, EKF, mode, battery, readable at the field with no laptop. |
| Telemetry | 433MHz SiK | Ground monitoring during tests. |
| RC | FlySky FS-i6X / FS-iA10B, iBUS | 10 channels, dedicated arm and kill switches. |

### Serial architecture

The Pixhawk's serial ports are allocated so that every subsystem talks to the
flight controller in a language it already understands, rather than through a
custom protocol:

- SERIAL1: SiK telemetry
- SERIAL2: ESP32 obstacle module, MAVLink2 at 115200
- SERIAL3: GPS
- SERIAL4: UNO Q mission computer, MAVLink2 at 115200
- SERIAL5: TF-Luna rangefinder at 115200

The ESP32 presents itself as MAVLink component 195 and emits standard
`OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` messages, so ArduPilot's existing
avoidance code consumes them with no modification. The UNO Q is component 191
and commands the aircraft through standard guided-mode messages. Nothing in
this design requires a firmware fork.

## 5. The detection model

**Task framing.** Single class, `puddle`. Every source dataset is collapsed to
that one class by [training/merge_datasets.py](../training/merge_datasets.py).
Multi-class sets keep only water-named classes; other boxes (tyres, bottles,
containers) are dropped, and an image left with no boxes stays in the set as a
negative, which is free hard-negative data.

One deliberate exclusion: classes named `pool` or `water tank` are **not**
kept. They are mosquito breeding sites, but they are not drop targets for this
aircraft, and training the model to fire on them would produce confident
detections we would then have to suppress.

**Model.** `yolo26n`, 640px, trained on the RTX 3050 laptop. Chosen over
YOLOv8n for higher accuracy at similar size, faster CPU ONNX inference, and an
NMS-free export, which matters because it removes a postprocessing step from
the UNO Q's CPU. Attention-based successors were rejected: they are too slow on
this class of ARM CPU for any benefit they bring.

**Training data.** Dataset v1 was 11,725 train / 3,069 val images merged from
public Roboflow sets. Dataset v2, merged 2026-07-25, is approximately 21,700
train / 4,500 val. Full attribution in
[03_dataset_citations.md](03_dataset_citations.md).

**Results.** Run 1 (dataset v1, YOLOv8n baseline, stopped around epoch 160 of
200 after plateauing):

| Metric | Value |
|---|---|
| Precision | 0.79 |
| Recall | 0.72 |
| mAP50 | 0.789 |
| mAP50-95 | 0.474 |

`FILL: v2 / yolo26n results once the retrain finishes on the RTX 3050.`

**Known failure modes, found by inspecting run 1 predictions** rather than by
reading the metrics: sheet water (a thin film across a wide surface, which has
no puddle-like outline) and strong glare. Dataset v2 targets these directly,
including first-party nadir photographs taken at survey height, which no
public dataset provides.

**Why not cloud inference.** Three reasons, in order of importance. A drone
over a construction site has no dependable link. Round-trip latency to a cloud
endpoint is incompatible with a descent decision loop. And the challenge is
about physical AI at the edge, so putting the model in a datacentre would be
answering a different question. The model runs on the aircraft or the project
has failed.

## 6. Mission logic and safety

The onboard mission code lives in [uno_q/](../uno_q/README.md) and was
developed and tested against ArduPilot SITL before touching hardware. Three
behaviours in it are worth calling out because each one exists in response to
something that has already gone wrong on this project or is known to be
physically risky.

**Target latching.** The target's coordinates are locked on first detection at
survey altitude, and detections are ignored for the rest of the manoeuvre. The
reason is a finding from inspecting run 1 predictions: close-range frames,
where the water fills most of the frame, are exactly where the model is least
reliable. A naive implementation that re-detects during descent would let the
least trustworthy frames steer the aircraft. Latching means the most
informative view, the wide one from altitude, is the one that decides.

**Descent abort is upward, always.** The TF-Luna uses 850nm infrared, and
still water is close to a mirror at that wavelength. A specular dropout, where
the beam reflects away and the sensor reports nothing usable, is expected
rather than hypothetical. The rule is that a descent with no trustworthy height
reference aborts upward and abandons the target. A missed puddle costs
nothing; a blind descent costs the aircraft.

The subtlety is that a naive "no reading means abort" rule would abort every
single descent, because the TF-Luna cannot see the ground from 15m at all: the
first part of every descent is legitimately blind. The implemented rule
distinguishes the two cases. Abort if the ground return was acquired and then
lost, or if it was never acquired by the altitude at which the sensor must be
able to see the ground. There is a third guard on top: if the EKF's altitude
says we are below drop height but the rangefinder never confirmed it, the two
altitude sources disagree and the descent aborts. Given that a corrupted
altitude estimate is the direct cause of two of this project's three crashes,
we do not let one altitude source act alone.

**The pilot always wins.** If the flight mode changes away from GUIDED for any
reason, the mission code stands down and stops commanding the aircraft. It
never fights the human on the sticks. Crash 3 taught us how bad it is when
aircraft and pilot disagree about what should happen next.

Both behaviours are demonstrated in simulation by
[uno_q/sitl_test.py](../uno_q/sitl_test.py), which runs two scenarios against
a simulated hexacopter: a nominal mission, which must produce exactly one drop
and complete the survey, and a rangefinder-dropout drill, which must produce
zero drops, abort upward, and still complete the survey. Both pass. This is the
functionality evidence for the parts of the loop that are not yet flown.

## 7. Base station mode

After landing, the UNO Q stops being a mission computer and becomes a report
server: a heatmap of detections against the survey area, the treated sites, and
the images that triggered each drop. The point is that a survey produces a
municipal work product, not just a flight. Repeated surveys are also what turns
"standing-water candidate" into "confirmed breeding site", per section 3.

`FILL: implementation in progress, TODO 13.`

## 8. Engineering honesty: what is not done

Judges can tell the difference between a project that has been flown and one
that has been described. As of 2026-07-26:

- The airframe is a rebuild in progress after crash 3.
- **Vibration is the open blocker.** Median vibration was approximately 20.6
  against a safe limit of 15. Until an unloaded hover clears that gate, no
  altitude-holding flight mode is trustworthy on this aircraft. This is the
  single item most likely to constrain the demo.
- The ESP32 obstacle module compiles cleanly in both real and fake sensor
  modes but has never been flashed to hardware.
- The detect-descend-treat loop is proven in simulation, not yet in flight.
- Digital Sky registration is parked because the portal blocks self-registration
  for this class of build. See [05](05_compliance_narrative.md).

The [descope ladder](../PROJECT_STATE.md) we committed to, in order: Loiter
plus onboard detection plus drop is the minimum judgeable demonstration; then
the base-station report; then the obstacle array; then fully automatic guided
descent.

## 9. Reproducibility

The whole project is one git repository, and every decision is in an
append-only log in `PROJECT_STATE.md` with a date and an author, because two
people on two machines built this.

- Pixhawk parameters are pushed with [tools/push_params.py](../tools/push_params.py),
  which acknowledges every individual write. We do not use ground-station bulk
  parameter load, having found it silently drops writes.
- Training is one command against a merged dataset, checkpointed every epoch
  and resumable.
- The mission code runs unchanged against the simulator and the aircraft; only
  the connection string differs.

## 10. AI assistance

Disclosed in full in
[08_ai_authorship_disclosure.md](08_ai_authorship_disclosure.md).
