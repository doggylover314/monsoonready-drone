# Questions a judge might ask

Every design decision here has a reason, and this is those reasons in a form
short enough to say out loud.

## The concept

**Why not spray, like agricultural drones?**
Larval control does not need spraying. Granular Bti only has to land in the
water. Dropping granules removes the tank, the pump, the nozzle, the drift and
most of the weight, which turns a chemical-application problem into a
navigation problem. The whole dispenser is a servo and a tube.

**How do you know the water you found is a breeding site?**
From one pass we do not, and we do not claim to. The model finds standing
water. Stagnation is a property of time, so it takes the same candidate showing
up across repeated passes on different days, plus someone confirming on the
ground. One flight produces candidates. A survey programme produces breeding
sites.

**Is this not just object detection with a drone attached?**
Detection is the easy part. The engineering is what happens after it: latching
the target so unreliable close-range frames cannot steer the aircraft,
descending on a rangefinder that is expected to fail over water, aborting
upward when it does, throwing away detections outside the geofence, and never
fighting the pilot.

**Who would use it?**
A municipal vector-control programme, as a survey and treatment tool alongside
ground crews rather than instead of them.

## The model

**Why run it onboard instead of streaming to a server?**
A drone over a construction site has no dependable link. Cloud round-trip does
not fit a descent decision. And the challenge is about physical AI at the edge,
so putting the model in a datacentre answers a different question.

**Why `yolo26n` and not YOLOv8n, or something bigger?**
Better accuracy at a smaller size, roughly twice as fast on CPU ONNX, and an
NMS-free export that takes a postprocessing stage off the board. The attention
models from v12 onward are too slow on an A53 to justify what they add. We
benchmarked the bigger sizes on the actual board rather than guessing:
`yolo26s` came in at 1921 ms per frame and `yolo26m` at 4378 ms, against 489 ms
for the nano.

**What are your numbers?**

| | Precision | Recall | mAP50 | mAP50-95 |
|--|--|--|--|--|
| Run 1, `yolov8n`, 11.7k images | 0.744 | 0.687 | 0.725 | 0.431 |
| Run 2, `yolo26n`, 21.7k images | 0.795 | 0.708 | 0.766 | 0.467 |

Both on the same v2 validation set, which is the only comparison worth making.
Run 2 wins everything with 2.4M parameters against 3.0M. Run 1 originally
reported higher figures, 0.789 mAP50, but that was against its own easier v1
validation set, and re-scoring it honestly dropped it to 0.725.

**What frame rate do you get?**
489 ms median on the UNO Q's CPU, so about 2 frames per second, measured on the
board. The mission takes stills while hovering rather than processing video, so
one second per frame was the bar and there is roughly twice the headroom. The
laptop and the board produce identical predictions on the same 24 images, down
to the confidence value.

**Why one class instead of detecting containers, tyres and tanks?**
They are not drop targets. A tyre holding water is a real breeding site, but
the granules have to reach the water. A model firing on containers produces
confident detections the mission logic then has to suppress. `pool` and
`water tank` are filtered out of the training data for the same reason.

**Where does it fail?**
Sheet water, meaning a thin film with no puddle-shaped outline, and strong
glare. Both turned up by looking at run-1 predictions image by image, which is
the only way that class of failure surfaces. Dataset v2 targets both, including
our own photographs shot straight down from survey height, which no public
dataset provides.

**Is 0.708 recall good enough?**
For this mission recall matters more than precision. A missed puddle is an
untreated site. A false positive costs a few grams of granules and some flight
time. The threshold would be tuned toward recall in deployment. The figure is
also measured against public datasets whose images are mostly not shot from
15 m looking down, so treat it as a proxy for the real task rather than a
measurement of it.

**How did you avoid train/test contamination?**
No two datasets in the merge come from the same underlying image pool. Roboflow
versions of one dataset are re-splits of the same photographs, so merging two
versions would put the same image in train and val. Each source's test split is
folded into validation, on the basis that the real test set is the drone.

## The aircraft

**Why a hexacopter rather than a quad?**
Payload margin, and it survives losing a motor. With a dispenser, camera,
rangefinder, obstacle ring and companion computer aboard, the margin matters.

**This drone has crashed three times. Why should we believe it flies?**
Because each cause is known and each produced a rule. Two of the three trace to
the same chain: vibration corrupts the altitude estimate, and an aircraft with
a bad altitude estimate fights the pilot. The third was AltHold in wind when
Loiter was the right mode. Full write-ups in `02_crash_postmortems.md`.

**Do you meet the vibration gate?**
Yes, now. The gate is a median VibeZ under 15 in hover. Recent logs read 7.4 to
9.0 with zero clipping. It used to be the open blocker on this project, and it
took removing the rubber motor dampeners and rebuilding onto the F550 to get
there.

**What happens if the rangefinder fails during descent?**
It aborts upward and abandons the target. A missed puddle costs nothing, a
blind descent costs the aircraft. This is not hypothetical. The TF-Luna uses
850 nm infrared and still water at that wavelength behaves close to a mirror,
so dropout over the exact thing being descended toward is expected. It is also
why the aircraft descends beside the puddle on dry ground and then crosses over
it holding that height, rather than descending over the water.

**The rangefinder cannot see the ground from 15 m. Does it not abort every
time?**
That is the subtlety the rule had to get right. A naive "no reading means
abort" would abort every descent, because the first part of every descent is
legitimately blind. The rule separates the cases: abort if the return was
acquired and then lost, or if it was never acquired by the height at which the
sensor must be able to see ground. A third guard sits on top. If the EKF says
the aircraft is below drop height and the rangefinder never confirmed, the two
sources disagree and it aborts. After two crashes caused by a corrupted
altitude estimate, no single altitude source acts alone.

**Have you tested that?**
In simulation, as a scripted drill against ArduPilot SITL. The rangefinder goes
silent mid-descent and the run passes only if the aircraft aborts upward, drops
nothing, and still finishes the survey. It passes.

**What stops the drone fighting the pilot?**
Any mode change away from GUIDED stands the mission code down and it stops
commanding entirely. The pilot always wins.

**Why does the obstacle ring use an ESP32 instead of the flight controller?**
Cost, and I2C addressing. All seven VL53L0X sensors share one address, so they
need a multiplexer and sequenced reads, which is fiddly work that does not
belong on a flight controller. The ESP32 does it and emits standard
`OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` as component 195, so ArduPilot's own
avoidance consumes them unmodified. No firmware fork.

**Does the ring work?**
Four of the six ring positions plus the upward one. One position never got a
sensor because there is no room for it on that side of the frame, and one
sensor has never answered. Both are marked absent in firmware, and that turned
out to matter: while they were marked present, the ESP32 retried them every
five seconds and each retry blocked its loop for about a second, which was long
enough for ArduPilot to call the proximity sensor dead and refuse to arm.

Worth being precise about what the ring does during an autonomous flight. Read
from ArduPilot 4.7 source, simple avoidance is applied to guided velocity
targets and not to guided position targets. The survey rows are position
targets, so the ring is not steering the aircraft there. The descent uses
velocity targets, so it is active then.

**What if the obstacle module dies in flight?**
Enabling the proximity backend makes the ESP32 an arming dependency, so a dead
module is caught on the ground instead of in the air. Field recovery is
`PRX1_TYPE=0` and a reboot, which flies without the ring.

**What if the camera dies in flight?**
The mission tracks how long the detector has gone without a usable frame and
aborts if it passes the limit. This came out of a real failure. At the farm the
camera stopped enumerating after preflight had already passed, and without that
check the aircraft would have flown a complete blind survey, landed cleanly,
and reported zero detections with nothing explaining why.

## Process

**How much of this did AI write?**
Substantially, for code and documentation. The architecture, all the decisions,
all the assembly and all the testing and debugging were ours. Full scope,
including where the assistant was wrong, in `08_ai_authorship_disclosure.md`.

**Is this legal?**
It is a student prototype flown in controlled test conditions, dropping mustard
seed rather than a pesticide. Digital Sky registration was not completed, and
that is stated openly. `05_compliance_narrative.md` sets out what lawful
deployment would need and rates its own confidence in each claim.

**What would you do with more time?**
Fly the full loop on video, measure the dose properly from a full hopper rather
than an under-filled one, and run a real survey across several days so the
persistence claim is demonstrated rather than described. That last one is the
honest gap between what the system does and what the concept promises.

**What was the hardest part?**
Each of us answers this one personally, from what actually happened rather than
from a document. Candidates: the prop-nut handedness discovery, the ground
station silently dropping parameter writes, the flight where the altitude
estimate lied, or three days spent on an arming failure that turned out to be
two dead sensors blocking a serial link.
