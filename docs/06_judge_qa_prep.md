# Questions a judge might ask

Short answers, one per decision. The long versions are in the other files.

## The concept

**Why not spray, like agricultural drones?**
Larval control does not need spraying. Granular Bti only has to land in the
water, which removes the tank, the pump, the nozzle, the drift and most of the
weight. The whole dispenser is a servo and a tube.

**How do you know the water you found is a breeding site?**
From one pass we do not, and we do not claim to. Stagnation is a property of
time, so it takes the same candidate appearing across passes on different days
plus someone confirming on the ground.

**Is this not just object detection with a drone attached?**
Detection is the easy part. The engineering is what happens after: latching the
target so unreliable close-range frames cannot steer the aircraft, descending on
a rangefinder expected to fail over water, aborting upward when it does, and
never fighting the pilot.

**Who would use it?**
A municipal vector-control programme, alongside ground crews rather than
instead of them.

## The model

**Why onboard instead of streaming to a server?**
No dependable link over a construction site, cloud round-trip does not fit a
descent decision, and the challenge is about physical AI at the edge.

**Why `yolo26n` and not YOLOv8n, or something bigger?**
Better accuracy at a smaller size, roughly twice as fast on CPU ONNX, and an
NMS-free export. We benchmarked the bigger sizes on the actual board rather than
guessing: `yolo26s` at 1921 ms a frame and `yolo26m` at 4378 ms, against 489 ms
for the nano.

**What are your numbers?**

| | Precision | Recall | mAP50 | mAP50-95 |
|--|--|--|--|--|
| Run 1, `yolov8n`, 11.7k images | 0.744 | 0.687 | 0.725 | 0.431 |
| Run 2, `yolo26n`, 21.7k images | 0.795 | 0.708 | 0.766 | 0.467 |

Both on the same v2 validation set. Run 1 originally reported 0.789 mAP50 on its
own easier v1 set, and re-scoring it honestly dropped it to 0.725.

**What frame rate do you get?**
489 ms median on the board, about 2 fps. The mission takes stills while hovering
rather than processing video, so one second a frame was the bar. Laptop and
board give identical predictions on the same 24 images.

**Why one class instead of containers, tyres and tanks?**
They are not drop targets. The granules have to reach the water, and a model
firing on containers produces confident detections the mission logic then has to
suppress.

**Where does it fail?**
Sheet water, a thin film with no puddle-shaped outline, and strong glare. Both
turned up by looking at run-1 predictions image by image, which is the only way
that class of failure surfaces. Dataset v2 targets both.

**Is 0.708 recall good enough?**
Recall matters more than precision here. A missed puddle is an untreated site; a
false positive costs a few grams. The figure is measured against public datasets
mostly not shot from 15 m looking down, so it is a proxy for the real task.

**How did you avoid train/test contamination?**
No two datasets in the merge come from the same image pool. Roboflow versions of
one dataset are re-splits of the same photographs.

## The aircraft

**This drone has crashed three times. Why should we believe it flies?**
Each cause is known and each produced a rule. Two of the three trace to the same
chain: vibration corrupts the altitude estimate, and an aircraft with a bad
altitude estimate fights the pilot. Details in `02_crash_postmortems.md`.

**Do you meet the vibration gate?**
Yes, now. The gate is a median VibeZ under 15 in hover and recent logs read 7.4
to 9.0 with zero clipping. It used to be the open blocker.

**What happens if the rangefinder fails during descent?**
It aborts upward and abandons the target. A missed puddle costs nothing, a blind
descent costs the aircraft. Not hypothetical: the TF-Luna uses 850 nm infrared
and still water behaves close to a mirror at that wavelength, which is also why
it descends beside the puddle and crosses over holding that height.

**The rangefinder cannot see the ground from 15 m. Does it not abort every
time?**
That is the subtlety the rule had to get right, because the first part of every
descent is legitimately blind. It separates the cases: abort if the return was
acquired then lost, or never acquired by the height where the sensor must see
ground, or if the EKF says the aircraft is below drop height and the rangefinder
never confirmed. After two crashes caused by a corrupted altitude estimate, no
single altitude source acts alone.

**Have you tested that?**
In simulation, as a scripted drill against SITL. The rangefinder goes silent
mid-descent and the run passes only if the aircraft aborts upward, drops
nothing, and still finishes the survey. It passes.

**What stops the drone fighting the pilot?**
Any mode change away from GUIDED stands the mission code down and it stops
commanding.

**Why an ESP32 for the ring instead of the flight controller?**
Cost, and I2C addressing. All seven VL53L0X share one address, so they need a
multiplexer and sequenced reads. The ESP32 emits standard `OBSTACLE_DISTANCE`
and `DISTANCE_SENSOR` as component 195, so ArduPilot consumes them unmodified.

**Does the ring work?**
Four of the six positions plus the upward one. Marking the other two absent in
firmware mattered: while they were marked present, the ESP32 retried them every
five seconds and each retry blocked its loop for about a second, long enough for
ArduPilot to call the sensor dead and refuse to arm.

**Does it actually avoid anything during an autonomous flight?**
Two mechanisms, one per phase, and getting this right meant reading ArduPilot
source rather than trusting parameter names. Simple avoidance applies only to
guided velocity targets, which is the descent. Survey rows send position targets
and get path planning instead, which needs `GUID_OPTIONS` bit 6 and `OA_TYPE`
together. We found the survey unprotected because neither was set. Both are set
now, and neither has flown.

**What if the obstacle module dies in flight?**
The proximity backend makes it an arming dependency, so a dead module is caught
on the ground. Field recovery is `PRX1_TYPE=0` and a reboot.

**What if the camera dies in flight?**
The mission tracks how long the detector has gone without a usable frame and
aborts past the limit. This came from a real failure: at the farm the camera
stopped enumerating after preflight passed, and without the check the aircraft
would have flown a full blind survey and reported zero detections with nothing
explaining why.

## Process

**How much of this did AI write?**
Substantially, for code and documentation. The architecture, the decisions, the
assembly and the testing were ours. Full scope, including where the assistant
was wrong, in `08_ai_authorship_disclosure.md`.

**Is this legal?**
A student prototype in controlled test conditions, dropping mustard seed rather
than a pesticide. Digital Sky registration was not completed and that is stated
openly in `05_compliance_narrative.md`.

**What would you do with more time?**
Fly the full loop on video, measure the dose from a full hopper, and run a real
survey across several days so the persistence claim is demonstrated rather than
described. That last one is the honest gap between what the system does and what
the concept promises.

**What was the hardest part?**
Each of us answers this from what actually happened. Candidates: the prop-nut
handedness discovery, the ground station silently dropping parameter writes, the
flight where the altitude estimate lied, or three days on an arming failure that
turned out to be two dead sensors blocking a serial link.
