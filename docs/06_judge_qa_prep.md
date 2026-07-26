# Judge Q&A preparation

The standing rule on this project is that we must be able to explain every
design decision. This document is that, in question form. Each answer is short
enough to say out loud.

Answer from knowledge, not from a script. If a question comes that is not
here, the correct answer is the honest one, including "we do not know yet".

---

## The concept

**Why not just spray from the drone like agricultural drones do?**
Because larval control does not need spraying. Granular Bti only has to land in
the water. Dropping granules removes the tank, the pump, the nozzle, the spray
drift and most of the weight, and it turns a chemical-application problem into
a navigation problem. Our whole dispenser is a servo and a tube.

**How do you know the water you found is actually a mosquito breeding site?**
We do not, from one pass, and we do not claim to. The model detects
standing-water candidates. Stagnation is a property of time, so it is
established by the same candidate persisting across repeated passes on
different days, plus operator confirmation. One flight produces candidates; a
survey programme produces breeding sites.

**Is this not just object detection with a drone attached?**
The detection is the easy part. The engineering is in what happens after the
detection: latching a target so unreliable close-range frames cannot steer the
aircraft, descending on a rangefinder that is expected to fail over water,
aborting upward when it does, and never fighting the pilot. That logic is the
project.

**Who would use this?**
A municipal vector-control programme, as a survey and treatment tool. Not a
replacement for ground crews. Section 5 of the compliance narrative is explicit
that coordination with the responsible municipal body is a precondition.

---

## The AI and edge compute

**Why run the model on the drone instead of streaming to a server?**
Three reasons, in order. A drone over a construction site has no dependable
link. Cloud round-trip latency does not fit a descent decision loop. And the
challenge is about physical AI at the edge, so moving the model to a datacentre
would be answering a different question.

**Why yolo26n and not YOLOv8n, or something larger?**
yolo26n gives better accuracy at a similar size, faster CPU ONNX inference, and
an NMS-free export, which removes a postprocessing step from the UNO Q's CPU.
Larger attention-based models are too slow on this class of ARM CPU to justify
what they add. The nano size is a deliberate match to the compute we have.

**What frame rate do you get?**
`FILL: measured ONNX benchmark figure from the UNO Q.` Note when answering that
the mission takes stills while hovering rather than processing continuous
video, so roughly one second per frame is acceptable. That is a design
consequence, not an excuse: hovering to take a considered still is a better
sampling strategy than a blurred frame at speed.

**Why one class instead of detecting containers, tyres, tanks and so on?**
Because those are not drop targets. A tyre with water in it is a real breeding
site, but the granules have to reach the water, and the model firing on
containers would produce confident detections the mission logic then has to
suppress. We filter pool and water-tank classes out of the training data for
the same reason. Better not to learn it than to learn it and override it.

**What are your numbers?**
Run 1, the baseline on dataset v1: precision 0.79, recall 0.72, mAP50 0.789,
mAP50-95 0.474, plateaued around epoch 160. `FILL: v2 / yolo26n figures.`

**Where does it fail?**
Sheet water, meaning a thin film spread across a wide surface with no
puddle-like outline, and strong glare. We found these by inspecting run 1's
predictions image by image rather than by reading the metrics, which is the
only way to find that kind of failure. Dataset v2 targets both directly,
including first-party nadir photographs at survey height that no public dataset
provides.

**Is 0.72 recall good enough?**
For this mission, recall matters more than precision, because a missed puddle
is a site left untreated while a false positive costs a few grams of granules
and some flight time. We would tune the operating threshold toward recall in
deployment. It is also worth saying that the metric is measured against public
datasets whose images are mostly not shot from 15m looking down, so it is a
proxy for our real task rather than a measurement of it.

**How did you avoid train/test contamination?**
No two datasets in the merge come from the same underlying image pool.
Roboflow versions of one dataset are re-splits and re-augmentations of the same
photographs, so merging two versions would put the same image in train and val
and inflate the score. We also fold each source's test split into our
validation split, on the basis that the real test set is the drone.

---

## The aircraft and safety

**Why a hexacopter rather than a quad?**
Payload margin and tolerance of a motor failure. With a dispenser, camera,
rangefinder, obstacle ring and companion computer aboard, the margin matters.

**This drone has crashed three times. Why should we believe it flies?**
Because we know exactly why each one happened and what changed, which is more
than most projects can say. Two of the three trace to the same chain:
vibration corrupts the altitude estimate, and a drone with a bad altitude
estimate fights you. The third was flying in AltHold in wind when Loiter was
the right mode. Full write-ups are in the documentation. The most important
consequence is that we now have a hard vibration gate: median VibeZ below 15 in
hover before any altitude-holding or position-holding mode is used.

**And do you meet that gate?**
Not yet. Removing the rubber motor dampeners took median vibration from about
30 to about 20.6, against a gate of 15. It is the open blocker on the project.
We would rather show you a flight that met the gate than a flight that just
happened to work.

*(Do not soften this answer. It is the strongest thing in the Q&A set, because
it demonstrates a team that sets a criterion and then respects it when the
criterion is inconvenient.)*

**What happens if the rangefinder fails during the descent?**
The aircraft aborts upward and abandons the target. A missed puddle costs
nothing; a blind descent costs the aircraft. This is not hypothetical: the
TF-Luna uses 850nm infrared and still water at that wavelength behaves close to
a mirror, so dropout over the exact thing we are descending toward is expected.

**But the rangefinder cannot see the ground from 15m. Does it not abort every
time?**
That was the subtlety we had to get right. A naive "no reading means abort"
rule would abort every descent, because the first part of every descent is
legitimately blind. The implemented rule distinguishes two cases: abort if the
ground return was acquired and then lost, or if it was never acquired by the
altitude where the sensor must be able to see the ground. There is a third
guard on top: if the EKF altitude says we are below drop height and the
rangefinder never confirmed, the two sources disagree and we abort. After two
crashes caused by a corrupted altitude estimate, we do not let one altitude
source act alone.

**Have you tested that?**
In simulation, yes, as a scripted drill against ArduPilot SITL: the rangefinder
goes silent mid-descent and the run passes only if the aircraft aborts upward,
drops nothing, and still completes the survey. `FILL: over-water bench test of
the actual TF-Luna, indoor and outdoor, nadir and angled, is TODO 6.` If that
bench test shows dropout is severe, the fallback already agreed is to hover and
descend beside the puddle rather than over it, and the mission code has the
lateral-offset parameters for it.

**What stops the drone fighting the pilot?**
If the flight mode changes away from GUIDED for any reason, the mission code
stands down and stops commanding. The pilot always wins.

**Why does the obstacle ring use an ESP32 instead of the flight controller?**
Cost and I2C addressing. Seven VL53L0X sensors share one address, so they need
a multiplexer and sequenced reads, which is fiddly work that does not belong on
the flight controller. The ESP32 does it and emits standard MAVLink
`OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` messages as component 195, so
ArduPilot's existing avoidance code consumes them unmodified. No firmware fork.

**What is the failure mode if the obstacle module dies in flight?**
Worth being precise here: enabling the proximity sensor makes the ESP32 an
arming dependency, so a dead module is caught on the ground rather than in the
air. The field recovery, if it fails at the flying site, is to set the
proximity type to zero and reboot, which flies the aircraft without the ring.

---

## Process and honesty

**How much of this did AI write?**
See [08_ai_authorship_disclosure.md](08_ai_authorship_disclosure.md) and answer
from it directly. The short version: AI assistance was used substantially for
code and documentation, the architecture and every decision were ours, and all
testing and debugging were ours. Do not be defensive about this; the disclosure
is a strength when it is volunteered and a weakness when it is extracted.

**Is this legal?**
It is a student prototype flown in controlled test conditions, dropping inert
salt rather than a pesticide. We have not completed Digital Sky registration,
and we say so. The documentation sets out what a lawful deployment would
require. See [05_compliance_narrative.md](05_compliance_narrative.md) and do
not go beyond what that document supports.

**What would you do with more time?**
Solve vibration properly, fly the full loop, and run a real survey over several
days so that the persistence-based stagnation claim in the write-up is
demonstrated rather than described. That last one is the honest gap between
what the system does and what the concept promises.

**What was the hardest part?**
`FILL: answer this personally, both of you, and do not give the same answer.
A specific, slightly unflattering true story here is worth more than a polished
one. The prop-nut handedness discovery, the ground station silently dropping
parameter writes, or the day the altitude estimate lied are all good material.`
