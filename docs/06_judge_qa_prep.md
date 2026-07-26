# Judge Q&A → Design Decisions

The standing rule on this project is that every design decision must be
explicable. This is that rule in question-and-answer form, one entry per
decision, each short enough to say out loud.

---

## 1. Concept

**Why not spray, like agricultural drones do?**
Larval control does not need spraying. Granular Bti only has to land in the
water. Dropping granules removes the tank, the pump, the nozzle, the spray
drift and most of the weight, turning a chemical-application problem into a
navigation problem. The entire dispenser is a servo and a tube.

**How do you know the water you found is a breeding site?**
From one pass, we do not, and we do not claim to. The model detects
standing-water candidates. Stagnation is a property of time, so it is
established by the same candidate persisting across repeated passes on
different days, plus operator confirmation. One flight produces candidates; a
survey programme produces breeding sites.

**Is this not just object detection with a drone attached?**
The detection is the easy part. The engineering is in what happens after it:
latching a target so unreliable close-range frames cannot steer the aircraft,
descending on a rangefinder that is expected to fail over water, aborting
upward when it does, and never fighting the pilot.

**Who would use this?**
A municipal vector-control programme, as a survey and treatment tool, not a
replacement for ground crews. `05_compliance_narrative.md` §4 makes coordination
with the responsible municipal body a precondition for deployment.

---

## 2. AI and edge compute

**Why run the model onboard instead of streaming to a server?**
Three reasons in order: a drone over a construction site has no dependable
link; cloud round-trip latency does not fit a descent decision loop; and the
challenge is about physical AI at the edge, so moving the model to a datacentre
answers a different question.

**Why `yolo26n` and not YOLOv8n, or something larger?**
Better accuracy at similar size, roughly 2× faster CPU ONNX inference, and an
NMS-free export that removes a postprocessing stage from the UNO Q's CPU.
Larger attention-based models are too slow on an A53-class CPU to justify what
they add. The nano size is a deliberate match to available compute.

**What frame rate do you get?**
TBD, pending the ONNX benchmark on the board. The mission takes stills while
hovering rather than processing continuous video, so roughly one second per
frame is acceptable. That is a design consequence rather than an excuse:
hovering for a considered still is a better sampling strategy than a blurred
frame taken at speed.

**Why one class, instead of detecting containers, tyres and tanks?**
They are not drop targets. A tyre holding water is a real breeding site, but
the granules have to reach the water. A model firing on containers produces
confident detections the mission logic then has to suppress. `pool` and
`water tank` classes are filtered out of the training data for the same reason:
better not to learn it than to learn it and override it.

**What are your numbers?**
Run 1, the dataset-v1 baseline: precision 0.79, recall 0.72, mAP50 0.789,
mAP50-95 0.474, plateaued around epoch 160. v2 / `yolo26n` figures TBD.

**Where does it fail?**
Sheet water, meaning a thin film across a wide surface with no puddle-like
outline, and strong glare. Both were found by inspecting run-1 predictions
image by image rather than by reading metrics, which is the only way that class
of failure surfaces. Dataset v2 targets both, including first-party nadir
photographs at survey height that no public dataset provides.

**Is 0.72 recall good enough?**
For this mission recall matters more than precision: a missed puddle is an
untreated site, while a false positive costs a few grams of granules and some
flight time. The operating threshold would be tuned toward recall in
deployment. The metric is also measured against public datasets whose images
are mostly not shot from 15 m looking down, so it is a proxy for the real task
rather than a measurement of it.

**How did you avoid train/test contamination?**
No two datasets in the merge come from the same underlying image pool.
Roboflow versions of one dataset are re-splits of the same photographs, so
merging two versions would place the same image in train and val and inflate
the score. Each source's test split is folded into validation, on the basis
that the real test set is the drone.

---

## 3. Aircraft and safety

**Why a hexacopter rather than a quad?**
Payload margin and tolerance of a motor failure. With a dispenser, camera,
rangefinder, obstacle ring and companion computer aboard, the margin matters.

**This drone has crashed three times. Why should we believe it flies?**
Because the cause of each is known and each produced a rule. Two of the three
trace to one chain: vibration corrupts the altitude estimate, and an aircraft
with a bad altitude estimate fights the pilot. The third was AltHold in wind
when Loiter was the correct mode. Full write-ups in
`02_crash_postmortems.md`. The most important consequence is a hard vibration
gate: median VibeZ below 15 in hover before any altitude- or position-holding
mode.

**Do you meet that gate?**
Not yet. Removing the rubber motor dampeners took median vibration from
approximately 30 to approximately 20.6, against a gate of 15. It is the open
blocker on the project.

**What happens if the rangefinder fails during descent?**
The aircraft aborts upward and abandons the target. A missed puddle costs
nothing; a blind descent costs the aircraft. This is not hypothetical: the
TF-Luna uses 850 nm infrared, and still water at that wavelength behaves close
to a mirror, so dropout over the exact thing being descended toward is
expected.

**The rangefinder cannot see the ground from 15 m. Does it not abort every
time?**
That is the subtlety the rule had to get right. A naive "no reading means
abort" would abort every descent, because the first part of every descent is
legitimately blind. The implemented rule distinguishes cases: abort if the
ground return was acquired and then lost, or if it was never acquired by the
altitude at which the sensor must be able to see ground. A third guard sits on
top: if EKF altitude says the aircraft is below drop height and the rangefinder
never confirmed, the sources disagree and it aborts. After two crashes caused
by a corrupted altitude estimate, no single altitude source acts alone. Full
table in `01_project_writeup.md` §6.2.

**Have you tested that?**
In simulation, as a scripted drill against ArduPilot SITL: the rangefinder goes
silent mid-descent, and the run passes only if the aircraft aborts upward,
drops nothing, and still completes the survey. It passes. The over-water bench
test of the physical TF-Luna is TODO 6 and decides descend-over versus
descend-beside; if dropout proves severe, the agreed fallback is to hover and
descend beside the puddle, and the mission code already carries the
lateral-offset parameters for it.

**What stops the drone fighting the pilot?**
Any flight mode change away from GUIDED stands the mission code down; it stops
commanding entirely. The pilot always wins.

**Why does the obstacle ring use an ESP32 instead of the flight controller?**
Cost and I²C addressing. Seven VL53L0X sensors share one address, so they need
a multiplexer and sequenced reads, which is fiddly work that does not belong on
the flight controller. The ESP32 does it and emits standard `OBSTACLE_DISTANCE`
and `DISTANCE_SENSOR` as component 195, so ArduPilot's existing avoidance code
consumes them unmodified. No firmware fork.

**What if the obstacle module dies in flight?**
Enabling the proximity backend makes the ESP32 an **arming dependency**, so a
dead module is caught on the ground rather than in the air. Field recovery at
the flying site is `PRX1_TYPE=0` and reboot, which flies without the ring.

---

## 4. Process

**How much of this did AI write?**
Substantially, for code and documentation; the architecture, all decisions, all
assembly, and all testing and debugging were the team's. Full scope, including
where the assistant was wrong, in `08_ai_authorship_disclosure.md`.

**Is this legal?**
It is a student prototype flown in controlled test conditions, dropping inert
salt rather than a pesticide. Digital Sky registration was not completed, which
is stated openly. `05_compliance_narrative.md` sets out what lawful deployment
would require and rates its own confidence in each claim.

**What would you do with more time?**
Solve vibration properly, fly the full loop, and run a real survey across
several days so that the persistence-based stagnation claim is demonstrated
rather than described. That last item is the honest gap between what the system
does and what the concept promises.

**What was the hardest part?**
Answered personally by each team member, from experience rather than from this
document. Candidate material: the prop-nut handedness discovery, the ground
station silently dropping parameter writes, or the flight where the altitude
estimate lied.
