# MonsoonReady — shooting script (Saturday 2026-08-15, one continuous take)

Rules the take must obey: 5-10 minutes, NO cuts, publicly viewable later,
OPENS with a Google search for today's date on a screen, must show
components + assembly + functionality. Two people, one phone (Vivo X300 FE,
landscape, held horizontal).

Roles as written: **R = Reyansh** (narrates, flies/starts the mission),
**C = camera** (Raghav; follows, keeps subject in frame, NEVER stops
recording). Swap freely, but decide before rolling.

Print this. Each person carries a copy. Rehearse the WALK once without
recording: the take fails on forgetting and on walking, not on flying.

## Stage BEFORE pressing record (nothing may need setup mid-take)

- [ ] Laptop on the table (verandah or car bonnet), AWAKE, sleep disabled,
      brightness max, in shade, with TWO things ready:
      1. a browser tab already at google.com
      2. the detector ready to run over the tray / saved frames
- [ ] Dashboard (drone.reysen.net) open on the hotspot phone or laptop
- [ ] Aircraft on the weighted cloth sheet, pack in, hopper LOADED with
      salt, gate CLOSED, props tight (pliers), SD in
- [ ] Tray placed on the survey path, filled from the pump, water calm
- [ ] Transmitter ON, GPS already at 3D fix (do the 5-min wait BEFORE
      rolling, not during)
- [ ] Rehearsal flight already flown and log checked
- [ ] Phone: do-not-disturb ON (a call kills the take), storage checked

## The take (~8 min; times are targets, not alarms)

**0:00-0:30 — Date proof (disqualification criterion, get it clean).**
C films the laptop screen. R types "today's date" into Google, waits for the
result to render, holds 3 seconds.
> R: "Today is Saturday, the 15th of August 2026 — there it is on Google.
> I'm Reyansh, that's Raghav on camera, and this is MonsoonReady, our entry
> for the Arduino Physical AI Challenge. It's a drone that finds the
> stagnant-water puddles where dengue and malaria mosquitoes breed, and
> treats them from the air."

**0:30-2:15 — Components (C walks the airframe slowly, close-ups on each
part as R names it).**
> R: "The airframe is an F550 hexacopter. Flight control is a Pixhawk
> running ArduPilot — GPS and compass on the mast, telemetry radio here.
> Under the belly: an Arduino UNO Q, the companion computer. It runs Linux,
> and it runs our puddle-detection model on board — nothing is offloaded to
> a phone or the cloud. Next to it, the camera the model sees through, and
> a TF-Luna laser rangefinder measuring height above the ground.
> Around the frame: six time-of-flight distance sensors on an ESP32 — an
> obstacle ring that feeds ArduPilot's avoidance.
> And this is the treatment payload: a hopper we built, with a servo gate.
> In deployment it drops Bti, a larvicide that kills mosquito larvae and
> nothing else. Today it drops plain salt, because we're on a working farm."

**2:15-3:00 — Assembly evidence (C on the wiring bay, R points).**
> R: "We built this, so here's the inside: the power bay — one buck
> converter powers the Arduino, a separate one powers the servo and the
> sensor ring, so a stalled servo can never brown out the computer. The
> sensor ring is hand-soldered to a central hub. The hopper is our own
> box-and-gate design. Bought: frame, flight controller, motors. Made:
> everything that makes it MonsoonReady."

**3:00-4:00 — The AI, on the laptop (C films the screen; R runs it).**
> R: "The brain: a YOLO26-nano network we trained ourselves on about
> twenty-two thousand images of puddles and stagnant water. On the
> Arduino's own CPU it runs an image in about half a second, which is fine,
> because the aircraft photographs while it flies."
R runs the detector over the tray (live) or saved field frames; boxes
appear.
> R: "There's the detection — that box is what the aircraft acts on."

**4:00-6:30 — The flight (C steps back, keeps aircraft AND tray in frame;
R at the laptop/transmitter).**

VARIANT A — autonomous loop (only if the Linux->Pixhawk link passed at
home):
> R: "Now the whole loop, autonomously. I'm starting the mission — from
> here the aircraft flies the survey, finds the water, and treats it. I'm
> holding the transmitter only as a safety pilot."
R starts the mission. Narrate sparsely, let it happen:
> R (as it flies): "It's surveying... that pause means the detector fired —
> it's fixing the puddle's GPS position... now it comes in beside the
> water, low... and that's the drop." 
> R (after RTL/land): "Landed itself. Every detection it made is logged
> with coordinates — that's the map on the dashboard."

VARIANT B — piloted fallback (if the link didn't make it):
> R: "Full disclosure first: the detection you saw runs on the drone, but
> the command link that closes the loop isn't flight-ready today, so this
> flight is me flying, with the detector recording what it sees. The drop
> is commanded manually."
Fly the same survey line over the tray, trigger the drop over/beside it,
land.

**6:30-8:00 — Result and honesty (C follows R to the tray, then the
laptop).**
R walks to the tray, shows the salt in/around the water.
> R: "Treated. In deployment that's Bti doing this to a real breeding
> site."
C pans to the laptop/dashboard.
> R: "The mission log: every fix, every detection with its GPS coordinates,
> every drop, live on the base station."
Closing, straight to camera:
> R (variant A): "Everything you saw — survey, detection, drop — happened
> on the aircraft: the model on the Arduino, the flying on the Pixhawk. We
> flew it, filmed it, and showed you exactly what it did. Thanks for
> watching."
> R (variant B): "What's real today: on-board detection, GPS-logged
> sightings, and the treatment mechanism. What's next: the command link
> that closes the loop without a pilot. We'd rather show you exactly where
> it stands. Thanks for watching."
C holds the aircraft in frame 3 seconds, then stops recording.

## Immediately after stopping

- [ ] WATCH THE TAKE start to end before anything is torn down (audio
      audible? date legible? drop visible?). If it fails, that's what
      attempts 2-3 are for.
- [ ] Copy the file to a laptop RIGHT THERE. Second copy to Drive tonight.

## Salvage rules mid-take (no cuts allowed, so recover IN the take)

- Fluffed a line: pause, breathe, say it again. Judges accept humans.
- Aircraft refuses a step: R narrates what it should have done, C keeps
  rolling, land it, state plainly it will be retried — then decide on the
  ground whether this take stands or the next attempt starts.
- NEVER fake a result on camera. An overclaim they catch is fatal; a
  limitation stated plainly scores.
