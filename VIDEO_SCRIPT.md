# MonsoonReady, shooting script

One continuous take. The rules it has to obey: five to ten minutes, no cuts,
publicly viewable afterwards, opens with a Google search for today's date on a
screen, and it has to show the components, the assembly and the thing working.

Two people, one phone (Vivo X300 FE, landscape, held horizontal).

R is Reyansh, narrating and flying. C is Raghav on camera, following, keeping
the subject in frame, never stopping the recording. Swap if you like, but decide
before rolling.

The first attempt was 2026-08-15 at the farm and the camera died on the board.
The second field day was 2026-08-21 and it never armed. This one is at the field
near the house.

Print this. One copy each. Rehearse the walk once without recording, because
takes fail on forgetting a line and on walking, not on flying.

## Before pressing record

Nothing may need setting up mid-take.

- [ ] Laptop on the table, awake, sleep disabled, brightness up, in shade, with
      a browser tab already at google.com and the detector ready to run
- [ ] Dashboard open on the hotspot phone or the laptop
- [ ] Aircraft on the weighted sheet, pack in, hopper loaded with mustard seed,
      gate closed, props tight with pliers, SD card in
- [ ] Tray on the survey path, filled, water calm
- [ ] Transmitter on, GPS already at a 3D fix. Do the five-minute wait before
      rolling, not during.
- [ ] Rehearsal flight flown and its log checked
- [ ] Phone on do-not-disturb. A call kills the take. Check storage.

## The take, about eight minutes

Times are targets, not alarms.

**0:00 to 0:30, date proof.** This is a disqualification criterion, so get it
clean. C films the laptop screen while R types "today's date" into Google, waits
for it to render, holds three seconds.

> R: "Today is [date], there it is on Google. I'm Reyansh, that's Raghav on
> camera, and this is MonsoonReady, our entry for the Arduino Physical AI
> Challenge. It's a drone that finds the standing water where dengue and malaria
> mosquitoes breed, and treats it from the air."

**0:30 to 2:15, components.** C walks the airframe slowly, close on each part as
R names it.

> R: "The airframe is an F550 hexacopter. Flight control is a Pixhawk running
> ArduPilot, GPS and compass up on the mast, telemetry radio here. Under the
> belly: an Arduino UNO Q, the companion computer. It runs Linux, and it runs
> our puddle-detection model on board. Nothing is offloaded to a phone or the
> cloud. Next to it, the camera the model sees through, and a TF-Luna laser
> rangefinder measuring height above the ground. Around the frame, time-of-
> flight distance sensors on an ESP32, an obstacle ring feeding ArduPilot's own
> avoidance. And this is the treatment payload: a hopper we built, with a servo
> gate. In deployment it drops Bti, a larvicide that kills mosquito larvae and
> nothing else. Today it drops mustard seed, because this is a test flight and
> Bti is a real pesticide."

**2:15 to 3:00, assembly.** C on the wiring bay, R pointing.

> R: "We built this, so here's the inside. The power bay: one buck converter
> powers the Arduino, a separate one powers the servo and the sensor ring, so a
> stalled servo can never brown out the computer. The sensor ring is
> hand-soldered to a central hub. The hopper is our own box-and-gate design.
> Bought: frame, flight controller, motors. Made: everything that makes it
> MonsoonReady."

**3:00 to 4:00, the AI.** C films the laptop screen, R runs it.

> R: "The brain is a YOLO26-nano network we trained ourselves, on about
> twenty-two thousand images of puddles and standing water. On the Arduino's own
> CPU it runs an image in about half a second, which is fine, because the
> aircraft photographs while it hovers rather than processing video."

R runs the detector over the tray, or over saved field frames. Boxes appear.

> R: "There's the detection. That box is what the aircraft acts on."

**4:00 to 6:30, the flight.** C steps back and keeps both the aircraft and the
tray in frame. R at the laptop and transmitter.

Variant A, the autonomous loop, only if the link passed at home:

> R: "Now the whole loop, autonomously. I'm starting the mission. From here the
> aircraft flies the survey, finds the water, and treats it. I'm holding the
> transmitter only as a safety pilot."

R starts the mission. Narrate sparsely and let it happen.

> R, as it flies: "It's surveying. That pause means the detector fired, it's
> fixing the puddle's GPS position. Now it comes in beside the water, low. And
> that's the drop."
>
> R, after it lands: "Landed itself. Every detection it made is logged with
> coordinates, and that's the map on the dashboard."

Variant B, piloted fallback, if the link did not make it:

> R: "Full disclosure first. The detection you just saw runs on the drone, but
> the command link that closes the loop isn't flight-ready today, so this flight
> is me flying, with the detector recording what it sees. The drop is commanded
> manually."

Fly the same survey line over the tray, trigger the drop beside it, land.

**6:30 to 8:00, result and honesty.** C follows R to the tray, then to the
laptop. R shows the seed in and around the water.

> R: "Treated. In deployment that's Bti doing this to a real breeding site."

C pans to the dashboard.

> R: "The mission log. Every fix, every detection with its GPS coordinates,
> every drop, live on the base station."

Closing, straight to camera:

> R, variant A: "Everything you saw, survey, detection, drop, happened on the
> aircraft. The model on the Arduino, the flying on the Pixhawk. We flew it,
> filmed it, and showed you exactly what it did. Thanks for watching."
>
> R, variant B: "What's real today: onboard detection, GPS-logged sightings, and
> the treatment mechanism. What's next is the command link that closes the loop
> without a pilot. We'd rather show you exactly where it stands. Thanks for
> watching."

C holds the aircraft in frame for three seconds, then stops recording.

## Straight after stopping

- [ ] Watch the take start to end before anything is torn down. Audio audible?
      Date legible? Drop visible? If it failed, that is what the next attempt is
      for.
- [ ] Copy the file to a laptop right there. Second copy to Drive tonight.

## Salvage rules, mid-take

No cuts are allowed, so recovery has to happen inside the take.

Fluffed a line: pause, breathe, say it again. Judges accept humans.

The aircraft refuses a step: R narrates what it should have done, C keeps
rolling, land it, say plainly that it will be retried. Decide on the ground
whether this take stands or the next attempt starts.

Never fake a result on camera. An overclaim they catch is fatal. A limitation
stated plainly scores.
