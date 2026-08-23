# MonsoonReady, shooting script

One take, five to ten minutes, no cuts. It opens with today's date on a screen,
covers the components and the assembly, and shows the thing working. Goes public
afterwards.

Two people, one phone held landscape. R is Reyansh, narrating and flying. C is
Raghav on camera, following, never stopping the recording.

Print it. One copy each. Walk it through once without recording. Takes fail on a
forgotten line or a wander into frame; the flying is the part already known.

## Before you press record

Nothing may need setting up mid-take.

- [ ] Laptop awake, sleep off, brightness up, in shade. One tab on google.com,
      detector ready to run.
- [ ] Dashboard open
- [ ] Aircraft on the sheet, pack in, hopper loaded with mustard seed, gate
      closed, props tight, SD in
- [ ] Tray on the survey path, filled, water calm
- [ ] Transmitter on, GPS already at a 3D fix. Do the five-minute wait now, not
      on camera.
- [ ] Test flight already flown and its log checked
- [ ] Phone on do-not-disturb. A call kills the take. Check storage.

## The take

Times are targets, not alarms.

**0:00, the date.** This is a disqualification criterion, so get it clean. C
films the screen while R types "today's date" into Google, waits for it to
render, holds three seconds.

> R: "Today is [date], there it is on Google. I'm Reyansh, that's Raghav on
> camera, and this is MonsoonReady, our entry for the Arduino Physical AI
> Challenge. It's a drone that finds the standing water where dengue and malaria
> mosquitoes breed, and treats it from the air."

**0:30, the components.** C walks the airframe slowly, close on each part as R
names it.

> R: "F550 hexacopter. Flight control is a Pixhawk running ArduPilot, GPS on the
> mast, telemetry radio here. Under the belly is an Arduino UNO Q, the companion
> computer. It runs Linux, and it runs our puddle-detection model on board.
> Nothing goes to a phone or the cloud. Next to it, the camera the model sees
> through, and a TF-Luna laser measuring height above the ground. Around the
> frame, time-of-flight sensors on an ESP32, an obstacle ring feeding distances
> to ArduPilot. Right now it only reports them. Bright daylight makes those
> sensors see obstacles that aren't there, so we've taken the ring off the
> flight controls until we fix it. And this is the payload: a hopper we built,
> with a
> servo gate. In deployment it drops Bti, a larvicide that kills mosquito larvae
> and nothing else. Today it drops mustard seed, because Bti is a real pesticide
> and this is a test flight."

**2:15, the assembly.** C on the wiring bay, R pointing.

> R: "We built this, so here's the inside. One buck converter powers the
> Arduino, a separate one powers the servo and the sensor ring, so a stalled
> servo can never brown out the computer. The ring is hand-soldered to a central
> hub. The hopper is our own design. Bought: frame, flight controller, motors.
> Made: everything that makes it MonsoonReady."

**3:00, the AI.** C films the laptop screen, R runs it.

> R: "The brain is a YOLO26-nano network we trained ourselves on about
> twenty-two thousand images of standing water. On the Arduino's own CPU it runs
> an image in about half a second. That's fine, because the aircraft photographs
> while it hovers rather than processing video."

Boxes appear.

> R: "There's the detection. That box is what the aircraft acts on."

**4:00, the flight.** C steps back and keeps the aircraft and the tray both in
frame.

Variant A, autonomous:

> R: "Now the whole loop. I'm starting the mission, and from here the aircraft
> flies the survey, finds the water and treats it. I'm holding the transmitter
> only as a safety pilot."

Narrate sparsely. Let it happen.

> R: "It's surveying. That pause means the detector fired, it's fixing the
> puddle's position. Now it comes in beside the water, low. And that's the
> drop."
>
> R, after landing: "Landed itself. Every detection is logged with coordinates,
> and that's the map on the dashboard."

Variant B, piloted:

> R: "Full disclosure first. The detection you just saw runs on the drone, but
> the link that closes the loop isn't flight-ready today, so this flight is me
> flying, with the detector recording what it sees. I'm commanding the drop by
> hand."

Fly the same line over the tray, drop beside it, land.

**6:30, the result.** R walks to the tray and shows the seed in the water.

> R: "Treated. In deployment that's Bti doing this to a real breeding site."

C pans to the dashboard.

> R: "The mission log. Every detection with its coordinates, every drop, live on
> the base station."

Closing, straight to camera:

> R, variant A: "Everything you saw, survey, detection, drop, happened on the
> aircraft. The model on the Arduino, the flying on the Pixhawk. We flew it and
> filmed it, and what you saw is what it did. Thanks for watching."
>
> R, variant B: "Real today: onboard detection, GPS-logged sightings, and a
> treatment mechanism that works. Still to come is closing the loop without a
> pilot. We'd rather show you where it actually stands. Thanks for watching."

C holds the aircraft in frame three seconds, then stops.

## Straight after

- [ ] Watch it back before anything is packed. Audio audible, date legible, drop
      visible.
- [ ] Copy the file to a laptop right there. Second copy to Drive tonight.

## Recovering mid-take

No cuts, so recovery happens inside the take.

Fluffed a line: pause, breathe, say it again. Judges accept humans.

The aircraft refuses a step: R says what it should have done, C keeps rolling,
land it, say plainly that it will be retried. Decide on the ground whether this
take stands.

Never fake a result on camera. Judges who catch an overclaim stop believing the
rest of it, and a limitation you state yourself costs you nothing.
