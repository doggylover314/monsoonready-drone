# MonsoonReady, shooting script

One take, five to ten minutes, no cuts. It opens with today's date on a screen,
covers the components and the assembly, and shows the thing working. Goes public
afterwards.

Three people. Reyansh and Raghav are both on camera the whole time, standing
either side of the aircraft. A third person films, holding the phone landscape,
and never stops the recording. Camera directions below are for them.

Reyansh and Raghav narrate roughly half each. Every block belongs to one person
start to finish. Nobody talks over anybody, and nobody finishes somebody else's
paragraph. Whoever is not speaking handles the hardware being pointed at.

Print three copies. Walk it through once without recording. Takes fail on a
forgotten line or a wander out of frame; the flying is the part already known.

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
- [ ] Camera operator has read the shot directions and knows the running order

## The take

Times are targets, not alarms.

**0:00, the date.** Camera on the laptop screen while Reyansh types "today's
date" into Google. Hold three seconds, then widen to both of them. This is a
disqualification criterion, so get it clean.

**Reyansh:** "Today is [date], there it is on Google. I'm Reyansh, this is
Raghav, and this is MonsoonReady, our entry for the Arduino Physical AI
Challenge. It's a drone that finds the standing water where dengue and malaria
mosquitoes breed, and treats it from the air."

**0:30, the airframe.** Both at the aircraft. Reyansh points at each part as
Raghav names it. Camera close on the parts, then wide enough to keep both in
shot.

**Raghav:** "We're looking at an F550 hexacopter. Six motors, six ESCs, and it
lifts about a kilo of payload. Flight control is a Pixhawk running ArduPilot,
GPS up on the mast where it's clear of the electronics, telemetry radio here,
and the power module that reports what the battery is doing. All of that is
standard hardware. What makes it MonsoonReady is underneath."

**1:15, the sensing.** They swap roles without moving. Raghav points, Reyansh
talks. Camera low and close under the belly.

**Reyansh:** "Under the belly is the Arduino UNO Q. It runs our puddle-detection
model on board, and nothing goes to a phone or the cloud. Next to it, the camera
the model sees through, and a TF-Luna laser measuring height above the ground.
Around the frame, time-of-flight sensors on an ESP32, an obstacle ring feeding
distances to ArduPilot. Right now it only reports them. Bright daylight makes
those sensors see obstacles that aren't there, so we've taken the ring off the
flight controls until we fix it. Every one of those we soldered and mounted
ourselves."

**2:00, the payload.** Reyansh holds the hopper open to camera. Raghav talks.

**Raghav:** "And underneath all of that is the part that does the actual work. A
hopper we built, with a servo gate under it. In deployment it drops Bti, a
larvicide that kills mosquito larvae and nothing else. Today it drops mustard
seed, because Bti is a real pesticide and this is a test flight."

**2:30, the assembly.** Camera on the wiring bay. Reyansh holds the frame steady
and clear of the shot.

**Raghav:** "We built this, so here's the inside. One buck converter powers the
Arduino, a separate one powers the servo and the sensor ring, so a stalled servo
can never brown out the computer. The ring is hand-soldered to a central hub.
The hopper is our own design. Bought: frame, flight controller, motors. Made:
everything that makes it MonsoonReady."

**3:15, the AI.** Both at the laptop. The aircraft has not flown yet, so this is
the model run on a still from an earlier flight. Reyansh starts it. Camera over
their shoulders, screen legible.

**Reyansh:** "The brain is a YOLO26-nano network we trained ourselves on about
twenty-six thousand images of standing water. On the Arduino's own CPU it runs
an image in about half a second. That's fine, because the aircraft photographs
while it hovers rather than processing video."

Boxes appear on the laptop screen.

**Raghav:** "That's the model finding standing water in a still from an earlier
flight, same weights that run on the Arduino. Each box becomes a ground
coordinate for the aircraft to fly to."

**4:00, the flight.** Camera steps back far enough to hold the aircraft, the
tray and both of them at once. Reyansh has the transmitter.

Variant A, autonomous:

**Reyansh:** "Now the whole loop. I'm starting the mission, and from here the
aircraft flies the survey, finds the water and treats it. I'm holding the
transmitter only as a safety pilot."

Narrate sparsely. Let it happen. Raghav carries the commentary, because Reyansh
is flying. Camera favours the aircraft and keeps both of them in the edge of
frame.

**Raghav:** "It's surveying. That pause means the detector fired, it's fixing
the puddle's position. Now it comes in beside the water, low. And that's the
drop."

**Reyansh, after landing:** "Landed itself. Every detection is logged with
coordinates, and that's the map on the dashboard."

Variant B, piloted:

**Reyansh, variant B:** "Full disclosure first. The detection you just saw runs
on the drone, but the link that closes the loop isn't flight-ready today, so
this flight is me flying, with the detector recording what it sees. I'm
commanding the drop by hand."

Fly the same line over the tray, drop beside it, land.

**6:30, the result.** Both walk to the tray. Reyansh shows the seed in the
water. Camera follows them in, close on the tray, then pans to the dashboard.

**Raghav:** "Treated. In deployment that's Bti on a real breeding site. And
here's the mission log: every detection, every drop, live on the base station."

Closing. Both straight to camera, side by side, aircraft between them. Reyansh
first, then Raghav.

**Reyansh, variant A:** "Everything you saw, survey, detection, drop, happened
on the aircraft. The model on the Arduino, the flying on the Pixhawk."

**Raghav, variant A:** "We flew it and filmed it, and what you saw is what it
did. Thanks for watching."

**Reyansh, variant B:** "Real today: onboard detection, GPS-logged sightings,
and a treatment mechanism that works. Still to come is closing the loop without
a pilot."

**Raghav, variant B:** "We'd rather show you where it actually stands. Thanks
for watching."

Camera holds the two of them and the aircraft for three seconds, then stops.

## Straight after

- [ ] Watch it back before anything is packed. Audio audible, date legible, drop
      visible, both of them in frame throughout.
- [ ] Copy the file off the camera operator's phone right there. Second copy to
      Drive tonight.

## Recovering mid-take

No cuts, so recovery happens inside the take.

Fluffed a line: pause, breathe, say it again. Judges accept humans.

The aircraft refuses a step: whoever is not flying says what it should have
done, camera keeps rolling, land it, say plainly that it will be retried. Decide
on the ground whether this take stands.

The battery failsafe interrupts the mission: the aircraft breaks off, climbs to
15 metres and flies home by itself. Let it. Do not switch out of RTL, because
taking manual control on camera makes a working safeguard look like a fault. It
fired inside two minutes on the last flight, so this is the likeliest
interruption today. Camera keeps the aircraft in frame through the landing while
Reyansh names it as it happens.

**Reyansh, if it fires:** "That's the battery failsafe. It's set to break off
the survey and bring the aircraft home on its own, and that's what it's doing
now. We'll fly the rest of the run once the pack is charged."

Never fake a result on camera. Judges who catch an overclaim stop believing the
rest of it, and a limitation you state yourself costs you nothing.
