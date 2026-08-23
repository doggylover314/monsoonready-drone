# MonsoonReady, shooting script

One take, five to ten minutes, no cuts. It opens with today's date on a screen,
covers the components and the assembly, and shows the thing working. Goes public
afterwards.

Two people, one phone held landscape. R is Reyansh, C is Raghav. Both narrate,
roughly half each. Whoever is not speaking holds the camera or works the
hardware. The recording never stops.

Every block below belongs to one person start to finish. Nobody talks over
anybody, and nobody finishes somebody else's paragraph.

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

**0:00, the date.** C films the screen while R types "today's date" into Google
and waits for it to render. Hold three seconds. This is a disqualification
criterion, so get it clean.

**R:** "Today is [date], there it is on Google. I'm Reyansh, that's Raghav
behind the camera, and this is MonsoonReady, our entry for the Arduino Physical
AI Challenge. It's a drone that finds the standing water where dengue and
malaria mosquitoes breed, and treats it from the air."

**0:30, the airframe.** R walks the aircraft slowly and points at each part. C
films close and talks.

**C:** "We're looking at an F550 hexacopter. Six motors, six ESCs, and it lifts
about a kilo of payload. Flight control is a Pixhawk running ArduPilot, GPS up
on the mast where it's clear of the electronics, telemetry radio here, and the
power module that reports what the battery is doing. All of that is standard
hardware. What makes it MonsoonReady is underneath."

**1:15, the sensing.** They swap. C films, R points and talks.

**R:** "Under the belly is an Arduino UNO Q, the companion computer. It runs
Linux, and it runs our puddle-detection model on board. Nothing goes to a phone
or the cloud. Next to it, the camera the model sees through, and a TF-Luna laser
measuring height above the ground. Around the frame, time-of-flight sensors on
an ESP32, an obstacle ring feeding distances to ArduPilot. Right now it only
reports them. Bright daylight makes those sensors see obstacles that aren't
there, so we've taken the ring off the flight controls until we fix it."

**2:00, the payload.** R holds the hopper open to camera. C keeps filming and
picks up the line.

**C:** "And underneath all of that is the part that does the actual work. A
hopper we built, with a servo gate under it. In deployment it drops Bti, a
larvicide that kills mosquito larvae and nothing else. Today it drops mustard
seed, because Bti is a real pesticide and this is a test flight."

**2:30, the assembly.** C on the wiring bay, R holding the frame steady.

**C:** "We built this, so here's the inside. One buck converter powers the
Arduino, a separate one powers the servo and the sensor ring, so a stalled servo
can never brown out the computer. The ring is hand-soldered to a central hub.
The hopper is our own design. Bought: frame, flight controller, motors. Made:
everything that makes it MonsoonReady."

**3:15, the AI.** C films the laptop screen, R runs it.

**R:** "The brain is a YOLO26-nano network we trained ourselves on about
twenty-six thousand images of standing water. On the Arduino's own CPU it runs
an image in about half a second. That's fine, because the aircraft photographs
while it hovers rather than processing video."

Boxes appear.

**C:** "Detection. Every box the model draws gets a ground coordinate, and that
coordinate is what the aircraft flies to."

**4:00, the flight.** C steps back and keeps the aircraft and the tray both in
frame. R has the transmitter.

Variant A, autonomous:

**R:** "Now the whole loop. I'm starting the mission, and from here the aircraft
flies the survey, finds the water and treats it. I'm holding the transmitter
only as a safety pilot."

Narrate sparsely. Let it happen. C carries the flight, because R is flying.

**C:** "It's surveying. That pause means the detector fired, it's fixing the
puddle's position. Now it comes in beside the water, low. And that's the drop."

**R, after landing:** "Landed itself. Every detection is logged with
coordinates, and that's the map on the dashboard."

Variant B, piloted:

**R, variant B:** "Full disclosure first. The detection you just saw runs on the
drone, but the link that closes the loop isn't flight-ready today, so this
flight is me flying, with the detector recording what it sees. I'm commanding
the drop by hand."

Fly the same line over the tray, drop beside it, land.

**6:30, the result.** R walks to the tray and shows the seed in the water. C
films it, then pans to the dashboard.

**C:** "Treated. In deployment that's Bti doing this to a real breeding site.
And here's the mission log: every detection with its coordinates, every drop,
live on the base station."

Closing, both straight to camera. R first, then C.

**R, variant A:** "Everything you saw, survey, detection, drop, happened on the
aircraft. The model on the Arduino, the flying on the Pixhawk."

**C, variant A:** "We flew it and filmed it, and what you saw is what it did.
Thanks for watching."

**R, variant B:** "Real today: onboard detection, GPS-logged sightings, and a
treatment mechanism that works. Still to come is closing the loop without a
pilot."

**C, variant B:** "We'd rather show you where it actually stands. Thanks for
watching."

C holds the aircraft in frame three seconds, then stops.

## Straight after

- [ ] Watch it back before anything is packed. Audio audible, date legible, drop
      visible.
- [ ] Copy the file to a laptop right there. Second copy to Drive tonight.

## Recovering mid-take

No cuts, so recovery happens inside the take.

Fluffed a line: pause, breathe, say it again. Judges accept humans.

The aircraft refuses a step: whoever is not filming says what it should have
done, C keeps rolling, land it, say plainly that it will be retried. Decide on
the ground whether this take stands.

The battery failsafe interrupts the mission: the aircraft breaks off, climbs to
15 metres and flies home by itself. Let it. Do not switch out of RTL, because
taking manual control on camera makes a working safeguard look like a fault. It
fired inside two minutes on the last flight, so this is the likeliest
interruption today. C keeps the aircraft in frame through the landing while R
names it as it happens.

**R, if it fires:** "That's the battery failsafe. It's set to break off the
survey and bring the aircraft home on its own, and that's what it's doing now.
We'll fly the rest of the run once the pack is charged."

Never fake a result on camera. Judges who catch an overclaim stop believing the
rest of it, and a limitation you state yourself costs you nothing.
