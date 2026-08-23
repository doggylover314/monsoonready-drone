# MonsoonReady

An F550 hexacopter that finds standing water after monsoon rain and drops
granular Bti larvicide into it. The YOLO model runs on an Arduino UNO Q bolted
to the aircraft, so it locates in the air. No ground station. No cloud.

Arduino Physical AI Challenge India 2026. Built by Reyansh and Raghav.

Test flights drop mustard seed. Not larvicide.

## Why granules

Aedes aegypti lays eggs in small pools: flat roofs, building sites, blocked
drains, tarpaulins, an old tank nobody emptied. Larvae are stuck in the water
until they hatch, which makes them the easy thing to kill. Council crews already
do this on foot, slowly, and only where they can walk.

Granular Bti has one requirement, which is landing in the water. No tank, no
pump, no nozzle, nothing to drift downwind. What is left is a much smaller
problem: find the water, get over it, drop a measured amount.

## The loop

Pixhawk flies the survey rows while the UNO Q takes downward photos and runs
them through the model. Survey altitude is a trade and gets set per site: at
15 m a half-metre puddle is 44 pixels wide and the model finds it about one
frame in ten, at 3 m it is 220 pixels. Water in frame means the coordinates get
locked right then, at altitude. The aircraft repositions beside the water, comes down
on the TF-Luna, crosses over the middle, opens the gate, crosses back. Climb,
next row. Once it lands, the UNO Q stops being a mission computer and becomes a
web dashboard showing where it flew and what it treated.

It descends beside the puddle rather than over it because the TF-Luna uses
850 nm infrared, and still water at that wavelength acts like a mirror. Dry
ground three metres to the side gives an honest height, and the aircraft holds
that height across the water.

## What we claim

The model finds standing water. That is the whole claim.

It cannot tell you the water has been there long enough to breed anything. From
survey altitude, this morning's puddle and a two-week-old breeding site are the
same handful of pixels. Stagnation has to come from repeat visits: fly again on
another day, see which pools are still there, then have someone confirm on the
ground. One flight produces candidates and nothing more.

## Hardware

| Part | Why this one |
|------|--------------|
| F550 hexacopter | Carries the payload and survives losing a motor. Replaced the S550 we destroyed. |
| Pixhawk 2.4.8, ArduCopter 4.7.0 | Mature guided-mode MAVLink, and logs good enough to work out what went wrong. |
| 6x A2212 920KV, 1045 props, 45A BLHeli_32 ESCs | Motors swapped from EMAX MT2213 when we could not get matching props in India. ESCs rated well above what the motors pull. |
| 3S 8000mAh LiPo | One pack. It lives on the drone. |
| Arduino UNO Q, 4GB | Runs the detector. The whole point of the project. |
| Logitech B525, 720p | Already owned, and UVC works on the UNO Q today. That beat any spec gain from buying something. |
| TF-Luna, downward | Native ArduPilot support. Gives height and puddle size for the dose. |
| VL53L0X ring on a TCA9548A, read by an ESP32 | Cheapest proximity ring the flight controller understands without a firmware fork. |
| MG90 metal-gear servo | Opens a gate on a tube. Metal gears because a stripped nylon gear means no drop. |
| SH1106 OLED, SiK 433 MHz, FlySky FS-i6X | Prearm status without a laptop, ground monitoring, and arm and kill on their own switches. |

```
Pixhawk 2.4.8
  USB      UNO Q companion, MAVLink2
  TELEM1   ESP32 obstacle ring, MAVLink2 115200, component 195
  TELEM2   SiK 433 MHz radio
  SERIAL3  NEO-M8N GPS and compass
  SERIAL4  TF-Luna, 115200
```

The ESP32 sends plain `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR`, so ArduPilot's
own avoidance reads them unmodified. The UNO Q commands the aircraft as
component 191.

Four of the six ring positions work, plus the upward one. Marking the other two
absent in firmware mattered more than it sounds: while they were marked present,
the ESP32 retried them every five seconds and each retry blocked its loop for
about a second, which was long enough for ArduPilot to call the proximity sensor
dead and refuse to arm on roughly a fifth of our attempts.

## The model

One class, `puddle`, and every source dataset gets collapsed to it. An image left
with no boxes stays in as a negative, which is free hard-negative data. We
deliberately drop `pool` and `water tank`: real breeding sites, but not things
this aircraft should drop into, and teaching the model to find them would only
mean writing code to ignore them later.

`yolo26n` at 640 px, trained on the RTX 3050. We tried `yolov8n` first. The
attention models from v12 onward are too slow on an A53, and cloud inference is
not an option over a building site with no signal.

| | Precision | Recall | mAP50 | mAP50-95 |
|--|--|--|--|--|
| Run 1, `yolov8n`, 11.7k images | 0.744 | 0.687 | 0.725 | 0.431 |
| Run 2, `yolo26n`, 21.7k images | 0.795 | 0.708 | 0.766 | 0.467 |

Both on the same v2 validation set, the only fair comparison. Run 2 wins
everything with fewer parameters. On the UNO Q it runs at 489 ms a frame, and
the laptop and the board give identical predictions on the same 24 images, down
to the confidence value. Augmentation includes 180 degree rotation and vertical
flip, because a photo taken straight down has no correct way up.

Three failure modes, all found by looking at run-1 predictions one image at a
time rather than by reading metrics: sheet water, meaning a thin film with no
puddle-shaped outline; glare off the surface; and close range, where the water
fills the frame. The first two were answered with our own photos from survey
height. The third changed the architecture instead.

## Mission logic

Each of these exists because something went wrong, or was going to.

**The target gets locked at altitude.** The first detection at survey altitude
sets the coordinates and the aircraft stops looking. Close-range frames are where the
model is least reliable, so re-detecting would hand steering to its worst input.

**Descents abort upward.** The rule separates the cases rather than treating
every missing reading as a fault:

| Situation | What it means | What happens |
|--|--|--|
| No reading, still high | Out of range. Normal. | Keep descending |
| No reading, below 6 m | Should be seeing ground | Abort upward |
| Had a reading, lost it | Dropout | Abort upward |
| EKF below drop height, rangefinder never confirmed | The two disagree | Abort upward |
| Good reading at drop height | Confirmed | Drop |

A missed puddle costs nothing. A blind descent costs the aircraft.

| Rule | Why |
|--|--|
| The pilot always wins. Any mode change away from guided stands the mission down. | It must never fight the sticks. |
| Takeoff, each survey leg and each approach have timeouts. | ArduPilot does not acknowledge guided position targets, so a refused destination looks exactly like one the aircraft is still flying to. Without a timeout the mission sits there until the battery failsafe. |
| Detections outside the geofence are thrown away. | The camera sees about eight metres either side and the rows sit four metres inside the boundary, so water at the edge of frame can be outside the fence. Flying there is either refused silently or breaches and triggers an RTL. |
| No latch above 1.5 HDOP or below 8 satellites. | We measured the position wandering ten metres on a bad day, which is wider than the puddle. |

## The dose

Bti label rates work out at 0.28 to 2.24 g/m², about 1.1 g/m² mid-label. Gate
dwell comes from that and the measured flow.

| Puddle | Bti at 1.1 g/m² | Gate open |
|--|--|--|
| 1 m² | 1.1 g | 0.23 s |
| 2 m² | 2.2 g | 0.46 s |
| 4 m² | 4.4 g | 0.92 s |
| 6 m² | 6.6 g | 1.38 s |

Treat that as a placeholder. The flow rate behind it, 4.8 g/s, is the midpoint
of 4.2 to 5.3 g/s measured at the two shortest dwells, and every one of those
runs came from an under-filled hopper that was starving by the end. A full
hopper has never been measured. The material was also mustard seed, and the gate
passes a volume per second, so grams depend on bulk density, which we have no
verified figure for. The seconds are honest for seed. The grams for Bti are
arithmetic, not measurement.

At 1 m² the dwell is close to the servo's own travel time, so the dose there is
set by how fast the gate moves rather than how long it stays open.

Fine salt was the first test material. It bridged, which turned out to be
cohesion between hundred-micron grains rather than the hole being too small.

## Simulation

`uno_q/sitl_test.py` runs two scenarios against ArduPilot SITL. Nominal: one
drop at 2.98 m, survey finishes, RTL, no abort. Rangefinder dropout: zero drops,
aborts upward, survey still finishes. Both pass.

## Where it stands

Working: the airframe, which clears its vibration gate; the ring; the dashboard;
parameter management; the detector on the board; the geofence and route
generation; and the whole mission loop in simulation.

Not done: the full autonomous loop has never flown. The aircraft has been armed
with the mission commanded, but between GPS quality, a fence drawn too tight
around the take-off spot and the proximity refusals above, there is no complete
automatic flight on video.

## Reproducing it

One repository, and every decision is in a dated, author-tagged, append-only log
in `PROJECT_STATE.md`, because two people on two machines built this. The
commands are in the top-level `README.md`.

One thing worth saying: we do not use bulk parameter load from a ground station,
because it drops writes silently. `tools/parameters.py` acknowledges every one.
