# MonsoonReady

An F550 hexacopter that finds standing water after monsoon rain and drops
granular Bti larvicide into it. The YOLO model runs on an Arduino UNO Q bolted
to the aircraft, so it decides in the air. No ground station. No cloud.

Arduino Physical AI Challenge India 2026. Built by Reyansh and Raghav.

## Why granules

Aedes aegypti lays eggs in small pools: flat roofs, building sites, blocked
drains, tarpaulins, an old tank nobody emptied. Larvae are stuck in the water
until they hatch, which makes them the easy thing to kill. Council crews
already do this on foot. It is slow, and they only treat what they can walk to.

Granular Bti has one requirement, which is landing in the water. No tank. No
pump, no nozzle, nothing to drift downwind. What is left is a much smaller
problem: find the water, get over it, drop a measured amount.

## The loop

Pixhawk flies the survey rows at 15 m while the UNO Q takes downward photos and
runs them through the model. Water in frame means the coordinates get locked
right then, at altitude. The aircraft repositions beside the water, comes down
on the TF-Luna, crosses over the middle, opens the gate, crosses back. Climb,
next row.

It descends beside the puddle rather than over it because the TF-Luna uses
850 nm infrared and still water at that wavelength acts like a mirror. Dry
ground three metres to the side gives an honest height. The aircraft holds that
height across the water.

Once it lands the UNO Q stops being a mission computer and turns into a web
dashboard: where it flew, what it found, what it treated.

Test flights drop mustard seed. Not larvicide.

## What we claim

The model finds standing water. That is the whole claim.

It cannot tell you the water has been there long enough to breed anything.
One photo does not carry that information, and from 15 m this morning's puddle
and a two-week-old breeding site are the same handful of pixels.

Stagnation has to come from repeat visits. Fly the area again on a different
day, see which pools are still sitting there, then have someone confirm on the
ground. A single flight produces candidates and nothing more.

## Hardware

| Part | Why this one |
|------|--------------|
| F550 hexacopter | Six motors carry the payload and survive losing one. Replaced the S550 we destroyed. |
| Pixhawk 2.4.8, ArduCopter 4.7.0 | Guided-mode MAVLink is mature, and the logs are good enough to work out what went wrong. |
| 6x A2212 920KV, 1045 props | Swapped from EMAX MT2213 when we could not get matching props in India. |
| 6x 45A BLHeli_32 ESCs | Rated well above what the motors pull, so they stay cool. |
| 3S 8000mAh LiPo | One pack. It lives on the drone. |
| Arduino UNO Q, 4GB | Runs the detector. The whole point of the project. |
| Logitech B525, 720p | We already owned it and UVC works on the UNO Q today. That beat any spec gain from buying something. |
| TF-Luna, downward | ArduPilot supports it natively. Gives height above the ground and puddle size for the dose. |
| VL53L0X ring on a TCA9548A, read by an ESP32 | Cheapest proximity ring the flight controller understands without a firmware fork. |
| MG90 metal-gear servo | Opens a gate on a tube. Metal gears because a stripped nylon gear means no drop. |
| SH1106 OLED | Prearm status, sats, mode, battery, readable in the field without a laptop. |
| SiK 433 MHz | Ground monitoring during tests. |
| FlySky FS-i6X | Ten channels, with arm and kill on their own switches. |

### Wiring

```
Pixhawk 2.4.8
  USB      UNO Q companion, MAVLink2
  TELEM1   ESP32 obstacle ring, MAVLink2 115200, component 195
  TELEM2   SiK 433 MHz radio
  SERIAL3  NEO-M8N GPS and compass
  SERIAL5  TF-Luna, 115200
```

The ESP32 sends plain `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` messages, so
ArduPilot's own avoidance reads them with nothing modified. The UNO Q commands
the aircraft as component 191 using standard guided-mode messages.

Four of the six ring positions work. One never got a sensor, there is no room
on that side of the frame. One has a sensor that has never answered. Both are
marked absent in the firmware, which matters more than it sounds: while they
were still marked present, the ESP32 retried them every five seconds and each
retry blocked its main loop for about a second. That gap was long enough for
ArduPilot to declare the proximity sensor dead and refuse to arm, on roughly a
fifth of our attempts.

## The model

One class, `puddle`. Every source dataset gets collapsed to it. Multi-class
sets keep only the water classes and everything else gets dropped, and an image
that ends up with no boxes stays in as a negative, which is free hard-negative
data.

We deliberately do not keep `pool` or `water tank`. They really are breeding
sites, but they are not things this aircraft should drop into, and teaching the
model to find them would only mean writing code to ignore them later.

`yolo26n` at 640 px, trained on the RTX 3050. We tried `yolov8n` first. The
attention models from v12 onward are too slow on an A53 CPU to be worth it, and
cloud inference is not an option for a drone over a building site with no
signal.

Two runs:

| | Precision | Recall | mAP50 | mAP50-95 |
|--|--|--|--|--|
| Run 1, `yolov8n`, 11.7k images | 0.744 | 0.687 | 0.725 | 0.431 |
| Run 2, `yolo26n`, 21.7k images | 0.795 | 0.708 | 0.766 | 0.467 |

Both scored on the same v2 validation set, which is the only fair comparison.
Run 2 wins on everything with fewer parameters. On the UNO Q it runs at roughly
490 ms per frame, and the laptop and the board produce identical predictions on
the same 24 images, down to the confidence value.

Training augmentation includes 180 degree rotation and vertical flip, because a
photo taken straight down has no correct way up.

Three failure modes, found by looking at run-1 predictions one image at a time
rather than by reading the metrics:

- Sheet water, a thin film with no puddle-shaped outline. Answer: our own
  photos taken from survey height.
- Glare, where the sun bounces off the surface. Same answer, shot at different
  times of day.
- Close range, where the water fills most of the frame. That one changed the
  architecture instead of the dataset.

## Mission logic

The parts worth knowing about, each of which exists because something went
wrong or was going to.

**The target gets locked at altitude.** First detection from 15 m sets the
coordinates and the aircraft stops looking after that. Close-range frames are
exactly where the model is least reliable, so a design that kept re-detecting
would hand steering to its worst input.

**Descents abort upward.** The rule separates the cases instead of treating
every missing reading as a fault:

| Situation | What it means | What happens |
|--|--|--|
| No reading, still high | Out of sensor range. Normal. | Keep descending |
| No reading, below 6 m | Should be seeing ground | Abort upward |
| Had a reading, lost it | Dropout | Abort upward |
| EKF says below drop height, rangefinder never confirmed | The two disagree | Abort upward |
| Good reading at drop height | Confirmed | Drop |

A missed puddle costs nothing. A blind descent costs the aircraft.

**The pilot always wins.** Any mode change away from guided and the mission
stands down and stops sending commands. It never fights the sticks.

**Nothing waits forever.** Takeoff, each survey leg and each approach have
timeouts. Guided position targets are not acknowledged by ArduPilot, so a
refused destination looks exactly like one the aircraft is still flying to, and
without a timeout the mission would sit there until the battery failsafe.

**Detections outside the geofence are thrown away.** The camera sees about
eight metres either side of the aircraft and the survey rows sit only four
metres inside the boundary, so water at the edge of frame can be outside the
fence entirely. Flying there would either be refused silently or trigger a
fence breach and an RTL.

**Bad GPS means no latch.** Above 1.5 HDOP or below 8 satellites, detections
are ignored. We measured the position wandering ten metres on a bad day, which
is wider than the puddle.

## Simulation

`uno_q/sitl_test.py` runs two scenarios against ArduPilot SITL:

| Scenario | Pass | Result |
|--|--|--|
| Nominal | One drop, survey finishes, RTL, no abort | Pass, dropped at 2.98 m |
| Rangefinder dropout | Zero drops, aborts upward, survey still finishes | Pass |

## Where it stands

Working: the airframe, the ring, the dashboard, parameter management, the
detector on the board, the geofence, route generation from the fence shape, and
the whole mission loop in simulation.

Not done: the full autonomous loop has not flown yet. We have had the aircraft
armed and the mission commanded, but between GPS quality, a fence drawn too
tight around the take-off spot and the proximity refusals above, we have not
got a complete automatic flight on video.

The dose in grams per second is also unmeasured. The hopper bridges with fine
salt, which turned out to be cohesion between hundred-micron grains rather than
the hole being too small, so we changed the test material to mustard seed
instead of enlarging the hole.

## Reproducing it

One git repository. Every decision is in a dated, author-tagged, append-only
log in `PROJECT_STATE.md`, because two people on two machines built this.

| Task | Command |
|--|--|
| Merge datasets | `./python training/merge_datasets.py` |
| Train | `./python training/train.py` |
| Export ONNX | `./python training/export.py` |
| Push parameters | `./python tools/parameters.py push` |
| Mission tests | `./python uno_q/sitl_test.py` |

We do not use bulk parameter load from a ground station. It drops writes
silently. `tools/parameters.py` acknowledges every single write instead.
