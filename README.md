# MonsoonReady Dragonfly

An F550 hexacopter that finds standing water after monsoon rain and drops
granular Bti larvicide into it. The YOLO model runs on an Arduino UNO Q bolted
to the aircraft, so it locates in the air. No ground station, no cloud.

Arduino Physical AI Challenge India 2026, built by Reyansh and Raghav. Test
flights drop mustard seed, never larvicide.

Read `docs/README.md` for the full writeup: hardware, model, mission logic,
crashes, datasets, legal position and evidence. Read `PROJECT_STATE.md` for
state, decisions and what is left to do.

## What is in here

| Path | What it holds |
|------|---------------|
| `PROJECT_STATE.md` | State, decisions and TODO. The source of truth. |
| `docs/` | The judged documentation, one file. |
| `uno_q/` | Everything that runs on the aircraft: mission state machine, MAVLink IO, detector, dropper, mission log. |
| `uno_q/basestation/` | The Flask dashboard. Map, flight control, fence and route editing. |
| `training/` | Dataset merge, YOLO training, ONNX export. |
| `esp32_obstacle_avoidance/` | Proximity ring firmware. Sends standard MAVLink to the Pixhawk. |
| `tools/` | Parameter push and pull, bench probes, log analysis, ring checks. |
| `param_dumps/` | `pixhawk_full_setup.param`, the hand-maintained config that gets pushed. |
| `Build Log.txt` | Reyansh keeps this by hand. |

IP addresses and accounts live in `PRIVATE.md`, which is gitignored. On a new
machine, copy `PRIVATE.sample.md` and fill it in. Shoot-day working documents
live in `field_ops/`, also gitignored, because they change on the morning of
every field day.

## The hardware, briefly

F550 hexacopter, Pixhawk 2.4.8 on ArduCopter 4.7.0, NEO-M8N GPS on a mast. The
Arduino UNO Q is the companion computer, hanging off the Pixhawk's USB, and it
puts camera stills through ONNX and commands the aircraft in guided mode. A
TF-Luna points down, every descent is gated on it, and losing the return aborts
upward. An ESP32 on TELEM1 reads a ring of VL53L0X sensors and sends the Pixhawk
standard `OBSTACLE_DISTANCE`. Four of the six ring positions work plus the
upward one; one never got a sensor because the frame has no room, and one has
never answered, and marking both absent in firmware is what stopped the arming
failures. SiK 433 MHz sits on TELEM2 and an MG90 metal-gear servo runs the
hopper gate.

## Where it stands

Working: the airframe, which now clears its vibration gate. The detector on the
board at about 489 ms a frame, precision 0.795 and recall 0.708. The dashboard,
the geofence, the route generation that fits rows inside it, and the whole
mission loop in simulation.

Not working yet: a complete autonomous flight on video. Two field days failed. A
camera that stopped enumerating on the board took the first one, and prearm
refusals took the second.

## Running it

Nothing on the board starts by itself. Every program is started by hand.

```
./start_dashboard                    # on the UNO Q, then open port 8080
./python tools/parameters.py push    # Pixhawk on USB
./python uno_q/sitl_test.py          # mission tests against SITL
```
