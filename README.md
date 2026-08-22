# MonsoonReady

An F550 hexacopter that finds standing water after monsoon rain and drops
granular Bti larvicide into it. The YOLO model runs on an Arduino UNO Q bolted
to the aircraft, so it locates in the air. No ground station. No cloud.

Arduino Physical AI Challenge India 2026. Built by Reyansh and Raghav.

Test flights drop mustard seed. Not larvicide.

## What is in here

| Path | What it holds |
|------|---------------|
| `PROJECT_STATE.md` | State, decisions and TODO. Read this first, it is the source of truth. |
| `docs/` | The judged documentation set. Start at `docs/01_project_writeup.md`. |
| `uno_q/` | Everything that runs on the aircraft: mission state machine, MAVLink IO, detector, dropper, mission log. |
| `uno_q/basestation/` | The Flask dashboard. Map, flight control, fence and route editing. |
| `training/` | Dataset merge, YOLO training, ONNX export. |
| `esp32_obstacle_avoidance/` | Proximity ring firmware. Sends standard MAVLink to the Pixhawk. |
| `tools/` | Parameter push and pull, bench probes, log analysis, ring checks. |
| `param_dumps/` | `pixhawk_full_setup.param` is the hand-maintained config that gets pushed. |
| `Build Log.txt` | Reyansh keeps this by hand. |

Machine-specific details, meaning IP addresses and accounts, live in
`PRIVATE.md`, which is gitignored. On a new machine, copy `PRIVATE.sample.md`
and fill it in.

## Where it stands

Working: the airframe, which now clears its vibration gate; the detector on the
board at about 489 ms a frame, precision 0.795 and recall 0.708; the dashboard;
the geofence and the route generation that fits rows inside it; and the whole
mission loop in simulation.

Not working yet: a complete autonomous flight on video. Two field days failed. A
camera that stopped enumerating on the board took the first one, and prearm
refusals took the second.

## The hardware, briefly

- F550 hexacopter. Pixhawk 2.4.8 on ArduCopter 4.7.0, NEO-M8N GPS on a mast.
- Arduino UNO Q as companion computer, on the Pixhawk's USB. Camera stills go
  through ONNX and it commands the aircraft in guided mode.
- TF-Luna pointing down. Every descent is gated on it, and losing the return
  aborts upward.
- An ESP32 on TELEM1 reads a ring of VL53L0X sensors and sends the Pixhawk
  standard `OBSTACLE_DISTANCE`. Four of the six ring positions work, plus the
  upward one. One never got a sensor, the frame has no room, and one has never
  answered. Marking both absent in firmware is what stopped the arming failures.
- SiK 433 MHz on TELEM2. MG90 metal-gear servo on the hopper gate.

## Running it

Nothing on the board starts by itself. Every program is started by hand.

```
./start_dashboard          # on the UNO Q, then open port 8080
./python tools/parameters.py push    # on the laptop, Pixhawk on USB
./python uno_q/sitl_test.py          # mission tests against SITL
```
