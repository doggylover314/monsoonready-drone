# uno_q/ — onboard mission code (UNO Q Linux side)

Runs on the UNO Q (Debian, ~/venv python with pymavlink), talking MAVLink2 to the
Pixhawk on SERIAL4 @115200 as component 191 (onboard computer). Tested against
ArduPilot SITL 4.7.0 on the dev machines before it ever touches the aircraft.

## Files

- `mavlink_io.py` — link layer: connection, message-interval setup, single-threaded
  telemetry pump (GLOBAL_POSITION_INT, DISTANCE_SENSOR, HEARTBEAT), guided-mode
  commands (mode/arm/takeoff/goto/velocity) with ACK checking.
- `mission.py` — the detect->descend->treat state machine. Config at top of file.
- `detector.py` — DetectionSource interface. FakeDetector for SITL; the real ONNX
  camera detector plugs in behind the same interface later.
- `dropper.py` — Dropper interface. LogDropper for SITL; UNO Q GPIO/servo impl is an
  open item (SG90 PWM source: Linux GPIO is jittery, STM32 side likely owns it).
- `sitl_test.py` — scripted scenarios: happy path (survey -> latch -> descend ->
  drop -> resume -> RTL) and rangefinder-dropout drill (must abort upward).

## State machine

IDLE -> TAKEOFF -> SURVEY -> (detection) LATCH -> APPROACH -> DESCEND -> DROP
-> CLIMB -> SURVEY (next wp) ... -> DONE (RTL). Any pilot mode change away from
GUIDED => STANDDOWN (mission stops commanding, never fights the pilot).

Design rules baked in (from PROJECT_STATE):
- TARGET LATCHING: target lat/lon locked at survey altitude on first detection.
  No re-detection during descent (RUN1 spotcheck: close-range frames unreliable).
- Descent abort policy: DISTANCE_SENSOR stale (> timeout) or invalid during
  DESCEND => abort UPWARD to survey alt, skip target, resume survey. Never
  continue a descent blind (TF-Luna 850nm specular dropout over still water).
- Descend-beside option: `lateral_offset_n/e_m` config offsets the drop point;
  pending TF-Luna-over-water bench verdict (TODO 6).
- EKF-altitude belt-and-braces: during descent, if relative alt says we are at
  drop height but the rangefinder never confirmed, abort (no drop without a
  valid rangefinder reading).

## Run against SITL

From repo root (ardupilot clone sits in ../ardupilot):

    ../ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --no-mavproxy

then in another shell:

    .venv/bin/python uno_q/sitl_test.py                # happy path
    .venv/bin/python uno_q/sitl_test.py --drill dropout  # abort drill

SITL needs a simulated downward rangefinder (RNGFND1_TYPE=100 SITL type); the
test script sets/checks this via param protocol and reboots SITL if needed.

## On the aircraft (later)

Connection string becomes the serial device the STM32 byte-shovel exposes
(TODO 12); everything else is unchanged. Real detector = ONNX best.pt export
from Reyansh's training runs; real dropper = servo impl once PWM source decided.
