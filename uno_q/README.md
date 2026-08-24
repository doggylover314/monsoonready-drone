# The mission computer

Onboard code for the Arduino UNO Q, running under its own `~/venv` python with
pymavlink. It drives the detect, descend, treat loop over the Pixhawk's USB link
as MAVLink component 191: find standing water in downward stills and latch the
target at survey altitude, come down on the downward rangefinder and abort
upward on any loss of height reference, then drop granules through the servo
gate, climb out and resume the survey.

Link layer and state machine are separate, and the detector and dropper sit
behind interfaces, so the SITL stand-ins swap for the ONNX model and the real
servo without touching `mission.py`. After landing the same board becomes the
base station, serving the mission's JSONL event log as a dashboard. That has its
own README in `basestation/`.

## Files

| File | Purpose |
|------|---------|
| `mavlink_io.py` | Connection, message intervals, telemetry pump, guided commands. No mission logic. |
| `mission.py` | The state machine. Every tunable lives in `MissionConfig` at the top. |
| `detector.py` | `DetectionSource` interface, `FakeDetector` for SITL, `OnnxDetector` for the camera and yolo26n, plus lat/lon helpers |
| `detect_worker.py` | Runs the detector in its own process and publishes the latest result |
| `camera.py`, `camera_geom.py` | Capture by device name, and the nadir pinhole projection from detection pixel to North/East ground offset |
| `dropper.py` | `Dropper` interface, `LogDropper` for SITL, `PixhawkServoDropper` for the flight servo |
| `fence.py`, `make_waypoints.py` | Geofence handling and survey row generation inside it |
| `missionlog.py` | Per-mission JSONL event log. The schema lives here and nowhere else. |
| `run_mission.py` | The onboard runner. Wires the real detector, dropper and log together, handles signals, commands RTL on any failure. |
| `sitl_test.py` | Two scripted scenarios: nominal mission and rangefinder-dropout drill |
| `predict.py` | Recurrence scoring across past missions, read-only over the same JSONL |
| `basestation/` | The post-landing dashboard, fed only by the JSONL logs |

## The state machine

```
IDLE -> TAKEOFF -> SURVEY <-----------------------+
                     |                            |
                     | detection, target LATCHED  |
                     v                            |
                  APPROACH                        |
                     |                            |
                     v                            |
                  DESCEND ---- abort --> ABORT_CLIMB
                     |                            |
                     v                            |
                   DROP --> CLIMB ----------------+
                                        waypoints exhausted
                                                  |
                                                  v
                                             DONE -> RTL

any mode change away from GUIDED, from any state -> STANDDOWN
```

TAKEOFF climbs to `survey_alt_m` and exits within `alt_tol_m`. SURVEY flies
waypoints and polls the detector every tick, exiting to APPROACH on a detection
or to DONE when the waypoints run out. APPROACH repositions over the latched
target at survey altitude. DESCEND comes down at `descent_mps` watching the
rangefinder, reaching DROP on a valid reading at or below `drop_alt_m` and
ABORT_CLIMB on any abort condition. DROP holds zero velocity, fires the dropper
and dwells. CLIMB and ABORT_CLIMB return to survey altitude and clear the latch.
STANDDOWN stops commanding entirely and is terminal, because the pilot has the
aircraft. DONE sets `end_mode`, which defaults to RTL.

## Why it is shaped this way

Target lat/lon is locked on the first detection at survey altitude, and the
detector is not polled again until the aircraft is back in SURVEY. Run-1
spot-checking showed close-range frames, where water fills most of the frame,
are where the model is least reliable. Re-detecting during descent would hand
steering authority to the worst frames, so latching keeps the decision with the
wide view from altitude.

Descents abort upward, but not on every blind moment. A TF-Luna is good to about
8 m, so a descent that starts above that is legitimately blind at first and a
naive "no reading means abort" rule would abort every time. The implemented rule
tracks whether ground was ever acquired this descent:

| Condition | Action | Reason |
|-----------|--------|--------|
| No reading, `rel_alt >= rng_expect_m` | Continue | Out of sensor range, expected |
| No reading, `rel_alt < rng_expect_m`, past the grace window | Abort up | Should see ground by now |
| Acquired, then stale beyond `rng_timeout_s` or invalid | Abort up | Dropout, likely specular off still water |
| `rel_alt < drop_alt_m - floor_margin_m`, never confirmed | Abort up | EKF and rangefinder disagree |
| Valid reading at or below `drop_alt_m` | Drop | Confirmed at drop height |

At the current 5 m survey the descent starts already below `rng_expect_m`, which
is why `rng_grace_s` exists: without it one stale tick on entry aborts the
descent instantly. An abort clears the target and resumes the survey, and it
never retries the same puddle. 850 nm infrared against still water behaves close
to a mirror, so dropout over the target is expected rather than exceptional, and
two of this project's three crashes came from trusting a single corrupted
altitude source.

The aircraft descends beside the puddle rather than over it, holds the captured
altitude while it translates across, releases, and translates back before
climbing. No rangefinder abort applies while it is over water. Setting the
lateral offsets to zero restores the old vertical descent, which is what the
SITL drills use.

The pilot always wins. Every tick checks the heartbeat's mode, and anything
other than GUIDED moves the machine to STANDDOWN, which commands nothing
further.

## Link layer notes

One pump, `step()`, receives everything and updates the telemetry cache, and
commands that need an ack pump the same loop while waiting. No locks, no races,
and identical behaviour on the laptop over TCP and on the board over USB.

Every command is acknowledged. `command_ack()` retries on
`TEMPORARILY_REJECTED`, treats `IN_PROGRESS` as an ack still coming, and raises
on a hard rejection. `arm()` additionally retries on `FAILED`, because ArduPilot
answers FAILED while prearm checks are still settling after boot.

Streams are requested rather than assumed, through
`MAV_CMD_SET_MESSAGE_INTERVAL`, at 5 Hz for `GLOBAL_POSITION_INT` and 10 Hz for
`DISTANCE_SENSOR`. Only the downward rangefinder is consumed: `DISTANCE_SENSOR`
is filtered on `MAV_SENSOR_ROTATION_PITCH_270` so the ESP32's upward sensor on
component 195 cannot be mistaken for the TF-Luna. Only the autopilot's heartbeat
sets mode and armed state, filtered on `MAV_COMP_ID_AUTOPILOT1`, for the same
reason. Velocity setpoints are rate-limited to 5 Hz, because ArduPilot discards
guided setpoints older than a few seconds so they must be resent continuously,
just not every tick.

## Testing against SITL

Build once, from a sibling clone matching the board's 4.7.0:

```bash
git clone --depth 1 --branch Copter-4.7.0 --recurse-submodules --shallow-submodules https://github.com/ArduPilot/ardupilot.git ../ardupilot
```

Start the simulator in its own terminal. It is a server, so it runs until killed
and never exits on its own. Do not pipe it through `tail`, which swallows its
output.

```bash
../ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --no-mavproxy --speedup 5 --add-param-file=uno_q/sitl_rangefinder.parm
```

Then, from the repository root:

```bash
.venv/bin/python uno_q/sitl_test.py
```

```bash
.venv/bin/python uno_q/sitl_test.py --drill dropout
```

The nominal run passes on exactly one drop, no abort, survey complete, RTL and
disarm. The dropout drill suppresses the rangefinder below 6 m and passes only
on zero drops, ABORT_CLIMB reached, and the survey still completing. That
suppression happens client-side in `rng_suppress_below_m`, so no SITL parameter
surgery is needed and the flight controller behaves normally.

`sitl_test.py` checks `RNGFND1_TYPE == 100` before starting and exits with
instructions if the parameter file was not loaded. Without a simulated
rangefinder every descent would abort and the drill would pass for the wrong
reason.

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `RNGFND1_TYPE != 100` on startup | `sim_vehicle.py` launched without `--add-param-file=uno_q/sitl_rangefinder.parm` |
| Connection refused on `tcp:127.0.0.1:5760` | SITL not up yet. `connect_retry()` waits 60 s; beyond that, read the simulator terminal. |
| `no heartbeat from flight controller` | Wrong connection string, or the port is held by another client. QGC takes it exclusively on serial. |
| Every descent aborts immediately | No `DISTANCE_SENSOR` arriving. Check the stream request and that the sensor is downward-facing, `RNGFND1_ORIENT 25`. |
| Mission ends in `STANDDOWN` unexpectedly | Something changed flight mode. Expected if a pilot or a failsafe intervened. |
| `command N: no ACCEPTED ack` on arm | Prearm checks failing. In SITL wait for EKF settle; on the aircraft, read the prearm message. |

## On the aircraft

`run_mission.py` is the onboard entry point, and it wires the same `Mission`
that SITL proves to the real detector, dropper and log. `setsid` is not
optional. Detached, the runner survives the ssh session that started it;
attached, closing that session delivers SIGHUP to an aircraft that may be
mid-descent.

The PWM source question is settled: the pulse comes from a Pixhawk output, not
the UNO Q, and `dropper.py`'s module docstring carries the full reasoning.
`SERVO<n>_FUNCTION=0` plus `SERVO<n>_MIN` and `_MAX` have to be pushed before
`DO_SET_SERVO` does anything.
