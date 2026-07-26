# UNO Q Mission Computer → ArduPilot (MAVLink)

Onboard mission code for the **Arduino UNO Q** (Debian, `~/venv` python with
pymavlink) that drives the **detect → descend → treat** loop over **`SERIAL4`
at 115200** as MAVLink **component 191**:

- **detect** standing water in downward stills and **latch** the target at
  survey altitude;
- **descend** on the downward rangefinder, aborting **upward** on any loss of
  height reference;
- **drop** granules through the servo gate, climb out, resume the survey.

The link layer and the state machine are cleanly separated, and the detector
and dropper sit behind interfaces so the SITL stand-ins can be swapped for the
ONNX model and the real servo without touching `mission.py`. After landing,
the same board becomes the base station: the mission's JSONL event log is
served as a heatmap/report dashboard (see `basestation/README.md`).

---

## 1. Files

| File | Purpose |
|------|---------|
| `mavlink_io.py` | Link layer: connection, message intervals, telemetry pump, guided commands. No mission logic. |
| `mission.py` | The state machine. **All** tunables live in `MissionConfig` at the top. |
| `detector.py` | `DetectionSource` interface + `FakeDetector` (SITL) + `OnnxDetector` (stub) + lat/lon helpers |
| `dropper.py` | `Dropper` interface + `LogDropper` (SITL) + `ServoDropper` (stub) |
| `sitl_test.py` | Scripted scenarios: nominal mission and rangefinder-dropout drill |
| `sitl_rangefinder.parm` | SITL parameters mirroring the TF-Luna (0.2 to 8 m, downward) |
| `missionlog.py` | Per-mission JSONL event log; the schema lives here and only here. Passed to `Mission` as `recorder=`; without it the mission logs nothing (SITL tests unchanged). |
| `basestation/` | Post-landing Flask heatmap/report dashboard (TODO 13), fed solely by the JSONL logs. Auto-launched via `MissionConfig.basestation_cmd`. Own README. |

---

## 2. State machine

```
IDLE ──► TAKEOFF ──► SURVEY ◄────────────────────────┐
                       │                             │
                       │ detection (target LATCHED)  │
                       ▼                             │
                    APPROACH                         │
                       │ over target, at survey alt  │
                       ▼                             │
                    DESCEND ──── abort ──► ABORT_CLIMB
                       │ rng <= drop_alt_m           │
                       ▼                             │
                     DROP ──► CLIMB ─────────────────┘
                                              wp exhausted
                                                     │
                                                     ▼
                                                DONE ──► RTL

any mode change away from GUIDED, from any state ──► STANDDOWN
```

| State | Behaviour | Exit |
|-------|-----------|------|
| `TAKEOFF` | Guided takeoff to `survey_alt_m` | Within `alt_tol_m` of survey altitude |
| `SURVEY` | Fly waypoints; poll the detector every tick | Detection → `APPROACH`; waypoints exhausted → `DONE` |
| `APPROACH` | Reposition over the latched target at survey altitude | Within `wp_radius_m` and `alt_tol_m` |
| `DESCEND` | Descend at `descent_mps`, watching the rangefinder | Valid `rng <= drop_alt_m` → `DROP`; any abort condition → `ABORT_CLIMB` |
| `DROP` | Hold zero velocity, fire the dropper, dwell `drop_dwell_s` | Dwell elapsed |
| `CLIMB` / `ABORT_CLIMB` | Climb at `climb_mps`, clear the latch | Back at survey altitude → `SURVEY` |
| `STANDDOWN` | Stop commanding entirely | Terminal. The pilot has the aircraft. |
| `DONE` | Set `end_mode` (default `RTL`) | Terminal |

---

## 3. Configuration (`MissionConfig` in `mission.py`)

| Field | Default | Meaning |
|-------|---------|---------|
| `waypoints` | — | `[(lat, lon), ...]` survey pattern |
| `survey_alt_m` | `15.0` | Survey and detection altitude |
| `drop_alt_m` | `3.0` | **Rangefinder** AGL that triggers the drop |
| `descent_mps` / `climb_mps` | `0.5` / `1.0` | Vertical rates |
| `wp_radius_m` / `alt_tol_m` | `1.5` / `1.0` | Arrival tolerances |
| `rng_timeout_s` | `1.0` | A reading older than this is stale |
| `rng_expect_m` | `6.0` | EKF altitude by which the rangefinder **must** have acquired ground |
| `floor_margin_m` | `1.0` | EKF altitude below `drop_alt_m` that counts as sources disagreeing |
| `drop_dwell_s` | `2.0` | Hold time over the target while granules fall |
| `lateral_offset_n_m` / `_e_m` | `0.0` | **Descend-beside** offsets. Stay 0 until the TF-Luna over-water bench test (TODO 6) decides. |
| `end_mode` | `'RTL'` | Mode set on completion |

---

## 4. Safety rules, and why they are shaped this way

### 4.1 Target latching

Target lat/lon is locked on the **first detection at survey altitude**, and the
detector is not polled again until the aircraft returns to `SURVEY`.

Run-1 spot-checking showed close-range frames, where water fills most of the
frame, are where the model is **least** reliable. Re-detecting during descent
would hand steering authority to the worst frames. Latching keeps the decision
with the wide view from altitude.

### 4.2 Descent aborts upward, but not on every blind moment

The TF-Luna is a ~8 m sensor and the survey altitude is 15 m, so **the first
part of every descent is legitimately blind**. A naive "no reading means abort"
rule would abort 100% of descents. The implemented rule tracks whether ground
was ever acquired this descent (`_rng_acquired`):

| Condition | Action | Reason |
|-----------|--------|--------|
| No reading, `rel_alt >= rng_expect_m` | **Continue** | Out of sensor range; expected |
| No reading, `rel_alt < rng_expect_m` | **Abort up** | Should see ground by now |
| Acquired, then stale beyond `rng_timeout_s` or invalid | **Abort up** | Dropout, likely specular off still water |
| `rel_alt < drop_alt_m - floor_margin_m`, never confirmed | **Abort up** | EKF and rangefinder disagree |
| Valid reading `<= drop_alt_m` | **Drop** | Confirmed at drop height |

An abort clears the target and resumes the survey; it never retries the same
puddle. 850 nm infrared against still water behaves close to a mirror, so
dropout over the target is expected rather than exceptional, and two of this
project's three crashes came from trusting a single corrupted altitude source.

### 4.3 The pilot always wins

Every tick checks the heartbeat's mode. Anything other than `GUIDED` moves the
machine to `STANDDOWN`, which commands nothing further.

---

## 5. Link layer (`mavlink_io.py`) design notes

- **Single-threaded.** One pump, `step()`, receives everything and updates the
  telemetry cache; commands that need an ACK pump the same loop while waiting.
  No locks, no races, and identical behaviour on the laptop over TCP and on the
  UNO Q over serial.
- **Every command is acknowledged.** `command_ack()` retries on
  `TEMPORARILY_REJECTED`, treats `IN_PROGRESS` as "final ack still coming", and
  raises on a hard rejection. `arm()` additionally retries on `FAILED`, because
  ArduPilot answers FAILED while prearm checks are still settling after boot.
- **Streams are requested, not assumed:** `MAV_CMD_SET_MESSAGE_INTERVAL` for
  `GLOBAL_POSITION_INT` at 5 Hz and `DISTANCE_SENSOR` at 10 Hz.
- **Only the downward rangefinder is consumed.** `DISTANCE_SENSOR` messages are
  filtered on `MAV_SENSOR_ROTATION_PITCH_270`, so the ESP32's upward sensor on
  component 195 cannot be mistaken for the TF-Luna.
- **Only the autopilot's heartbeat sets mode and armed state**, filtered on
  `MAV_COMP_ID_AUTOPILOT1`, for the same reason.
- **Velocity setpoints are rate-limited** to `SETPOINT_RESEND_S` (0.2 s = 5 Hz).
  ArduPilot discards guided setpoints older than a few seconds, so they must be
  resent continuously, but not every tick.

---

## 6. Test against SITL

Build once (sibling clone, matching the board's 4.7.0):

```bash
git clone --depth 1 --branch Copter-4.7.0 --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git ../ardupilot
cd ../ardupilot && ./waf configure --board sitl && ./waf copter
```

Start the simulator in its own terminal. **It is a server: it runs until killed
and never exits on its own.** Do not pipe it through `tail`, which swallows its
output.

```bash
../ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --no-mavproxy --speedup 5 --add-param-file=uno_q/sitl_rangefinder.parm
```

Then, from the repository root:

```bash
.venv/bin/python uno_q/sitl_test.py                   # nominal
.venv/bin/python uno_q/sitl_test.py --drill dropout   # rangefinder-loss drill
```

| Scenario | Pass criteria |
|----------|---------------|
| Nominal | Exactly one drop, no abort, survey completes, RTL, disarm |
| `--drill dropout` | Rangefinder suppressed below 6 m; zero drops, `ABORT_CLIMB` reached, survey still completes |

The drill suppresses the reading **client-side**, in `rng_suppress_below_m`, so
no SITL parameter surgery is needed and the flight controller behaves normally.

Nominal run, 2026-07-26:

```
[mission] TAKEOFF -> SURVEY (wp 0)
[mission] SURVEY -> APPROACH (latched -35.3630373,149.1653697)
[mission] APPROACH -> DESCEND
[dropper] TRIGGER #1 (simulated)
[mission] DESCEND -> DROP (rng=2.98m)
[mission] DROP -> CLIMB (treated)
[mission] CLIMB -> SURVEY (wp 1)
...
[test] drops=1 abort=None
[test] PASS (none)
```

Dropout drill, same date:

```
[mission] APPROACH -> DESCEND
[mission] DESCEND -> ABORT_CLIMB (rangefinder dropout during descent)
[mission] ABORT_CLIMB -> SURVEY (wp 1)
...
[test] drops=0 abort='rangefinder dropout during descent'
[test] PASS (dropout)
```

`sitl_test.py` checks `RNGFND1_TYPE == 100` before starting and exits with
instructions if the parameter file was not loaded, because without a simulated
rangefinder every descent would abort and the drill would pass for the wrong
reason.

---

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| `RNGFND1_TYPE != 100` on startup | `sim_vehicle.py` launched without `--add-param-file=uno_q/sitl_rangefinder.parm` |
| Connection refused on `tcp:127.0.0.1:5760` | SITL not up yet. `connect_retry()` waits 60 s; beyond that, check the simulator terminal. |
| `no heartbeat from flight controller` | Wrong connection string, or the port is held by another client (QGC takes it exclusively on serial) |
| Every descent aborts immediately | No `DISTANCE_SENSOR` arriving. Check the stream request, and that the sensor is downward-facing (`RNGFND1_ORIENT 25`). |
| Mission ends in `STANDDOWN` unexpectedly | Something changed flight mode. Expected if a pilot or a failsafe intervened. |
| `command N: no ACCEPTED ack` on arm | Prearm checks failing. In SITL, wait for EKF settle; on the aircraft, read the prearm message. |

---

## 8. On the aircraft

Unchanged code; only the connection string differs, becoming the serial device
the STM32 byte-shovel exposes (TODO 12). Three items remain open:

| Item | Blocked on |
|------|-----------|
| `OnnxDetector` | Camera bench (TODO 2) and the v2 ONNX export from the training laptop |
| Pixel → ground offset | Camera intrinsics + survey altitude. `FakeDetector` currently reports a target at the aircraft's own position. |
| `ServoDropper` | PWM source decision (TODO 12): Linux userspace soft-PWM is jittery; the STM32 side via Bridge is the cleaner path |
