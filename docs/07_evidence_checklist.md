# Evidence Set → Artefacts and What They Prove

The artefact set backing the documentation. Organising principle: **every claim
in `01_project_writeup.md` has something behind it**, so that a reader who asks
"how do you know?" is one file away from the answer.

Location: `docs/evidence/`, using the filenames below so cross-references
resolve. Large image sets should be downsized before committing; originals can
live outside the repository.

---

## 1. Training and model

| Artefact | Contents | Supports |
|----------|----------|----------|
| `training_curve.png` | Results plot from the run directory: mAP against epoch | Training was real and converged |
| `training_terminal.png` | Terminal during training: GPU, batch size, epoch progress | Trained locally on the RTX 3050, not in a cloud notebook |
| `dataset_counts.txt` | `merge_datasets.py` output: per-source counts, classes kept and dropped | Dataset construction was deliberate and reproducible |
| `spotcheck_grid.png` | Grid of validation predictions, **including failures** | Honest evaluation. Sheet-water and glare cases are the evidence for the v2 rationale. |
| `onnx_benchmark.txt` | Inference timing on the UNO Q | The edge-AI claim, in numbers |
| `unoq_detection.png` | UNO Q screen capture: image, box, confidence, inference time | The single most important artefact in the set |

`training/spotcheck/` and `training/spotcheck_aerial/` already hold run-1
prediction images. They are the raw material for `spotcheck_grid.png` and
should survive the v2 retrain, since the run-1 versus v2 comparison on identical
images is itself evidence.

---

## 2. Bench

| Artefact | Contents | Supports |
|----------|----------|----------|
| `hopper_flow_test.mp4` | Salt flowing through tube and gate, several cycles | The dispenser works and does not bridge or clog |
| `hopper_dose.png` | A measured dose on a scale | Dosing is quantified, not approximate |
| `tfluna_water_bench.png` | TF-Luna over a water basin: indoor and outdoor, nadir and angled | The over-water dropout question was tested rather than assumed. Decides descend-over versus descend-beside (TODO 6). |
| `esp32_mavlink_inspector.png` | QGC MAVLink Inspector: `OBSTACLE_DISTANCE` + `DISTANCE_SENSOR` at ~10 Hz from component 195 | The obstacle module talks to the flight controller |
| `esp32_fake_banner.png` | Boot banner showing REAL versus FAKE sensor mode | The fake-data safety interlock exists |
| `oled_status.jpg` | OLED showing prearm status, satellites, mode, battery | Field readiness without a laptop |
| `power_calibration.jpg` | Multimeter against reported voltage and current | Power monitoring is calibrated, not assumed |
| `vibration_log.png` | VibeZ plot from a hover log against the gate of 15 | The gate is measured, **whatever the result** |

The last row is captured whether it passes or fails. A vibration plot that
fails the gate, published beside the statement that automatic modes were
therefore not flown, is stronger evidence of engineering judgment than a
passing test.

---

## 3. Flight

| Artefact | Contents | Supports |
|----------|----------|----------|
| `hover_test.mp4` | Unloaded hover, stable | The rebuild flies |
| `gps_health.png` | Satellites and HDOP before arming | The crash-3 procedural lesson is applied |
| `loiter_flight.mp4` | Position hold in wind | The crash-2 lesson is applied |
| `full_loop.mp4` | Detect, descend, drop, climb, resume | The judged loop |
| `flight_log.bin` | ArduPilot log from the demo flight | All of the above, verifiable by anyone who knows ArduPilot |

`.bin` logs are kept from **every** flight, not only successful ones.
Logs-first troubleshooting is a standing project rule, and a hard question about
a flight is answerable from its log.

---

## 4. Build

| Artefact | Contents | Supports |
|----------|----------|----------|
| `build_progress_*.jpg` | Frame assembly stages, wiring, service loops, foam mounting | The build is the team's own work |
| `payload_layout.jpg` | Camera, TF-Luna, hopper, UNO Q mounted, with weights | Payload budget is managed, not hoped for |
| `weight_measurement.jpg` | All-up weight on a scale | Feeds the regulatory category question directly |
| `sensor_ring.jpg` | Seven VL53L0X mounted, field of view clear of legs and props | The 25° FOV mounting constraint was checked |
| `conformal_coating.jpg` | Coating applied, baro and connectors masked | Monsoon readiness, which is the name of the project |

---

## 5. Simulation

| Artefact | Contents | Supports |
|----------|----------|----------|
| `sitl_happy_path.txt` | Terminal output, nominal scenario | The mission logic completes the loop |
| `sitl_dropout_drill.txt` | Terminal output, dropout drill | The abort-upward behaviour, tested |

Both are reproducible today:

```bash
.venv/bin/python uno_q/sitl_test.py                  > docs/evidence/sitl_happy_path.txt
.venv/bin/python uno_q/sitl_test.py --drill dropout  > docs/evidence/sitl_dropout_drill.txt
```

---

## 6. Documentation

| Artefact | Contents | Supports |
|----------|----------|----------|
| `dataset_licences.png` | Roboflow Universe pages showing CC BY 4.0 and citation blocks | Licence compliance, and fills the TBD rows in `03_dataset_citations.md` |
| `bti_product.jpg` | Larvicide product packaging | Sourcing is real |

---

## 7. Priority order

If capture time runs short, in order:

1. `unoq_detection.png` — the edge-AI claim
2. `full_loop.mp4` — the functionality claim
3. `vibration_log.png` — the engineering-honesty claim
4. `sitl_dropout_drill.txt` — the safety-design claim
5. `spotcheck_grid.png` — the honest-evaluation claim

Those five carry the three things the project is actually judged on.
