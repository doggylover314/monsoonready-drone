# Evidence, and what each piece proves

Every claim in `01_project_writeup.md` should have something behind it, so that
anyone asking "how do you know?" is one file away from the answer.

Files go in `docs/evidence/`, which does not exist yet. Use the names below so
the cross-references resolve. Downsize large image sets before committing and
keep the originals outside the repository.

## Model and training

| File | What it is | What it proves |
|--|--|--|
| `training_curve.png` | mAP against epoch from the run directory | Training was real and converged |
| `training_terminal.png` | Terminal during training, GPU and batch size visible | Trained locally on the RTX 3050, not in a cloud notebook |
| `dataset_counts.txt` | `merge_datasets.py` output, per-source counts and classes kept or dropped | Dataset construction was deliberate and reproducible |
| `spotcheck_grid.png` | Validation predictions including the failures | Honest evaluation. The sheet-water and glare cases are the argument for dataset v2. |
| `onnx_benchmark.txt` | Inference timing on the UNO Q | The edge-AI claim in numbers. Done: 489 ms median, 2.05 fps. |
| `unoq_detection.png` | UNO Q screen capture, image, box, confidence, timing | The single most important artefact in the set |

`training/spotcheck/` and `training/spotcheck_aerial/` already hold run-1
prediction images. They feed `spotcheck_grid.png` and should survive the
retrain, because run 1 against run 2 on identical images is itself evidence.

## Bench

| File | What it is | What it proves |
|--|--|--|
| `hopper_flow_test.mp4` | Mustard seed through the tube and gate, several cycles | The dispenser flows and does not bridge |
| `hopper_dose.png` | A weighed dose on a scale | Dosing is measured. Note the current numbers come from an under-filled hopper and need re-running full. |
| `tfluna_water_bench.png` | TF-Luna over a water basin, indoor and outdoor, nadir and angled | The over-water dropout question was tested rather than assumed. The answer decided descend-beside. |
| `esp32_mavlink_inspector.png` | QGC MAVLink Inspector showing `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` at about 10 Hz from component 195 | The obstacle module talks to the flight controller |
| `ring_channels.txt` | `tools/ring_channels.py` output, per-sensor stats | Which ring positions work, measured rather than assumed |
| `oled_status.jpg` | OLED with prearm status, satellites, mode, battery | Field readiness without a laptop |
| `power_calibration.jpg` | Multimeter against reported voltage and current | Power monitoring is calibrated |
| `vibration_log.png` | VibeZ plot from a hover log against the gate of 15 | The gate is measured, whatever the result |

Capture the vibration plot whether it passes or fails. Log 34 is the failing
one, median 46 with 5927 clip events, from the flight with a damaged prop.
Recent logs read 7.4 to 9.0. Both together are stronger than the passing one
alone.

## Flight

| File | What it is | What it proves |
|--|--|--|
| `hover_test.mp4` | Unloaded hover, stable | The rebuild flies |
| `gps_health.png` | Satellites and HDOP before arming | The crash-3 lesson is applied |
| `loiter_flight.mp4` | Position hold in wind | The crash-2 lesson is applied |
| `full_loop.mp4` | Detect, descend, drop, climb, resume | The judged loop |
| `flight_log.bin` | ArduPilot log from the demo flight | All of the above, verifiable by anyone who knows ArduPilot |

Keep the `.bin` from every flight, not just the good ones. Logs-first
troubleshooting is a standing rule here, and a hard question about a flight is
answerable from its log.

## Build

| File | What it is | What it proves |
|--|--|--|
| `build_progress_*.jpg` | Frame assembly, wiring, service loops, foam mounting | The build is the team's own work |
| `payload_layout.jpg` | Camera, TF-Luna, hopper and UNO Q mounted, with weights | The payload budget is managed |
| `weight_measurement.jpg` | All-up weight on a scale | Feeds the regulatory category question directly |
| `sensor_ring.jpg` | The VL53L0X positions, field of view clear of legs and props | The 25 degree FOV mounting constraint was checked |
| `conformal_coating.jpg` | Coating applied, baro and connectors masked | Monsoon readiness, which is the name of the project |

## Simulation

| File | What it is | What it proves |
|--|--|--|
| `sitl_happy_path.txt` | Terminal output, nominal scenario | The mission logic completes the loop |
| `sitl_dropout_drill.txt` | Terminal output, dropout drill | Abort-upward behaviour, tested |

Both reproducible right now, with SITL already running:

```bash
./python uno_q/sitl_test.py > docs/evidence/sitl_happy_path.txt
```

```bash
./python uno_q/sitl_test.py --drill dropout > docs/evidence/sitl_dropout_drill.txt
```

## Documentation

| File | What it is | What it proves |
|--|--|--|
| `dataset_licences.png` | Roboflow Universe pages showing CC BY 4.0 and the citation blocks | Licence compliance, and fills the TBD rows in `03_dataset_citations.md` |
| `bti_product.jpg` | Larvicide product packaging | Sourcing is real |

## If time runs short

In order:

1. `unoq_detection.png`, the edge-AI claim
2. `full_loop.mp4`, the functionality claim
3. `vibration_log.png`, the engineering-honesty claim
4. `sitl_dropout_drill.txt`, the safety-design claim
5. `spotcheck_grid.png`, the honest-evaluation claim

Those five carry the three things the project is judged on.
