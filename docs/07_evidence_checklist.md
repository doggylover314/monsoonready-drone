# Evidence, and what each piece proves

Every claim in `01_project_writeup.md` should have something behind it. Files go
in `docs/evidence/`, which does not exist yet. Use the names below so the
cross-references resolve, and downsize large image sets before committing.

## Model and training

| File | What it proves |
|--|--|
| `training_curve.png` | mAP against epoch. Training was real and converged. |
| `training_terminal.png` | GPU and batch size visible. Trained locally on the RTX 3050, not in a cloud notebook. |
| `dataset_counts.txt` | `merge_datasets.py` output. Dataset construction was deliberate and reproducible. |
| `spotcheck_grid.png` | Validation predictions including failures. The sheet-water and glare cases are the argument for dataset v2. |
| `onnx_benchmark.txt` | The edge-AI claim in numbers. Done: 489 ms median, 2.05 fps. |
| `unoq_detection.png` | UNO Q screen capture with box, confidence and timing. The single most important artefact. |

`training/spotcheck/` and `training/spotcheck_aerial/` already hold run-1
prediction images. They feed `spotcheck_grid.png` and should survive the
retrain, because run 1 against run 2 on identical images is itself evidence.

## Bench

| File | What it proves |
|--|--|
| `hopper_flow_test.mp4` | Mustard seed through the tube and gate. The dispenser flows and does not bridge. |
| `hopper_dose.png` | A weighed dose. Current numbers come from an under-filled hopper and need re-running full. |
| `tfluna_water_bench.png` | TF-Luna over a water basin. The over-water dropout question was tested, not assumed, and the answer decided descend-beside. |
| `esp32_mavlink_inspector.png` | `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` at 10 Hz from component 195. The ring talks to the flight controller. |
| `ring_channels.txt` | `tools/ring_channels.py` output. Which positions work, measured. |
| `oled_status.jpg` | Prearm status, satellites, mode, battery. Field readiness without a laptop. |
| `power_calibration.jpg` | Multimeter against reported voltage and current. |
| `vibration_log.png` | VibeZ against the gate of 15, whatever the result. |

Capture the vibration plot whether it passes or fails. Log 34 is the failing one
at median 46 with 5927 clip events, from the flight with a damaged prop. Recent
logs read 7.4 to 9.0. Both together beat the passing one alone.

## Flight

| File | What it proves |
|--|--|
| `hover_test.mp4` | Unloaded hover. The rebuild flies. |
| `gps_health.png` | Satellites and HDOP before arming. The crash-3 lesson applied. |
| `loiter_flight.mp4` | Position hold in wind. The crash-2 lesson applied. |
| `full_loop.mp4` | Detect, descend, drop, climb, resume. The judged loop. |
| `flight_log.bin` | The whole flight, verifiable by anyone who knows ArduPilot |

Keep the `.bin` from every flight, not just the good ones. Logs-first
troubleshooting is a standing rule here.

## Build

| File | What it proves |
|--|--|
| `build_progress_*.jpg` | Assembly, wiring, service loops, foam mounting. The build is ours. |
| `payload_layout.jpg` | Camera, TF-Luna, hopper and UNO Q mounted, with weights |
| `weight_measurement.jpg` | All-up weight. Feeds the regulatory category question. |
| `sensor_ring.jpg` | Ring positions, field of view clear of legs and props |
| `conformal_coating.jpg` | Coating applied, baro and connectors masked |

## Simulation

`sitl_happy_path.txt` and `sitl_dropout_drill.txt`, the terminal output from both
scenarios. Reproducible right now with SITL running:

```bash
./python uno_q/sitl_test.py > docs/evidence/sitl_happy_path.txt
```

```bash
./python uno_q/sitl_test.py --drill dropout > docs/evidence/sitl_dropout_drill.txt
```

## Documentation

`dataset_licences.png`, the Roboflow Universe pages showing CC BY 4.0 and the
citation blocks, which also fills the TBD rows in `03_dataset_citations.md`. And
`bti_product.jpg`, the larvicide packaging.

## If time runs short

1. `unoq_detection.png`, the edge-AI claim
2. `full_loop.mp4`, the functionality claim
3. `vibration_log.png`, the engineering-honesty claim
4. `sitl_dropout_drill.txt`, the safety-design claim
5. `spotcheck_grid.png`, the honest-evaluation claim
