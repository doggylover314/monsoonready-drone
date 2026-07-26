# Evidence checklist

Photographs, screenshots and logs to capture, and what each one proves. Neither
of us can produce these from a keyboard, so this is a capture list for the two
of you.

The organising principle: **every claim in the write-up should have an artefact
behind it.** A judge reading "the model runs onboard at N frames per second"
should be one click from the terminal output that says so. Work down this list
and the documentation stops being assertions.

Suggested home: `docs/evidence/` with the filenames below, so links from the
other documents do not break. That folder is not in git yet, and large image
sets should be checked against the repository's size before committing; if it
gets heavy, keep the originals elsewhere and commit downsized copies.

## Training and model evidence

| File | What to capture | Proves |
|---|---|---|
| `training_curve.png` | The results plot from the run directory, showing mAP against epoch | Training was real and converged |
| `training_terminal.png` | Terminal during training: GPU, batch size, epoch progress | Trained locally on the RTX 3050, not in a cloud notebook |
| `dataset_counts.txt` | Output of the merge script showing per-source counts and which classes were kept and dropped | Dataset construction was deliberate and is reproducible |
| `spotcheck_grid.png` | A grid of validation predictions, **including the failures** | Honest evaluation. Include sheet water and glare cases; they are the evidence for the v2 rationale. |
| `onnx_benchmark.txt` | Inference timing output on the UNO Q | The edge-AI claim, in numbers |
| `unoq_detection.png` | Screen capture from the UNO Q: image, bounding box, confidence, inference time | The single most important artefact in the set |

The existing `training/spotcheck/` and `training/spotcheck_aerial/` folders
already hold prediction images from run 1. Those are the raw material for the
spotcheck grid; do not delete them when v2 finishes, because the comparison
between run 1 and v2 on the same images is itself good evidence.

## Bench evidence

| File | What to capture | Proves |
|---|---|---|
| `hopper_flow_test.mp4` | Salt flowing through the tube and gate, several cycles | The dispenser works and does not bridge or clog |
| `hopper_dose.png` | A measured dose on a scale | Dosing is quantified, not approximate |
| `tfluna_water_bench.png` | TF-Luna readings over a water basin, indoor and outdoor, nadir and angled | The over-water dropout question was tested, not assumed. This is TODO 6 and it decides descend-over versus descend-beside. |
| `esp32_mavlink_inspector.png` | Ground station MAVLink inspector showing `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` arriving at about 10Hz from component 195 | The obstacle module actually talks to the flight controller |
| `esp32_fake_banner.png` | Boot banner showing REAL versus FAKE sensor mode | The safety interlock on fake data exists |
| `oled_status.jpg` | The OLED showing prearm status, satellites, mode, battery | Field readiness without a laptop |
| `power_calibration.jpg` | Multimeter against the reported voltage and current | Power monitoring is calibrated, not assumed |
| `vibration_log.png` | VibeZ plot from a hover log, against the gate of 15 | The gate is measured, whatever the result. **Capture this even if it fails the gate.** |

That last row matters. A vibration plot that fails the gate, published
alongside the statement that we did not fly automatic modes because of it, is
better evidence of engineering judgment than any passing test.

## Flight evidence

| File | What to capture | Proves |
|---|---|---|
| `hover_test.mp4` | Unloaded hover, stable | The rebuild flies |
| `gps_health.png` | Satellites and HDOP before arming | The crash-3 procedural lesson is applied |
| `loiter_flight.mp4` | Position hold in wind | The crash-2 lesson is applied |
| `full_loop.mp4` | Detect, descend, drop, climb, resume | The judged loop |
| `flight_log.bin` | The ArduPilot log from the demo flight | Everything above, verifiable by anyone who knows ArduPilot |

Keep the `.bin` log from every flight, not just the good ones. Logs-first
troubleshooting is a standing rule on this project, and a judge who asks a hard
question about a flight is answerable from the log.

## Build evidence

| File | What to capture | Proves |
|---|---|---|
| `build_progress_*.jpg` | Frame assembly stages, wiring, service loops, foam mounting | The build is ours |
| `payload_layout.jpg` | Camera, TF-Luna, hopper and UNO Q mounted, with weights | Payload budget is managed, not hoped for |
| `weight_measurement.jpg` | All-up weight on a scale | Feeds the regulatory category question directly |
| `sensor_ring.jpg` | The seven VL53L0X sensors mounted, showing field of view clear of legs and propellers | The mounting constraint was checked |
| `conformal_coating.jpg` | Coating applied with the barometer and connectors masked | Monsoon readiness, which is the name of the project |

## Simulation evidence

| File | What to capture | Proves |
|---|---|---|
| `sitl_happy_path.txt` | Terminal output of the nominal scenario | The mission logic completes the loop |
| `sitl_dropout_drill.txt` | Terminal output of the dropout drill | The abort-upward behaviour, tested |

Both are producible today by running
[uno_q/sitl_test.py](../uno_q/sitl_test.py); the runs of 2026-07-26 passed and
the output is worth saving to files rather than left in a terminal.

## Documentation evidence

| File | What to capture | Proves |
|---|---|---|
| `dataset_licences.png` | Screenshots of each Roboflow Universe page showing CC BY 4.0 and the citation block | Licence compliance, and it fills the gaps in doc 03 at the same time |
| `bti_product.jpg` | The larvicide product packaging | Sourcing is real, per the project notes |

## Priority if time runs short

In order: `unoq_detection.png`, `full_loop.mp4`, `vibration_log.png`,
`sitl_dropout_drill.txt`, `spotcheck_grid.png`. Those five carry the edge-AI
claim, the functionality claim, and the engineering-honesty claim, which are
the three things this project is actually being judged on.
