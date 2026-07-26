# MonsoonReady — standing-water detect & treat hexacopter

Arduino Physical AI Challenge India entry. An F550 hexacopter surveys a site;
an Arduino UNO Q detects standing-water candidates in downward stills with an
onboard YOLO (ONNX), descends and drops granular larvicide (demo: inert
salt), then becomes a base station serving a heatmap/report of the site. The
judged detect → descend → treat loop runs onboard.

---

## 1. Repository map

| Path | Contents |
|------|----------|
| `PROJECT_STATE.md` | Single source of truth: state, decisions, TODO, session continuity. Read first. |
| `docs/` | Judged documentation set: writeup, crash postmortems, dataset citations, video storyboard, compliance, judge Q&A, evidence checklist, AI-authorship disclosure. |
| `uno_q/` | Onboard mission code: state machine, MAVLink IO, detector/dropper interfaces, JSONL mission log, SITL tests. |
| `uno_q/basestation/` | Post-landing Flask heatmap/report dashboard. |
| `training/` | YOLO pipeline: dataset merge, training, ONNX export for the UNO Q. |
| `esp32_obstacle_avoidance/` | 7x VL53L0X proximity ring firmware feeding the Pixhawk (OBSTACLE_DISTANCE / DISTANCE_SENSOR). |
| `tools/` | Pixhawk parameter push/pull via pymavlink with per-write ack. |
| `param_dumps/` | Parameter files; `pixhawk_full_setup.param` is the hand-maintained tracked config. |
| `Build Log.txt` | Owner-maintained build and flight log. |

Machine and account details live in `PRIVATE.md` (gitignored; template
`PRIVATE.sample.md`).

---

## 2. Hardware in one line each

- F550 hexa, Pixhawk 2.4.8 on ArduPilot 4.7.0, NEO-M8N GPS + compass.
- Arduino UNO Q companion (Debian): camera stills → ONNX → mission logic,
  MAVLink on SERIAL4.
- TF-Luna rangefinder on SERIAL5 gates every descent; loss of return aborts
  the descent upward.
- ESP32 + 7x VL53L0X proximity ring on TELEM2.
- SG90 servo gate hopper for granules.
