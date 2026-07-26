# MonsoonReady — mosquito-breeding-water detect & treat hexacopter

Arduino Physical AI Challenge India entry. An F550 hexacopter surveys a site,
an Arduino UNO Q detects standing-water candidates in downward stills with an
onboard YOLO (ONNX), descends and drops granular larvicide (demo: inert salt),
then turns into a base station serving a heatmap/report of the site.

## Repo map

- `PROJECT_STATE.md` — single source of truth: state, decisions, TODO,
  session continuity. Read this first; CLAUDE.md tells the AIs to.
- `docs/` — judged documentation set (writeup, crash postmortems, dataset
  citations, video storyboard, compliance, judge Q&A, evidence checklist,
  AI-authorship disclosure).
- `uno_q/` — onboard mission code (state machine, MAVLink IO, detector and
  dropper interfaces, JSONL mission log, SITL tests) and
  `uno_q/basestation/` — the post-landing Flask heatmap/report dashboard.
- `training/` — YOLO training pipeline on the RTX 3050 laptop (dataset merge,
  train, ONNX export for the UNO Q).
- `esp32_obstacle_avoidance/` — 7x VL53L0X proximity ring firmware feeding
  the Pixhawk OBSTACLE_DISTANCE/DISTANCE_SENSOR.
- `tools/` — Pixhawk parameter push/pull via pymavlink (per-write ack; QGC
  bulk load is not trusted).
- `param_dumps/` — parameter files; `pixhawk_full_setup.param` is the
  hand-maintained tracked config, dumps are regenerable.
- `Build Log.txt` — owner-maintained build/flight log (manual; not edited by
  the AIs).

Private machine/account details live in `PRIVATE.md` (gitignored; template
`PRIVATE.sample.md`).
