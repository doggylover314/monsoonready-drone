# Base Station — post-landing heatmap / report server

After landing, the UNO Q stops being a mission computer and becomes the base
station: a small Flask app serving a dashboard of everything the mission(s)
logged. It is read-only by design; it never touches MAVLink or the aircraft.
Its only data source is the per-mission JSONL log written by
`uno_q/missionlog.py`, so a mission logged is automatically a mission
reportable, and the same files double as documentation evidence.

---

## 1. Data flow

```
mission.py ──recorder=MissionLog──► <data_dir>/missions/mission_<id>.jsonl
                                             │
app.py  ◄────── reads (never writes) ────────┘
   │
   ├── /api/missions            per-flight summaries
   ├── /api/events/<id|all>     event stream, one flight or accumulated
   ├── /api/site_meta           optional site-image bounds
   ├── /api/site_image          optional site-image file
   └── /                        static/index.html (the dashboard)
```

`mission.py` auto-launches the server on `DONE`/`STANDDOWN` when
`MissionConfig.basestation_cmd` is set (a plain argv list). SITL tests leave
it unset.

---

## 2. Files

| File | Purpose |
|------|---------|
| `app.py` | Read-only Flask server. `--data-dir ~/monsoonready_data --host 0.0.0.0 --port 8080` defaults. |
| `static/index.html` | Self-contained dashboard (vanilla JS + canvas, no external assets, works offline). |
| `gen_fake_mission.py` | Writes two plausible fake flights through `MissionLog`, for development without hardware. |

---

## 3. Run

On the UNO Q (Flask is a one-time install, offline afterwards):

```bash
~/venv/bin/pip install flask
~/venv/bin/python ~/uno_q/basestation/app.py
```

View at `http://drone:8080` on the tailnet, or at https://drone.reysen.net
through the cloudflared tunnel running on the board (forwards to
localhost:8080; token in `/etc/cloudflared`). The public URL needs the board
to have internet at viewing time.

On a dev machine, without a flight:

```bash
python3 uno_q/basestation/gen_fake_mission.py   # 2 fake flights
pip install flask                                # in a venv
python3 uno_q/basestation/app.py                 # then open localhost:8080
```

---

## 4. Dashboard

Default view accumulates all flights; a selector narrows to one. Every layer
is a checkbox:

| Layer | Encoding |
|-------|----------|
| Heatmap | Detection density, sequential blue ramp (light = sparse, dark = dense) |
| Detections | Blue dot per detector fire, confidence in tooltip |
| Treated | Green circle + check where a drop released |
| Aborts | Red X where a descent aborted, reason in tooltip |
| Flight track | 1 Hz breadcrumb polyline per flight |
| Site image | Optional georeferenced background (section 5) |

Persistent sites: detections within 5 m of each other seen in ≥ 2 distinct
flights get a dashed ring — the "standing-water candidate confirmed by
persistence" framing, computed only on the accumulated view.

Also: stat tiles, hover tooltips, an event table (text alternative to the
map), scale bar, north arrow, light/dark theme following the OS, and a 15 s
poll so a newly landed flight appears while the page is open.

---

## 5. Site image background

Drop a north-up aerial screenshot into the data dir plus `site_image.json`
giving its geographic corner bounds:

```json
{"file": "site.png", "lat_top": 12.9720, "lat_bottom": 12.9712,
 "lon_left": 77.5942, "lon_right": 77.5951}
```

The dashboard stretches the image between those corners under the data
layers. Without the file, the schematic canvas (which needs no network and no
calibration) is used alone.

---

## 6. Design notes

- **JSONL, one event per line, flushed per event.** A crash mid-mission loses
  at most the line being written; the reader skips truncated lines. The
  schema is defined in `missionlog.py` and only there.
- **The server never aggregates across requests** — every response is
  recomputed from the files, so `scp`-ing a mission file into the data dir
  (or deleting one) is immediately reflected.
- **The dashboard is one file** with zero external requests, so it renders
  identically over the tailnet, over the tunnel, or opened from disk next to
  a copied data dir.

---

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| https://drone.reysen.net gives 502 | Tunnel is up but `app.py` is not running on the board |
| Dashboard says "No mission data yet" | Data dir empty or wrong `--data-dir`; check `<data_dir>/missions/*.jsonl` exists |
| Site image checkbox missing | No `site_image.json` in the data dir, or its `file` does not exist |
| A flight shows "in progress / interrupted" | Its JSONL has no `mission_end` line: mission crashed or is still flying |
