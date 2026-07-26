# basestation/ — post-mission heatmap + report server (TODO 13)

After landing, the UNO Q stops being a mission computer and becomes the base
station: a small Flask app serving a dashboard of everything the mission(s)
logged. Read-only by design; it never touches MAVLink or the aircraft.

## Data flow

    mission.py --(recorder=MissionLog)--> <data_dir>/missions/mission_<id>.jsonl
    app.py  reads those files  -->  /api/*  -->  static/index.html (dashboard)

- `uno_q/missionlog.py` defines the JSONL schema (one event per line, flushed
  per event, crash-tolerant). The base station has no other data source, so a
  mission logged is automatically a mission reportable, and the same files are
  documentation evidence.
- `mission.py` auto-launches the server on DONE/STANDDOWN when
  `MissionConfig.basestation_cmd` is set (a plain argv list). SITL tests leave
  it unset.

## Run

On the UNO Q (Flask one-time install: `~/venv/bin/pip install flask`):

    ~/venv/bin/python ~/uno_q/basestation/app.py

Defaults: `--data-dir ~/monsoonready_data --host 0.0.0.0 --port 8080`.
View at `http://drone:8080` on the tailnet, or publicly at
https://drone.reysen.net (cloudflared tunnel on the board forwards to
localhost:8080; token in /etc/cloudflared; needs board internet at viewing
time, i.e. the phone hotspot in the field).

Laptop development without a flight:

    python3 uno_q/basestation/gen_fake_mission.py     # 2 fake flights
    pip install flask                                  # in a venv
    python3 uno_q/basestation/app.py                   # then open :8080

## Dashboard

- Accumulated view across all flights (default) or a single flight.
- Toggleable layers: detection heatmap (blue = density), detection dots,
  treated drops (green check), aborts (red X, reason in tooltip), flight
  track, optional site image.
- Persistent sites: detections within 5 m of each other seen in >= 2 distinct
  flights get a dashed ring — that is the "standing water candidate confirmed
  by persistence" framing, computed only on the all-flights view.
- Stats tiles, hover tooltips, an event table (accessibility/table view), a
  scale bar and north arrow. Light + dark theme follow the OS. Auto-refreshes
  every 15 s so a newly landed flight appears while the page is open.

## Optional site image background

Drop an aerial screenshot (north-up) into the data dir plus `site_image.json`:

    {"file": "site.png", "lat_top": 12.9720, "lat_bottom": 12.9712,
     "lon_left": 77.5942, "lon_right": 77.5951}

The corners are the image's geographic bounds; the dashboard stretches the
image between them under the data layers.
