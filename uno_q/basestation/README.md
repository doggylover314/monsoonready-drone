# The dashboard

A small Flask app on the UNO Q. It started as a post-landing report server and
grew into the thing that actually flies the aircraft, so the old "read-only by
design" line no longer holds and this file says so plainly. It now draws the map
and the flight history, and it also arms, starts and stops the mission, runs the
self-test, takes photos, edits and pushes the geofence, generates survey routes
and joins the board to a wifi network.

Flight history still comes from one place, the per-mission JSONL written by
`uno_q/missionlog.py`, so a mission logged is a mission reportable and the same
files double as documentation evidence.

## Data flow

```
mission.py --recorder=MissionLog--> <data_dir>/missions/mission_<id>.jsonl
                                            |
dashboard.py <------ reads --------- -------+
   |
   +-- /api/missions          per-flight summaries
   +-- /api/events/<id|all>   event stream, one flight or accumulated
   +-- /api/site_meta         optional site-image bounds
   +-- /api/site_image        optional site-image file
   +-- /                      static/index.html
```

Everything above is read-only over the log files. The control endpoints are a
separate group, `/api/control` and its `start`, `stop` and `test` posts, plus
`/api/waypoints`, `/api/fence`, `/api/fence/push`, `/api/photo` and `/api/wifi`.
Those talk to the Pixhawk or launch processes, and they are refused whenever a
mission or a self-test already owns the serial port. `mission.py` auto-launches
the server on DONE or STANDDOWN when `MissionConfig.basestation_cmd` is set, and
the SITL tests leave it unset.

## Files

`dashboard.py` is the server, defaulting to `--data-dir ~/monsoonready_data
--host 0.0.0.0 --port 8080`. `static/index.html` is the whole front end, vanilla
JS and canvas with no external assets, so it works offline.
`gen_fake_mission.py` writes two plausible fake flights through `MissionLog` for
development without hardware.

## Running it

On the UNO Q, where Flask is a one-time install and offline afterwards:

```bash
~/venv/bin/pip install flask
```

```bash
~/venv/bin/python ~/uno_q/basestation/dashboard.py
```

View it at `http://drone:8080` on the tailnet, or at https://drone.reysen.net
through the cloudflared tunnel running on the board, which forwards to
localhost:8080 with its token in `/etc/cloudflared`. The public URL needs the
board to have internet at viewing time.

On a dev machine, without a flight:

```bash
python3 uno_q/basestation/gen_fake_mission.py
```

```bash
python3 uno_q/basestation/dashboard.py
```

## The map

The default view accumulates every flight and a selector narrows to one. Layers
are checkboxes: a heatmap of detection density on a sequential blue ramp, a blue
dot per detector fire with confidence in the tooltip, a green circle and check
where a drop released, a red X with a reason where a descent aborted, a 1 Hz
breadcrumb polyline per flight, and an optional georeferenced site image. A live
aircraft marker appears on its own layer during a running mission and never for
a past flight.

Detections within 5 m of each other seen in two or more distinct flights get a
dashed ring. That is the standing-water-confirmed-by-persistence framing, and it
is computed only on the accumulated view, because a single flight cannot show
persistence.

There are also stat tiles, hover tooltips, an event table as a text alternative
to the map, a scale bar, a north arrow, a theme following the OS, and a 15 s
poll so a newly landed flight appears while the page is open.

## Site image background

Drop a north-up aerial screenshot into the data dir along with
`site_image.json` giving its geographic corner bounds:

```json
{"file": "site.png", "lat_top": 12.9720, "lat_bottom": 12.9712,
 "lon_left": 77.5942, "lon_right": 77.5951}
```

The dashboard stretches the image between those corners under the data layers.
Without the file it falls back to the schematic canvas, which needs no network
and no calibration.

## Design notes

One event per line in JSONL, flushed per event, so a crash mid-mission loses at
most the line being written and the reader skips truncated lines. The schema is
defined in `missionlog.py` and only there.

The server never aggregates across requests. Every response is recomputed from
the files, which means `scp`-ing a mission file into the data dir, or deleting
one, shows up immediately.

The front end is one file with zero external requests, so it renders identically
over the tailnet, through the tunnel, or opened from disk next to a copied data
dir.

Flask runs threaded. It used to serve one request at a time, and a live-position
open could take up to 8 s, which queued a START click behind it.

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| The public URL gives 502 | The tunnel is up but `dashboard.py` is not running on the board |
| "No mission data yet" | Data dir empty or wrong `--data-dir`. Check `<data_dir>/missions/*.jsonl` exists. |
| Site image checkbox missing | No `site_image.json` in the data dir, or its `file` does not exist |
| A flight shows as in progress or interrupted | Its JSONL has no `mission_end` line, so the mission crashed or is still flying |
| A control button is refused | A mission or self-test owns the serial port. Only one client at a time gets the Pixhawk. |
| ARM greys out again on its own | Expected. The armed state expires after 8 s and START has to be pressed inside that window. |
