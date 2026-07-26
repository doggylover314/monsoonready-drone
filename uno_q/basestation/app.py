"""MonsoonReady base station — Flask server for the post-mission dashboard.

Runs on the UNO Q after landing (auto-launched by mission.py via
MissionConfig.basestation_cmd, or started by hand). Read-only: it serves the
per-mission JSONL logs written by uno_q/missionlog.py plus one static HTML
dashboard. No MAVLink, no writes.

    ~/venv/bin/python app.py --data-dir ~/monsoonready_data --port 8080

Reachable on the tailnet at http://drone:8080 and publicly through the
cloudflared tunnel at https://drone.reysen.net (tunnel terminates on the
board and forwards to localhost:8080; token in /etc/cloudflared).

Optional site image background: put an aerial screenshot in the data dir plus
site_image.json beside it:
    {"file": "site.png", "lat_top": .., "lat_bottom": ..,
     "lon_left": .., "lon_right": ..}
Corners are the geographic bounds of the (north-up) image.
"""

import argparse
import json
import os
import re

from flask import Flask, abort, jsonify, send_from_directory

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
MISSION_ID_RE = re.compile(r'^[A-Za-z0-9_\-]+$')


def missions_dir(data_dir):
    return os.path.join(data_dir, 'missions')


def list_mission_ids(data_dir):
    d = missions_dir(data_dir)
    if not os.path.isdir(d):
        return []
    ids = [f[len('mission_'):-len('.jsonl')] for f in os.listdir(d)
           if f.startswith('mission_') and f.endswith('.jsonl')]
    return sorted(ids)


def load_events(data_dir, mission_id):
    """Tolerant reader: a crash can truncate the last line; skip bad lines."""
    path = os.path.join(missions_dir(data_dir), f'mission_{mission_id}.jsonl')
    events = []
    with open(path) as f:
        for line in f:
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            ev['mission'] = mission_id
            events.append(ev)
    return events


def summarize(mission_id, events):
    by = lambda kind: [e for e in events if e['e'] == kind]
    end = by('mission_end')
    return {
        'id': mission_id,
        't_start': events[0]['t'] if events else None,
        't_end': events[-1]['t'] if events else None,
        'final': end[-1]['final'] if end else 'in progress / interrupted',
        'detections': len(by('detection')),
        'drops': len(by('drop')),
        'aborts': len(by('abort')),
        'fixes': len(by('fix')),
    }


def make_app(data_dir):
    app = Flask(__name__)
    data_dir = os.path.expanduser(data_dir)

    @app.get('/')
    def index():
        return send_from_directory(STATIC_DIR, 'index.html')

    @app.get('/api/missions')
    def api_missions():
        out = [summarize(mid, load_events(data_dir, mid))
               for mid in list_mission_ids(data_dir)]
        return jsonify(out)

    @app.get('/api/events/<mission_id>')
    def api_events(mission_id):
        if mission_id == 'all':
            evs = []
            for mid in list_mission_ids(data_dir):
                evs.extend(load_events(data_dir, mid))
            return jsonify(evs)
        if (not MISSION_ID_RE.match(mission_id)
                or mission_id not in list_mission_ids(data_dir)):
            abort(404)
        return jsonify(load_events(data_dir, mission_id))

    @app.get('/api/site_meta')
    def api_site_meta():
        p = os.path.join(data_dir, 'site_image.json')
        if not os.path.isfile(p):
            abort(404)
        with open(p) as f:
            return jsonify(json.load(f))

    @app.get('/api/site_image')
    def api_site_image():
        p = os.path.join(data_dir, 'site_image.json')
        if not os.path.isfile(p):
            abort(404)
        with open(p) as f:
            name = os.path.basename(json.load(f)['file'])
        return send_from_directory(data_dir, name)

    return app


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8080)
    args = ap.parse_args()
    make_app(args.data_dir).run(host=args.host, port=args.port)


if __name__ == '__main__':
    main()
