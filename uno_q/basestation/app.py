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
import signal
import subprocess
import sys

from flask import Flask, abort, jsonify, request, send_from_directory

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
MISSION_ID_RE = re.compile(r'^[A-Za-z0-9_\-]+$')
# <repo>/, resolved from this file so it is right wherever the clone lives.
REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MISSION_LOG = os.path.expanduser('~/mission.log')
PUMP_LOG = os.path.expanduser('~/pump.log')


def find_pids(needle):
    """PIDs of PYTHON processes whose command line contains `needle`.

    Reads /proc rather than tracking Popen handles, so the answer stays right
    across a base-station restart and covers a process someone started by
    hand over SSH. Linux-only, which is what the board is.

    The 'python' requirement is not cosmetic: without it an editor open on
    run_mission.py, or a `grep run_mission.py`, counts as a running mission,
    and the consequence is the START button refusing with "a mission is
    already running" in the middle of the take. Excludes this process and
    its own children by pid so the server can never find itself.
    """
    out = []
    me = os.getpid()
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        except OSError:
            continue
        if needle in cmd and 'python' in cmd.lower():
            out.append(int(entry))
    return out


def tail(path, lines=14):
    try:
        with open(path, errors='replace') as f:
            return ''.join(f.readlines()[-lines:])
    except OSError:
        return ''


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
        # Only gates that actually actuated count as treatments, matching the
        # dashboard tile and mission_end's dropper.succeeded. ok absent means
        # true (pre-2026-08-01 logs predate the field).
        'drops': len([e for e in by('drop') if e.get('ok', True)]),
        'aborts': len(by('abort')),
        'fixes': len(by('fix')),
    }


def make_app(data_dir, control=None):
    app = Flask(__name__)
    data_dir = os.path.expanduser(data_dir)
    # control is None => read-only dashboard, exactly as before. A dict =>
    # the flight-control endpoints below are live. Opt-in on purpose: these
    # arm and fly an aircraft, so the capability must be asked for
    # (app.py --enable-control), never inherited by accident.
    ctl = control

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

    # ---------------- flight control (only when --enable-control) ----------

    @app.get('/api/control')
    def api_control():
        """Always present so the dashboard can grey the panel out when
        control is off, rather than guessing from a 404."""
        if not ctl:
            return jsonify({'enabled': False})
        mission = find_pids('run_mission.py')
        pump = find_pids('mav_shovel_pump.py')
        return jsonify({
            'enabled': True,
            'mission_running': bool(mission), 'mission_pids': mission,
            'pump_running': bool(pump), 'pump_pids': pump,
            'waypoints': ctl['waypoints'],
            'waypoints_exist': os.path.isfile(
                os.path.expanduser(ctl['waypoints'])),
            'conn': ctl['conn'], 'camera': ctl['camera'],
            'no_drop': ctl['no_drop'],
            'log': tail(MISSION_LOG),
            'pump_log': tail(PUMP_LOG, 6),
        })

    @app.post('/api/control/pump')
    def api_pump():
        if not ctl:
            abort(403)
        if find_pids('mav_shovel_pump.py'):
            return jsonify({'ok': True, 'note': 'pump already running'})
        py = ctl['python']
        with open(PUMP_LOG, 'a') as log:
            subprocess.Popen([py, os.path.join(REPO, 'uno_q',
                                               'mav_shovel_pump.py')],
                             cwd=REPO, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
        app.logger.warning('CONTROL: pump started')
        return jsonify({'ok': True})

    @app.post('/api/control/start')
    def api_start():
        """Launch run_mission detached. start_new_session is the same
        protection `setsid` gives on the command line: the mission must not
        die of a SIGHUP when this server or an ssh session goes away."""
        if not ctl:
            abort(403)
        if find_pids('run_mission.py'):
            return jsonify({'ok': False,
                            'error': 'a mission is already running'}), 409
        wp = os.path.expanduser(ctl['waypoints'])
        if not os.path.isfile(wp):
            return jsonify({'ok': False,
                            'error': f'no waypoint file at {wp}: run '
                                     f'make_waypoints.py first'}), 400
        if not find_pids('mav_shovel_pump.py'):
            return jsonify({'ok': False,
                            'error': 'the pump is not running: start it '
                                     'first (button above)'}), 400
        body = request.get_json(silent=True) or {}
        cmd = [ctl['python'], os.path.join(REPO, 'uno_q', 'run_mission.py'),
               '--conn', ctl['conn'], '--waypoints', wp,
               '--camera', str(ctl['camera']),
               '--data-dir', ctl['data_dir']]
        if ctl['no_drop'] or body.get('no_drop'):
            cmd.append('--no-drop')
        if body.get('dry_run'):
            cmd.append('--dry-run')
        with open(MISSION_LOG, 'a') as log:
            log.write(f"\n===== dashboard start: {' '.join(cmd)} =====\n")
            log.flush()
            p = subprocess.Popen(cmd, cwd=REPO, stdout=log,
                                 stderr=subprocess.STDOUT,
                                 start_new_session=True)
        app.logger.warning('CONTROL: mission started pid %s', p.pid)
        return jsonify({'ok': True, 'pid': p.pid})

    @app.post('/api/control/stop')
    def api_stop():
        """SIGTERM, never SIGKILL. run_mission's handler treats it as 'wind
        up at the next tick', which commands RTL through the normal path. A
        kill would abandon the aircraft holding its last setpoint."""
        if not ctl:
            abort(403)
        pids = find_pids('run_mission.py')
        if not pids:
            return jsonify({'ok': False, 'error': 'no mission running'}), 409
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError as exc:
                return jsonify({'ok': False, 'error': str(exc)}), 500
        app.logger.warning('CONTROL: SIGTERM -> %s (graceful RTL)', pids)
        return jsonify({'ok': True, 'pids': pids})

    return app


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--enable-control', action='store_true',
                    help='expose the flight-control endpoints and the Flight '
                         'control panel. OFF by default: these arm and fly '
                         'the aircraft, and anyone on the LAN can reach this '
                         'server.')
    ap.add_argument('--waypoints', default='wp_farm.txt',
                    help='waypoint file the Start button flies')
    ap.add_argument('--conn', default='udpin:127.0.0.1:14555')
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--no-drop', action='store_true',
                    help='the Start button flies with the LOGGING dropper: '
                         'the whole loop, gate never moves. Use for the '
                         'rehearsal.')
    ap.add_argument('--mission-python', default=None,
                    help='interpreter for the mission (default: this one, '
                         'which is the venv that is already running flask)')
    args = ap.parse_args()

    control = None
    if args.enable_control:
        control = {
            'python': args.mission_python or sys.executable,
            'waypoints': args.waypoints, 'conn': args.conn,
            'camera': args.camera, 'no_drop': args.no_drop,
            'data_dir': args.data_dir,
        }
        print('FLIGHT CONTROL ENABLED. Anyone who can reach '
              f'http://<this-host>:{args.port} can start and stop the '
              'mission.')
    make_app(args.data_dir, control).run(host=args.host, port=args.port)


if __name__ == '__main__':
    main()
