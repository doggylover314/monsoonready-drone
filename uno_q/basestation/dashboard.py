"""MonsoonReady dashboard - Flask server on the UNO Q (was app.py; renamed
2026-08-16 so the script and its log carry the same name: dashboard.py ->
~/logs/dashboard.log).

Serves the mission map and, with --enable-control, the flight-control panel:
start/stop the mission, run the no-motors self-test, take a manual photo.
Started BY HAND (SCOPE RULES 6: nothing on the board runs automatically):

    ~/venv/bin/python uno_q/basestation/dashboard.py --enable-control \
        --waypoints wp_field.txt

Every control action, launch attempt, and failure reason goes to
~/logs/dashboard.log (SCOPE RULES 1). Plain page GETs and the 3 s status
poll are NOT logged (user, 2026-08-16: the dashboard is for humans, not a
request ledger).

Mission data = per-flight JSONL from uno_q/missionlog.py under
~/monsoonready_data; schema changes happen in missionlog.py ONLY.

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
import time

from flask import Flask, abort, jsonify, request, send_from_directory

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from boardlog import BoardLog                              # noqa: E402
import detect_worker as dw                                 # noqa: E402

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
MISSION_ID_RE = re.compile(r'^[A-Za-z0-9_\-]+$')
# <repo>/, resolved from this file so it is right wherever the clone lives.
REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
LOG_DIR = os.path.expanduser('~/logs')

log = None   # BoardLog, bound in main() (module-level so handlers see it)


def find_pids(needle):
    """PIDs of PYTHON processes whose command line contains `needle`.

    Reads /proc rather than tracking Popen handles, so the answer stays right
    across a dashboard restart and covers a process someone started by
    hand over SSH. Linux-only, which is what the board is.

    The 'python' requirement is not cosmetic: without it an editor open on
    run_mission.py, or a `grep run_mission.py`, counts as a running mission,
    and the consequence is the START button refusing with "a mission is
    already running" in the middle of the take. Excludes this process by pid
    so the server can never find itself.
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


def newest_jpg(d):
    try:
        files = [f for f in os.listdir(d) if f.endswith('.jpg')]
    except OSError:
        return None
    return max(files) if files else None       # names are IST timestamps


def make_app(data_dir, control=None):
    app = Flask(__name__)
    data_dir = os.path.expanduser(data_dir)
    manual_dir = os.path.join(data_dir, 'manual_photos')
    layout_path = os.path.join(data_dir, 'layout.json')
    selftest_path = os.path.join(data_dir, 'selftest.json')
    # control is None => read-only dashboard, exactly as before. A dict =>
    # the flight-control endpoints below are live. Opt-in on purpose: these
    # arm and fly an aircraft, so the capability must be asked for
    # (--enable-control), never inherited by accident.
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

    # ---------------- layout (server-stored so BOTH laptops see the same
    # arrangement, user 2026-08-16; survives reboots and refreshes) ---------

    @app.get('/api/layout')
    def api_layout_get():
        if not os.path.isfile(layout_path):
            return jsonify({})
        with open(layout_path) as f:
            return jsonify(json.load(f))

    @app.post('/api/layout')
    def api_layout_post():
        body = request.get_json(silent=True)
        if not isinstance(body, dict) or len(json.dumps(body)) > 100_000:
            abort(400)
        tmp = layout_path + '.tmp'
        os.makedirs(data_dir, exist_ok=True)
        with open(tmp, 'w') as f:
            json.dump(body, f)
        os.replace(tmp, layout_path)
        return jsonify({'ok': True})

    # ---------------- flight control (only when --enable-control) ----------

    @app.get('/api/control')
    def api_control():
        """Always present so the dashboard can grey the panel out when
        control is off, rather than guessing from a 404. Polled every 3 s;
        deliberately NOT logged."""
        if not ctl:
            return jsonify({'enabled': False})
        mission = find_pids('run_mission.py')
        testing = find_pids('test_everything.py')
        st = None
        try:
            st_m = os.stat(selftest_path).st_mtime
            with open(selftest_path) as f:
                st = json.load(f)
            st['age_s'] = round(time.time() - st_m, 1)
        except (OSError, ValueError):
            pass
        return jsonify({
            'enabled': True,
            'mission_running': bool(mission), 'mission_pids': mission,
            'testing': bool(testing),
            'selftest': st,
            'waypoints': ctl['waypoints'],
            'waypoints_exist': os.path.isfile(
                os.path.expanduser(ctl['waypoints'])),
            'conn': ctl['conn'], 'camera': ctl['camera'],
            'no_drop': ctl['no_drop'],
            'log': tail(os.path.join(LOG_DIR, 'run_mission.log')),
            'manual_photo': newest_jpg(manual_dir),
        })

    @app.post('/api/control/start')
    def api_start():
        """Launch run_mission detached. start_new_session is the same
        protection `setsid` gives on the command line: the mission must not
        die of a SIGHUP when this server or an ssh session goes away."""
        if not ctl:
            abort(403)
        if find_pids('run_mission.py'):
            log.warn('START refused: a mission is already running')
            return jsonify({'ok': False,
                            'error': 'a mission is already running'}), 409
        wp = os.path.expanduser(ctl['waypoints'])
        if not os.path.isfile(wp):
            log.warn(f'START refused: no waypoint file at {wp}')
            return jsonify({'ok': False,
                            'error': f'no waypoint file at {wp}: run '
                                     f'make_waypoints.py first'}), 400
        body = request.get_json(silent=True) or {}
        cmd = [ctl['python'], os.path.join(REPO, 'uno_q', 'run_mission.py'),
               '--conn', ctl['conn'], '--waypoints', wp,
               '--camera', str(ctl['camera']),
               '--data-dir', ctl['data_dir']]
        if ctl['no_drop'] or body.get('no_drop'):
            cmd.append('--no-drop')
        if body.get('dry_run'):
            cmd.append('--dry-run')
        # run_mission owns ~/logs/run_mission.log itself (boardlog); stdout
        # here would only duplicate it.
        p = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL,
                             start_new_session=True)
        log(f"START: pid {p.pid}: {' '.join(cmd)}")
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
            log.warn('STOP refused: no mission running')
            return jsonify({'ok': False, 'error': 'no mission running'}), 409
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError as exc:
                log.error(f'STOP: SIGTERM {pid} failed: {exc}')
                return jsonify({'ok': False, 'error': str(exc)}), 500
        log(f'STOP: SIGTERM -> {pids} (graceful RTL)')
        return jsonify({'ok': True, 'pids': pids})

    @app.post('/api/control/test')
    def api_test():
        """Run test_everything.py (30 s, no motors, no servos). Detached;
        the dashboard polls /api/control until selftest.json refreshes."""
        if not ctl:
            abort(403)
        if find_pids('run_mission.py'):
            log.warn('TEST refused: mission running')
            return jsonify({'ok': False,
                            'error': 'mission running; test would steal its '
                                     'camera and serial port'}), 409
        if find_pids('test_everything.py'):
            return jsonify({'ok': True, 'note': 'test already running'})
        cmd = [ctl['python'],
               os.path.join(REPO, 'uno_q', 'test_everything.py'),
               '--conn', ctl['conn'], '--camera', str(ctl['camera']),
               '--out', selftest_path]
        p = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL,
                             start_new_session=True)
        log(f'TEST: started pid {p.pid}')
        return jsonify({'ok': True, 'pid': p.pid})

    @app.post('/api/control/photo')
    def api_photo():
        """Manual photo (user spec 2026-08-16): saved to manual_photos/ and
        shown on the dashboard, never processed by the detector.

        Two paths: a running detect_worker owns the camera, so ask IT via
        the request file; with no worker, open the camera directly here.
        """
        if not ctl:
            abort(403)
        os.makedirs(manual_dir, exist_ok=True)
        worker_alive = bool(find_pids('detect_worker.py'))
        if worker_alive:
            try:
                os.remove(dw.MANUAL_DONE)
            except OSError:
                pass
            with open(dw.MANUAL_REQ, 'w') as f:
                f.write(manual_dir)
            deadline = time.time() + 5.0
            while time.time() < deadline:
                try:
                    with open(dw.MANUAL_DONE) as f:
                        reply = json.load(f)
                    break
                except (OSError, ValueError):
                    time.sleep(0.2)
            else:
                log.error('PHOTO: worker did not answer within 5 s')
                return jsonify({'ok': False,
                                'error': 'worker did not answer'}), 500
            if not reply.get('ok'):
                log.error(f"PHOTO: worker failed: {reply.get('error')}")
                return jsonify({'ok': False,
                                'error': reply.get('error')}), 500
            name = os.path.basename(reply['path'])
            log(f'PHOTO: via worker: {name}')
            return jsonify({'ok': True, 'file': name})
        # No worker: open the camera ourselves, briefly.
        try:
            from camera import CameraError, open_camera
            import cv2
            cap, node = open_camera(ctl['camera'], log=log)
            for _ in range(8):            # let auto-exposure settle
                cap.grab()
            ok, frame = cap.read()
            cap.release()
            if not ok or frame is None:
                raise CameraError(f'{node} opened but gave no frame')
            name = 'manual_' + dw.stamp_name(0)
            cv2.imwrite(os.path.join(manual_dir, name), frame)
            log(f'PHOTO: direct: {name}')
            return jsonify({'ok': True, 'file': name})
        except Exception as exc:                        # noqa: BLE001
            log.error(f'PHOTO: direct capture failed: {exc}')
            return jsonify({'ok': False, 'error': str(exc)}), 500

    @app.get('/api/manual_photos/<name>')
    def api_manual_photo(name):
        if not re.match(r'^[A-Za-z0-9_.\-]+\.jpg$', name):
            abort(404)
        return send_from_directory(manual_dir, name)

    return app


def main():
    global log
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--enable-control', action='store_true',
                    help='expose the flight-control endpoints and the Flight '
                         'control panel. OFF by default: these arm and fly '
                         'the aircraft, and anyone on the LAN can reach this '
                         'server.')
    ap.add_argument('--waypoints', default='wp_field.txt',
                    help='waypoint file the Start button flies')
    ap.add_argument('--conn', default='auto',
                    help="run_mission --conn; 'auto' resolves the Pixhawk's "
                         "USB from /dev/serial/by-id")
    ap.add_argument('--camera', default='auto',
                    help="run_mission --camera; 'auto' resolves the USB "
                         "camera by name")
    ap.add_argument('--no-drop', action='store_true',
                    help='the Start button flies with the LOGGING dropper: '
                         'the whole loop, gate never moves. Use for the '
                         'rehearsal.')
    ap.add_argument('--mission-python', default=None,
                    help='interpreter for the mission (default: this one, '
                         'which is the venv that is already running flask)')
    args = ap.parse_args()

    log = BoardLog('dashboard')
    log(f"===== dashboard starting: {' '.join(sys.argv)} =====")

    control = None
    if args.enable_control:
        control = {
            'python': args.mission_python or sys.executable,
            'waypoints': args.waypoints, 'conn': args.conn,
            'camera': args.camera, 'no_drop': args.no_drop,
            'data_dir': args.data_dir,
        }
        log(f'FLIGHT CONTROL ENABLED on port {args.port}: anyone who can '
            f'reach this host can start and stop the mission')
    make_app(args.data_dir, control).run(host=args.host, port=args.port)


if __name__ == '__main__':
    main()
