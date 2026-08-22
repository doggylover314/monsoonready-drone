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
import fence                                               # noqa: E402
import wifi                                                # noqa: E402

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


def read_waypoints_tolerant(path):
    """Same 'lat,lon per line, # comments' format run_mission.read_waypoints
    enforces, but tolerant: bad lines are reported, not fatal. The dashboard
    must still render 9 good waypoints when line 3 is mangled."""
    wps, errors = [], []
    try:
        f = open(os.path.expanduser(path))
    except OSError as exc:
        return [], [str(exc)]
    with f:
        for n, line in enumerate(f, 1):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            try:
                lat, lon = (float(v) for v in line.split(','))
                wps.append([lat, lon])
            except ValueError:
                errors.append(f"line {n}: expected 'lat,lon', got {line!r}")
    return wps, errors


def make_app(data_dir, control=None):
    app = Flask(__name__)
    data_dir = os.path.expanduser(data_dir)
    manual_dir = os.path.join(data_dir, 'manual_photos')
    layout_path = os.path.join(data_dir, 'layout.json')
    selftest_path = os.path.join(data_dir, 'selftest.json')
    fence_path = os.path.join(data_dir, 'fence.json')
    # control is None => read-only dashboard, exactly as before. A dict =>
    # the flight-control endpoints below are live. Opt-in on purpose: these
    # arm and fly an aircraft, so the capability must be asked for
    # (--enable-control), never inherited by accident.
    ctl = control

    @app.get('/')
    def index():
        resp = send_from_directory(STATIC_DIR, 'index.html')
        # Explicit, not left to Werkzeug defaults: the whole UI is this one
        # file, and a browser reusing a stale copy makes every fix look
        # broken (2026-08-16, satellite tiles). no-cache = revalidate every
        # load; unchanged file still answers 304, so nothing gets slower.
        resp.headers['Cache-Control'] = 'no-cache'
        return resp

    def ui_mtime():
        """mtime of index.html, sent with every /api/control poll. The page
        remembers the first value it sees and turns a later change into a
        RELOAD banner: an open tab survives a git pull untouched, and on
        2026-08-16 a pre-pull tab kept drawing the old grey map for hours
        while the board had been serving the fixed page all along."""
        try:
            return os.stat(os.path.join(STATIC_DIR, 'index.html')).st_mtime
        except OSError:
            return None

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
            return jsonify({'enabled': False, 'ui_mtime': ui_mtime()})
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
            'enabled': True, 'ui_mtime': ui_mtime(),
            'mission_running': bool(mission), 'mission_pids': mission,
            'testing': bool(testing),
            'selftest': st,
            'waypoints': ctl['waypoints'],
            'waypoints_exist': os.path.isfile(
                os.path.expanduser(ctl['waypoints'])),
            'conn': ctl['conn'], 'camera': ctl['camera'],
            'no_drop': ctl['no_drop'],
            'log': tail(os.path.join(LOG_DIR, 'run_mission.log')),
        })

    # ---------------- waypoints (map shows the planned route BEFORE flight
    # and lets it be edited by dragging, user 2026-08-16) -------------------

    @app.get('/api/waypoints')
    def api_waypoints_get():
        """The route the Start button would fly. Read-only; polling-free
        (fetched once at load and after each save), so not logged."""
        if not ctl:
            return jsonify({'available': False})
        path = os.path.expanduser(ctl['waypoints'])
        wps, errors = ([], []) if not os.path.isfile(path) \
            else read_waypoints_tolerant(path)
        return jsonify({'available': True, 'path': ctl['waypoints'],
                        'exists': os.path.isfile(path),
                        'waypoints': wps, 'errors': errors})

    @app.post('/api/waypoints')
    def api_waypoints_post():
        """Overwrite the waypoint file with the dragged route.

        Refused while a mission runs: run_mission read the file at launch,
        so a mid-flight save could NOT redirect the aircraft, and letting it
        appear to succeed would leave the operator believing it had."""
        if not ctl:
            abort(403)
        if find_pids('run_mission.py'):
            log.warn('WAYPOINTS save refused: mission running')
            return jsonify({'ok': False,
                            'error': 'mission running; the flying route was '
                                     'read at launch and cannot be changed '
                                     'from here'}), 409
        body = request.get_json(silent=True) or {}
        wps = body.get('waypoints')
        if (not isinstance(wps, list) or not 1 <= len(wps) <= 50
                or not all(isinstance(p, (list, tuple)) and len(p) == 2
                           and all(isinstance(v, (int, float)) for v in p)
                           and abs(p[0]) <= 90 and abs(p[1]) <= 180
                           for p in wps)):
            log.warn(f'WAYPOINTS save refused: bad body '
                     f'({type(wps).__name__}, '
                     f'{len(wps) if isinstance(wps, list) else "-"} items)')
            return jsonify({'ok': False,
                            'error': 'need 1..50 [lat,lon] pairs'}), 400
        path = os.path.expanduser(ctl['waypoints'])
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            f.write('# edited on the dashboard '
                    + time.strftime('%Y-%m-%d %H:%M:%S') + '\n')
            for lat, lon in wps:
                f.write(f'{lat:.7f},{lon:.7f}\n')
        os.replace(tmp, path)
        log(f'WAYPOINTS: wrote {len(wps)} waypoints to {path}')
        return jsonify({'ok': True, 'count': len(wps)})

    @app.post('/api/waypoints/generate')
    def api_waypoints_generate():
        """Build a survey route, from the FENCE when there is one.

        Two generators, same maths as make_waypoints.py (imported, not
        duplicated):

          * a fence is saved -> build_coverage fills its inside with rows,
            every waypoint at least `inset` m from the boundary, rows along
            `heading`, arranged to begin at the point the operator clicked.
            This needs NO LINK and NO GPS, so the route can be planned at
            home the night before, which is exactly what the from-aircraft
            generator could not do (user, 2026-08-19).
          * no fence -> the old from-where-it-stands serpentine, which does
            need the link and a 3D fix.

        Does NOT write the file either way: the route comes back for the
        operator to look at (and drag) on the map, and only Save writes it.
        """
        if not ctl:
            abort(403)
        b = request.get_json(silent=True) or {}
        try:
            rows = int(b.get('rows', 3))
            spacing = float(b.get('spacing', 12))
            length = float(b.get('length', 20))
            inset = float(b.get('inset', 4))
            heading = b.get('heading')
            heading = None if heading in (None, '') else float(heading)
            start = b.get('start') or None
            if start is not None:
                start = (float(start[0]), float(start[1]))
        except (TypeError, ValueError, IndexError):
            return jsonify({'ok': False, 'error': 'bad numbers'}), 400
        if not (1 <= rows <= 25) or not (1 <= spacing <= 50) \
                or not (2 <= length <= 500) or not (0 <= inset <= 50):
            return jsonify({'ok': False,
                            'error': 'rows 1-25, spacing 1-50 m, '
                                     'row length 2-500 m, keep-out 0-50 m'}), 400

        poly = fence.load(fence_path)
        if len(poly) >= 3:
            from make_waypoints import build_coverage
            wps, info = build_coverage(poly, heading, spacing, inset, start)
            if info.get('problem'):
                log.warn(f"GENERATE refused: {info['problem']}")
                return jsonify({'ok': False, 'error': info['problem']}), 400
            log(f"GENERATE: {len(wps)} waypoints covering the {len(poly)}-corner "
                f"fence, {info['rows']} rows on {info['lines']} lines "
                f"{spacing:g} m apart along {info['heading']} deg, "
                f"{inset:g} m keep-out, {info['path_m']} m of path, "
                f"{info['dropped']} piece(s) dropped as unreachable "
                f"(not saved yet)")
            return jsonify({'ok': True, 'waypoints': [list(p) for p in wps],
                            'heading': info['heading'], 'source': 'fence',
                            'rows': info['rows'], 'path_m': info['path_m'],
                            'dropped': info['dropped'], 'inset': inset})

        busy = find_pids('run_mission.py') or find_pids('test_everything.py')
        if busy:
            log.warn(f'GENERATE refused: port busy (pids {busy})')
            return jsonify({'ok': False,
                            'error': 'the Pixhawk link is busy (mission or '
                                     'self-test running)'}), 409
        try:
            from make_waypoints import build_serpentine
            from mavlink_io import MavIO
            io = MavIO(ctl['conn'], log=log)
            io.wait_ready(timeout=15)
            io.setup_streams()
            deadline = time.time() + 25
            while time.time() < deadline:
                io.step()
                t = io.tel
                if (t.lat is not None and (abs(t.lat) > 0.01
                                           or abs(t.lon) > 0.01)
                        and t.heading_deg is not None):
                    break
            else:
                log.warn('GENERATE: no GPS fix in 25 s')
                return jsonify({'ok': False,
                                'error': 'no GPS fix in 25 s: the aircraft '
                                         'needs sky view before a route can '
                                         'be made from its position'}), 400
            t = io.tel
            head = t.heading_deg if heading is None else heading
            wps = build_serpentine(t.lat, t.lon, head, rows, spacing, length)
        except Exception as exc:                        # noqa: BLE001
            log.error(f'GENERATE failed: {exc}')
            return jsonify({'ok': False, 'error': str(exc)}), 500
        finally:
            try:
                io.conn.close()
            except Exception:                           # noqa: BLE001
                pass
        log(f'GENERATE: {len(wps)} waypoints, {rows} rows x {length:g} m, '
            f'{spacing:g} m apart, heading {head:.0f} (not saved yet)')
        return jsonify({'ok': True, 'waypoints': [list(p) for p in wps],
                        'heading': round(head, 1), 'source': 'aircraft',
                        'from': [round(t.lat, 7), round(t.lon, 7)]})

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
        body = request.get_json(silent=True) or {}
        only = str(body.get('only', ''))
        if only and not re.match(r'^[a-z,\-]{1,60}$', only):
            abort(400)
        cmd = [ctl['python'],
               os.path.join(REPO, 'uno_q', 'test_everything.py'),
               '--conn', ctl['conn'], '--camera', str(ctl['camera']),
               '--out', selftest_path]
        if only:
            cmd += ['--only', only]
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

    # ---------------- Wi-Fi settings (user 2026-08-16; the WISP router
    # plan failed, so the board joins the phone hotspot directly) ----------
    # Passwords are never logged and never put in a command line (see
    # wifi.py). Switching networks kills the very connection serving this
    # page, which the panel says out loud before you press the button.

    @app.get('/api/wifi')
    def api_wifi():
        if not ctl:
            return jsonify({'enabled': False})
        return jsonify({'enabled': True, **wifi.status()})

    @app.post('/api/wifi/scan')
    def api_wifi_scan():
        if not ctl:
            abort(403)
        try:
            nets = wifi.scan()
        except wifi.WifiError as exc:
            log.warn(f'WIFI scan failed: {exc}')
            return jsonify({'ok': False, 'error': str(exc)}), 500
        log(f'WIFI: scan found {len(nets)} networks')
        return jsonify({'ok': True, 'networks': nets})

    @app.post('/api/wifi/connect')
    def api_wifi_connect():
        if not ctl:
            abort(403)
        if find_pids('run_mission.py'):
            log.warn('WIFI connect refused: mission running')
            return jsonify({'ok': False,
                            'error': 'a mission is running; switching the '
                                     'network now would cut the dashboard '
                                     'off mid-flight'}), 409
        b = request.get_json(silent=True) or {}
        ssid = str(b.get('ssid') or '').strip()
        saved = bool(b.get('saved'))
        password = b.get('password') or ''
        if not ssid or len(ssid) > 64:
            return jsonify({'ok': False, 'error': 'ssid required'}), 400
        # The password is deliberately absent from this line and from every
        # other line this program writes.
        log(f'WIFI: connecting to {ssid!r} (saved={saved}, '
            f'password {"given" if password else "none"})')
        try:
            note = (wifi.connect_saved(ssid) if saved
                    else wifi.connect_new(ssid, password))
        except wifi.WifiError as exc:
            log.error(f'WIFI connect to {ssid!r} failed: {exc}')
            return jsonify({'ok': False, 'error': str(exc)}), 500
        st = wifi.status()
        log(f'WIFI: {note}; now on {st.get("connection")} {st.get("ips")}')
        return jsonify({'ok': True, 'note': note, **st})

    # ---------------- geofence polygon (user 2026-08-16: the field is an
    # irregular shape hemmed in by trees, so it is drawn by hand) ----------

    @app.get('/api/fence')
    def api_fence_get():
        poly = fence.load(fence_path)
        return jsonify({'polygon': poly, 'count': len(poly),
                        'problem': fence.validate(poly)})

    @app.post('/api/fence')
    def api_fence_post():
        """Store the drawn polygon. Storing is NOT pushing: nothing reaches
        the aircraft until the operator presses Push."""
        if not ctl:
            abort(403)
        body = request.get_json(silent=True) or {}
        poly = body.get('polygon')
        if not isinstance(poly, list) or not all(
                isinstance(p, (list, tuple)) and len(p) == 2
                and all(isinstance(v, (int, float)) for v in p)
                for p in poly):
            return jsonify({'ok': False,
                            'error': 'need a list of [lat,lon] pairs'}), 400
        bad = fence.validate(poly)
        if bad:
            return jsonify({'ok': False, 'error': bad}), 400
        n = fence.save(poly, fence_path)
        log(f'FENCE: saved {n} corners to {fence_path} (not pushed yet)')
        return jsonify({'ok': True, 'count': n})

    @app.post('/api/fence/push')
    def api_fence_push():
        """Upload the polygon to the Pixhawk and read it back to prove it.

        Refused while the mission owns the serial port. Read-back is not
        decoration: MISSION_ACK only says the exchange was liked, and a
        fence the operator believes in but the aircraft never stored is
        worse than no fence at all.
        """
        if not ctl:
            abort(403)
        busy = find_pids('run_mission.py') or find_pids('test_everything.py')
        if busy:
            log.warn(f'FENCE push refused: link busy (pids {busy})')
            return jsonify({'ok': False,
                            'error': 'the Pixhawk link is busy (mission or '
                                     'self-test running)'}), 409
        poly = fence.load(fence_path)
        bad = fence.validate(poly)
        if bad:
            return jsonify({'ok': False, 'error': bad}), 400
        # VERIFY ON A FRESH LINK, NOT THE ONE THAT JUST UPLOADED (2026-08-19).
        # The dashboard's push kept reporting "the Pixhawk holds 0" while
        # `fence.py read` on the SAME board, the SAME link and the SAME code
        # listed every corner seconds later. The one structural difference was
        # the connection: the CLI reads down a socket that has just been
        # opened, the dashboard re-used the socket that had just finished a
        # mission-item exchange. So the verify step now does what the working
        # path does, and each stage is logged and named in the error, because
        # the previous message ("sent 7 but holds 0") could not distinguish an
        # upload that failed from a read that did.
        stage, io = 'connect', None
        try:
            from mavlink_io import MavIO
            io = MavIO(ctl['conn'], log=log)
            io.wait_ready(timeout=15)
            stage = 'upload'
            note = fence.push(io, poly, log=log)
            log(f'FENCE: {note}; reopening the link to verify')
        except Exception as exc:                        # noqa: BLE001
            log.error(f'FENCE {stage} failed: {exc}')
            return jsonify({'ok': False,
                            'error': f'{stage}: {exc}'}), 500
        finally:
            if io is not None:
                try:
                    io.conn.close()
                except Exception:                       # noqa: BLE001
                    pass

        stage, io = 'verify', None
        time.sleep(1.0)                 # let the port settle before reopening
        try:
            from mavlink_io import MavIO
            io = MavIO(ctl['conn'], log=log)
            io.wait_ready(timeout=15)
            back = fence.read_back(io, log=log)
        except Exception as exc:                        # noqa: BLE001
            log.error(f'FENCE verify failed after a good upload: {exc}')
            return jsonify({'ok': False,
                            'error': f'the upload was accepted but the '
                                     f'read-back could not run ({exc}). The '
                                     f'fence may well be loaded; check with '
                                     f'the fence read on the board.'}), 500
        finally:
            if io is not None:
                try:
                    io.conn.close()
                except Exception:                       # noqa: BLE001
                    pass

        if len(back) != len(poly):
            log.error(f'FENCE read-back mismatch: sent {len(poly)}, '
                      f'stored {len(back)}')
            return jsonify({'ok': False,
                            'error': f'the Pixhawk ACCEPTED {len(poly)} '
                                     f'corners but reads back {len(back)}. '
                                     f'The upload itself was not refused, so '
                                     f'this is storage or read-back, not your '
                                     f'drawing.'}), 500
        log(f'FENCE: {note}; read back {len(back)} corners on a fresh link')
        return jsonify({'ok': True, 'count': len(back),
                        'note': f'{len(back)} corners are in the Pixhawk '
                                f'(read back and matched)'})

    # ---------------- satellite tiles (user 2026-08-16: "the satellite
    # imagery isn't working at all") ----------------------------------------
    # The page tries Esri directly from the browser first. When the LAPTOP
    # has no route to the internet (at the field it is on the board's or the
    # phone's network, which may not forward), it falls back to this proxy,
    # which fetches from the BOARD instead and caches every tile on disk.
    # Consequences that make this worth the code: whichever machine has
    # internet, the map works; and once a site's tiles are cached, the map
    # keeps working with NO internet at all, which is the actual field case.
    tile_dir = os.path.join(data_dir, 'tiles')
    ESRI = ('https://server.arcgisonline.com/ArcGIS/rest/services/'
            'World_Imagery/MapServer/tile/{z}/{y}/{x}')
    tile_state = {'fail_logged': False}

    @app.get('/api/tile/<int:z>/<int:x>/<int:y>')
    def api_tile(z, x, y):
        if not (0 <= z <= 19) or not (0 <= x < 2 ** z) or not (0 <= y < 2 ** z):
            abort(404)
        path = os.path.join(tile_dir, str(z), str(x), f'{y}.jpg')
        if os.path.isfile(path):
            return send_from_directory(os.path.dirname(path),
                                       os.path.basename(path),
                                       max_age=86400)
        try:
            import urllib.request
            req = urllib.request.Request(
                ESRI.format(z=z, x=x, y=y),
                headers={'User-Agent': 'MonsoonReady-dashboard'})
            with urllib.request.urlopen(req, timeout=8) as r:
                blob = r.read()
        except Exception as exc:                        # noqa: BLE001
            # Once per dashboard run: a field with no internet would
            # otherwise write one line per tile per pan.
            if not tile_state['fail_logged']:
                tile_state['fail_logged'] = True
                log.warn(f'satellite tiles unavailable from the board: {exc}')
            abort(504)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'wb') as f:
            f.write(blob)
        os.replace(tmp, path)
        return send_from_directory(os.path.dirname(path),
                                   os.path.basename(path), max_age=86400)

    # ---------------- photo browser (read-only, user 2026-08-16) -----------
    # 'auto' = every frame the worker captured (future training data);
    # 'manual' = the dashboard button's shots. Names are IST timestamps, so
    # reverse-sorted = newest first. Available without --enable-control:
    # viewing photos is as read-only as viewing missions.
    photo_dirs = {'auto': os.path.join(data_dir, 'photos'),
                  'manual': manual_dir}

    @app.get('/api/photos/<which>')
    def api_photos_list(which):
        d = photo_dirs.get(which)
        if not d:
            abort(404)
        try:
            names = sorted((f for f in os.listdir(d) if f.endswith('.jpg')),
                           reverse=True)
        except OSError:
            names = []
        off = max(0, request.args.get('offset', 0, type=int))
        lim = min(60, max(1, request.args.get('limit', 12, type=int)))
        return jsonify({'total': len(names), 'files': names[off:off + lim]})

    @app.get('/api/photos/<which>/<name>')
    def api_photos_file(which, name):
        d = photo_dirs.get(which)
        if not d or not re.match(r'^[A-Za-z0-9_.\-]+\.jpg$', name):
            abort(404)
        return send_from_directory(d, name)

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

    # No per-request lines (user, 2026-08-16): werkzeug logs every GET at
    # INFO, which is 1200 poll lines an hour into the captured log. Errors
    # still surface. Control actions are logged explicitly above.
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)

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
