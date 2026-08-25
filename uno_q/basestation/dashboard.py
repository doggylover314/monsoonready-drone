"""Flask mission map and flight-control server on UNO Q. Logs all control
actions and launch failures to ~/logs/dashboard.log (plain GETs excluded).

Mission data: per-flight JSONL in ~/monsoonready_data; schema changes in
missionlog.py only. Optional aerial background: site_image.json with
geographic bounds (corners as lat/lon).
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
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
# Everything that opens the Pixhawk serial port exclusively. The dashboard's
# own map link must stand down for any of them: losing that race aborts a
# flight, or fails a param push with a busy port.
# check_log.py is deliberately absent: it opens a .bin file, never the port.
PIXHAWK_TOOLS = ('run_mission.py', 'test_everything.py', 'parameters.py',
                 'bench.py', 'level_cal.py', 'wiring_check.py', 'fence.py',
                 'servo_jog.py', 'flow_test.py', 'esp32_mute.py',
                 'ring_channels.py')

log = None   # BoardLog, bound in main() for handler scope


def find_pids(needle):
    """PIDs of Python processes with needle in command line (reads /proc,
    requires 'python' to exclude editors/greps, excludes self).
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
        with open(path, errors='replace', encoding='utf-8') as f:
            return ''.join(f.readlines()[-lines:])
    except OSError:
        return ''


# Mission settings the launch buttons may override, with the range each is
# clamped to. These go onto run_mission.py's command line, so a value that
# survives here reaches the aircraft: the bounds are the last check.
#   survey_alt  1 m floor is below the rangefinder's RNGFND1_MIN 0.2 plus
#               GNDCLR; 40 m is India's DGCA ceiling for this class.
#   photo_hold  0 disables the hold entirely (mission.py:453 tests > 0).
#               30 s is far past useful and only guards a typo.
#   conf        the detector's own threshold, a probability.
# Sanity bound on a saved route, not a policy. The old value was 50, written
# 2026-08-16 when routes were dragged out by hand a dozen points at a time.
# Survey altitude has since come down to 5 m, where the across-track footprint
# is 3.00 m and the generator legitimately produces hundreds of points for a
# modest fence, so 50 refused perfectly good routes. Nothing downstream cares:
# run_mission.py reads the file with no limit and the points are flown as
# GUIDED targets one at a time, never uploaded as an FC mission, so no
# autopilot storage limit applies either. What actually limits a route is
# flight time on one battery, and that is the operator's judgement, not a
# number this endpoint can know.
MAX_WAYPOINTS = 2000

# Ceiling on the satellite-imagery correction, ~111 m. Supplier
# georeferencing error is metres, not hundreds of metres, so anything past
# this means the operator clicked the wrong feature and the map would end up
# further from the truth than it started.
SAT_OFFSET_MAX_DEG = 0.001


def read_sat_offset(data_dir):
    """Stored imagery correction, or zeros. Never raises: a missing or
    corrupt file must leave the map usable, just uncorrected."""
    try:
        with open(os.path.join(data_dir, 'sat_offset.json'),
                  encoding='utf-8') as f:
            d = json.load(f)
        return {'lat': float(d['lat']), 'lon': float(d['lon'])}
    except (OSError, ValueError, KeyError, TypeError):
        return {'lat': 0.0, 'lon': 0.0}

MISSION_LIMITS = {
    'survey_alt': (1.0, 40.0),
    'photo_hold': (0.0, 30.0),
    'conf': (0.05, 0.95),
    # Must sit UNDER the lateral offset or the crossing is skipped and the
    # gate opens beside the water. The ceiling is deliberately below the
    # 1.5 m default offset so this box cannot recreate the 2026-08-25 bug.
    'cross_min': (0.0, 1.0),
}


def mission_opt(body, key, fallback):
    """One numeric launch override from the request body, or `fallback`.

    Absent or null means "use the dashboard's own default", which is what an
    older page that does not send the field will do. Anything present must be
    a real number inside MISSION_LIMITS; a bad one raises rather than being
    silently clamped, because quietly flying 5 m when the operator asked for
    50 is worse than refusing to launch.
    """
    if key not in body or body[key] is None:
        return fallback
    lo, hi = MISSION_LIMITS[key]
    try:
        val = float(body[key])
    except (TypeError, ValueError):
        raise ValueError(f'{key} must be a number, got {body[key]!r}')
    if not math.isfinite(val) or not lo <= val <= hi:
        raise ValueError(f'{key} must be between {lo:g} and {hi:g}, got {val:g}')
    return val


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
    with open(path, encoding='utf-8') as f:
        for line in f:
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            ev['mission'] = mission_id
            events.append(ev)
    return events


def last_fix(data_dir, mission_id, tail_bytes=65536):
    """Newest 'fix' record of a running mission, or None.

    Reads only the tail of the JSONL. The map polls this once a second while
    a mission flies, and load_events() would re-read and re-parse the whole
    growing file every time, on the board, during the flight.
    """
    path = os.path.join(missions_dir(data_dir), f'mission_{mission_id}.jsonl')
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            start = max(0, f.tell() - tail_bytes)
            f.seek(start)
            chunk = f.read()
    except OSError:
        return None
    lines = chunk.split(b'\n')
    if start:
        lines = lines[1:]           # a seek lands mid-line; drop the fragment
    for line in reversed(lines):
        if b'"fix"' not in line:
            continue
        try:
            ev = json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue                # truncated final line during a write
        if ev.get('e') == 'fix' and ev.get('lat') is not None:
            return ev
    return None


def summarize(mission_id, events):
    by = lambda kind: [e for e in events if e['e'] == kind]
    end = by('mission_end')
    return {
        'id': mission_id,
        't_start': events[0]['t'] if events else None,
        't_end': events[-1]['t'] if events else None,
        'final': end[-1]['final'] if end else 'in progress / interrupted',
        'detections': len(by('detection')),
        # Only actuated gates counted (ok defaults true for pre-2026 logs)
        'drops': len([e for e in by('drop') if e.get('ok', True)]),
        'aborts': len(by('abort')),
        'fixes': len(by('fix')),
    }


def read_waypoints_tolerant(path):
    """Parse lat,lon waypoints (same format as run_mission, tolerant of
    bad lines; returns errors list too).
    """
    wps, errors = [], []
    try:
        f = open(os.path.expanduser(path), encoding='utf-8')
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
    # control: None=read-only dashboard; dict=live flight-control endpoints
    ctl = control

    @app.get('/')
    def index():
        resp = send_from_directory(STATIC_DIR, 'index.html')
        # Revalidate on every load to prevent stale UI after git pull
        resp.headers['Cache-Control'] = 'no-cache'
        return resp

    def ui_mtime():
        """Timestamp of index.html sent with /api/control poll; client
        detects changes and prompts reload.
        """
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

    # Satellite imagery is georeferenced by its supplier, and at this site
    # Esri's is off by a constant vector: the aircraft's own GPS and the
    # image disagree by several metres in a fixed direction (confirmed
    # 2026-08-24 by carrying the aircraft and watching the dot track
    # correctly while staying offset). That matters far beyond cosmetics,
    # because a fence drawn by clicking the image inherits the error, so the
    # aircraft is refused arming for breaching a fence it is standing inside,
    # and any route generated in that fence surveys the wrong patch.
    #
    # The correction is stored per site, on the board, not in one browser:
    # whoever opens the dashboard must see the same corrected map, and the
    # fence that gets pushed to the Pixhawk depends on it.
    @app.get('/api/sat_offset')
    def api_sat_offset_get():
        return jsonify(read_sat_offset(data_dir))

    @app.post('/api/sat_offset')
    def api_sat_offset_post():
        if not ctl:
            abort(403)
        body = request.get_json(silent=True) or {}
        try:
            dlat, dlon = float(body['lat']), float(body['lon'])
        except (KeyError, TypeError, ValueError):
            return jsonify({'ok': False,
                            'error': 'need numeric lat and lon'}), 400
        if not (math.isfinite(dlat) and math.isfinite(dlon)) \
                or abs(dlat) > SAT_OFFSET_MAX_DEG \
                or abs(dlon) > SAT_OFFSET_MAX_DEG:
            log.warn(f'SAT OFFSET refused: {dlat}, {dlon}')
            return jsonify({'ok': False,
                            'error': f'offset must be within '
                                     f'{SAT_OFFSET_MAX_DEG} deg (about '
                                     f'{SAT_OFFSET_MAX_DEG * 111320:.0f} m); '
                                     f'a bigger one means the wrong point was '
                                     f'clicked'}), 400
        path = os.path.join(data_dir, 'sat_offset.json')
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump({'lat': dlat, 'lon': dlon,
                       'set_at': time.strftime('%Y-%m-%d %H:%M:%S')}, f)
        os.replace(tmp, path)
        north = dlat * 111320.0
        log(f'SAT OFFSET: imagery shifted {dlat:.7f},{dlon:.7f} deg '
            f'({north:+.1f} m north) -> {path}')
        return jsonify({'ok': True, 'lat': dlat, 'lon': dlon})

    @app.get('/api/site_meta')
    def api_site_meta():
        p = os.path.join(data_dir, 'site_image.json')
        if not os.path.isfile(p):
            abort(404)
        with open(p, encoding='utf-8') as f:
            return jsonify(json.load(f))

    @app.get('/api/site_image')
    def api_site_image():
        p = os.path.join(data_dir, 'site_image.json')
        if not os.path.isfile(p):
            abort(404)
        with open(p, encoding='utf-8') as f:
            name = os.path.basename(json.load(f)['file'])
        return send_from_directory(data_dir, name)

    # Layout: server-stored so both laptops see the same arrangement (user
    # 2026-08-16); survives reboots and refreshes.

    @app.get('/api/layout')
    def api_layout_get():
        if not os.path.isfile(layout_path):
            return jsonify({})
        with open(layout_path, encoding='utf-8') as f:
            return jsonify(json.load(f))

    @app.post('/api/layout')
    def api_layout_post():
        if not ctl:
            abort(403)
        body = request.get_json(silent=True)
        if not isinstance(body, dict) or len(json.dumps(body)) > 100_000:
            abort(400)
        tmp = layout_path + '.tmp'
        os.makedirs(data_dir, exist_ok=True)
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(body, f)
        os.replace(tmp, layout_path)
        return jsonify({'ok': True})

    # Flight control: only active when --enable-control is set.

    @app.get('/api/control')
    def api_control():
        """Control status; always present so dashboard detects disabled state
        (polled every 3 s, not logged).
        """
        if not ctl:
            return jsonify({'enabled': False, 'ui_mtime': ui_mtime()})
        mission = find_pids('run_mission.py')
        testing = find_pids('test_everything.py')
        st = None
        try:
            st_m = os.stat(selftest_path).st_mtime
            with open(selftest_path, encoding='utf-8') as f:
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
            # Seed values for the mission-settings boxes. Sent every poll so
            # a reloaded page shows what the next launch will actually use,
            # not whatever was last typed into a since-closed tab.
            'survey_alt': ctl['survey_alt'], 'conf': ctl['conf'],
            'photo_hold': ctl['photo_hold'],
            'cross_min': ctl['cross_min'],
            'log': tail(os.path.join(LOG_DIR, 'run_mission.log')),
        })

    # Waypoints: the map shows the planned route before flight and lets it
    # be edited by dragging (user 2026-08-16).

    # Live position: the mission writes fixes to its JSONL, so the map can
    # show the aircraft while flying, with no link of its own. Before launch
    # there is no mission and so no position, which is what the map lacked.
    # The dashboard opens its own link, but only while nothing else owns the
    # port: run_mission.py and test_everything.py both take it exclusively,
    # and losing that race would abort a flight.
    live = {'io': None, 'fix': None, 'at': 0.0, 'hold': 0.0,
            'lock': threading.Lock(), 'swap': threading.Lock()}

    def port_owner():
        """Name of the tool currently holding the Pixhawk port, or None."""
        return next((n for n in PIXHAWK_TOOLS if find_pids(n)), None)

    def live_close(why):
        # threaded=True means another request can be inside api_live_position
        # holding live['lock'] across an 8 s wait_ready. Take the handle out
        # under the same lock so two closers cannot both see it.
        with live['swap']:
            io = live['io']
            live['io'] = None
        if io is not None:
            try:
                io.conn.close()
            except Exception:                           # noqa: BLE001
                pass
            log(f'LIVE: link closed ({why})')

    def live_release(seconds, why):
        """Drop the link and refuse to reopen it for a while.

        Called just before spawning anything that needs the Pixhawk. Without
        the hold, the 3 s poll could reopen the port in the gap between the
        spawn and the child actually opening it.
        """
        live['hold'] = time.time() + seconds
        live_close(why)

    @app.get('/api/live_position')
    def api_live_position():
        """Where the aircraft is when no mission is running (polled)."""
        if not ctl:
            abort(403)
        busy = port_owner()
        if busy:
            live_close(f'{busy} owns the port')
            # The mission owns the link, so its JSONL is the only place the
            # position exists. Served from here too, so the map has ONE
            # once-a-second source in the air and on the ground instead of
            # waiting on the 3 s events refetch to move the aircraft.
            ids = list_mission_ids(data_dir)
            fix = last_fix(data_dir, ids[-1]) if ids else None
            return jsonify({'ok': True, 'source': 'mission', 'busy': busy,
                            'fix': fix,
                            'mission': ids[-1] if ids else None})
        if time.time() < live['hold']:
            return jsonify({'ok': True, 'source': 'holding'})
        if not live['lock'].acquire(blocking=False):
            return jsonify({'ok': True, 'source': 'link', 'fix': live['fix']})
        try:
            if live['io'] is None:
                from mavlink_io import MavIO
                io = MavIO(ctl['conn'], log=log)
                io.wait_ready(timeout=8)
                io.setup_streams()
                live['io'] = io
                log('LIVE: opened the Pixhawk link for the map')
            io = live['io']
            deadline = time.time() + 0.5
            while time.time() < deadline:
                io.step()
            t = io.tel
            if t.lat is not None and (abs(t.lat) > 0.01 or abs(t.lon) > 0.01):
                # hdop travels with the fix because it is the number that
                # explains a wrong-looking position, and on 2026-08-23 it
                # read 99.99 while nothing on this page said so.
                live['fix'] = {'lat': t.lat, 'lon': t.lon, 'alt': t.rel_alt_m,
                               'rng': t.rng_m if t.rng_valid else None,
                               'heading': t.heading_deg, 'mode': t.mode,
                               'armed': bool(t.armed), 'sats': t.sats,
                               'hdop': t.hdop}
                live['at'] = time.time()
            return jsonify({
                'ok': True, 'source': 'link', 'fix': live['fix'],
                'age_s': (round(time.time() - live['at'], 1)
                          if live['at'] else None)})
        except Exception as exc:                        # noqa: BLE001
            live_close(f'error: {exc}')
            return jsonify({'ok': False, 'source': 'link',
                            'error': str(exc)})
        finally:
            live['lock'].release()

    @app.get('/api/waypoints')
    def api_waypoints_get():
        """Route for Start button; fetched at load and after edits, not polled."""
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
        """Save dragged route; refused mid-flight because run_mission read
        the route at launch.
        """
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
        if (not isinstance(wps, list) or not 1 <= len(wps) <= MAX_WAYPOINTS
                or not all(isinstance(p, (list, tuple)) and len(p) == 2
                           and all(isinstance(v, (int, float)) for v in p)
                           and abs(p[0]) <= 90 and abs(p[1]) <= 180
                           for p in wps)):
            log.warn(f'WAYPOINTS save refused: bad body '
                     f'({type(wps).__name__}, '
                     f'{len(wps) if isinstance(wps, list) else "-"} items)')
            return jsonify({'ok': False,
                            'error': f'need 1..{MAX_WAYPOINTS} [lat,lon] '
                                     f'pairs'}), 400
        path = os.path.expanduser(ctl['waypoints'])
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write('# edited on the dashboard '
                    + time.strftime('%Y-%m-%d %H:%M:%S') + '\n')
            for lat, lon in wps:
                f.write(f'{lat:.7f},{lon:.7f}\n')
        os.replace(tmp, path)
        log(f'WAYPOINTS: wrote {len(wps)} waypoints to {path}')
        return jsonify({'ok': True, 'count': len(wps)})

    @app.post('/api/waypoints/generate')
    def api_waypoints_generate():
        """Generate survey route from saved fence (no link/GPS) or from
        aircraft position (requires GPS fix). Unsaved: operator can drag
        before Save writes file.
        """
        if not ctl:
            abort(403)
        b = request.get_json(silent=True) or {}
        try:
            rows = int(b.get('rows', 3))
            # spacing None/'' = derive from altitude for `overlap` m between rows
            spacing = b.get('spacing')
            spacing = None if spacing in (None, '') else float(spacing)
            length = float(b.get('length', 20))
            inset = float(b.get('inset', 4))
            overlap = float(b.get('overlap', 1.0))
            max_leg = b.get('max_leg')
            max_leg = None if max_leg in (None, '') else float(max_leg)
            heading = b.get('heading')
            heading = None if heading in (None, '') else float(heading)
            start = b.get('start') or None
            if start is not None:
                start = (float(start[0]), float(start[1]))
        except (TypeError, ValueError, IndexError):
            log.warn('GENERATE refused: a numeric field could not be read')
            return jsonify({'ok': False, 'error': 'bad numbers'}), 400
        if not (1 <= rows <= 25) \
                or (spacing is not None and not (1 <= spacing <= 50)) \
                or not (2 <= length <= 500) or not (0 <= inset <= 50) \
                or not (0 <= overlap <= 20) \
                or (max_leg is not None and not (0 <= max_leg <= 200)):
            return jsonify({'ok': False,
                            'error': 'rows 1-25, spacing 1-50 m, '
                                     'row length 2-500 m, keep-out 0-50 m, '
                                     'overlap 0-20 m, '
                                     'waypoint spacing 0-200 m'}), 400

        # Row and waypoint spacing derived from the camera footprint at
        # mission altitude, so `overlap` m is shared between adjacent frames
        # both across rows and along them. The camera is mounted rotated
        # (1280 px axis fore-aft); spacing_for_overlap knows.
        from make_waypoints import spacing_for_overlap
        alt = ctl.get('survey_alt', 5.0)
        row_rec, leg_rec = spacing_for_overlap(alt, overlap)
        if spacing is None:
            spacing = round(row_rec, 1)
        if max_leg is None:
            max_leg = leg_rec
        log(f"GENERATE: at {alt:g} m with {overlap:g} m overlap the frames "
            f"want rows {row_rec:.1f} m apart and waypoints {leg_rec:.1f} m "
            f"apart; using rows {spacing:g} m, waypoints {max_leg:.1f} m")

        poly = fence.load(fence_path)
        if len(poly) >= 3 and str(b.get('mode', '')) == 'centre':
            from make_waypoints import build_centreline, densify
            wps, info = build_centreline(poly, heading, inset, start)
            if info.get('problem'):
                log.warn(f"CENTRELINE refused: {info['problem']}")
                return jsonify({'ok': False, 'error': info['problem']}), 400
            wps = densify(wps, max_leg)
            log(f"CENTRELINE: {len(wps)} waypoints on one {info['path_m']} m "
                f"line down the middle of the {len(poly)}-corner fence along "
                f"{info['heading']} deg, {inset:g} m keep-out, waypoints "
                f"{max_leg:.1f} m apart, {info['dropped']} shorter piece(s) "
                f"ignored (not saved yet)")
            return jsonify({'ok': True, 'waypoints': [list(p) for p in wps],
                            'heading': info['heading'], 'source': 'centreline',
                            'rows': 1, 'path_m': info['path_m'],
                            'dropped': info['dropped'], 'inset': inset})

        if len(poly) >= 3:
            from make_waypoints import build_coverage, densify
            wps, info = build_coverage(poly, heading, spacing, inset, start)
            if info.get('problem'):
                log.warn(f"GENERATE refused: {info['problem']}")
                return jsonify({'ok': False, 'error': info['problem']}), 400
            wps = densify(wps, max_leg)
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

        busy = port_owner()
        if busy:
            log.warn(f'GENERATE refused: port busy ({busy})')
            return jsonify({'ok': False,
                            'error': f'the Pixhawk link is busy '
                                     f'({busy} is running)'}), 409
        live_release(30, 'route generation needs the port')
        io = None
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
            from make_waypoints import densify
            wps = densify(
                build_serpentine(t.lat, t.lon, head, rows, spacing, length),
                max_leg)
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
        """Launch run_mission detached; start_new_session prevents SIGHUP
        termination if server or SSH session dies.
        """
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
        # START and DRY RUN both post here, so every override below applies
        # to both by construction. Keep it that way: a dry run that flies
        # different settings from the mission proves nothing about it.
        try:
            opts = {k: mission_opt(body, k, ctl[k]) for k in MISSION_LIMITS}
        except ValueError as exc:
            log.warn(f'START refused: {exc}')
            return jsonify({'ok': False, 'error': str(exc)}), 400
        cmd = [ctl['python'], os.path.join(REPO, 'uno_q', 'run_mission.py'),
               '--conn', ctl['conn'], '--waypoints', wp,
               '--camera', str(ctl['camera']),
               '--data-dir', ctl['data_dir'],
               '--survey-alt', str(opts['survey_alt']),
               '--conf', str(opts['conf']),
               '--photo-hold', str(opts['photo_hold']),
               '--cross-min', str(opts['cross_min'])]
        if ctl['no_drop'] or body.get('no_drop'):
            cmd.append('--no-drop')
        if body.get('dry_run'):
            cmd.append('--dry-run')
        live_release(30, 'mission starting')
        # run_mission owns ~/logs/run_mission.log; suppress stdout duplication
        p = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL,
                             start_new_session=True)
        log(f"START: pid {p.pid}: {' '.join(cmd)}")
        return jsonify({'ok': True, 'pid': p.pid})

    @app.post('/api/control/stop')
    def api_stop():
        """Send SIGTERM for graceful RTL (not SIGKILL); run_mission handler
        winds up cleanly.
        """
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
        """Run test_everything.py (30 s, no motors); detached. Dashboard
        polls /api/control until selftest.json updates.
        """
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
        live_release(20, 'self-test starting')
        if only:
            cmd += ['--only', only]
        p = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL,
                             start_new_session=True)
        log(f'TEST: started pid {p.pid}')
        return jsonify({'ok': True, 'pid': p.pid})

    @app.post('/api/control/photo')
    def api_photo():
        """Manual photo: saved to manual_photos/, never processed by detector.

        Two paths: request running detect_worker via file, or open camera
        directly if no worker.
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
            with open(dw.MANUAL_REQ, 'w', encoding='utf-8') as f:
                f.write(manual_dir)
            deadline = time.time() + 5.0
            while time.time() < deadline:
                try:
                    with open(dw.MANUAL_DONE, encoding='utf-8') as f:
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
        # No worker: open the camera directly here, briefly.
        cap = None
        try:
            from camera import CameraError, open_camera
            import cv2
            cap, node = open_camera(ctl['camera'], log=log)
            for _ in range(8):            # let auto-exposure settle
                cap.grab()
            ok, frame = cap.read()
            # Released in the finally below: a raise from grab()/read() used
            # to jump past this line to the outer except and leak the handle,
            # after which nothing could open the camera again.
            cap.release()
            cap = None
            if not ok or frame is None:
                raise CameraError(f'{node} opened but gave no frame')
            name = 'manual_' + dw.stamp_name(0)
            cv2.imwrite(os.path.join(manual_dir, name), frame)
            log(f'PHOTO: direct: {name}')
            return jsonify({'ok': True, 'file': name})
        except Exception as exc:                        # noqa: BLE001
            log.error(f'PHOTO: direct capture failed: {exc}')
            return jsonify({'ok': False, 'error': str(exc)}), 500
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:                       # noqa: BLE001
                    pass

    # Wi-Fi settings: passwords never logged or in command line (see wifi.py)
    # Switching networks kills the connection serving this page

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

    # Geofence polygon: drawn by hand on map

    @app.get('/api/fence')
    def api_fence_get():
        poly = fence.load(fence_path)
        return jsonify({'polygon': poly, 'count': len(poly),
                        'problem': fence.validate(poly)})

    @app.post('/api/fence')
    def api_fence_post():
        """Store drawn polygon (not pushed to aircraft until Push button)."""
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
        """Upload polygon to Pixhawk and verify by reading back (not just ACK).

        Refused while mission or test owns the serial port.
        """
        if not ctl:
            abort(403)
        busy = port_owner()
        if busy:
            log.warn(f'FENCE push refused: link busy ({busy})')
            return jsonify({'ok': False,
                            'error': f'the Pixhawk link is busy '
                                     f'({busy} is running)'}), 409
        live_release(90, 'fence push and verify need the port')
        poly = fence.load(fence_path)
        bad = fence.validate(poly)
        if bad:
            return jsonify({'ok': False, 'error': bad}), 400
        # Verify on fresh link (not reusing socket from upload, which caches stale state)
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
        # "matched" has to mean the coordinates, not just the count.
        # 1e-6 deg is about 0.11 m, well under the upload's own 1e-7
        # rounding and far under any GPS error that matters here.
        off = [i for i, (a, b) in enumerate(zip(poly, back), 1)
               if abs(a[0] - b[0]) > 1e-6 or abs(a[1] - b[1]) > 1e-6]
        if off:
            log.error(f'FENCE read-back differs at corner(s) {off}')
            return jsonify({'ok': False,
                            'error': f'the Pixhawk stored {len(back)} corners '
                                     f'but corner(s) {off} came back at '
                                     f'different coordinates. Do not fly this '
                                     f'fence.'}), 500
        log(f'FENCE: {note}; read back {len(back)} corners on a fresh link '
            f'and every coordinate matches')
        return jsonify({'ok': True, 'count': len(back),
                        'note': f'{len(back)} corners are in the Pixhawk '
                                f'(read back and matched)'})

    # Satellite tiles: page tries Esri directly, proxies through board when
    # offline, caches on disk for map to work with no internet
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
            # Log once to avoid spam in offline field
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

    # Photo browser: 'auto'=captured frames (training data), 'manual'=dashboard
    # shots. Reverse-sorted newest first. Available without --enable-control.
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
    # Survey altitude (m): passed to run_mission for tuning between flights
    # 5 m: at 15 m, 55 cm targets shrink to 44 px below model threshold
    ap.add_argument('--survey-alt', type=float, default=5.0)
    # Confidence threshold: measured at 5 m, 60% recall at 0.25 vs 42% at 0.5;
    # artificial light lowers scores further; false positives acceptable for one take
    ap.add_argument('--conf', type=float, default=0.25)
    # Photo hold (s): camera uses long exposure at night; hold keeps frame sharp
    ap.add_argument('--photo-hold', type=float, default=1.0)
    ap.add_argument('--cross-min', type=float, default=0.3)
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

    # Suppress werkzeug INFO (logs every GET including polls = ~1200 lines/hour);
    # errors still surface
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)

    control = None
    if args.enable_control:
        control = {
            'python': args.mission_python or sys.executable,
            'waypoints': args.waypoints, 'conn': args.conn,
            'camera': args.camera, 'no_drop': args.no_drop,
            'data_dir': args.data_dir,
            'survey_alt': args.survey_alt, 'conf': args.conf,
            'photo_hold': args.photo_hold,
            'cross_min': args.cross_min,
        }
        log(f'FLIGHT CONTROL ENABLED on port {args.port}: anyone who can '
            f'reach this host can start and stop the mission')
    # threaded: live_position can hold a request for up to 8 s while the
    # link opens; single-threaded that would queue the Start click behind it.
    make_app(args.data_dir, control).run(host=args.host, port=args.port,
                                         threaded=True)


if __name__ == '__main__':
    main()
