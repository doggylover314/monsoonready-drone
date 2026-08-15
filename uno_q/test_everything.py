#!/usr/bin/env python3
"""Test every component EXCEPT motors and servos, in one 30-second budget.

The dashboard's "Test everything" button runs this (user spec 2026-08-16);
it also runs by hand:

    ~/venv/bin/python uno_q/test_everything.py

What it checks, in order:
  logs        ~/logs is writable
  disk        free space on the data filesystem
  camera      resolved BY NAME, opened, one real frame captured and saved
  pixhawk     heartbeat from component 1 over the USB link
  gps         fix type, satellite count, HDOP against the arming rules
  battery     voltage from SYS_STATUS
  luna        downward rangefinder (TF-Luna) producing a distance
  ring        ESP32 proximity ring: how many DISTANCE_SENSOR orientations
              are reporting (upward RNGFND2 counted separately)

DELIBERATELY ABSENT: anything that arms, spins a motor, or moves the gate
servo (user: "everything except motors and servo"). Read-only on the
aircraft; the only commands sent are SET_MESSAGE_INTERVAL stream requests.

Results go three places: this process's exit code (number of failed tests),
~/logs/test_everything.log (boardlog), and --out (default
~/monsoonready_data/selftest.json), which is what the dashboard displays.

It REFUSES to run while a mission is active: the mission owns the serial
port and the camera, and stealing either mid-flight to run a bench test is
how a self-test becomes a crash.
"""

import argparse
import json
import os
import shutil
import sys
import time

from boardlog import BoardLog
from camera import CameraError, open_camera

TOTAL_BUDGET_S = 30.0          # user spec: 30 s TOTAL, not per component
CAMERA_BUDGET_S = 8.0          # slice for the camera; the rest is MAVLink
MIN_DISK_FREE_GB = 2.0
# The project's own arming rules (FIELD_CHECKLIST): 10 sats, HDOP 1.5.
MIN_SATS = 10
MAX_HDOP = 1.5
MIN_BATT_V = 10.8              # BATT_LOW_VOLT; below this fix before flying


def mission_running():
    """True if a run_mission.py process exists (same test the dashboard
    uses: python process with the script name in its cmdline)."""
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        except OSError:
            continue
        if 'run_mission.py' in cmd and 'python' in cmd.lower():
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='auto')
    ap.add_argument('--camera', default='auto')
    ap.add_argument('--out', default='~/monsoonready_data/selftest.json')
    args = ap.parse_args()
    log = BoardLog('test_everything')
    out_path = os.path.expanduser(args.out)
    t_start = time.monotonic()
    deadline = t_start + TOTAL_BUDGET_S
    results = []

    def report(name, ok, detail):
        results.append({'name': name, 'ok': bool(ok), 'detail': detail})
        (log.info if ok else log.error)(
            f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")

    log('===== test_everything: 30 s budget, no motors, no servos =====')

    if mission_running():
        report('mission-clash', False,
               'a mission is RUNNING; refusing to touch its camera and '
               'serial port. Stop the mission first.')
        _finish(out_path, results, log, t_start)
        return 1

    # ---- logs writable ----
    try:
        probe = os.path.expanduser('~/logs/.write_probe')
        with open(probe, 'w') as f:
            f.write('ok')
        os.remove(probe)
        report('logs', True, '~/logs writable')
    except OSError as exc:
        report('logs', False, f'~/logs NOT writable: {exc}')

    # ---- disk space ----
    try:
        du = shutil.disk_usage(os.path.expanduser('~'))
        free_gb = du.free / 1e9
        report('disk', free_gb >= MIN_DISK_FREE_GB,
               f'{free_gb:.1f} GB free'
               + ('' if free_gb >= MIN_DISK_FREE_GB else
                  f' (< {MIN_DISK_FREE_GB:g} GB: photos and logs will '
                  f'starve it)'))
    except OSError as exc:
        report('disk', False, str(exc))

    # ---- camera ----
    cam_deadline = min(time.monotonic() + CAMERA_BUDGET_S, deadline)
    try:
        cap, node = open_camera(args.camera, log=log)
        ok, frame = cap.read()
        while not ok and time.monotonic() < cam_deadline:
            ok, frame = cap.read()
        if ok and frame is not None:
            h, w = frame.shape[:2]
            snap = '/tmp/selftest_camera.jpg'
            import cv2
            cv2.imwrite(snap, frame)
            report('camera', True, f'{node}, {w}x{h} frame saved to {snap}')
        else:
            report('camera', False, f'{node} opened but produced no frame')
        cap.release()
    except CameraError as exc:
        report('camera', False, str(exc))

    # ---- everything MAVLink, on one connection for the rest of the budget --
    try:
        from mavlink_io import MavIO
        io = MavIO(args.conn, log=log)
        io.wait_ready(timeout=max(3.0, deadline - time.monotonic() - 8.0))
        report('pixhawk', True,
               f'heartbeat from system {io.conn.target_system}, '
               f'mode {io.tel.mode}')
    except Exception as exc:                            # noqa: BLE001
        report('pixhawk', False, str(exc))
        for name in ('gps', 'battery', 'luna', 'ring'):
            report(name, False, 'skipped: no Pixhawk link')
        _finish(out_path, results, log, t_start)
        return sum(not r['ok'] for r in results)

    io.setup_streams()
    ring_orients = set()
    luna = None                 # (distance_m, valid)
    listen_until = deadline
    while time.monotonic() < listen_until:
        msg = io.step()
        if msg is None:
            continue
        if msg.get_type() == 'DISTANCE_SENSOR':
            if msg.orientation == 25:                  # PITCH_270 = down
                luna = (msg.current_distance / 100.0,
                        msg.min_distance < msg.current_distance
                        < msg.max_distance)
            else:
                ring_orients.add(msg.orientation)
        # Stop early once every answer is in: GPS+battery arrive at 1 Hz,
        # the ring's orientations within a couple of seconds.
        tel = io.tel
        if (luna is not None and tel.sats is not None
                and tel.batt_v is not None and len(ring_orients) >= 6
                and time.monotonic() - t_start > 10):
            break

    tel = io.tel
    if tel.fix_type is None:
        report('gps', False, 'no GPS_RAW_INT arrived at all')
    else:
        ok = (tel.fix_type >= 3 and (tel.sats or 0) >= MIN_SATS
              and tel.hdop is not None and tel.hdop <= MAX_HDOP)
        report('gps', ok,
               f'fix {tel.fix_type} ({"3D" if tel.fix_type >= 3 else "NO 3D"})'
               f', {tel.sats} sats, HDOP {tel.hdop} '
               f'(rules: >= {MIN_SATS} sats, <= {MAX_HDOP} HDOP). '
               + ('' if ok else 'Indoors this is expected; outdoors WAIT '
                                'before arming.'))

    if tel.batt_v is None:
        report('battery', False, 'no voltage in SYS_STATUS')
    else:
        report('battery', tel.batt_v >= MIN_BATT_V,
               f'{tel.batt_v:.2f} V'
               + (f' ({tel.batt_pct}%)' if tel.batt_pct is not None else '')
               + ('' if tel.batt_v >= MIN_BATT_V else
                  f' -- below {MIN_BATT_V} V, charge or swap before flying'))

    if luna is None:
        report('luna', False,
               'no downward DISTANCE_SENSOR: TF-Luna silent (SERIAL4)')
    else:
        d, valid = luna
        state = ('valid' if valid
                 else 'OUT OF RANGE, normal on the bench past 8 m or at sky')
        report('luna', valid, f'{d:.2f} m ({state})')

    if not ring_orients:
        report('ring', False,
               'no ring DISTANCE_SENSOR messages: ESP32 silent or dead '
               '(power-cycle re-runs its boot probe)')
    else:
        report('ring', True,
               f'{len(ring_orients)} orientation(s) reporting: '
               f'{sorted(ring_orients)} (boot-latched; a missing sector '
               f'needs a cold power cycle)')

    code = _finish(out_path, results, log, t_start)
    return code


def _finish(out_path, results, log, t_start):
    fails = sum(not r['ok'] for r in results)
    payload = {'t': time.time(), 'took_s': round(time.monotonic() - t_start, 1),
               'ok_all': fails == 0, 'results': results}
    try:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        tmp = out_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(payload, f, indent=1)
        os.replace(tmp, out_path)
    except OSError as exc:
        log.error(f'could not write {out_path}: {exc}')
    log(f'===== done in {payload["took_s"]}s: '
        f'{len(results) - fails}/{len(results)} passed =====')
    return fails


if __name__ == '__main__':
    sys.exit(main())
