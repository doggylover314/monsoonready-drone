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

# The full run plan, published in selftest.json BEFORE the run starts so the
# dashboard can list every component name immediately and fill in ticks and
# crosses as they land (user, 2026-08-16: the names must never disappear,
# only grey out). Order matches the order they are executed below.
COMPONENTS = ['logs', 'disk', 'camera', 'pixhawk', 'gps', 'battery',
              'luna', 'ring', 'fence', 'prearm']

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
    ap.add_argument('--fence-file', default='~/monsoonready_data/fence.json',
                    help='the polygon the dashboard drew, compared against '
                         'what the aircraft actually holds')
    ap.add_argument('--only', default='',
                    help='comma-separated subset of ' + ','.join(COMPONENTS)
                         + " (the dashboard's Check arming button uses "
                           "--only prearm for a fast loop while fixing "
                           "whatever is blocking the arm)")
    args = ap.parse_args()
    log = BoardLog('test_everything')
    out_path = os.path.expanduser(args.out)
    t_start = time.monotonic()
    only = [c.strip() for c in args.only.split(',') if c.strip()]
    bad = [c for c in only if c not in COMPONENTS]
    if bad:
        sys.exit(f"--only: unknown component(s) {bad}; pick from {COMPONENTS}")
    # A subset run gets a proportionally shorter budget: waiting 30 s to
    # re-read one prearm message would make the fix-and-retry loop useless.
    plan = only or list(COMPONENTS)
    budget = TOTAL_BUDGET_S if not only else max(8.0, 6.0 * len(only))
    deadline = t_start + budget
    results = []
    want = lambda name: name in plan

    def report(name, ok, detail):
        if not want(name):
            return
        results.append({'name': name, 'ok': bool(ok), 'detail': detail})
        (log.info if ok else log.error)(
            f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
        # Publish after EVERY component, not only at the end: the dashboard
        # polls this file and fills each tick/cross in as it happens.
        _write(out_path, results, log, t_start, running=True, plan=plan)

    log(f'===== test_everything: {budget:.0f} s budget, no motors, no '
        f'servos, plan={plan} =====')
    # Empty results + the plan, so the panel greys every name the instant
    # the run starts instead of showing the previous run's verdicts.
    _write(out_path, [], log, t_start, running=True, plan=plan)

    if mission_running():
        report('mission-clash', False,
               'a mission is RUNNING; refusing to touch its camera and '
               'serial port. Stop the mission first.')
        _finish(out_path, results, log, t_start, plan)
        return 1

    # ---- logs writable ----
    if want('logs'):
        try:
            probe = os.path.expanduser('~/logs/.write_probe')
            with open(probe, 'w') as f:
                f.write('ok')
            os.remove(probe)
            report('logs', True, '~/logs writable')
        except OSError as exc:
            report('logs', False, f'~/logs NOT writable: {exc}')

    # ---- disk space ----
    if want('disk'):
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
    if want('camera'):
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

    # Nothing left needs the aircraft (e.g. --only logs,disk): stop before
    # opening a serial port for no reason.
    if not any(want(n) for n in ('pixhawk', 'gps', 'battery', 'luna', 'ring',
                                 'fence', 'prearm')):
        return _finish(out_path, results, log, t_start, plan)

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
        for name in COMPONENTS[COMPONENTS.index('gps'):]:
            report(name, False, 'skipped: no Pixhawk link')
        _finish(out_path, results, log, t_start, plan)
        return sum(not r['ok'] for r in results)

    io.setup_streams()
    ring_orients = set()
    luna = None                 # (distance_m, rated_min_m, rated_max_m)
    up_m = None                 # ring's upward sensor (RNGFND2, orient 24)
    # A subset run must not sit here burning its budget on sensors nobody
    # asked about: --only prearm should answer in seconds.
    sensor_names = ('gps', 'battery', 'luna', 'ring')
    listen_until = deadline if any(want(n) for n in sensor_names) \
        else time.monotonic()
    if want('prearm') or want('fence'):
        listen_until = min(listen_until, deadline - 5.0)   # leave room
    while time.monotonic() < listen_until:
        msg = io.step()
        if msg is None:
            continue
        if msg.get_type() == 'DISTANCE_SENSOR':
            if msg.orientation == 25:                  # PITCH_270 = down
                luna = (msg.current_distance / 100.0,
                        msg.min_distance / 100.0, msg.max_distance / 100.0)
            elif msg.orientation == 24:                # PITCH_90 = up
                up_m = msg.current_distance / 100.0
            else:
                ring_orients.add(msg.orientation)
        # Stop early once every WANTED answer is in: GPS+battery arrive at
        # 1 Hz, the ring's orientations within a couple of seconds. 5 bins
        # is the ring's healthy ceiling (6 sensors, ch2 dead chip).
        tel = io.tel
        have = (
            (luna is not None or not want('luna'))
            and (tel.sats is not None or not want('gps'))
            and (tel.batt_v is not None or not want('battery'))
            and (len(ring_orients) >= 5 or not want('ring')))
        if have and time.monotonic() - t_start > 10:
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
        # % comes from VOLTAGE (survives reboots), not ArduPilot's coulomb
        # counter, which resets to ~100% every boot and lies after a swap.
        est = tel.batt_pct_est
        report('battery', tel.batt_v >= MIN_BATT_V,
               f'{tel.batt_v:.2f} V'
               + (f' (~{est}% by rest voltage, +/-10; sags low under load)'
                  if est is not None else '')
               + ('' if tel.batt_v >= MIN_BATT_V else
                  f' -- below {MIN_BATT_V} V, charge or swap before flying'))

    if luna is None:
        report('luna', False,
               'no downward DISTANCE_SENSOR: TF-Luna silent (SERIAL4)')
    else:
        # THE ON-GROUND RULE (same rule wiring_check has carried since the
        # bench: an aircraft on its legs LEGITIMATELY reads below the rated
        # minimum). Landed, the lens sits ~0.13 m over the floor
        # (RNGFND1_GNDCLR) and the TF-Luna's rated minimum is 0.2 m, so
        # ArduPilot flags the reading out-of-range BY DESIGN while the sensor
        # is in fact alive and measuring. That is a PASS: the failure this
        # test exists to catch is silence, not sitting on the ground.
        d, dmin, dmax = luna
        if 0.0 < d < dmin:
            report('luna', True,
                   f'{d:.2f} m ON-GROUND reading (alive; below the rated '
                   f'{dmin:.2f} m minimum because the aircraft is sitting '
                   f'{d:.2f} m over the floor; valid again in flight)')
        elif dmin <= d <= dmax:
            report('luna', True, f'{d:.2f} m (valid return)')
        else:
            report('luna', False,
                   f'{d:.2f} m: no usable return (past {dmax:.0f} m or at '
                   f'the sky); point it at the floor to prove it')

    up_note = (f'; up sensor {up_m:.2f} m' if up_m is not None
               else '; up sensor SILENT (RNGFND2, known-flaky power plug)')
    if not ring_orients:
        report('ring', False,
               'no ring DISTANCE_SENSOR messages: ESP32 silent or dead '
               '(cold power cycle)' + up_note)
    else:
        n = len(ring_orients)
        # 6 sensors land in 6 of ArduPilot's eight 45-deg bins; ch2 (120
        # deg) is a KNOWN DEAD CHIP (proven at four bus speeds 2026-08-14),
        # so 5 bins is the healthy ceiling, 4 = one more sensor out. The
        # 2026-08-16 firmware re-inits dropped sensors every 5 s, so a bin
        # that stays missing across minutes is hardware, not the old
        # boot-latch.
        report('ring', n >= 4,
               f'{n} of ~5 achievable bearing bins: {sorted(ring_orients)} '
               f'(6 fitted, ch2 dead chip = permanently blind 120deg'
               + (', self-heal firmware NOT yet flashed'
                  if n < 4 else '') + ')' + up_note)

    # ---- fence: does the AIRCRAFT hold the polygon that was drawn? ------
    # This check exists because the firmware will not do it. Tested in SITL
    # 2026-08-16: with FENCE_TYPE bit 2 set and zero corners stored, Copter
    # 4.7 arms happily (loaded() is "a load happened", and an empty fence
    # loads fine), so a fence that was drawn but never pushed protects
    # nothing and says nothing. Here it is a loud FAIL.
    if want('fence'):
        try:
            import fence as fence_mod
            drawn = fence_mod.load(args.fence_file)
            stored = fence_mod.read_back(io, log=log)
            if not stored and not drawn:
                report('fence', False,
                       'NO polygon fence anywhere: none drawn and none in '
                       'the Pixhawk. Only the altitude ceiling and the '
                       'circle are protecting the aircraft. Draw one on the '
                       'dashboard map (Draw fence) and press Push fence.')
            elif not stored:
                report('fence', False,
                       f'{len(drawn)} corners drawn on the dashboard but the '
                       f'Pixhawk holds NONE: the fence was never pushed and '
                       f'is protecting nothing. Press Push fence.')
            elif not drawn:
                report('fence', True,
                       f'{len(stored)} corners stored in the Pixhawk (no '
                       f'local copy on this board to compare against)')
            else:
                same = (len(stored) == len(drawn) and all(
                    abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6
                    for a, b in zip(drawn, stored)))
                report('fence', same,
                       f'{len(stored)} corners in the Pixhawk'
                       + (' and they match the drawn fence' if same else
                          f' do NOT match the {len(drawn)} drawn on the '
                          f'dashboard: press Push fence'))
        except Exception as exc:                        # noqa: BLE001
            report('fence', False, f'could not read the fence back: {exc}')

    # ---- prearm: WHY the aircraft will not arm (user, 2026-08-16: "make
    # sure that the dashboard shows prearm issues and all that so it can be
    # easily fixed"). RUN_PREARM_CHECKS forces the autopilot to re-report
    # every failing check immediately instead of on its 30 s display cycle
    # (AP_Arming.cpp:109-111), so the fix-and-retry loop at the field takes
    # seconds. Nothing here arms anything: it only asks and listens.
    if want('prearm'):
        if io.tel.armed:
            report('prearm', True,
                   'the aircraft is ARMED right now, so the prearm checks '
                   'already passed (props are live: stay clear)')
        else:
            t_ask = time.monotonic()
            try:
                io.run_prearm_checks()
            except Exception as exc:                    # noqa: BLE001
                log.warn(f'RUN_PREARM_CHECKS not accepted: {exc}')
            # 4 s covers the forced report plus a 1 Hz SYS_STATUS carrying
            # the pass/fail bit.
            listen = time.monotonic() + 4.0
            while time.monotonic() < listen:
                io.step()
            msgs = io.tel.prearm_messages(since_t=t_ask - 1.0)
            ok_bit = io.tel.prearm_ok()
            bad = io.tel.unhealthy_sensors()
            if msgs:
                report('prearm', False,
                       'WILL NOT ARM: ' + ' | '.join(msgs[:3])
                       + (f' (+{len(msgs) - 3} more, see the log)'
                          if len(msgs) > 3 else ''))
            elif ok_bit is False:
                report('prearm', False,
                       'WILL NOT ARM: prearm checks are failing but the '
                       'autopilot sent no reason within 4 s'
                       + (f'; unhealthy: {", ".join(bad)}' if bad else ''))
            elif ok_bit is True:
                report('prearm', True,
                       'prearm checks PASSING: the aircraft would arm now'
                       + (f' (reported unhealthy but not blocking: '
                          f'{", ".join(bad)})' if bad else ''))
            else:
                report('prearm', False,
                       'no verdict in 4 s: no PreArm message and no '
                       'SYS_STATUS prearm bit from the autopilot')

    return _finish(out_path, results, log, t_start, plan)


def _write(out_path, results, log, t_start, running, plan=None):
    """Atomically publish the run so far. Called after every component."""
    fails = sum(not r['ok'] for r in results)
    payload = {'t': time.time(), 'took_s': round(time.monotonic() - t_start, 1),
               'ok_all': fails == 0 and not running, 'running': running,
               'plan': plan or list(COMPONENTS), 'results': results}
    try:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        tmp = out_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(payload, f, indent=1)
        os.replace(tmp, out_path)
    except OSError as exc:
        log.error(f'could not write {out_path}: {exc}')
    return fails


def _finish(out_path, results, log, t_start, plan=None):
    fails = _write(out_path, results, log, t_start, running=False, plan=plan)
    log(f'===== done in {round(time.monotonic() - t_start, 1)}s: '
        f'{len(results) - fails}/{len(results)} passed =====')
    return fails


if __name__ == '__main__':
    sys.exit(main())
