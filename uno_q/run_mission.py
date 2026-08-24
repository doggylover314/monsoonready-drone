"""Onboard mission runner for UNO Q.
Usage: setsid nohup python uno_q/run_mission.py --waypoints wp_field.txt &

Logs to ~/logs/run_mission.log. A signal triggers a graceful shutdown into
RTL; an exception triggers an emergency RTL. Waypoint file format: lat,lon
per line.
"""

import argparse
import atexit
import json
import os
import signal
import subprocess
import sys
import time

from boardlog import BoardLog
from camera_geom import DEFAULT_HFOV_DEG, MOUNT_YAW_DEG, CameraGeometry
from detect_worker import DEFAULT_OUT as DET_FILE_DEFAULT
from detector import FileDetector, OnnxDetector
from dropper import LogDropper, PixhawkServoDropper
from mavlink_io import MavIO
from mission import Mission, MissionConfig
from missionlog import MissionLog

# Found relative to this file, so it works on the laptop, the board, and
# any checkout.
DEFAULT_MODEL = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models', 'best.onnx')


def read_waypoints(path, log):
    wps = []
    try:
        f = open(os.path.expanduser(path))
    except OSError as exc:
        log.error(f"waypoint file unreadable: {exc}")
        sys.exit(f"waypoint file unreadable: {exc}")
    with f:
        for n, line in enumerate(f, 1):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            try:
                lat, lon = (float(v) for v in line.split(','))
            except ValueError:
                log.error(f"{path}:{n}: expected 'lat,lon', got {line!r}")
                sys.exit(f"{path}:{n}: expected 'lat,lon', got {line!r}")
            wps.append((lat, lon))
    if not wps:
        log.error(f"{path}: no waypoints")
        sys.exit(f"{path}: no waypoints")
    return wps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='auto',
                    help="'auto' (default) = the Pixhawk's USB, resolved "
                         "from /dev/serial/by-id. SITL: tcp:127.0.0.1:5760.")
    ap.add_argument('--baud', type=int, default=115200)
    # Resolved from THIS FILE's location so checkout always carries it.
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--survey-alt', type=float, default=5.0)
    # Has to stay above floor_margin_m or the EKF floor abort goes dead:
    # that check is rel_alt < drop_alt - margin, which at 1.0 and a 1.0
    # margin means rel_alt < 0 and can never fire.
    ap.add_argument('--drop-alt', type=float, default=1.0)
    # Metres from the detected puddle centre. TF-Luna cannot range still
    # water, so the aircraft descends this far to the side before crossing
    # over to release. North by default is a site decision; 0 disables
    # the offset.
    ap.add_argument('--offset-n', type=float, default=1.5,
                    help='metres north of the puddle to descend over')
    ap.add_argument('--offset-e', type=float, default=0.0,
                    help='metres east of the puddle to descend over')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--photo-hold', type=float, default=1.0,
                    help='seconds to hold at each waypoint so the frame is '
                         'taken stationary. 0 flies the rows continuously')
    # 'auto' resolves the camera by name (camera.py). A bare index can race
    # the Venus codecs; use a number or /dev/videoN to pin it on the bench.
    ap.add_argument('--camera', default='auto')
    ap.add_argument('--frame-w', type=int, default=1280)
    ap.add_argument('--frame-h', type=int, default=720)
    # Defaults to the measured 56.2 deg in camera_geom; geometry is always
    # on unless 0 is passed to disable it.
    ap.add_argument('--hfov-deg', type=float, default=DEFAULT_HFOV_DEG,
                    help=f'MEASURED horizontal FOV, default {DEFAULT_HFOV_DEG} '
                         f'(camera_geom.DEFAULT_HFOV_DEG). 0 = nadir only.')
    # Camera mounted rotated: 1280 px axis fore-aft (camera_geom.MOUNT_YAW_DEG)
    ap.add_argument('--mount-yaw-deg', type=float, default=MOUNT_YAW_DEG)
    ap.add_argument('--servo-channel', type=int, default=9,
                    help='AUX OUT 1 = ch9 (wired 2026-08-02); '
                         'SERVO9_FUNCTION=0 must be pushed or DO_SET_SERVO '
                         'is silently ignored')
    # Use PixhawkServoDropper defaults for consistency (single source of truth).
    ap.add_argument('--servo-closed-us', type=int,
                    default=PixhawkServoDropper.DEFAULT_CLOSED_US)
    ap.add_argument('--servo-open-us', type=int,
                    default=PixhawkServoDropper.DEFAULT_OPEN_US)
    ap.add_argument('--det-file', default=DET_FILE_DEFAULT,
                    help='where detect_worker.py publishes results')
    ap.add_argument('--inline-detector', action='store_true',
                    help='old behaviour: inference inside the mission loop, '
                         'blocking it for the model latency each poll')
    ap.add_argument('--no-drop', action='store_true',
                    help='log drops instead of moving the servo')
    ap.add_argument('--dry-run', action='store_true',
                    help='never arm; exercise detector and logging only')
    ap.add_argument('--no-basestation', action='store_true')
    args = ap.parse_args()

    log = BoardLog('run_mission')
    log(f"[run] ===== launch: {' '.join(sys.argv)} =====")

    wps = read_waypoints(args.waypoints, log)
    model = os.path.expanduser(args.model)
    if not os.path.exists(model):
        log.error(f"model not found: {model}")
        sys.exit(f"model not found: {model}")

    log(f"[run] connecting {args.conn} ...")
    io = MavIO(args.conn, baud=args.baud, log=log)
    io.wait_ready()
    log(f"[run] heartbeat from system {io.conn.target_system}")
    io.setup_streams()

    geom = None
    if args.hfov_deg:
        geom = CameraGeometry(args.frame_w, args.frame_h, args.hfov_deg)
        fw, fh = geom.footprint_m(args.survey_alt)
        log(f"[run] camera {args.hfov_deg:.1f}deg hfov -> footprint at "
            f"{args.survey_alt:.0f}m is {fw:.1f} x {fh:.1f} m")
    else:
        log("[run] no --hfov-deg: detections assumed directly below "
            "the aircraft (nadir)")

    # Inference runs in a separate process (detect_worker.py) so it can't
    # block MAVLink: inline inference costs 511ms/frame with yolo26n and up
    # to 1518ms/frame with yolo26s.
    if args.inline_detector:
        detector = OnnxDetector(model, camera=args.camera, conf=args.conf,
                                geom=geom, mount_yaw_deg=args.mount_yaw_deg,
                                log=log)
    else:
        _spawn_or_reuse_worker(args, model, log)
        detector = FileDetector(args.det_file, conf=args.conf,
                                geom=geom, mount_yaw_deg=args.mount_yaw_deg,
                                log=log)
    if not detector.preflight():
        log.error("[run] camera preflight failed, refusing to fly a blind "
                  "survey")
        sys.exit("[run] camera preflight failed, refusing to fly a blind survey")

    if args.dry_run:
        # Runs before MissionLog is created, so a dry run never leaves a
        # phantom in-progress mission record.
        log("[run] DRY RUN: not arming. Polling the detector for 30s.")
        deadline = time.monotonic() + 30
        seen = 0
        while time.monotonic() < deadline:
            io.step()
            det = detector.poll(io.tel)
            if det:
                seen += 1
                log(f"[run] detection {det.lat:.7f},{det.lon:.7f} "
                    f"conf {det.confidence:.2f}")
        if io.tel.lat is None:
            log("[run] NOTE: no GPS fix arrived, so the detector was never "
                "polled (it needs a position to attach a detection to). "
                "Indoors this is expected and the dry run proved nothing "
                "beyond the link and the camera.")
        else:
            log(f"[run] dry run complete, {seen} detection(s)")
        return

    if args.no_drop:
        dropper = LogDropper()
        log("[run] DROPS DISABLED (logging dropper)")
    else:
        # Raises if the gate does not answer: that is the pre-arm test.
        dropper = PixhawkServoDropper(
            io, channel=args.servo_channel,
            closed_us=args.servo_closed_us, open_us=args.servo_open_us)
        log(f"[run] dropper armed on servo channel {args.servo_channel} "
            f"({args.servo_closed_us}us closed / {args.servo_open_us}us open)")

    recorder = MissionLog(args.data_dir)
    log(f"[run] logging to {recorder.path}")

    bs_cmd = None
    if not args.no_basestation:
        bs = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'basestation', 'dashboard.py')
        bs_cmd = [sys.executable, bs, '--data-dir', args.data_dir]

    # The fence defines the detection boundary. The camera sees roughly 8m
    # either side of the aircraft, while the rows themselves sit only
    # metres inside it. A missing fence disables the check, and that gets
    # logged so the protection can never silently fail.
    import fence as fence_mod
    poly = fence_mod.load()
    if len(poly) >= 3:
        log(f"[run] geofence loaded: {len(poly)} corners; detections outside "
            f"it (or whose descent point is) will be ignored")
    else:
        log("[run] NO GEOFENCE FILE: detections will NOT be fence-checked. "
            "Draw and save a fence on the dashboard to enable that check.")

    cfg = MissionConfig(waypoints=wps, survey_alt_m=args.survey_alt,
                        drop_alt_m=args.drop_alt,
                        photo_hold_s=args.photo_hold,
                        lateral_offset_n_m=args.offset_n,
                        lateral_offset_e_m=args.offset_e,
                        fence=poly,
                        basestation_cmd=bs_cmd)

    stop = {'why': None}

    def _wind_up(signum, _frame):
        # Sets the flag and returns; the state machine RTLs on its next
        # tick through the normal path.
        if stop['why'] is None:
            stop['why'] = f"{signal.Signals(signum).name} received"
            log(f"[run] {stop['why']}: winding up, {cfg.end_mode} at the "
                f"next tick")

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _wind_up)

    log(f"[run] {len(wps)} waypoints, survey {args.survey_alt}m, "
        f"drop at {args.drop_alt}m rangefinder")
    mission = Mission(io, detector, dropper, cfg, log=log, recorder=recorder,
                      should_stop=lambda: stop['why'])
    try:
        final = mission.run()
        log(f"[run] finished in state {final}, "
            f"drops={getattr(dropper, 'succeeded', dropper.fired)}")
    except BaseException as exc:                       # noqa: BLE001
        # Aircraft airborne; emergency RTL before re-raising.
        log.error(f"[run] MISSION RAISED in state {mission.state}: "
                  f"{type(exc).__name__}: {exc}")
        _emergency_rtl(io, cfg.end_mode, log)
        try:
            recorder.mission_end(f'CRASHED:{mission.state}',
                                 getattr(dropper, 'succeeded', None))
        except Exception:                              # noqa: BLE001
            pass
        raise
    finally:
        try:
            recorder.close()
        except Exception:                              # noqa: BLE001
            pass


def _worker_pids():
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
        if 'detect_worker.py' in cmd and 'python' in cmd.lower():
            out.append(int(entry))
    return out


def _spawn_or_reuse_worker(args, model, log):
    """Start detect_worker.py, unless one is already publishing fresher than STALE_S.

    Avoids a double-spawn on the V4L2 device. Any worker spawned here is
    stopped at exit; it logs to ~/logs/detect_worker.log with stdout sent
    to /dev/null.
    """
    det_file = os.path.expanduser(args.det_file)
    try:
        fresh = (time.time() - os.stat(det_file).st_mtime
                 <= FileDetector.STALE_S)
    except OSError:
        fresh = False
    if fresh:
        # A leftover, hand-started worker filters at its own conf before
        # writing, so reusing one with a higher threshold would silently
        # blind the mission.
        try:
            with open(det_file) as f:
                wconf = json.load(f).get('conf')
        except (OSError, ValueError):
            wconf = None
        if wconf is not None and abs(wconf - args.conf) < 1e-6:
            log(f"[run] detect worker already publishing {det_file} at "
                f"conf {wconf:g}, reusing it")
            return None
        log(f"[run] a worker is publishing {det_file} at conf "
            f"{wconf if wconf is not None else 'unknown'}, mission wants "
            f"{args.conf:g}: stopping it and spawning a fresh one")
        for pid in _worker_pids():
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
        deadline = time.time() + 5
        while _worker_pids() and time.time() < deadline:
            time.sleep(0.2)
        if _worker_pids():
            log.error('[run] old detect worker refused to die; the camera '
                      'may be double-opened')
    worker_py = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'detect_worker.py')
    proc = subprocess.Popen(
        [sys.executable, worker_py, '--model', model,
         '--camera', str(args.camera), '--conf', str(args.conf),
         '--out', det_file],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    log(f"[run] detect worker spawned (pid {proc.pid}, "
        f"log ~/logs/detect_worker.log)")
    atexit.register(_stop_worker, proc, log)
    return proc


def _stop_worker(proc, log):
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log("[run] detect worker stopped")


def _emergency_rtl(io, mode, log, attempts=3):
    """Attempt emergency RTL; best-effort, fails cleanly if link is dead."""
    for i in range(1, attempts + 1):
        try:
            confirmed = io.set_mode(mode)
            log(f"[run] emergency {mode} commanded (attempt {i})"
                + ("" if confirmed else ", NOT confirmed by heartbeat"))
            return True
        except Exception as exc:                       # noqa: BLE001
            log.error(f"[run] emergency {mode} attempt {i} failed: {exc}")
    log.error(f"[run] COULD NOT COMMAND {mode}. Aircraft is holding in "
              f"GUIDED. PILOT: take it on the mode switch.")
    return False


if __name__ == '__main__':
    main()
