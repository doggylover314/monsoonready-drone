"""Onboard mission runner: the thing that actually flies on the UNO Q.

Wires the real parts together, where sitl_test.py wires the fake ones:

    sitl_test.py                 run_mission.py
    ------------                 --------------
    FakeDetector (planted)       FileDetector reading detect_worker.py's
                                 output (camera + ONNX in their own process;
                                 --inline-detector restores in-loop inference)
    LogDropper (prints)          PixhawkServoDropper (moves the gate)
    tcp:127.0.0.1:5760           auto (the Pixhawk's USB via the hub)
    no base station              basestation_cmd set, launched on DONE

Everything else, above all mission.py, is identical. That is the point: the
state machine that was proven in simulation is the one that flies.

    setsid nohup ~/venv/bin/python uno_q/run_mission.py \
        --waypoints wp_field.txt &

--conn defaults to 'auto' (2026-08-16): the link is now the Pixhawk's own
USB plug on the board's hub, resolved from /dev/serial/by-id by
mavlink_io.resolve_conn, so no device number can shuffle underneath us. The
byte-shovel (STM32 + pump + SERIAL5) this replaces is deleted; SITL still
works by passing --conn tcp:127.0.0.1:5760.

Everything prints AND appends to ~/logs/run_mission.log (boardlog): the
program owns its log file, so it logs identically launched by hand, by the
dashboard, or detached under nohup. The farm-day rule (SCOPE RULES 1):
every launch, every command, every failure reason, in the file.

--hfov-deg DEFAULTS to the measured 56.2 deg (camera_geom, tape measure
2026-08-15), so it no longer has to be passed and no flight can silently
fall back to the nadir assumption. Re-measure with
uno_q/calibrate_camera.py if the camera or its housing ever changes, and
update camera_geom.DEFAULT_HFOV_DEG, not a command line.

LAUNCH IT DETACHED, exactly like that. `setsid` puts the runner in its own
session so closing the ssh connection does not deliver SIGHUP to an aircraft
in the middle of a descent. This is the primary defence; the signal handling
below is the backstop for when it was forgotten.

Waypoint file: one "lat,lon" per line, blank lines and #comments ignored.

DRY RUN FIRST. --no-drop swaps in the logging dropper so the whole loop can
be flown with nothing to clean up afterwards, and --dry-run additionally
refuses to arm, so the detector and the log can be exercised on the bench.

CONTAINMENT. Everything from arming onward is wrapped, because the failure
this protects against is specific and bad: the old runner called
Mission.run() bare, so any exception, or a SIGHUP from a dropped ssh
session, killed the loop wherever it stood. Mid-DESCEND that leaves a copter
a few metres over water with the autopilot still holding the last velocity
setpoint it was given and nothing left alive to send another. Now:

  * SIGINT / SIGTERM / SIGHUP ask the state machine to wind up at the next
    tick, which commands RTL through the normal path;
  * any exception out of run() triggers a best-effort RTL and is logged;
  * the mission log is closed either way, so the flight is never missing its
    mission_end record.

What this still cannot cover: SIGKILL, a panic, or the UNO Q losing power.
Nothing in userspace can. In those cases the aircraft holds position in
GUIDED and the backstops are the pilot's mode switch and the battery
failsafe, which is the same place the design has always put final authority.
"""

import argparse
import atexit
import os
import signal
import subprocess
import sys
import time

from boardlog import BoardLog
from camera_geom import DEFAULT_HFOV_DEG, CameraGeometry
from detect_worker import DEFAULT_OUT as DET_FILE_DEFAULT
from detector import FileDetector, OnnxDetector
from dropper import LogDropper, PixhawkServoDropper
from mavlink_io import MavIO
from mission import Mission, MissionConfig
from missionlog import MissionLog

# <repo>/models/best.onnx, found relative to this file so it is correct on the
# laptop, on the board, and in any future checkout location.
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
    # Resolved from THIS FILE's location, not from $HOME. The old default was
    # ~/uno_q/best.onnx, a path that stopped existing when the board was
    # reflashed on 2026-08-13; models/best.onnx has been tracked in git since,
    # so the checkout always carries it wherever the checkout happens to be.
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--survey-alt', type=float, default=15.0)
    ap.add_argument('--drop-alt', type=float, default=3.0)
    ap.add_argument('--conf', type=float, default=0.5)
    # 'auto' = resolve the camera BY NAME (camera.py). Bare indexes are an
    # enumeration race with the Venus codecs and losing that race is exactly
    # the 2026-08-15 farm failure. A number or /dev/videoN pins it for bench.
    ap.add_argument('--camera', default='auto')
    ap.add_argument('--frame-w', type=int, default=1280)
    ap.add_argument('--frame-h', type=int, default=720)
    # Defaults to the MEASURED value in camera_geom (56.2 deg, tape measure at
    # the farm 2026-08-15), so geometry is ALWAYS on and no flight can
    # accidentally fall back to the nadir assumption by forgetting a flag.
    # Pass --hfov-deg 0 to deliberately disable geometry.
    ap.add_argument('--hfov-deg', type=float, default=DEFAULT_HFOV_DEG,
                    help=f'MEASURED horizontal FOV, default {DEFAULT_HFOV_DEG} '
                         f'(camera_geom.DEFAULT_HFOV_DEG). 0 = nadir only.')
    ap.add_argument('--mount-yaw-deg', type=float, default=0.0)
    ap.add_argument('--servo-channel', type=int, default=9,
                    help='AUX OUT 1 = ch9 (wired 2026-08-02); '
                         'SERVO9_FUNCTION=0 must be pushed or DO_SET_SERVO '
                         'is silently ignored')
    # ONE SOURCE OF TRUTH, and this is why. These were hard-coded 1000/1900
    # while dropper.py had been reversed to 1600 closed / 1000 open, so
    # run_mission's idea of "closed" WAS THE OPEN POSITION. Flying that empties
    # the hopper on the ground the moment the servo is initialised. Same class
    # of bug as wiring_check's hard-coded 1900/1000, found the same way.
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

    # Detection default (2026-08-01): inference in its own PROCESS
    # (detect_worker.py), results read from a file. Measured reason: in-line
    # inference blocks this single-threaded loop 511ms/frame with yolo26n and
    # 1518ms with yolo26s, during which no MAVLink is pumped and a pilot
    # flipping the mode switch goes unnoticed.
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
        # Deliberately BEFORE the MissionLog is created. Building it first
        # left a phantom "in progress" flight on the judged dashboard after
        # every bench dry-run.
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

    cfg = MissionConfig(waypoints=wps, survey_alt_m=args.survey_alt,
                        drop_alt_m=args.drop_alt, basestation_cmd=bs_cmd)

    stop = {'why': None}

    def _wind_up(signum, _frame):
        # Signal handlers must do almost nothing: set a flag and return. The
        # state machine notices on its next tick and exits through the normal
        # path, which is what commands RTL and closes the log.
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
        # Anything at all, including KeyboardInterrupt that landed between
        # ticks. The aircraft is airborne; get it home before re-raising.
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


def _spawn_or_reuse_worker(args, model, log):
    """Start detect_worker.py unless one is already publishing.

    'Already publishing' = the output file is fresher than FileDetector's
    staleness window, which is the same test FileDetector applies in flight:
    whoever wrote that recently owns the camera. This covers both a
    hand-started worker and one left behind by a SIGKILLed runner, and it is
    what makes double-spawning (two processes fighting over one V4L2 device)
    impossible from this entry point.

    The worker writes its own ~/logs/detect_worker.log (boardlog), so its
    stdout goes to /dev/null here: redirecting it into the same file would
    duplicate every line.

    A worker WE spawn is stopped again at interpreter exit (atexit covers
    every path out of main, including sys.exit and the wind-up-on-signal
    path). A reused worker is left running: we did not start it.
    """
    det_file = os.path.expanduser(args.det_file)
    try:
        fresh = (time.time() - os.stat(det_file).st_mtime
                 <= FileDetector.STALE_S)
    except OSError:
        fresh = False
    if fresh:
        log(f"[run] detect worker already publishing {det_file}, reusing it")
        return None
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
    """Best effort, and it really is best effort: if the runner is dying
    because the link died, this cannot work and says so rather than hanging."""
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
