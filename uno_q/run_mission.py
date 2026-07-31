"""Onboard mission runner: the thing that actually flies on the UNO Q.

Wires the real parts together, where sitl_test.py wires the fake ones:

    sitl_test.py                 run_mission.py
    ------------                 --------------
    FakeDetector (planted)       OnnxDetector (camera + yolo26n ONNX)
    LogDropper (prints)          PixhawkServoDropper (moves the gate)
    tcp:127.0.0.1:5760           /dev/ttyXXX at 115200 (SERIAL4/5)
    no base station              basestation_cmd set, launched on DONE

Everything else, above all mission.py, is identical. That is the point: the
state machine that was proven in simulation is the one that flies.

    ~/venv/bin/python ~/uno_q/run_mission.py \
        --conn /dev/ttyHS0 --model ~/uno_q/best.onnx \
        --waypoints waypoints.txt --hfov-deg 58.2

Waypoint file: one "lat,lon" per line, blank lines and #comments ignored.

DRY RUN FIRST. --no-drop swaps in the logging dropper so the whole loop can
be flown with nothing to clean up afterwards, and --dry-run additionally
refuses to arm, so the detector and the log can be exercised on the bench.
"""

import argparse
import os
import sys
import time

from camera_geom import CameraGeometry
from detector import OnnxDetector
from dropper import LogDropper, PixhawkServoDropper
from mavlink_io import MavIO
from mission import Mission, MissionConfig
from missionlog import MissionLog


def read_waypoints(path):
    wps = []
    with open(os.path.expanduser(path)) as f:
        for n, line in enumerate(f, 1):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            try:
                lat, lon = (float(v) for v in line.split(','))
            except ValueError:
                sys.exit(f"{path}:{n}: expected 'lat,lon', got {line!r}")
            wps.append((lat, lon))
    if not wps:
        sys.exit(f"{path}: no waypoints")
    return wps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='/dev/ttyHS0',
                    help='serial device or SITL tcp: string')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--model', default='~/uno_q/best.onnx')
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--survey-alt', type=float, default=15.0)
    ap.add_argument('--drop-alt', type=float, default=3.0)
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--camera', type=int, default=1)
    ap.add_argument('--frame-w', type=int, default=1280)
    ap.add_argument('--frame-h', type=int, default=720)
    ap.add_argument('--hfov-deg', type=float, default=None,
                    help='MEASURED horizontal FOV (camera_geom.calibrate_fov). '
                         'Omit to keep the nadir assumption.')
    ap.add_argument('--mount-yaw-deg', type=float, default=0.0)
    ap.add_argument('--servo-channel', type=int, default=9)
    ap.add_argument('--servo-closed-us', type=int, default=1000)
    ap.add_argument('--servo-open-us', type=int, default=1900)
    ap.add_argument('--no-drop', action='store_true',
                    help='log drops instead of moving the servo')
    ap.add_argument('--dry-run', action='store_true',
                    help='never arm; exercise detector and logging only')
    ap.add_argument('--no-basestation', action='store_true')
    args = ap.parse_args()

    wps = read_waypoints(args.waypoints)
    model = os.path.expanduser(args.model)
    if not os.path.exists(model):
        sys.exit(f"model not found: {model}")

    print(f"[run] connecting {args.conn} ...")
    io = MavIO(args.conn, baud=args.baud)
    io.wait_ready()
    print(f"[run] heartbeat from system {io.conn.target_system}")
    io.setup_streams()

    geom = None
    if args.hfov_deg:
        geom = CameraGeometry(args.frame_w, args.frame_h, args.hfov_deg)
        fw, fh = geom.footprint_m(args.survey_alt)
        print(f"[run] camera {args.hfov_deg:.1f}deg hfov -> footprint at "
              f"{args.survey_alt:.0f}m is {fw:.1f} x {fh:.1f} m")
    else:
        print("[run] no --hfov-deg: detections assumed directly below "
              "the aircraft (nadir)")

    detector = OnnxDetector(model, camera=args.camera, conf=args.conf,
                            geom=geom, mount_yaw_deg=args.mount_yaw_deg)

    if args.no_drop or args.dry_run:
        dropper = LogDropper()
        print("[run] DROPS DISABLED (logging dropper)")
    else:
        dropper = PixhawkServoDropper(
            io, channel=args.servo_channel,
            closed_us=args.servo_closed_us, open_us=args.servo_open_us)

    recorder = MissionLog(args.data_dir)
    print(f"[run] logging to {recorder.path}")

    bs_cmd = None
    if not args.no_basestation:
        bs = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'basestation', 'app.py')
        bs_cmd = [sys.executable, bs, '--data-dir', args.data_dir]

    cfg = MissionConfig(waypoints=wps, survey_alt_m=args.survey_alt,
                        drop_alt_m=args.drop_alt, basestation_cmd=bs_cmd)

    if args.dry_run:
        print("[run] DRY RUN: not arming. Polling the detector for 30s.")
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            io.step()
            det = detector.poll(io.tel)
            if det:
                print(f"[run] detection {det.lat:.7f},{det.lon:.7f} "
                      f"conf {det.confidence:.2f}")
        return

    print(f"[run] {len(wps)} waypoints, survey {args.survey_alt}m, "
          f"drop at {args.drop_alt}m rangefinder")
    final = Mission(io, detector, dropper, cfg, recorder=recorder).run()
    print(f"[run] finished in state {final}, drops={dropper.fired}")


if __name__ == '__main__':
    main()
