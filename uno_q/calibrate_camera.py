#!/usr/bin/env python3
"""Measure the camera's horizontal FOV, the one number that decides whether
the aircraft flies to the PUDDLE or merely to where it was standing.

WHY THIS MATTERS MORE THAN IT LOOKS. run_mission only builds a
CameraGeometry when --hfov-deg is given. Without it, detector._locate falls
straight through to the nadir assumption and reports the puddle at the
AIRCRAFT'S OWN lat/lon. The aircraft then descends and drops where it was
when it first saw the water, which at 15 m with a ~17 m footprint can be
most of half a footprint away from the target. For a 60 cm tray that is a
clean miss. Ten minutes with a tape measure converts that into metres.

TWO METHODS. Both need a tape measure and nothing else.

  1. WALL (no pixel measuring, best in the field)
       Point the camera square at a wall, lens exactly D metres from it.
       Look at the live frame, mark on the wall where the LEFT and RIGHT
       edges of the image fall, measure between the marks -> W.
         ./python uno_q/calibrate_camera.py --distance 2.0 --width 2.22

  2. OBJECT (when marking frame edges is awkward)
       Lay something of known length L flat, square to the camera, D metres
       away. Save a frame, open it, read off how many PIXELS it spans, and
       give the frame width in pixels too.
         ./python uno_q/calibrate_camera.py --distance 2.0 \
             --object-m 1.0 --object-px 590 --frame-px 1280

  --grab N  first captures one frame from /dev/videoN and writes it to
            fov_frame.jpg, so method 2 has an image to measure in and
            method 1 has proof the camera was aimed square.

ACCURACY: the offset error is PROPORTIONAL to the FOV error, so a 10%
mistake here puts a frame-edge puddle ~10% of the footprint from where the
aircraft flies. Measure D and W to the centimetre and it is a non-issue.
Do it at the SAME RESOLUTION the mission uses (1280x720 by default): FOV
changes when a sensor crops rather than scales.

Prints the --hfov-deg value to pass to run_mission, and the ground
footprint at survey altitude so the survey row spacing can be sanity
checked against it.
"""

import argparse
import math
import sys

from camera_geom import CameraGeometry, calibrate_fov


def grab_frame(camera, path):
    try:
        import cv2
    except ImportError:
        print("opencv not available; skipping --grab")
        return False
    cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.grab()
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        print(f"no frame from /dev/video{camera}")
        return False
    cv2.imwrite(path, frame)
    h, w = frame.shape[:2]
    print(f"saved {path} ({w}x{h}) -- measure in THIS image, and check the "
          f"camera was square to the target")
    return True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--distance', type=float,
                    help='metres from lens to wall/object')
    ap.add_argument('--width', type=float,
                    help='method 1: metres between the marked frame edges')
    ap.add_argument('--object-m', type=float,
                    help='method 2: real length of the object, metres')
    ap.add_argument('--object-px', type=float,
                    help='method 2: how many pixels it spans')
    ap.add_argument('--frame-px', type=float, default=1280,
                    help='method 2: frame width in pixels (default 1280)')
    ap.add_argument('--grab', type=int, default=None, metavar='N',
                    help='capture one frame from /dev/videoN first')
    ap.add_argument('--frame-w', type=int, default=1280)
    ap.add_argument('--frame-h', type=int, default=720)
    ap.add_argument('--survey-alt', type=float, default=15.0)
    args = ap.parse_args()

    if args.grab is not None:
        grab_frame(args.grab, 'fov_frame.jpg')

    if args.distance and args.width:
        hfov = calibrate_fov(args.distance, args.width)
        how = (f"wall method: {args.width:.3f} m visible at "
               f"{args.distance:.3f} m")
    elif args.distance and args.object_m and args.object_px:
        if args.object_px <= 0 or args.object_m <= 0 or args.distance <= 0:
            sys.exit("distance, object-m and object-px must all be positive")
        # Pixels per metre at that distance -> the full frame's real width
        # there -> the same arctangent as the wall method.
        visible_w = args.frame_px / args.object_px * args.object_m
        hfov = calibrate_fov(args.distance, visible_w)
        how = (f"object method: {args.object_m:.3f} m spans "
               f"{args.object_px:.0f} px of {args.frame_px:.0f}, so the frame "
               f"is {visible_w:.3f} m wide at {args.distance:.3f} m")
    else:
        sys.exit("give either --distance and --width, or --distance, "
                 "--object-m, --object-px. See --help for the procedures.")

    if not 0 < hfov < 180:
        sys.exit(f"computed {hfov:.1f} deg, which is not a sane FOV; "
                 f"re-check the measurements")
    geom = CameraGeometry(args.frame_w, args.frame_h, hfov)
    fw, fh = geom.footprint_m(args.survey_alt)
    print(f"\n{how}")
    print(f"HORIZONTAL FOV = {hfov:.2f} deg   (vertical {geom.vfov_deg:.2f})")
    print(f"ground footprint at {args.survey_alt:.0f} m: "
          f"{fw:.1f} x {fh:.1f} m")
    print(f"a 0.6 m target at {args.survey_alt:.0f} m spans about "
          f"{0.6 / fw * args.frame_w:.0f} px of {args.frame_w}, i.e. "
          f"{0.6 / fw * 640:.0f} px in the model's 640 input")
    print(f"\nPASS THIS ON EVERY FLIGHT:  --hfov-deg {hfov:.1f}")
    print("Without it the mission assumes every puddle is directly below "
          "the aircraft and the drop can miss by half a footprint.")


if __name__ == '__main__':
    main()
