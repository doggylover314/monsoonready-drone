"""Unit tests for the pixel -> ground geometry. Pure maths, no hardware.

    .venv/bin/python uno_q/test_camera_geom.py
"""

import math
import sys

from camera_geom import (CameraGeometry, calibrate_fov, camera_to_ned,
                         letterbox_to_frame)

FAILS = []


def check(name, got, want, tol=1e-6):
    ok = abs(got - want) <= tol
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: got {got:.6f} want {want:.6f}")
    if not ok:
        FAILS.append(name)


def main():
    cam = CameraGeometry(1280, 720, hfov_deg=60.0)

    print("footprint and fov")
    # At 10 m, half-width = 10*tan(30) = 5.7735, so full width = 11.547 m.
    w, h = cam.footprint_m(10.0)
    check("footprint width @10m", w, 2 * 10 * math.tan(math.radians(30)))
    check("footprint aspect", w / h, 1280 / 720)
    # Vertical FOV must follow from the aspect ratio, not be independent.
    check("vfov derived", cam.vfov_deg,
          math.degrees(2 * math.atan((720 / 2) / cam.f_px)))

    print("nadir is the image centre")
    r, d = cam.ground_offset(640, 360, 15.0)
    check("centre right", r, 0.0)
    check("centre down", d, 0.0)

    print("frame edges map to half the footprint")
    r, _ = cam.ground_offset(1280, 360, 10.0)
    check("right edge", r, 10 * math.tan(math.radians(30)))
    r, _ = cam.ground_offset(0, 360, 10.0)
    check("left edge", r, -10 * math.tan(math.radians(30)))

    print("offset scales linearly with altitude")
    r10, _ = cam.ground_offset(1000, 360, 10.0)
    r20, _ = cam.ground_offset(1000, 360, 20.0)
    check("double altitude doubles offset", r20, 2 * r10)

    print("heading rotation")
    # Puddle above image centre = ahead of the aircraft.
    fwd_right, fwd_down = 0.0, -5.0
    n, e = camera_to_ned(fwd_right, fwd_down, heading_deg=0)
    check("nose north: north", n, 5.0)
    check("nose north: east", e, 0.0)
    n, e = camera_to_ned(fwd_right, fwd_down, heading_deg=90)
    check("nose east: north", n, 0.0, tol=1e-9)
    check("nose east: east", e, 5.0)
    n, e = camera_to_ned(fwd_right, fwd_down, heading_deg=180)
    check("nose south: north", n, -5.0)
    # Image-right while pointing north is due east.
    n, e = camera_to_ned(5.0, 0.0, heading_deg=0)
    check("image right, nose north: east", e, 5.0)
    check("image right, nose north: north", n, 0.0)
    # A camera mounted 90 deg rotated in the airframe must cancel out.
    n, e = camera_to_ned(0.0, -5.0, heading_deg=0, mount_yaw_deg=90)
    check("mount yaw 90 sends forward east", e, 5.0)

    print("letterbox undo")
    # 1280x720 into a 640 box: s = 640/1280 = 0.5, so 720*0.5=360 tall,
    # padded (640-360)//2 = 140 top.
    x, y = letterbox_to_frame(320, 320, 1280, 720, 640)
    check("letterbox centre x", x, 640.0)
    check("letterbox centre y", y, 360.0)
    x, y = letterbox_to_frame(0, 140, 1280, 720, 640)
    check("letterbox top-left x", x, 0.0)
    check("letterbox top-left y", y, 0.0)

    print("fov calibration helper")
    # A wall 1 m away showing 2*tan(30) m of width is a 60 degree lens.
    check("calibrate 60deg", calibrate_fov(1.0, 2 * math.tan(math.radians(30))),
          60.0, tol=1e-9)

    print("end-to-end: puddle at a frame corner, drone heading north")
    # Bottom-right corner at 15 m: right and behind the aircraft.
    r, d = cam.ground_offset(1280, 720, 15.0)
    n, e = camera_to_ned(r, d, heading_deg=0)
    check("corner is behind (south)", n, -d, tol=1e-9)
    check("corner is to the right (east)", e, r, tol=1e-9)
    if n >= 0:
        FAILS.append("corner should be south of the drone")

    print()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s): {FAILS}")
        sys.exit(1)
    print("PASS: all geometry checks")


if __name__ == '__main__':
    main()
