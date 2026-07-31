"""Pixel -> ground offset for a nadir-mounted camera (TODO 11).

Turns "the puddle is at pixel (px, py) in a WxH frame" into "the puddle is N
metres north and E metres east of the drone", which is what lets the mission
fly to a puddle it saw off to one side instead of assuming everything is
directly beneath it.

Geometry, for a camera pointing straight down:

    image centre ..... the point directly under the drone (nadir)
    half-width ....... h * tan(hfov / 2) metres on the ground
    a pixel offset ... scales linearly in TANGENT space, not in pixels

So the ground offset of a pixel is

    x_right_m = (2*px/W - 1) * h * tan(hfov/2)
    y_down_m  = (2*py/H - 1) * h * tan(vfov/2)

where x_right/y_down are in CAMERA frame (right and down as the image is
drawn). Those are then rotated by the drone's heading into North/East.

Two things this deliberately does not model, because they are not worth the
error they would remove at these angles and altitudes:
  * lens distortion (the B525 is a webcam, not a metric camera);
  * aircraft roll/pitch. A hovering copter sits within a couple of degrees of
    level, and the mission only ever detects while holding position at survey
    altitude. Tilt is the dominant error term if that ever changes: at 15 m,
    2 degrees of tilt shifts the nadir point by ~0.5 m.

HFOV IS A MEASURED NUMBER, NOT A DATASHEET NUMBER. Run calibrate_fov() with a
tape measure once the camera is in its final housing; lenses vary between
units and the stated diagonal FOV of a webcam is frequently optimistic.
Until it is measured, DEFAULT_HFOV_DEG is a placeholder and every offset
computed from it inherits its error proportionally.
"""

import math

# Placeholder ONLY. Replace with the calibrate_fov() result for this camera.
# Marked so a grep for VERIFY finds it.
DEFAULT_HFOV_DEG = 60.0  # VERIFY: measure, do not trust


class CameraGeometry:
    """Nadir pinhole projection for one camera at one resolution.

    hfov_deg: horizontal field of view of the FULL frame, in degrees.
    vfov is derived from the frame aspect ratio rather than taken separately,
    because a rectilinear lens shares one focal length across both axes.
    """

    def __init__(self, width, height, hfov_deg=DEFAULT_HFOV_DEG):
        if width <= 0 or height <= 0:
            raise ValueError("frame size must be positive")
        if not 0 < hfov_deg < 180:
            raise ValueError("hfov_deg must be in (0, 180)")
        self.width = width
        self.height = height
        self.hfov_deg = hfov_deg
        # Focal length in pixels; one value for both axes (square pixels).
        self.f_px = (width / 2) / math.tan(math.radians(hfov_deg) / 2)

    @property
    def vfov_deg(self):
        return math.degrees(2 * math.atan((self.height / 2) / self.f_px))

    def ground_offset(self, px, py, height_m):
        """Pixel -> (right_m, down_m) in camera frame at `height_m` AGL.

        right_m: metres toward the right edge of the image.
        down_m:  metres toward the bottom edge of the image.
        """
        if height_m <= 0:
            raise ValueError("height_m must be positive")
        right_m = (px - self.width / 2) / self.f_px * height_m
        down_m = (py - self.height / 2) / self.f_px * height_m
        return right_m, down_m

    def footprint_m(self, height_m):
        """(width_m, height_m) of ground visible at this altitude. Useful for
        planning survey leg spacing so passes overlap instead of leaving gaps."""
        w = 2 * height_m * math.tan(math.radians(self.hfov_deg) / 2)
        h = w * self.height / self.width
        return w, h


def camera_to_ned(right_m, down_m, heading_deg, mount_yaw_deg=0.0):
    """Rotate a camera-frame offset into (north_m, east_m).

    heading_deg: aircraft heading, degrees clockwise from north (MAVLink hdg).
    mount_yaw_deg: camera rotation within the airframe, clockwise, 0 when the
        top of the image points out the nose of the aircraft.

    With the camera looking down and its top edge toward the nose, image
    "up" (negative down_m) is aircraft forward, and image "right" is aircraft
    right. So in body frame: forward = -down_m, right = +right_m. Rotating
    body->NED by heading then gives the result below.
    """
    theta = math.radians(heading_deg + mount_yaw_deg)
    forward_m, right_body_m = -down_m, right_m
    north_m = forward_m * math.cos(theta) - right_body_m * math.sin(theta)
    east_m = forward_m * math.sin(theta) + right_body_m * math.cos(theta)
    return north_m, east_m


def letterbox_to_frame(bx, by, frame_w, frame_h, box_size):
    """Undo the detector's letterbox: model-input pixel -> original frame pixel.

    OnnxDetector scales the frame by s = min(box/h, box/w) and centres it in a
    box_size square with grey padding. A box coordinate coming back from the
    model is therefore in letterboxed space and must be un-padded and
    un-scaled before it means anything about the real image.
    """
    s = min(box_size / frame_h, box_size / frame_w)
    nh, nw = round(frame_h * s), round(frame_w * s)
    top, left = (box_size - nh) // 2, (box_size - nw) // 2
    return (bx - left) / s, (by - top) / s


def calibrate_fov(distance_m, visible_width_m):
    """Measure horizontal FOV with a tape measure. Returns degrees.

    Procedure, done once, with the camera in its final housing:
      1. Point the camera squarely at a wall, lens `distance_m` from it.
      2. Capture one frame at the SAME resolution the mission uses.
      3. Mark on the wall where the left and right edges of the frame fall.
      4. Measure between the marks -> visible_width_m.

    Accuracy matters: the offset error is proportional. Getting the FOV 10%
    wrong puts a puddle seen at the frame edge about 10% of the footprint
    away from where the drone flies, which at 15 m altitude is roughly a
    metre.
    """
    if distance_m <= 0 or visible_width_m <= 0:
        raise ValueError("distance and width must be positive")
    return math.degrees(2 * math.atan((visible_width_m / 2) / distance_m))
