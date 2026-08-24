"""Pixel -> ground offset for a nadir-mounted camera (TODO 11).

Turns "the puddle is at pixel (px, py) in a WxH frame" into "the puddle is N
metres north and E metres east of the drone", which is what lets the mission
fly to a puddle it saw off to one side instead of assuming everything is
directly beneath it.

Geometry, for a camera pointing straight down:

    image centre ..... the point directly under the drone (nadir)
    half-width ....... h * tan(hfov / 2) metres on the ground
    a pixel offset ... scales linearly in tangent space, not in pixels

So the ground offset of a pixel is

    x_right_m = (2*px/W - 1) * h * tan(hfov/2)
    y_down_m  = (2*py/H - 1) * h * tan(vfov/2)

where x_right/y_down are in camera frame (right and down as the image is
drawn). Those are then rotated by the drone's heading into North/East.

Two things this deliberately does not model, because they are not worth the
error they would remove at these angles and altitudes:
  * lens distortion (the B525 is a webcam, not a metric camera);
  * aircraft roll/pitch. A hovering copter sits within a couple of degrees of
    level, and the mission only ever detects while holding position at survey
    altitude. Tilt is the dominant error term if that ever changes: at 15 m,
    2 degrees of tilt shifts the nadir point by ~0.5 m.

HFOV is a measured number, not a datasheet number. Run calibrate_fov() with a
tape measure once the camera is in its final housing; lenses vary between
units and the stated diagonal FOV of a webcam is frequently optimistic.
Until it is measured, DEFAULT_HFOV_DEG is a placeholder and every offset
computed from it inherits its error proportionally.
"""

import math

# Measured, no longer a placeholder. 2026-08-15 at the farm, wall method with
# a tape measure (uno_q/calibrate_camera.py): 0.950 m of wall visible at
# 0.890 m => 2*atan(0.475/0.890) = 56.18 deg, at the mission's own 1280x720.
# Consequences that follow from this number, recorded so they are not
# recomputed: footprint at 15 m is 16.0 x 9.0 m; a 0.6 m target at 15 m is
# only ~24 px in the model's 640 input, which is why the farm target is a
# metres-wide wet patch and not the tray.
DEFAULT_HFOV_DEG = 56.2

# The camera is mounted rotated 90 deg: the 1280 px axis runs fore-aft, so a
# forward-facing frame covers more ground ahead-behind than side-to-side
# (user, 2026-08-24). Verify the sign before a live drop: object in front of
# the nose appearing in the left half of the photo means +90 is right;
# appearing in the right half means this must be -90.
MOUNT_YAW_DEG = 90.0


def footprint_track_m(geom, height_m, mount_yaw_deg=MOUNT_YAW_DEG):
    """(across_track_m, along_track_m) of ground covered at height_m.

    footprint_m() is axis-labelled (1280 axis, 720 axis) and says nothing
    about the airframe; this maps it through the mount rotation so survey
    row spacing and waypoint spacing use the right sides.
    """
    w, h = geom.footprint_m(height_m)
    if abs(math.sin(math.radians(mount_yaw_deg))) > 0.5:
        return h, w         # rotated mount: 720 axis across, 1280 along
    return w, h


class CameraGeometry:
    """Nadir pinhole projection for one camera at one resolution.

    hfov_deg: horizontal field of view of the full frame, in degrees.
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


def ground_area_m2(geom, px1, py1, px2, py2, height_m):
    """Approximate ground area of a detection box, in square metres.

    Computed by projecting the box's two opposite corners to the ground and
    multiplying the resulting side lengths, not by scaling the pixel area by a
    single factor: ground_offset is linear in tangent space, so a box far from
    the image centre covers more ground per pixel than one at the centre, and
    a single scale factor would understate it.

    Three errors this carries, all of which matter if a dose is sized from it:
      * a bounding box overestimates any non-rectangular puddle, without
        bound. An L-shaped or diagonal puddle can be a small fraction of its
        box. Honest area needs a segmentation model, which is a different
        training run (PROJECT_STATE records the dataset already has polygons
        that the detect model discards).
      * it inherits the FOV error proportionally, squared: a 10% FOV error is
        a ~20% area error.
      * height_m must be true AGL. Above the rangefinder's range that is the
        EKF's above-home figure, which is wrong over sloping ground.
    Treat the result as an order-of-magnitude input to dosing, not a
    measurement, and say so wherever it is displayed.
    """
    ax, ay = geom.ground_offset(px1, py1, height_m)
    bx, by = geom.ground_offset(px2, py2, height_m)
    return abs(bx - ax) * abs(by - ay)


def camera_to_ned(right_m, down_m, heading_deg, mount_yaw_deg=0.0):
    """Rotate a camera-frame offset into (north_m, east_m).

    heading_deg: aircraft heading, degrees clockwise from north (MAVLink hdg).
    mount_yaw_deg: camera rotation within the airframe. 0 when the top of the
        image points out the nose. Positive is clockwise as seen in the image,
        which for a belly-mounted downward camera means clockwise looking up
        at the aircraft from the ground, not looking down at it from above.
        Stated explicitly because the sign is otherwise a coin flip: rotate
        the camera 90 degrees, fly the mission, and if targets land to the
        left of the puddle instead of the right, negate this.

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
      2. Capture one frame at the same resolution the mission uses.
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
