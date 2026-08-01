"""Detection sources for the mission state machine.

The mission only ever sees this interface; swapping FakeDetector (SITL) for the
real ONNX camera detector later must not change mission.py.
"""

import math
import time

from camera_geom import camera_to_ned, letterbox_to_frame

# A rangefinder reading older than this is not trusted as an AGL source.
RNG_FRESH_S = 1.0


class Detection:
    def __init__(self, lat, lon, confidence=1.0):
        self.lat = lat
        self.lon = lon
        self.confidence = confidence


class DetectionSource:
    def poll(self, tel):
        """Called every tick while in SURVEY. Return Detection or None."""
        raise NotImplementedError


class FakeDetector(DetectionSource):
    """SITL stand-in: 'sees' a planted puddle once the drone flies within
    radius_m of it (proxy for the nadir camera footprint at survey alt)."""

    def __init__(self, lat, lon, radius_m=6.0, max_fires=1):
        self.lat = lat
        self.lon = lon
        self.radius_m = radius_m
        self.fires_left = max_fires

    def poll(self, tel):
        if self.fires_left <= 0 or tel.lat is None:
            return None
        if dist_m(tel.lat, tel.lon, self.lat, self.lon) <= self.radius_m:
            self.fires_left -= 1
            return Detection(self.lat, self.lon)
        return None


class OnnxDetector(DetectionSource):
    """Real detector: B525 still -> yolo26n ONNX -> Detection at drone nadir.

    The yolo26 export is end-to-end (output (1,300,6) rows of
    x1,y1,x2,y2,conf,cls, NMS baked in), so postprocessing is one threshold.
    Preprocessing (letterbox 640, RGB, /255) is byte-identical to
    spotcheck_onnx.py, which proved laptop/board parity on 2026-07-29.

    Detection position comes from camera_geom when a CameraGeometry is
    supplied (box centre -> ground offset -> NED -> lat/lon), and degrades to
    the drone's own lat/lon (nadir) whenever the geometry or the telemetry it
    needs is missing. It never raises mid-mission for want of a heading.

    Behaviour shaped by the mission loop being single-threaded:
      - poll() runs capture+inference at most every interval_s (board
        inference is ~0.5s; unthrottled polling would starve the MAVLink
        pump). Between inferences poll() returns None instantly.
      - A fire is suppressed within skip_radius_m of any earlier fire,
        else the just-treated puddle would re-latch forever after CLIMB.

    frame_source: optional callable returning a BGR image, for tests and
    SITL (defaults to the B525 via OpenCV V4L2 at MJPG 1280x720).
    """

    SIZE = 640

    def __init__(self, model_path, camera=1, conf=0.5, interval_s=1.0,
                 skip_radius_m=8.0, frame_source=None, log=print,
                 geom=None, mount_yaw_deg=0.0):
        import numpy as np
        import onnxruntime as ort
        # conf <= 0 would accept the end-to-end export's zero-confidence
        # padding rows as detections (the (1,300,6) output is always 300 rows
        # long, most of them empty), so the aircraft would fly to whatever
        # pixel happened to sort first. Refuse it here.
        if not 0 < conf <= 1:
            raise ValueError(f"conf must be in (0, 1], got {conf}")
        self._np = np
        self.conf = conf
        self.interval_s = interval_s
        self.skip_radius_m = skip_radius_m
        self.log = log
        # geom: CameraGeometry, or None to keep the nadir assumption.
        self.geom = geom
        self.mount_yaw_deg = mount_yaw_deg
        self._last_t = 0.0
        self._fired = []          # (lat, lon) of every past fire
        self._warned = False
        self._size_warned = False
        self.sess = ort.InferenceSession(
            model_path, providers=['CPUExecutionProvider'])
        self._in = self.sess.get_inputs()[0].name
        if frame_source is not None:
            self._grab = frame_source
        else:
            import cv2
            self._cv2 = cv2
            self._lock_focus(camera)
            self.cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._grab = self._grab_camera

    def _lock_focus(self, camera):
        """Focus to infinity, best effort. Two separate v4l2-ctl calls:
        both in one transaction fails (EACCES, seen on the board)."""
        import subprocess
        dev = f"/dev/video{camera}"
        for ctrl in ("focus_automatic_continuous=0", "focus_absolute=0"):
            r = subprocess.run(['v4l2-ctl', '-d', dev, f'--set-ctrl={ctrl}'],
                               capture_output=True)
            if r.returncode != 0:
                self.log(f"[detector] focus ctrl failed ({ctrl}): "
                         f"{r.stderr.decode().strip()}")

    def _grab_camera(self):
        self.cap.grab()                    # flush possibly-stale buffer
        ok, frame = self.cap.read()
        return frame if ok else None

    def preflight(self):
        """Bench check, called before arming. True if the camera works.

        Exists because the alternative is discovering a dead camera by
        flying an entire survey and landing with zero detections, which
        looks identical to "there were no puddles".
        """
        frame = self._grab()
        if frame is None:
            self.log("[detector] PREFLIGHT FAIL: no frame from the camera")
            return False
        h, w = frame.shape[:2]
        self.log(f"[detector] preflight ok: {w}x{h} frame")
        if self.geom is not None and (w, h) != (self.geom.width,
                                                self.geom.height):
            self.log(f"[detector] PREFLIGHT FAIL: camera negotiated {w}x{h} "
                     f"but the geometry was built for {self.geom.width}x"
                     f"{self.geom.height}. Fix --frame-w/--frame-h (and "
                     f"remeasure the FOV at that resolution) rather than "
                     f"flying with offsets scaled by the wrong factor.")
            return False
        return True

    def poll(self, tel):
        if tel.lat is None:
            return None
        now = time.monotonic()
        if now - self._last_t < self.interval_s:
            return None
        self._last_t = now

        frame = self._grab()
        if frame is None:
            if not self._warned:
                self.log("[detector] camera frame grab FAILED")
                self._warned = True
            return None
        self._warned = False

        np = self._np
        h, w = frame.shape[:2]
        # The geometry is only valid at the resolution it was built for. If
        # MJPG negotiation quietly fell back to something else mid-flight,
        # every offset would be scaled wrong with no visible symptom, so drop
        # to the nadir assumption instead: a known bounded error beats an
        # invisible systematic one.
        if self.geom is not None and (w, h) != (self.geom.width,
                                                self.geom.height):
            if not self._size_warned:
                self.log(f"[detector] frame is {w}x{h}, geometry expects "
                         f"{self.geom.width}x{self.geom.height}: falling back "
                         f"to nadir for the rest of the flight")
                self._size_warned = True
            self.geom = None
        s = min(self.SIZE / h, self.SIZE / w)
        nh, nw = round(h * s), round(w * s)
        top, left = (self.SIZE - nh) // 2, (self.SIZE - nw) // 2
        boxed = np.full((self.SIZE, self.SIZE, 3), 114, dtype=np.uint8)
        if not hasattr(self, '_cv2'):
            import cv2
            self._cv2 = cv2
        boxed[top:top + nh, left:left + nw] = self._cv2.resize(frame, (nw, nh))
        x = boxed[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255

        out = self.sess.run(None, {self._in: x})[0][0]
        # EVERY row above threshold, best first, not just the argmax. A single
        # frame routinely holds a puddle we already treated and one we have
        # not; taking only the strongest row means the treated one suppresses
        # its neighbour on every frame until it leaves the footprint.
        rows = sorted((r for r in out if float(r[4]) >= self.conf),
                      key=lambda r: -float(r[4]))
        for row in rows:
            conf = float(row[4])
            lat, lon, how = self._locate(tel, row, w, h)
            # Dedup on the SITE, not on where the drone happened to be: the
            # same puddle seen from two positions must resolve to one target.
            if any(dist_m(lat, lon, flat, flon) <= self.skip_radius_m
                   for flat, flon in self._fired):
                continue                  # already treated this site
            self._fired.append((lat, lon))
            self.log(f"[detector] puddle conf {conf:.2f} {how}"
                     + (f" ({len(rows)} candidates this frame)"
                        if len(rows) > 1 else ""))
            return Detection(lat, lon, conf)
        return None

    @staticmethod
    def _height_agl(tel):
        """Height above the ground the camera is looking at, and its source.

        ground_offset()'s contract is AGL, not altitude above home, and the
        two are only the same over ground level with the launch point. Prefer
        the downward rangefinder whenever it has a fresh valid return, since
        that IS the AGL by definition; fall back to the EKF's above-home
        figure otherwise (which is what survey altitude will normally use,
        the TF-Luna being blind above ~8 m).
        """
        if (tel.rng_valid and tel.rng_m is not None
                and time.monotonic() - tel.rng_t < RNG_FRESH_S):
            return tel.rng_m, 'rng'
        if tel.rel_alt_m is not None and tel.rel_alt_m > 0:
            return tel.rel_alt_m, 'ekf'
        return None, 'none'

    def _locate(self, tel, row, frame_w, frame_h):
        """Detection box centre -> (lat, lon, description).

        Falls back to the nadir assumption whenever the geometry is not
        configured or the telemetry it needs is missing, so a lost heading
        degrades the target to 'below us' instead of throwing mid-mission.
        """
        height_m, source = self._height_agl(tel)
        if (self.geom is None or height_m is None
                or tel.heading_deg is None):
            return tel.lat, tel.lon, 'at nadir'
        cx = (float(row[0]) + float(row[2])) / 2
        cy = (float(row[1]) + float(row[3])) / 2
        px, py = letterbox_to_frame(cx, cy, frame_w, frame_h, self.SIZE)
        right_m, down_m = self.geom.ground_offset(px, py, height_m)
        n, e = camera_to_ned(right_m, down_m, tel.heading_deg,
                             self.mount_yaw_deg)
        lat, lon = offset_latlon(tel.lat, tel.lon, n, e)
        return lat, lon, f"offset {n:+.1f}m N {e:+.1f}m E @{height_m:.1f}m {source}"


def dist_m(lat1, lon1, lat2, lon2):
    """Equirectangular approximation; fine for survey-scale distances."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371000.0 * math.hypot(dlat, dlon)


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 6371000.0
    dlon = east_m / (6371000.0 * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)
