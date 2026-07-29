"""Detection sources for the mission state machine.

The mission only ever sees this interface; swapping FakeDetector (SITL) for the
real ONNX camera detector later must not change mission.py.
"""

import math


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

    Detection position = the drone's own lat/lon (nadir assumption);
    pixel->ground offset from camera intrinsics is a later refinement.

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
                 skip_radius_m=8.0, frame_source=None, log=print):
        import numpy as np
        import onnxruntime as ort
        self._np = np
        self.conf = conf
        self.interval_s = interval_s
        self.skip_radius_m = skip_radius_m
        self.log = log
        self._last_t = 0.0
        self._fired = []          # (lat, lon) of every past fire
        self._warned = False
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

    def poll(self, tel):
        import time
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
        best = max((float(r[4]) for r in out), default=0.0)
        if best < self.conf:
            return None
        for flat, flon in self._fired:
            if dist_m(tel.lat, tel.lon, flat, flon) <= self.skip_radius_m:
                return None               # already treated this area
        self._fired.append((tel.lat, tel.lon))
        self.log(f"[detector] puddle conf {best:.2f} at nadir")
        return Detection(tel.lat, tel.lon, best)


def dist_m(lat1, lon1, lat2, lon2):
    """Equirectangular approximation; fine for survey-scale distances."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371000.0 * math.hypot(dlat, dlon)


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 6371000.0
    dlon = east_m / (6371000.0 * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)
