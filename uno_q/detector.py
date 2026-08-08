"""Detection sources for the mission state machine.

The mission only ever sees this interface; swapping FakeDetector (SITL) for the
real ONNX camera detector later must not change mission.py.
"""

import math
import os
import time

from camera_geom import camera_to_ned, letterbox_to_frame

# A rangefinder reading older than this is not trusted as an AGL source.
RNG_FRESH_S = 1.0


def quiet_import_onnxruntime():
    """Import onnxruntime with stderr parked on /dev/null for the duration.

    ort 1.28 probes /sys/class/drm for GPUs AT IMPORT TIME and warns loudly
    when the probe fails, which on this GPU-less board is the expected
    outcome every single run. set_default_logger_severity() cannot suppress
    it: the function does not exist until the import that emits the warning
    has already completed. The redirect is at the fd level because the
    warning comes from C++ code that writes fd 2 directly, not sys.stderr.
    A real import failure still surfaces: the exception propagates after
    stderr is restored.
    """
    import os as _os
    saved = _os.dup(2)
    devnull = _os.open(_os.devnull, _os.O_WRONLY)
    _os.dup2(devnull, 2)
    try:
        import onnxruntime as ort
    finally:
        _os.dup2(saved, 2)
        _os.close(saved)
        _os.close(devnull)
    ort.set_default_logger_severity(3)   # errors only from here on, too
    return ort


class Detection:
    def __init__(self, lat, lon, confidence=1.0):
        self.lat = lat
        self.lon = lon
        self.confidence = confidence


class DetectionSource:
    def poll(self, tel):
        """Called every tick while in SURVEY. Return Detection or None."""
        raise NotImplementedError


class _RowResolver(DetectionSource):
    """Shared 'model rows -> Detection' logic for OnnxDetector (in-process
    inference) and FileDetector (detect_worker.py results read from a file).

    Rows are in LETTERBOX (model input) space: x1,y1,x2,y2,conf[,cls].
    at_t: the time.monotonic() the frame's telemetry was captured at, so
    rangefinder freshness is judged against frame time, not read time.
    """

    def _init_resolver(self, conf, skip_radius_m, geom, mount_yaw_deg, log):
        # conf <= 0 would accept the end-to-end export's zero-confidence
        # padding rows as detections (the (1,300,6) output is always 300 rows
        # long, most of them empty), so the aircraft would fly to whatever
        # pixel happened to sort first. Refuse it here.
        if not 0 < conf <= 1:
            raise ValueError(f"conf must be in (0, 1], got {conf}")
        self.conf = conf
        self.skip_radius_m = skip_radius_m
        self.geom = geom
        self.mount_yaw_deg = mount_yaw_deg
        self.log = log
        self._fired = []          # (lat, lon) of every past fire
        self._size_warned = False

    SIZE = 640

    def _resolve(self, tel, rows, w, h, at_t=None):
        """Rows above threshold -> first un-treated site, or None."""
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
        # EVERY row above threshold, best first, not just the argmax. A single
        # frame routinely holds a puddle we already treated and one we have
        # not; taking only the strongest row means the treated one suppresses
        # its neighbour on every frame until it leaves the footprint.
        rows = sorted((r for r in rows if float(r[4]) >= self.conf),
                      key=lambda r: -float(r[4]))
        for row in rows:
            conf = float(row[4])
            lat, lon, how = self._locate(tel, row, w, h, at_t)
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
    def _height_agl(tel, at_t=None):
        """Height above the ground the camera is looking at, and its source.

        ground_offset()'s contract is AGL, not altitude above home, and the
        two are only the same over ground level with the launch point. Prefer
        the downward rangefinder whenever it has a fresh valid return, since
        that IS the AGL by definition; fall back to the EKF's above-home
        figure otherwise (which is what survey altitude will normally use,
        the TF-Luna being blind above ~8 m).
        """
        ref = at_t if at_t is not None else time.monotonic()
        if (tel.rng_valid and tel.rng_m is not None
                and ref - tel.rng_t < RNG_FRESH_S):
            return tel.rng_m, 'rng'
        if tel.rel_alt_m is not None and tel.rel_alt_m > 0:
            return tel.rel_alt_m, 'ekf'
        return None, 'none'

    def _locate(self, tel, row, frame_w, frame_h, at_t=None):
        """Detection box centre -> (lat, lon, description).

        Falls back to the nadir assumption whenever the geometry is not
        configured or the telemetry it needs is missing, so a lost heading
        degrades the target to 'below us' instead of throwing mid-mission.
        """
        height_m, source = self._height_agl(tel, at_t)
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


class OnnxDetector(_RowResolver):
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

    NOTE 2026-08-01: in-process poll() blocks the mission loop for the full
    capture+inference time, which is why flights now default to
    detect_worker.py + FileDetector (below). This class stays both as the
    inference engine the worker itself uses (infer_rows) and as the
    --inline-detector fallback.

    frame_source: optional callable returning a BGR image, for tests and
    SITL (defaults to the B525 via OpenCV V4L2 at MJPG 1280x720).
    """

    def __init__(self, model_path, camera=1, conf=0.5, interval_s=1.0,
                 skip_radius_m=8.0, frame_source=None, log=print,
                 geom=None, mount_yaw_deg=0.0):
        import numpy as np
        ort = quiet_import_onnxruntime()
        self._init_resolver(conf, skip_radius_m, geom, mount_yaw_deg, log)
        self._np = np
        self.interval_s = interval_s
        self._last_t = 0.0
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

    def infer_rows(self):
        """Grab one frame and run the model. No telemetry, no dedup: this is
        the piece detect_worker.py runs in its own process.

        Returns (t_frame_wall, w, h, rows, frame) with rows = every model row
        at or above conf, in letterbox space; or None if the camera gave no
        frame. t_frame is time.time() AT CAPTURE, not after inference: the
        consumer pairs it with the telemetry from when the frame was taken.
        The BGR frame comes back too so a caller can save annotated evidence
        without grabbing a second, different image.
        """
        frame = self._grab()
        t_frame = time.time()
        if frame is None:
            return None
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
        rows = [r for r in out if float(r[4]) >= self.conf]
        return t_frame, w, h, rows, frame

    def poll(self, tel):
        if tel.lat is None:
            return None
        now = time.monotonic()
        if now - self._last_t < self.interval_s:
            return None
        self._last_t = now

        res = self.infer_rows()
        if res is None:
            if not self._warned:
                self.log("[detector] camera frame grab FAILED")
                self._warned = True
            return None
        self._warned = False
        _t, w, h, rows, _frame = res
        return self._resolve(tel, rows, w, h)


class _TelSnap:
    """Frozen copy of the Telemetry fields _locate needs, stamped with when
    it was taken, so a detection computed seconds later still uses the fix
    from frame time."""

    __slots__ = ('t_wall', 't_mono', 'lat', 'lon', 'rel_alt_m',
                 'heading_deg', 'rng_m', 'rng_valid', 'rng_t')

    def __init__(self, tel):
        self.t_wall = time.time()
        self.t_mono = time.monotonic()
        self.lat = tel.lat
        self.lon = tel.lon
        self.rel_alt_m = tel.rel_alt_m
        self.heading_deg = tel.heading_deg
        self.rng_m = tel.rng_m
        self.rng_valid = tel.rng_valid
        self.rng_t = tel.rng_t


class FileDetector(_RowResolver):
    """Reads detect_worker.py results from a file instead of inferring.

    WHY (2026-08-01, measured): in-process inference blocks the single-
    threaded mission loop for the model's whole latency (yolo26n 511ms,
    yolo26s 1518ms on the board), and for that time no MAVLink is pumped and
    no pilot-override is noticed. With inference in its own process the loop
    never blocks; poll() here is a stat() plus, when the worker has produced
    a new result, one small JSON read.

    THE PAIRING RULE, which is the one subtle part: a result read now is for
    a frame captured one inference ago, and at survey speed the aircraft has
    moved metres since. So poll() keeps a short history of telemetry
    snapshots and computes the puddle position against the snapshot closest
    to the frame's capture time, not against the current fix. Same-machine
    wall clock on both sides, so the timestamps are comparable.

    Liveness: the worker writes every cycle even with zero detections, so
    the file's mtime is its heartbeat. Stale file => warn (worker died);
    payload camera_ok false => warn (worker alive, camera dead). Both
    degrade to 'no detections', never to an exception mid-flight.
    """

    STALE_S = 5.0
    _HIST_MIN_GAP_S = 0.05
    _HIST_LEN = 400            # ~20s of snapshots at the 0.05s cap

    def __init__(self, path, conf=0.5, skip_radius_m=8.0, log=print,
                 geom=None, mount_yaw_deg=0.0, stale_s=None):
        import collections
        import json
        self._json = json
        self._init_resolver(conf, skip_radius_m, geom, mount_yaw_deg, log)
        self.path = os.path.expanduser(path)
        self.stale_s = self.STALE_S if stale_s is None else stale_s
        self._hist = collections.deque(maxlen=self._HIST_LEN)
        self._last_seq = None
        self._last_mtime = None
        self._stale_warned = False
        self._cam_warned = False

    def _read(self):
        """Latest worker payload if it is new since last read, else None.
        Never raises: a half-written or vanished file is skipped, not fatal
        (the worker writes via os.replace, so this is belt and braces)."""
        try:
            mtime = os.stat(self.path).st_mtime
        except OSError:
            return None, True
        stale = time.time() - mtime > self.stale_s
        if mtime == self._last_mtime:
            return None, stale
        self._last_mtime = mtime
        try:
            with open(self.path) as f:
                payload = self._json.load(f)
        except (OSError, ValueError):
            return None, stale
        if payload.get('seq') == self._last_seq:
            return None, stale
        self._last_seq = payload.get('seq')
        return payload, stale

    def preflight(self):
        """Wait for a live worker producing correctly-sized frames.

        Replaces the camera preflight: the camera now lives in the worker,
        so 'fresh file with camera_ok' is the proof that the whole capture
        and inference path works before arming.
        """
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            payload, stale = self._read()
            if payload is not None and not stale:
                if payload.get('camera_ok') is False:
                    self.log("[detector] PREFLIGHT FAIL: worker reports no "
                             "frame from the camera")
                    return False
                w, h = payload.get('w'), payload.get('h')
                self.log(f"[detector] preflight ok: worker seq "
                         f"{payload['seq']}, {w}x{h} frame")
                if self.geom is not None and (w, h) != (self.geom.width,
                                                        self.geom.height):
                    self.log(f"[detector] PREFLIGHT FAIL: camera negotiated "
                             f"{w}x{h} but the geometry was built for "
                             f"{self.geom.width}x{self.geom.height}. Fix "
                             f"--frame-w/--frame-h (and remeasure the FOV at "
                             f"that resolution) rather than flying with "
                             f"offsets scaled by the wrong factor.")
                    return False
                return True
            time.sleep(0.5)
        self.log(f"[detector] PREFLIGHT FAIL: no fresh worker output at "
                 f"{self.path} within 30s (is detect_worker.py running?)")
        return False

    def _snap_for(self, t_frame):
        """History snapshot nearest the frame's capture time."""
        if not self._hist:
            return None
        return min(self._hist, key=lambda s: abs(s.t_wall - t_frame))

    def poll(self, tel):
        # Record telemetry history on every tick (cheap), even ticks with no
        # new result: the snapshot a future result needs is being taken NOW.
        if tel.lat is not None and (
                not self._hist or
                time.monotonic() - self._hist[-1].t_mono >= self._HIST_MIN_GAP_S):
            self._hist.append(_TelSnap(tel))

        payload, stale = self._read()
        if stale:
            if not self._stale_warned:
                self.log(f"[detector] worker output stale >"
                         f"{self.stale_s:.0f}s: detections are OFF until it "
                         f"recovers (mission continues)")
                self._stale_warned = True
        elif self._stale_warned:
            self.log("[detector] worker output fresh again")
            self._stale_warned = False
        if payload is None:
            return None
        if payload.get('camera_ok') is False:
            if not self._cam_warned:
                self.log("[detector] worker reports camera frame grab FAILED")
                self._cam_warned = True
            return None
        self._cam_warned = False
        if tel.lat is None or not payload.get('rows'):
            return None
        snap = self._snap_for(payload['t_frame'])
        if snap is None or snap.lat is None:
            return None
        return self._resolve(snap, payload['rows'],
                             payload['w'], payload['h'], at_t=snap.t_mono)


def dist_m(lat1, lon1, lat2, lon2):
    """Equirectangular approximation; fine for survey-scale distances."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371000.0 * math.hypot(dlat, dlon)


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 6371000.0
    dlon = east_m / (6371000.0 * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)
