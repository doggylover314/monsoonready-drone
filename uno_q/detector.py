"""Detection sources for the mission state machine.

Mission interface: swapping FakeDetector (SITL) for OnnxDetector must not require mission.py changes.
"""

import math
import os
import time

from camera_geom import camera_to_ned, ground_area_m2, letterbox_to_frame

# A rangefinder reading older than this is not trusted as an AGL source.
RNG_FRESH_S = 1.0


def quiet_import_onnxruntime():
    """Import onnxruntime, suppressing expected GPU-probe warnings from ort 1.28.

    ort probes /sys/class/drm at import time; on this GPU-less board the warning must be
    suppressed via fd redirection (set_default_logger_severity does not exist until after import).
    Real import failures still surface.
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
    def __init__(self, lat, lon, confidence=1.0, area_m2=None):
        self.lat = lat
        self.lon = lon
        self.confidence = confidence
        # Ground area of detection box in m2, or None if geometry/height missing.
        # Mission must treat None as "unknown", not "small".
        self.area_m2 = area_m2


class DetectionSource:
    def poll(self, tel):
        """Return Detection or None; called every tick in SURVEY."""
        raise NotImplementedError

    def blind_for_s(self):
        """Seconds detector output has been stale; 0.0 means healthy.

        None from poll() is ambiguous (no puddle vs. no eyes).
        Sources that cannot go blind (FakeDetector) return 0.0.
        """
        return 0.0


class _RowResolver(DetectionSource):
    """Converts model output rows (LETTERBOX space) to Detection via geom and telemetry.

    Rows format: x1,y1,x2,y2,conf[,cls]. at_t is frame capture time (for rangefinder freshness vs. read time).
    """

    def _init_resolver(self, conf, skip_radius_m, geom, mount_yaw_deg, log):
        # conf <= 0 would treat zero-confidence padding rows (model output is always 300 rows, mostly empty)
        # as detections, flying to whatever pixel sorted first. Must reject it.
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
        """Return first un-treated site above threshold, or None."""
        # Geometry is frame-resolution-specific. If MJPG resolution changed mid-flight,
        # offsets would be scaled wrong silently. Fall back to nadir: bounded error > invisible systematic error.
        if self.geom is not None and (w, h) != (self.geom.width,
                                                self.geom.height):
            if not self._size_warned:
                self.log(f"[detector] frame is {w}x{h}, geometry expects "
                         f"{self.geom.width}x{self.geom.height}: falling back "
                         f"to nadir for the rest of the flight")
                self._size_warned = True
            self.geom = None
        # All rows above threshold, best first. A frame holds both treated and untreated puddles;
        # taking only strongest would suppress neighbours on every frame until they leave footprint.
        rows = sorted((r for r in rows if float(r[4]) >= self.conf),
                      key=lambda r: -float(r[4]))
        for row in rows:
            conf = float(row[4])
            lat, lon, how, area = self._locate(tel, row, w, h, at_t)
            # Dedup on puddle site, not drone position: same puddle seen from two positions is one target.
            if any(dist_m(lat, lon, flat, flon) <= self.skip_radius_m
                   for flat, flon in self._fired):
                continue                  # already treated this site
            self._fired.append((lat, lon))
            self.log(f"[detector] puddle conf {conf:.2f} {how}"
                     + (f" ~{area:.1f} m2" if area is not None else
                        " (area unknown)")
                     + (f" ({len(rows)} candidates this frame)"
                        if len(rows) > 1 else ""))
            return Detection(lat, lon, conf, area)
        return None

    @staticmethod
    def _height_agl(tel, at_t=None):
        """Height above ground (AGL) and its source.

        Prefer fresh rangefinder (true AGL), fall back to EKF rel_alt (for altitude above home).
        TF-Luna blind above ~8 m.
        """
        ref = at_t if at_t is not None else time.monotonic()
        if (tel.rng_valid and tel.rng_m is not None
                and ref - tel.rng_t < RNG_FRESH_S):
            return tel.rng_m, 'rng'
        if tel.rel_alt_m is not None and tel.rel_alt_m > 0:
            return tel.rel_alt_m, 'ekf'
        return None, 'none'

    def _locate(self, tel, row, frame_w, frame_h, at_t=None):
        """Map detection box centre to (lat, lon, description).

        Falls back to nadir when geometry missing or telemetry incomplete, never throws mid-mission.
        """
        height_m, source = self._height_agl(tel, at_t)
        if (self.geom is None or height_m is None
                or tel.heading_deg is None):
            return tel.lat, tel.lon, 'at nadir', None
        cx = (float(row[0]) + float(row[2])) / 2
        cy = (float(row[1]) + float(row[3])) / 2
        px, py = letterbox_to_frame(cx, cy, frame_w, frame_h, self.SIZE)
        right_m, down_m = self.geom.ground_offset(px, py, height_m)
        n, e = camera_to_ned(right_m, down_m, tel.heading_deg,
                             self.mount_yaw_deg)
        lat, lon = offset_latlon(tel.lat, tel.lon, n, e)
        ax, ay = letterbox_to_frame(float(row[0]), float(row[1]),
                                    frame_w, frame_h, self.SIZE)
        bx, by = letterbox_to_frame(float(row[2]), float(row[3]),
                                    frame_w, frame_h, self.SIZE)
        area = ground_area_m2(self.geom, ax, ay, bx, by, height_m)
        return (lat, lon,
                f"offset {n:+.1f}m N {e:+.1f}m E @{height_m:.1f}m {source}",
                area)


class FakeDetector(DetectionSource):
    """SITL stand-in: detects planted puddle when drone enters radius_m (proxy for nadir footprint)."""

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

    Model output: (1,300,6) rows of x1,y1,x2,y2,conf,cls with NMS baked in; threshold only.
    Preprocessing: letterbox 640, RGB, /255 (byte-identical to spotcheck_onnx.py).
    Position: camera_geom (box centre -> ground offset -> NED -> lat/lon) or nadir fallback.
    Never raises mid-mission. Single-threaded: poll() throttled by interval_s; deduped within skip_radius_m.
    Blocking poll() is obsoleted by FileDetector + detect_worker.py in production flights.
    frame_source: optional BGR image source (defaults to B525 via V4L2 MJPG 1280x720).
    """

    def __init__(self, model_path, camera='auto', conf=0.5, interval_s=1.0,
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
            # Open by name, never bare index: /dev/video enumeration races with Venus codecs.
            # camera.open_camera resolves real device, locks focus, diagnoses failures clearly.
            import cv2
            from camera import open_camera
            self._cv2 = cv2
            self.cap, self.cam_node = open_camera(camera, log=log)
            self._grab = self._grab_camera

    def _grab_camera(self):
        self.cap.grab()                    # flush possibly-stale buffer
        ok, frame = self.cap.read()
        return frame if ok else None

    def preflight(self):
        """Bench check before arming. Returns True if camera works.

        Detects dead camera before flight (not after survey with zero detections).
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
        """Grab frame and run model; returns (t_frame, w, h, rows, frame) or None.

        This is the worker-process piece. rows = model output above conf in letterbox space.
        t_frame is capture time (not inference time) to pair with frame's original telemetry.
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
        # Sharpness (Laplacian variance) logged for analysis; motion blur is invisible to model.
        # Not a gate (no frame skipped), no gating without in-flight distribution.
        # Absolute values meaningless across scenes; compare within single flight only.
        self.last_sharpness = float(self._cv2.Laplacian(
            self._cv2.cvtColor(boxed, self._cv2.COLOR_BGR2GRAY),
            self._cv2.CV_64F).var())
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
    """Frozen telemetry snapshot at capture time, so detection computed later uses correct fix."""

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
    """Reads detect_worker.py results from file; mission loop never blocks on inference.

    Inference in separate process: loop pumps MAVLink freely (in-process blocks for model latency).
    Pairing rule: result is one frame old; poll() keeps telemetry history and uses snapshot nearest frame capture time.
    Liveness: file mtime is worker heartbeat; stale or camera_ok=false degrades to no detections, never exception.
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
        self._blind_since = None

    def _read(self):
        """Return latest payload if new, else None. Never raises; half-written or vanished file skipped."""
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
        """Wait for live worker with correct frame size.

        Waits up to 30s for fresh output file with camera_ok=true before arming.
        """
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            payload, stale = self._read()
            if payload is not None and not stale:
                if payload.get('camera_ok') is False:
                    self.log("[detector] PREFLIGHT FAIL: worker reports no "
                             "frame from the camera"
                             + (f" -- {payload['error']}"
                                if payload.get('error') else
                                " (no detail; see ~/logs/detect_worker.log)"))
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

    def blind_for_s(self):
        """Seconds since output file went stale (dead worker, dead camera, or full disk all stop writes)."""
        if self._blind_since is None:
            return 0.0
        return time.monotonic() - self._blind_since

    def _snap_for(self, t_frame):
        """Return telemetry snapshot closest to frame capture time."""
        if not self._hist:
            return None
        return min(self._hist, key=lambda s: abs(s.t_wall - t_frame))

    def poll(self, tel):
        # Record telemetry every tick (cheap); history needed by future results being taken now.
        if tel.lat is not None and (
                not self._hist or
                time.monotonic() - self._hist[-1].t_mono >= self._HIST_MIN_GAP_S):
            self._hist.append(_TelSnap(tel))

        payload, stale = self._read()
        # Track blindness onset (mission decides duration threshold).
        if stale and self._blind_since is None:
            self._blind_since = time.monotonic()
        elif not stale:
            self._blind_since = None
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
                self.log("[detector] worker reports camera frame grab FAILED"
                         + (f" -- {payload['error']}"
                            if payload.get('error') else ""))
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
    """Distance in metres; equirectangular approximation for survey-scale distances."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371000.0 * math.hypot(dlat, dlon)


def offset_latlon(lat, lon, north_m, east_m):
    """Add NED offset to lat/lon."""
    dlat = north_m / 6371000.0
    dlon = east_m / (6371000.0 * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)
