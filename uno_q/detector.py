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
    """Real detector: B525 still -> ONNX (yolo26n export) -> pixel box ->
    ground offset from survey alt + camera intrinsics. Needs camera bench
    (TODO 2) and the v2 model from the training laptop. Not implemented yet."""

    def __init__(self, *a, **kw):
        raise NotImplementedError("waits on camera bench + v2 ONNX export")


def dist_m(lat1, lon1, lat2, lon2):
    """Equirectangular approximation; fine for survey-scale distances."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371000.0 * math.hypot(dlat, dlon)


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 6371000.0
    dlon = east_m / (6371000.0 * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)
