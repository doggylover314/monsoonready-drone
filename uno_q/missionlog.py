"""Per-mission JSONL event log — the base station's only data source.

One file per mission under <data_dir>/missions/mission_<id>.jsonl, one JSON
object per line, append-only and flushed per event so a crash mid-mission
loses at most the current line. The base station (uno_q/basestation/) never
talks MAVLink; it just reads these files, which also makes every mission
replayable evidence for the docs.

Event schema (field "e" discriminates):
  mission_start  t, cfg{...}            mission config snapshot (dataclass dict)
  fix            t, lat, lon, alt, rng, state   ~1Hz position breadcrumb
  state          t, from, to, note      state machine transition
  detection      t, lat, lon, conf      raw detector fire (pre-offset)
  latch          t, lat, lon            latched target (after lateral offset)
  drop           t, lat, lon, rng      payload released
  abort          t, lat, lon, reason   descent aborted upward
  mission_end    t, final, drops

`now` is injectable so the fake-mission generator can write historic
timestamps through this same class (schema lives here and only here).
"""

import json
import os
import time
from dataclasses import asdict, is_dataclass


class MissionLog:
    def __init__(self, data_dir, mission_id=None, fix_period_s=1.0,
                 now=time.time):
        self.now = now
        self.fix_period_s = fix_period_s
        self._last_fix = 0.0
        missions_dir = os.path.join(os.path.expanduser(data_dir), 'missions')
        os.makedirs(missions_dir, exist_ok=True)
        if mission_id is None:
            mission_id = time.strftime('%Y%m%d_%H%M%S', time.localtime(now()))
        self.mission_id = mission_id
        self.path = os.path.join(missions_dir, f'mission_{mission_id}.jsonl')
        self._f = open(self.path, 'a')

    def _w(self, e, **kw):
        kw['e'] = e
        kw['t'] = round(self.now(), 2)
        self._f.write(json.dumps(kw) + '\n')
        self._f.flush()

    # ---- events (names match what mission.py calls) ----

    def mission_start(self, cfg):
        self._w('mission_start',
                cfg=asdict(cfg) if is_dataclass(cfg) else dict(cfg))

    def fix(self, tel, state):
        t = self.now()
        if t - self._last_fix < self.fix_period_s or tel.lat is None:
            return
        self._last_fix = t
        self._w('fix', lat=tel.lat, lon=tel.lon,
                alt=tel.rel_alt_m,
                rng=tel.rng_m if tel.rng_valid else None,
                state=state)

    def state(self, frm, to, note=''):
        self._w('state', **{'from': frm, 'to': to, 'note': note})

    def detection(self, lat, lon, conf):
        self._w('detection', lat=lat, lon=lon, conf=conf)

    def latch(self, lat, lon):
        self._w('latch', lat=lat, lon=lon)

    def drop(self, lat, lon, rng):
        self._w('drop', lat=lat, lon=lon, rng=rng)

    def abort(self, lat, lon, reason):
        self._w('abort', lat=lat, lon=lon, reason=reason)

    def mission_end(self, final, drops):
        self._w('mission_end', final=final, drops=drops)

    def close(self):
        self._f.close()
