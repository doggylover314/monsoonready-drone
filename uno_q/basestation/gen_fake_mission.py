"""Generate fake mission JSONL logs for dashboard development.

Writes through uno_q/missionlog.MissionLog with an injected clock, so the
schema stays defined in exactly one place. Default = two curated flights:

  flight A (yesterday): full 25 m survey square, 3 detections ->
      2 treated (drops), 1 rangefinder-dropout abort
  flight B (today): shorter pass, re-detects one flight-A site ~3 m away
      (becomes a "persistent site" on the accumulated view) + 1 new site

--flights N switches to RANDOM mode: N serpentine surveys, one per day,
drawing detection sites from a shared pool so some recur across flights
(persistence rings) and some are new; ~1 in 4 descents aborts. --seed makes
a run reproducible.

Usage (laptop or board; stdlib only):
    python gen_fake_mission.py [--data-dir ~/monsoonready_data]
    python gen_fake_mission.py --flights 12 --seed 7   # big random dataset

Then serve it:
    python dashboard.py --data-dir ~/monsoonready_data
"""

import argparse
import math
import os
import random
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from missionlog import MissionLog  # noqa: E402

HOME = (12.9716000, 77.5946000)   # arbitrary demo site
SURVEY_ALT, DROP_ALT = 15.0, 3.0


def off(n_m, e_m, base=HOME):
    lat = base[0] + math.degrees(n_m / 6371000.0)
    lon = base[1] + math.degrees(e_m / (6371000.0 * math.cos(math.radians(base[0]))))
    return lat, lon


class Clock:
    def __init__(self, t):
        self.t = float(t)

    def __call__(self):
        return self.t

    def tick(self, dt=1.0):
        self.t += dt


def tel(lat, lon, alt, rng=None):
    return SimpleNamespace(lat=lat, lon=lon, rel_alt_m=alt,
                           rng_m=rng, rng_valid=rng is not None)


def walk(log, clock, frm, to, alt, state, speed=2.0):
    """1 Hz fixes along a straight leg (positions in metres N/E of home)."""
    d = math.hypot(to[0] - frm[0], to[1] - frm[1])
    steps = max(1, int(d / speed))
    for i in range(1, steps + 1):
        n = frm[0] + (to[0] - frm[0]) * i / steps
        e = frm[1] + (to[1] - frm[1]) * i / steps
        log.fix(tel(*off(n, e), alt), state)
        clock.tick()
    return to


def vertical(log, clock, at, alt_from, alt_to, rate, state, rng_ok=True):
    alt = alt_from
    step = rate if alt_to > alt_from else -rate
    while abs(alt - alt_to) > rate:
        alt += step
        rng = alt if (rng_ok and alt <= 8.0) else None
        log.fix(tel(*off(*at), alt, rng), state)
        clock.tick()


def treat(log, clock, pos, puddle, conf):
    """Detection -> approach -> descend -> drop -> climb, back at survey alt."""
    lat, lon = off(*puddle)
    log.detection(lat, lon, conf)
    log.latch(lat, lon)
    log.state('SURVEY', 'APPROACH', f'latched {lat:.7f},{lon:.7f}')
    walk(log, clock, pos, puddle, SURVEY_ALT, 'APPROACH')
    log.state('APPROACH', 'DESCEND', '')
    vertical(log, clock, puddle, SURVEY_ALT, DROP_ALT, 0.5, 'DESCEND')
    log.drop(lat, lon, 2.96, dwell_s=1.2, area_m2=3.0)
    log.state('DESCEND', 'DROP', 'rng=2.96m')
    clock.tick(2)
    log.state('DROP', 'CLIMB', 'treated')
    vertical(log, clock, puddle, DROP_ALT, SURVEY_ALT, 1.0, 'CLIMB')
    return puddle


def abort_at(log, clock, pos, puddle, conf):
    """Detection whose descent loses the rangefinder (specular water)."""
    lat, lon = off(*puddle)
    log.detection(lat, lon, conf)
    log.latch(lat, lon)
    log.state('SURVEY', 'APPROACH', f'latched {lat:.7f},{lon:.7f}')
    walk(log, clock, pos, puddle, SURVEY_ALT, 'APPROACH')
    log.state('APPROACH', 'DESCEND', '')
    vertical(log, clock, puddle, SURVEY_ALT, 5.5, 0.5, 'DESCEND')
    log.abort(lat, lon, 'rangefinder dropout during descent')
    log.state('DESCEND', 'ABORT_CLIMB', 'rangefinder dropout during descent')
    vertical(log, clock, puddle, 5.5, SURVEY_ALT, 1.0, 'ABORT_CLIMB',
             rng_ok=False)
    return puddle


def flight(data_dir, t0, plan):
    clock = Clock(t0)
    log = MissionLog(data_dir, fix_period_s=0.5, now=clock)
    log.mission_start({'survey_alt_m': SURVEY_ALT, 'drop_alt_m': DROP_ALT,
                       'fake': True})
    log.state('IDLE', 'TAKEOFF', '')
    vertical(log, clock, (0, 0), 0, SURVEY_ALT, 2.0, 'TAKEOFF', rng_ok=False)
    pos, drops, prev = (0.0, 0.0), 0, 'TAKEOFF'
    for kind, wp in plan:
        if kind == 'wp':
            log.state(prev, 'SURVEY', '')
            pos = walk(log, clock, pos, wp, SURVEY_ALT, 'SURVEY')
            prev = 'SURVEY'
        elif kind == 'treat':
            pos = treat(log, clock, pos, wp[0], wp[1]); drops += 1
            prev = 'CLIMB'
        elif kind == 'abort':
            pos = abort_at(log, clock, pos, wp[0], wp[1])
            prev = 'ABORT_CLIMB'
    log.state('SURVEY', 'DONE', 'survey complete')
    log.mission_end('DONE', drops)
    log.close()
    return log.path


def random_plan(rng, pool):
    """One serpentine survey with events drawn from / added to the shared
    site pool (positions in metres N/E of home)."""
    rows = rng.randrange(3, 7)
    spacing = rng.uniform(8, 14)
    width = rng.uniform(30, 70)
    area_n, area_e = rows * spacing, width

    events = []
    for site in rng.sample(pool, min(len(pool), rng.randrange(0, 3))):
        jit = (site[0] + rng.uniform(-3, 3), site[1] + rng.uniform(-3, 3))
        events.append(jit)                       # revisit: persistent ring
    for _ in range(rng.randrange(1, 4)):
        site = (rng.uniform(2, area_n), rng.uniform(2, area_e))
        pool.append(site)
        events.append(site)
    rng.shuffle(events)

    plan = []
    for i in range(rows):
        n = i * spacing
        e = width if i % 2 == 0 else 0.0
        plan.append(('wp', (n, e)))
    plan.append(('wp', (0.0, 0.0)))
    for site in events:                          # detour events between legs
        kind = 'treat' if rng.random() < 0.75 else 'abort'
        conf = round(rng.uniform(0.55, 0.95) if kind == 'treat'
                     else rng.uniform(0.4, 0.8), 2)
        plan.insert(rng.randrange(1, len(plan)), (kind, (site, conf)))
    return plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--flights', type=int, default=None,
                    help='random mode: generate N random flights, one per day')
    ap.add_argument('--seed', type=int, default=None,
                    help='random-mode reproducibility')
    args = ap.parse_args()

    day = 86400
    now = time.time()

    if args.flights:
        rng = random.Random(args.seed)
        pool = []
        paths = []
        for i in range(args.flights):
            t0 = now - (args.flights - i) * day + rng.uniform(-4, 4) * 3600
            paths.append(flight(args.data_dir, t0, random_plan(rng, pool)))
        print('wrote:', *paths, sep='\n  ')
        return
    a = flight(args.data_dir, now - day - 3 * 3600, [
        ('wp', (25, 0)),
        ('treat', ((25, 12), 0.87)),
        ('wp', (25, 25)),
        ('abort', ((8, 25), 0.62)),
        ('wp', (0, 25)),
        ('treat', ((0, 10), 0.78)),
        ('wp', (0, 0)),
    ])
    b = flight(args.data_dir, now - 2 * 3600, [
        ('wp', (25, 0)),
        ('treat', ((24.6, 12.4), 0.91)),   # ~3 m from flight A site: persistent
        ('wp', (25, 25)),
        ('treat', ((20, 22), 0.71)),       # new site
        ('wp', (0, 25)),
        ('wp', (0, 0)),
    ])
    print('wrote:', a, b, sep='\n  ')


if __name__ == '__main__':
    main()
