"""Recurrence scoring over past missions (TODO 14): which sites hold water
again, and which are worth revisiting first.

This is the honest version of the "predictive model" idea. It does NOT
forecast rainfall or invent a physical model of drainage. It answers a
narrower question that the mission data can actually support:

    given every site this drone has found before, which ones keep coming
    back, and which are therefore worth flying over first next time?

That matters because of the claim boundary the project set for itself: the
detector finds standing-water CANDIDATES, and stagnation is only established
by the same site persisting across passes on different days. A site seen once
is a puddle. A site seen on four separate days is a breeding site. Recurrence
is the measurable part of that, so recurrence is what this scores.

Read-only over the same JSONL logs the base station serves; it never writes
to them and never touches the base station's code.

    .venv/bin/python uno_q/predict.py --data-dir ~/monsoonready_data
    .venv/bin/python uno_q/predict.py --json      # machine-readable

Scoring, deliberately simple enough to explain to a judge in one breath:

    score = flights_seen * recency_weight * treatment_gap

  flights_seen    distinct missions that saw the site. The core signal.
  recency_weight  halves every `half_life_days`, so a site that stopped
                  appearing fades instead of ranking forever on old history.
  treatment_gap   1.0 if it has never been successfully treated, 0.5 if it
                  has. A treated site still deserves rechecking, just not
                  ahead of one never treated.

Every term is a number you can defend. Nothing here is fitted, so there is
nothing to overfit and no training set to leak.
"""

import argparse
import json
import math
import os
import time

from detector import dist_m

CLUSTER_RADIUS_M = 5.0     # same radius the dashboard uses for clustering
HALF_LIFE_DAYS = 14.0

# Vocabulary shared with the dashboard, which draws a ring at RECURRING and
# up. The two used to both say "persistent" while meaning different things
# (dashboard: 2 flights; here: 3 flights over 7 days), so a judge could read
# "persistent" off the map and "recurring" off this table for the same site.
# This file holds the stricter definition and the dashboard follows it.
RECURRING_MIN_FLIGHTS = 2
PERSISTENT_MIN_FLIGHTS = 3
PERSISTENT_MIN_SPAN_DAYS = 7.0


class Site:
    """One geographic cluster of detections, merged across missions."""

    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        self._n = 1
        self.flights = set()
        self.detections = 0
        self.drops = 0          # gate cycles that actually actuated
        self.failed_drops = 0   # attempted but the gate never opened
        self.aborts = 0
        self.last_t = 0.0
        self.first_t = float('inf')

    def absorb(self, lat, lon, mission_id, t):
        # Running mean keeps the cluster centred as more sightings arrive,
        # rather than pinning it to whichever detection happened to be first.
        self._n += 1
        self.lat += (lat - self.lat) / self._n
        self.lon += (lon - self.lon) / self._n
        self.flights.add(mission_id)
        self.stamp(t)

    def stamp(self, t):
        """Fold one event time in. t=None (missing or null in the log) is
        ignored rather than treated as epoch zero, which used to turn one
        undated event into a 20000-day span."""
        if t is None:
            return
        self.last_t = max(self.last_t, t)
        self.first_t = min(self.first_t, t)

    @property
    def span_days(self):
        if self.first_t == float('inf') or self.last_t <= self.first_t:
            return 0.0
        return (self.last_t - self.first_t) / 86400.0

    def score(self, now, half_life_days=HALF_LIFE_DAYS):
        if half_life_days <= 0:
            raise ValueError(f"half_life_days must be > 0, got {half_life_days}")
        age_days = max(0.0, (now - self.last_t) / 86400.0)
        recency = math.pow(0.5, age_days / half_life_days)
        # Only a drop that actually actuated counts as treatment. A gate that
        # failed leaves the site untreated, so it must keep full priority.
        gap = 0.5 if self.drops > 0 else 1.0
        return len(self.flights) * recency * gap

    def verdict(self):
        """Plain-language label. Deliberately conservative wording: nothing
        here is allowed to assert that larvae are present."""
        n = len(self.flights)
        if n >= PERSISTENT_MIN_FLIGHTS and self.span_days >= PERSISTENT_MIN_SPAN_DAYS:
            return 'persistent (likely breeding site)'
        if n >= RECURRING_MIN_FLIGHTS:
            return 'recurring (confirm next pass)'
        return 'single sighting (unconfirmed)'


def _num(v):
    """True if v is a usable number. Explicitly rejects bool, which is an int
    in Python and would otherwise sail through as a coordinate."""
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def read_missions(data_dir):
    """Yield (mission_id, [event, ...]) for every log, oldest first.

    Tolerates a truncated final line, because a mission killed mid-write
    leaves one, and refusing to read the whole flight over it would be worse.
    """
    d = os.path.join(os.path.expanduser(data_dir), 'missions')
    if not os.path.isdir(d):
        return
    for fn in sorted(os.listdir(d)):
        if not (fn.startswith('mission_') and fn.endswith('.jsonl')):
            continue
        events = []
        with open(os.path.join(d, fn)) as f:
            for line in f:
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    continue          # truncated tail line
        yield fn[len('mission_'):-len('.jsonl')], events


def build_sites(data_dir, radius_m=CLUSTER_RADIUS_M):
    sites = []
    for mission_id, events in read_missions(data_dir):
        for ev in events:
            kind = ev.get('e')
            if kind not in ('detection', 'drop', 'abort'):
                continue
            lat, lon = ev.get('lat'), ev.get('lon')
            if not _num(lat) or not _num(lon):
                continue
            t = ev.get('t') if _num(ev.get('t')) else None
            hit = None
            for s in sites:
                if dist_m(lat, lon, s.lat, s.lon) <= radius_m:
                    hit = s
                    break
            if hit is None:
                hit = Site(lat, lon)
                hit.flights.add(mission_id)
                hit.stamp(t)
                sites.append(hit)
            else:
                hit.absorb(lat, lon, mission_id, t)
            if kind == 'detection':
                hit.detections += 1
            elif kind == 'drop':
                # ok is absent in logs written before 2026-08-01, when every
                # recorded drop was assumed to have worked; absent means true.
                if ev.get('ok', True):
                    hit.drops += 1
                else:
                    hit.failed_drops += 1
            else:
                hit.aborts += 1
    return sites


def rank(data_dir, now=None, radius_m=CLUSTER_RADIUS_M,
         half_life_days=HALF_LIFE_DAYS):
    now = time.time() if now is None else now
    sites = build_sites(data_dir, radius_m)
    return sorted(sites, key=lambda s: s.score(now, half_life_days),
                  reverse=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default='~/monsoonready_data')
    ap.add_argument('--radius-m', type=float, default=CLUSTER_RADIUS_M)
    ap.add_argument('--half-life-days', type=float, default=HALF_LIFE_DAYS)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args()
    if args.half_life_days <= 0:
        ap.error("--half-life-days must be > 0")
    if args.radius_m <= 0:
        ap.error("--radius-m must be > 0")

    now = time.time()
    sites = rank(args.data_dir, now, args.radius_m, args.half_life_days)

    if args.json:
        print(json.dumps([{
            'lat': round(s.lat, 7), 'lon': round(s.lon, 7),
            'score': round(s.score(now, args.half_life_days), 3),
            'flights': len(s.flights), 'detections': s.detections,
            'drops': s.drops, 'failed_drops': s.failed_drops,
            'aborts': s.aborts,
            'span_days': round(s.span_days, 1),
            'verdict': s.verdict(),
        } for s in sites], indent=2))
        return

    if not sites:
        print(f"No mission logs under {args.data_dir}. Fly something first, "
              f"or generate fake flights with basestation/gen_fake_mission.py")
        return

    flights = len({f for s in sites for f in s.flights})
    print(f"{len(sites)} site(s) across {flights} flight(s), "
          f"ranked by revisit priority\n")
    print(f"{'#':>2}  {'score':>6}  {'seen':>4}  {'drops':>5}  {'span':>6}  "
          f"position                  verdict")
    for i, s in enumerate(sites, 1):
        print(f"{i:>2}  {s.score(now, args.half_life_days):>6.2f}  "
              f"{len(s.flights):>4}  {s.drops:>5}  {s.span_days:>5.1f}d  "
              f"{s.lat:>10.6f},{s.lon:<11.6f}  {s.verdict()}")
    print("\nScore = flights_seen x recency (half-life "
          f"{args.half_life_days:g}d) x 0.5 if already treated.")


if __name__ == '__main__':
    main()
