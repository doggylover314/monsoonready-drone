"""Generate a serpentine survey waypoint file from where the aircraft IS.

The farm plot's coordinates are unknowable until the aircraft stands on it,
so waypoints are made on site, from its own GPS, in one command:

  1. Put the aircraft at the plot corner where the survey should START.
  2. Point its NOSE along the direction the rows should run.
  3. With a 3D fix (link is the Pixhawk's USB, resolved automatically):
         ~/venv/bin/python uno_q/make_waypoints.py --out wp_field.txt
  4. Fly:  run_mission.py --waypoints wp_field.txt

Defaults make a 20 m x 10 m box (3 rows, 5 m apart): small enough to watch
and film, wide enough that the camera footprint (16.0 m across at 15 m
with the B525's measured 56.2 deg HFOV) overlaps rows heavily. The tray
goes under the MIDDLE row.

Read-only toward the aircraft: it listens for position and heading, writes
a text file, and commands nothing.
"""

import argparse
import math
import sys
import time

from mavlink_io import MavIO

EARTH_M_PER_DEG_LAT = 111320.0


def build_serpentine(lat0, lon0, heading_deg, rows, spacing_m, length_m):
    """Waypoints (lat, lon) for a serpentine starting AT (lat0, lon0),
    rows running along heading_deg, stepping right of that heading."""
    h = math.radians(heading_deg)
    fwd = (math.cos(h), math.sin(h))                    # (north, east)
    right = (math.cos(h + math.pi / 2), math.sin(h + math.pi / 2))

    def offset(north_m, east_m):
        dlat = north_m / EARTH_M_PER_DEG_LAT
        dlon = east_m / (EARTH_M_PER_DEG_LAT * math.cos(math.radians(lat0)))
        return (lat0 + dlat, lon0 + dlon)

    wps = []
    for i in range(rows):
        bn = right[0] * spacing_m * i
        be = right[1] * spacing_m * i
        near = offset(bn, be)
        far = offset(bn + fwd[0] * length_m, be + fwd[1] * length_m)
        wps.extend([near, far] if i % 2 == 0 else [far, near])
    return wps


def _seg_dist(px, py, ax, ay, bx, by):
    """Distance from a point to a segment, in the same units as the inputs."""
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _inside(px, py, poly):
    """Ray casting. poly is [(x, y), ...] in metres, implicitly closed."""
    hit = False
    for i in range(len(poly)):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % len(poly)]
        if (ay > py) != (by > py):
            x = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < x:
                hit = not hit
    return hit


def build_coverage(polygon, heading_deg=None, spacing_m=5.0, inset_m=4.0,
                   start=None, step_m=0.5, min_row_m=1.0):
    """Serpentine rows that COVER the inside of a geofence polygon.

    build_serpentine above lays a fixed rows x length box from wherever the
    aircraft happens to stand, which is a different job: it can hang the box
    half outside the fence, and it cannot be planned at home. This takes the
    fence the operator has already drawn and fills it, so the route is a
    property of the FIELD rather than of where the aircraft was parked, and
    needs no GPS fix to build.

    polygon    [(lat, lon), ...], the fence corners, in order
    heading_deg  direction the rows run; default = along the longest fence
                 edge, which is the fewest turns for a field-shaped polygon
    inset_m    every waypoint stays at least this far INSIDE the fence
    start      (lat, lon) the operator picked; the route is arranged so its
               first waypoint is the row end nearest that point. Without it,
               the start is the southwest-most row end (deterministic, so the
               same fence always generates the same route).

    HOW THE INSET IS ENFORCED: each scan line is sampled every step_m, a
    sample is kept only if it is inside the polygon AND at least inset_m from
    every edge, and the kept samples become runs. That is exact for any
    polygon shape to within step_m, with no polygon-offsetting maths to get
    wrong.

    CONCAVE FENCES (user's is one: it has a corner that turns back on itself,
    so a scan line through the pinch has TWO separate pieces). Every piece is
    flown, not just the longest, because the dropped ones are real ground the
    camera never sees. What makes that safe is the leg check: the straight
    line the aircraft flies from one piece to the next must itself stay
    inset_m inside the fence, sampled the same way. A piece whose entry leg
    would cut across the notch is DROPPED and counted in info['dropped'],
    never flown and never silently omitted. On a convex fence the check
    costs nothing: the inset region of a convex polygon is convex, so a leg
    between two points inside it is inside it.

    Returns (waypoints, info). waypoints is [(lat, lon), ...], empty if the
    inset leaves no room, in which case info['problem'] says so.
    """
    if len(polygon) < 3:
        return [], {'problem': 'a fence needs at least 3 corners'}
    lat0, lon0 = polygon[0]
    mlon = EARTH_M_PER_DEG_LAT * math.cos(math.radians(lat0))
    if abs(mlon) < 1.0:
        return [], {'problem': 'polygon is at a pole; not supported'}

    def to_m(lat, lon):
        return ((lat - lat0) * EARTH_M_PER_DEG_LAT, (lon - lon0) * mlon)

    def to_ll(n, e):
        return (lat0 + n / EARTH_M_PER_DEG_LAT, lon0 + e / mlon)

    poly = [to_m(a, b) for a, b in polygon]

    if heading_deg is None:
        best, heading_deg = -1.0, 0.0
        for i in range(len(poly)):
            (an, ae), (bn, be) = poly[i], poly[(i + 1) % len(poly)]
            d = math.hypot(bn - an, be - ae)
            if d > best:
                best, heading_deg = d, math.degrees(math.atan2(be - ae, bn - an))
    h = math.radians(heading_deg)
    ca, sa = math.cos(h), math.sin(h)
    # (north, east) -> (along the rows, across them). fwd = (cos h, sin h) and
    # right = (-sin h, cos h), matching build_serpentine's frame exactly.
    fwd = lambda n, e: n * ca + e * sa                            # noqa: E731
    rgt = lambda n, e: -n * sa + e * ca                           # noqa: E731
    back = lambda a, c: (a * ca - c * sa, a * sa + c * ca)        # noqa: E731

    alo = min(fwd(*p) for p in poly)
    ahi = max(fwd(*p) for p in poly)
    clo = min(rgt(*p) for p in poly) + inset_m
    chi = max(rgt(*p) for p in poly) - inset_m
    if chi < clo or ahi - alo <= 2 * inset_m:
        return [], {'problem': f'a {inset_m:g} m keep-out leaves nothing '
                               f'inside this fence: shrink it or draw a '
                               f'bigger fence'}

    n_lines = int((chi - clo) / spacing_m) + 1
    pad = ((chi - clo) - (n_lines - 1) * spacing_m) / 2.0          # centre them

    def clear(n, e):
        if not _inside(n, e, poly):
            return False
        for i in range(len(poly)):
            (ax, ay), (bx, by) = poly[i], poly[(i + 1) % len(poly)]
            if _seg_dist(n, e, ax, ay, bx, by) < inset_m:
                return False
        return True

    _legs = {}

    def leg_ok(p, q):
        """Is the straight flight from p to q inside the keep-out everywhere?

        Memoised because the traversal below is tried from every possible
        starting end, and the same legs come up again and again.
        """
        v = _legs.get((p, q))
        if v is not None:
            return v
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        steps = max(2, int(d / step_m) + 1)
        v = True
        for i in range(steps + 1):
            t = i / steps
            if not clear(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t):
                v = False
                break
        _legs[(p, q)] = v
        return v

    lines = []                   # [[(near_pt, far_pt), ...] per scan line]
    for k in range(n_lines):
        c = clo + pad + k * spacing_m
        segs, run = [], None
        a = alo
        while a <= ahi + step_m / 2:
            if clear(*back(a, c)):
                run = (a, a) if run is None else (run[0], a)
            else:
                if run and run[1] - run[0] >= min_row_m:
                    segs.append(run)
                run = None
            a += step_m
        if run and run[1] - run[0] >= min_row_m:
            segs.append(run)
        if segs:
            lines.append([(back(s[0], c), back(s[1], c)) for s in segs])

    if not lines:
        return [], {'problem': f'no row survived a {inset_m:g} m keep-out at '
                               f'{spacing_m:g} m spacing'}

    # NEAREST-REACHABLE, not a fixed line-by-line sweep. On a rectangle the
    # two are identical: after flying a row, the nearest unflown end IS the
    # next row's near end, which is the serpentine. They differ exactly where
    # the fence is concave, and there the sweep is wrong: it would either fly
    # a leg through the notch or drop pieces it could have reached by going
    # round. Greedy also makes the start point mean what it says, since the
    # first row is simply the end nearest the click.
    segs = [s for line in lines for s in line]

    def greedy(from_pt):
        used = [False] * len(segs)
        out, cur, first = [], from_pt, True
        while True:
            best = None
            for i, (p0, p1) in enumerate(segs):
                if used[i]:
                    continue
                for a, b in ((p0, p1), (p1, p0)):
                    d = math.hypot(a[0] - cur[0], a[1] - cur[1])
                    if best is not None and d >= best[0]:
                        continue
                    if not first and not leg_ok(cur, a):
                        continue      # would cut a corner outside the fence
                    best = (d, i, a, b)
            if best is None:
                break
            _, i, a, b = best
            used[i] = True
            out.extend([a, b])
            cur, first = b, False
        return (out, used.count(False),
                sum(math.hypot(q[0] - p[0], q[1] - p[1])
                    for p, q in zip(out, out[1:])))

    if start is not None:
        pts, dropped, length = greedy(to_m(start[0], start[1]))
    else:
        # No click, so no operator intent to honour: try starting from EVERY
        # row end and keep the best route. The old rule (southwest-most end)
        # is arbitrary once the fence is concave - on the user's own fence it
        # began in the middle and paid a 64 m dead leg to come back for the
        # pieces below the notch.
        pts, dropped, length = min((greedy(p) for s in segs for p in s),
                                   key=lambda r: (r[1], r[2]))
    if not pts:
        return [], {'problem': f'no row survived a {inset_m:g} m keep-out at '
                               f'{spacing_m:g} m spacing'}

    return ([to_ll(n, e) for n, e in pts],
            {'heading': round(heading_deg % 360, 1), 'rows': len(pts) // 2,
             'spacing': spacing_m, 'inset': inset_m, 'dropped': dropped,
             'lines': len(lines), 'path_m': round(length), 'problem': None})


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # wp_field.txt since 2026-08-16: filming moved to the nearby field, and
    # start_dashboard.sh's START button flies ~/wp_field.txt.
    ap.add_argument('--out', default='wp_field.txt')
    ap.add_argument('--conn', default='auto',
                    help="'auto' (default) = the Pixhawk's USB via "
                         "/dev/serial/by-id, or SITL tcp:...")
    ap.add_argument('--rows', type=int, default=3)
    ap.add_argument('--spacing', type=float, default=5.0,
                    help='m between rows (camera covers ~16.7 m at 15 m alt)')
    ap.add_argument('--length', type=float, default=20.0, help='row length m')
    ap.add_argument('--heading', type=float, default=None,
                    help='row direction, deg (default: where the nose points)')
    args = ap.parse_args()
    if args.rows < 1 or args.spacing <= 0 or args.length <= 0:
        sys.exit("rows >= 1, spacing > 0, length > 0")

    print(f"connecting {args.conn} ...")
    io = MavIO(args.conn)
    try:
        io.wait_ready(timeout=20)
    except (TimeoutError, RuntimeError) as exc:
        sys.exit(f"{exc}\n(if nothing arrived at all: check the Pixhawk's "
                 f"USB plug on the hub: ls -l /dev/serial/by-id/)")
    io.setup_streams()

    print("waiting for a real position fix ...")
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        io.step()
        t = io.tel
        # 0.0 is what GLOBAL_POSITION_INT carries before a fix; a real
        # fix anywhere on Earth is nonzero in at least one axis.
        if (t.lat is not None and (abs(t.lat) > 0.01 or abs(t.lon) > 0.01)
                and t.heading_deg is not None):
            break
    else:
        sys.exit("no GPS fix in 120s: sky view? bench.py gps to watch sats")

    t = io.tel
    heading = args.heading if args.heading is not None else t.heading_deg
    wps = build_serpentine(t.lat, t.lon, heading, args.rows,
                           args.spacing, args.length)

    with open(args.out, 'w') as f:
        f.write(f"# serpentine from ({t.lat:.7f},{t.lon:.7f}) "
                f"heading {heading:.0f}deg, {args.rows} rows x "
                f"{args.length:.0f}m, {args.spacing:.0f}m apart\n")
        for lat, lon in wps:
            f.write(f"{lat:.7f},{lon:.7f}\n")

    far_m = max(
        math.hypot((lat - t.lat) * EARTH_M_PER_DEG_LAT,
                   (lon - t.lon) * EARTH_M_PER_DEG_LAT
                   * math.cos(math.radians(t.lat)))
        for lat, lon in wps)
    print(f"{args.out}: {len(wps)} waypoints, rows along {heading:.0f} deg, "
          f"farthest point {far_m:.0f} m from here")
    print("walk the tray to under the MIDDLE row before flying")
    return 0


if __name__ == '__main__':
    sys.exit(main())
