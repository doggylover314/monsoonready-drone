"""Generate waypoint file for serpentine survey from aircraft's GPS position.

Usage: position aircraft at survey corner, point nose along row direction, then:
  python uno_q/make_waypoints.py --out wp_field.txt
  run_mission.py --waypoints wp_field.txt

Reads position and heading only. Defaults: 3 rows 5 m apart, 20 m x 10 m box.
B525 HFOV 56.2 deg gives footprint 16.0 m at 15 m alt. Place tray under middle row.
"""

import argparse
import math
import sys
import time

from mavlink_io import MavIO

EARTH_M_PER_DEG_LAT = 111320.0


def spacing_for_overlap(alt_m, overlap_m=1.0, width=1280, height=720,
                        hfov_deg=None):
    """(row_spacing_m, waypoint_spacing_m) for overlap_m between frames.

    1280 px axis is ACROSS track, 720 px ALONG (camera_to_ned: image up = forward).
    Minimum 1 m returned to prevent huge routes at low altitude.
    """
    from camera_geom import CameraGeometry
    geom = (CameraGeometry(width, height) if hfov_deg is None
            else CameraGeometry(width, height, hfov_deg))
    across, along = geom.footprint_m(alt_m)
    return max(1.0, across - overlap_m), max(1.0, along - overlap_m)


def densify(points, max_leg_m):
    """Split legs longer than max_leg_m into equal pieces.

    Builders emit only row ends. Photo hold at waypoint means spacing sets interval.
    max_leg_m <= 0 returns list unchanged.
    """
    if max_leg_m is None or max_leg_m <= 0 or len(points) < 2:
        return list(points)
    out = [points[0]]
    for (alat, alon), (blat, blon) in zip(points, points[1:]):
        dn = (blat - alat) * EARTH_M_PER_DEG_LAT
        de = ((blon - alon) * EARTH_M_PER_DEG_LAT
              * math.cos(math.radians(alat)))
        n = int(math.ceil(math.hypot(dn, de) / max_leg_m))
        for i in range(1, n + 1):
            t = i / n
            out.append((alat + (blat - alat) * t, alon + (blon - alon) * t))
    return out


def build_serpentine(lat0, lon0, heading_deg, rows, spacing_m, length_m):
    """Waypoints (lat, lon) for serpentine starting at (lat0, lon0),
    rows along heading_deg, stepping right."""
    h = math.radians(heading_deg)
    fwd = (math.cos(h), math.sin(h))                    # north, east vectors
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
    """Serpentine waypoints filling interior of polygon; deterministic per fence.

    Parameters: polygon [(lat, lon)...] in order; heading_deg (default: longest edge);
    inset_m (distance inside fence); start (operator's click; default: southwest-most);
    step_m (sampling interval); min_row_m (minimum row length).

    Inset enforced via scan lines sampled every step_m, kept if inside polygon AND
    >= inset_m from all edges. Accurate to step_m. Handles concave fences: all pieces
    flown with leg checks preventing transitions across notches.

    Returns (waypoints, info dict). waypoints empty if inset too tight; info['problem']
    describes issue, info['dropped'] counts skipped pieces on concave fences.
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
    # (north, east) to (along rows, across). fwd=(cos h, sin h), right=(-sin h, cos h).
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
        """Is straight flight from p to q inside keep-out zone?

        Memoised: traversal tries all starting ends, same legs repeat.
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

    lines = []                   # segments per scan line
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

    # Greedy nearest-reachable ordering (not line-by-line sweep) handles concave
    # fences: sweep would cut corners or drop reachable pieces. Greedy makes start
    # point meaningful: first row is nearest end to operator's click.
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
        # No start point: try all row ends, keep best route (fewest drops, shortest).
        # Southwest-most default is arbitrary on concave fences.
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
    # Default wp_field.txt is used by start_dashboard.sh START button.
    ap.add_argument('--out', default='wp_field.txt')
    ap.add_argument('--conn', default='auto',
                    help="'auto' (default) = the Pixhawk's USB via "
                         "/dev/serial/by-id, or SITL tcp:...")
    ap.add_argument('--rows', type=int, default=3)
    ap.add_argument('--spacing', type=float, default=12.0,
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
        # GLOBAL_POSITION_INT = 0.0 before fix; real fix has nonzero lat or lon.
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
