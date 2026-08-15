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


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='wp_farm.txt')
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
