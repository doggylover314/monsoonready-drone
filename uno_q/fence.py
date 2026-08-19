"""Geofence polygon: store it, push it to the Pixhawk, read it back.

WHY THIS FILE EXISTS (user, 2026-08-16): "I will set the geofence manually so
that the drone doesnt go into any trees. The field is not a cylinder or a
rectangle, it is a weird shape I have to draw myself." A polygon INCLUSION
fence is the only fence shape that fits such a field, and a polygon is not a
parameter: it is uploaded as mission items and lives in the flight
controller's own fence storage, surviving reboots. Parameter work stays where
it has always lived (tools/parameters.py on the laptop); this file never
writes a parameter.

Verified against the aircraft's own firmware (ArduCopter 4.7.0 source at
/media/sleuther/Stuff/ardupilot-SITL, tag Copter-4.7.0):
  * upload is the MISSION-ITEM protocol with mission_type =
    MAV_MISSION_TYPE_FENCE (1)  [MissionItemProtocol_Fence.cpp:15-77]
  * each corner is MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001) with
    param1 = total vertex count and x/y = lat/lon * 1e7 as int32
    [MissionItemProtocol_Fence.cpp:100-143]
  * the legacy FENCE_POINT protocol is COMPILED OUT in 4.7
    (AC_Fence_config.h:11-20), so an old-style upload silently does nothing
  * minimum 3 vertices; the polygon must NOT repeat its first corner
    (AP_Math/polygon.cpp handles open lists; the loader stores what it is
    given)  [AC_PolyFence_loader.cpp:998-1001]
  * with the polygon bit set in FENCE_TYPE and no valid polygon loaded,
    prearm REFUSES with "Polygon fence(s) invalid"
    [AC_Fence.cpp:428-440] -- which is why the field procedure is: push
    params (laptop, night before), draw and push the polygon (dashboard, on
    site), then arm.

Run by hand on the board if the dashboard is not up:

    ~/venv/bin/python uno_q/fence.py show
    ~/venv/bin/python uno_q/fence.py push
    ~/venv/bin/python uno_q/fence.py clear
"""

import argparse
import json
import os
import sys
import time

from pymavlink import mavutil

MISSION_TYPE_FENCE = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
CMD_VERTEX_INCLUSION = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
MIN_VERTICES = 3
# The board's fence storage is small and board-dependent; 60 corners is far
# more than a field boundary needs and keeps one upload comfortably bounded.
MAX_VERTICES = 60

DEFAULT_PATH = '~/monsoonready_data/fence.json'


# ---------------------------------------------------------------- storage

def load(path=DEFAULT_PATH):
    """[[lat, lon], ...] from disk; [] when nothing has been drawn yet."""
    p = os.path.expanduser(path)
    if not os.path.isfile(p):
        return []
    try:
        with open(p) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return []
    poly = data.get('polygon') if isinstance(data, dict) else data
    out = []
    for pt in poly or []:
        try:
            lat, lon = float(pt[0]), float(pt[1])
        except (TypeError, ValueError, IndexError):
            continue
        if abs(lat) <= 90 and abs(lon) <= 180:
            out.append([lat, lon])
    return out


def save(polygon, path=DEFAULT_PATH):
    """Atomic write. An empty polygon is legal and means 'no fence drawn'."""
    p = os.path.expanduser(path)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + '.tmp'
    with open(tmp, 'w') as f:
        json.dump({'polygon': [[round(a, 7), round(b, 7)] for a, b in polygon],
                   'saved_t': time.time()}, f, indent=1)
    os.replace(tmp, p)
    return len(polygon)


def validate(polygon):
    """Reason string if this polygon may not be pushed, else None."""
    if not polygon:
        return None                      # empty = an explicit clear
    if len(polygon) < MIN_VERTICES:
        return (f'a fence needs at least {MIN_VERTICES} corners '
                f'(this one has {len(polygon)})')
    if len(polygon) > MAX_VERTICES:
        return f'too many corners ({len(polygon)}; limit {MAX_VERTICES})'
    for lat, lon in polygon:
        if not (abs(lat) <= 90 and abs(lon) <= 180):
            return f'corner out of range: {lat}, {lon}'
    if (polygon[0][0] == polygon[-1][0] and polygon[0][1] == polygon[-1][1]
            and len(polygon) > 3):
        return ('the last corner repeats the first; ArduPilot closes the '
                'polygon itself, so drop the duplicate')
    return None


# ---------------------------------------------------------------- upload

def push(io, polygon, log=print, timeout=25.0):
    """Upload the polygon as fence mission items. Returns a status string.

    Raises RuntimeError on refusal or timeout: a fence that did not land is
    never reported as success, because the operator would then fly believing
    a boundary exists.
    """
    bad = validate(polygon)
    if bad:
        raise RuntimeError(bad)
    conn = io.conn
    ts, tc = conn.target_system, conn.target_component
    n = len(polygon)
    log(f'[fence] uploading {n} corners (mission_type=FENCE)')
    conn.mav.mission_count_send(ts, tc, n, MISSION_TYPE_FENCE)

    sent = set()
    deadline = time.monotonic() + timeout
    last_count_t = time.monotonic()
    while time.monotonic() < deadline:
        msg = conn.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=1.0)
        if msg is None:
            # The autopilot can miss the opening COUNT on a busy link; one
            # repeat every 3 s is cheap and turns a silent hang into a
            # completed upload.
            if n and time.monotonic() - last_count_t > 3.0:
                last_count_t = time.monotonic()
                conn.mav.mission_count_send(ts, tc, n, MISSION_TYPE_FENCE)
            continue
        if getattr(msg, 'mission_type', MISSION_TYPE_FENCE) != MISSION_TYPE_FENCE:
            continue                     # a waypoint/rally exchange, not ours
        t = msg.get_type()
        if t == 'MISSION_ACK':
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                log(f'[fence] ACCEPTED: {n} corners are now in the Pixhawk')
                return f'{n} corners accepted'
            raise RuntimeError(
                f'the Pixhawk REJECTED the fence (MISSION_ACK type '
                f'{msg.type}); nothing was stored')
        seq = msg.seq
        if not 0 <= seq < n:
            continue
        lat, lon = polygon[seq]
        conn.mav.mission_item_int_send(
            ts, tc, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            CMD_VERTEX_INCLUSION,
            0,                            # current
            0,                            # autocontinue
            float(n),                     # param1 = vertex count (required)
            0.0, 0.0, 0.0,
            int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0,
            MISSION_TYPE_FENCE)
        sent.add(seq)
    raise RuntimeError(
        f'no MISSION_ACK in {timeout:.0f}s ({len(sent)}/{n} corners sent); '
        f'the fence is NOT loaded')


def clear(io, log=print, timeout=10.0):
    """Erase the stored fence (count 0). Same protocol, no items."""
    conn = io.conn
    conn.mav.mission_clear_all_send(conn.target_system, conn.target_component,
                                    MISSION_TYPE_FENCE)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = conn.recv_match(type='MISSION_ACK', blocking=True, timeout=1.0)
        if msg is None:
            continue
        if getattr(msg, 'mission_type', MISSION_TYPE_FENCE) != MISSION_TYPE_FENCE:
            continue
        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            log('[fence] cleared')
            return 'fence cleared'
        raise RuntimeError(f'clear refused (MISSION_ACK type {msg.type})')
    raise RuntimeError(f'no MISSION_ACK to the clear in {timeout:.0f}s')


def read_back(io, log=print, timeout=20.0, attempts=3, settle_s=1.5):
    """Download the fence the Pixhawk actually holds. [[lat, lon], ...].

    This is the only honest proof that an upload worked: the ACK says the
    autopilot liked the exchange, the read-back says what it stored.

    WHY IT RETRIES ON AN EMPTY ANSWER (2026-08-18, observed twice on the real
    aircraft): a MISSION_COUNT of 0 arrives from a Pixhawk that demonstrably
    holds a fence. The user's own paste shows `fence.py read` returning
    nothing and then, seconds later and unchanged, all seven corners; the
    dashboard's verify-after-push hit the same thing and reported "sent 7 but
    the Pixhawk holds 0" for a fence that was in storage. A single zero is
    therefore not evidence of an empty fence, and treating it as evidence is
    what made a working fence look broken. A NON-empty answer is trusted
    immediately; only zero is retried, so a genuinely empty fence costs
    attempts * settle_s seconds to confirm and nothing else changes.
    """
    for attempt in range(attempts):
        got = _read_back_once(io, timeout)
        if got:
            return got
        if attempt < attempts - 1:
            log(f'[fence] read-back came back empty ({attempt + 1}/{attempts}), '
                f'retrying in {settle_s:g}s')
            time.sleep(settle_s)
    return []


def _read_back_once(io, timeout=20.0):
    """One download attempt. Returns [] for an empty or unanswered fence."""
    conn = io.conn
    ts, tc = conn.target_system, conn.target_component
    conn.mav.mission_request_list_send(ts, tc, MISSION_TYPE_FENCE)
    total, got = None, {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = conn.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM_INT'],
                              blocking=True, timeout=1.0)
        if msg is None:
            continue
        if getattr(msg, 'mission_type', MISSION_TYPE_FENCE) != MISSION_TYPE_FENCE:
            continue
        if msg.get_type() == 'MISSION_COUNT':
            total = msg.count
            if total == 0:
                break
            conn.mav.mission_request_int_send(ts, tc, 0, MISSION_TYPE_FENCE)
            continue
        got[msg.seq] = [msg.x / 1e7, msg.y / 1e7]
        if total is not None and len(got) >= total:
            break
        nxt = len(got)
        conn.mav.mission_request_int_send(ts, tc, nxt, MISSION_TYPE_FENCE)
    # Politeness the protocol requires: tell the vehicle the download ended.
    conn.mav.mission_ack_send(ts, tc, mavutil.mavlink.MAV_MISSION_ACCEPTED,
                              MISSION_TYPE_FENCE)
    return [got[i] for i in sorted(got)]


# ---------------------------------------------------------------- CLI

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('action', choices=['show', 'push', 'clear', 'read'])
    ap.add_argument('--file', default=DEFAULT_PATH)
    ap.add_argument('--conn', default='auto')
    args = ap.parse_args()

    if args.action == 'show':
        poly = load(args.file)
        print(f'{len(poly)} corners in {args.file}')
        for i, (lat, lon) in enumerate(poly, 1):
            print(f'  {i:2d}  {lat:.7f},{lon:.7f}')
        bad = validate(poly)
        if bad:
            print(f'NOT PUSHABLE: {bad}')
        return 0

    from mavlink_io import MavIO
    io = MavIO(args.conn)
    io.wait_ready(timeout=20)
    if args.action == 'push':
        poly = load(args.file)
        print(push(io, poly))
        back = read_back(io)
        print(f'read back {len(back)} corners from the Pixhawk')
    elif args.action == 'clear':
        print(clear(io))
    else:
        for i, (lat, lon) in enumerate(read_back(io), 1):
            print(f'  {i:2d}  {lat:.7f},{lon:.7f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
