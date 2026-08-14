"""End-to-end proof of the Linux -> Pixhawk link through the byte-shovel.

Run on the UNO Q after flashing sketch_mav_shovel:

    ~/venv/bin/python uno_q/test_shovel.py

PASS looks like BOTH of:
  * this script printing HEARTBEAT lines from sys 1 comp 1 - that is the
    Pixhawk's own 1 Hz SERIAL5 output arriving Pixhawk -> D0 -> Serial1 ->
    ring -> Bridge -> router -> unix socket -> here, i.e. the RECEIVE
    direction that the probe sketch's LED could never report with the
    aircraft shut;
  * `./python tools/bench.py nodes` on the LAPTOP hearing comp 191 - those
    heartbeats now originate from THIS script through the shovel, proving
    the TRANSMIT direction end to end from Linux.

Together that is the whole autonomy link: what MavIO needs is exactly these
two byte streams, which mav_shovel_pump.py then carries full-time.
"""

import base64
import os
import sys
import time

# Force MAVLink2 framing (0xFD, 21-byte heartbeat), matching what the probe
# sketch sent and what SERIAL5_PROTOCOL=2 means. Must be set before import.
os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil                                    # noqa: E402

from router_client import RouterClient, RouterError

RUN_S = 20.0
POLL_S = 0.05          # 20 Hz drain, far above the 1 Hz heartbeat need
HEARTBEAT_PERIOD_S = 1.0


def main():
    try:
        rc = RouterClient()
    except OSError as exc:
        sys.exit(f"cannot open the router socket: {exc}")

    # Offline MAVLink2 parser for inbound bytes, and an encoder that stamps
    # us as the UNO Q's established identity: system 1, component 191.
    parser = mavutil.mavlink.MAVLink(None)
    parser.robust_parsing = True
    enc = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=191)

    print(f"shoveling for {RUN_S:.0f}s ...")
    t_end = time.monotonic() + RUN_S
    next_hb = 0.0
    rx_bytes = 0
    heard = {}

    while time.monotonic() < t_end:
        now = time.monotonic()

        if now >= next_hb:
            next_hb = now + HEARTBEAT_PERIOD_S
            hb = enc.heartbeat_encode(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            data = hb.pack(enc)
            try:
                wrote = rc.call("mav_write",
                                base64.b64encode(data).decode())
            except RouterError as exc:
                sys.exit(f"mav_write failed: {exc}")
            if wrote != len(data):
                print(f"  short write: {wrote}/{len(data)}")

        try:
            b64 = rc.call("mav_read")
        except RouterError as exc:
            sys.exit(f"mav_read failed: {exc}")
        chunk = base64.b64decode(b64) if b64 else b""
        rx_bytes += len(chunk)
        if chunk:
            msgs = parser.parse_buffer(chunk) or []
            for m in msgs:
                key = (m.get_srcSystem(), m.get_srcComponent())
                heard[key] = heard.get(key, 0) + 1
                if m.get_type() == 'HEARTBEAT':
                    print(f"  HEARTBEAT from sys {key[0]} comp {key[1]}")
        time.sleep(POLL_S)

    try:
        stats = rc.call("mav_stats")
    except RouterError:
        stats = "unavailable"
    rc.close()

    print(f"\n{rx_bytes} bytes received; talkers heard: "
          f"{ {k: v for k, v in sorted(heard.items())} }")
    print(f"sketch stats rx,dropped,tx = {stats}")
    if any(comp == 1 for (_, comp) in heard):
        print("RECEIVE DIRECTION PROVEN: the Pixhawk is audible from Linux.")
        print("Now run on the LAPTOP:  ./python tools/bench.py nodes")
        print("comp 191 there = transmit direction proven = LINK COMPLETE.")
        return 0
    print("NO PIXHAWK TRAFFIC. In order of cheapness: is the shovel sketch "
          "flashed (not the old probe)? mav_stats rx staying 0 means "
          "nothing reaches Serial1: check SERIAL5_PROTOCOL=2/BAUD=115 and "
          "the TX5->D0 conductor, which the probe could never test.")
    return 1


if __name__ == '__main__':
    sys.exit(main())
