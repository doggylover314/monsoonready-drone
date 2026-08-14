"""Full-time pump: byte-shovel RPC <-> local UDP, so MavIO needs no changes.

    ~/venv/bin/python uno_q/mav_shovel_pump.py        # terminal 1, stays up
    then the mission connects with conn_str udpin:127.0.0.1:14555

WHY UDP IN THE MIDDLE: MavIO already accepts any pymavlink connection
string, and pymavlink's udpin: transport locks onto the first peer that
sends to it and replies to that peer. This pump binds 127.0.0.1:14556, sends
every byte the Pixhawk produces to 14555, and whatever the mission writes
comes back to 14556 and is shoveled out. MavIO therefore flies with
`udpin:127.0.0.1:14555` and zero code changes; the pump is the only process
that knows the router exists. Loopback UDP loss is not a real concern, and
MAVLink is loss-tolerant by design anyway.

Poll rate: 50 Hz. The serial leg tops out at ~11520 B/s; mav_read returns up
to 768 B per call, so 50 Hz gives 3x headroom. The sketch's 4 KiB ring
absorbs stalls; check `mav_stats` (printed at exit and every 10 s) - a
rising dropped counter is the sign the pump is not keeping up.
"""

import base64
import select
import socket
import sys
import time

from router_client import RouterClient, RouterError

MISSION_ADDR = ("127.0.0.1", 14555)   # MavIO's udpin side
PUMP_ADDR = ("127.0.0.1", 14556)      # this pump's own socket
POLL_S = 0.02
STATS_EVERY_S = 10.0
# Raw-byte cap per mav_write call; the sketch's decode buffer is 768.
WRITE_CHUNK = 512


def main():
    try:
        rc = RouterClient()
    except OSError as exc:
        sys.exit(f"cannot open the router socket: {exc}")

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(PUMP_ADDR)
    udp.setblocking(False)

    print(f"pumping: router <-> udp {MISSION_ADDR[0]}:{MISSION_ADDR[1]} "
          f"(mission side) / {PUMP_ADDR[1]} (pump side)")
    up = down = 0
    next_stats = time.monotonic() + STATS_EVERY_S

    while True:
        # Pixhawk -> mission
        try:
            b64 = rc.call("mav_read")
        except RouterError as exc:
            sys.exit(f"mav_read failed ({exc}); is the shovel sketch up?")
        data = base64.b64decode(b64) if b64 else b""
        if data:
            udp.sendto(data, MISSION_ADDR)
            down += len(data)

        # mission -> Pixhawk
        while True:
            r, _, _ = select.select([udp], [], [], 0)
            if not r:
                break
            pkt, _addr = udp.recvfrom(65535)
            for i in range(0, len(pkt), WRITE_CHUNK):
                chunk = pkt[i:i + WRITE_CHUNK]
                try:
                    rc.call("mav_write", base64.b64encode(chunk).decode())
                except RouterError as exc:
                    sys.exit(f"mav_write failed ({exc})")
            up += len(pkt)

        now = time.monotonic()
        if now >= next_stats:
            next_stats = now + STATS_EVERY_S
            try:
                stats = rc.call("mav_stats")
            except RouterError:
                stats = "?"
            print(f"  up {up} B, down {down} B, sketch rx,drop,tx = {stats}")

        time.sleep(POLL_S)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\npump stopped")
