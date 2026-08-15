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
# Consecutive mav_read failures tolerated before declaring the shovel dead.
MAX_FAILS = 20


def main():
    try:
        rc = RouterClient()
    except OSError as exc:
        sys.exit(f"cannot open the router socket: {exc}")

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp.bind(PUMP_ADDR)
    except OSError as exc:
        # Almost always "a pump is already running", which is FINE - the
        # existing one is doing the job. Said plainly because the raw
        # traceback (exit 1 behind setsid nohup, output in a log nobody has
        # opened yet) looks exactly like the link having failed.
        sys.exit(f"cannot bind {PUMP_ADDR[0]}:{PUMP_ADDR[1]} ({exc}).\n"
                 f"A pump is probably ALREADY RUNNING, which is fine - do not "
                 f"start a second one. Check with:  pgrep -af "
                 f"mav_shovel_pump.py\n"
                 f"To replace it:  pkill -f mav_shovel_pump.py  then start "
                 f"again.")
    udp.setblocking(False)

    print(f"pumping: router <-> udp {MISSION_ADDR[0]}:{MISSION_ADDR[1]} "
          f"(mission side) / {PUMP_ADDR[1]} (pump side)")
    up = down = 0
    next_stats = time.monotonic() + STATS_EVERY_S
    # A single RPC hiccup must not kill the pump: the aircraft may be
    # airborne and this process is its only link. Only a sustained run of
    # failures means the shovel is really gone. (Even then the Pixhawk's
    # GUID_TIMEOUT=3 stops and holds it, which is the designed backstop.)
    fails = 0

    while True:
        # Pixhawk -> mission
        try:
            b64 = rc.call("mav_read")
            fails = 0
        except RouterError as exc:
            fails += 1
            print(f"  mav_read failed ({fails}/{MAX_FAILS}): {exc}")
            if fails >= MAX_FAILS:
                sys.exit("shovel unreachable; PILOT: take the aircraft on "
                         "the mode switch")
            time.sleep(0.1)
            continue
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
                    # Dropping one outbound frame is survivable; MAVLink
                    # setpoints are resent continuously by design.
                    print(f"  mav_write dropped a frame: {exc}")
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
