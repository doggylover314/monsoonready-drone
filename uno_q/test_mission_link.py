"""Prove the MISSION STACK sees the Pixhawk through the shovel + pump.

This is the last software unknown before the farm: test_shovel proved raw
bytes flow; this proves the exact objects the mission flies with (MavIO ->
udpin -> pump -> router -> STM32 -> SERIAL5) produce live telemetry.

Two terminals on the BOARD (the pump is a server, it gets its own):

    terminal 1:  ~/venv/bin/python uno_q/mav_shovel_pump.py
    terminal 2:  ~/venv/bin/python uno_q/test_mission_link.py

PASS = "MISSION STACK LIVE" with a mode and a battery voltage. Position may
be None indoors (no GPS fix) - that is the sky's fault, not the stack's.
Nothing here arms, changes mode, or moves a servo: read-only by design,
because it will be run the night before the flight.
"""

import sys
import time

from mavlink_io import MavIO

CONN = 'udpin:127.0.0.1:14555'
RUN_S = 15.0


def main():
    print(f"connecting MavIO to {CONN} (start the pump first) ...")
    io = MavIO(CONN)
    try:
        io.wait_ready(timeout=20)
    except (TimeoutError, RuntimeError) as exc:
        sys.exit(f"{exc}\n(if nothing at all arrived: is "
                 f"mav_shovel_pump.py running in the other terminal?)")
    print("heartbeat received: RECEIVE direction live at MAVLink level")

    # setup_streams sends SET_MESSAGE_INTERVAL commands and waits for their
    # COMMAND_ACKs, so it succeeding IS the transmit-direction proof: the
    # Pixhawk heard a command that originated in this process. It raises on
    # no-ack, so reaching the next line means both directions work.
    try:
        io.setup_streams()
    except (TimeoutError, RuntimeError) as exc:
        sys.exit(f"stream request got no ACK - transmit direction is the "
                 f"suspect (Serial1 TX -> RX5 conductor): {exc}")
    print("stream request ACKed: TRANSMIT direction proven end to end")

    t_end = time.monotonic() + RUN_S
    while time.monotonic() < t_end:
        io.step()

    t = io.tel
    age = time.monotonic() - t.heartbeat_t
    print(f"\nheartbeat age {age:.1f}s   mode {t.mode}   armed {t.armed}")
    print(f"position lat {t.lat} lon {t.lon} rel_alt {t.rel_alt_m}")
    print(f"rangefinder {t.rng_m} m (valid {t.rng_valid})   "
          f"velocity NED {t.vn_mps} {t.ve_mps} {t.vd_mps}")
    if t.mode is not None and age < 3.0:
        print("\nMISSION STACK LIVE: MavIO flies tomorrow on this exact "
              "connection string.")
        return 0
    print("\nheartbeat arrived but telemetry is stale/empty; run it again "
          "and check the pump terminal for errors.")
    return 1


if __name__ == '__main__':
    sys.exit(main())
