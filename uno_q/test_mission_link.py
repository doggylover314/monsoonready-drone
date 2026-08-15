"""Prove the MISSION STACK sees the Pixhawk over its USB link.

This proves the exact objects the mission flies with (MavIO on the
Pixhawk's USB port through the board's hub, resolved from
/dev/serial/by-id) produce live telemetry. One terminal on the BOARD:

    ~/venv/bin/python uno_q/test_mission_link.py

PASS = "MISSION STACK LIVE" with a mode and a battery voltage. Position may
be None indoors (no GPS fix) - that is the sky's fault, not the stack's.
Nothing here arms, changes mode, or moves a servo: read-only by design,
because it will be run the night before the flight.

(Until 2026-08-16 this went through the STM32 byte-shovel on SERIAL5; that
chain is deleted, the wires are out, and 'auto' finds the USB device.)
"""

import sys
import time

from mavlink_io import MavIO

CONN = 'auto'
RUN_S = 15.0


def main():
    print(f"connecting MavIO to {CONN} ...")
    io = MavIO(CONN)
    try:
        io.wait_ready(timeout=20)
    except (TimeoutError, RuntimeError) as exc:
        sys.exit(f"{exc}\n(if nothing at all arrived: is the Pixhawk's USB "
                 f"plug seated in the hub? ls -l /dev/serial/by-id/)")
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
