"""Scripted SITL scenarios for the mission state machine.

Start SITL first (hexa, matching the F550; rangefinder params come from the
parm file; speedup makes the run quick):

  ../ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa \
      --no-mavproxy --speedup 5 \
      --add-param-file=uno_q/sitl_rangefinder.parm

Then:

  .venv/bin/python uno_q/sitl_test.py                 # happy path
  .venv/bin/python uno_q/sitl_test.py --drill dropout # rangefinder-loss drill

Happy path PASS: survey -> latch -> descend -> exactly one drop -> resume ->
survey completes -> RTL, no aborts.
Dropout drill PASS: the rangefinder 'goes silent' below 6m (a client-side
hook, no SIM param surgery), the mission aborts upward, zero drops, and the
survey still completes.
"""

import argparse
import sys
import time

from pymavlink import mavutil

from detector import FakeDetector, offset_latlon
from dropper import LogDropper
from mavlink_io import MavIO
from mission import Mission, MissionConfig
from missionlog import MissionLog


def connect_retry(conn_str, timeout=60):
    """SITL takes a few seconds to open TCP 5760 after launch; retry politely."""
    deadline = time.monotonic() + timeout
    while True:
        try:
            return MavIO(conn_str)
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(1)


def wait_position(io, timeout=120):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        io.step()
        if io.tel.lat is not None and abs(io.tel.lat) > 1e-3:
            return io.tel.lat, io.tel.lon
    raise TimeoutError("no GLOBAL_POSITION_INT with a real fix from SITL")


def get_param(io, name, timeout=5):
    io.conn.mav.param_request_read_send(
        io.conn.target_system, io.conn.target_component, name.encode(), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = io.step()
        if (msg is not None and msg.get_type() == 'PARAM_VALUE'
                and msg.param_id == name):
            return msg.param_value
    raise TimeoutError(f"no PARAM_VALUE for {name}")


def wait_disarm(io, timeout=300):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        io.step()
        if not io.tel.armed:
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='tcp:127.0.0.1:5760')
    ap.add_argument('--drill', choices=['none', 'dropout'], default='none')
    ap.add_argument('--record', metavar='DATA_DIR', default=None,
                    help='write a base-station JSONL mission log here')
    args = ap.parse_args()

    print(f"[test] connecting {args.conn} ...")
    io = connect_retry(args.conn)
    io.wait_ready()
    print(f"[test] heartbeat from sys {io.conn.target_system}")

    # Guard: SITL must simulate the rangefinder or every descent aborts.
    if int(get_param(io, 'RNGFND1_TYPE')) != 100:
        sys.exit("RNGFND1_TYPE != 100: relaunch sim_vehicle with "
                 "--add-param-file=uno_q/sitl_rangefinder.parm")

    io.setup_streams()
    home = wait_position(io)
    print(f"[test] home {home[0]:.7f},{home[1]:.7f}")

    # 25m survey square; puddle planted on the second leg.
    wps = [offset_latlon(*home, 25, 0), offset_latlon(*home, 25, 25),
           offset_latlon(*home, 0, 25), offset_latlon(*home, 0, 0)]
    puddle = offset_latlon(*home, 25, 12)

    detector = FakeDetector(*puddle, radius_m=6.0, max_fires=1)
    dropper = LogDropper()
    recorder = MissionLog(args.record) if args.record else None
    if recorder:
        print(f"[test] recording to {recorder.path}")
    mission = Mission(io, detector, dropper,
                      MissionConfig(waypoints=wps), recorder=recorder)
    if args.drill == 'dropout':
        mission.rng_suppress_below_m = 6.0

    final = mission.run()
    print(f"[test] mission returned in state {final}")
    landed = wait_disarm(io)
    print(f"[test] disarmed={landed}")

    seen = {s for _, s, _ in mission.history}
    if args.drill == 'none':
        ok = (final == 'DONE' and dropper.fired == 1
              and 'DROP' in seen and 'ABORT_CLIMB' not in seen)
    else:
        ok = (final == 'DONE' and dropper.fired == 0
              and 'ABORT_CLIMB' in seen and 'DROP' not in seen)

    print("[test] history:")
    for t, s, note in mission.history:
        print(f"    {s:<12} {note}")
    print(f"[test] drops={dropper.fired} abort={mission.abort_reason!r}")
    print(f"[test] {'PASS' if ok else 'FAIL'} ({args.drill})")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
