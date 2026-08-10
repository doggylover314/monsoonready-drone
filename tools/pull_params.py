#!/usr/bin/env python3
"""Dump EVERY parameter from the Pixhawk and build the canonical full file.

Run with QGC CLOSED (it owns the serial port otherwise):
    .././python pull_params.py [/dev/ttyACM0]

Outputs, in the project root:
  * pixhawk_all_params_dump.param  : raw NAME,VALUE of every param on the
    board at pull time (backup; never edit).
  * pixhawk_every_param.param      : the same complete set with the deltas
    from pixhawk_full_setup.param overlaid and marked. This is the file the
    user asked for: every single parameter the firmware exposes, defined.

The dump preserves the board's own calibration values (compass, accel, RC),
which is why this is generated from the vehicle and not from ArduPilot docs.
"""

import sys
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
SETUP = ROOT / "param_dumps" / "pixhawk_full_setup.param"
DUMP = ROOT / "param_dumps" / "pixhawk_all_params_dump.param"
FULL = ROOT / "param_dumps" / "pixhawk_every_param.param"

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"


def fetch_all(port: str) -> dict[str, float]:
    m = mavutil.mavlink_connection(port, baud=115200)
    print(f"waiting for heartbeat on {port} ...")
    m.wait_heartbeat(timeout=30)
    print(f"connected: sys {m.target_system} comp {m.target_component}")
    m.mav.param_request_list_send(m.target_system, m.target_component)

    params: dict[str, float] = {}
    total = None
    last_rx = time.time()
    while True:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
        if msg is None:
            if time.time() - last_rx > 10:
                break  # stream ended
            continue
        last_rx = time.time()
        name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        params[name.strip("\x00")] = msg.param_value
        total = msg.param_count
        if total and len(params) >= total:
            break
        if len(params) % 100 == 0:
            print(f"  {len(params)}/{total or '?'}")

    if total and len(params) < total:
        # retry missing indexes once via targeted requests
        print(f"got {len(params)}/{total}, retrying stragglers ...")
        m.mav.param_request_list_send(m.target_system, m.target_component)
        deadline = time.time() + 30
        while len(params) < total and time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
            if msg is None:
                continue
            name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            params[name.strip("\x00")] = msg.param_value
    print(f"fetched {len(params)} parameters (board reports {total})")
    return params


def fmt(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else repr(round(float(v), 6))


def setup_deltas() -> dict[str, str]:
    deltas = {}
    if SETUP.exists():
        for line in SETUP.read_text().splitlines():
            line = line.split("#", 1)[0].strip()
            if "," in line:
                name, val = line.split(",", 1)
                deltas[name.strip()] = val.strip()
    return deltas


def main() -> None:
    params = fetch_all(PORT)
    if not params:
        sys.exit("no parameters received; is QGC closed and the board on USB?")

    DUMP.write_text(
        "# Raw full dump, do not edit. Board state at pull time.\n"
        + "\n".join(f"{k},{fmt(v)}" for k, v in sorted(params.items())) + "\n")

    deltas = setup_deltas()
    unknown = [k for k in deltas if k not in params]
    lines = ["# EVERY parameter on this firmware, explicitly defined.",
             "# Base = board dump; lines marked  # SETUP  carry the project's",
             "# chosen values from pixhawk_full_setup.param.",
             "# Regenerate after any calibration: tools/pull_params.py"]
    for k, v in sorted(params.items()):
        if k in deltas:
            lines.append(f"{k},{deltas[k]}  # SETUP (board had {fmt(v)})")
        else:
            lines.append(f"{k},{fmt(v)}")
    FULL.write_text("\n".join(lines) + "\n")

    print(f"wrote {DUMP.name} and {FULL.name}")
    if unknown:
        print("WARNING: these setup params do NOT exist on this firmware, "
              "check names:\n  " + "\n  ".join(unknown))


if __name__ == "__main__":
    main()
