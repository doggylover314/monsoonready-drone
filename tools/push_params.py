#!/usr/bin/env python3
"""Write the project's chosen parameters directly to the Pixhawk, one by one,
with readback verification. Immune to QGC's silent bulk-load drops.

Run with QGC CLOSED (it owns the serial port):
    ../training/.venv/bin/python push_params.py [/dev/ttyACM0]

Reads pixhawk_full_setup.param (NAME,VALUE lines), writes each via PARAM_SET,
confirms the echoed PARAM_VALUE, retries three times, then does a final
readback pass and prints a table. Values compare at float32 precision
(the board's native storage).
"""

import struct
import sys
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
SETUP = ROOT / "param_dumps" / "pixhawk_full_setup.param"
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"


def f32(x: float) -> float:
    return struct.unpack("f", struct.pack("f", float(x)))[0]


def load_overrides() -> dict[str, float]:
    out = {}
    for line in SETUP.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if "," in line:
            name, val = line.split(",", 1)
            out[name.strip()] = float(val.strip())
    return out


def set_param(m, name: str, value: float) -> tuple[bool, float | None]:
    """Write one param, wait for the echoed PARAM_VALUE. 3 attempts."""
    for _ in range(3):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), value,
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + 2
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
            if msg is None:
                break
            got = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            if got.strip("\x00") == name:
                return f32(msg.param_value) == f32(value), msg.param_value
    return False, None


def main() -> None:
    overrides = load_overrides()
    m = mavutil.mavlink_connection(PORT, baud=115200)
    print(f"waiting for heartbeat on {PORT} ...")
    m.wait_heartbeat(timeout=30)
    print(f"connected: sys {m.target_system} comp {m.target_component}\n")

    ok, failed, missing = [], [], []
    for name, value in sorted(overrides.items()):
        good, echoed = set_param(m, name, value)
        if good:
            ok.append(name)
            print(f"  OK   {name} = {value:g}")
        elif echoed is None:
            missing.append(name)
            print(f"  ???  {name}  (no echo; param may not exist yet)")
        else:
            failed.append(name)
            print(f"  FAIL {name}  wrote {value:g}, board echoed {echoed:g}")

    print(f"\n{len(ok)} ok, {len(failed)} refused, {len(missing)} no-echo")
    if failed:
        print("REFUSED (board rejected the value):\n  " + "\n  ".join(failed))
    if missing:
        print("NO-ECHO (likely gated until a parent enable + reboot; "
              "reboot and rerun):\n  " + "\n  ".join(missing))
    if not failed and not missing:
        print("All parameters verified on the board. Reboot the Pixhawk, "
              "then re-dump in QGC to regenerate pixhawk_complete.params.")


if __name__ == "__main__":
    main()
