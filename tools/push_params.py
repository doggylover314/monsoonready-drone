#!/usr/bin/env python3
"""Write the project's chosen parameters directly to the Pixhawk, one by one,
with readback verification. Immune to QGC's silent bulk-load drops.

Run with QGC CLOSED (it owns the serial port):
    ../training/.venv/bin/python push_params.py [/dev/ttyACM0]

Reads pixhawk_full_setup.param (NAME,VALUE lines), writes each via PARAM_SET,
confirms the echoed PARAM_VALUE for THAT name, retrying three times. The check
is the write-time echo, not a separate readback pass: a value that a later
write or a parent-enable resets would still be reported OK, so re-run the tool
after a reboot if that matters. Values compare at float32 precision (the
board's native storage). Exits non-zero if anything was refused or unechoed.
"""

import os
import struct
import sys
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
SETUP = ROOT / "param_dumps" / "pixhawk_full_setup.param"
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
# Baud matters only for real serial links. USB CDC ignores it; a SiK ground
# radio does not (its PC-side port is 57600 by default), so:
#     push_params.py /dev/ttyUSB0 57600
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200


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
    if not overrides:
        sys.exit(f"{SETUP} yielded no NAME,VALUE lines. Refusing to report "
                 f"success on an empty push (a QGC-saved, tab-separated file "
                 f"parses to nothing here).")
    print(f"pushing {len(overrides)} parameters from {SETUP.name}")
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    print(f"waiting for heartbeat on {PORT} at {BAUD} ...")
    # wait_heartbeat() returns the FIRST heartbeat of any kind and does not
    # pin target_system; with the ESP32 on the bus that leaves the target at
    # 0 = broadcast and every write goes to nobody. Documented at length in
    # wiring_check.wait_autopilot, reused here rather than repeated.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from wiring_check import wait_autopilot
    if not wait_autopilot(m):
        sys.exit("no AUTOPILOT heartbeat (is QGC closed and the board "
                 "powered?). Refusing to write parameters to a link that was "
                 "never established.")
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
        sys.exit(0)
    # The verdict has to reach the exit code: this tool is chained in the
    # bench sequence, and a refused safety parameter must stop the chain.
    sys.exit(1)


if __name__ == "__main__":
    main()
