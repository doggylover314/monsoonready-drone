#!/usr/bin/env python3
"""Mute and unmute the ESP32 obstacle ring so the SiK radio has the link to
itself.

    ./python tools/esp32_mute.py status     # what the board is set to now
    ./python tools/esp32_mute.py off        # ring silenced, radio clear
    ./python tools/esp32_mute.py on         # ring back, as the param file has it

On the UNO Q the interpreter is ~/venv/bin/python, not ./python.

Why this exists. ArduPilot forwards any MAVLink message with no target_system
field to every other link it has learned a route on (MAVLink_routing::forward,
read from the Copter 4.7.0 source on 2026-08-17). OBSTACLE_DISTANCE carries no
target field, and the ESP32 sends one every 100 ms with all 72 sector slots
filled (unused slots are 65535, never 0, so MAVLink2 truncation saves nothing:
167 payload bytes + 12 = 179 on the wire). With the up sensor and the 1 Hz
heartbeat that is roughly 2.2 kB/s re-sent out TELEM2 into a 57600 SiK link.
It competes with everything QGC asks for, which is the leading explanation for
a full parameter download going from about a minute to ten and then failing.

This is a switch, not the fix. It disables a working sensor to buy radio
bandwidth, so it belongs in calibration and parameter sessions on the ground,
never in flight. The permanent options are recorded in PROJECT_STATE.md.

Both parameters move together, deliberately: turning off SERIAL1 alone would
leave PRX1_TYPE=2 pointing at a port that can no longer deliver data, and this
project's own record (2026-08-13, 2026-08-14) treats a proximity sensor that
is configured but not streaming as an arming risk. off parks PRX1_TYPE at 0;
on restores it from param_dumps/pixhawk_full_setup.param, so the restored
values can never drift from the project's own config.

Serial protocol and proximity type both bind at startup, so each change
reboots the flight controller, then confirms the result two ways: reading
the parameters back, and counting what the ESP32 actually puts on the link.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pymavlink import mavutil                                     # noqa: E402
from mavlink_link import connect, send_and_ack                    # noqa: E402
from parameters import await_param, f32, fmt, load_param_file, write_param  # noqa: E402

# The two parameters that decide whether the ring exists as far as the flight
# controller is concerned. SERIAL1 is TELEM1, where the ESP32 is wired.
NAMES = ('SERIAL1_PROTOCOL', 'PRX1_TYPE')
MUTED = {'SERIAL1_PROTOCOL': -1,   # -1 = port disabled: nothing to forward
         'PRX1_TYPE': 0}           # 0 = no proximity sensor configured

ESP32_COMPID = 195                 # the ring announces itself as component 195


def live_values():
    """The values to restore come from the project's param file, never from here.

    Hard-coding "SERIAL1_PROTOCOL 2, PRX1_TYPE 2" in this script would create a
    second source of truth that silently rots the day the file changes.
    """
    cfg = load_param_file()
    missing = [n for n in NAMES if n not in cfg]
    if missing:
        sys.exit("pixhawk_full_setup.param does not define " +
                 ", ".join(missing) + ", so this script cannot know what to "
                 "restore. Add the line(s) there first.")
    return {n: cfg[n] for n in NAMES}


def read_param(m, name, attempts=3):
    """One parameter, retried, because a dropped request looks like a typo."""
    for _ in range(attempts):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        p = await_param(m, name, timeout=4.0)
        if p is not None:
            return p.param_value
    return None


def is_armed(m, timeout=8.0):
    """Armed state from the autopilot's own heartbeat. None if none arrived.

    A reboot in flight is the one way this script could hurt anything, so it
    refuses to act on anything but a heartbeat it has actually seen.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb is None:
            continue
        if hb.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            continue
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


def ring_traffic(m, seconds=6.0):
    """Count what the ESP32 puts on this link, which is the real proof.

    Two independent counts: anything from component 195, and OBSTACLE_DISTANCE
    of any origin. No stream request is needed, because forwarded packets
    arrive unbidden; that is the whole problem being measured.
    """
    end, from_esp, obstacle = time.monotonic() + seconds, 0, 0
    while time.monotonic() < end:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_srcComponent() == ESP32_COMPID:
            from_esp += 1
        if msg.get_type() == 'OBSTACLE_DISTANCE':
            obstacle += 1
    return from_esp, obstacle


def reconnect(args, tries=6, first_wait=8.0):
    """Reopen the link after a reboot.

    Over USB the device node disappears and can come back under a different
    name, so the port is re-resolved from scratch every attempt rather than
    reused. connect() exits the process on failure, hence the SystemExit
    catch: here a failure is expected until the board finishes booting.
    """
    time.sleep(first_wait)
    for attempt in range(tries):
        try:
            m, _, _ = connect(args.conn, args.baud, quiet=(attempt > 0))
            return m
        except SystemExit:
            print(f"  board still booting, retry {attempt + 2}/{tries} ...")
            time.sleep(5)
    sys.exit("the flight controller did not come back after the reboot. "
             "Power-cycle it, then run `status` to see where it stands.")


def show(m):
    """Print both parameters and return them, so no caller reads them twice.

    Over a loaded radio every extra round trip is another chance of a lost
    packet, and this script exists precisely because that link is lossy.
    """
    values = {}
    for name in NAMES:
        values[name] = read_param(m, name)
        print(f"  {name} = "
              f"{'NO REPLY' if values[name] is None else fmt(values[name])}")
    return values


def apply(args, wanted, expect_traffic):
    """Write both parameters, reboot, verify by read-back and by listening."""
    m, _, _ = connect(args.conn, args.baud)

    armed = is_armed(m)
    if armed is None:
        sys.exit("no autopilot heartbeat, so the armed state is unknown. "
                 "Refusing to reboot the flight controller blind.")
    if armed:
        sys.exit("THE AIRCRAFT IS ARMED. This command reboots the flight "
                 "controller. Disarm first.")

    print("before:")
    show(m)

    print("writing:")
    failed = []
    for name, value in wanted.items():
        ok, echoed = write_param(m, name, value)
        if ok:
            print(f"  OK   {name} = {fmt(value)}")
        else:
            failed.append(name)
            print(f"  FAIL {name}: asked {fmt(value)}, board echoed "
                  f"{'nothing' if echoed is None else fmt(echoed)}")
    if failed:
        sys.exit("refusing to reboot with " + ", ".join(failed) + " unwritten; "
                 "a half-applied change is worse than none.")

    print("rebooting the flight controller (both parameters bind at startup) ...")
    print(f"  reboot command: {send_and_ack(m, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1, timeout=3.0)}"
          "   (NO ACK is normal here: the board reboots instead of replying)")
    try:
        m.close()
    except Exception:                                             # noqa: BLE001
        pass

    m = reconnect(args)
    print("after:")
    got = show(m)
    bad = [n for n, want in wanted.items()
           if got[n] is None or f32(got[n]) != f32(want)]
    if bad:
        sys.exit("these did not survive the reboot: " + ", ".join(bad))

    print(f"listening {6} s for the ring ...")
    from_esp, obstacle = ring_traffic(m)
    print(f"  {from_esp} messages from component {ESP32_COMPID}, "
          f"{obstacle} OBSTACLE_DISTANCE")
    silent = (from_esp == 0 and obstacle == 0)
    if expect_traffic and silent:
        print("The ring is configured but nothing is arriving. The ESP32 is "
              "powered off, unplugged, or its TELEM1 wiring is disturbed. The "
              "parameters are correct either way.")
    elif not expect_traffic and not silent:
        print("STILL TALKING. The parameters read back correctly, so this is "
              "worth understanding before trusting the radio: check that the "
              "ESP32 really is on TELEM1 and not another MAVLink port.")
    elif not expect_traffic:
        print("Ring silent. The radio now has the link to itself.")
    else:
        print("Ring live again.")


def main():
    ap = argparse.ArgumentParser(
        description="Silence the ESP32 obstacle ring so the SiK radio is not "
                    "sharing its bandwidth with 10 Hz proximity data.")
    ap.add_argument('cmd', choices=['off', 'on', 'status'])
    ap.add_argument('--conn', default=None,
                    help='serial device; omit to auto-pick when exactly one '
                         'is present')
    ap.add_argument('--baud', type=int, default=None,
                    help='omit to follow the port type: 57600 for a SiK '
                         'radio, 115200 for USB')
    args = ap.parse_args()

    if args.cmd == 'status':
        m, _, _ = connect(args.conn, args.baud)
        show(m)
        from_esp, obstacle = ring_traffic(m)
        print(f"  {from_esp} messages from component {ESP32_COMPID}, "
              f"{obstacle} OBSTACLE_DISTANCE in 6 s")
        return

    if args.cmd == 'off':
        apply(args, MUTED, expect_traffic=False)
    else:
        apply(args, live_values(), expect_traffic=True)


if __name__ == '__main__':
    main()
