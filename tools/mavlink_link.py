#!/usr/bin/env python3
"""Shared MAVLink transport plumbing. A LIBRARY, not a tool: nothing here
prints a verdict or exits with a judgement, and it has no CLI.

Everything that talks to the Pixhawk needs the same four things right, and
each one has cost this project a wasted bench session at least once:

  WHICH PORT      device names differ by OS and move between reboots
  WHICH BAUD      115200 on the Pixhawk's USB, 57600 for a SiK ground radio
  WHICH TARGET    there are TWO MAVLink talkers on this bus, and locking onto
                  the wrong one leaves every command addressed to nobody
  WHICH ACK       the reply you want is behind a pile of streamed telemetry

These lived in wiring_check.py until 2026-08-10, so bench.py, flow_test.py and
the param tools all had to import from a 623-line PASS/FAIL test just to open
a serial port. Importing a test to get a library is backwards, and it meant a
change to the wiring check could break the param push. They live here now.
"""

import glob
import os
import sys
import time

from pymavlink import mavutil


def serial_candidates():
    """Serial devices that could plausibly be the Pixhawk or a SiK radio.

    Linux and macOS name these completely differently, and this repo is used
    from both (the owner's Linux laptop and Raghav's MacBook), so a Linux-only
    glob silently finds nothing on a Mac and the tool reports "nothing plugged
    in" while the hardware sits there working.

    On macOS use the /dev/cu.* names, never /dev/tty.*: opening a tty.* device
    blocks waiting for carrier detect, which a USB serial adapter never
    asserts, so the tool would hang instead of failing.
    """
    return sorted(
        glob.glob('/dev/ttyACM*') +          # Linux: Pixhawk USB CDC
        glob.glob('/dev/ttyUSB*') +          # Linux: SiK radio, ESP32
        glob.glob('/dev/cu.usbmodem*') +     # macOS: Pixhawk USB CDC
        glob.glob('/dev/cu.usbserial*') +    # macOS: FTDI-based SiK
        glob.glob('/dev/cu.SLAB_USBtoUART*'))  # macOS: CP210x-based SiK


def is_usb_cdc(port):
    """True for a directly-attached Pixhawk (115200), false for a radio."""
    return 'ACM' in port or 'usbmodem' in port


def require_port(conn):
    """Fail with something actionable when the device node is not there.

    pymavlink's own failure is a two-screen traceback ending in ENOENT, which
    buries the only useful question: which serial devices DO exist right now?
    Ports move constantly here (Pixhawk USB is a ttyACM, the SiK radio and
    the ESP32 both want ttyUSB0, and whichever was plugged in first wins).
    """
    if conn.startswith(('tcp:', 'udp:', 'tcpin:')) or os.path.exists(conn):
        return
    found = serial_candidates()
    msg = f"{conn} does not exist. "
    if found:
        msg += ("Serial devices present right now: " + ", ".join(found) +
                ". Pass one with --conn (and --baud 57600 for the SiK "
                "radio, 115200 for the Pixhawk's USB).")
    else:
        msg += ("NO serial devices at all: nothing is plugged in, or the "
                "aircraft/radio is unpowered.")
    sys.exit(msg)


def resolve_link(conn, baud):
    """Work out which port and baud to use, and say so out loud.

    Ports move constantly on this bench, and differ by OS: on Linux the
    Pixhawk's USB is a ttyACM while the SiK radio and the ESP32 both land on
    ttyUSB with whichever was plugged in first taking the lower number; on
    macOS they are /dev/cu.usbmodem* and /dev/cu.usbserial* (or SLAB_*).
    Hard-coding a default just produces a traceback on the wrong machine, so
    when the caller does not name a port we pick the only candidate if there
    is exactly one, and refuse to guess when there is more than one.

    Baud follows the port type unless the caller asked for a specific rate:
    115200 for a directly-attached Pixhawk, 57600 for a SiK ground radio.
    """
    if conn is not None:
        require_port(conn)
        return conn, baud if baud else (115200 if is_usb_cdc(conn) else 57600)
    found = serial_candidates()
    if not found:
        sys.exit("no serial devices found: nothing plugged in, or the "
                 "aircraft/radio is unpowered.")
    if len(found) > 1:
        sys.exit("several serial devices present (" + ", ".join(found) +
                 "); name one with --conn, since guessing between a radio "
                 "and something else would be a coin flip.")
    port = found[0]
    rate = baud if baud else (115200 if is_usb_cdc(port) else 57600)
    print(f"using the only serial device present: {port} at {rate}")
    return port, rate


def wait_autopilot(m, timeout=30):
    """Lock onto the AUTOPILOT's heartbeat, not whatever heartbeat lands first.

    mavutil.wait_heartbeat() returns the first HEARTBEAT of ANY kind, and this
    bus has two senders: the flight controller (sys 1 comp 1) and the ESP32
    (sys 1 comp 195). When the ESP32's arrives first, wait_heartbeat returns
    it, but pymavlink refuses to lock its sysid onto it (correctly: the ESP32
    declares MAV_TYPE_ONBOARD_CONTROLLER + MAV_AUTOPILOT_INVALID, both of
    which probably_vehicle_heartbeat() rejects). target_system is then left at
    0 = BROADCAST, so every command goes out addressed to nobody in
    particular and the acks do not come back reliably. That is exactly the
    "no ack" seen on 2026-08-02, on the runs whose banner said "system 0".

    So: keep reading until an actual autopilot heartbeat shows up, then pin
    the target explicitly. Same filter mavlink_io.py already uses.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb is None:
            continue
        if (hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                and hb.get_srcComponent()
                == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
            m.target_system = hb.get_srcSystem()
            m.target_component = hb.get_srcComponent()
            return True
    return False


def connect(conn=None, baud=None, quiet=False):
    """Resolve the link, open it, and lock onto the autopilot. Exits with a
    plain-language reason rather than a traceback if any step fails.

    Returns (connection, port, baud) because callers need the baud afterwards:
    stream rates and ack timeouts both depend on whether this is USB or a
    radio, and re-deriving it from the port name in three places is how they
    drift apart.
    """
    port, rate = resolve_link(conn, baud)
    if not quiet:
        print(f"connecting {port} at {rate} ...")
    m = mavutil.mavlink_connection(port, baud=rate, source_system=250)
    if not wait_autopilot(m):
        sys.exit("no autopilot heartbeat. Is the board powered, the radio "
                 "paired, and QGC CLOSED? QGC owns the serial port while it "
                 "is connected.")
    if not quiet:
        print(f"autopilot is system {m.target_system} component "
              f"{m.target_component}")
    return m, port, rate


def request_streams(m, baud):
    """Ask for telemetry at a rate the link can actually carry.

    A SiK link is far slower than its 57600 serial port suggests. Requesting
    the usual rates saturates it, and the resulting gaps look exactly like
    hardware faults to every check in this repo.
    """
    rate = 2 if baud <= 57600 else 4
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                   rate, 1)


def drain_statustext(m):
    """Any STATUSTEXT waiting right now. ArduPilot explains a refused motor
    test here ("Motor Test: Safety switch", "Motor Test: RC not calibrated"),
    and the ack alone is just MAV_RESULT_FAILED with no reason."""
    out = []
    while True:
        s = m.recv_match(type='STATUSTEXT', blocking=False)
        if s is None:
            return out
        out.append(s.text)


def send_and_ack(m, cmd, *params, timeout=5.0):
    """Send a COMMAND_LONG and wait for ITS ack.

    Drains the receive backlog first: this link streams everything at 4 Hz and
    a command sent on top of an unread pile means the ack is behind seconds of
    stale telemetry. Matching on ack.command matters too, because the FC also
    acks the stream requests this script makes.
    """
    while m.recv_match(blocking=False) is not None:
        pass
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
    deadline = time.time() + timeout
    while time.time() < deadline:
        ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack is not None and ack.command == cmd:
            return mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
    return 'NO ACK'


if __name__ == '__main__':
    sys.exit("mavlink_link.py is a library, not a tool. Use tools/bench.py "
             "for probes, tools/parameters.py for parameters, "
             "tools/wiring_check.py for the wiring verdict.")
