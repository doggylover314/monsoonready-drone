#!/usr/bin/env python3
"""Bench wiring check: one listen on the Pixhawk USB, one PASS/FAIL line per
wired subsystem (2026-08-02 port assignments).

    training/.venv/bin/python tools/wiring_check.py            # auto-detect
    training/.venv/bin/python tools/wiring_check.py --wiggle   # + servo
    training/.venv/bin/python tools/wiring_check.py --conn /dev/ttyUSB1

Port and baud are worked out when exactly one serial device is present: a
ttyUSB is taken as the SiK radio (57600), a ttyACM as the Pixhawk USB
(115200). With several present it refuses to guess.

Pixhawk on USB, battery or USB power, PROPS OFF. Listens for --seconds
(default 25) and then judges.

OVER THE SiK RADIO instead of USB: the aircraft runs on battery, the ground
radio is the PC's serial port (--conn /dev/ttyUSB0, --baud 57600 = the
radio's PC-side rate, unrelated to SERIAL2_BAUD on the aircraft). Everything
still works, and the run additionally PROVES the SiK link, which USB never
can. The stream rate is halved automatically on a low-baud link because 4 Hz
of everything does not fit through the air link, and the losses would print
as spurious FAILs. Param pushes are still best done on USB: they are many
small round trips and each retry costs radio time.

What each check proves, and what it cannot:

  FC         heartbeat from the autopilot at all
  GPS        SERIAL3 wiring: the autopilot's GPS-driver-present bit, since
             GPS_RAW_INT keeps flowing with no receiver attached. Sat count
             is reported but not required (0 sats indoors is normal)
  COMPASS    I2C splice: mag sensor present+enabled+healthy per SYS_STATUS
  TF-LUNA    SERIAL4 half of the split cable: a downward DISTANCE_SENSOR
             reading INSIDE the sensor's own min/max, because ArduPilot keeps
             publishing the message with a 0/out-of-range value when the
             sensor is disconnected
  ESP32      TELEM1: heartbeat from compid 195; plus OBSTACLE_DISTANCE ring
             and the upward DISTANCE_SENSOR when the sketch is in a
             transmitting mode (fake mode needs the GPIO4 jumper)
  RC         RCIN: the autopilot's RC-receiver HEALTH bit, which it clears
             on link loss or failsafe. NOT the channel values: a receiver in
             failsafe emits perfectly in-range numbers, which is why this
             check passed with the transmitter off until 2026-08-08
  UP-SENSOR  the 7th VL53L0X on mux ch6: upward DISTANCE_SENSOR flow
             (needs RNGFND2_TYPE=10 pushed)
  SERVO      only with --wiggle (bare flag = ch9 = AUX1): DO_SET_SERVO
             open/close cycle. Needs SERVOn_FUNCTION=0 pushed. WATCH THE GATE:
             an ack proves the command was accepted, only your eyes prove the
             servo moved.
  MOTORS     only with --motor-test, PROPS OFF: spins each motor in turn to
             prove ESC wiring, motor order and direction (all verified by eye)

  NOT CHECKABLE FROM USB: SiK radio on TELEM2 (verify separately: QGC over
  the radio link with USB unplugged), buzzer/switch (audible/visible), OLED
  (look at it), UNO Q SERIAL5 half (blocked on TODO 7 anyway).

Uses only pymavlink (same venv as the other tools). Read-only except the
explicit --wiggle.
"""

import argparse
import glob
import os
import sys
import time

from pymavlink import mavutil

DOWN = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270   # 25
UP = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_90      # 24

MAG_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
GPS_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
# The autopilot's own opinion of the RC link. This is the ONLY trustworthy
# "is the transmitter on" signal: see the RC check below for why the channel
# values are not.
RC_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER

# Must match the ESP32: mavlink_proximity.h SECTOR_NO_DATA, config.h ring size.
SECTOR_NO_DATA = 65535
RING_SECTORS = 6
# Bearing of each ring sector, for naming the dead one in plain language
# (config.h: bearing = SENSOR_ANGLE_OFFSET_DEG + 60*s, clockwise from nose).
# Offset is 0 on this airframe: the ring sits BETWEEN the arms with sensor 0
# facing straight out the nose (measured 2026-08-06). Keep in step with
# config.h SENSOR_ANGLE_OFFSET_DEG.
RING_ANGLE_OFFSET_DEG = 0
SECTOR_BEARING = [RING_ANGLE_OFFSET_DEG + 60 * s for s in range(RING_SECTORS)]


def serial_candidates():
    """Serial devices that could plausibly be the Pixhawk or a SiK radio.

    Linux and macOS name these completely differently, and this repo is used
    from both (the owner's Linux laptop and Raghav's MacBook), so a
    Linux-only glob silently finds nothing on a Mac and the tool reports
    "nothing plugged in" while the hardware sits there working.

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default=None,
                    help='serial device; omit to auto-pick when '
                         'exactly one is present')
    ap.add_argument('--baud', type=int, default=None,
                    help='omit to follow the port type: 115200 '
                         'for the Pixhawk USB, 57600 for a SiK radio')
    ap.add_argument('--seconds', type=float, default=12.0,
                    help='MAXIMUM listen time. The loop exits as soon as '
                         'every check has what it needs, so a fully healthy '
                         'aircraft finishes in a few seconds; the full '
                         'window is only ever spent waiting for something '
                         'that never arrives.')
    ap.add_argument('--wiggle', type=int, nargs='?', const=9, default=None,
                    metavar='CH',
                    help='cycle the dropper servo; bare flag = ch9 (AUX1, as '
                         'wired). Props off. Watch the gate.')
    ap.add_argument('--motor-test', action='store_true',
                    help='PROPS OFF. Spin each of the 6 motors in turn at '
                         'low throttle to prove ESC wiring and motor order.')
    ap.add_argument('--motor-throttle', type=int, default=8,
                    help='percent throttle for --motor-test (default 8)')
    ap.add_argument('--motors', type=int, default=6)
    ap.add_argument('--expect-esp32', action='store_true',
                    help='require the obstacle ring to be present and '
                         'reporting. OFF by default because the ring was '
                         'parked on 2026-08-06 and the ESP32 is unplugged; '
                         'without this flag its absence is reported but does '
                         'not fail the run. Turn it on if the ring is '
                         'refitted, so a silent ring is a failure again.')
    args = ap.parse_args()
    args.conn, args.baud = resolve_link(args.conn, args.baud)

    print(f"connecting {args.conn} ...")
    m = mavutil.mavlink_connection(args.conn, baud=args.baud,
                                   source_system=250)
    if not wait_autopilot(m):
        raise SystemExit("FAIL: no autopilot heartbeat on USB")
    print(f"autopilot is system {m.target_system} component "
          f"{m.target_component}\nlistening (up to {args.seconds:.0f}s, "
          f"exits early once everything has reported) ...")
    # Ask for everything at a modest rate; ArduPilot honours this legacy
    # request and it is one call instead of one per message id. Over a SiK
    # radio 4 Hz of everything does not fit (the air link is far slower than
    # its 57600 serial port), and the dropped messages would show up as
    # spurious FAILs, so slow the stream down on any low-baud link.
    rate = 4 if args.baud > 57600 else 2
    # A radio link adds latency and drops packets, so an ack that would
    # arrive comfortably over USB can miss a 5s window and print a false
    # "NO ACK" on a command that actually worked. Be more patient on a slow
    # link rather than teaching the operator to ignore the ack line.
    ack_timeout = 5.0 if args.baud > 57600 else 12.0
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)

    seen = {
        'fc_hb': 0, 'esp_hb': 0, 'obst': 0, 'rng_down': 0, 'rng_up': 0,
        'gps_msgs': 0, 'rc_msgs': 0, 'radio': 0,
    }
    rssi = remrssi = None
    ring_ok = set()            # ring sectors that ever reported real data
    sats = fix = -1
    rng_down_m = None
    rng_bounds = None
    rc_frames = []
    rc_present = rc_healthy = False
    gps_present = False
    mag_present = mag_healthy = False
    sys_status_seen = False

    def all_satisfied():
        """Everything a healthy aircraft must produce. Deliberately requires
        TWO heartbeats from each source: one proves the sender exists, two
        prove it is still sending. Nothing here waits for a GPS fix, which
        indoors would never come."""
        core = (seen['fc_hb'] >= 2 and seen['gps_msgs'] and gps_present
                and seen['rng_down'] and sys_status_seen and rc_healthy)
        if not args.expect_esp32:
            return core
        return (core and seen['esp_hb'] >= 2 and seen['obst']
                and len(ring_ok) == RING_SECTORS and seen['rng_up'])

    t_start = time.time()
    t_end = t_start + args.seconds
    while time.time() < t_end:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'HEARTBEAT':
            if msg.get_srcComponent() == 191:
                continue                      # a running mission process
            if msg.get_srcComponent() == 195:
                seen['esp_hb'] += 1
            elif msg.get_srcSystem() == m.target_system:
                seen['fc_hb'] += 1
        elif t == 'OBSTACLE_DISTANCE':
            seen['obst'] += 1
            # The message arriving proves the ESP32 is transmitting, NOT that
            # every sensor works: a dead ring sensor still occupies its slot,
            # filled with SECTOR_NO_DATA. Checking only that the message
            # exists hid a dead ch1 for days (2026-08-06), so score sectors.
            for s in range(RING_SECTORS):
                if msg.distances[s] != SECTOR_NO_DATA:
                    ring_ok.add(s)
        elif t == 'DISTANCE_SENSOR':
            if msg.orientation == DOWN:
                seen['rng_down'] += 1
                rng_down_m = msg.current_distance / 100.0
                # Keep the sensor's own declared bounds so the verdict can
                # tell a real return from the 0 / out-of-range value a
                # disconnected serial rangefinder reports while ArduPilot
                # keeps dutifully publishing the message.
                rng_bounds = (msg.min_distance / 100.0,
                              msg.max_distance / 100.0)
            elif msg.orientation == UP:
                seen['rng_up'] += 1
        elif t == 'GPS_RAW_INT':
            seen['gps_msgs'] += 1
            sats, fix = msg.satellites_visible, msg.fix_type
        elif t == 'RC_CHANNELS':
            seen['rc_msgs'] += 1
            # Deliberately NOT used to decide whether the link is alive: a
            # receiver in failsafe keeps emitting perfectly in-range values
            # (held last-known, or its programmed failsafe positions), so
            # "the numbers look plausible" passed with the transmitter
            # switched OFF (found 2026-08-08). Kept only to show movement.
            vals = [getattr(msg, f'chan{i}_raw') for i in range(1, 9)]
            rc_frames.append(tuple(vals))
        elif t in ('RADIO_STATUS', 'RADIO'):
            # Injected by the SiK ground radio itself, so its presence proves
            # the whole radio path end to end. Only ever seen on a radio link.
            seen['radio'] += 1
            rssi, remrssi = msg.rssi, msg.remrssi
        elif t == 'SYS_STATUS':
            sys_status_seen = True
            mag_present = bool(msg.onboard_control_sensors_enabled & MAG_BIT)
            mag_healthy = bool(msg.onboard_control_sensors_health & MAG_BIT)
            # The autopilot's own RC verdict. ArduPilot clears this health bit
            # when the RC link is lost or in failsafe, which is exactly the
            # condition the channel values fail to show.
            rc_present = bool(msg.onboard_control_sensors_enabled & RC_BIT)
            rc_healthy = bool(msg.onboard_control_sensors_health & RC_BIT)
            gps_present = bool(msg.onboard_control_sensors_enabled & GPS_BIT)

        if all_satisfied():
            print(f"  everything reporting after "
                  f"{time.time() - t_start:.1f}s, no need to keep listening")
            break

    def verdict(name, ok, detail):
        print(f"  {'PASS' if ok else 'FAIL':4}  {name:<10} {detail}")
        return ok

    print("\nresults:")
    ok = True
    gps_word = 'up' if gps_present else 'NOT PRESENT (receiver not detected)'
    ok &= verdict('FC', seen['fc_hb'] > 0,
                  f"{seen['fc_hb']} heartbeats")
    # GPS_RAW_INT keeps flowing with NO receiver attached (fix 0, sats 0), so
    # counting messages proves only that the autopilot is talking to us. The
    # enabled bit is the autopilot saying a GPS driver actually came up.
    ok &= verdict('GPS', seen['gps_msgs'] > 0 and gps_present,
                  f"{seen['gps_msgs']} msgs, driver {gps_word}, "
                  f"fix_type {fix}, {sats} sats "
                  f"(0 sats indoors is normal, a missing driver is not)")
    ok &= verdict('COMPASS', mag_present and mag_healthy,
                  "mag enabled+healthy" if sys_status_seen
                  else "no SYS_STATUS received")
    rng_real = (rng_down_m is not None and rng_bounds is not None
                and rng_bounds[0] <= rng_down_m <= rng_bounds[1])
    ok &= verdict('TF-LUNA', seen['rng_down'] > 0 and rng_real,
                  f"{seen['rng_down']} downward DISTANCE_SENSOR msgs"
                  + (f", {rng_down_m:.2f} m" if rng_down_m is not None
                     else "") + " (0 = wire OR params not pushed)")
    if not args.expect_esp32 and seen['esp_hb'] == 0:
        print("  ----  ESP32      absent, and not expected: the obstacle ring "
              "is parked (2026-08-06). Pass --expect-esp32 once it is "
              "refitted so a silent ring fails again.")
    else:
        ok &= verdict('ESP32', seen['esp_hb'] > 0 and seen['obst'] > 0,
                      f"{seen['esp_hb']} comp195 heartbeats, "
                      f"{seen['obst']} OBSTACLE_DISTANCE. Heartbeats but 0 "
                      f"ring msgs = alive and not transmitting: in fake mode "
                      f"that means the GPIO4 jumper is missing")
        dead = [s for s in range(RING_SECTORS) if s not in ring_ok]
        ok &= verdict('RING', seen['obst'] > 0 and not dead,
                      f"{len(ring_ok)}/{RING_SECTORS} sectors reporting"
                      + ("" if not dead else
                         ": DEAD " + ", ".join(f"s{s}({SECTOR_BEARING[s]}deg)"
                                               for s in dead)
                         + ". The ring message still streams with a dead "
                           "sensor's slot filled as no-data, so this is "
                           "invisible unless the sectors are scored"))
        ok &= verdict('UP-SENSOR', seen['rng_up'] > 0,
                      f"{seen['rng_up']} upward DISTANCE_SENSOR msgs (mux "
                      f"ch6). An empty ceiling is NOT the explanation for 0: "
                      f"a clear reading is still transmitted (as max+1). 0 "
                      f"means the read failed, so read the ESP32 boot lines")
    moving = len(set(rc_frames)) > 1
    health_word = 'healthy' if rc_healthy else \
        'UNHEALTHY (link lost or in failsafe)'
    ok &= verdict('RC', rc_healthy,
                  f"{seen['rc_msgs']} RC_CHANNELS msgs; autopilot reports "
                  f"receiver {'present' if rc_present else 'ABSENT'}/"
                  f"{health_word}"
                  + (", channels moving" if moving else
                     ", channels static (either you did not touch the sticks "
                     "or the receiver is holding failsafe values)"))
    # Is this run coming over the air? RADIO_STATUS is the nice proof, but a
    # SiK only injects it when its MAVLink framing mode is on, so its absence
    # proves nothing. The link itself is the better evidence: autopilot
    # telemetry cannot arrive on a non-USB serial port unless it crossed the
    # radio (the Pixhawk's own USB is a ttyACM, and the ESP32's USB port
    # carries no autopilot traffic at all).
    via_radio = ('ACM' not in args.conn
                 and not args.conn.startswith(('tcp:', 'udp:', 'tcpin:')))
    if via_radio:
        detail = (f"autopilot telemetry arrived over {args.conn}, so the "
                  f"whole radio path works")
        if seen['radio']:
            detail += (f"; {seen['radio']} RADIO_STATUS msgs, local rssi "
                       f"{rssi} remote {remrssi} (higher is better, and the "
                       f"two ends should be within ~20 of each other)")
        else:
            detail += ("; no RADIO_STATUS injected, which just means the "
                       "radio's MAVLink framing mode is off, so no rssi "
                       "figures are available")
        ok &= verdict('SiK', True, detail)
    else:
        print("  ----  SiK        not exercised: this run was on USB. Re-run "
              "with --conn /dev/ttyUSB0 --baud 57600")
    print("  ----  BUZZ/SW    audible/visible only")
    if not args.motor_test:
        print("  ----  MOTORS     not tested; --motor-test spins them "
              "(PROPS OFF) or use QGC's motor test")

    if args.wiggle is not None:
        print(f"\nservo wiggle on ch{args.wiggle} (props off, watch the "
              f"gate): open 1900 ...")
        for us in (1900, 1000):
            res = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                               args.wiggle, us, timeout=ack_timeout)
            print(f"  {us}us -> {res}")
            time.sleep(1.5)
        print("  ack proves acceptance; movement is verified by eye. No "
              "movement + ACCEPTED = SERVOn_FUNCTION not 0, or wiring/power.")

    if args.motor_test:
        motor_test(m, args.motors, args.motor_throttle, rc_healthy,
                   ack_timeout)

    print(f"\n{'ALL WIRED CHECKS PASS' if ok else 'SOMETHING FAILED, see above'}")
    raise SystemExit(0 if ok else 1)


def motor_test(m, motors, throttle_pct, rc_live, ack_timeout=5.0):
    """Spin each motor briefly, in ArduPilot's TEST ORDER, one at a time.

    TRANSMITTER MUST BE ON. Observed 2026-08-02: identical commands at the
    same 8% throttle did nothing with the TX off and spun the motors with it
    on, so ArduPilot is gating the motor test on live RC input. (The exact
    check in the firmware has not been read; the behaviour is empirical, but
    it reproduced three times.) The check below refuses to send rather than
    let a dead-quiet run look like broken ESCs.

    MAV_CMD_DO_MOTOR_TEST numbers motors in ArduPilot's test sequence, which
    for a hexa X goes clockwise from the front-right. That is the point of the
    test: motor 1 must be the front-right arm, and each subsequent number the
    next one clockwise. A motor that spins out of sequence is a swapped ESC
    signal lead, which flies exactly once.

    Direction is checked by eye at the same time (alternating CW/CCW per
    ArduPilot's hexa layout). Neither can be checked in software: this only
    commands the spin, your eyes do the verifying.
    """
    if not rc_live:
        print("\nMOTOR TEST SKIPPED: no live RC. Switch the transmitter on "
              "and re-run; without it the commands are refused and every "
              "motor looks dead.")
        return
    print(f"\nMOTOR TEST: {motors} motors, {throttle_pct}% throttle, 2s each.")
    print("PROPS MUST BE OFF. Type 'spin' to continue, anything else aborts.")
    try:
        if input("> ").strip().lower() != 'spin':
            print("  aborted, nothing commanded")
            return
    except EOFError:
        print("  no console input available, aborted")
        return
    for i in range(1, motors + 1):
        res = send_and_ack(
            m, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            i,                                             # motor number
            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,   # throttle type
            throttle_pct, 2,                               # value, seconds
            0,                                             # motor count
            mavutil.mavlink.MOTOR_TEST_ORDER_DEFAULT,
            timeout=ack_timeout)
        print(f"  motor {i}: {res}  <- which arm spun? note it")
        time.sleep(3)
    print("  expected: 1 = front-right, then clockwise. Any other order is a "
          "swapped ESC lead. Directions must alternate; fix by swapping any "
          "two motor phase wires, never by reordering the signal leads.")


if __name__ == '__main__':
    main()
