#!/usr/bin/env python3
"""Bench wiring check: one listen on the Pixhawk USB, one PASS/FAIL line per
wired subsystem (2026-08-02 port assignments).

    training/.venv/bin/python tools/wiring_check.py                 # over USB
    training/.venv/bin/python tools/wiring_check.py --wiggle        # + servo
    training/.venv/bin/python tools/wiring_check.py \
        --conn /dev/ttyUSB0 --baud 57600                            # over SiK

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
  GPS        SERIAL3 wiring: message flow + sat count (sats/fix will be poor
             indoors; wiring verdict is 'messages arrive', not 'fix')
  COMPASS    I2C splice: mag sensor present+enabled+healthy per SYS_STATUS
  TF-LUNA    SERIAL4 half of the split cable: downward DISTANCE_SENSOR flow
             (needs the swapped SERIAL4/5 + RNGFND1 params pushed first)
  ESP32      TELEM1: heartbeat from compid 195; plus OBSTACLE_DISTANCE ring
             and the upward DISTANCE_SENSOR when the sketch is in a
             transmitting mode (fake mode needs the GPIO4 jumper)
  RC         RCIN: RC_CHANNELS values look live (transmitter must be ON;
             0/65535 everywhere = no RX signal)
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
import time

from pymavlink import mavutil

DOWN = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270   # 25
UP = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_90      # 24

MAG_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
GPS_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS


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
    ap.add_argument('--conn', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
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
    args = ap.parse_args()

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
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)

    seen = {
        'fc_hb': 0, 'esp_hb': 0, 'obst': 0, 'rng_down': 0, 'rng_up': 0,
        'gps_msgs': 0, 'rc_msgs': 0, 'radio': 0,
    }
    rssi = remrssi = None
    sats = fix = -1
    rng_down_m = None
    rc_live = False
    mag_present = mag_healthy = False
    sys_status_seen = False

    def all_satisfied():
        """Everything a healthy aircraft must produce. Deliberately requires
        TWO heartbeats from each source: one proves the sender exists, two
        prove it is still sending. Nothing here waits for a GPS fix, which
        indoors would never come."""
        return (seen['fc_hb'] >= 2 and seen['esp_hb'] >= 2
                and seen['gps_msgs'] and seen['rng_down'] and seen['obst']
                and seen['rng_up'] and sys_status_seen and rc_live)

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
        elif t == 'DISTANCE_SENSOR':
            if msg.orientation == DOWN:
                seen['rng_down'] += 1
                rng_down_m = msg.current_distance / 100.0
            elif msg.orientation == UP:
                seen['rng_up'] += 1
        elif t == 'GPS_RAW_INT':
            seen['gps_msgs'] += 1
            sats, fix = msg.satellites_visible, msg.fix_type
        elif t == 'RC_CHANNELS':
            seen['rc_msgs'] += 1
            vals = [getattr(msg, f'chan{i}_raw') for i in range(1, 9)]
            if any(800 < v < 2200 for v in vals):
                rc_live = True
        elif t in ('RADIO_STATUS', 'RADIO'):
            # Injected by the SiK ground radio itself, so its presence proves
            # the whole radio path end to end. Only ever seen on a radio link.
            seen['radio'] += 1
            rssi, remrssi = msg.rssi, msg.remrssi
        elif t == 'SYS_STATUS':
            sys_status_seen = True
            mag_present = bool(msg.onboard_control_sensors_enabled & MAG_BIT)
            mag_healthy = bool(msg.onboard_control_sensors_health & MAG_BIT)

        if all_satisfied():
            print(f"  everything reporting after "
                  f"{time.time() - t_start:.1f}s, no need to keep listening")
            break

    def verdict(name, ok, detail):
        print(f"  {'PASS' if ok else 'FAIL':4}  {name:<10} {detail}")
        return ok

    print("\nresults:")
    ok = True
    ok &= verdict('FC', seen['fc_hb'] > 0,
                  f"{seen['fc_hb']} heartbeats")
    ok &= verdict('GPS', seen['gps_msgs'] > 0,
                  f"{seen['gps_msgs']} msgs, fix_type {fix}, {sats} sats "
                  f"(sats/fix poor indoors is normal; msgs=0 is a wire)")
    ok &= verdict('COMPASS', mag_present and mag_healthy,
                  "mag enabled+healthy" if sys_status_seen
                  else "no SYS_STATUS received")
    ok &= verdict('TF-LUNA', seen['rng_down'] > 0,
                  f"{seen['rng_down']} downward DISTANCE_SENSOR msgs"
                  + (f", {rng_down_m:.2f} m" if rng_down_m is not None
                     else "") + " (0 = wire OR params not pushed)")
    ok &= verdict('ESP32', seen['esp_hb'] > 0 and seen['obst'] > 0,
                  f"{seen['esp_hb']} comp195 heartbeats, "
                  f"{seen['obst']} OBSTACLE_DISTANCE (the 6-sensor ring). "
                  f"Heartbeats but 0 ring msgs = alive and not transmitting: "
                  f"in fake mode that means the GPIO4 jumper is missing")
    ok &= verdict('UP-SENSOR', seen['rng_up'] > 0,
                  f"{seen['rng_up']} upward DISTANCE_SENSOR msgs (mux ch6). "
                  f"0 with the sensor wired = ch6 init failed; the ESP32 "
                  f"serial monitor prints per-sensor init")
    ok &= verdict('RC', rc_live,
                  f"{seen['rc_msgs']} RC_CHANNELS msgs, "
                  + ("live values" if rc_live else
                     "no live values (is the transmitter on?)"))
    if seen['radio']:
        ok &= verdict('SiK', True,
                      f"{seen['radio']} RADIO_STATUS msgs, local rssi {rssi} "
                      f"remote {remrssi} (this whole run came over the radio, "
                      f"which IS the SiK test; higher rssi is better, and the "
                      f"two ends should be within ~20 of each other)")
    else:
        print("  ----  SiK        not exercised: this run was not over the "
              "radio. Re-run with --conn /dev/ttyUSB0 --baud 57600")
    print("  ----  BUZZ/SW    audible/visible only")
    if not args.motor_test:
        print("  ----  MOTORS     not tested; --motor-test spins them "
              "(PROPS OFF) or use QGC's motor test")

    if args.wiggle is not None:
        print(f"\nservo wiggle on ch{args.wiggle} (props off, watch the "
              f"gate): open 1900 ...")
        for us in (1900, 1000):
            res = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                               args.wiggle, us)
            print(f"  {us}us -> {res}")
            time.sleep(1.5)
        print("  ack proves acceptance; movement is verified by eye. No "
              "movement + ACCEPTED = SERVOn_FUNCTION not 0, or wiring/power.")

    if args.motor_test:
        motor_test(m, args.motors, args.motor_throttle, rc_live)

    print(f"\n{'ALL WIRED CHECKS PASS' if ok else 'SOMETHING FAILED, see above'}")
    raise SystemExit(0 if ok else 1)


def motor_test(m, motors, throttle_pct, rc_live):
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
            mavutil.mavlink.MOTOR_TEST_ORDER_DEFAULT)
        print(f"  motor {i}: {res}  <- which arm spun? note it")
        time.sleep(3)
    print("  expected: 1 = front-right, then clockwise. Any other order is a "
          "swapped ESC lead. Directions must alternate; fix by swapping any "
          "two motor phase wires, never by reordering the signal leads.")


if __name__ == '__main__':
    main()
