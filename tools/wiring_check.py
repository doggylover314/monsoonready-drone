#!/usr/bin/env python3
"""Bench wiring check: one listen on the Pixhawk USB, one PASS/FAIL line per
wired subsystem (2026-08-02 port assignments).

    training/.venv/bin/python tools/wiring_check.py                # laptop
    training/.venv/bin/python tools/wiring_check.py --wiggle 14    # + servo test

Pixhawk on USB, battery or USB power, PROPS OFF. Listens for --seconds
(default 25) and then judges. What each check proves, and what it cannot:

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
  SERVO      only with --wiggle N: DO_SET_SERVO open/close cycle. Needs
             SERVOn_FUNCTION=0 pushed. WATCH THE GATE: an ack here proves the
             command was accepted, only your eyes prove the servo moved.

  NOT CHECKABLE FROM USB: SiK radio on TELEM2 (verify separately: QGC over
  the radio link with USB unplugged), buzzer/switch (audible/visible),
  ESCs/motors (QGC motor test, props off, calibration step), UNO Q SERIAL5
  half (blocked on TODO 7 anyway).

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=25.0)
    ap.add_argument('--wiggle', type=int, default=None, metavar='CH',
                    help='cycle the dropper servo on this output channel '
                         '(e.g. 14 = AUX6). Props off. Watch the gate.')
    args = ap.parse_args()

    print(f"connecting {args.conn} ...")
    m = mavutil.mavlink_connection(args.conn, baud=args.baud,
                                   source_system=250)
    hb = m.wait_heartbeat(timeout=30)
    if hb is None:
        raise SystemExit("FAIL: no heartbeat on USB at all")
    print(f"heartbeat from system {m.target_system}\n"
          f"listening {args.seconds:.0f}s ...")
    # Ask for everything at a modest rate; ArduPilot honours this legacy
    # request and it is one call instead of one per message id.
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    seen = {
        'fc_hb': 0, 'esp_hb': 0, 'obst': 0, 'rng_down': 0, 'rng_up': 0,
        'gps_msgs': 0, 'rc_msgs': 0,
    }
    sats = fix = -1
    rng_down_m = None
    rc_live = False
    mag_present = mag_healthy = False
    sys_status_seen = False

    t_end = time.time() + args.seconds
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
        elif t == 'SYS_STATUS':
            sys_status_seen = True
            mag_present = bool(msg.onboard_control_sensors_enabled & MAG_BIT)
            mag_healthy = bool(msg.onboard_control_sensors_health & MAG_BIT)

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
    ok &= verdict('ESP32', seen['esp_hb'] > 0,
                  f"{seen['esp_hb']} comp195 heartbeats, "
                  f"{seen['obst']} OBSTACLE_DISTANCE, "
                  f"{seen['rng_up']} upward rangefinder "
                  f"(hb only = alive but not transmitting: fake mode "
                  f"without GPIO4 jumper)")
    ok &= verdict('RC', rc_live,
                  f"{seen['rc_msgs']} RC_CHANNELS msgs, "
                  + ("live values" if rc_live else
                     "no live values (is the transmitter on?)"))
    print("  ----  SiK        not checkable from USB: connect QGC over the "
          "radio with USB unplugged")
    print("  ----  BUZZ/SW    audible/visible only")
    print("  ----  MOTORS     QGC motor test, props off (calibration step)")

    if args.wiggle is not None:
        print(f"\nservo wiggle on ch{args.wiggle} (props off, watch the "
              f"gate): open 1900 ...")
        for us in (1900, 1000):
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                args.wiggle, us, 0, 0, 0, 0, 0)
            ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            res = 'no ack' if ack is None else \
                mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
            print(f"  {us}us -> {res}")
            time.sleep(1.5)
        print("  ack proves acceptance; movement is verified by eye. No "
              "movement + ACCEPTED = SERVOn_FUNCTION not 0, or wiring/power.")

    print(f"\n{'ALL WIRED CHECKS PASS' if ok else 'SOMETHING FAILED, see above'}")
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
