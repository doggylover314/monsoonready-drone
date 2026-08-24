#!/usr/bin/env python3
"""Bench wiring check: PASS/FAIL per subsystem via Pixhawk USB or SiK radio.

    ./python tools/wiring_check.py
    ./python tools/wiring_check.py --wiggle
    ./python tools/wiring_check.py --conn /dev/ttyUSB0 --baud 57600

Listens --seconds (default 12); exits when all checks satisfied. Requires two
heartbeats per source (first = existence, second = still-sending). Stream
rate: 4 Hz over USB, 2 Hz over SiK radio (auto). Read-only except --wiggle.
Uses pymavlink. FC/GPS/COMPASS/TF-LUNA/ESP32/RC/UP-SENSOR/SERVO/MOTORS checks.
Not checkable from USB: SiK TELEM2, buzzer, OLED, UNO Q SERIAL5.
"""

import argparse
import os
import sys
import time

from pymavlink import mavutil

# Single source of truth for gate pulses, shared with dropper.py
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'uno_q'))
from dropper import PixhawkServoDropper as _Gate

# Link handling in mavlink_link.py; this file is wiring verdict only.
from mavlink_link import (connect, drain_statustext,  # noqa: F401
                          require_port, resolve_link, send_and_ack,
                          serial_candidates, wait_autopilot)

DOWN = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270   # 25
UP = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_90      # 24

MAG_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
GPS_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
# Autopilot's RC-link verdict, trustworthy unlike channel values alone
RC_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER

# Must match ESP32 mavlink_proximity.h SECTOR_NO_DATA and config.h ring size
SECTOR_NO_DATA = 65535
RING_SECTORS = 6
# Bearing of each ring sector (SENSOR_ANGLE_OFFSET_DEG + 60*s, clockwise).
# Offset = 0 (ring between arms, sensor 0 faces nose). Keep in step with
# config.h SENSOR_ANGLE_OFFSET_DEG.
RING_ANGLE_OFFSET_DEG = 0
SECTOR_BEARING = [RING_ANGLE_OFFSET_DEG + 60 * s for s in range(RING_SECTORS)]


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
                    choices=range(1, 21), metavar='1-20',
                    help='percent throttle for --motor-test (default 8)')
    ap.add_argument('--motors', type=int, default=6)
    ap.add_argument('--servo-closed-us', type=int,
                    default=_Gate.DEFAULT_CLOSED_US,
                    help='gate CLOSED pulse (default from dropper.py)')
    ap.add_argument('--servo-open-us', type=int,
                    default=_Gate.DEFAULT_OPEN_US,
                    help='gate OPEN pulse (default from dropper.py). Open '
                         'below closed = counter-clockwise travel.')
    ap.add_argument('--expect-esp32', action='store_true',
                    help='require the obstacle ring to be present and '
                         'reporting. OFF by default because the ring was '
                         'parked on 2026-08-06 and the ESP32 is unplugged; '
                         'without this flag its absence is reported but does '
                         'not fail the run. Turn it on if the ring is '
                         'refitted, so a silent ring is a failure again.')
    args = ap.parse_args()
    if args.wiggle is not None and not 1 <= args.wiggle <= 16:
        sys.exit(f"--wiggle {args.wiggle} is not an output channel (1-16)")
    if args.wiggle not in (None, 9):
        print(f"NOTE: this project wires the dropper to ch9 (AUX1); you asked "
              f"for ch{args.wiggle}. A channel whose SERVOn_FUNCTION is not 0 "
              f"will ack ACCEPTED and move nothing.")
    # connect() is the shared path every other MAVLink tool uses: it
    # resolves the link, opens it, locks onto the autopilot and exits with a
    # readable reason instead of a traceback. This was the only outlier.
    m, args.conn, args.baud = connect(args.conn, args.baud)
    print(f"autopilot is system {m.target_system} component "
          f"{m.target_component}\nlistening (up to {args.seconds:.0f}s, "
          f"exits early once everything has reported) ...")
    # Low-baud links (SiK radio) can't handle 4 Hz of everything. Slow to
    # 2 Hz to avoid spurious FAILs from dropped messages.
    rate = 4 if args.baud > 57600 else 2
    # Radio link adds latency and drops packets. Long timeout prevents
    # false "NO ACK" for slow links.
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
    rc_ever_bad = mag_ever_bad = False
    gps_present = False
    mag_present = mag_healthy = False
    sys_status_seen = False

    def all_satisfied():
        """Healthy aircraft must produce all core messages. Requires two
        heartbeats per source: first proves sender exists, second proves
        still-sending. Does not wait for GPS fix (indoors won't arrive)."""
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
                continue                      # mission process, not autopilot
            if msg.get_srcComponent() == 195:
                seen['esp_hb'] += 1
            elif msg.get_srcSystem() == m.target_system:
                seen['fc_hb'] += 1
        elif t == 'OBSTACLE_DISTANCE':
            seen['obst'] += 1
            # Message proves ESP32 transmitting, not that all sensors work.
            # Score sectors, not just presence, to catch dead sensors.
            for s in range(RING_SECTORS):
                d = msg.distances[s]
                # 0 cm = failed I2C read, not actual obstacle. Mirror
                # TF-LUNA bounds check.
                if d != SECTOR_NO_DATA and 0 < d <= msg.max_distance:
                    ring_ok.add(s)
        elif t == 'DISTANCE_SENSOR':
            if msg.orientation == DOWN:
                seen['rng_down'] += 1
                rng_down_m = msg.current_distance / 100.0
                # Keep sensor's declared bounds to distinguish real returns
                # from the 0/out-of-range value a disconnected rangefinder
                # publishes while ArduPilot keeps dutifully streaming.
                rng_bounds = (msg.min_distance / 100.0,
                              msg.max_distance / 100.0)
            elif msg.orientation == UP:
                seen['rng_up'] += 1
        elif t == 'GPS_RAW_INT':
            seen['gps_msgs'] += 1
            sats, fix = msg.satellites_visible, msg.fix_type
        elif t == 'RC_CHANNELS':
            seen['rc_msgs'] += 1
            # Not used for link status: failsafe receivers emit valid values.
            # Kept only to show channel movement.
            vals = [getattr(msg, f'chan{i}_raw') for i in range(1, 9)]
            rc_frames.append(tuple(vals))
        elif t in ('RADIO_STATUS', 'RADIO'):
            # Injected by SiK radio only, proves end-to-end radio path.
            seen['radio'] += 1
            rssi, remrssi = msg.rssi, msg.remrssi
        elif t == 'SYS_STATUS':
            sys_status_seen = True
            mag_present = bool(msg.onboard_control_sensors_enabled & MAG_BIT)
            mag_healthy = bool(msg.onboard_control_sensors_health & MAG_BIT)
            # Autopilot's RC verdict: cleared on link loss or failsafe
            # (channel values won't show this).
            rc_present = bool(msg.onboard_control_sensors_enabled & RC_BIT)
            rc_healthy = bool(msg.onboard_control_sensors_health & RC_BIT)
            # Sticky: a safety property has to hold the whole window, not
            # just the final sample.
            if not rc_healthy:
                rc_ever_bad = True
            if not bool(msg.onboard_control_sensors_health & MAG_BIT):
                mag_ever_bad = True
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
    # Two heartbeats required: first proves existence, second proves
    # still-sending. Single beat then brown-out used to pass.
    ok &= verdict('FC', seen['fc_hb'] >= 2,
                  f"{seen['fc_hb']} heartbeats")
    # GPS_RAW_INT flows with no receiver attached. Message count proves only
    # autopilot talks to us; enabled bit proves GPS driver came up.
    ok &= verdict('GPS', seen['gps_msgs'] > 0 and gps_present,
                  f"{seen['gps_msgs']} msgs, driver {gps_word}, "
                  f"fix_type {fix}, {sats} sats "
                  f"(0 sats indoors is normal, a missing driver is not)")
    # Detail printed after verdict, not before: old code printed
    # "mag enabled+healthy" even on FAIL.
    if not sys_status_seen:
        mag_detail = "no SYS_STATUS received"
    elif mag_present and mag_healthy:
        mag_detail = "mag enabled+healthy"
    else:
        mag_detail = (f"enabled={mag_present} healthy={mag_healthy}"
                      + ("  (no compass detected, or COMPASS_USE=0)"
                         if not mag_present else
                         "  (detected but reporting unhealthy)"))
    ok &= verdict('COMPASS', mag_present and mag_healthy, mag_detail)
    # Disconnected rangefinder publishes 0.00. Healthy one on legs reads
    # below RNGFND1_MIN. Treating below-min as broken broke working Luna.
    rng_real = (rng_down_m is not None and rng_bounds is not None
                and 0.0 < rng_down_m <= rng_bounds[1])
    rng_low = rng_real and rng_down_m < rng_bounds[0]
    ok &= verdict('TF-LUNA', seen['rng_down'] > 0 and rng_real,
                  f"{seen['rng_down']} downward DISTANCE_SENSOR msgs"
                  + (f", {rng_down_m:.2f} m" if rng_down_m is not None
                     else "")
                  + ("  (below RNGFND1_MIN, which is expected with the "
                     "aircraft on its legs)" if rng_low else "")
                  + ("" if rng_real else
                     "  <- 0.00 or out of range is what ArduPilot publishes "
                     "when the serial rangefinder is disconnected"))
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
    ok &= verdict('RC', rc_healthy and not rc_ever_bad,
                  f"{seen['rc_msgs']} RC_CHANNELS msgs; autopilot reports "
                  f"receiver {'present' if rc_present else 'ABSENT'}/"
                  f"{health_word}"
                  + ("; DROPPED OUT at least once during the window, which "
                     "is an intermittent link, not a healthy one"
                     if rc_ever_bad else "")
                  + (", channels moving" if moving else
                     ", channels static (either you did not touch the sticks "
                     "or the receiver is holding failsafe values)"))
    # Over air link? RADIO_STATUS is nice proof but may be off. Better
    # evidence: autopilot telemetry on non-USB serial must cross radio
    # (Pixhawk USB = ttyACM, ESP32 USB carries no autopilot traffic).
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
        travel_deg = abs(args.servo_open_us - args.servo_closed_us) / \
            _Gate.US_PER_DEG
        way = 'counter-clockwise' if args.servo_open_us < args.servo_closed_us \
            else 'clockwise'
        print(f"\nservo wiggle on ch{args.wiggle} (props off, watch the gate): "
              f"closed {args.servo_closed_us}us -> open {args.servo_open_us}us"
              f", about {travel_deg:.0f} deg {way}")
        wiggle_ok = True
        try:
            for us in (args.servo_open_us, args.servo_closed_us):
                res = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                   args.wiggle, us, timeout=ack_timeout)
                print(f"  {us}us -> {res}")
                if res != 'MAV_RESULT_ACCEPTED':
                    wiggle_ok = False
                time.sleep(1.5)
        finally:
            # Gate must never be left open. Re-command closed unconditionally.
            closed = send_and_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                  args.wiggle, args.servo_closed_us,
                                  timeout=ack_timeout)
            print(f"  final close {args.servo_closed_us}us -> {closed}")
            if closed != 'MAV_RESULT_ACCEPTED':
                print("  COULD NOT CONFIRM THE GATE CLOSED. Check it by eye "
                      "before flying; a gate left open dumps the payload.")
                wiggle_ok = False
        ok &= wiggle_ok
        print("  ack proves acceptance; movement is verified by eye. No "
              "movement + ACCEPTED = SERVOn_FUNCTION not 0, or wiring/power.")

    if args.motor_test:
        ok &= motor_test(m, args.motors, args.motor_throttle,
                         rc_healthy and not rc_ever_bad, ack_timeout)

    print(f"\n{'ALL WIRED CHECKS PASS' if ok else 'SOMETHING FAILED, see above'}")
    raise SystemExit(0 if ok else 1)


def motor_test(m, motors, throttle_pct, rc_live, ack_timeout=5.0):
    """Spin each motor in ArduPilot's test order, one at a time.

    Transmitter must be on: ArduPilot gates motor test on live RC.
    Motors numbered clockwise from front-right. Out-of-sequence motor =
    swapped ESC signal lead. Direction alternates CW/CCW per hexa layout.
    Verify by eye: software only commands spin, eyes verify motion.
    """
    if not rc_live:
        print("\nMOTOR TEST SKIPPED: no live RC. Switch the transmitter on "
              "and re-run; without it the commands are refused and every "
              "motor looks dead.")
        return False
    print(f"\nMOTOR TEST: {motors} motors, {throttle_pct}% throttle, 2s each.")
    print("PROPS MUST BE OFF. Type 'spin' to continue, anything else aborts.")
    try:
        if input("> ").strip().lower() != 'spin':
            print("  aborted, nothing commanded")
            return False
    except EOFError:
        print("  no console input available, aborted")
        return False
    all_ok = True
    for i in range(1, motors + 1):
        res = send_and_ack(
            m, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            i,                                             # motor number
            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,   # throttle type
            throttle_pct, 2,                               # value, seconds
            0,                                             # motor count
            mavutil.mavlink.MOTOR_TEST_ORDER_DEFAULT,
            timeout=ack_timeout)
        if res == 'MAV_RESULT_ACCEPTED':
            print(f"  motor {i}: {res}  <- which arm spun? note it")
        else:
            print(f"  motor {i}: {res}   (not accepted, so nothing spun)")
            all_ok = False
        for line in drain_statustext(m):
            print(f"     FC says: {line}")
        time.sleep(3)
    print("  expected: 1 = front-right, then clockwise. Any other order is a "
          "swapped ESC lead. Directions must alternate; fix by swapping any "
          "two motor phase wires, never by reordering the signal leads.")
    return all_ok


if __name__ == '__main__':
    main()
