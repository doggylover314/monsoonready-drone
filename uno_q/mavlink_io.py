"""MAVLink link layer for the UNO Q mission computer.

Single-threaded by design: one pump (step()) receives everything and updates
the telemetry cache; commands that need an ACK pump the same loop while they
wait. No locks, no races, and identical behavior on the laptop (SITL over TCP)
and on the UNO Q (SERIAL4 via the STM32 byte-shovel, 115200).

We are component 191 (MAV_COMP_ID_ONBOARD_COMPUTER) on the vehicle's system id.
"""

import time

from pymavlink import mavutil

# ArduPilot rejects guided setpoints older than a few seconds; resend at 5Hz.
SETPOINT_RESEND_S = 0.2

# Pulse widths outside this are refused. Brackets the gate's MEASURED travel
# (closed 560us, open 1760us, servo_jog by eye 2026-08-14) with margin on both
# sides, narrow enough to catch a typo before it reaches a servo. Shared with
# dropper.py so the construction-time check and the send-time check agree, and
# SERVO9_MIN/_MAX on the board must bracket the same range
# (param_dumps/pixhawk_full_setup.param sets 500/1800).
PWM_MIN_US = 500
PWM_MAX_US = 1800

# POSITION_TARGET_TYPEMASK: use position only / velocity only.
MASK_POSITION_ONLY = 0x0DF8  # ignore vel+accel+yaw+yaw_rate
MASK_VELOCITY_ONLY = 0x0DC7  # ignore pos+accel+yaw+yaw_rate


class Telemetry:
    """Latest-value cache; timestamps are time.monotonic() at receive."""

    def __init__(self):
        self.lat = None            # deg
        self.lon = None            # deg
        self.rel_alt_m = None      # above home
        self.heading_deg = None
        # Ground-frame velocity, NED, m/s. vd is positive DOWNWARD, so a
        # descent reads positive. mission.py uses it to confirm the aircraft
        # has actually stopped before the gate opens.
        self.vn_mps = None
        self.ve_mps = None
        self.vd_mps = None
        self.pos_t = 0.0
        self.rng_m = None          # downward rangefinder, meters
        self.rng_valid = False
        self.rng_t = 0.0
        self.mode = None           # ArduCopter mode name, e.g. 'GUIDED'
        self.armed = False
        self.heartbeat_t = 0.0


class MavIO:
    def __init__(self, conn_str, source_system=1, source_component=191,
                 baud=115200):
        # baud is ignored by tcp:/udp: connection strings and applies to the
        # serial device on the aircraft (SERIAL4/5 at 115200).
        self.conn = mavutil.mavlink_connection(
            conn_str, source_system=source_system,
            source_component=source_component, baud=baud)
        self.tel = Telemetry()
        self._mode_names = {}   # custom_mode -> name, filled after heartbeat
        self._last_setpoint_t = 0.0

    # ---------- connection ----------

    def wait_ready(self, timeout=60):
        """Heartbeat + mode map. Raises on timeout."""
        hb = self.conn.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise TimeoutError("no heartbeat from flight controller")
        self._mode_names = {v: k for k, v in self.conn.mode_mapping().items()}
        self._on_msg(hb)

    def request_stream(self, msg_id, hz):
        """MAV_CMD_SET_MESSAGE_INTERVAL, verified by ACK."""
        self.command_ack(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            p1=msg_id, p2=1e6 / hz)

    def setup_streams(self):
        self.request_stream(
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
        self.request_stream(
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)

    # ---------- pump ----------

    def step(self, max_wait=0.02):
        """Receive+dispatch at most one message; call in a tight loop."""
        msg = self.conn.recv_match(blocking=True, timeout=max_wait)
        if msg is not None:
            self._on_msg(msg)
        return msg

    def _on_msg(self, msg):
        t = msg.get_type()
        now = time.monotonic()
        tel = self.tel
        if t == 'GLOBAL_POSITION_INT':
            tel.lat = msg.lat / 1e7
            tel.lon = msg.lon / 1e7
            tel.rel_alt_m = msg.relative_alt / 1000.0
            tel.heading_deg = msg.hdg / 100.0 if msg.hdg != 65535 else None
            tel.vn_mps = msg.vx / 100.0     # GLOBAL_POSITION_INT is cm/s
            tel.ve_mps = msg.vy / 100.0
            tel.vd_mps = msg.vz / 100.0
            tel.pos_t = now
        elif t == 'DISTANCE_SENSOR':
            # Only the downward-facing sensor (TF-Luna / SITL equivalent).
            if msg.orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270:
                tel.rng_m = msg.current_distance / 100.0
                tel.rng_valid = (msg.min_distance < msg.current_distance
                                 < msg.max_distance)
                tel.rng_t = now
        elif t == 'HEARTBEAT':
            # Ignore heartbeats from other components (e.g. ESP32 compid 195).
            if (msg.get_srcSystem() == self.conn.target_system
                    and msg.get_srcComponent()
                    == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                tel.mode = self._mode_names.get(msg.custom_mode, tel.mode)
                tel.armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                tel.heartbeat_t = now

    # ---------- commands ----------

    def command_ack(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                    timeout=3.0, retries=3, retry_failed=False):
        """command_long + wait for its COMMAND_ACK, pumping telemetry meanwhile.

        retry_failed: also retry on MAV_RESULT_FAILED (ArduPilot answers FAILED
        for arm attempts while prearm checks are still settling, e.g. EKF).
        """
        for _ in range(retries):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                cmd, 0, p1, p2, p3, p4, p5, p6, p7)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                msg = self.step()
                if (msg is None or msg.get_type() != 'COMMAND_ACK'
                        or msg.command != cmd):
                    continue
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
                if msg.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    continue  # final ack still coming
                if (msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
                        or (retry_failed and msg.result
                            == mavutil.mavlink.MAV_RESULT_FAILED)):
                    break  # retry outer loop
                raise RuntimeError(f"command {cmd} rejected: result={msg.result}")
            time.sleep(0.5)
        raise TimeoutError(f"command {cmd}: no ACCEPTED ack")

    def set_mode(self, name):
        mode_id = self.conn.mode_mapping()[name]
        self.command_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=mode_id)

    def arm(self):
        # Generous retries: covers EKF settle time after SITL boot.
        self.command_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1,
            timeout=5.0, retries=60, retry_failed=True)

    def takeoff(self, alt_m):
        self.command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt_m)

    def set_servo(self, channel, pwm_us):
        """MAV_CMD_DO_SET_SERVO: drive a Pixhawk output directly.

        channel is the SERVO output number (1-based, matching SERVOn_*
        params), pwm_us the pulse width in microseconds. The output's
        SERVOn_FUNCTION must be 0 (Disabled) for ArduPilot to hand manual
        control of it to this command.

        VERIFY ON THE BENCH: whether ArduPilot NACKs DO_SET_SERVO when
        SERVOn_FUNCTION is not 0 is not confirmed from a primary source. Do
        not rely on the ACK alone to prove the parameter is right; watch the
        gate move.
        """
        if not PWM_MIN_US <= pwm_us <= PWM_MAX_US:
            raise ValueError(f"pwm_us out of servo range: {pwm_us}")
        return self.command_ack(mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                p1=channel, p2=pwm_us)

    def goto(self, lat, lon, rel_alt_m):
        """Guided position target (global frame, alt relative to home)."""
        self.conn.mav.set_position_target_global_int_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MASK_POSITION_ONLY,
            int(lat * 1e7), int(lon * 1e7), rel_alt_m,
            0, 0, 0, 0, 0, 0, 0, 0)

    def velocity_ned(self, vn, ve, vd, force=False):
        """Guided velocity target; must be resent continuously (rate-limited).

        force=True bypasses the rate limiter. That exists for one specific
        case and it is a safety case: a CHANGE of setpoint must never be the
        message the limiter happens to swallow. Resending "keep descending"
        early is wasted bandwidth; dropping "stop descending" means the
        autopilot keeps acting on the previous command, and the caller
        carries on believing it has stopped (review finding 2026-08-01: the
        gate opened ~0.65-1 m below the intended release height because the
        zero-velocity setpoint before the drop was silently rate-limited
        away).
        """
        now = time.monotonic()
        if not force and now - self._last_setpoint_t < SETPOINT_RESEND_S:
            return
        self._last_setpoint_t = now
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MASK_VELOCITY_ONLY,
            0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)
