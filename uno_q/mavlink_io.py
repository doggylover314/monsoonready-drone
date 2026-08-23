"""MAVLink link layer for the UNO Q mission computer.

Single-threaded: step() pump receives and dispatches telemetry; commands
needing ACK pump the same loop. No locks. Identical behavior on laptop
(SITL over TCP) and UNO Q (Pixhawk USB via hub).

Component 191 (MAV_COMP_ID_ONBOARD_COMPUTER). Connection 'auto' resolves
/dev/serial/by-id stable Pixhawk entries.
"""

import glob
import time

from pymavlink import mavutil

# ArduPilot/Pixhawk USB CDC paths in /dev/serial/by-id.
# Example: usb-ArduPilot_Pixhawk1-bdshot_2D00...-if00
_PIXHAWK_ID_PATTERNS = ('*ArduPilot*', '*Pixhawk*', '*PX4*', '*fmu*')


def resolve_conn(conn):
    """Resolve 'auto' to Pixhawk /dev/serial/by-id path; pass through others."""
    if conn != 'auto':
        return conn
    hits = sorted({p for pat in _PIXHAWK_ID_PATTERNS
                   for p in glob.glob(f'/dev/serial/by-id/{pat}')
                   if p.endswith('-if00')})
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise RuntimeError(
            "conn 'auto': no autopilot on USB (/dev/serial/by-id has no "
            "ArduPilot/Pixhawk entry). CHECK THE PIXHAWK'S USB PLUG AND THE "
            "HUB, then ls -l /dev/serial/by-id/")
    raise RuntimeError(
        f"conn 'auto': {len(hits)} autopilot-like USB devices, refusing to "
        f"guess: {', '.join(hits)}. Pass --conn with the right one.")

# Guided setpoint resend rate: ArduPilot rejects stale commands (5 Hz).
SETPOINT_RESEND_S = 0.2

# Component 191 heartbeat rate (1 Hz, MAVLink convention).
HEARTBEAT_PERIOD_S = 1.0

# Gate travel: 560us closed, 1760us open. Bounded with margin to catch typos.
# Shared with dropper.py. SERVO9_MIN/_MAX on board: 500/1800.
PWM_MIN_US = 500
PWM_MAX_US = 1800

# Typemask constants: position-only and velocity-only setpoints.
MASK_POSITION_ONLY = 0x0DF8  # ignore vel, accel, yaw, yaw_rate
MASK_VELOCITY_ONLY = 0x0DC7  # ignore pos, accel, yaw, yaw_rate


# STATUSTEXT cache size and severity name mapping.
STATUSTEXT_KEEP = 40
SEVERITY_NAMES = {0: 'EMERGENCY', 1: 'ALERT', 2: 'CRITICAL', 3: 'ERROR',
                  4: 'WARNING', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'}

# SYS_STATUS sensor bits (verified against ArduCopter 4.7.0).
# Bit 28: PREARM_CHECK indicates prearm checks passing.
SENSOR_BITS = [
    (1 << 0, 'gyro'), (1 << 1, 'accel'), (1 << 2, 'compass'),
    (1 << 3, 'baro'), (1 << 5, 'GPS'), (1 << 8, 'rangefinder'),
    (1 << 16, 'RC receiver'), (1 << 20, 'geofence'), (1 << 21, 'AHRS'),
    (1 << 24, 'logging'), (1 << 25, 'battery'), (1 << 26, 'proximity'),
]
PREARM_OK_BIT = 1 << 28            # MAV_SYS_STATUS_PREARM_CHECK

# Rest-voltage to SoC for one LiPo cell (standard published approximation).
# Survives reboots; coulomb counter resets at boot. Approximate +/-10%;
# reads low under motor load due to voltage sag.
_LIPO_CELL_PCT = [
    (4.20, 100), (4.15, 95), (4.11, 90), (4.08, 85), (4.02, 75),
    (3.97, 65), (3.92, 55), (3.87, 45), (3.85, 40), (3.82, 35),
    (3.79, 25), (3.75, 20), (3.70, 15), (3.65, 10), (3.60, 5),
    (3.30, 0),
]


def volt_to_pct(volts, cells=3):
    """Estimate SoC % from pack voltage (3S default, None if unknown).

    Linear interpolation over _LIPO_CELL_PCT, clamped 0-100.
    """
    if volts is None or volts <= 0:
        return None
    v = volts / cells
    if v >= _LIPO_CELL_PCT[0][0]:
        return 100
    for (v_hi, p_hi), (v_lo, p_lo) in zip(_LIPO_CELL_PCT, _LIPO_CELL_PCT[1:]):
        if v >= v_lo:
            frac = (v - v_lo) / (v_hi - v_lo)
            return int(round(p_lo + frac * (p_hi - p_lo)))
    return 0


class Telemetry:
    """Telemetry latest-value cache (timestamps: time.monotonic() at receive)."""

    def __init__(self):
        self.lat = None            # deg
        self.lon = None            # deg
        self.rel_alt_m = None      # above home
        self.heading_deg = None
        # NED velocity (vd positive downward). Used to confirm stop before gate.
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
        self.batt_v = None         # volts (SYS_STATUS)
        self.batt_pct = None       # ArduPilot -1 means unknown
        self.sats = None           # GPS_RAW_INT satellites_visible
        self.hdop = None           # GPS_RAW_INT eph / 100
        self.fix_type = None       # 3 = 3D fix
        # Last STATUSTEXTs from autopilot (newest last, capped STATUSTEXT_KEEP).
        # Includes prearm refusals ("PreArm: ...") needed for arming diagnosis.
        self.statustexts = []      # [{'t': monotonic, 'sev': int, 'text': str}]
        # SYS_STATUS sensor bitmasks, None until the first SYS_STATUS.
        self.sensors_present = None
        self.sensors_enabled = None
        self.sensors_health = None

    def prearm_messages(self, since_t=None):
        """Distinct arming-refusal texts, oldest first.

        ArduCopter prefixes "PreArm: " when idle, "Arm: " on arm attempt
        (severity CRITICAL). Prefix-match (not substring) excludes noise.
        """
        out = []
        for s in self.statustexts:
            if since_t is not None and s['t'] < since_t:
                continue
            low = s['text'].lower()
            if (low.startswith('prearm:') or low.startswith('arm:')) \
                    and s['text'] not in out:
                out.append(s['text'])
        return out

    def unhealthy_sensors(self):
        """Sensor names enabled but unhealthy. None if no SYS_STATUS received yet."""
        if self.sensors_health is None or self.sensors_enabled is None:
            return None
        return [name for bit, name in SENSOR_BITS
                if (self.sensors_enabled & bit) and not (self.sensors_health & bit)]

    def prearm_ok(self):
        """PREARM_CHECK bit status: True/False if known, None otherwise."""
        if self.sensors_health is None or self.sensors_present is None:
            return None
        if not (self.sensors_present & PREARM_OK_BIT):
            return None                # firmware does not publish the bit
        return bool(self.sensors_health & PREARM_OK_BIT)

    @property
    def batt_pct_est(self):
        """Battery % from voltage (survives reboots; coulomb counter resets)."""
        return volt_to_pct(self.batt_v)


class MavIO:
    def __init__(self, conn_str, source_system=1, source_component=191,
                 baud=115200, log=print):
        # baud ignored by TCP/UDP and USB CDC; only matters for real UART.
        self.log = log
        resolved = resolve_conn(conn_str)
        if resolved != conn_str:
            self.log(f"[mavio] conn 'auto' -> {resolved}")
        self.conn = mavutil.mavlink_connection(
            resolved, source_system=source_system,
            source_component=source_component, baud=baud)
        self.tel = Telemetry()
        self._mode_names = {}   # custom_mode -> name, filled after heartbeat
        self._last_setpoint_t = 0.0
        self._last_hb_t = 0.0

    # ---------- connection ----------

    def wait_ready(self, timeout=60):
        """Wait for autopilot (component 1) heartbeat + mode map. Raises on timeout.

        MUST be component 1, not 195 (ESP32 ring). pymavlink locks target_system
        only to VEHICLE types; ESP32-first leaves it at 0, breaking mode mapping.
        """
        deadline = time.monotonic() + timeout
        hb = None
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='HEARTBEAT', blocking=True,
                                       timeout=1.0)
            if (msg is not None and msg.get_srcComponent()
                    == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                hb = msg
                break
        if hb is None:
            raise TimeoutError(
                f"no heartbeat from the AUTOPILOT (component 1) in "
                f"{timeout}s. Other components on the bus do not count; if "
                f"the ring (195) is audible but the autopilot is not, the "
                f"Pixhawk end of the link is the fault.")
        # Force the lock if pymavlink declined it (it declines for every
        # non-vehicle heartbeat, and a corrupt frame can lock it to 0).
        if self.conn.target_system != hb.get_srcSystem():
            self.conn.target_system = hb.get_srcSystem()
        mapping = self.conn.mode_mapping()
        if not mapping or 'GUIDED' not in mapping:
            raise RuntimeError(
                f"no usable mode map for target_system "
                f"{self.conn.target_system}; refusing to fly with mode "
                f"names that may mean the wrong modes")
        self._mode_names = {v: k for k, v in mapping.items()}
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
        # Battery and GPS: 1 Hz suffices for mission log and self-test (human-facing).
        self.request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
        self.request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1)

    # ---------- pump ----------

    def step(self, max_wait=0.02):
        """Receive and dispatch at most one message; call in a tight loop.

        Also emits 1 Hz heartbeat as component 191 so the bus sees this node.
        """
        now = time.monotonic()
        if now - self._last_hb_t >= HEARTBEAT_PERIOD_S:
            self._last_hb_t = now
            try:
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE)
            except Exception:                             # noqa: BLE001
                pass    # dead link handled elsewhere, never crash here
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
        elif t == 'SYS_STATUS':
            tel.batt_v = (msg.voltage_battery / 1000.0
                          if msg.voltage_battery != 65535 else None)
            tel.batt_pct = (msg.battery_remaining
                            if msg.battery_remaining >= 0 else None)
            tel.sensors_present = msg.onboard_control_sensors_present
            tel.sensors_enabled = msg.onboard_control_sensors_enabled
            tel.sensors_health = msg.onboard_control_sensors_health
        elif t == 'GPS_RAW_INT':
            tel.sats = msg.satellites_visible
            tel.hdop = msg.eph / 100.0 if msg.eph != 65535 else None
            tel.fix_type = msg.fix_type
        elif t == 'STATUSTEXT':
            # STATUSTEXT logging required (SCOPE RULES 1): prearm refusals here only.
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            text = text.split('\x00')[0].strip()
            if text:
                tel.statustexts.append(
                    {'t': now, 'sev': int(msg.severity), 'text': text})
                del tel.statustexts[:-STATUSTEXT_KEEP]
                self.log(f"[ap] {SEVERITY_NAMES.get(msg.severity, msg.severity)}"
                         f": {text}")
        elif t == 'HEARTBEAT':
            # Ignore heartbeats from other components (e.g., ESP32 component 195).
            if (msg.get_srcSystem() == self.conn.target_system
                    and msg.get_srcComponent()
                    == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                tel.mode = self._mode_names.get(msg.custom_mode, tel.mode)
                tel.armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                tel.heartbeat_t = now

    # ---------- commands ----------

    _CMD_NAMES = {
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL: 'SET_MESSAGE_INTERVAL',
        mavutil.mavlink.MAV_CMD_DO_SET_MODE: 'DO_SET_MODE',
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: 'ARM_DISARM',
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: 'TAKEOFF',
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO: 'DO_SET_SERVO',
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS: 'RUN_PREARM_CHECKS',
    }

    def run_prearm_checks(self):
        """Request autopilot prearm checks via MAV_CMD_RUN_PREARM_CHECKS.

        Forces failing checks to STATUSTEXT immediately (vs. 30 s throttle).
        ACCEPTED does not mean passed; check STATUSTEXTs and SYS_STATUS prearm bit.
        """
        return self.command_ack(mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                                timeout=2.0, retries=1)

    def command_ack(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                    timeout=3.0, retries=3, retry_failed=False):
        """Send command_long and wait for COMMAND_ACK, pumping telemetry meanwhile.

        Logged per SCOPE RULES 1. retry_failed: also retry on MAV_RESULT_FAILED
        (ArduPilot issues FAILED while prearm checks settle, e.g., EKF).
        """
        name = self._CMD_NAMES.get(cmd, f'cmd{cmd}')
        for attempt in range(1, retries + 1):
            self.log(f"[mavio] -> {name} p1={p1:g} p2={p2:g} p7={p7:g}"
                     + (f" (attempt {attempt})" if attempt > 1 else ""))
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
                    self.log(f"[mavio] <- {name} ACCEPTED")
                    return True
                if msg.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    continue  # final ack still coming
                if (msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
                        or (retry_failed and msg.result
                            == mavutil.mavlink.MAV_RESULT_FAILED)):
                    self.log(f"[mavio] <- {name} result={msg.result}, "
                             f"retrying")
                    break  # retry outer loop
                self.log(f"[mavio] <- {name} REJECTED result={msg.result}")
                raise RuntimeError(f"command {cmd} rejected: result={msg.result}")
            time.sleep(0.5)
        self.log(f"[mavio] {name}: no ACCEPTED ack after {retries} tries")
        raise TimeoutError(f"command {cmd}: no ACCEPTED ack")

    def set_mode(self, name, confirm_s=5.0):
        """Change mode and wait for heartbeat confirmation. Returns bool.

        COMMAND_ACK arrives in ~10 ms but mode confirmation needs HEARTBEAT (1 Hz).
        Caller sees old mode for up to 1 s. Returns False if accepted but unconfirmed.
        """
        mapping = self._mode_names or {}
        if name not in set(mapping.values()):
            # Use cached _mode_names (from wait_ready) rather than mode_mapping()
            # to keep all lookups consistent with the verified autopilot map.
            raise ValueError(f"mode {name!r} is not in this vehicle's mode "
                             f"map; wait_ready must run first")
        mode_id = next(k for k, v in mapping.items() if v == name)
        self.command_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=mode_id)
        deadline = time.monotonic() + confirm_s
        while time.monotonic() < deadline:
            self.step()
            if self.tel.mode == name:
                return True
        return False

    def arm(self, retries=12, timeout=5.0):
        """Arm and return success/failure. Default retries: 12 x 5 s (prearm settling).

        Reason in tel.statustexts. Caller must act on False; does not raise.
        """
        return self.command_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1,
            timeout=timeout, retries=retries, retry_failed=True)

    def takeoff(self, alt_m):
        """Takeoff to relative altitude in meters."""
        self.command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt_m)

    def set_servo(self, channel, pwm_us):
        """Send MAV_CMD_DO_SET_SERVO: drive Pixhawk output directly.

        channel: 1-based SERVO number. pwm_us: pulse width microseconds.
        SERVOn_FUNCTION must be 0 (Disabled). Verify with actual servo movement.
        """
        if not PWM_MIN_US <= pwm_us <= PWM_MAX_US:
            raise ValueError(f"pwm_us out of servo range: {pwm_us}")
        return self.command_ack(mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                p1=channel, p2=pwm_us)

    def goto(self, lat, lon, rel_alt_m):
        """Guided position target (global frame, alt relative to home)."""
        self.log(f"[mavio] -> goto {lat:.7f},{lon:.7f} @{rel_alt_m:.1f}m")
        self.conn.mav.set_position_target_global_int_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MASK_POSITION_ONLY,
            int(lat * 1e7), int(lon * 1e7), rel_alt_m,
            0, 0, 0, 0, 0, 0, 0, 0)

    def velocity_ned(self, vn, ve, vd, force=False):
        """Guided velocity target (rate-limited, 5 Hz minimum).

        force=True bypasses rate limiter for setpoint CHANGES only (critical:
        drop commands must not be swallowed; drop too late if limited).
        """
        now = time.monotonic()
        if not force and now - self._last_setpoint_t < SETPOINT_RESEND_S:
            return
        if force:
            # Log forced sends (setpoint changes: stop, abort-climb). Keep-alive resends are noise.
            self.log(f"[mavio] -> velocity NED {vn:g},{ve:g},{vd:g} (forced)")
        self._last_setpoint_t = now
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MASK_VELOCITY_ONLY,
            0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)
