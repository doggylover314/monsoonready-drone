"""MAVLink link layer for the UNO Q mission computer.

Single-threaded by design: one pump (step()) receives everything and updates
the telemetry cache; commands that need an ACK pump the same loop while they
wait. No locks, no races, and identical behavior on the laptop (SITL over TCP)
and on the UNO Q (the Pixhawk's own USB through the hub, /dev/ttyACM*).

We are component 191 (MAV_COMP_ID_ONBOARD_COMPUTER) on the vehicle's system id.

CONNECTION 'auto' (2026-08-16): the flight link is now the Pixhawk's USB
plug on the board's hub, so the right device is discoverable rather than
guessable: /dev/serial/by-id/ carries one stable ArduPilot entry per
autopilot, immune to ACM number shuffling the same way the camera is now
immune to video number shuffling. resolve_conn('auto') finds it or explains
why not. (Proven 2026-08-16 00:10: 8/8 heartbeats three times while the
camera streamed 600 frames beside it; the byte-shovel this replaces is
deleted.)
"""

import glob
import time

from pymavlink import mavutil

# What an ArduPilot autopilot's USB CDC port looks like in /dev/serial/by-id.
# This board's Pixhawk: usb-ArduPilot_Pixhawk1-bdshot_2D00...-if00.
_PIXHAWK_ID_PATTERNS = ('*ArduPilot*', '*Pixhawk*', '*PX4*', '*fmu*')


def resolve_conn(conn):
    """'auto' -> the Pixhawk's /dev/serial/by-id path; anything else passes
    through untouched (SITL tcp:, explicit devices, udp for bench rigs)."""
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

# ArduPilot rejects guided setpoints older than a few seconds; resend at 5Hz.
SETPOINT_RESEND_S = 0.2

# Our own heartbeat rate as component 191. 1 Hz is the MAVLink convention and
# is what makes `tools/bench.py nodes` able to see the UNO Q at all.
HEARTBEAT_PERIOD_S = 1.0

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


# STATUSTEXT cache depth and the MAV_SEVERITY names used when logging them.
STATUSTEXT_KEEP = 40
SEVERITY_NAMES = {0: 'EMERGENCY', 1: 'ALERT', 2: 'CRITICAL', 3: 'ERROR',
                  4: 'WARNING', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'}

# SYS_STATUS sensor bits worth naming for a human. Values are MAVLink's
# MAV_SYS_STATUS_SENSOR_* (verified against the aircraft's own firmware,
# ArduCopter 4.7.0: libraries/GCS_MAVLink/GCS.cpp:429-623 and
# ArduCopter/GCS_Copter.cpp:23-100). PREARM_CHECK (bit 28) is the one flag
# that says "the prearm checks are passing right now".
SENSOR_BITS = [
    (1 << 0, 'gyro'), (1 << 1, 'accel'), (1 << 2, 'compass'),
    (1 << 3, 'baro'), (1 << 5, 'GPS'), (1 << 8, 'rangefinder'),
    (1 << 16, 'RC receiver'), (1 << 20, 'geofence'), (1 << 21, 'AHRS'),
    (1 << 24, 'logging'), (1 << 25, 'battery'), (1 << 26, 'proximity'),
]
PREARM_OK_BIT = 1 << 28            # MAV_SYS_STATUS_PREARM_CHECK

# Rest-voltage -> state-of-charge for one LiPo cell, the standard published
# approximation. WHY this exists (user, 2026-08-16): ArduPilot's own
# battery_remaining is a coulomb counter that RESETS TO ~100% at every boot,
# so after a battery swap or reboot it lies until a full flight drains it
# (the bench showed 11.88 V as "99%" when the pack was really ~60%).
# Voltage survives reboots. Caveats, stated wherever the estimate is shown:
# it is approximate (+/-10 points), and it reads LOW while motors run
# because load sag drops the terminal voltage below the rest voltage.
_LIPO_CELL_PCT = [
    (4.20, 100), (4.15, 95), (4.11, 90), (4.08, 85), (4.02, 75),
    (3.97, 65), (3.92, 55), (3.87, 45), (3.85, 40), (3.82, 35),
    (3.79, 25), (3.75, 20), (3.70, 15), (3.65, 10), (3.60, 5),
    (3.30, 0),
]


def volt_to_pct(volts, cells=3):
    """Estimate battery % from pack voltage (3S default). None if unknown.

    Linear interpolation over _LIPO_CELL_PCT, clamped to 0..100.
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
        self.batt_v = None         # volts (SYS_STATUS)
        self.batt_pct = None       # -1 from ArduPilot = unknown -> None
        self.sats = None           # GPS_RAW_INT satellites_visible
        self.hdop = None           # GPS_RAW_INT eph / 100
        self.fix_type = None       # 3 = 3D fix
        # Last STATUSTEXTs from the autopilot, newest last, capped at
        # STATUSTEXT_KEEP. This is where ArduPilot puts the REASON it will
        # not arm ("PreArm: ..."), which the dashboard has to show for the
        # arming tests to be fixable at the field (user, 2026-08-16).
        self.statustexts = []      # [{'t': monotonic, 'sev': int, 'text': str}]
        # SYS_STATUS sensor bitmasks, None until the first SYS_STATUS.
        self.sensors_present = None
        self.sensors_enabled = None
        self.sensors_health = None

    def prearm_messages(self, since_t=None):
        """Distinct arming-refusal texts, oldest first.

        ArduCopter 4.7 prefixes them "PreArm: " while idle and "Arm: "
        during an actual arm attempt (AP_Arming.cpp:336-387), at severity
        CRITICAL. Matching the prefix (not a keyword search) keeps ordinary
        chatter such as "EKF3 IMU0 is using GPS" out of the list.
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
        """Names of sensors the autopilot reports as enabled but NOT healthy.

        None if no SYS_STATUS has arrived yet (which is itself worth saying,
        rather than claiming everything is fine).
        """
        if self.sensors_health is None or self.sensors_enabled is None:
            return None
        return [name for bit, name in SENSOR_BITS
                if (self.sensors_enabled & bit) and not (self.sensors_health & bit)]

    def prearm_ok(self):
        """True/False from the PREARM_CHECK health bit, None if unknown."""
        if self.sensors_health is None or self.sensors_present is None:
            return None
        if not (self.sensors_present & PREARM_OK_BIT):
            return None                # firmware does not publish the bit
        return bool(self.sensors_health & PREARM_OK_BIT)

    @property
    def batt_pct_est(self):
        """Voltage-derived battery %, ~accurate across reboots (see
        volt_to_pct); ArduPilot's batt_pct resets to 100 every boot."""
        return volt_to_pct(self.batt_v)


class MavIO:
    def __init__(self, conn_str, source_system=1, source_component=191,
                 baud=115200, log=print):
        # baud is ignored by tcp:/udp: connection strings and by USB CDC
        # (the Pixhawk's USB ignores the number entirely); it only matters if
        # a real UART ever carries this link again.
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
        """Heartbeat FROM THE AUTOPILOT + mode map. Raises on timeout.

        IT MUST BE COMPONENT 1'S HEARTBEAT, not merely the first heartbeat.
        The ESP32 obstacle ring is component 195 on the same vehicle, beats
        at about the same 1 Hz, and ArduPilot forwards it to us over
        SERIAL5, so roughly half of all connections see it first. pymavlink
        only locks target_system onto a heartbeat it judges to be a
        VEHICLE, and MAV_TYPE_ONBOARD_CONTROLLER is explicitly excluded
        (mavutil.probably_vehicle_heartbeat), so an ESP32-first connection
        leaves target_system at 0 -- whose default mav_type is 1, FIXED
        WING. mode_mapping() then hands back the PLANE map, and every mode
        name in this project silently means something else:
            GUIDED -> 15, which is AUTOTUNE on Copter
            RTL    -> 11, which is DRIFT on Copter
            custom_mode 4 reads back as "ACRO", so Mission's pilot-override
            test sees a non-GUIDED mode and stands down instantly.
        Reproduced 2026-08-15 by feeding both heartbeat orderings through
        pymavlink; the plane map has 26 entries, the copter map 27. This
        could not appear before 2026-08-14 because the ring was silent, and
        it cannot appear in SITL, which has no second component.
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
        # Battery and GPS quality, for the 1 Hz mission log line and the
        # self-test. 1 Hz: these inform humans, not control loops.
        self.request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
        self.request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1)

    # ---------- pump ----------

    def step(self, max_wait=0.02):
        """Receive+dispatch at most one message; call in a tight loop.

        Also emits our own 1 Hz heartbeat as component 191. WHY IT IS HERE:
        the probe sketch used to heartbeat from firmware, so `bench.py nodes`
        could see the UNO Q on the bus. sketch_mav_shovel replaced it and
        only FORWARDS bytes, and MavIO previously sent commands but never a
        heartbeat, so comp 191 vanished from the bus and the nodes check read
        as "the UNO Q is not reaching the Pixhawk" while the link was in fact
        working perfectly (2026-08-15 at the farm: the same session's
        SET_MESSAGE_INTERVAL was ACKed, which is proof of the TX path).
        Announcing ourselves is also simply correct MAVLink citizenship for a
        companion computer.
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
                pass    # a dead link is the pump's problem, never a crash here
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
            # EVERY autopilot message is logged (SCOPE RULES 1) and kept in
            # the cache. Prearm refusals arrive here and nowhere else, so
            # dropping them is how "it just will not arm" stays a mystery.
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

    _CMD_NAMES = {
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL: 'SET_MESSAGE_INTERVAL',
        mavutil.mavlink.MAV_CMD_DO_SET_MODE: 'DO_SET_MODE',
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: 'ARM_DISARM',
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: 'TAKEOFF',
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO: 'DO_SET_SERVO',
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS: 'RUN_PREARM_CHECKS',
    }

    def run_prearm_checks(self):
        """Ask the autopilot to run its prearm checks and report NOW.

        Copter 4.7 handles MAV_CMD_RUN_PREARM_CHECKS (401) in
        GCS_Common.cpp:4995-5004: it calls pre_arm_checks(true), which
        forces every failing check to be sent as STATUSTEXT immediately
        instead of waiting out the normal 30 s display throttle
        (AP_Arming.cpp:109-111). Without this, a ground station can sit for
        half a minute before it learns why the aircraft will not arm.

        ACCEPTED does NOT mean the checks passed: the handler returns
        ACCEPTED unconditionally when disarmed (and TEMPORARILY_REJECTED
        while armed). The verdict is in the STATUSTEXTs and in the
        SYS_STATUS prearm bit.
        """
        return self.command_ack(mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                                timeout=2.0, retries=1)

    def command_ack(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                    timeout=3.0, retries=3, retry_failed=False):
        """command_long + wait for its COMMAND_ACK, pumping telemetry meanwhile.

        Every send and every ack result is logged (SCOPE RULES 1): the farm
        day was undiagnosable partly because nothing recorded which commands
        went out and what the autopilot said back.

        retry_failed: also retry on MAV_RESULT_FAILED (ArduPilot answers FAILED
        for arm attempts while prearm checks are still settling, e.g. EKF).
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
        """Change mode and WAIT for the heartbeat to say so. Returns bool.

        The COMMAND_ACK proves the autopilot accepted the command; it does
        not prove tel.mode has caught up, because tel.mode only moves on a
        HEARTBEAT and those arrive at 1 Hz. Every command here ACKs in tens
        of milliseconds, so a caller that inspects tel.mode straight after
        set_mode reads the PREVIOUS mode for up to a second. Mission's
        pilot-override test does exactly that on its first loop iteration,
        and would stand down before the survey ever started.

        Returns False if the mode was accepted but never confirmed, so the
        caller can decide; it does not raise, because at mission end an
        unconfirmed RTL is still an accepted RTL.
        """
        mapping = self._mode_names or {}
        if name not in set(mapping.values()):
            # _mode_names is the reverse map built in wait_ready from a
            # verified autopilot heartbeat. Trusting it rather than calling
            # mode_mapping() again keeps every mode lookup on the one map
            # that was checked.
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
        self.log(f"[mavio] -> goto {lat:.7f},{lon:.7f} @{rel_alt_m:.1f}m")
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
        if force:
            # Only forced sends are logged: they are the setpoint CHANGES
            # (stop, abort-climb). The 5 Hz keep-alive resends would be noise.
            self.log(f"[mavio] -> velocity NED {vn:g},{ve:g},{vd:g} (forced)")
        self._last_setpoint_t = now
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MASK_VELOCITY_ONLY,
            0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)
