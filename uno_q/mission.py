"""Detect -> descend -> treat mission state machine (UNO Q onboard logic).

States:
  IDLE -> TAKEOFF -> SURVEY -> APPROACH -> DESCEND -> CROSS -> DROP -> RETURN
  -> CLIMB -> SURVEY ... -> DONE (RTL). DESCEND beside water (TF-Luna ranges
  dry ground); CROSS over puddle at captured altitude; RETURN beside water
  before climb. Offsets 0 = old vertical sequence. ABORT_CLIMB re-approaches
  same site cfg.site_retries times, then re-joins SURVEY. Mode change away
  from GUIDED => STANDDOWN (pilot control). should_stop() => STOPPED (end_mode
  still commanded).

Non-negotiable behaviors (PROJECT_STATE):
  * TARGET LATCHING: survey-altitude detection locks lat/lon; ignored outside
    SURVEY (close-range frames unreliable).
  * Rangefinder dropout during DESCEND: abort upward, skip target, resume.
    "Dropout" = (a) acquired then stale/invalid or (b) never acquired by
    rng_expect_m EKF altitude (sensor ~8m; first 15m descent blind-normal, lose
    ground return when low is not). TF-Luna 850nm specular risk over water.
  * No drop without fresh valid rangefinder reading at drop_alt_m AND descent
    arrested (vd within settle_vd_mps). Stop command != actual stop; gate dwell
    blocks loop while autopilot continues previous setpoint.
  * EKF floor: rel_alt < (drop_alt_m - floor_margin_m) without drop condition
    fired => abort (altitude sources disagree).
"""

import math
import socket
import subprocess
import time
from dataclasses import dataclass, field

from detector import offset_latlon, dist_m
# Fence geometry: make_waypoints owns point-in-polygon and point-to-edge
# distance; route generator and mission agree on "inside" by construction.
from make_waypoints import _inside, _seg_dist


def _port_in_use(port, host='127.0.0.1'):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.25)
        return s.connect_ex((host, port)) == 0


class _NoLog:
    """Null recorder: no-op for all methods when recording disabled."""
    def __getattr__(self, _name):
        return lambda *a, **kw: None


@dataclass
class MissionConfig:
    waypoints: list = field(default_factory=list)  # [(lat, lon), ...]
    survey_alt_m: float = 15.0
    drop_alt_m: float = 2.0        # rangefinder AGL that triggers the drop
    descent_mps: float = 0.5
    climb_mps: float = 1.0
    wp_radius_m: float = 1.5
    # Hold at waypoint before advancing; settles airframe for stable imagery
    # (moving camera smears puddle-sized targets). Spacing set in
    # make_waypoints.densify; zero disables hold.
    photo_hold_s: float = 2.0
    alt_tol_m: float = 1.0
    rng_timeout_s: float = 1.0
    rng_expect_m: float = 6.0      # EKF alt by which rangefinder must have acquired
    floor_margin_m: float = 1.0
    drop_dwell_s: float = 2.0      # hold position after gate closes
    # Dose (dwell seconds) = area_m2 * dose_s_per_m2, clamped to min/max.
    # Clamps safety-net unknown areas (bounding box FOV error squared). Unknown
    # area uses default, never maximum, to avoid stranding later sites.
    dose_s_per_m2: float = 0.4
    dose_s_min: float = 0.3
    dose_s_max: float = 3.0
    dose_s_default: float = 1.0    # unknown area fallback
    # Gate must not open while sinking (granules release below intended height).
    # DROP commands stop, waits for autopilot achievement (settle_max_s bounded
    # to prevent hang on missing velocity feed; drops on timeout).
    settle_vd_mps: float = 0.15
    settle_max_s: float = 3.0
    # TF-Luna cannot range still water: descend beside (dry ground, rangefinder
    # works), then translate over puddle HOLDING CAPTURED REL_ALT, release,
    # translate back beside for climb. No rangefinder abort over water (dropout
    # expected). Offsets default north; override with run_mission flags for
    # one-sided ground. Zero offsets restore vertical descent (SITL drills).
    lateral_offset_n_m: float = 3.0
    lateral_offset_e_m: float = 0.0
    cross_timeout_s: float = 20.0  # per translate; abort out, climb back
    # Re-approach attempts for each latched site before abandoning.
    site_retries: int = 1
    # Geofence as [(lat, lon), ...]. Empty = no fence; checks skipped. Camera
    # swath 16 m @ 15 m altitude (8 m either side); puddles at frame edge near
    # boundary are outside. Breach cases: ArduPilot silently refuses GUIDED
    # destination (no ack), or aircraft breaches, triggering RTL (reads as pilot
    # override, stands down mission).
    fence: list = field(default_factory=list)
    fence_margin_m: float = 2.0    # all commanded points stay this far inside
    takeoff_timeout_s: float = 60.0
    survey_leg_timeout_s: float = 120.0
    approach_timeout_s: float = 90.0
    # Ignore detections until fix quality matches arming gate (HDOP 1.4,
    # 10 sats). Position wander observed exceeds drop margin tolerance. Site
    # stays in water and can be found again on better fix later.
    hdop_max: float = 1.4
    min_sats: int = 10
    # CROSS arrival tighter tolerance: gate on 3 m cross could open 1.4 m off
    # centre; 0.5 m radius keeps 2 m puddle treated at centre.
    cross_radius_m: float = 0.5
    # Blind detector tolerance (worker or camera dead) while camera still
    # matters before abort.
    detector_blind_s: float = 15.0
    # Arm attempts (12 x 5s ~ 60s total) for EKF/GPS settling.
    arm_retries: int = 12
    end_mode: str = 'RTL'
    # Base station launch argv (None = disabled).
    basestation_cmd: list = None
    basestation_port: int = 8080   # checked to prevent double-launch


class Mission:
    # Terminal states: DONE (survey complete), STANDDOWN (pilot took aircraft),
    # STOPPED (runner requested, but end_mode still commanded unlike STANDDOWN),
    # NOARM (arming failed, never airborne; RTL not sent to disarmed), ABORTED
    # (airborne failure; end_mode sent for safety). STOPPED kept distinct from
    # DONE so interrupted flight never shows as completed survey.
    TERMINAL = ('DONE', 'STANDDOWN', 'STOPPED', 'NOARM', 'ABORTED')

    def __init__(self, io, detector, dropper, cfg, log=print, recorder=None,
                 should_stop=None):
        self.io = io
        self.det = detector
        self.dropper = dropper
        self.cfg = cfg
        self.log = log
        self.rec = recorder if recorder is not None else _NoLog()
        # Callable returning truthy reason string to stop, or None.
        self.should_stop = should_stop
        self.state = 'IDLE'
        self.history = []          # (t, state, note)
        self.wp_i = 0
        self.target = None         # latched (lat, lon): the DESCENT point, beside
        self.puddle = None         # the water itself: where the gate opens
        self.target_area_m2 = None # estimated puddle area at latch time
        self.abort_reason = None
        self._retries_left = 0     # re-approaches remaining for this site
        # Altitude reference captured beside water at drop height; used for
        # all over-water flight (Luna reads nothing there).
        self._drop_rel_alt = None
        self._t_state = 0.0        # time of last state entry
        self._rng_acquired = False # ground return seen during current descent
        self._last_fix_gripe = None # so a poor fix is logged once, not per tick
        self._hold_since = None     # photo hold start, None when not holding
        self._t_dropped = None     # monotonic time the gate finished cycling
        self._drop_ok = True       # last gate actuation succeeded
        # Pilot-override test fires only after GUIDED observed in heartbeat;
        # tel.mode pre-takeoff stale, gated until confirmed.
        self._seen_guided = False
        # Test hook: dropout drill below this rel_alt; None = disabled.
        self.rng_suppress_below_m = None
        self._last_tel_log = 0.0   # telemetry logging timestamp (1 Hz)

    # ---------- helpers ----------

    def _set(self, state, note=''):
        self.log(f"[mission] {self.state} -> {state}"
                 + (f" ({note})" if note else ""))
        self.rec.state(self.state, state, note)
        self.state = state
        self._t_state = time.monotonic()
        self.history.append((self._t_state, state, note))

    def _elapsed(self):
        return time.monotonic() - self._t_state

    def _tel_line(self, tel):
        """One telemetry snapshot per second into log; enables replay from log alone."""
        now = time.monotonic()
        if now - self._last_tel_log < 1.0:
            return
        self._last_tel_log = now
        f = lambda v, spec='.1f': format(v, spec) if v is not None else '-'
        self.log(f"[tel] {self.state} mode={tel.mode} armed={tel.armed} "
                 f"lat={f(tel.lat, '.7f')} lon={f(tel.lon, '.7f')} "
                 f"alt={f(tel.rel_alt_m)}m rng={f(tel.rng_m, '.2f')}m"
                 f"{'' if tel.rng_valid else '(stale)'} "
                 f"vd={f(tel.vd_mps, '.2f')} batt={f(tel.batt_v, '.2f')}V"
                 f"/~{f(tel.batt_pct_est, '.0f')}% sats={f(tel.sats, 'd')} "
                 f"hdop={f(tel.hdop, '.2f')}")

    def _rng_fresh(self):
        tel = self.io.tel
        if (self.rng_suppress_below_m is not None
                and tel.rel_alt_m is not None
                and tel.rel_alt_m < self.rng_suppress_below_m):
            return False  # simulated dropout (drill)
        return (tel.rng_valid
                and time.monotonic() - tel.rng_t < self.cfg.rng_timeout_s)

    def _inside_fence(self, lat, lon, margin=None):
        """Is point >= margin metres inside geofence? No fence returns True."""
        poly = self.cfg.fence
        if not poly or len(poly) < 3:
            return True
        margin = self.cfg.fence_margin_m if margin is None else margin
        lat0, lon0 = poly[0]
        mlat = 111320.0
        mlon = mlat * math.cos(math.radians(lat0))
        if abs(mlon) < 1.0:
            return True
        pts = [((a - lat0) * mlat, (b - lon0) * mlon) for a, b in poly]
        p = ((lat - lat0) * mlat, (lon - lon0) * mlon)
        if not _inside(p[0], p[1], pts):
            return False
        return all(_seg_dist(p[0], p[1], *pts[i], *pts[(i + 1) % len(pts)])
                   >= margin for i in range(len(pts)))

    def _gps_ok(self):
        """Fix quality check for latch/drop. Returns (ok, why). Missing HDOP or sats treated OK."""
        tel, cfg = self.io.tel, self.cfg
        if tel.hdop is not None and tel.hdop > cfg.hdop_max:
            return False, f'HDOP {tel.hdop:.2f} > {cfg.hdop_max:g}'
        if tel.sats is not None and tel.sats < cfg.min_sats:
            return False, f'{tel.sats} sats < {cfg.min_sats}'
        return True, ''

    def _at_wp(self, lat, lon, alt=None, radius=None):
        tel = self.io.tel
        if tel.lat is None:
            return False
        radius = self.cfg.wp_radius_m if radius is None else radius
        if dist_m(tel.lat, tel.lon, lat, lon) > radius:
            return False
        if alt is not None and abs(tel.rel_alt_m - alt) > self.cfg.alt_tol_m:
            return False
        return True

    # ---------- main loop ----------

    def run(self):
        cfg = self.cfg
        io = self.io
        self.rec.mission_start(cfg)
        if io.set_mode('GUIDED'):
            self._seen_guided = True
        else:
            self.log("[mission] GUIDED was accepted but never confirmed by a "
                     "heartbeat; the override check stays disarmed until it is")
        # Must check arm result and end if refused, reporting autopilot reason.
        arm_t = time.monotonic()
        if not io.arm(retries=cfg.arm_retries):
            why = io.tel.prearm_messages(since_t=arm_t - 5.0)
            self._set('NOARM', '; '.join(why) if why
                      else 'the autopilot refused to arm and gave no reason')
        else:
            io.takeoff(cfg.survey_alt_m)
            self._set('TAKEOFF')

        while self.state not in self.TERMINAL:
            io.step()
            tel = io.tel
            self.rec.fix(tel, self.state)  # throttled inside the recorder
            self._tel_line(tel)

            if tel.mode == 'GUIDED':
                self._seen_guided = True

            # Pilot override: mode flipped from under mission. Only meaningful
            # after GUIDED seen (stale telemetry before then).
            if (self.state != 'IDLE' and self._seen_guided
                    and tel.mode is not None and tel.mode != 'GUIDED'):
                self._set('STANDDOWN', f"mode={tel.mode}, pilot has aircraft")
                break

            # Blind detector abort only during TAKEOFF/SURVEY/APPROACH (after
            # DESCEND, target latched and rangefinder governs).
            blind = self.det.blind_for_s() if hasattr(self.det, 'blind_for_s') \
                else 0.0
            if (self.state in ('TAKEOFF', 'SURVEY', 'APPROACH')
                    and blind > self.cfg.detector_blind_s):
                self._set('ABORTED',
                          f"detector blind for {blind:.0f}s (worker or camera "
                          f"dead); ending rather than flying a blind survey")
                break

            # Runner stop request (signal, lost console). Checked after pilot
            # test, before state handler to prevent new descent on exit.
            if self.should_stop is not None:
                why = self.should_stop()
                if why:
                    self._set('STOPPED', str(why))
                    break

            getattr(self, '_st_' + self.state.lower())()

        if self.state in ('DONE', 'STOPPED', 'ABORTED'):
            confirmed = io.set_mode(cfg.end_mode)
            self.log(f"[mission] {self.state.lower()}, {cfg.end_mode} "
                     + ("confirmed" if confirmed
                        else "ACCEPTED but NOT confirmed by heartbeat, WATCH "
                             "THE AIRCRAFT and be ready on the sticks"))
        # Count successful gate cycles (not attempts); "drops: 3" means three
        # puddles treated.
        self.rec.mission_end(
            self.state,
            getattr(self.dropper, 'succeeded',
                    getattr(self.dropper, 'fired', None)))
        self._launch_basestation()
        return self.state

    def _launch_basestation(self):
        """Fire-and-forget launch; skips if already running (avoids EADDRINUSE)."""
        cmd = self.cfg.basestation_cmd
        if not cmd:
            return
        if _port_in_use(self.cfg.basestation_port):
            self.log(f"[mission] base station already up on port "
                     f"{self.cfg.basestation_port}, not launching a second")
            return
        try:
            subprocess.Popen(cmd, start_new_session=True)
        except OSError as exc:
            self.log(f"[mission] base station launch FAILED: {exc}")
            return
        self.log(f"[mission] base station launched: {cmd}")

    # ---------- state handlers ----------

    def _st_takeoff(self):
        tel = self.io.tel
        if (tel.rel_alt_m is not None
                and tel.rel_alt_m >= self.cfg.survey_alt_m - self.cfg.alt_tol_m):
            self._goto_current_wp()
            return
        if self._elapsed() >= self.cfg.takeoff_timeout_s:
            # ABORTED commands end_mode; partial climb goes to RTL, not hover.
            self._set('ABORTED',
                      f"never reached {self.cfg.survey_alt_m:g} m in "
                      f"{self.cfg.takeoff_timeout_s:.0f}s "
                      f"(at {tel.rel_alt_m if tel.rel_alt_m is None else round(tel.rel_alt_m, 1)} m)")

    def _goto_current_wp(self):
        # Every arrival starts fresh hold (timeout-skipped or re-entered).
        self._hold_since = None
        if self.wp_i >= len(self.cfg.waypoints):
            self._set('DONE', 'survey complete')
            return
        lat, lon = self.cfg.waypoints[self.wp_i]
        self.io.goto(lat, lon, self.cfg.survey_alt_m)
        self._set('SURVEY', f"wp {self.wp_i}")

    def dose_for(self, area_m2):
        """Dwell seconds for given area. Returns (seconds, description). Unknown area defaults, never max."""
        cfg = self.cfg
        if area_m2 is None:
            return cfg.dose_s_default, 'area unknown, default dose'
        raw = area_m2 * cfg.dose_s_per_m2
        dwell = min(cfg.dose_s_max, max(cfg.dose_s_min, raw))
        note = f'{area_m2:.1f} m2'
        if dwell != raw:
            note += f' (clamped from {raw:.2f}s)'
        return dwell, note

    def _st_survey(self):
        det = self.det.poll(self.io.tel)
        if det is not None:
            # Latch gated on fix quality (wander observed exceeds puddle size).
            # Skip if poor fix; site can be re-found on better fix later.
            ok, why = self._gps_ok()
            if not ok:
                if why != self._last_fix_gripe:
                    self.log(f"[mission] detection IGNORED, fix is poor: {why}")
                    self._last_fix_gripe = why
                det = None
            else:
                self._last_fix_gripe = None
        if det is not None:
            # Check both water and descent point against fence (swath ~8 m either
            # side; puddle at frame edge near boundary is outside).
            beside = offset_latlon(det.lat, det.lon,
                                   self.cfg.lateral_offset_n_m,
                                   self.cfg.lateral_offset_e_m)
            if not (self._inside_fence(det.lat, det.lon)
                    and self._inside_fence(*beside)):
                self.log(f"[mission] detection at {det.lat:.7f},{det.lon:.7f} "
                         f"IGNORED: it or its descent point is outside the "
                         f"geofence (margin {self.cfg.fence_margin_m:g} m). "
                         f"Flying there would either be silently refused or "
                         f"breach the fence and trigger RTL.")
                self.rec.detection(det.lat, det.lon, det.confidence)
                det = None
        if det is not None:
            # Latch target now at survey altitude; ignore later detections.
            self.rec.detection(det.lat, det.lon, det.confidence)
            self.puddle = (det.lat, det.lon)
            self.target = offset_latlon(det.lat, det.lon,
                                        self.cfg.lateral_offset_n_m,
                                        self.cfg.lateral_offset_e_m)
            self.target_area_m2 = getattr(det, 'area_m2', None)
            self._retries_left = self.cfg.site_retries
            self.rec.latch(*self.target)
            self.io.goto(*self.target, self.cfg.survey_alt_m)
            self._set('APPROACH',
                      f"latched {self.target[0]:.7f},{self.target[1]:.7f}")
            return
        lat, lon = self.cfg.waypoints[self.wp_i]
        if self._at_wp(lat, lon):
            if self.cfg.photo_hold_s > 0:
                if self._hold_since is None:
                    self._hold_since = time.monotonic()
                    return
                if time.monotonic() - self._hold_since < self.cfg.photo_hold_s:
                    return
            self.wp_i += 1
            self._goto_current_wp()
            return
        if self._elapsed() >= self.cfg.survey_leg_timeout_s:
            # Skip single leg on timeout (wind/avoidance/unreachable), not whole
            # flight; logged to avoid silent gap.
            self.log(f"[mission] waypoint {self.wp_i} not reached in "
                     f"{self.cfg.survey_leg_timeout_s:.0f}s, skipping it")
            self.wp_i += 1
            self._goto_current_wp()

    def _st_approach(self):
        if self._at_wp(*self.target, alt=self.cfg.survey_alt_m):
            self._rng_acquired = False
            self._set('DESCEND')
            return
        if self._elapsed() >= self.cfg.approach_timeout_s:
            # SET_POSITION_TARGET_GLOBAL_INT carries no ack; refused destination
            # indistinguishable from pending. Abandon site, resume survey.
            self.log(f"[mission] APPROACH timed out after "
                     f"{self.cfg.approach_timeout_s:.0f}s; the destination may "
                     f"have been refused (position targets are not acked). "
                     f"Abandoning this site and resuming the survey.")
            self.target = self.puddle = None
            self._goto_current_wp()

    def _st_descend(self):
        cfg, tel = self.cfg, self.io.tel
        fresh = self._rng_fresh()
        if fresh:
            self._rng_acquired = True

        # Case (a): acquired then lost ground.
        if self._rng_acquired and not fresh:
            self._abort('rangefinder dropout during descent')
            return
        # Case (b): low enough to see ground, doesn't.
        if (not self._rng_acquired and tel.rel_alt_m is not None
                and tel.rel_alt_m < cfg.rng_expect_m):
            self._abort('no rangefinder acquisition by expected altitude')
            return
        if self._below_floor():
            # Altitude sources disagree; abort.
            self._abort('EKF floor hit without rangefinder drop condition')
            return
        if fresh and tel.rng_m <= cfg.drop_alt_m:
            # Forced send (bypasses rate limiter; setpoint change critical).
            # Gate opens in DROP state once aircraft stopped.
            self.io.velocity_ned(0, 0, 0, force=True)
            self._t_dropped = None
            # Last trustworthy AGL; all over-water flight uses this reference.
            self._drop_rel_alt = tel.rel_alt_m
            note = f"rng={tel.rng_m:.2f}m"
            if self._cross_m() <= cfg.wp_radius_m:
                self._set('DROP', note + ", stopping before release")
            elif self._drop_rel_alt is None:
                # No altitude reference; release beside water, not blind across.
                self._set('DROP', note + ", no rel_alt; releasing beside water")
            else:
                self.io.goto(*self.puddle, self._drop_rel_alt)
                self._set('CROSS', note + f", crossing {self._cross_m():.1f} m "
                                          f"to the water at "
                                          f"{self._drop_rel_alt:.1f} m")
            return
        self.io.velocity_ned(0, 0, +cfg.descent_mps)

    def _cross_m(self):
        """Distance from descent point to water."""
        if self.puddle is None or self.target is None:
            return 0.0
        return dist_m(self.target[0], self.target[1],
                      self.puddle[0], self.puddle[1])

    def _st_cross(self):
        """Translate beside -> over water at captured altitude. No rangefinder abort
        here (still water reads nothing; was previous abort cause)."""
        if self._at_wp(*self.puddle, alt=self._drop_rel_alt,
                       radius=self.cfg.cross_radius_m):
            self.io.velocity_ned(0, 0, 0, force=True)
            self._t_dropped = None
            self._set('DROP', 'over the water, stopping before release')
            return
        if self._elapsed() >= self.cfg.cross_timeout_s:
            self._abort(f'did not reach the water in '
                        f'{self.cfg.cross_timeout_s:.0f}s')

    def _st_return(self):
        """Translate over water -> back beside it; Luna regains ground target.
        Never aborts (granules already released). Timeout climbs from current position."""
        if (self._at_wp(*self.target, alt=self._drop_rel_alt)
                or self._elapsed() >= self.cfg.cross_timeout_s):
            self._set('CLIMB', 'beside the water again'
                      if self._at_wp(*self.target, alt=self._drop_rel_alt)
                      else 'return timed out, climbing from here')

    def _below_floor(self):
        tel, cfg = self.io.tel, self.cfg
        return (tel.rel_alt_m is not None
                and tel.rel_alt_m < cfg.drop_alt_m - cfg.floor_margin_m)

    def _stopped(self):
        """True once descent arrested or settle timeout reached (timeout is bigger risk)."""
        if self._elapsed() >= self.cfg.settle_max_s:
            self.log(f"[mission] settle timed out after "
                     f"{self.cfg.settle_max_s:.1f}s, releasing anyway")
            return True
        vd = self.io.tel.vd_mps
        if vd is None:
            # No velocity feed; use half settle timeout for autopilot response.
            return self._elapsed() >= self.cfg.settle_max_s / 2
        return abs(vd) <= self.cfg.settle_vd_mps

    def _abort(self, reason):
        self.abort_reason = reason
        tel = self.io.tel
        # Reverse descent on this tick (forced past rate limiter; waiting for
        # slot means still descending).
        self.io.velocity_ned(0, 0, -self.cfg.climb_mps, force=True)
        self.rec.abort(tel.lat, tel.lon, reason)
        self._set('ABORT_CLIMB', reason)

    def _st_drop(self):
        """Hold, stop, release, hold again. Split: dropper.trigger() blocks
        during dwell (state machine pauses); previous setpoint continues."""
        # Rate-limited (forced send already on DESCEND->DROP; every tick would
        # flood 115200 serial with setpoints).
        self.io.velocity_ned(0, 0, 0)
        tel = self.io.tel

        if self._t_dropped is None:
            if self._below_floor():
                # Sank through floor while settling; abort.
                self._abort('EKF floor hit while settling for release')
                return
            if not self._stopped():
                return
            dwell, why = self.dose_for(self.target_area_m2)
            self.log(f"[mission] dose {dwell:.2f}s ({why})")
            ok = self.dropper.trigger(dwell)
            self._t_dropped = time.monotonic()
            self._drop_ok = ok is not False
            # Record drop position and success; failed gate must not mark treated.
            self.rec.drop(tel.lat, tel.lon,
                          tel.rng_m if tel.rng_valid else None,
                          ok=(ok is not False),
                          dwell_s=dwell, area_m2=self.target_area_m2)
            if ok is False:
                self.log("[mission] DROP FAILED: gate did not actuate")
            return

        if time.monotonic() - self._t_dropped >= self.cfg.drop_dwell_s:
            note = 'treated' if self._drop_ok else 'drop failed'
            if self._cross_m() > self.cfg.wp_radius_m \
                    and self._drop_rel_alt is not None:
                self.io.goto(*self.target, self._drop_rel_alt)
                self._set('RETURN', note + ', crossing back beside the water')
            else:
                self._set('CLIMB', note)

    def _st_climb(self):
        self._climb_then_resume()

    def _st_abort_climb(self):
        self._climb_then_resume()

    def _climb_then_resume(self):
        cfg, tel = self.cfg, self.io.tel
        if (tel.rel_alt_m is not None
                and tel.rel_alt_m >= cfg.survey_alt_m - cfg.alt_tol_m):
            # Aborted descent usually bad luck, not site failure; detector won't
            # re-offer (coordinates latched). Re-approach same point until
            # retries spent.
            if (self.state == 'ABORT_CLIMB' and self._retries_left > 0
                    and self.target is not None):
                self._retries_left -= 1
                self._drop_rel_alt = None
                self._rng_acquired = False
                self.io.goto(*self.target, cfg.survey_alt_m)
                self._set('APPROACH', f"retrying the site, "
                                      f"{self._retries_left} retry left "
                                      f"after this")
                return
            self.target = None
            self.puddle = None
            self.target_area_m2 = None
            self._drop_rel_alt = None
            self._goto_current_wp()   # resume survey where we left off
            return
        self.io.velocity_ned(0, 0, -cfg.climb_mps)

    def _st_done(self):
        pass

    def _st_standdown(self):
        pass

    def _st_stopped(self):
        pass

    def _st_idle(self):
        pass
