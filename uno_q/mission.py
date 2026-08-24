"""Detect -> descend -> treat mission state machine (UNO Q onboard logic).

States:
  IDLE -> TAKEOFF -> SURVEY -> APPROACH -> DESCEND -> CROSS -> DROP -> RETURN
  -> CLIMB -> SURVEY ... -> DONE (RTL). DESCEND happens beside the water,
  since TF-Luna only ranges dry ground; CROSS moves over the puddle at the
  captured altitude; RETURN comes back beside the water before the climb.
  Offsets of 0 reproduce the old vertical sequence. ABORT_CLIMB re-approaches
  the same site cfg.site_retries times, then rejoins SURVEY. A mode change
  away from GUIDED goes to STANDDOWN (pilot has the aircraft). should_stop()
  returning truthy goes to STOPPED (end_mode is still commanded there).

Non-negotiable behaviors (PROJECT_STATE):
  * Target latching: a detection at survey altitude locks in lat/lon;
    detections outside SURVEY are ignored, since close-range frames are
    unreliable.
  * Rangefinder dropout during DESCEND aborts upward, skips the target, and
    resumes the survey. Dropout means either (a) a reading was acquired then
    went stale or invalid, or (b) none was acquired by rng_expect_m EKF
    altitude (sensor range ~8m, so a descent starting above that is blind
    at first and that is normal; losing the ground return once low is not). TF-Luna is 850nm and
    risks specular reflection over water.
  * No drop happens without a fresh, valid rangefinder reading at drop_alt_m
    and the descent arrested (vd within settle_vd_mps). A stop command is
    not the same as an actual stop: gate dwell blocks the loop while the
    autopilot is still finishing the previous setpoint.
  * EKF floor: rel_alt dropping below (drop_alt_m - floor_margin_m) without
    the drop condition having fired means the altitude sources disagree, so
    abort.
"""

import math
import socket
import subprocess
import time
from dataclasses import dataclass, field
from typing import Optional

from detector import offset_latlon, dist_m
# make_waypoints owns point-in-polygon and point-to-edge distance, so the
# route generator and this file agree on what "inside" means by construction.
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
    survey_alt_m: float = 5.0
    drop_alt_m: float = 1.0        # rangefinder AGL that triggers the drop
    descent_mps: float = 0.5
    climb_mps: float = 1.0
    wp_radius_m: float = 1.5
    # Hold at each waypoint before advancing: settles the airframe for
    # stable imagery, since a moving camera smears puddle-sized targets.
    # Waypoint spacing lives in make_waypoints.densify; zero disables the hold.
    photo_hold_s: float = 1.0
    alt_tol_m: float = 1.0
    rng_timeout_s: float = 1.0
    rng_expect_m: float = 6.0      # EKF alt by which rangefinder must have acquired
    # At a 5 m survey the descent starts already below rng_expect_m, so the
    # altitude test alone would abort on the first stale tick. This gives
    # acquisition some time first.
    rng_grace_s: float = 3.0
    climb_timeout_s: float = 30.0  # give up waiting for altitude, resume anyway
    floor_margin_m: float = 0.5
    drop_dwell_s: float = 2.0      # hold position after gate closes
    # Dose (dwell seconds) = area_m2 * dose_s_per_m2, clamped to min/max. The
    # clamp is a safety net: a bounding-box area estimate squares whatever
    # FOV error is in the box. An unknown area falls back to the default
    # dose, never the max, so one bad estimate can't strand the sites still
    # to come.
    dose_s_per_m2: float = 0.4
    dose_s_min: float = 0.3
    dose_s_max: float = 3.0
    dose_s_default: float = 1.0    # unknown area fallback
    # Gate must not open while sinking, or granules release below the
    # intended height. DROP state commands a stop, then waits for the
    # autopilot to actually achieve it; settle_max_s bounds that wait so a
    # missing velocity feed can't hang the mission, and it drops anyway once
    # the timeout expires.
    settle_vd_mps: float = 0.15
    settle_max_s: float = 3.0
    # TF-Luna cannot range still water, so the aircraft descends beside the
    # puddle where the ground gives it a return, then translates over the
    # water holding the captured rel_alt, releases, and translates back
    # beside it before climbing. No rangefinder abort while over water:
    # dropout there is expected, not a fault. Offsets default north;
    # override with the run_mission flags for one-sided ground. Zero
    # offsets restore the old vertical descent, used for SITL drills.
    lateral_offset_n_m: float = 1.5
    lateral_offset_e_m: float = 0.0
    cross_timeout_s: float = 20.0  # per translate; abort out, climb back
    # Re-approach attempts for each latched site before abandoning.
    site_retries: int = 1
    # Geofence as [(lat, lon), ...]; empty means no fence and checks are
    # skipped. Camera swath is 16 m at 15 m altitude (8 m either side), so a
    # puddle near the frame edge close to the boundary can sit just outside
    # it. A breach shows up two ways: ArduPilot silently refuses the GUIDED
    # destination (no ack), or the aircraft actually breaches and RTLs,
    # which reads as a pilot override and stands the mission down.
    fence: list = field(default_factory=list)
    fence_margin_m: float = 2.0    # all commanded points stay this far inside
    takeoff_timeout_s: float = 60.0
    survey_leg_timeout_s: float = 120.0
    approach_timeout_s: float = 90.0
    # Ignore detections until fix quality matches the arming gate (HDOP 1.4,
    # 10 sats): position wander was observed to exceed the drop margin below
    # that. The site stays in the water and can be re-found on a better fix
    # later.
    hdop_max: float = 1.4
    min_sats: int = 10
    # CROSS arrival tolerance, tighter than wp_radius_m: the gate must open
    # over the water, not near it.
    cross_radius_m: float = 0.5
    # How long a blind detector (dead worker or camera) is tolerated during
    # the phases where the camera still matters, before the mission aborts.
    detector_blind_s: float = 15.0
    # Arm attempts (12 x 5s ~ 60s total) for EKF/GPS settling.
    arm_retries: int = 12
    end_mode: str = 'RTL'
    # Base station launch argv (None = disabled).
    basestation_cmd: Optional[list] = None
    basestation_port: int = 8080   # checked to prevent double-launch


class Mission:
    # Terminal states: DONE (survey complete), STANDDOWN (pilot took the
    # aircraft), STOPPED (runner requested it, but end_mode is still
    # commanded, unlike STANDDOWN), NOARM (arming failed, so it never got
    # airborne and RTL is not sent to a disarmed vehicle), ABORTED (an
    # airborne failure, so end_mode is sent for safety). STOPPED stays
    # distinct from DONE so an interrupted flight never shows up as a
    # completed survey.
    TERMINAL = ('DONE', 'STANDDOWN', 'STOPPED', 'NOARM', 'ABORTED')

    def __init__(self, io, detector, dropper, cfg, log=print, recorder=None,
                 should_stop=None):
        self.io = io
        self.det = detector
        self.dropper = dropper
        self.cfg = cfg
        self.log = log
        self.rec = recorder if recorder is not None else _NoLog()
        # Callable that returns a truthy reason to stop, or None.
        self.should_stop = should_stop
        self.state = 'IDLE'
        self.history = []          # (t, state, note)
        self.wp_i = 0
        self.target = None         # latched (lat, lon): the descent point, beside
        self.puddle = None         # the water itself: where the gate opens
        self.target_area_m2 = None # estimated puddle area at latch time
        self.abort_reason = None
        self._retries_left = 0     # re-approaches remaining for this site
        # Altitude reference captured beside the water at drop height, used
        # for all over-water flight since the Luna reads nothing there.
        self._drop_rel_alt = None
        self._t_state = 0.0        # time of last state entry
        self._rng_acquired = False # ground return seen during current descent
        self._last_fix_gripe = None # so a poor fix is logged once, not per tick
        self._hold_since = None     # photo hold start, None when not holding
        self._t_dropped = None     # monotonic time the gate finished cycling
        self._drop_ok = True       # last gate actuation succeeded
        # Pilot-override test fires only after GUIDED has been observed in
        # a heartbeat: tel.mode is stale before takeoff, so it stays gated
        # until confirmed.
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
        """One telemetry line per second to the log; the log alone is then enough to replay a flight."""
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
        """Is the point at least margin metres inside the geofence? True when there is no fence."""
        poly = self.cfg.fence
        if not poly or len(poly) < 3:
            return True
        margin = self.cfg.fence_margin_m if margin is None else margin
        lat0, lon0 = poly[0]
        mlat = 111320.0
        mlon = mlat * math.cos(math.radians(lat0))
        if abs(mlon) < 1.0:
            # Within ~0.0005 deg of a pole the east-west scale collapses and
            # the containment test is meaningless. Said out loud rather than
            # quietly answering "inside".
            self.log("[mission] fence check DISABLED: this longitude scale "
                     "is degenerate (are these coordinates near a pole?)")
            return True
        pts = [((a - lat0) * mlat, (b - lon0) * mlon) for a, b in poly]
        p = ((lat - lat0) * mlat, (lon - lon0) * mlon)
        if not _inside(p[0], p[1], pts):
            return False
        return all(_seg_dist(p[0], p[1], *pts[i], *pts[(i + 1) % len(pts)])
                   >= margin for i in range(len(pts)))

    def _gps_ok(self):
        """Fix-quality check used before latch and drop. Returns (ok, why); a missing HDOP or sat count counts as OK."""
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
            # Arm result must be checked: end here if refused, reporting the
            # autopilot's own reason.
            arm_t = time.monotonic()
            if not io.arm(retries=cfg.arm_retries):
                why = io.tel.prearm_messages(since_t=arm_t - 5.0)
                self._set('NOARM', '; '.join(why) if why
                          else 'the autopilot refused to arm and gave no '
                               'reason')
            else:
                io.takeoff(cfg.survey_alt_m)
                self._set('TAKEOFF')
        else:
            # Every state handler below sends GUIDED-only commands, so arming
            # into an unconfirmed mode would take off and then ignore every
            # setpoint. NOARM is terminal: the loop below falls straight
            # through to the same epilogue any other ending gets.
            self._set('NOARM', 'GUIDED was accepted but never confirmed by a '
                               'heartbeat, so the aircraft would have ignored '
                               'every mission command; did not arm')

        while self.state not in self.TERMINAL:
            io.step()
            tel = io.tel
            self.rec.fix(tel, self.state)  # throttled inside the recorder
            self._tel_line(tel)

            if tel.mode == 'GUIDED':
                self._seen_guided = True

            # Pilot override: the mode got flipped out from under the
            # mission. Only meaningful once GUIDED has been seen, since
            # telemetry is stale before then.
            if (self.state != 'IDLE' and self._seen_guided
                    and tel.mode is not None and tel.mode != 'GUIDED'):
                self._set('STANDDOWN', f"mode={tel.mode}, pilot has aircraft")
                break

            # Blind-detector abort applies only during TAKEOFF/SURVEY/APPROACH;
            # after DESCEND the target is latched and the rangefinder
            # governs instead.
            blind = self.det.blind_for_s() if hasattr(self.det, 'blind_for_s') \
                else 0.0
            if (self.state in ('TAKEOFF', 'SURVEY', 'APPROACH')
                    and blind > self.cfg.detector_blind_s):
                self._set('ABORTED',
                          f"detector blind for {blind:.0f}s (worker or camera "
                          f"dead); ending rather than flying a blind survey")
                break

            # Runner stop request (signal, lost console), checked after the
            # pilot-override test but before the state handler, so a new
            # descent can't start on the way out.
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
        # Count successful gate cycles, not attempts: "drops: 3" means three
        # puddles treated.
        self.rec.mission_end(
            self.state,
            getattr(self.dropper, 'succeeded',
                    getattr(self.dropper, 'fired', None)))
        self._launch_basestation()
        return self.state

    def _launch_basestation(self):
        """Fire-and-forget launch; skips if one is already running (avoids EADDRINUSE)."""
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
            # ABORTED commands end_mode, so a partial climb goes to RTL,
            # not into a hover.
            self._set('ABORTED',
                      f"never reached {self.cfg.survey_alt_m:g} m in "
                      f"{self.cfg.takeoff_timeout_s:.0f}s "
                      f"(at {tel.rel_alt_m if tel.rel_alt_m is None else round(tel.rel_alt_m, 1)} m)")

    def _goto_current_wp(self):
        # Every arrival starts a fresh hold, whether the previous one timed
        # out or this waypoint is being re-entered.
        self._hold_since = None
        if self.wp_i >= len(self.cfg.waypoints):
            self._set('DONE', 'survey complete')
            return
        lat, lon = self.cfg.waypoints[self.wp_i]
        self.io.goto(lat, lon, self.cfg.survey_alt_m)
        self._set('SURVEY', f"wp {self.wp_i}")

    def dose_for(self, area_m2):
        """Dwell seconds for a given area. Returns (seconds, description); an unknown area gets the default, never the max."""
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
            # Latching is gated on fix quality, since observed wander can
            # exceed puddle size. A poor fix skips the site; it can be
            # re-found on a better fix later.
            ok, why = self._gps_ok()
            if not ok:
                if why != self._last_fix_gripe:
                    self.log(f"[mission] detection IGNORED, fix is poor: {why}")
                    self._last_fix_gripe = why
                # Recorded like the fence rejection below: a puddle seen
                # repeatedly under a bad fix otherwise leaves no trace at
                # all in the mission JSONL, which is the evidence record.
                self.rec.detection(det.lat, det.lon, det.confidence)
                self.det.unskip(det.lat, det.lon)   # re-offer on a better fix
                det = None
            else:
                self._last_fix_gripe = None
        if det is not None:
            # Check both the water and the descent point against the fence:
            # the swath is ~8 m either side, so a puddle at the frame edge
            # near the boundary can sit outside it.
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
                self.det.unskip(det.lat, det.lon)
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
            # A single leg times out (wind, avoidance, an unreachable point)
            # and gets skipped rather than the whole flight; logged so the
            # gap is never silent.
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
            # SET_POSITION_TARGET_GLOBAL_INT carries no ack, so a refused
            # destination looks identical to one still pending. Abandon the
            # site and resume the survey.
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

        # Case (a): acquired, then lost the ground.
        if self._rng_acquired and not fresh:
            self._abort('rangefinder dropout during descent')
            return
        # Case (b): low enough to see the ground, but doesn't. Grace period
        # first, since at a 5 m survey this state can begin already below
        # rng_expect_m.
        if (not self._rng_acquired and tel.rel_alt_m is not None
                and tel.rel_alt_m < cfg.rng_expect_m
                and self._elapsed() >= cfg.rng_grace_s):
            self._abort('no rangefinder acquisition by expected altitude')
            return
        if self._below_floor():
            # Altitude sources disagree: abort.
            self._abort('EKF floor hit without rangefinder drop condition')
            return
        if fresh and tel.rng_m <= cfg.drop_alt_m:
            ok, why = self._gps_ok()
            if not ok:
                # Warn rather than abort: height comes from the rangefinder
                # here, and the target was already latched on a good fix.
                # The seed may still land off centre, so the log records why.
                self.log(f"[mission] fix degraded at release ({why}); "
                         f"the drop position may be off")
            # Forced send, bypassing the rate limiter: this setpoint change
            # is critical. The gate itself opens in DROP once the aircraft
            # has stopped.
            self.io.velocity_ned(0, 0, 0, force=True)
            self._t_dropped = None
            # Last trustworthy AGL; all over-water flight uses this reference.
            self._drop_rel_alt = tel.rel_alt_m
            note = f"rng={tel.rng_m:.2f}m"
            if self._cross_m() <= cfg.wp_radius_m:
                self._set('DROP', note + ", stopping before release")
            elif self._drop_rel_alt is None:
                # No altitude reference: release beside the water rather
                # than crossing blind.
                self._set('DROP', note + ", no rel_alt: releasing beside the "
                                         "water rather than crossing blind")
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
        """Translate from beside the water to over it, holding the captured altitude.
        No rangefinder abort here: still water reads nothing, which used to be a false abort cause."""
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
        """Translate from over the water back beside it, where the Luna regains a ground target.
        Never aborts, since the granules are already released. On timeout it climbs from wherever it is."""
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
        """True once the descent is arrested, or the settle timeout is reached; the timeout is the bigger risk of the two."""
        if self._elapsed() >= self.cfg.settle_max_s:
            self.log(f"[mission] settle timed out after "
                     f"{self.cfg.settle_max_s:.1f}s, releasing anyway")
            return True
        vd = self.io.tel.vd_mps
        if vd is None:
            # No velocity feed: fall back to half the settle timeout and
            # trust the autopilot got there.
            return self._elapsed() >= self.cfg.settle_max_s / 2
        return abs(vd) <= self.cfg.settle_vd_mps

    def _abort(self, reason):
        self.abort_reason = reason
        tel = self.io.tel
        # Reverse the descent this tick, forced past the rate limiter:
        # waiting for a send slot would mean still sinking.
        self.io.velocity_ned(0, 0, -self.cfg.climb_mps, force=True)
        self.rec.abort(tel.lat, tel.lon, reason)
        self._set('ABORT_CLIMB', reason)

    def _st_drop(self):
        """Hold, stop, release, hold again. dropper.trigger() blocks for the dwell,
        so the state machine pauses there while the previous setpoint keeps being sent."""
        # Rate-limited: the forced send already happened on the
        # DESCEND->DROP transition, and sending on every tick would flood
        # the 115200 serial link with setpoints.
        self.io.velocity_ned(0, 0, 0)
        tel = self.io.tel

        if self._t_dropped is None:
            if self._below_floor():
                # Sank through the floor while settling: abort.
                self._abort('EKF floor hit while settling for release')
                return
            if not self._stopped():
                return
            dwell, why = self.dose_for(self.target_area_m2)
            self.log(f"[mission] dose {dwell:.2f}s ({why})")
            ok = self.dropper.trigger(dwell)
            self._t_dropped = time.monotonic()
            self._drop_ok = ok is not False
            # Record the drop position and whether it succeeded; a failed
            # gate must never be recorded as a site treated.
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
        timed_out = self._elapsed() >= cfg.climb_timeout_s
        if timed_out:
            self.log(f"[mission] {self.state} still short of "
                     f"{cfg.survey_alt_m:g} m after {cfg.climb_timeout_s:.0f}s "
                     f"(rel_alt {tel.rel_alt_m}); resuming anyway")
        if timed_out or (tel.rel_alt_m is not None
                and tel.rel_alt_m >= cfg.survey_alt_m - cfg.alt_tol_m):
            # An aborted descent is usually bad luck, not a bad site, and
            # the detector won't re-offer it since the coordinates are
            # already latched. Re-approach the same point until the
            # retries run out.
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
