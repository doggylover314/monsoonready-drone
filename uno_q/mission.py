"""Detect -> descend -> treat mission state machine (UNO Q onboard logic).

States:
  IDLE -> TAKEOFF -> SURVEY -> APPROACH -> DESCEND -> CROSS -> DROP -> RETURN
  -> CLIMB -> SURVEY ... -> DONE (RTL). DESCEND happens BESIDE the water,
  where the TF-Luna can range dry ground; CROSS translates over the puddle at
  the altitude captured there, and RETURN goes back beside it so the Luna has
  a target again before the climb. With both lateral offsets at 0 the two
  cross steps collapse and the sequence is the old vertical one.
  ABORT_CLIMB re-approaches the same site cfg.site_retries times before it
  gives up and re-joins SURVEY. Any external mode change
  away from GUIDED => STANDDOWN (pilot has the aircraft; never fight them).
  A should_stop() request from the runner => STOPPED, which still commands
  end_mode: nobody has taken the aircraft, we are just leaving.

Non-negotiable behaviors (PROJECT_STATE):
  * TARGET LATCHING: first detection at survey altitude locks the target
    lat/lon; detections are ignored outside SURVEY. No re-detection during
    descent (close-range frames unreliable, RUN1 spotcheck).
  * Rangefinder dropout during DESCEND => abort UPWARD, skip target, resume
    survey. "Dropout" = (a) reading was acquired then went stale/invalid, or
    (b) never acquired by rng_expect_m EKF altitude (sensor range is ~8m;
    the first part of a 15m descent is legitimately blind, losing the ground
    return once we are low is not). Never descend blind past that point
    (TF-Luna 850nm specular risk over still water).
  * No drop without a fresh, valid rangefinder reading at drop_alt_m, and no
    drop until the descent has actually ARRESTED (vd within settle_vd_mps).
    Commanding a stop is not the same as having stopped, and the gate dwell
    blocks the loop while whatever the autopilot last accepted continues.
  * EKF floor: rel_alt below (drop_alt_m - floor_margin_m) without the drop
    condition having fired => abort (altitude sources disagree).
"""

import socket
import subprocess
import time
from dataclasses import dataclass, field

from detector import offset_latlon, dist_m


def _port_in_use(port, host='127.0.0.1'):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.25)
        return s.connect_ex((host, port)) == 0


class _NoLog:
    """Recorder null-object: mission code calls recorder methods
    unconditionally; without a recorder they all no-op."""
    def __getattr__(self, _name):
        return lambda *a, **kw: None


@dataclass
class MissionConfig:
    waypoints: list = field(default_factory=list)  # [(lat, lon), ...]
    survey_alt_m: float = 15.0
    drop_alt_m: float = 3.0        # rangefinder AGL that triggers the drop
    descent_mps: float = 0.5
    climb_mps: float = 1.0
    wp_radius_m: float = 1.5
    alt_tol_m: float = 1.0
    rng_timeout_s: float = 1.0
    rng_expect_m: float = 6.0      # EKF alt by which rangefinder must have acquired
    floor_margin_m: float = 1.0
    drop_dwell_s: float = 2.0      # hold position AFTER the gate closes again
    # AREA-PROPORTIONAL DOSING. The gate is a fixed aperture, so dose is set
    # by how long it stays open: dwell = area_m2 * dose_s_per_m2, clamped.
    # The clamps are the safety net, because the area estimate is a bounding
    # box (it overestimates non-rectangular water) inheriting the FOV error
    # squared. An UNKNOWN area falls back to dose_s_default, never to the
    # maximum: over-dosing a puddle you cannot measure wastes the hopper on
    # one site and strands later ones untreated.
    dose_s_per_m2: float = 0.4
    dose_s_min: float = 0.3
    dose_s_max: float = 3.0
    dose_s_default: float = 1.0    # used when the area is unknown
    # The gate must not open while the aircraft is still sinking, or the
    # granules leave from below the intended release height. DROP commands a
    # stop and waits for the autopilot to actually achieve it, bounded by
    # settle_max_s so a missing or noisy velocity feed can never hang a
    # descent (drop anyway at that point: we are stopped-commanded and
    # holding, which is the same place the old code dropped from).
    settle_vd_mps: float = 0.15
    settle_max_s: float = 3.0
    # DESCEND BESIDE, CROSS OVER, RELEASE, CROSS BACK (user, 2026-08-17).
    # The TF-Luna cannot range still water, so the descent happens over dry
    # ground this far from the puddle centre, where the rangefinder works
    # normally. The AGL it reports at drop height is the honest one; the
    # aircraft then translates over the water HOLDING THE REL_ALT IT HAD AT
    # THAT MOMENT, releases, and translates back beside so the Luna has a
    # target again before the climb. Nothing aborts on a rangefinder dropout
    # while over the water, because a dropout there is the expected reading.
    # Direction is a site choice, not a physical constant: north by default,
    # override with run_mission's --offset-n / --offset-e when the ground
    # beside the water is only good on one side. Set both to 0 and the cross
    # steps collapse to nothing, which is the old descend-directly-overhead
    # behaviour and is what the SITL drills still exercise.
    lateral_offset_n_m: float = 3.0
    lateral_offset_e_m: float = 0.0
    cross_timeout_s: float = 20.0  # per translate; then abort (out) or climb (back)
    # An abort used to burn the site for the whole flight: the detector latches
    # a puddle when it FIRES, so an aborted descent left a site that could
    # never be retried, and a one-target flight ended with zero drops. The
    # mission now re-approaches the SAME latched coordinates this many times
    # before giving up, which needs no detector change because the coordinates
    # are already held here.
    site_retries: int = 1
    end_mode: str = 'RTL'
    # argv to launch the base station after the mission ends (None = don't).
    # The onboard runner sets this; SITL tests leave it off.
    basestation_cmd: list = None
    basestation_port: int = 8080   # checked first, so we never double-launch


class Mission:
    # Terminal states. STOPPED = the runner asked us to wind up (signal, ssh
    # drop, operator Ctrl+C); unlike STANDDOWN the pilot has NOT taken the
    # aircraft, so end_mode still gets commanded. Kept distinct from DONE so
    # the log and the dashboard never show an interrupted flight as a
    # completed survey.
    TERMINAL = ('DONE', 'STANDDOWN', 'STOPPED')

    def __init__(self, io, detector, dropper, cfg, log=print, recorder=None,
                 should_stop=None):
        self.io = io
        self.det = detector
        self.dropper = dropper
        self.cfg = cfg
        self.log = log
        self.rec = recorder if recorder is not None else _NoLog()
        # Returns a truthy reason string to wind the mission up, or None.
        self.should_stop = should_stop
        self.state = 'IDLE'
        self.history = []          # (t, state, note)
        self.wp_i = 0
        self.target = None         # latched (lat, lon): the DESCENT point, beside
        self.puddle = None         # the water itself: where the gate opens
        self.target_area_m2 = None # estimated puddle area at latch time
        self.abort_reason = None
        self._retries_left = 0     # re-approaches remaining for this site
        # Rel-alt captured beside the water at the moment the Luna said we
        # were at drop height. It is the altitude reference for everything
        # that happens over the water, where the Luna reads nothing.
        self._drop_rel_alt = None
        self._t_state = 0.0        # time of last state entry
        self._rng_acquired = False # ground return seen during current descent
        self._t_dropped = None     # monotonic time the gate finished cycling
        self._drop_ok = True       # did the last gate actuation report success
        # The pilot-override test may only fire once GUIDED has actually
        # been OBSERVED in a heartbeat. tel.mode carries the pre-takeoff
        # mode until the next 1 Hz heartbeat lands, so an ungated test reads
        # a stale STABILIZE on the first loop pass and stands down before
        # the survey begins. Belt and braces with set_mode's own confirm.
        self._seen_guided = False
        # Test hook (dropout drill): below this rel_alt, pretend the
        # rangefinder went silent. None = disabled.
        self.rng_suppress_below_m = None
        self._last_tel_log = 0.0   # 1 Hz telemetry line (SCOPE RULES 1)

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
        """One telemetry snapshot per second into the log (SCOPE RULES 1):
        enough to replay any flight from the log alone. The farm day could
        not be reconstructed because nothing recorded what the aircraft was
        doing between events."""
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

    def _at_wp(self, lat, lon, alt=None):
        tel = self.io.tel
        if tel.lat is None:
            return False
        if dist_m(tel.lat, tel.lon, lat, lon) > self.cfg.wp_radius_m:
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
        io.arm()
        io.takeoff(cfg.survey_alt_m)
        self._set('TAKEOFF')

        while self.state not in self.TERMINAL:
            io.step()
            tel = io.tel
            self.rec.fix(tel, self.state)  # throttled inside the recorder
            self._tel_line(tel)

            if tel.mode == 'GUIDED':
                self._seen_guided = True

            # Pilot override: someone flipped the mode from under us. Only
            # meaningful once GUIDED has been seen at least once (see
            # _seen_guided); before that a non-GUIDED reading is stale
            # telemetry, not a pilot.
            if (self.state != 'IDLE' and self._seen_guided
                    and tel.mode is not None and tel.mode != 'GUIDED'):
                self._set('STANDDOWN', f"mode={tel.mode}, pilot has aircraft")
                break

            # Runner asked us to wind up (signal / lost console). Checked
            # AFTER the pilot test and BEFORE the state handler so we never
            # start a new descent on the way out.
            if self.should_stop is not None:
                why = self.should_stop()
                if why:
                    self._set('STOPPED', str(why))
                    break

            getattr(self, '_st_' + self.state.lower())()

        if self.state in ('DONE', 'STOPPED'):
            confirmed = io.set_mode(cfg.end_mode)
            self.log(f"[mission] {self.state.lower()}, {cfg.end_mode} "
                     + ("confirmed" if confirmed
                        else "ACCEPTED but NOT confirmed by heartbeat, WATCH "
                             "THE AIRCRAFT and be ready on the sticks"))
        # Count SUCCESSFUL gate cycles, not attempts: "drops: 3" on a judged
        # dashboard has to mean three puddles got Bti.
        self.rec.mission_end(
            self.state,
            getattr(self.dropper, 'succeeded',
                    getattr(self.dropper, 'fired', None)))
        self._launch_basestation()
        return self.state

    def _launch_basestation(self):
        """Fire-and-forget: the base station outlives the mission process.

        Skipped if something already answers on its port, because a second
        instance just dies on EADDRINUSE and the traceback lands in whatever
        log the judge is least likely to be looking at.
        """
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

    def _goto_current_wp(self):
        if self.wp_i >= len(self.cfg.waypoints):
            self._set('DONE', 'survey complete')
            return
        lat, lon = self.cfg.waypoints[self.wp_i]
        self.io.goto(lat, lon, self.cfg.survey_alt_m)
        self._set('SURVEY', f"wp {self.wp_i}")

    def dose_for(self, area_m2):
        """Seconds to hold the gate open for a puddle of this area.

        Returns (seconds, description). Unknown area is NOT treated as large.
        """
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
            # TARGET LATCH: lock now, at survey altitude; ignore later detections.
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
            self.wp_i += 1
            self._goto_current_wp()

    def _st_approach(self):
        if self._at_wp(*self.target, alt=self.cfg.survey_alt_m):
            self._rng_acquired = False
            self._set('DESCEND')

    def _st_descend(self):
        cfg, tel = self.cfg, self.io.tel
        fresh = self._rng_fresh()
        if fresh:
            self._rng_acquired = True

        # Dropout case (a): had the ground, lost it.
        if self._rng_acquired and not fresh:
            self._abort('rangefinder dropout during descent')
            return
        # Dropout case (b): low enough that the sensor must see ground, doesn't.
        if (not self._rng_acquired and tel.rel_alt_m is not None
                and tel.rel_alt_m < cfg.rng_expect_m):
            self._abort('no rangefinder acquisition by expected altitude')
            return
        if self._below_floor():
            # Altitude sources disagree => never keep descending.
            self._abort('EKF floor hit without rangefinder drop condition')
            return
        if fresh and tel.rng_m <= cfg.drop_alt_m:
            # Commanded stop goes out FORCED: this is a setpoint change, and
            # the rate limiter must not be allowed to eat it (see
            # mavlink_io.velocity_ned). The gate itself opens in _st_drop,
            # once the aircraft has actually stopped.
            self.io.velocity_ned(0, 0, 0, force=True)
            self._t_dropped = None
            # THE LAST TRUSTWORTHY AGL. Everything over the water is flown
            # against this number, because the Luna will read nothing there.
            self._drop_rel_alt = tel.rel_alt_m
            note = f"rng={tel.rng_m:.2f}m"
            if self._cross_m() <= cfg.wp_radius_m:
                self._set('DROP', note + ", stopping before release")
            elif self._drop_rel_alt is None:
                # No altitude reference means no safe way to hold height over
                # water. Release beside it rather than fly blind across.
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
        """Ground distance from the descent point to the water itself."""
        if self.puddle is None or self.target is None:
            return 0.0
        return dist_m(self.target[0], self.target[1],
                      self.puddle[0], self.puddle[1])

    def _st_cross(self):
        """Beside the water -> over its centre, holding the captured altitude.

        No rangefinder test here on purpose: still water returns nothing, and
        treating that as a fault is exactly the abort that used to end the
        flight with zero drops.
        """
        if self._at_wp(*self.puddle, alt=self._drop_rel_alt):
            self.io.velocity_ned(0, 0, 0, force=True)
            self._t_dropped = None
            self._set('DROP', 'over the water, stopping before release')
            return
        if self._elapsed() >= self.cfg.cross_timeout_s:
            self._abort(f'did not reach the water in '
                        f'{self.cfg.cross_timeout_s:.0f}s')

    def _st_return(self):
        """Over the water -> back beside it, so the Luna has ground again.

        Never aborts: the granules are already gone and the only thing left to
        do is get out. A timeout here just climbs from where it is.
        """
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
        """True once the descent has actually arrested, or we have waited
        long enough that continuing to wait is the bigger risk."""
        if self._elapsed() >= self.cfg.settle_max_s:
            self.log(f"[mission] settle timed out after "
                     f"{self.cfg.settle_max_s:.1f}s, releasing anyway")
            return True
        vd = self.io.tel.vd_mps
        if vd is None:
            # No velocity feed: fall back to giving the autopilot a fixed
            # window to act on the stop command.
            return self._elapsed() >= self.cfg.settle_max_s / 2
        return abs(vd) <= self.cfg.settle_vd_mps

    def _abort(self, reason):
        self.abort_reason = reason
        tel = self.io.tel
        # Reverse the descent on THIS tick, forced past the rate limiter. An
        # abort that waits up to 0.2 s for its setpoint slot is still
        # descending during the part of the flight the abort exists for.
        self.io.velocity_ned(0, 0, -self.cfg.climb_mps, force=True)
        self.rec.abort(tel.lat, tel.lon, reason)
        self._set('ABORT_CLIMB', reason)

    def _st_drop(self):
        """Hold, stop, release, hold again.

        Split in two because dropper.trigger() BLOCKS for the gate dwell: for
        that whole second the state machine is not running, so whatever
        setpoint the autopilot last accepted is what the aircraft is doing.
        It had better be "stop".
        """
        # Rate-limited on purpose: the forced send that MATTERS already went
        # out on the DESCEND->DROP transition. Forcing every tick here would
        # push setpoints at the pump rate (~50 Hz) down a 115200 serial link
        # shared with telemetry.
        self.io.velocity_ned(0, 0, 0)
        tel = self.io.tel

        if self._t_dropped is None:
            if self._below_floor():
                # Sank through the floor while settling: no release, climb.
                self._abort('EKF floor hit while settling for release')
                return
            if not self._stopped():
                return
            dwell, why = self.dose_for(self.target_area_m2)
            self.log(f"[mission] dose {dwell:.2f}s ({why})")
            ok = self.dropper.trigger(dwell)
            self._t_dropped = time.monotonic()
            self._drop_ok = ok is not False
            # Record where the gate ACTUALLY opened, and whether it did. A
            # dropper that failed must not leave a "treated" mark on the map
            # (it would also halve predict.py's revisit score for a site that
            # never got any Bti).
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
            # RETRY BEFORE GIVING UP. An aborted descent is usually a moment
            # of bad luck (a dropout, a gust), not a verdict on the site, and
            # the detector will never offer this puddle again: it latched the
            # coordinates when it fired. Re-approach the SAME point, and only
            # abandon it once the retries are spent.
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
