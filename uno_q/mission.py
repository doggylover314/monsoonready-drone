"""Detect -> descend -> treat mission state machine (UNO Q onboard logic).

States:
  IDLE -> TAKEOFF -> SURVEY -> APPROACH -> DESCEND -> DROP -> CLIMB -> SURVEY
  ... -> DONE (RTL). ABORT_CLIMB re-joins SURVEY. Any external mode change
  away from GUIDED => STANDDOWN (pilot has the aircraft; never fight them).

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
  * No drop without a fresh, valid rangefinder reading at drop_alt_m.
  * EKF floor: rel_alt below (drop_alt_m - floor_margin_m) without the drop
    condition having fired => abort (altitude sources disagree).
"""

import time
from dataclasses import dataclass, field

from detector import offset_latlon, dist_m


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
    drop_dwell_s: float = 2.0
    # descend-beside knobs; stay 0 until TF-Luna-over-water bench (TODO 6)
    lateral_offset_n_m: float = 0.0
    lateral_offset_e_m: float = 0.0
    end_mode: str = 'RTL'


class Mission:
    def __init__(self, io, detector, dropper, cfg, log=print):
        self.io = io
        self.det = detector
        self.dropper = dropper
        self.cfg = cfg
        self.log = log
        self.state = 'IDLE'
        self.history = []          # (t, state, note)
        self.wp_i = 0
        self.target = None         # latched (lat, lon)
        self.abort_reason = None
        self._t_state = 0.0        # time of last state entry
        self._rng_acquired = False # ground return seen during current descent
        # Test hook (dropout drill): below this rel_alt, pretend the
        # rangefinder went silent. None = disabled.
        self.rng_suppress_below_m = None

    # ---------- helpers ----------

    def _set(self, state, note=''):
        self.log(f"[mission] {self.state} -> {state}"
                 + (f" ({note})" if note else ""))
        self.state = state
        self._t_state = time.monotonic()
        self.history.append((self._t_state, state, note))

    def _elapsed(self):
        return time.monotonic() - self._t_state

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
        io.set_mode('GUIDED')
        io.arm()
        io.takeoff(cfg.survey_alt_m)
        self._set('TAKEOFF')

        while self.state not in ('DONE', 'STANDDOWN'):
            io.step()
            tel = io.tel

            # Pilot override: someone flipped the mode from under us.
            if (self.state != 'IDLE' and tel.mode is not None
                    and tel.mode != 'GUIDED'):
                self._set('STANDDOWN', f"mode={tel.mode}, pilot has aircraft")
                break

            getattr(self, '_st_' + self.state.lower())()

        if self.state == 'DONE':
            io.set_mode(cfg.end_mode)
            self.log(f"[mission] complete, {cfg.end_mode} set")
        return self.state

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

    def _st_survey(self):
        det = self.det.poll(self.io.tel)
        if det is not None:
            # TARGET LATCH: lock now, at survey altitude; ignore later detections.
            self.target = offset_latlon(det.lat, det.lon,
                                        self.cfg.lateral_offset_n_m,
                                        self.cfg.lateral_offset_e_m)
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
        if fresh and tel.rng_m <= cfg.drop_alt_m:
            self.io.velocity_ned(0, 0, 0)
            self.dropper.trigger()
            self._set('DROP', f"rng={tel.rng_m:.2f}m")
            return
        # Altitude sources disagree => never keep descending.
        if (tel.rel_alt_m is not None
                and tel.rel_alt_m < cfg.drop_alt_m - cfg.floor_margin_m):
            self._abort('EKF floor hit without rangefinder drop condition')
            return
        self.io.velocity_ned(0, 0, +cfg.descent_mps)

    def _abort(self, reason):
        self.abort_reason = reason
        self._set('ABORT_CLIMB', reason)

    def _st_drop(self):
        self.io.velocity_ned(0, 0, 0)
        if self._elapsed() >= self.cfg.drop_dwell_s:
            self._set('CLIMB', 'treated')

    def _st_climb(self):
        self._climb_then_resume()

    def _st_abort_climb(self):
        self._climb_then_resume()

    def _climb_then_resume(self):
        cfg, tel = self.cfg, self.io.tel
        if (tel.rel_alt_m is not None
                and tel.rel_alt_m >= cfg.survey_alt_m - cfg.alt_tol_m):
            self.target = None
            self._goto_current_wp()   # resume survey where we left off
            return
        self.io.velocity_ned(0, 0, -cfg.climb_mps)

    def _st_done(self):
        pass

    def _st_standdown(self):
        pass

    def _st_idle(self):
        pass
