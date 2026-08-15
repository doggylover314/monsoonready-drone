"""Detect -> descend -> treat mission state machine (UNO Q onboard logic).

States:
  IDLE -> TAKEOFF -> SURVEY -> APPROACH -> DESCEND -> DROP -> CLIMB -> SURVEY
  ... -> DONE (RTL). ABORT_CLIMB re-joins SURVEY. Any external mode change
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
    # descend-beside knobs; stay 0 until TF-Luna-over-water bench (TODO 6)
    lateral_offset_n_m: float = 0.0
    lateral_offset_e_m: float = 0.0
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
        self.target = None         # latched (lat, lon)
        self.target_area_m2 = None # estimated puddle area at latch time
        self.abort_reason = None
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
            self.target = offset_latlon(det.lat, det.lon,
                                        self.cfg.lateral_offset_n_m,
                                        self.cfg.lateral_offset_e_m)
            self.target_area_m2 = getattr(det, 'area_m2', None)
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
            self._set('DROP', f"rng={tel.rng_m:.2f}m, stopping before release")
            return
        self.io.velocity_ned(0, 0, +cfg.descent_mps)

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
            self._set('CLIMB', 'treated' if self._drop_ok else 'drop failed')

    def _st_climb(self):
        self._climb_then_resume()

    def _st_abort_climb(self):
        self._climb_then_resume()

    def _climb_then_resume(self):
        cfg, tel = self.cfg, self.io.tel
        if (tel.rel_alt_m is not None
                and tel.rel_alt_m >= cfg.survey_alt_m - cfg.alt_tol_m):
            self.target = None
            self.target_area_m2 = None
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
