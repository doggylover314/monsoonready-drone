"""Payload drop actuation. The servo is an MG90 (metal gear).

SERVO POWER: the XY-3606 buck, set to 5.00 V by meter. Never the Pixhawk rail
and never the UNO Q rail, so a stalled or stripped gate cannot brown out
anything that flies the aircraft. This has never been in question and is not
what the PWM-source decision below is about.

PWM SOURCE = PIXHAWK OUTPUT PIN. Settled 2026-07-31, re-examined 2026-08-01
when the user asked for the signal to come from the UNO Q instead. It still
comes from the Pixhawk, and here is the whole reason, so nobody has to take
this on trust:

  1. A hobby servo holds position from a 50 Hz pulse whose width it reads to
     roughly a microsecond. Linux is not a real-time OS, so a userspace
     soft-PWM thread gets descheduled at the kernel's convenience and every
     hiccup becomes a pulse-width error the servo reads as "move". Driving
     the gate from the UNO Q's Linux side is therefore out on its own.
  2. That leaves the UNO Q's STM32 side, which does have hardware timers. But
     the UNO Q runs a Zephyr core, and the stock Arduino Servo library does
     not support Zephyr (verified 2026-07-31). Using the STM32 means writing
     a board-specific PWM implementation.
  3. And getting a command to that implementation means the Linux->STM32
     Bridge, which is TODO 12 and BLOCKED: the UNO Q docs contradict
     themselves about whether Serial1 (D0/D1) is free or claimed by the
     router, and it is not resolvable without bench time on the board.
  4. The Pixhawk, meanwhile, already has hardware PWM outputs built for
     exactly this, already powered, already wired, already talking MAVLink to
     us, and with MAIN 1-6 taken by the six motors an AUX output is free. The
     AUX5-6 NODMA limitation blocks bidirectional DShot only, not ordinary
     PWM servo output.

So the UNO Q route is two unwritten pieces of firmware and one unresolved
hardware question, against zero new code for the Pixhawk route. It is also
the only one of the two whose signal level is known good: Pixhawk servo rails
are 5 V logic, while the UNO Q's GPIO is 3.3 V and no datasheet consulted so
far states an MG90 input-high threshold (VERIFY if the decision is ever
revisited; "it usually works" is not a spec).

If the Pixhawk output turns out to be unavailable, ServoDropper at the bottom
of this file is the fallback shape, still blocked on points 2 and 3.
"""

import time

from mavlink_io import PWM_MIN_US, PWM_MAX_US


class Dropper:
    def trigger(self, dwell_s=None):
        raise NotImplementedError


class LogDropper(Dropper):
    """SITL/bench stand-in: records and logs instead of moving a servo."""

    def __init__(self, log=print):
        self.fired = 0
        self.succeeded = 0
        self.times = []
        self._log = log

    def trigger(self, dwell_s=None):
        self.fired += 1
        self.succeeded += 1
        self.times.append(time.monotonic())
        self.dwells = getattr(self, 'dwells', [])
        self.dwells.append(dwell_s)
        self._log(f"[dropper] TRIGGER #{self.fired} (simulated"
                  + (f", dwell {dwell_s:.2f}s)" if dwell_s else ")"))
        return True


def _default_bridge_call():
    """Resolve the Bridge call on the board, or explain why it failed.

    Imported lazily so that importing this module on a laptop (SITL, tests)
    never needs the Arduino runtime present.
    """
    try:
        from arduino.app_utils import Bridge
    except ImportError as exc:      # pragma: no cover - board-only path
        raise RuntimeError(
            "arduino.app_utils not importable: ServoDropper needs the UNO Q "
            "Arduino runtime, or an explicit bridge_call=. Original: "
            f"{exc}") from exc
    return lambda method, *args: Bridge.call(method, *args)


class PixhawkServoDropper(Dropper):
    """MG90 gate driven by a Pixhawk servo output. RECOMMENDED.

    See the module docstring for why the pulse comes from the Pixhawk and not
    the UNO Q. Cost: one Pixhawk output. Motors occupy MAIN 1-6 on this hexa,
    so an AUX output is free.

    SETUP (tools/parameters.py push, then reboot):
        SERVO<channel>_FUNCTION = 0     Disabled, so DO_SET_SERVO controls it
        SERVO<channel>_MIN / _MAX       bracket closed_us and open_us

    Wiring: servo SIGNAL from the Pixhawk output pin, servo POWER from the
    XY-3606 buck, grounds common.

    closed_us/open_us come from the bench flow test (TODO 3), not from
    guesswork: 1000-2000us is the conventional full range, but the useful
    open angle is whatever passes granules without the gate fouling.

    FAILURE POLICY, and the two halves are deliberately different:
      * On the ground, in __init__, a failed gate command RAISES. That close
        is the pre-arm proof that the whole chain works: the parameter, the
        channel, the wire, the servo. Swallowing it means flying an entire
        survey with a dead dropper and logging every puddle as treated.
      * In the air, in trigger(), a failed gate command NEVER raises, it
        returns False. A missed puddle is a missed puddle; an exception
        thrown into the state machine mid-descent is a runaway.
    """

    # GATE TRAVEL, measured on the bench 2026-08-10 and then REVERSED at the
    # user's instruction. Observation: 1000us -> 1900us (900us) swung the horn
    # 90 degrees CLOCKWISE, which implies roughly 10us per degree ON THIS
    # SERVO (a ratio, not a datasheet figure: MG90 travel per microsecond
    # varies by unit, so treat it as calibration, not spec). The gate must
    # instead open 60 degrees COUNTER-CLOCKWISE, so open has to sit BELOW
    # closed in pulse width, 60 * 10 = 600us below it.
    #   closed 1600us  ->  open 1000us   = 600us = ~60 deg CCW
    # Both stay inside the 800-2200us guard. VERIFY BY EYE on the bench: if
    # the throw is not 60 degrees, adjust DEG_PER_US rather than guessing new
    # pulse numbers, and if it turns the wrong way swap these two values.
    US_PER_DEG = 10.0
    DEFAULT_CLOSED_US = 1600
    DEFAULT_OPEN_US = 1000

    def __init__(self, io, channel=9, closed_us=DEFAULT_CLOSED_US,
                 open_us=DEFAULT_OPEN_US,
                 dwell_s=1.0, log=print, sleep=time.sleep):
        for name, v in (('closed_us', closed_us), ('open_us', open_us)):
            if not PWM_MIN_US <= v <= PWM_MAX_US:
                raise ValueError(
                    f"{name}={v} outside {PWM_MIN_US}-{PWM_MAX_US}us. Checked "
                    f"here rather than at send time because an out-of-range "
                    f"open_us only shows up as a mission that never drops, "
                    f"and an in-range closed_us hides it completely.")
        if closed_us == open_us:
            raise ValueError(
                f"closed_us and open_us are both {closed_us}: the gate would "
                f"never move")
        self.io = io
        self.channel = channel
        self.closed_us = closed_us
        self.open_us = open_us
        self.dwell_s = dwell_s
        self.log = log
        self._sleep = sleep
        self.fired = 0          # gate cycles attempted
        self.succeeded = 0      # gate cycles the autopilot accepted
        self.times = []
        self.gate_open = False  # best known gate state
        # Close on construction: a gate left open by a crashed run should be
        # shut on the ground, not discovered open over a puddle. This doubles
        # as the pre-arm end-to-end test, so it is allowed to fail loudly.
        if self._safe(self.closed_us, 'initial close') is None:
            raise RuntimeError(
                f"dropper pre-arm close FAILED on servo channel {channel}. "
                f"Refusing to fly a dropper that has not proved it works. "
                f"Check SERVO{channel}_FUNCTION=0, SERVO{channel}_MIN/_MAX, "
                f"the signal wire, and that the MG90 has 5V from the XY-3606. "
                f"Use --no-drop to fly the survey without a dropper.")

    def _safe(self, pwm_us, why):
        """Send a gate command; return the result, or None if it failed.

        Never raises: see the failure policy in the class docstring. The one
        caller allowed to treat None as fatal is __init__, on the ground.
        """
        try:
            return self.io.set_servo(self.channel, pwm_us)
        except Exception as exc:                      # noqa: BLE001
            self.log(f"[dropper] set_servo({self.channel}, {pwm_us}) "
                     f"FAILED ({why}): {exc}")
            return None

    def trigger(self, dwell_s=None):
        """Open, dwell, close. Returns True only if the gate actually opened.

        dwell_s overrides the configured dwell for THIS drop, which is how a
        bigger puddle gets a bigger dose: the gate is a fixed aperture, so the
        only quantity available to vary is how long it stays open. Dose is
        therefore proportional to time only if the granule flow rate is
        constant, which is exactly what the TODO 3 bench test measures and
        which has NOT been measured yet: until it has, the numbers below are
        proportional, not calibrated in grams.
        """
        self.fired += 1
        self.times.append(time.monotonic())
        dwell = self.dwell_s if dwell_s is None else max(0.05, float(dwell_s))
        self.log(f"[dropper] TRIGGER #{self.fired}: gate -> {self.open_us}us "
                 f"for {dwell:.2f}s")

        opened = self._safe(self.open_us, 'open')
        if opened is None:
            self.log("[dropper] gate did NOT open: this site is UNTREATED")
            return False
        self.gate_open = True
        self.succeeded += 1

        self._sleep(dwell)

        if self._safe(self.closed_us, 'close') is None:
            # The payload went out, so the drop itself succeeded, but a gate
            # stuck open empties the hopper into one puddle. Say so loudly;
            # the operator can still land on the pilot's mode switch.
            self.log(f"[dropper] WARNING gate may still be OPEN on channel "
                     f"{self.channel}: hopper is draining, land soon")
        else:
            self.gate_open = False
        return True


class ServoDropper(Dropper):
    """MG90 gate on the hopper, actuated through the STM32 over the Bridge.

    ALTERNATIVE to PixhawkServoDropper, kept for the case where the Pixhawk
    output is unavailable. BLOCKED on two things, both open: a
    Zephyr-compatible PWM implementation in the sketch (the stock Servo
    library refuses to build for this core), and TODO 12, which is whether
    the Bridge leaves D0/D1 usable at all.

    The sketch WOULD provide two RPC methods. It does not exist yet; there is
    no uno_q/sketch_bridge/ in this repo and nothing here is exercised by any
    test. Treat the signature below as the design, not as an interface you
    can call today:
        servo_set(angle)  -> int   move the gate, returns the angle applied
        servo_detach()    -> int   stop pulsing, so the servo stops holding

    Signal level is also unresolved for this route: UNO Q GPIO is 3.3 V and
    no consulted datasheet states an MG90 input-high threshold. VERIFY on a
    scope before trusting it.

    open_deg/closed_deg are the two gate positions and MUST be set from the
    bench flow test (TODO 3), not guessed: the right open angle is whatever
    passes granules without the gate fouling the tube, and it depends on how
    the hatch was built.

    dwell_s is how long the gate stays open. It comes from the same flow test:
    long enough to pass the intended dose, short enough not to empty the
    hopper into one puddle. mission.py independently holds position for
    drop_dwell_s, which should be >= this.

    detach_after: stop the pulse train once the gate is closed again. A
    hobby servo under continuous command fights every nudge and draws current
    all flight; releasing it saves power and heat on a gate that only has to
    hold a light flap shut.
    """

    def __init__(self, open_deg=90, closed_deg=0, dwell_s=1.0,
                 bridge_call=None, detach_after=True, log=print,
                 sleep=time.sleep):
        for name, v in (('open_deg', open_deg), ('closed_deg', closed_deg)):
            if not 0 <= v <= 180:
                raise ValueError(f"{name} must be 0..180, got {v}")
        self.open_deg = open_deg
        self.closed_deg = closed_deg
        self.dwell_s = dwell_s
        self.detach_after = detach_after
        self.log = log
        self._sleep = sleep
        self._call = bridge_call if bridge_call is not None else _default_bridge_call()
        self.fired = 0
        self.succeeded = 0
        self.times = []
        # Close on construction: if the gate was left open by a crashed run,
        # the first thing a new mission should do is shut it, on the ground,
        # rather than discover it open over a puddle.
        self._safe('servo_set', self.closed_deg, why='initial close')
        if self.detach_after:
            self._safe('servo_detach', why='initial detach')

    def _safe(self, method, *args, why=''):
        """Bridge calls must never take the mission down. A failed drop is a
        missed puddle; an exception in the state machine is a runaway."""
        try:
            return self._call(method, *args)
        except Exception as exc:                      # noqa: BLE001
            self.log(f"[dropper] {method}{args} FAILED ({why}): {exc}")
            return None

    def trigger(self, dwell_s=None):
        self.fired += 1
        self.times.append(time.monotonic())
        dwell = self.dwell_s if dwell_s is None else max(0.05, float(dwell_s))
        self.log(f"[dropper] TRIGGER #{self.fired}: gate -> {self.open_deg}deg "
                 f"for {dwell:.2f}s")
        opened = self._safe('servo_set', self.open_deg, why='open')
        if opened is None:
            self.log("[dropper] gate did NOT open: this site is UNTREATED")
            return False
        self.succeeded += 1
        self._sleep(dwell)
        self._safe('servo_set', self.closed_deg, why='close')
        if self.detach_after:
            self._safe('servo_detach', why='release')
        return True
