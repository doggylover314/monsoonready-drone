"""Payload drop actuation.

PWM SOURCE DECISION (TODO 12, settled 2026-07-31): the SG90 is driven from the
STM32 side, not from Linux GPIO.

Reasoning. A hobby servo holds position from a 50 Hz pulse whose width it reads
to roughly 1 microsecond. Linux is not a real-time OS: a userspace soft-PWM
thread gets descheduled whenever the kernel feels like it, and every hiccup
becomes a pulse-width error the servo interprets as "move". The visible result
is a gate that twitches, buzzes, and draws current it should not, on an
aircraft whose 5 V rail also feeds the flight-critical companion computer. The
STM32 generates the pulse train in hardware timers and never varies.

The Linux side therefore only says "open" or "close"; the sketch owns the
timing. That call crosses the Bridge (Arduino_RouterBridge on the sketch side,
the arduino-router service on Linux). See uno_q/sketch_bridge/.

VERIFY ON HARDWARE: the exact Linux-side import for a plain ssh-run script
(as opposed to an App Lab application) is the one part of this not yet
confirmed on the board. `bridge_call` is injectable precisely so that fixing
it is a one-line change at the call site rather than an edit to this class.
"""

import time


class Dropper:
    def trigger(self):
        raise NotImplementedError


class LogDropper(Dropper):
    """SITL/bench stand-in: records and logs instead of moving a servo."""

    def __init__(self, log=print):
        self.fired = 0
        self.times = []
        self._log = log

    def trigger(self):
        self.fired += 1
        self.times.append(time.monotonic())
        self._log(f"[dropper] TRIGGER #{self.fired} (simulated)")


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
    """SG90 gate driven by a Pixhawk servo output. RECOMMENDED.

    Why this rather than the UNO Q: the flight controller already has
    hardware PWM outputs built for exactly this, already powered, already
    wired, and already talking to us. The UNO Q route needs the STM32 to
    generate the pulse train, and the stock Arduino Servo library does not
    support Zephyr boards (verified 2026-07-31), so it would mean a
    board-specific PWM implementation to solve a problem the Pixhawk does
    not have.

    Cost: one Pixhawk output. Motors occupy MAIN 1-6 on this hexa, so an AUX
    output is free. The NODMA limitation on AUX5-6 that blocked bidirectional
    DShot does not affect ordinary PWM servo output.

    SETUP (tools/push_params.py, then reboot):
        SERVO<channel>_FUNCTION = 0     Disabled, so DO_SET_SERVO controls it
        SERVO<channel>_MIN / _MAX       bracket closed_us and open_us

    Wiring change from the original plan: the servo SIGNAL comes from the
    Pixhawk output pin instead of a UNO Q GPIO. Servo POWER stays on the
    XY-3606 buck, exactly as before, so a stalled servo still cannot brown
    out anything that matters. Grounds already common.

    closed_us/open_us come from the bench flow test (TODO 3), not from
    guesswork: 1000-2000us is the conventional full range, but the useful
    open angle is whatever passes granules without the gate fouling.
    """

    def __init__(self, io, channel=9, closed_us=1000, open_us=1900,
                 dwell_s=1.0, log=print, sleep=time.sleep):
        self.io = io
        self.channel = channel
        self.closed_us = closed_us
        self.open_us = open_us
        self.dwell_s = dwell_s
        self.log = log
        self._sleep = sleep
        self.fired = 0
        self.times = []
        # Close on construction: a gate left open by a crashed run should be
        # shut on the ground, not discovered open over a puddle.
        self._safe(self.closed_us, 'initial close')

    def _safe(self, pwm_us, why):
        """A failed drop is a missed puddle; an exception inside the state
        machine is a runaway. Never let one become the other."""
        try:
            return self.io.set_servo(self.channel, pwm_us)
        except Exception as exc:                      # noqa: BLE001
            self.log(f"[dropper] set_servo({self.channel}, {pwm_us}) "
                     f"FAILED ({why}): {exc}")
            return None

    def trigger(self):
        self.fired += 1
        self.times.append(time.monotonic())
        self.log(f"[dropper] TRIGGER #{self.fired}: gate -> {self.open_us}us")
        self._safe(self.open_us, 'open')
        self._sleep(self.dwell_s)
        self._safe(self.closed_us, 'close')


class ServoDropper(Dropper):
    """SG90 gate on the hopper, actuated through the STM32 over the Bridge.

    ALTERNATIVE to PixhawkServoDropper, kept for the case where the Pixhawk
    output is unavailable. BLOCKED: needs a Zephyr-compatible PWM
    implementation in the sketch, since the stock Servo library refuses to
    build for this core.

    The sketch provides two RPC methods (see uno_q/sketch_bridge/):
        servo_set(angle)  -> int   move the gate, returns the angle applied
        servo_detach()    -> int   stop pulsing, so the servo stops holding

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

    def trigger(self):
        self.fired += 1
        self.times.append(time.monotonic())
        self.log(f"[dropper] TRIGGER #{self.fired}: gate -> {self.open_deg}deg")
        self._safe('servo_set', self.open_deg, why='open')
        self._sleep(self.dwell_s)
        self._safe('servo_set', self.closed_deg, why='close')
        if self.detach_after:
            self._safe('servo_detach', why='release')
