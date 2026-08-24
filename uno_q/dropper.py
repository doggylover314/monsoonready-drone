"""Payload drop actuation. MG90 metal-gear servo on the hopper gate.

Servo power comes from the XY-3606 buck at 5.00 V, separate from the
Pixhawk and UNO Q rails. The PWM signal comes from a Pixhawk AUX output,
which has hardware timers; MAIN 1-6 are the six motors, so AUX is free.

ServoDropper, at the bottom of this file, is an unused alternative that
drives the servo from the UNO Q's STM32 over the Bridge: a future option,
not currently wired.
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
    """Resolve the UNO Q Bridge. The import is lazy, so laptop/SITL runs need no board."""
    try:
        from arduino.app_utils import Bridge
    except ImportError as exc:      # pragma: no cover - board-only path
        raise RuntimeError(
            "arduino.app_utils not importable: ServoDropper needs the UNO Q "
            "Arduino runtime, or an explicit bridge_call=. Original: "
            f"{exc}") from exc
    return lambda method, *args: Bridge.call(method, *args)


class PixhawkServoDropper(Dropper):
    """Hopper gate driven by a Pixhawk servo output. This is the flight configuration.

    Board setup (tools/parameters.py push, then reboot):
        SERVO<channel>_FUNCTION = 0     Disabled, so DO_SET_SERVO owns it
        SERVO<channel>_MIN / _MAX       bracket closed_us and open_us

    Wiring: signal from the Pixhawk AUX pin, power from the XY-3606 buck,
    grounds common.

    Failure policy is deliberately asymmetric:
      * __init__'s close fails -> raise. This is the ground-side proof of
        the whole chain.
      * trigger() fails -> return False. Raising mid-descent would be a
        runaway, not a fix.
    """

    # Gate travel, set on the bench with tools/servo_jog.py on 2026-08-22.
    # SERVO9_MIN must stay <= 500 or the close gets clamped short.
    # SERVO9_TRIM 560 parks the gate part-open at boot; set it to 500 instead.
    US_PER_DEG = 10.0
    DEFAULT_CLOSED_US = 500
    DEFAULT_OPEN_US = 1600

    def __init__(self, io, channel=9, closed_us=DEFAULT_CLOSED_US,
                 open_us=DEFAULT_OPEN_US,
                 dwell_s=1.0, log=print, sleep=time.sleep):
        for name, v in (('closed_us', closed_us), ('open_us', open_us)):
            if not PWM_MIN_US <= v <= PWM_MAX_US:
                raise ValueError(
                    f"{name}={v} outside {PWM_MIN_US}-{PWM_MAX_US}us")
        if closed_us == open_us:
            raise ValueError(
                f"closed_us and open_us are both {closed_us}: gate cannot move")
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
        self.gate_open = False
        # Closes on construction: shuts a gate a crashed run left open, and
        # doubles as the pre-arm end-to-end check.
        if self._safe(self.closed_us, 'initial close') is None:
            raise RuntimeError(
                f"dropper pre-arm close FAILED on servo channel {channel}. "
                f"Check SERVO{channel}_FUNCTION=0, SERVO{channel}_MIN/_MAX, "
                f"the signal wire, and 5V from the XY-3606. "
                f"Use --no-drop to fly the survey without a dropper.")

    def _safe(self, pwm_us, why):
        """Send a gate command. Returns the result, or None on failure."""
        try:
            return self.io.set_servo(self.channel, pwm_us)
        except Exception as exc:                      # noqa: BLE001
            self.log(f"[dropper] set_servo({self.channel}, {pwm_us}) "
                     f"FAILED ({why}): {exc}")
            return None

    def trigger(self, dwell_s=None):
        """Open, dwell, close. True only if the gate opened.

        dwell_s scales the dose for this drop: the aperture is fixed, so open
        time is the only variable. tools/flow_test.py converts it to grams.
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
            # Payload went out, but a gate stuck open drains the hopper.
            self.log(f"[dropper] WARNING gate may still be OPEN on channel "
                     f"{self.channel}: hopper is draining, land soon")
        else:
            self.gate_open = False
        return True


class ServoDropper(Dropper):
    """Hopper gate driven from the UNO Q STM32 over the Bridge.

    A future alternative to PixhawkServoDropper, for when no Pixhawk
    output is available. Needs a Zephyr PWM implementation in the sketch,
    plus 3.3 V level verification on the MG90 input.

    Expects two Bridge RPC methods:
        servo_set(angle)  -> int   move the gate, returns the angle applied
        servo_detach()    -> int   stop pulsing, so the servo stops holding

    detach_after releases the servo once closed, to save current and heat.
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
        # Close on construction, same reason as PixhawkServoDropper.
        self._safe('servo_set', self.closed_deg, why='initial close')
        if self.detach_after:
            self._safe('servo_detach', why='initial detach')

    def _safe(self, method, *args, why=''):
        """Bridge calls never propagate: a missed drop beats a dead mission."""
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
