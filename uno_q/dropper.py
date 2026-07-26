"""Payload drop actuation.

Interface is stable; the real implementation is an open hardware question:
the SG90 signal pin comes from UNO Q GPIO (3.3V, usually fine per
PROJECT_STATE), but the PWM *source* is undecided — Linux userspace soft-PWM
is jittery; the cleaner path is the STM32 side via Bridge (TODO 12 decides).
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


class ServoDropper(Dropper):
    """Real SG90 hatch. Blocked on PWM-source decision (Linux GPIO vs STM32
    Bridge); implement after TODO 12."""

    def __init__(self, *a, **kw):
        raise NotImplementedError("PWM source undecided; see module docstring")
