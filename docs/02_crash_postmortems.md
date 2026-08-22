# The three crashes

All three happened on the S550, the airframe before this one. Two share a root
cause, which is the part worth knowing.

Sources: `Build Log.txt` and the crash entries in `PROJECT_STATE.md`.

| | Trigger | Damage | Rule it produced |
|--|--|--|--|
| 1 | Prop nut unthreaded in flight | Prop departed, aircraft tumbled | Handed nut caps, Loctite, prop check every preflight |
| 2 | AltHold in wind | Drifted about 44 m into a tree, one arm snapped | Loiter by default, GPS gate before any GPS mode |
| 3 | Vibration corrupted the altitude estimate | Climbed to about 47 m, disarmed in the air, centre plate destroyed | Median VibeZ under 15 in hover. Abort means Stabilize, not disarm. |

## 1. The prop nut

A nut came off in flight. The imbalance shook the aircraft hard enough that the
altitude estimate degraded, it climbed to roughly 12 m, and then the propeller
left the motor and it tumbled.

Cause: nut handedness. Half the motors on a multirotor turn each way, so a nut
that tightens on one unscrews on its neighbour. No thread lock either.

The lesson outlived the fix. The mechanical failure was the trigger, but the
aircraft became unflyable when the altitude estimate went bad, which happened
before the prop physically left. That is crash 3 as well.

## 2. AltHold in wind

Flown manually in AltHold, which holds height and does nothing about position.
The wind took it 44 m into a tree. Loiter would have held station.

Not a technical failure. A pilot picked the wrong mode, so the fix is
procedural: Loiter unless the plan is deliberate manual practice, and the mode
switch laid out Stabilize / AltHold / Loiter so nobody has to hunt. The GPS rule
came out of this too, meaning ten satellites or more, HDOP under 1.5, no EKF
complaints, after two to five minutes of settling.

## 3. Vibration and the phantom fall

The airframe shook enough on its own to poison the state estimate. The aircraft
decided it was falling and climbed to fight a descent that was not happening,
reaching about 47 m. Wind was pushing it toward ground it could not be retrieved
from, and Reyansh disarmed it in the air.

It had also been armed within seconds of power-on, at HDOP 65 to 99. The GPS was
fine, it just had not been given time. Bench-tested after the crash, the same
unit reached 10 satellites and HDOP under 1.0 in thirty seconds, which settles
whose fault that was.

The disarm is recorded as a deliberate choice, because calling it a loss of
control would hide the reasoning. A powered aircraft arriving in that spot was
the worse outcome.

Everything else on the bench passed: battery, all six motors, ESCs including the
resoldered one, GPS, Pixhawk, telemetry radio, receiver. The power module and
the buzzer failed and were replaced.

## Where vibration stands now

It used to be the open blocker on this project. Not any more.

Taking the rubber motor dampeners off moved the median from about 30 to about
20.6, and the rebuild onto the F550 did the rest. Recent flight logs read a
median VibeZ of 7.4 to 9.0 against a gate of 15, with zero clipping.

Log 34, the worst on record, sat at 46 with 5927 clip events, and that flight
turned out to have a damaged prop. It is kept deliberately: a failing vibration
plot next to a passing one is better evidence than a passing one alone.

## The common thread

Two of three go the same way:

```
vibration -> corrupted altitude estimate -> the flight controller acts on it
          -> violent, unwanted behaviour
```

The third was a mode choice that position hold would have covered.

This is why the mission code treats altitude with suspicion. The descent needs
the rangefinder and the EKF altitude to agree before it will drop, and it aborts
upward when they do not. The rule exists because of crash 3, not because it
seemed like good practice.
