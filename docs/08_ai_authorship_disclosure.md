# How AI was used

We used Claude heavily. It wrote code, researched parts and datasets, and
drafted these documents. Here is the split, including the times it was wrong.

## The short version, for the submission and the video

> This project was built with substantial AI assistance. Claude wrote and
> reviewed code, researched component and dataset choices, and drafted the
> documentation. The architecture, the engineering decisions, all of the
> building, and all of the testing and debugging are ours. We understood
> everything we accepted, and we corrected it when it was wrong.

## Who did what

| Area | The assistant | Us |
|------|---------------|-----|
| Python in `tools/`, `training/`, `uno_q/` | Wrote it | Specified it, designed the state machine, reviewed, ran, debugged |
| ESP32 firmware | Wrote it | Specified and reviewed. It has never flashed the board. |
| Parts and datasets | Options and comparisons | Every actual choice |
| These documents | Drafted from the state file, the build log and the source | Corrected and cut |
| The idea | Nothing | Ours, including the point that granular larvicide means no spray system |
| The architecture | Nothing | Hexacopter, serial layout, detection onboard, target latching, abort upward, ESP32 speaking standard MAVLink instead of something custom |
| Building and flying it | Nothing | Assembly, soldering, wiring, calibration, every flight, all three crashes |
| Debugging | Nothing | It has never had the aircraft in front of it |

## Where it was wrong

Four that cost real time.

**Parameter names.** ArduPilot renames things between versions and the assistant
kept using old names from memory. `WPNAV_SPEED` does not exist on 4.7, it is
`WP_SPD`, and that one would have failed silently. Same with `RTL_ALT` to
`RTL_ALT_M`, `RNGFND1_GNDCLEAR` to `RNGFND1_GNDCLR`, and `ARMING_CHECK`, which
4.7 replaced with `ARMING_SKIPCHK` and inverted while it was at it, so the old
name reads as a dead link rather than an error. The fix was a rule: check the
board or the source, never memory. It kept happening anyway.

**Hardware this board cannot do.** It planned a bidirectional-DShot RPM notch
filter before anyone checked whether the timer channels have DMA. They do not.
We moved to an in-flight FFT notch.

**A rangefinder rule that would have aborted every descent.** The proposed logic
treated "no reading" as a fault, but the TF-Luna is good to about 8 m, so the
first part of any descent starting above that is blind and always will be. Caught
in review, and the rule now separates "never acquired" from "acquired then
lost".

**Two theories that survived on plausibility alone.** It decided our arming
failures came from the firmware's 201 cm "clear" value being read as no data,
and held that for three days. Reading the ArduPilot source killed it: the driver
timestamps every message before checking any sector. The real cause was the
ESP32 blocking for a second at a time while it retried two dead sensors.
Separately, it assumed for days that `AVOID_ENABLE=7` meant the obstacle ring
protected the whole flight, when the source shows avoidance is applied only to
guided velocity targets and the survey rows send position targets. Both are the
same failure: trusting a plausible model of the system instead of going and
looking.

It is good at "write this, check that" and it needs somebody who knows the
hardware sitting next to it.

## How the two of us worked

Two people, two machines, one repository, an assistant on each side. Neither
assistant can see the other's session, so everything goes through
`PROJECT_STATE.md`: current state, a dated decision log nobody edits after the
fact, and a section saying who is working on what. Every change updates it and
every session starts by reading it. That turned out to be the whole trick.

## If a judge asks

Read them the statement above, then go into specifics. The assistant wrote code
we specified and can each explain, it did not decide what to build, and it could
not have assembled, flown, crashed, diagnosed or repaired any of it.
