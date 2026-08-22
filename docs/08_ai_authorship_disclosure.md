# How AI was used

We used Claude heavily. It wrote code, it researched parts and datasets, and it
drafted these documents. Here is the split, in detail, including the times it
got things wrong.

## The short version, for the submission and the video

> This project was built with substantial AI assistance. Claude wrote and
> reviewed code, researched component and dataset choices, and drafted the
> documentation. The architecture, the engineering decisions, all of the
> building, and all of the testing and debugging are ours. We understood
> everything we accepted, and we corrected it when it was wrong.

## Who did what

| Area | The assistant | Us |
|------|---------------|-----|
| Python in `tools/` and `training/` | Wrote it | Said what it should do, reviewed, ran, debugged |
| ESP32 firmware | Wrote it | Specified and reviewed. It has never flashed the board. |
| Mission code in `uno_q/` | Wrote it | Designed the state machine, tested it in SITL, reviewed it |
| Parts and datasets | Gave us options and comparisons | Every actual choice |
| These documents | Drafted from the state file, the build log and the source | Corrected and cut |
| The idea | Nothing | Ours, including the point that granular larvicide means no spray system |
| The architecture | Nothing | Hexacopter, serial layout, detection onboard, target latching, abort upward, ESP32 speaking standard MAVLink instead of something custom |
| Building it | Nothing | Assembly, soldering, wiring, mounting, calibration |
| Flying it | Nothing | Every flight, and all three crashes |
| Debugging | Nothing | It has never had the aircraft in front of it |

## Where it was wrong

Four that cost us real time.

**Parameter names.** ArduPilot renames things between versions and the
assistant kept using old names from memory. `WPNAV_SPEED` does not exist on
4.7, it is `WP_SPD`, and that one would have failed silently. Same story with
`RTL_ALT` to `RTL_ALT_M` and `RNGFND1_GNDCLEAR` to `RNGFND1_GNDCLR`. The fix
was a rule: check the board or the source, never memory.

**Hardware this board cannot do.** It planned a bidirectional-DShot RPM notch
filter before anyone checked whether the timer channels have DMA. They do not.
We moved to an in-flight FFT notch instead.

**A rangefinder rule that would have aborted every descent.** The proposed
logic treated "no reading" as a fault, but the TF-Luna cannot see the ground
from 15 m at all, so the first part of every descent is blind and always will
be. Caught in review. The rule now separates "never acquired" from "acquired
then lost".

**A theory that survived three days without evidence.** It decided our arming
failures came from the firmware's 201 cm "clear" value being read as no data.
Reading the actual ArduPilot source killed that: the driver timestamps every
message before it checks any sector. The real cause was the ESP32 blocking for
a second at a time while it retried two dead sensors. Same class of mistake as
the parameter names, which is trusting a plausible model of the system instead
of going and looking.

The pattern is consistent. It is good at "write this, check that" and it needs
somebody who knows the hardware sitting next to it.

## How the two of us worked

Two people, two machines, one repository, and an AI assistant on each side.
Neither assistant can see the other's session, so everything goes through one
file. `PROJECT_STATE.md` holds current state, a dated decision log nobody
edits after the fact, and a section saying who is working on what. Every
change updates it. Every session starts by reading it.

That turned out to be the whole trick for keeping two humans and two
assistants pointed the same way.

## If a judge asks

Read them the statement above, then go into specifics. The line that matters:
the assistant wrote code we specified and can each explain, it did not decide
what to build, and it could not have assembled, flown, crashed, diagnosed or
repaired any of it.
