# AI assistance disclosure

Stated up front, in full, because a disclosure that is volunteered is a
strength and one that is extracted under questioning is a problem.

## The short version, for the submission and the video credit

> This project was built with substantial AI assistance. Claude was used to
> write and review code, to research component and dataset choices, and to
> draft documentation. The system architecture, every engineering decision,
> all hardware assembly, and all testing and debugging are the work of the
> two team members. Nothing in this submission was accepted without being
> understood, and where the assistant was wrong we corrected it.

## What AI was actually used for

**Code.** Most of the Python and the ESP32 firmware in this repository was
drafted with AI assistance: the parameter tools, the dataset merge script, the
training and export scripts, the obstacle-module firmware, and the onboard
mission state machine. All of it was specified by us, reviewed by us, and
tested by us.

**Research.** Comparing candidate datasets and models, checking what a
particular ArduPilot parameter does, and working out the constraints of the
hardware. This is also where the assistant was most often wrong and had to be
checked against primary sources: the firmware, the datasheets, or the board
itself.

**Documentation.** This documentation set was drafted with AI assistance from
the project's own state file, build log and source code.

## What was not AI

- The concept, including the decision that granular larvicide removes the need
  for a spray system.
- Every architectural decision: hexacopter, serial allocation, the choice to run
  detection onboard, target latching, abort-upward-on-dropout, the ESP32
  translating to standard MAVLink messages rather than a custom protocol.
- All physical work: assembly, soldering, wiring, mounting, calibration.
- All flying, and all three crashes.
- All debugging. The assistant does not have the aircraft.
- The engineering judgments that constrain the project, particularly the
  vibration gate and the decision to respect it when it is inconvenient.

## Where the assistant was wrong

Kept because a disclosure listing only successes is not a disclosure.

- It has repeatedly needed correcting on ArduPilot parameter names, which are
  renamed between firmware versions. Several parameters in the 4.7 setup were
  only right because they were checked against the board and the source.
- It suggested approaches that assumed hardware capabilities this board does
  not have. The bidirectional-DShot RPM notch filter is the example: it was
  planned, then found impossible on this flight controller because the required
  timer channels have no DMA, and the design moved to an in-flight FFT-based
  notch instead.
- It initially reasoned about a rangefinder abort rule that would have aborted
  every single descent, because it did not account for the sensor being blind
  at survey altitude. That was caught in review and the rule was rewritten to
  distinguish "never acquired" from "acquired then lost".

The pattern in all three is the same: the assistant is useful at the level of
"write this, check that", and needs a human who knows the system to catch the
cases where its model of the hardware is wrong.

## Working method

Two people on two machines, each with an AI assistant, sharing one git
repository. Because neither assistant can see the other's session, the project
keeps a single machine-readable state file, `PROJECT_STATE.md`, that carries
current state, an append-only decision log with dates and authors, and a
session-continuity section. Every change updates it, and every session begins
by reading it.

That file is the interesting part of the process, honestly. Coordinating two
humans and two AI assistants against a deadline turned out to need the same
thing that coordinating a distributed team needs: one written source of truth
that nobody is allowed to work around.

## If a judge asks directly

Answer with the short version above, then offer specifics. Do not minimise the
extent of the assistance, and do not let it be characterised as the AI having
built the drone. The distinction that matters is this: the assistant wrote code
we specified and could each explain; it did not decide what to build, and it
could not have assembled, flown, crashed, diagnosed or fixed any of it.
