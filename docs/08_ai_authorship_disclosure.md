# AI Assistance → Disclosure

Stated up front and in full. A disclosure that is volunteered is a strength; one
extracted under questioning is a problem.

---

## 1. Statement for the submission and video credit

> This project was built with substantial AI assistance. Claude was used to
> write and review code, to research component and dataset choices, and to
> draft documentation. The system architecture, every engineering decision, all
> hardware assembly, and all testing and debugging are the work of the two team
> members. Nothing in this submission was accepted without being understood,
> and where the assistant was wrong it was corrected.

---

## 2. Scope

| Area | AI assistance | Human work |
|------|---------------|------------|
| Python tooling (`tools/`, `training/`) | Drafted | Specified, reviewed, run, debugged |
| ESP32 firmware | Drafted | Specified, reviewed; never flashed by the assistant |
| UNO Q mission code (`uno_q/`) | Drafted | Architecture specified, SITL-tested, reviewed |
| Component and dataset research | Candidate options, comparisons | Every final choice |
| This documentation set | Drafted from the repository's own state file, build log and source | Reviewed and corrected |
| **Concept** | None | Entirely the team's, including the insight that granular larvicide removes the need for a spray system |
| **Architecture** | None | Hexacopter, serial allocation, onboard detection, target latching, abort-upward, ESP32 emitting standard MAVLink rather than a custom protocol |
| **Physical build** | None | Assembly, soldering, wiring, mounting, calibration |
| **Flying** | None | All flights, and all three crashes |
| **Debugging** | None | The assistant does not have the aircraft |
| **Engineering judgment** | None | Particularly the vibration gate, and the decision to respect it when inconvenient |

---

## 3. Where the assistant was wrong

Recorded because a disclosure listing only successes is not a disclosure.

| Error | Correction |
|-------|-----------|
| ArduPilot parameter names, repeatedly. They are renamed between firmware versions. | Several 4.7 parameters were only correct because they were checked against the board and the source. `ARMING_CHECK` → `ARMING_SKIPCHK`, `RTL_ALT` → `RTL_ALT_M`, `RNGFND1_GNDCLEAR` → `RNGFND1_GNDCLR` were all found this way. |
| Assumed hardware capabilities this board lacks | The bidirectional-DShot RPM notch filter was planned, then found impossible on this flight controller because the required timer channels have no DMA. Design moved to an in-flight FFT-based notch. |
| Proposed a rangefinder abort rule that would have aborted **every** descent | It did not account for the sensor being blind at survey altitude. Caught in review; rewritten to distinguish "never acquired" from "acquired then lost". |

The pattern is consistent: the assistant is useful at the level of "write this,
check that", and needs a human who knows the system to catch the cases where
its model of the hardware is wrong.

---

## 4. Working method

Two people, two machines, each with an AI assistant, sharing one git
repository. Because neither assistant can see the other's session, the project
keeps a single machine-readable state file, `PROJECT_STATE.md`, carrying
current state, an append-only decision log with dates and authors, and a
session-continuity section. Every change updates it; every session begins by
reading it.

Coordinating two humans and two AI assistants against a deadline turned out to
need what any distributed team needs: one written source of truth that nobody
is allowed to work around.

---

## 5. If asked directly

The statement in §1, then specifics from §2 and §3. The distinction that
matters: the assistant wrote code the team specified and can each explain; it
did not decide what to build, and it could not have assembled, flown, crashed,
diagnosed or repaired any of it.
