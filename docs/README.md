# MonsoonReady documentation set (TODO 19)

Documentation is one of the four judged categories for the Arduino Physical AI
Challenge India (innovation / functionality / documentation / presentation),
submission deadline 2026-08-15. This folder is the written half of the
submission; the demo video is the other half.

Written from PROJECT_STATE.md, `Build Log.txt`, and the code in this repo, on
2026-07-26 (Raghav / Mac). Everything here is traceable to one of those
sources. Anything not yet known is marked `FILL:` rather than guessed, because
a confident wrong number in front of judges is worse than an open blank.

| Doc | What it is | Status |
|---|---|---|
| [01_project_writeup.md](01_project_writeup.md) | The main submission narrative: problem, system, why each choice | Draft complete; needs final results + photos |
| [02_crash_postmortems.md](02_crash_postmortems.md) | Three crashes, root cause, evidence, what changed | Complete from logged facts |
| [03_dataset_citations.md](03_dataset_citations.md) | Licence and attribution for every training set | Structure complete; URLs/BibTeX need Reyansh |
| [04_demo_video_storyboard.md](04_demo_video_storyboard.md) | Shot-by-shot plan for the demo video | Complete; shoot against it |
| [05_compliance_narrative.md](05_compliance_narrative.md) | Regulatory position, stated honestly | Draft; needs Raghav's verification pass |
| [06_judge_qa_prep.md](06_judge_qa_prep.md) | Every design decision, defensible in Q&A | Complete for decisions made so far |
| [07_evidence_checklist.md](07_evidence_checklist.md) | Photos, screenshots and logs to capture, and what each one proves | Complete; capture is on you two |
| [08_ai_authorship_disclosure.md](08_ai_authorship_disclosure.md) | How AI assistance was used, disclosed plainly | Complete |

## How to use this set

The write-up is the spine. The other documents exist so that the write-up can
stay readable while every claim in it still has something behind it: a judge
who asks "how do you know the vibration was the cause?" gets pointed at the
post-mortems, and one who asks "where did the training images come from?" gets
pointed at the citations.

## Open items that need a human

These are the blanks that cannot be filled from the repo. They are repeated in
context inside the individual documents.

1. **Dataset URLs and the mosquito-set BibTeX** (doc 03). Reyansh downloaded
   the sets and has the Roboflow Universe pages open in his history. CC BY 4.0
   requires attribution, so this is a licence obligation, not a nicety.
2. **Final model numbers** (docs 01, 06). Run 1 numbers are in place as the
   baseline; the v2 / yolo26n results replace them once that run finishes on
   the RTX 3050.
3. **Regulatory verification** (doc 05). Drone Rules category thresholds and
   any aerial-application SOP must be checked against the current DGCA text
   before submission. The draft says what we believe and flags what to verify.
4. **All photographic evidence** (doc 07). Neither of us can produce these from
   a keyboard.
