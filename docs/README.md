# The documentation set

Written for the Arduino Physical AI Challenge India, which judges innovation,
functionality, documentation and presentation. Documentation is one of the four,
so this folder is a deliverable rather than a side effect.

Everything here comes from `PROJECT_STATE.md`, `Build Log.txt` and the source in
this repository. Nothing is invented. Where a number exists but nobody has
measured it yet, the file says `TBD` and leaves it there rather than filling the
gap with something plausible.

On dates: 2026-08-15 was the shoot day. It was never the upload day. The
submission goes in on 2026-08-23.

| File | What is in it |
|--|--|
| `01_project_writeup.md` | The main narrative. Problem, system, hardware, model, mission logic, and what is not finished. |
| `02_crash_postmortems.md` | Three crashes on the old S550. Cause, evidence, and the rule each one produced. |
| `03_dataset_citations.md` | Every training set with its licence, plus the sets rejected and why |
| `04_demo_video_storyboard.md` | The running order for the video, and the fallback version |
| `05_compliance_narrative.md` | Where this sits legally, confidence-rated, gaps stated |
| `06_judge_qa_prep.md` | Every design decision as a question and an answer |
| `07_evidence_checklist.md` | Each photo, log and capture, and the claim it backs |
| `08_ai_authorship_disclosure.md` | How AI was used, including where it was wrong |

## How they fit together

`01` is the spine. The rest exist so `01` can stay short while every claim in it
still has something behind it.

```
01_project_writeup            the claims
 |- "two of three crashes were vibration"  -> 02
 |- "trained on public data"               -> 03  licence and attribution
 |- "mustard seed, not larvicide"          -> 05  why, and what deployment needs
 |- "aborts upward on dropout"             -> 06  the reasoning, defensible live
 |- "AI-assisted"                          -> 08  scope of assistance
 |- every measured number                  -> 07  the artefact that proves it
04 is the video, built from the same claims in the same order.
```

## Conventions

`TBD` marks a value that exists but has not been measured or written down
anywhere. It is never a guess dressed up.

Work items live in `PROJECT_STATE.md`. These files describe the project, the
state file tracks what is left to do on it, and mixing the two is how a document
ends up half stale.

Firmware throughout is ArduCopter 4.7.0, Pixhawk1-bdshot, flashed 2026-07-25.
