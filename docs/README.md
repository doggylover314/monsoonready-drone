# The documentation set

Written for the Arduino Physical AI Challenge India, judged on innovation,
functionality, documentation and presentation. Documentation is one of the four,
so this folder is a deliverable.

Everything here comes from `PROJECT_STATE.md`, `Build Log.txt` and the source.
Nothing is invented. Where a number exists but nobody has measured it yet, the
file says `TBD` and leaves it, rather than filling the gap with something
plausible.

2026-08-15 was the shoot day. It was never the upload day. The submission goes
in on 2026-08-23.

| File | What is in it |
|--|--|
| `01_project_writeup.md` | The main narrative. Problem, system, hardware, model, mission logic, dose, and what is not finished. |
| `02_crash_postmortems.md` | Three crashes on the old S550, and the rule each produced |
| `03_dataset_citations.md` | Every training set with its licence, plus the sets rejected and why |
| `04_demo_video_storyboard.md` | Running order for the video, and the fallback |
| `05_compliance_narrative.md` | Where this sits legally, confidence-rated, gaps stated |
| `06_judge_qa_prep.md` | Every design decision as a question and an answer |
| `07_evidence_checklist.md` | Each photo, log and capture, and the claim it backs |
| `08_ai_authorship_disclosure.md` | How AI was used, including where it was wrong |
| `block_diagram.svg` | Power and serial wiring on one page |

`01` is the spine. The rest exist so it can stay short while every claim in it
still has something behind it: crashes to `02`, licences to `03`, the mustard
seed to `05`, the descent rule to `06`, the AI scope to `08`, and every measured
number to `07`. `04` is the same claims in the order the video says them.

Work items live in `PROJECT_STATE.md`. These files describe the project, the
state file tracks what is left to do on it, and mixing the two is how a document
ends up half stale.

Firmware throughout is ArduCopter 4.7.0, Pixhawk1-bdshot, flashed 2026-07-25.
