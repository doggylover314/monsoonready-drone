# MonsoonReady → Documentation Set

Written documentation for the **Arduino Physical AI Challenge India**
(submission deadline **2026-08-15**; judged on innovation, functionality,
documentation and presentation). Documentation is one of the four judged
categories, so this folder is a deliverable, not a side effect.

Content is drawn from `PROJECT_STATE.md`, `Build Log.txt`, and the source in
this repository. Nothing here is invented: a value that has not been measured
yet is written **`TBD`** rather than estimated.

---

## 1. Files

| File | Contents |
|------|----------|
| `01_project_writeup.md` | The submission narrative: problem, system, hardware, model, mission logic, and what is not finished |
| `02_crash_postmortems.md` | The three S550 crashes: cause, evidence, and the rules each one produced |
| `03_dataset_citations.md` | Every training set with licence and attribution, plus the sets rejected and why |
| `04_demo_video_storyboard.md` | Shot-by-shot plan for the demo video, with a descope variant |
| `05_compliance_narrative.md` | Regulatory position, confidence-rated, with the gaps stated |
| `06_judge_qa_prep.md` | Every design decision as a question and answer |
| `07_evidence_checklist.md` | The artefact set: each photo, log and capture, and the claim it supports |
| `08_ai_authorship_disclosure.md` | How AI assistance was used, including where it was wrong |

---

## 2. How the set fits together

`01` is the spine. Everything else exists so that `01` can stay readable while
every claim in it still has something behind it:

```
01_project_writeup            the claims
 ├── "vibration is unsolved"      → 02  (how we learned it, and the gate)
 ├── "trained on public data"     → 03  (licence + attribution, CC BY 4.0)
 ├── "salt, not larvicide"        → 05  (why, and what deployment would need)
 ├── "aborts upward on dropout"   → 06  (the reasoning, defensible live)
 ├── "AI-assisted"                → 08  (scope of assistance)
 └── every measured number        → 07  (the artefact that proves it)
04 is the video built from the same claims, in the same order.
```

---

## 3. Conventions

- **`TBD`** marks a value that exists but has not been measured or recorded
  yet, for example a benchmark figure or a dataset URL. It is never a guess.
- **Bold verify notes** mark a claim believed correct but not confirmed
  against a primary source. Regulatory statements in `05` carry explicit
  confidence levels for this reason.
- **Remaining work is tracked in `PROJECT_STATE.md` TODO 19**, not in these
  documents. The documents describe the project; the state file tracks what is
  left to do on it.
- Firmware version referenced throughout is **ArduCopter 4.7.0**
  (Pixhawk1-bdshot), flashed 2026-07-25.
