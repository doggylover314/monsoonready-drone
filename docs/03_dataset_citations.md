# Training Data → Sources, Licences, Attribution

Every public dataset used to train the water detector, with its licence. The
public sets are **CC BY 4.0**, which makes attribution a **licence
obligation**, not a courtesy. This document is that attribution.

`TBD` marks a URL or citation block not yet recorded in the repository. Each is
recoverable from the dataset's Roboflow Universe page, which carries a
**"Cite this Project"** block producing exact BibTeX. Approximating these is
worse than leaving them blank.

---

## 1. Dataset v1 → 11,725 train / 3,069 val

Used for training run 1, the baseline reported in `01_project_writeup.md` §5.4.

| Set (as named in `training/exports/`) | Images | Classes used | Licence | Citation |
|---|---|---|---|---|
| mosquito v1 | 5,069 incl. splits | `puddle` only | CC BY 4.0 | TBD — URL + BibTeX |
| puddle_Detect | ~4,930 | single class | CC BY 4.0 | TBD — URL |
| hanyang puddle-detection | ~1,500 | single class | CC BY 4.0 | TBD — URL |
| hanyang puddle | ~1,500 | single class | CC BY 4.0 | TBD — URL |
| water | ~1,000 | single class | CC BY 4.0 | TBD — URL |
| yinjia part 2 | TBD | single class | CC BY 4.0 | TBD — URL |

---

## 2. Dataset v2 additions → ~21,700 train / ~4,500 val

Merged 2026-07-25.

| Set | Images | Classes used | Licence | Citation |
|---|---|---|---|---|
| Thesis, mosquito-breeding-grounds-2 | 3,470 | `Temporary Water Sites` only, of 7 | CC BY 4.0 | `universe.roboflow.com/thesis-kjmym/mosquito-breeding-grounds-2` **(verify the exact citation block)** |
| Fumigation habitats | ~1,060 | puddle / probable-stagnant-water | CC BY 4.0 | TBD — URL |
| Fumigation habitats2 | ~1,290 | puddle / probable-stagnant-water | CC BY 4.0 | TBD — URL |
| First-party nadir photographs | TBD | `puddle` | Original work | This project |

The first-party images are the significant addition: photographs taken looking
**straight down from height**, which is the actual camera geometry of this
mission and something no public dataset provides. They include deliberate
negatives (shadows, tarpaulins, wet ground without pooling, rooftops, plastic
sheeting) targeting the run-1 failures in `01_project_writeup.md` §5.4.

---

## 3. Sets evaluated and rejected

| Set | Verdict | Reason |
|---|---|---|
| mosquito **v4** | **Excluded, actively harmful** | No puddle class at all. Merging it contributes images whose water is unlabelled, training the model that water is background. Kept in `training/excluded/` rather than deleted, so the decision stays visible. |
| mosquito-breeding-grounds **v3/v4/v5** | Excluded | Container-only spin-offs, no water class. Same poisoning problem. |
| 82myj | Skipped | 112 images; too small to matter |
| Eds breeding-detection | Skipped | 556 images, container-heavy rather than water |
| Fumigation vol3 / vol4 / vol5 | Skipped | No puddle class |
| Insect close-up sets | Skipped | Photographs of mosquitoes. Wrong task: this model detects water, not insects. |
| Various aedes / MBG sets | Skipped | Superseded by better-labelled sets over the same ground |

---

## 4. Methodology

### 4.1 No two versions of one image pool

Roboflow versions of a dataset are re-splits and re-augmentations of the **same
source photographs**. Merging two versions puts the same image in train and
val, inflating validation scores while teaching nothing. Every set in sections
1 and 2 comes from a distinct pool.

### 4.2 `pool` and `water tank` classes are deliberately dropped

They are genuine mosquito breeding sites, so keeping them looks defensible.
They are **not drop targets** for this aircraft: a swimming pool is permanent,
managed water, and a granule drop into one is both useless and unwelcome.
Training the model to fire on them would create confident detections the
mission logic would then have to suppress, which is worse than not learning
them. Implemented as `KEEP_NAMES` in `training/merge_datasets.py`.

### 4.3 How the merge works

`training/merge_datasets.py` collapses every export into one single-class
dataset:

| Input | Handling |
|-------|----------|
| Set with exactly one class | Kept whole, whatever the class is named. Covers sets whose only class is `water` or a non-English word for puddle. |
| Set with multiple classes | Only water-named classes kept; other boxes dropped |
| Image whose boxes were all dropped | **Kept as a negative**, with an empty label file |
| Roboflow `test` split | Folded into our validation split |

The negatives are free hard-negative data: images containing tyres, bottles and
containers but no water teach the model what standing water is not. The
test-split decision rests on the real test set being the drone.

---

## 5. Attribution text

For the submission document and video credits, once section 1 and 2 URLs are
recorded:

> The water-detection model was trained on publicly available datasets from
> Roboflow Universe, each licensed CC BY 4.0 and listed with full attribution
> in the project documentation, together with original photographs taken by
> the project team.
