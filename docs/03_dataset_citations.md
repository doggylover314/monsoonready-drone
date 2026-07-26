# Training data: sources, licences, attribution

Every public dataset used to train the water detector is listed here with its
licence. The sets are CC BY 4.0 as recorded in `PROJECT_STATE.md`, which means
attribution is a **licence obligation**, not a courtesy. This document is the
attribution.

> **Blocking gap.** The exact Roboflow Universe URLs and the BibTeX for the
> mosquito set are not recorded in this repo. Reyansh downloaded the sets and
> has the pages in his browser history. Every row below marked `FILL:` needs
> its canonical URL, author/workspace name, and version before submission.
> Do not approximate these. A wrong citation is worse than a missing one, and
> Roboflow Universe pages carry a "Cite this Project" block that produces the
> exact BibTeX.

## Dataset v1 (11,725 train / 3,069 val)

Used for training run 1, the baseline reported in the write-up.

| Set (as named in our exports) | Approx. images | Classes used | Licence | Citation |
|---|---|---|---|---|
| mosquito v1 | 5,069 incl. splits | `puddle` only | CC BY 4.0 | `FILL: URL + BibTeX from the Universe page's "Cite this Project" block` |
| puddle_Detect | approx. 4,930 | single class | CC BY 4.0 | `FILL: URL` |
| hanyang puddle-detection | approx. 1,500 | single class | CC BY 4.0 | `FILL: URL` |
| hanyang puddle | approx. 1,500 | single class | CC BY 4.0 | `FILL: URL` |
| water | approx. 1,000 | single class | CC BY 4.0 | `FILL: URL` |
| yinjia part 2 | `FILL` | single class | CC BY 4.0 | `FILL: URL` |

## Dataset v2 additions (merged 2026-07-25; approx. 21,700 train / 4,500 val)

| Set | Approx. images | Classes used | Licence | Citation |
|---|---|---|---|---|
| Thesis, mosquito-breeding-grounds-2 | 3,470 | `Temporary Water Sites` only, of 7 | CC BY 4.0 | universe.roboflow.com/thesis-kjmym/mosquito-breeding-grounds-2 (verify the exact citation block) |
| Fumigation habitats | approx. 1,060 | puddle / probable-stagnant-water classes | CC BY 4.0 | `FILL: URL` |
| Fumigation habitats2 | approx. 1,290 | puddle / probable-stagnant-water classes | CC BY 4.0 | `FILL: URL` |
| First-party nadir photographs | `FILL: count` | `puddle` | Ours | Original work, this project |

The first-party images are the important addition. They are photographs taken
looking straight down from height, which is the actual camera geometry of this
mission and something no public dataset provides, plus deliberate negatives:
shadows, tarpaulins, wet ground without pooling, rooftops, plastic sheeting.
These target the specific failures found in run 1.

## Sets we evaluated and rejected

Recording rejections matters, because "why did you not use dataset X" is a fair
judging question and because it stops us re-litigating decisions.

| Set | Verdict | Reason |
|---|---|---|
| mosquito v4 | **Excluded, actively harmful** | No puddle class at all. Merging it would have contributed only images whose water was unlabelled, training the model that water is background. Kept in `training/excluded/` rather than deleted, so the decision stays visible. |
| mosquito-breeding-grounds v3, v4, v5 | Excluded | Container-only spin-offs with no water class. Same poisoning problem as above. |
| 82myj | Skipped | 112 images. Too small to matter. |
| Eds breeding-detection | Skipped | 556 images, container-heavy rather than water. |
| Fumigation vol3, vol4, vol5 | Skipped | No puddle class. |
| Insect close-up sets | Skipped | Photographs of mosquitoes. Wrong task entirely: we detect water, not insects. |
| Various aedes / MBG sets | Skipped | Superseded by better-labelled sets covering the same ground. |

## Two methodology points worth defending

**We never take multiple versions of the same underlying image pool.** Roboflow
versions of one dataset are re-splits and re-augmentations of the same source
photographs. Merging two versions puts the same image in train and val, which
inflates validation scores while teaching nothing. Every set above comes from a
distinct pool.

**"Pool" and "water tank" classes are deliberately not kept.** They are
genuine mosquito breeding sites, so keeping them would look defensible. They
are not drop targets for this aircraft: a swimming pool is permanent, managed
water, and a granule drop into one is both useless and unwelcome. Training the
model to fire on them would create confident detections that the mission logic
would then have to suppress, which is a worse design than not learning them.
The filter that implements this is `KEEP_NAMES` in
[training/merge_datasets.py](../training/merge_datasets.py).

## How the merge works

[training/merge_datasets.py](../training/merge_datasets.py) collapses every
export into one single-class dataset:

- A set with exactly one class is kept whole, whatever that class is named,
  which covers sets whose only class is `water` or a non-English word for
  puddle.
- A multi-class set keeps only water-named classes; other boxes are dropped.
- An image whose boxes were all dropped stays in the dataset with an empty
  label file, becoming a **negative example**. This is free hard-negative data:
  images that contain tyres, bottles and containers but no water teach the
  model what standing water is not.
- Roboflow's `test` split is folded into our validation split, on the grounds
  that the real test set is the drone.

## Attribution text for the submission and the video

Once the URLs are filled in, the following goes in the submission document and
in the video credits:

> The water-detection model was trained on publicly available datasets from
> Roboflow Universe, each licensed CC BY 4.0, listed with full attribution in
> the project documentation, together with original photographs taken by the
> project team.
