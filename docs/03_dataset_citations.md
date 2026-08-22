# Training data and licences

Every public dataset behind the water detector. They are all CC BY 4.0, so
attribution is a licence obligation, and this file is that attribution.

`TBD` means a URL or BibTeX block nobody has written down yet. Each is on the
dataset's Roboflow Universe page under "Cite this Project", which produces the
exact citation. Guessing at them would be worse than leaving them blank.

## Dataset v1, 11,725 train and 3,069 val

Training run 1 used this.

| Set, as named in `training/exports/` | Images | Classes kept | Licence | Citation |
|---|---|---|---|---|
| mosquito v1 | 5,069 incl. splits | `puddle` only | CC BY 4.0 | TBD |
| puddle_Detect | ~4,930 | single class | CC BY 4.0 | TBD |
| hanyang puddle-detection | ~1,500 | single class | CC BY 4.0 | TBD |
| hanyang puddle | ~1,500 | single class | CC BY 4.0 | TBD |
| water | ~1,000 | single class | CC BY 4.0 | TBD |
| yinjia part 2 | TBD | single class | CC BY 4.0 | TBD |

## Dataset v2, about 21,700 train and 4,500 val

Merged 2026-07-25. Everything above, plus:

| Set | Images | Classes kept | Licence | Citation |
|---|---|---|---|---|
| Thesis, mosquito-breeding-grounds-2 | 3,470 | `Temporary Water Sites` only, of 7 | CC BY 4.0 | `universe.roboflow.com/thesis-kjmym/mosquito-breeding-grounds-2`, exact block still to verify |
| Fumigation habitats | ~1,060 | puddle, probable-stagnant-water | CC BY 4.0 | TBD |
| Fumigation habitats2 | ~1,290 | puddle, probable-stagnant-water | CC BY 4.0 | TBD |
| Our own nadir photographs | TBD | `puddle` | Original work | This project |

The first-party photographs matter more than their count suggests. They are shot
straight down from height, which is the geometry this mission actually flies, and
no public dataset provides it. They include deliberate negatives: shadows,
tarpaulins, wet ground with no pooling, rooftops, plastic sheeting.

One thing to disclose. Some v2 sources carry segmentation polygons rather than
boxes, 3,001 segments against 3,881 boxes on the validation split. Ultralytics
reads the boxes and drops the polygons, so training is unaffected, but the label
files are mixed and anyone re-using the merge should know.

## Sets we rejected

| Set | Verdict | Why |
|---|---|---|
| mosquito v4 | Excluded, actively harmful | No puddle class at all, so merging it adds images whose water is unlabelled, teaching the model that water is background. Kept in `training/excluded/` rather than deleted so the decision stays visible. |
| mosquito-breeding-grounds v3, v4, v5 | Excluded | Container-only spin-offs, no water class. Same problem. |
| 82myj | Skipped | 112 images, too small to matter |
| Eds breeding-detection | Skipped | 556 images, mostly containers |
| Fumigation vol3, vol4, vol5 | Skipped | No puddle class |
| Insect close-up sets | Skipped | Photographs of mosquitoes. This model detects water. |
| Various aedes and MBG sets | Skipped | Better-labelled sets cover the same ground |

## How the merge works

`training/merge_datasets.py` collapses everything into one single-class set.

| Input | What happens |
|--|--|
| Set with exactly one class | Kept whole, whatever the class is called. Covers sets whose only class is `water`, or a non-English word for puddle. |
| Set with several classes | Water-named classes kept, other boxes dropped |
| Image whose boxes all got dropped | Kept as a negative, empty label file |
| Roboflow `test` split | Folded into our validation split |

Those negatives are free training data: an image with tyres and bottles and no
water teaches the model what standing water is not. Folding in the test splits
rests on the real test set being the drone.

Two rules the merge enforces:

**No two versions of the same image pool.** Roboflow versions are re-splits and
re-augmentations of the same photographs. Merge two and the same image lands in
train and val, inflating the score while teaching nothing.

**`pool` and `water tank` are dropped on purpose.** They are real breeding sites,
so keeping them looks defensible, but they are not drop targets. A swimming pool
is permanent managed water and a granule drop into one is useless and unwelcome.
Training the model to fire on them would produce confident detections the mission
logic then has to suppress. This is `KEEP_NAMES` in `merge_datasets.py`.

## Credit line

For the submission and the video credits, once the URLs are recorded:

> The water-detection model was trained on publicly available datasets from
> Roboflow Universe, each licensed CC BY 4.0 and listed with full attribution
> in the project documentation, together with original photographs taken by
> the project team.
