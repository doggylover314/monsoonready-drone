#!/usr/bin/env python3
"""Merge multiple Roboflow YOLO-format exports into one single-class dataset.

Usage:
  1. On Roboflow, open each chosen dataset -> Download Dataset -> format
     "YOLOv8" (a.k.a. YOLOv8 PyTorch TXT) -> download zip.
  2. Unzip each into training/exports/<any-name>/ so each folder contains
     data.yaml and train/ valid/ (test/ optional) subfolders.
  3. .venv/bin/python merge_datasets.py     (venv python: needs PyYAML)
     -> writes training/dataset/ with images/labels for train+val and a
        data.yaml with the single class "puddle" (class 0).

Class handling per export:
  * exactly one class -> kept, whatever it is named (covers sets whose only
    class is "water", a foreign-language word for puddle, etc.);
  * multiple classes -> ONLY puddle/water-named classes are kept (see
    KEEP_NAMES); boxes of other classes (tire, bottle, pool, ...) are
    dropped so they never train as "puddle". An image left with no boxes
    stays in as a negative.

Re-runnable: wipes and rebuilds training/dataset/ each time.
File names are prefixed with the export folder name to avoid collisions.
"""

import shutil
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
EXPORTS = ROOT / "exports"
OUT = ROOT / "dataset"

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
# Roboflow split dir -> our split (test folds into val: more honest val metric,
# and the real test is the drone)
SPLIT_MAP = {"train": "train", "valid": "val", "test": "val"}

# Multi-class exports: keep a class if its lowercased name is here or
# contains "puddle". "water tanks"/"pool" deliberately do NOT match: they are
# breeding sites but not drop targets for this mission.
KEEP_NAMES = {"puddle", "water", "standing water", "stagnant water", "stagnant_water", "temporary water sites", "probable-stagnant-water-waist"}


def keep_ids(export: Path) -> set[int] | None:
    """Class ids to keep for this export, or None to keep everything."""
    data_yaml = export / "data.yaml"
    if not data_yaml.exists():
        print(f"WARNING: {export.name}: no data.yaml, keeping all classes")
        return None
    names = yaml.safe_load(data_yaml.read_text()).get("names")
    if isinstance(names, dict):
        items = [(int(k), str(v)) for k, v in names.items()]
    elif isinstance(names, list):
        items = list(enumerate(str(n) for n in names))
    else:
        print(f"WARNING: {export.name}: unreadable names, keeping all classes")
        return None
    if len(items) <= 1:
        return None  # single-class set: keep regardless of name
    kept = {i for i, n in items
            if n.lower() in KEEP_NAMES or "puddle" in n.lower()}
    dropped = [n for i, n in items if i not in kept]
    print(f"  {export.name}: keeping {[n for i, n in items if i in kept]}, "
          f"dropping {dropped}")
    if not kept:
        print(f"WARNING: {export.name}: no puddle-like class found; "
              f"images become pure negatives")
    return kept


def filter_label(src: Path, dst: Path, kept: set[int] | None) -> None:
    lines_out = []
    for line in src.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 5:  # class x y w h [seg points...]
            if kept is not None and int(parts[0]) not in kept:
                continue  # drop non-puddle box; image may become a negative
            parts[0] = "0"
            lines_out.append(" ".join(parts))
    dst.write_text("\n".join(lines_out) + ("\n" if lines_out else ""))


def main() -> None:
    exports = [d for d in sorted(EXPORTS.iterdir()) if d.is_dir()] if EXPORTS.exists() else []
    if not exports:
        sys.exit(f"No exports found. Unzip Roboflow YOLOv8 exports into {EXPORTS}/<name>/")

    if OUT.exists():
        shutil.rmtree(OUT)
    for split in ("train", "val"):
        (OUT / "images" / split).mkdir(parents=True)
        (OUT / "labels" / split).mkdir(parents=True)

    counts = {"train": 0, "val": 0}
    for exp in exports:
        kept = keep_ids(exp)
        for rf_split, split in SPLIT_MAP.items():
            img_dir = exp / rf_split / "images"
            lbl_dir = exp / rf_split / "labels"
            if not img_dir.is_dir():
                continue
            for img in img_dir.iterdir():
                if img.suffix.lower() not in IMG_EXTS:
                    continue
                stem = f"{exp.name}_{img.stem}"
                shutil.copy2(img, OUT / "images" / split / (stem + img.suffix.lower()))
                lbl = lbl_dir / (img.stem + ".txt")
                dst_lbl = OUT / "labels" / split / (stem + ".txt")
                if lbl.exists():
                    filter_label(lbl, dst_lbl, kept)
                else:  # negative image: empty label file
                    dst_lbl.write_text("")
                counts[split] += 1
        print(f"merged: {exp.name}")

    (OUT / "data.yaml").write_text(
        f"path: {OUT}\ntrain: images/train\nval: images/val\nnames:\n  0: puddle\n"
    )
    print(f"done: {counts['train']} train / {counts['val']} val images -> {OUT}")


if __name__ == "__main__":
    main()
