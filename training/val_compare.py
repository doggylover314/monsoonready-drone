#!/usr/bin/env python3
"""Validation-only model comparison on the CURRENT dataset's val split.

No training: each model gets one .val() pass over dataset/data.yaml's val
images and a metrics row. Used after run2 to score the kept run1 yolov8n
fallback on the (harder) v2 val set, so the two runs are compared on the
same benchmark instead of their own-era val sets.

Run:     .venv/bin/python val_compare.py
Output:  table on stdout; per-model plots/PR curves under runs/val_<label>/
GPU:     needs the GPU free (do not run while train.py is running).
"""

from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "dataset" / "data.yaml"

MODELS = [
    ("run1_v8n", ROOT / "runs" / "puddle_v1" / "weights" / "best.pt"),
    ("run2_26n", ROOT / "runs" / "puddle" / "weights" / "best.pt"),
]

rows = []
for label, weights in MODELS:
    if not weights.exists():
        print(f"[skip] {label}: {weights} not found")
        continue
    r = YOLO(str(weights)).val(
        data=str(DATA),
        imgsz=640,
        batch=16,
        project=str(ROOT / "runs"),
        name=f"val_{label}",
        exist_ok=True,
    )
    rows.append((label, r.box.mp, r.box.mr, r.box.map50, r.box.map))

print(f"\n=== val split of {DATA} ===")
print(f"{'model':<10} {'P':>6} {'R':>6} {'mAP50':>7} {'mAP50-95':>9}")
for label, p, rec, m50, m in rows:
    print(f"{label:<10} {p:>6.3f} {rec:>6.3f} {m50:>7.3f} {m:>9.3f}")
