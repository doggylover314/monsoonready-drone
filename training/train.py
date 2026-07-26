#!/usr/bin/env python3
"""Puddle-detector training. Pausable at any moment with Ctrl+C: ultralytics
checkpoints every epoch to runs/puddle/weights/last.pt, and this script
auto-resumes from it on the next run. Delete runs/puddle/ to start fresh.

Run:      .venv/bin/python train.py
Monitor:  runs/puddle/results.csv (mAP per epoch), runs/puddle/*.jpg previews
Export:   .venv/bin/python export.py   (after/any time during training)

Sized for the RTX 3050 4GB: yolo26n at 640px, batch 16 (run1 at batch 8 used
only ~1GB). If you hit CUDA out-of-memory, drop BATCH back to 8.
"""

from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "dataset" / "data.yaml"
RUN = ROOT / "runs" / "puddle"

EPOCHS = 600      # effectively "until plateau": patience=50 stops it earlier
BATCH = 16
IMGSZ = 640

last = RUN / "weights" / "last.pt"
if last.exists():
    print(f"Resuming from {last}")
    YOLO(last).train(resume=True)
else:
    if not DATA.exists():
        raise SystemExit("dataset/data.yaml missing: run merge_datasets.py first")
    # yolo26n over yolov8n (decided 2026-07-25): +3.6 COCO mAP, ~2x faster CPU
    # ONNX (NMS-free export, no postprocessing on the UNO Q), small-target-
    # aware assignment suits puddles at survey altitude.
    YOLO("yolo26n.pt").train(
        data=str(DATA),
        epochs=EPOCHS,
        batch=BATCH,
        imgsz=IMGSZ,
        patience=50,
        project=str(RUN.parent),
        name=RUN.name,
        exist_ok=True,
        # nadir drone views have no canonical "up": free augmentation
        degrees=180,
        flipud=0.5,
    )
