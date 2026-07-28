#!/usr/bin/env python3
"""Puddle-detector training. Pausable at any moment with Ctrl+C: ultralytics
checkpoints every epoch to runs/puddle/weights/last.pt, and this script
auto-resumes from it on the next run. Delete runs/puddle/ to start fresh.

Run:      .venv/bin/python train.py
Monitor:  runs/puddle/results.csv (mAP per epoch), runs/puddle/*.jpg previews
Export:   .venv/bin/python export.py   (after/any time during training)

Sized for the RTX 3050 4GB laptop: yolo26n at 640px, batch 8, workers 4.
Batch 16 fit the GPU fine but the dataloader workers' shared-memory buffers
plus desktop apps exhausted system RAM overnight (kernel OOM-killed the run
twice, 2026-07-27/28); batch 8 + 4 workers keeps the host RAM footprint small.
"""

from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "dataset" / "data.yaml"
RUN = ROOT / "runs" / "puddle"

EPOCHS = 600      # effectively "until plateau": patience=50 stops it earlier
BATCH = 8         # 16 fits the GPU but OOMs system RAM (see docstring)
WORKERS = 4
IMGSZ = 640

last = RUN / "weights" / "last.pt"
if last.exists():
    print(f"Resuming from {last}")
    # batch/workers are in ultralytics' resume-overridable set ("allow arg
    # updates to reduce memory"); the checkpoint carries batch=16 which OOMs
    # host RAM, so they must be overridden here, not just in the fresh path.
    YOLO(last).train(resume=True, batch=BATCH, workers=WORKERS)
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
        workers=WORKERS,
        imgsz=IMGSZ,
        patience=50,
        project=str(RUN.parent),
        name=RUN.name,
        exist_ok=True,
        # nadir drone views have no canonical "up": free augmentation
        degrees=180,
        flipud=0.5,
    )
