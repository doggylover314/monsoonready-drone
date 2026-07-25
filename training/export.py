#!/usr/bin/env python3
"""Export the best checkpoint so far for the UNO Q (Linux aarch64).

Produces ONNX next to the weights. ONNX runs on the UNO Q via onnxruntime
(pip install onnxruntime on the board). Run any time; uses best.pt if the
run has one, else last.pt.
"""

from pathlib import Path

from ultralytics import YOLO

W = Path(__file__).resolve().parent / "runs" / "puddle" / "weights"
ckpt = W / "best.pt" if (W / "best.pt").exists() else W / "last.pt"
if not ckpt.exists():
    raise SystemExit("No checkpoint yet: run train.py first")

print(f"Exporting {ckpt}")
YOLO(ckpt).export(format="onnx", imgsz=640, simplify=True)
print("Done. Copy the .onnx from runs/puddle/weights/ to the UNO Q.")
