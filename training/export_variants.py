#!/usr/bin/env python3
"""Export PRETRAINED yolo26 variants to ONNX so their speed can be measured
on the UNO Q without spending a training run first.

The point is a timing number, nothing else. These are the stock COCO
checkpoints, so their detections mean nothing for puddles: what transfers is
the architecture, the input shape and the operator mix, which is what decides
how long a forward pass takes on an A53. If a variant turns out to be fast
enough, THEN it is worth ~12 h of GPU to train it properly on dataset v2.

    training/.venv/bin/python training/export_variants.py yolo26s yolo26m

Ultralytics downloads any checkpoint it does not already have. If a name does
not exist upstream it will say so; nothing here guesses at what is published.
Output lands in training/variants/<name>.onnx.
"""

import sys
from pathlib import Path

from ultralytics import YOLO

OUT = Path(__file__).resolve().parent / "variants"


def main(names):
    OUT.mkdir(exist_ok=True)
    for name in names:
        print(f"\n=== {name} ===")
        # Exported at the SAME imgsz the mission uses. Changing this changes
        # the timing, so a 416 export is a separate experiment, not a tweak.
        path = YOLO(f"{name}.pt").export(format="onnx", imgsz=640,
                                         simplify=True)
        dest = OUT / f"{name}.onnx"
        Path(path).replace(dest)
        mb = dest.stat().st_size / 1e6
        print(f"{dest}  ({mb:.1f} MB)")
    print(f"\nCopy {OUT}/*.onnx to the UNO Q, then run uno_q/bench_models.py "
          f"there. Timing measured anywhere else is not the number we need.")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit("usage: export_variants.py <name> [<name> ...]  "
                         "e.g. yolo26s yolo26m")
    main(sys.argv[1:])
