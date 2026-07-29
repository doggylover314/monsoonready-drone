#!/usr/bin/env python3
"""ONNX benchmark + spot-check for the puddle model on the UNO Q.

Times inference on one image and prints every detection above the confidence
threshold; optionally writes an annotated copy for eyeballing. Expects the
yolo26 end-to-end export: output (1, 300, 6) rows of x1,y1,x2,y2,conf,cls in
letterboxed 640x640 pixel space (no NMS needed).

    ~/venv/bin/python benchmark_onnx.py --model ~/best.onnx --image test.jpg
    ~/venv/bin/python benchmark_onnx.py --model ~/best.onnx --image test.jpg \
        --save annotated.jpg

Timing note: the first run pays one-off graph-optimization cost; it is
excluded via warmup. Reported fps = single-image latency on the A53 CPU,
which is the mission-relevant number (stills while hovering, ~1s/frame ok).
"""

import argparse
import statistics
import time

import cv2
import numpy as np
import onnxruntime as ort

SIZE = 640


def letterbox(img):
    """Resize keeping aspect, pad with gray 114 to SIZE x SIZE (ultralytics
    convention). Returns the tensor image plus scale/pad for un-mapping."""
    h, w = img.shape[:2]
    s = min(SIZE / h, SIZE / w)
    nh, nw = round(h * s), round(w * s)
    top = (SIZE - nh) // 2
    left = (SIZE - nw) // 2
    out = np.full((SIZE, SIZE, 3), 114, dtype=np.uint8)
    out[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
    return out, s, left, top


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--model', required=True)
    ap.add_argument('--image', required=True)
    ap.add_argument('--runs', type=int, default=20)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--save', metavar='OUT_JPG',
                    help='write annotated copy of the image here')
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"cannot read {args.image}")
    boxed, s, left, top = letterbox(img)
    x = boxed[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0

    sess = ort.InferenceSession(args.model,
                                providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name
    print(f"model: {args.model}")
    print(f"image: {args.image} {img.shape[1]}x{img.shape[0]}")

    for _ in range(3):                       # warmup (graph opt, allocators)
        sess.run(None, {iname: x})
    times = []
    for _ in range(args.runs):
        t0 = time.perf_counter()
        out = sess.run(None, {iname: x})[0]
        times.append(time.perf_counter() - t0)
    ms = [t * 1000 for t in times]
    print(f"latency over {args.runs} runs: median {statistics.median(ms):.0f}ms"
          f"  mean {statistics.mean(ms):.0f}ms  min {min(ms):.0f}ms"
          f"  max {max(ms):.0f}ms  -> {1000 / statistics.median(ms):.2f} fps")

    dets = [r for r in out[0] if r[4] >= args.conf]
    print(f"detections (conf >= {args.conf}): {len(dets)}")
    for x1, y1, x2, y2, conf, cls in dets:
        # un-map letterbox coords back to original image pixels
        ox1, oy1 = (x1 - left) / s, (y1 - top) / s
        ox2, oy2 = (x2 - left) / s, (y2 - top) / s
        print(f"  puddle conf {conf:.2f} box "
              f"({ox1:.0f},{oy1:.0f})-({ox2:.0f},{oy2:.0f})")
        if args.save:
            cv2.rectangle(img, (int(ox1), int(oy1)), (int(ox2), int(oy2)),
                          (0, 200, 0), 2)
            cv2.putText(img, f"{conf:.2f}", (int(ox1), int(oy1) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
    if args.save:
        cv2.imwrite(args.save, img)
        print(f"annotated copy: {args.save}")


if __name__ == '__main__':
    main()
