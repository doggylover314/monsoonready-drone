#!/usr/bin/env python3
"""Time one or more ONNX detectors on the UNO Q. Board-side; onnxruntime,
numpy and cv2 only, no ultralytics, no torch.

    ~/venv/bin/python ~/uno_q/bench_models.py \
        --images ~/spotcheck \
        ~/uno_q/best.onnx ~/uno_q/yolo26s.onnx ~/uno_q/yolo26m.onnx

Answers one question: how long does a frame take, end to end, on this board.

WHY THIS SCRIPT AND NOT benchmark_onnx.py: that one profiles the flight model
on one image. This one compares SEVERAL models over the SAME images with the
same preprocessing, warms each up first, and reports the spread rather than a
single number, because the decision it feeds ("is yolo26s affordable") turns
on the slow frames, not the average one.

The preprocessing is byte-identical to detector.OnnxDetector on purpose. A
benchmark that preprocesses differently from the flight code is measuring a
program that will never run.

REPORTED NUMBERS
  preprocess   letterbox + colour + scale, per frame
  inference    the ONNX session run alone
  total        what the mission loop actually blocks for

`total` is the one that matters, because uno_q/mission.py is single threaded:
while poll() is inside this, no MAVLink is being pumped and no setpoint is
going out. That is the real cost of a bigger model, not the frame rate.
"""

import argparse
import glob
import os
import statistics
import sys
import time

SIZE = 640


def letterbox(frame, np, cv2, size=SIZE):
    """Identical to detector.OnnxDetector.poll's preprocessing."""
    h, w = frame.shape[:2]
    s = min(size / h, size / w)
    nh, nw = round(h * s), round(w * s)
    top, left = (size - nh) // 2, (size - nw) // 2
    boxed = np.full((size, size, 3), 114, dtype=np.uint8)
    boxed[top:top + nh, left:left + nw] = cv2.resize(frame, (nw, nh))
    return boxed[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255


def load_images(pattern, cv2, limit):
    paths = sorted(p for p in glob.glob(os.path.expanduser(pattern))
                   if p.lower().endswith(('.jpg', '.jpeg', '.png')))
    if not paths:
        raise SystemExit(f"no images matched {pattern}")
    paths = paths[:limit]
    frames = []
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"  skipped unreadable {p}")
            continue
        frames.append(img)
    if not frames:
        raise SystemExit("no readable images")
    return frames


def bench(model_path, frames, np, cv2, ort, warmup, threads):
    opts = ort.SessionOptions()
    if threads:
        opts.intra_op_num_threads = threads
    sess = ort.InferenceSession(model_path, opts,
                                providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0]
    out_shape = sess.get_outputs()[0].shape
    # Letterbox to the model's OWN input size, read from the ONNX, so a
    # 1280-input export is benchmarked as itself instead of erroring on a
    # 640 tensor. Dynamic-dim exports fall back to the mission's 640.
    size = inp.shape[2] if isinstance(inp.shape[2], int) else SIZE

    # Warm up on a real frame: the first run pays for lazy kernel selection
    # and memory arena growth, and counting it would flatter every model that
    # happens to be measured with a short run.
    for _ in range(warmup):
        sess.run(None, {inp.name: letterbox(frames[0], np, cv2, size)})

    pre_ms, inf_ms = [], []
    for f in frames:
        t0 = time.perf_counter()
        x = letterbox(f, np, cv2, size)
        t1 = time.perf_counter()
        sess.run(None, {inp.name: x})
        t2 = time.perf_counter()
        pre_ms.append((t1 - t0) * 1000)
        inf_ms.append((t2 - t1) * 1000)

    tot = [a + b for a, b in zip(pre_ms, inf_ms)]
    return {
        'input': f"{inp.name}{inp.shape}",
        'output': out_shape,
        'size_mb': os.path.getsize(model_path) / 1e6,
        'pre': statistics.median(pre_ms),
        'inf': statistics.median(inf_ms),
        'med': statistics.median(tot),
        'min': min(tot),
        'max': max(tot),
        'fps': 1000.0 / statistics.median(tot),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('models', nargs='+', help='ONNX files to compare')
    ap.add_argument('--images', default='~/spotcheck/*',
                    help='glob of test images (real frames, not blanks)')
    ap.add_argument('--limit', type=int, default=24)
    ap.add_argument('--warmup', type=int, default=3)
    ap.add_argument('--threads', type=int, default=0,
                    help='intra-op threads; 0 = onnxruntime default')
    args = ap.parse_args()

    import numpy as np
    import cv2
    import onnxruntime as ort

    # Silence the /sys/class/drm GPU-probe warning (no GPU on this board to
    # find; the probe failing is the expected outcome, not a problem).
    ort.set_default_logger_severity(3)
    print(f"onnxruntime {ort.__version__}, cpu count {os.cpu_count()}, "
          f"threads {args.threads or 'default'}")
    frames = load_images(args.images, cv2, args.limit)
    print(f"{len(frames)} images, first is {frames[0].shape[1]}x"
          f"{frames[0].shape[0]}\n")

    rows = []
    for m in args.models:
        m = os.path.expanduser(m)
        if not os.path.exists(m):
            print(f"MISSING {m}")
            continue
        print(f"benchmarking {os.path.basename(m)} ...")
        r = bench(m, frames, np, cv2, ort, args.warmup, args.threads)
        r['name'] = os.path.basename(m)
        rows.append(r)
        print(f"  input {r['input']} output {r['output']}")

    if not rows:
        sys.exit("nothing benchmarked")

    print(f"\n{'model':<22}{'MB':>6}{'pre':>8}{'infer':>9}{'TOTAL':>9}"
          f"{'min':>8}{'max':>8}{'fps':>7}")
    for r in rows:
        print(f"{r['name']:<22}{r['size_mb']:>6.1f}{r['pre']:>7.0f}m"
              f"{r['inf']:>8.0f}m{r['med']:>8.0f}m{r['min']:>7.0f}m"
              f"{r['max']:>7.0f}m{r['fps']:>7.2f}")

    base = rows[0]
    if len(rows) > 1:
        print(f"\nrelative to {base['name']}:")
        for r in rows[1:]:
            print(f"  {r['name']:<20} {r['med'] / base['med']:.2f}x slower")

    print("\nTOTAL is what mission.py blocks for on every detector poll. The "
          "mission loop is single threaded, so nothing is pumped and no "
          "setpoint goes out while it runs. Compare it against "
          "detector.OnnxDetector.interval_s (currently 1.0s): a model whose "
          "TOTAL exceeds that is the throttle, and the throttle no longer "
          "means what it says.")


if __name__ == '__main__':
    main()
