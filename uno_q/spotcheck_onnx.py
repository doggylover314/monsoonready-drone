#!/usr/bin/env python3
"""Batch spot-check: run the puddle ONNX over a folder of images.

Writes one JSON line per image (sorted filenames, deterministic) plus an
annotated copy of every image and a single grid montage for the docs. Run
the SAME script on the laptop and on the UNO Q over the SAME folder, then
diff the two results.jsonl files: matching detections = the deployed
ONNX + onnxruntime + preprocessing chain reproduces the training machine.

    ~/venv/bin/python spotcheck_onnx.py --model ~/best.onnx --dir spotcheck \
        --out spotcheck_results

Output layout: <out>/results.jsonl, <out>/annotated/*.jpg, <out>/grid.jpg
"""

import argparse
import json
import os
import time

import cv2
import numpy as np
import onnxruntime as ort

SIZE = 640


def letterbox(img):
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
    ap.add_argument('--dir', required=True, help='folder of images')
    ap.add_argument('--out', default='spotcheck_results')
    ap.add_argument('--conf', type=float, default=0.25)
    args = ap.parse_args()

    sess = ort.InferenceSession(args.model,
                                providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name
    ann_dir = os.path.join(args.out, 'annotated')
    os.makedirs(ann_dir, exist_ok=True)

    names = sorted(f for f in os.listdir(args.dir)
                   if f.lower().endswith(('.jpg', '.jpeg', '.png')))
    results, annotated, times = [], [], []
    for name in names:
        img = cv2.imread(os.path.join(args.dir, name))
        if img is None:
            continue
        boxed, s, left, top = letterbox(img)
        x = boxed[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255
        t0 = time.perf_counter()
        out = sess.run(None, {iname: x})[0]
        times.append(time.perf_counter() - t0)
        dets = []
        for x1, y1, x2, y2, conf, cls in out[0]:
            if conf < args.conf:
                continue
            ox1, oy1 = (x1 - left) / s, (y1 - top) / s
            ox2, oy2 = (x2 - left) / s, (y2 - top) / s
            dets.append({'conf': round(float(conf), 3),
                         'box': [round(float(v), 1)
                                 for v in (ox1, oy1, ox2, oy2)]})
            cv2.rectangle(img, (int(ox1), int(oy1)), (int(ox2), int(oy2)),
                          (0, 200, 0), 2)
            cv2.putText(img, f"{conf:.2f}", (int(ox1), max(14, int(oy1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 0), 2)
        results.append({'image': name, 'detections': dets})
        cv2.imwrite(os.path.join(ann_dir, name), img)
        annotated.append(img)
        print(f"{name}: {len(dets)} dets")

    with open(os.path.join(args.out, 'results.jsonl'), 'w') as f:
        for r in results:
            f.write(json.dumps(r) + '\n')

    # grid montage: 6 columns of 320x240 thumbnails
    if not annotated:
        # cv2.imwrite throws on a zero-height array, and ms[len(ms)//2] two
        # lines later would IndexError anyway. Say WHY nothing was found:
        # the usual cause is a directory of .bmp/.webp, which the extension
        # filter above skips silently.
        print(f"no images matched in {args.dir} (looking for "
              f".jpg/.jpeg/.png). Nothing to spot-check.")
        return
    cols, tw, th = 6, 320, 240
    rows = (len(annotated) + cols - 1) // cols
    grid = np.full((rows * th, cols * tw, 3), 30, dtype=np.uint8)
    for i, img in enumerate(annotated):
        thumb = cv2.resize(img, (tw, th))
        r, c = divmod(i, cols)
        grid[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = thumb
    cv2.imwrite(os.path.join(args.out, 'grid.jpg'), grid)

    n_det = sum(len(r['detections']) for r in results)
    ms = sorted(t * 1000 for t in times)
    print(f"\n{len(results)} images, {n_det} detections total")
    print(f"median inference {ms[len(ms) // 2]:.0f}ms")
    print(f"wrote {args.out}/results.jsonl, annotated/, grid.jpg")


if __name__ == '__main__':
    main()
