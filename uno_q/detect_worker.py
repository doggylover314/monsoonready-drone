#!/usr/bin/env python3
"""Camera + ONNX inference in a process of its own (2026-08-01 split).

Measured on the board: yolo26n blocks 511ms per frame, yolo26s 1518ms. Run
in-process (the old way) that is time the single-threaded mission loop is
not pumping MAVLink and not noticing a pilot override. Run here, the mission
loop never blocks: this process captures, infers, and atomically rewrites
one small JSON file; FileDetector in the mission process reads it.

    ~/venv/bin/python ~/uno_q/detect_worker.py \
        --model ~/best.onnx --camera 1 --conf 0.5

run_mission.py launches this itself (and reuses an already-running one, so
starting it by hand first is fine too). It owns the camera exclusively; do
not run it and an --inline-detector mission at the same time.

Output file contract (all FileDetector relies on):
  seq        increments every cycle; unchanged seq = nothing new
  t_frame    time.time() at CAPTURE (not after inference), for telemetry
             pairing in the mission process (same machine, same clock)
  w, h       frame size, so geometry mismatches are still caught
  rows       [x1,y1,x2,y2,conf] per detection at/above --conf, letterbox space
  camera_ok  false = this process is alive but the camera gave no frame
The file is written empty-rows-included every cycle: its mtime is the
heartbeat that tells the mission process this worker is alive.

Writes go to a temp file then os.replace(), which is atomic on the same
filesystem: the reader sees the old payload or the new one, never half.
Default output is under /tmp (tmpfs) so a mission's worth of once-a-second
writes never touches the SD card.
"""

import argparse
import json
import os
import time

from detector import OnnxDetector

DEFAULT_OUT = '/tmp/monsoonready_det.json'


def save_annotated(save_dir, seq, frame, rows, w, h):
    """Write the frame with its detection boxes drawn on it.

    The boxes come back from the model in LETTERBOX space (the padded 640
    square), so they have to be un-padded and un-scaled before they mean
    anything about the real image; camera_geom.letterbox_to_frame is the same
    conversion the flight code uses to locate a puddle, reused here so the
    picture cannot disagree with the mission's own maths.
    """
    import cv2
    from camera_geom import letterbox_to_frame
    os.makedirs(save_dir, exist_ok=True)
    img = frame.copy()
    for x1, y1, x2, y2, conf in rows:
        ax, ay = letterbox_to_frame(x1, y1, w, h, OnnxDetector.SIZE)
        bx, by = letterbox_to_frame(x2, y2, w, h, OnnxDetector.SIZE)
        cv2.rectangle(img, (int(ax), int(ay)), (int(bx), int(by)),
                      (0, 255, 0), 2)
        cv2.putText(img, f"{conf:.2f}", (int(ax), max(12, int(ay) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    path = os.path.join(save_dir, f"det_{seq:06d}.jpg")
    cv2.imwrite(path, img)
    return path


def write_atomic(path, payload):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(payload, f)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--camera', type=int, default=1)
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--save-dir', default=None,
                    help='write an annotated JPEG for every frame that has a '
                         'detection. This is the docs/07 "unoq_detection" '
                         'evidence and the only way to see WHAT the board '
                         'detected during a field flight, since nothing else '
                         'keeps the image. Off by default: it writes to disk '
                         'on every hit and must not run unattended in flight.')
    ap.add_argument('--interval', type=float, default=1.0,
                    help='min seconds between capture starts. Slower models '
                         'simply run back to back; 0 pegs a core for nothing '
                         'gained at mission timescales.')
    args = ap.parse_args()
    out = os.path.expanduser(args.out)

    # OnnxDetector is used as engine only (infer_rows): no telemetry, no
    # dedup, no geometry here. Site logic stays in the mission process, which
    # is the only place the telemetry to do it lives.
    det = OnnxDetector(os.path.expanduser(args.model), camera=args.camera,
                       conf=args.conf)
    print(f"[worker] model {args.model}, camera {args.camera}, "
          f"conf {args.conf}, writing {out} every <= {args.interval}s",
          flush=True)

    seq = 0
    while True:
        t0 = time.time()
        res = det.infer_rows()
        seq += 1
        if res is None:
            payload = {'seq': seq, 't_frame': t0, 'camera_ok': False,
                       'w': None, 'h': None, 'rows': []}
        else:
            t_frame, w, h, rows, frame = res
            payload = {'seq': seq, 't_frame': t_frame, 'camera_ok': True,
                       'w': w, 'h': h,
                       'rows': [[float(v) for v in r[:5]] for r in rows]}
            if args.save_dir and rows:
                save_annotated(args.save_dir, seq, frame, payload['rows'],
                               w, h)
        write_atomic(out, payload)
        if payload['rows']:
            best = max(r[4] for r in payload['rows'])
            print(f"[worker] seq {seq}: {len(payload['rows'])} detection(s), "
                  f"best {best:.2f}", flush=True)
        time.sleep(max(0.0, args.interval - (time.time() - t0)))


if __name__ == '__main__':
    main()
