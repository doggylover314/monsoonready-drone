#!/usr/bin/env python3
"""Camera + ONNX inference in a process of its own (2026-08-01 split).

Measured on the board: yolo26n blocks 511ms per frame, yolo26s 1518ms. Run
in-process (the old way), and that is time the single-threaded mission loop
is not pumping MAVLink and not noticing a pilot override. Run here instead
and the mission loop never blocks: this process captures, infers, and
atomically rewrites one small JSON file; FileDetector in the mission process
reads it.

    ~/venv/bin/python uno_q/detect_worker.py --model models/best.onnx

run_mission.py launches this itself (and reuses an already-running one, so
starting it by hand first is fine too). It owns the camera exclusively; do
not run it and an --inline-detector mission at the same time.

The photo pipeline (user spec 2026-08-16). The camera stays open for the
whole run and capture is paced by inference, one frame in hand at a time:
the next frame is grabbed as soon as the previous one's processing cycle
completes, so there is never an idle gap and never a backlog (the V4L2
buffer is pinned to 1 and flushed before every read, so a grab always
returns the freshest frame, not a stale queued one). Every captured frame
is saved full-resolution to --photo-dir as future training data; the folder
is capped at --photo-cap-gb with the oldest photos deleted first.

Manual photos (dashboard button): the dashboard writes a request file
(MANUAL_REQ) whose content is the target directory; this loop notices it,
saves the current frame there, and answers in MANUAL_DONE with the saved
path. Manual photos are never inferred on and never counted; they are only
saved and shown.

Output file contract (all FileDetector relies on):
  seq        increments every cycle; unchanged seq = nothing new
  t_frame    time.time() at capture (not after inference), for telemetry
             pairing in the mission process (same machine, same clock)
  w, h       frame size, so geometry mismatches are still caught
  rows       [x1,y1,x2,y2,conf] per detection at/above --conf, letterbox space
  camera_ok  false = this process is alive but the camera gave no frame
  error      (only with camera_ok false) plain-words diagnosis: the errno,
             the holding process, or "camera missing from USB". This is what
             the mission's preflight prints, so the farm failure mode now
             names itself instead of reading as a bare refusal.
The file is written empty-rows-included every cycle: its mtime is the
heartbeat that tells the mission process this worker is alive.

Writes go to a temp file then os.replace(), which is atomic on the same
filesystem: the reader sees the old payload or the new one, never half.
Default output is under /tmp (tmpfs) so the once-per-cycle writes never
touch the eMMC; photos do go to the eMMC, deliberately: they are the point.

A dead camera is reported every cycle but never retried automatically
(user, 2026-08-16: diagnosis belongs to the dashboard's test button, not to
an automatic loop). Restarting the worker is the retry.
"""

import argparse
import glob
import json
import os
import time

from boardlog import BoardLog, IST
from camera import CameraError, diagnose, open_camera
from datetime import datetime
from detector import OnnxDetector

DEFAULT_OUT = '/tmp/monsoonready_det.json'
DEFAULT_PHOTO_DIR = '~/monsoonready_data/photos'
DEFAULT_MANUAL_DIR = '~/monsoonready_data/manual_photos'
MANUAL_REQ = '/tmp/monsoonready_manual_photo'
MANUAL_DONE = '/tmp/monsoonready_manual_photo_done'
CAP_CHECK_EVERY = 100          # photos between folder-size enforcements


def save_annotated(save_dir, seq, frame, rows, w, h):
    """Write the frame with its detection boxes drawn on it.

    The boxes come back from the model in letterbox space (the padded 640
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


def stamp_name(seq):
    """IST-timestamped, seq-suffixed, so names sort in capture order even
    across a clock jump (NTP landing mid-run)."""
    return (datetime.now(IST).strftime('%Y%m%d_%H%M%S_%f')[:-3]
            + f'_{seq:06d}.jpg')


def enforce_cap(photo_dir, cap_bytes, log):
    """Delete the oldest photos until the folder is back under cap_bytes.
    Oldest by mtime, which is capture order. Never touches non-jpg files."""
    files = []
    total = 0
    for p in glob.glob(os.path.join(photo_dir, '*.jpg')):
        try:
            st = os.stat(p)
        except OSError:
            continue
        files.append((st.st_mtime, st.st_size, p))
        total += st.st_size
    if total <= cap_bytes:
        return
    files.sort()                              # oldest first
    freed = 0
    dropped = 0
    for _mt, size, p in files:
        if total - freed <= cap_bytes:
            break
        try:
            os.remove(p)
            freed += size
            dropped += 1
        except OSError:
            continue
    log.warn(f'photo folder over {cap_bytes / 1e9:.0f} GB: deleted '
             f'{dropped} oldest photo(s), freed {freed / 1e6:.0f} MB')


def manual_photo_requested():
    """Target dir if the dashboard asked for a manual photo, else None."""
    try:
        with open(MANUAL_REQ) as f:
            target = f.read().strip()
    except OSError:
        return None
    return target or os.path.expanduser(DEFAULT_MANUAL_DIR)


def take_manual_photo(cv2, frame, target_dir, seq, log):
    try:
        os.makedirs(target_dir, exist_ok=True)
        path = os.path.join(target_dir, 'manual_' + stamp_name(seq))
        cv2.imwrite(path, frame)
        reply = {'ok': True, 'path': path}
        log(f'[worker] manual photo saved: {path}')
    except OSError as exc:
        reply = {'ok': False, 'error': str(exc)}
        log.error(f'manual photo FAILED: {exc}')
    try:
        os.remove(MANUAL_REQ)
    except OSError:
        pass
    write_atomic(MANUAL_DONE, reply)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--camera', default='auto',
                    help="'auto' resolves the USB camera BY NAME (the only "
                         "mode flights use; /dev/video numbers are an "
                         "enumeration race, see camera.py). An integer or "
                         "/dev/videoN pins a node for bench work.")
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--photo-dir', default=DEFAULT_PHOTO_DIR,
                    help='every captured frame is saved here full-res as '
                         'future training data')
    ap.add_argument('--photo-cap-gb', type=float, default=10.0,
                    help='photo folder ceiling; OLDEST photos deleted first')
    ap.add_argument('--save-dir', default=None,
                    help='additionally write an ANNOTATED JPEG (boxes drawn) '
                         'for every frame that has a detection: the docs/07 '
                         '"unoq_detection" evidence. Off by default.')
    ap.add_argument('--interval', type=float, default=0.0,
                    help='min seconds between capture starts. 0 (default) = '
                         'the photo pipeline: capture the next frame the '
                         'moment the previous one finishes processing.')
    args = ap.parse_args()
    log = BoardLog('detect_worker')
    out = os.path.expanduser(args.out)
    photo_dir = os.path.expanduser(args.photo_dir)
    cap_bytes = int(args.photo_cap_gb * 1e9)
    os.makedirs(photo_dir, exist_ok=True)

    import cv2

    cam_error = None
    cap = None
    try:
        cap, node = open_camera(args.camera, log=log)
    except CameraError as exc:
        cam_error = str(exc)
        log.error(f'camera did not open: {cam_error}')

    # OnnxDetector here is just the inference engine (infer_rows): no
    # telemetry, no dedup, no geometry. That logic stays in the mission
    # process, the only place with the telemetry to run it. frame_source is
    # our own capture object, so this loop still gets direct access to the
    # raw frame for the photo saves.
    def grab():
        cap.grab()                       # flush the 1-deep buffer: freshest
        ok, frame = cap.read()
        return frame if ok else None

    det = OnnxDetector(os.path.expanduser(args.model), conf=args.conf,
                       frame_source=(grab if cap is not None
                                     else (lambda: None)), log=log)
    log(f'[worker] model {args.model}, conf {args.conf}, out {out}, '
        f'photos {photo_dir} (cap {args.photo_cap_gb:g} GB), '
        f'interval {args.interval:g}s')
    enforce_cap(photo_dir, cap_bytes, log)

    seq = 0
    photos_since_check = 0
    grab_fail_logged = False
    while True:
        t0 = time.time()
        res = det.infer_rows()
        seq += 1
        if res is None:
            err = cam_error
            if err is None and cap is not None:
                err = diagnose(node)
                if not grab_fail_logged:
                    log.error(f'camera stopped giving frames: {err}')
                    grab_fail_logged = True
            payload = {'seq': seq, 't_frame': t0, 'conf': args.conf,
                       'camera_ok': False,
                       'w': None, 'h': None, 'rows': [], 'error': err}
            write_atomic(out, payload)
            time.sleep(1.0)              # report cadence, not a retry
            continue
        grab_fail_logged = False
        t_frame, w, h, rows, frame = res
        payload = {'seq': seq, 't_frame': t_frame, 'conf': args.conf,
                   'camera_ok': True,
                   'w': w, 'h': h,
                   # Blur metric for every frame, detection or not (see
                   # OnnxDetector.infer_rows). Nothing acts on it yet; it is
                   # here so the pre-take flight produces the numbers a
                   # threshold would have to be chosen from.
                   'sharpness': getattr(det, 'last_sharpness', None),
                   'rows': [[float(v) for v in r[:5]] for r in rows]}

        try:
            cv2.imwrite(os.path.join(photo_dir, stamp_name(seq)), frame)
            photos_since_check += 1
        except (OSError, cv2.error) as exc:
            log.error(f'photo save failed: {exc}')
        if photos_since_check >= CAP_CHECK_EVERY:
            photos_since_check = 0
            enforce_cap(photo_dir, cap_bytes, log)

        target = manual_photo_requested()
        if target:
            take_manual_photo(cv2, frame, target, seq, log)

        if args.save_dir and payload['rows']:
            save_annotated(args.save_dir, seq, frame, payload['rows'], w, h)
        write_atomic(out, payload)
        sharp = payload['sharpness']
        sharp_txt = f', sharpness {sharp:.0f}' if sharp is not None else ''
        if payload['rows']:
            best = max(r[4] for r in payload['rows'])
            log(f'[worker] seq {seq}: {len(payload["rows"])} detection(s), '
                f'best {best:.2f}{sharp_txt}')
        elif sharp is not None and seq % 10 == 0:
            # Every tenth blank frame, so a whole flight's blur profile is in
            # the log without a line per frame.
            log(f'[worker] seq {seq}: no detections{sharp_txt}')
        time.sleep(max(0.0, args.interval - (time.time() - t0)))


if __name__ == '__main__':
    main()
