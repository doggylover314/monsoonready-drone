"""Find and open the USB camera BY NAME, never by bare index.

WHY THIS FILE EXISTS (2026-08-16, the farm post-mortem). /dev/video numbers on
the UNO Q are a race: the Qualcomm Venus codec devices and the USB camera all
register V4L2 nodes, and whoever probes first gets video0. Both orders have
been observed on this exact board within one evening:

    boot with camera direct:   camera = video0/1, codecs = video2/3
    replug behind the hub:     codecs = video0/1, camera = video2/3

cv2.VideoCapture(0) on the second layout opens the VIDEO ENCODER, which fails
with the exact error the farm produced three times ("can't open camera by
index"). The camera never moved; the number did. So: resolve the device from
/sys/class/video4linux/*/name, which names the driver behind each node, and
try only nodes that belong to a real camera.

A UVC camera registers TWO nodes (capture + metadata) with the same name; the
capture node has the lower number (observed both layouts). We try candidates
in ascending order and require an actual frame before declaring success, so a
metadata node can never be picked.

Also owns focus locking and the diagnosis path: when no camera opens, say
WHY in plain words (no camera on USB at all, or the exact OS error and which
process is holding the node).
"""

import glob
import os

# Substrings that mark a V4L2 node as NOT a camera (platform codecs).
_NOT_CAMERA = ('venus', 'codec', 'decoder', 'encoder')

# The mission's capture resolution: the B525's maximum. The geometry
# (camera_geom.DEFAULT_HFOV_DEG) was measured at this resolution.
CAM_W, CAM_H = 1280, 720


class CameraError(RuntimeError):
    pass


def camera_nodes():
    """[(node_path, name)] for every V4L2 node that looks like a camera,
    ascending node number. Empty list = no camera on the bus at all."""
    out = []
    for sys_dir in sorted(glob.glob('/sys/class/video4linux/video*'),
                          key=lambda p: int(p.rsplit('video', 1)[1])):
        try:
            with open(os.path.join(sys_dir, 'name')) as f:
                name = f.read().strip()
        except OSError:
            continue
        if any(s in name.lower() for s in _NOT_CAMERA):
            continue
        out.append(('/dev/' + os.path.basename(sys_dir), name))
    return out


def holders(node):
    """[(pid, comm)] of processes with the node open. Same-user only: /proc
    fd links of other users are unreadable, so an empty answer means 'none of
    OUR processes', not 'nobody'."""
    out = []
    for pid_dir in glob.glob('/proc/[0-9]*'):
        try:
            for fd in os.listdir(os.path.join(pid_dir, 'fd')):
                if os.readlink(os.path.join(pid_dir, 'fd', fd)) == node:
                    with open(os.path.join(pid_dir, 'comm')) as f:
                        out.append((int(os.path.basename(pid_dir)),
                                    f.read().strip()))
                    break
        except OSError:
            continue
    return out


def diagnose(node):
    """One plain-words line about why `node` cannot be used."""
    try:
        os.close(os.open(node, os.O_RDWR))
    except OSError as exc:
        who = holders(node)
        held = (' held by ' + ', '.join(f'{c}(pid {p})' for p, c in who)
                ) if who else ''
        return f'{node}: {exc.strerror} (errno {exc.errno}){held}'
    who = holders(node)
    if who:
        return (f'{node}: opens, but is streaming in '
                + ', '.join(f'{c}(pid {p})' for p, c in who)
                + ' (V4L2 capture is exclusive)')
    return f'{node}: opens but gave no frame'


def _lock_focus(node, log=print):
    """Focus to infinity, best effort. Two separate v4l2-ctl calls: both in
    one transaction fails (EACCES, seen on the board 2026-08-15)."""
    import subprocess
    for ctrl in ('focus_automatic_continuous=0', 'focus_absolute=0'):
        try:
            r = subprocess.run(['v4l2-ctl', '-d', node, f'--set-ctrl={ctrl}'],
                               capture_output=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as exc:
            log(f'[camera] focus ctrl skipped ({ctrl}): {exc}')
            return
        if r.returncode != 0:
            log(f'[camera] focus ctrl failed ({ctrl}): '
                f'{r.stderr.decode().strip()}')


def open_camera(camera=None, width=CAM_W, height=CAM_H, log=print):
    """Open the USB camera and prove it with a real frame.

    camera: None/'auto' = resolve by name (the only mode flights use);
    an int or '/dev/videoN' pins a specific node for bench work.
    Returns (cv2.VideoCapture, node_path). Raises CameraError with a
    plain-words diagnosis when nothing usable opens.
    """
    import cv2

    if camera in (None, '', 'auto'):
        nodes = camera_nodes()
        if not nodes:
            raise CameraError(
                'camera missing from USB: no camera-like V4L2 device exists '
                '(only codecs or nothing). CHECK THE CAMERA PLUG AND THE HUB.')
    else:
        node = camera if isinstance(camera, str) else f'/dev/video{camera}'
        if not node.startswith('/dev/'):
            node = f'/dev/video{node}'
        nodes = [(node, 'pinned by argument')]

    tried = []
    for node, name in nodes:
        _lock_focus(node, log)
        cap = cv2.VideoCapture(node, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, frame = cap.read()
            if ok and frame is not None:
                h, w = frame.shape[:2]
                log(f'[camera] {node} ({name}) open at {w}x{h}')
                if (w, h) != (width, height):
                    log(f'[camera] WARNING: asked {width}x{height}, camera '
                        f'negotiated {w}x{h}')
                return cap, node
        cap.release()
        tried.append(diagnose(node))
    raise CameraError('no camera produced a frame. '
                      + ' | '.join(tried))
