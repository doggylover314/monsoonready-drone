"""Append-only file logging for every program that runs on the board.

Every board program owns exactly one log file, named after the script
(SCOPE RULES 1, user 2026-08-15/16): run_mission.py -> ~/logs/run_mission.log,
detect_worker.py -> ~/logs/detect_worker.log, and so on. The program opens the
file itself, so it logs identically whether it was started by hand, by the
dashboard, or detached under nohup; shell redirection is no longer part of the
design.

Line format (user spec 2026-08-15, answers 6/7):

    2026-08-16 00:12:34.567 IST +1234.5s INFO  message

Both clocks on every line, because they fail in opposite ways: the wall clock
is authoritative when NTP has synced and lies by hours when it has not (the
board has no RTC battery; at the farm it woke up 6 h behind), while
seconds-since-boot (/proc/uptime) is always monotonic and always comparable to
dmesg, but means nothing across a reboot.

Retention (user, 2026-08-15): logs are never overwritten and never rotated
into .1/.2 files. Append forever. The single exception: when the file exceeds
100 MB at open time, the oldest lines are dropped, and only as many as needed
to get back under the limit. Trimming happens only at open (program start): an
in-flight rewrite would break a running `tail -f`, and no single run of any of
these programs writes 100 MB.

Usage:

    from boardlog import BoardLog
    log = BoardLog('run_mission')       # ~/logs/run_mission.log
    log('plain message')                # INFO; drop-in for log=print callers
    log.warn('...'); log.error('...')

BoardLog instances are callable so every existing `log=print` seam in this
codebase (Mission, detectors, MavIO) accepts one unchanged.
"""

import os
import sys
import time
from datetime import datetime, timezone, timedelta

LOG_DIR = os.path.expanduser('~/logs')
MAX_BYTES = 100 * 1024 * 1024
# IST is a fixed offset with no DST, so a static timezone is exact and does
# not depend on the tz database being installed on the board.
IST = timezone(timedelta(hours=5, minutes=30), 'IST')


def _uptime_s():
    try:
        with open('/proc/uptime', encoding='utf-8') as f:
            return float(f.read().split()[0])
    except (OSError, ValueError):
        return time.monotonic()   # non-Linux fallback (laptop unit tests)


def _trim_oldest(path, limit=MAX_BYTES):
    """Drop the oldest lines until the file is back under limit.

    Keeps the newest `limit` bytes rounded forward to the next line start, so
    exactly as much as necessary is lost and never a partial line. Atomic
    replace, so a crash mid-trim leaves the old intact file, not a stub.
    """
    try:
        size = os.path.getsize(path)
    except OSError:
        return
    if size <= limit:
        return
    # Trimming replaces the inode, so a second instance of the same program
    # appending to the old handle would lose every line it wrote afterwards.
    # The lock makes concurrent trims exclusive; a holder that is mid-append
    # simply trims later.
    lock = path + '.trimlock'
    try:
        lock_fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError:
        return                            # another process is trimming
    except OSError:
        return
    os.close(lock_fd)
    tmp = path + '.trim'
    try:
        with open(path, 'rb') as src:
            src.seek(size - limit)
            src.readline()                # discard the cut-in-half line
            with open(tmp, 'wb') as dst:
                dst.write(b'[boardlog] older lines trimmed here: file '
                          b'exceeded 100 MB at open\n')
                while True:
                    chunk = src.read(1 << 20)
                    if not chunk:
                        break
                    dst.write(chunk)
        os.replace(tmp, path)
    finally:
        for junk in (lock, tmp):
            try:
                os.remove(junk)
            except OSError:
                pass


class BoardLog:
    def __init__(self, name, log_dir=LOG_DIR, mirror=True, capture=True):
        """mirror: also print() every line, so an interactive run reads the
        same as before.

        capture: when a std stream is not a terminal (the program was spawned
        by the dashboard, or detached under nohup), point its fd into the log
        file. For stderr that catches C-level warnings (OpenCV, onnxruntime)
        and crash tracebacks, which no in-Python logger ever sees. For stdout
        it catches bare print()s from modules that predate boardlog; mirroring
        is disabled in that case, or every boardlog line would land in the
        file twice (once written, once via the mirrored print). An interactive
        terminal keeps both streams on screen."""
        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, f'{name}.log')
        _trim_oldest(self.path)
        self._f = open(self.path, 'a', buffering=1, encoding='utf-8')   # line buffered
        self.mirror = mirror
        # Kept so close() can restore them; without this a post-close print()
        # writes to a closed file and raises ValueError.
        self._saved_stdout = sys.stdout
        self._saved_stderr = sys.stderr
        if capture:
            try:
                if not os.isatty(2):
                    os.dup2(self._f.fileno(), 2)
                    sys.stderr = self._f
            except OSError:
                pass
            try:
                if not os.isatty(1):
                    os.dup2(self._f.fileno(), 1)
                    sys.stdout = self._f
                    self.mirror = False
            except OSError:
                pass

    def _write(self, level, msg):
        wall = datetime.now(IST).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        line = f'{wall} IST +{_uptime_s():.1f}s {level:5s} {msg}'
        try:
            self._f.write(line + '\n')
        except OSError:
            pass                      # a full disk must never crash a mission
        if self.mirror:
            print(line, flush=True)

    def __call__(self, msg):
        # Drop-in for log=print seams; treat bare calls as INFO.
        self._write('INFO', str(msg))

    def info(self, msg):
        self._write('INFO', msg)

    def warn(self, msg):
        self._write('WARN', msg)

    def error(self, msg):
        self._write('ERROR', msg)

    def close(self):
        """Restore the real stdout/stderr, then close the file.

        Order matters: leaving the capture in place after closing the file
        makes every later print() raise ValueError, which _write does not
        catch (it catches OSError). Restoring first means a post-close print
        goes to the console instead of exploding.
        """
        for name, saved in (('stdout', self._saved_stdout),
                            ('stderr', self._saved_stderr)):
            if saved is not None and getattr(sys, name, None) is not saved:
                setattr(sys, name, saved)
        try:
            self._f.close()
        except (OSError, ValueError):
            pass
