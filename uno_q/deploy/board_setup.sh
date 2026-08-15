#!/bin/bash
# Bring a freshly-reflashed UNO Q back to a state that can fly the mission.
#
#   bash ~/monsoonready/uno_q/deploy/board_setup.sh
#
# Idempotent: safe to re-run. It installs nothing outside $HOME and needs no
# root, so a forgotten sudo password cannot block it.
#
# WHAT IT DOES NOT DO, deliberately:
#   * clone the repo      - the operator is handling sync separately
#   * copy the ONNX model - as of 2026-08-13 models/best.onnx IS TRACKED, so a
#                           git pull delivers it. No scp needed any more.
#   * touch the Pixhawk   - nothing here transmits on any serial port.

set -uo pipefail
VENV="$HOME/venv"
# Resolve the repo from THIS script's own location, so it is right
# wherever the clone lives and needs no environment variable.
REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
ok=0; warn=0

say()  { printf '\n=== %s ===\n' "$*"; }
good() { printf '  OK    %s\n' "$*"; ok=$((ok+1)); }
bad()  { printf '  MISS  %s\n' "$*"; warn=$((warn+1)); }

say "python and venv"
python3 --version || { echo "FATAL: no python3"; exit 1; }
if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv "$VENV" 2>/dev/null || {
        echo "  venv module missing. On a Debian-ish image that is:"
        echo "     sudo apt-get install -y python3-venv"
        exit 1; }
    echo "  created $VENV"
fi
# A venv can exist WITH NO PIP IN IT. Debian-family images strip ensurepip out
# of the stdlib, so `python3 -m venv` happily produces bin/python and no
# bin/pip. Checking only for bin/python called that venv healthy and then died
# on the first pip line (seen on the board 2026-08-13, Python 3.13.5).
if ! "$VENV/bin/python" -m pip --version >/dev/null 2>&1; then
    echo "  venv has no pip; bootstrapping"
    "$VENV/bin/python" -m ensurepip --upgrade 2>/dev/null \
      || curl -sS https://bootstrap.pypa.io/get-pip.py | "$VENV/bin/python" \
      || { echo "  FATAL: could not get pip into $VENV."
           echo "  Try:  sudo apt-get install -y python3-venv python3-pip"
           exit 1; }
fi
"$VENV/bin/python" -m pip --version >/dev/null 2>&1 \
  && good "venv at $VENV (pip $("$VENV/bin/python" -m pip --version | cut -d" " -f2))" \
  || { echo "  FATAL: still no pip"; exit 1; }

say "python packages"
# pymavlink+pyserial = the Pixhawk link. numpy+opencv+onnxruntime = detection.
# opencv-python-headless, not opencv-python: the board has no display and the
# GUI build drags in X libraries that are not there.
"$VENV/bin/python" -m pip install --quiet --upgrade pip
# flask = the base station dashboard, which the mission auto-launches and
# which is FILMED in the demo video; msgpack = the Linux half of the MAVLink
# byte-shovel (router_client.py). Both were missing here and had to be
# installed by hand after the 2026-08-13 reflash (review 2026-08-15).
"$VENV/bin/python" -m pip install --quiet \
    pymavlink pyserial numpy opencv-python-headless onnxruntime flask msgpack \
    || { echo "  pip install failed. Check the board has internet."; exit 1; }
for mod in pymavlink serial numpy cv2 onnxruntime flask msgpack; do
    if "$VENV/bin/python" -c "import $mod" 2>/dev/null; then
        v=$("$VENV/bin/python" -c "import $mod;print(getattr($mod,'__version__','?'))" 2>/dev/null)
        good "$mod $v"
    else
        bad "$mod FAILED TO IMPORT"
    fi
done

say "camera"
if command -v v4l2-ctl >/dev/null 2>&1; then
    if v4l2-ctl --list-devices 2>/dev/null | grep -qi "B525\|webcam"; then
        good "USB camera enumerated:"
        v4l2-ctl --list-devices 2>/dev/null | sed 's/^/      /' | head -8
    else
        bad "no USB camera in v4l2-ctl --list-devices (check the hub and the splitter)"
    fi
else
    bad "v4l2-ctl not installed (apt-get install v4l-utils) - cannot verify the camera"
fi

say "serial ports (the Pixhawk link lives on one of these, or on none)"
# NOT `ls a* b* | sed || bad`: with pipefail set, ls exits nonzero when ANY
# glob fails to match even though it printed the ones that did, so the board's
# normal state (ttyS0-3 present, no ttyUSB/ttyACM) printed the devices AND
# claimed there were none, inflating the failure count.
serial_found=$(ls -d /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)
if [ -n "$serial_found" ]; then
    printf '%s\n' "$serial_found" | sed 's/^/      /'
else
    bad "no serial devices at all"
fi

say "repo and models"
[ -d "$REPO/.git" ] && good "repo at $REPO" || bad "no repo at $REPO (operator is syncing this separately)"
# models/best.onnx is TRACKED as of 2026-08-13, so the repo copy is the
# canonical one and arrives with the sync. A loose ~/best.onnx from before the
# reflash may still exist and may be STALE; the repo copy wins.
found_model=0
for m in "$REPO/models/best.onnx" "$HOME/best.onnx"; do
    [ -f "$m" ] && { good "model $m ($(du -h "$m" | cut -f1))"; found_model=1; }
done
[ "$found_model" -eq 0 ] && bad "NO best.onnx. It is tracked now, so this means the
        repo has not synced yet. Check the sync, then re-run this script."

say "data directory"
mkdir -p "$HOME/monsoonready_data" && good "$HOME/monsoonready_data"

printf '\n=== SUMMARY: %d ok, %d missing ===\n' "$ok" "$warn"
if [ "$warn" -eq 0 ]; then
    echo "Board is ready. Next, and it is the one that matters:"
    echo "  $VENV/bin/python $REPO/uno_q/find_pixhawk_uart.py"
    echo "That decides whether the Pixhawk is reachable from Linux directly"
    echo "or only through an STM32 sketch."
else
    echo "Fix the MISS lines above before trusting anything downstream."
fi
