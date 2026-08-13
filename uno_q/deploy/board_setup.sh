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
#   * copy the ONNX models - they are gitignored (74 MB of them) and must come
#                            from the laptop by scp. This script only CHECKS.
#   * touch the Pixhawk   - nothing here transmits on any serial port.

set -uo pipefail
VENV="$HOME/venv"
REPO="${MONSOONREADY_REPO:-$HOME/monsoonready}"
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
        echo "  If sudo is unavailable, try:  python3 -m pip install --user virtualenv"
        exit 1; }
    echo "  created $VENV"
fi
good "venv at $VENV"

say "python packages"
# pymavlink+pyserial = the Pixhawk link. numpy+opencv+onnxruntime = detection.
# opencv-python-headless, not opencv-python: the board has no display and the
# GUI build drags in X libraries that are not there.
"$VENV/bin/pip" install --quiet --upgrade pip
"$VENV/bin/pip" install --quiet \
    pymavlink pyserial numpy opencv-python-headless onnxruntime \
    || { echo "  pip install failed. Check the board has internet."; exit 1; }
for mod in pymavlink serial numpy cv2 onnxruntime; do
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
ls /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/      /' \
    || bad "no serial devices at all"

say "repo and models"
[ -d "$REPO/.git" ] && good "repo at $REPO" || bad "no repo at $REPO (operator is syncing this separately)"
found_model=0
for m in "$HOME/best.onnx" "$REPO/best.onnx"; do
    [ -f "$m" ] && { good "model $m ($(du -h "$m" | cut -f1))"; found_model=1; }
done
[ "$found_model" -eq 0 ] && bad "NO best.onnx ANYWHERE. It is gitignored, so it cannot arrive by git pull.
        From the LAPTOP:  scp training/exports/best.onnx arduino@<board>:~/"

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
