#!/bin/bash
# The one command to run over SSH after a board (re)boot: bring the dashboard
# up, detached, and print the URLs. Everything else (self-test, photos,
# starting the mission) then happens in the WebUI.
#
#     bash ~/monsoonready-drone/uno_q/start_dashboard.sh
#     bash ~/monsoonready-drone/uno_q/start_dashboard.sh --no-drop   # rehearsal
#
# Extra arguments pass straight through to dashboard.py.
#
# What it does, exactly:
#   * stops an already-running dashboard first (SIGTERM), so re-running after
#     a git pull serves the NEW code instead of failing on the busy port;
#   * NEVER touches run_mission: restarting the UI must not stop a flight;
#   * launches detached (setsid + nohup) so the dashboard survives this SSH
#     session dying at the field;
#   * waypoints default to ~/wp_field.txt, the same file make_waypoints.py
#     writes when run from the home directory.
#
# The dashboard writes its own ~/logs/dashboard.log (boardlog), so nothing is
# redirected here; /dev/null only eats what boardlog already captured.
set -eu

REPO="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
PY="$HOME/venv/bin/python"
DASH="$REPO/uno_q/basestation/dashboard.py"

LOG="${DASH_LOG:-$HOME/logs/dashboard.log}"
mkdir -p "$(dirname "$LOG")"
# Every message is persisted, not just printed: a failed restart over ssh at
# the field otherwise leaves no record (SCOPE RULES 1).
note() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] start_dashboard: $*" | tee -a "$LOG"; }

[ -x "$PY" ] || { note "FATAL: no venv python at $PY (run board_setup.sh)"; exit 1; }
[ -f "$DASH" ] || { note "FATAL: $DASH missing (bad checkout?)"; exit 1; }

# Anchored to the resolved path, not a bare "dashboard.py" substring, so an
# unrelated python process cannot be signalled.
PAT="python.*$DASH"

if pgrep -f "$PAT" >/dev/null 2>&1; then
    note "stopping the running dashboard: $(pgrep -f "$PAT" | tr '\n' ' ')"
    pkill -f "$PAT" || true
    # Wait for the port to actually free. A flat 1 s let the old process keep
    # 8080, the new one died on bind, and this script still said UP.
    for _ in $(seq 1 20); do
        pgrep -f "$PAT" >/dev/null 2>&1 || break
        sleep 0.5
    done
    if pgrep -f "$PAT" >/dev/null 2>&1; then
        note "SIGKILL: it did not exit in 10 s"
        pkill -9 -f "$PAT" || true
        sleep 1
    fi
fi

setsid nohup "$PY" "$DASH" \
    --enable-control --waypoints "$HOME/wp_field.txt" "$@" \
    >/dev/null 2>&1 &
NEW_PID=$!

# Check THE PID WE LAUNCHED, not "some matching process exists". The old
# check passed while our own process was already dead on a busy port.
sleep 2
if kill -0 "$NEW_PID" 2>/dev/null; then
    note "dashboard UP (pid $NEW_PID, flight control enabled)"
    echo "Open ONE of:"
    for ip in $(hostname -I 2>/dev/null); do
        echo "    http://$ip:8080"
    done
    echo "log: $LOG"
    echo "Next: press 'Test everything' on the page."
else
    note "dashboard FAILED to start (pid $NEW_PID is already gone)"
    echo "Last log lines:"
    tail -n 8 "$LOG" 2>/dev/null || echo "  (no log)"
    exit 1
fi
