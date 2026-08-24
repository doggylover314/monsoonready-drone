#!/bin/bash
# Stop the dashboard cleanly. Counterpart of start_dashboard.sh:
#
#     bash ~/monsoonready-drone/uno_q/stop_dashboard.sh
#
# SIGTERM first (Flask exits, port 8080 is freed), SIGKILL only if it is
# still alive 10 s later. NEVER touches run_mission: stopping the UI must
# not stop a flight. Appends what it did to ~/logs/dashboard.log (SCOPE
# RULES 1; override with DASH_LOG=... when testing off-board). Same process
# pattern start_dashboard.sh uses for its own restart-stop.
set -eu
# Anchored to this checkout's dashboard.py, not a bare substring: the old
# pattern would SIGKILL any python process whose argv merely contained it.
REPO="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
PAT="python.*$REPO/uno_q/basestation/dashboard.py"
LOG="${DASH_LOG:-$HOME/logs/dashboard.log}"
mkdir -p "$(dirname "$LOG")"
note() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] stop_dashboard: $*" | tee -a "$LOG"; }

if ! pgrep -f "$PAT" >/dev/null 2>&1; then
    note "nothing to stop (no dashboard process)"
    exit 0
fi

note "SIGTERM -> PID(s) $(pgrep -f "$PAT" | tr '\n' ' ')"
pkill -f "$PAT"
for _ in $(seq 1 20); do
    sleep 0.5
    if ! pgrep -f "$PAT" >/dev/null 2>&1; then
        note "stopped"
        exit 0
    fi
done

note "still alive after 10 s, SIGKILL"
pkill -9 -f "$PAT"
sleep 1
if pgrep -f "$PAT" >/dev/null 2>&1; then
    note "FAILED: dashboard still running"
    exit 1
fi
note "stopped (needed SIGKILL)"
