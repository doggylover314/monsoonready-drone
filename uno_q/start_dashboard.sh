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
set -u

REPO="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
PY="$HOME/venv/bin/python"
DASH="$REPO/uno_q/basestation/dashboard.py"

[ -x "$PY" ] || { echo "FATAL: no venv python at $PY (run board_setup.sh)"; exit 1; }
[ -f "$DASH" ] || { echo "FATAL: $DASH missing (bad checkout?)"; exit 1; }

if pgrep -f "python.*dashboard\.py" >/dev/null 2>&1; then
    echo "stopping the running dashboard ..."
    pkill -f "python.*dashboard\.py"
    sleep 1
fi

setsid nohup "$PY" "$DASH" \
    --enable-control --waypoints "$HOME/wp_field.txt" "$@" \
    >/dev/null 2>&1 &

sleep 2
if pgrep -f "python.*dashboard\.py" >/dev/null 2>&1; then
    echo "dashboard UP (flight control enabled). Open ONE of:"
    for ip in $(hostname -I 2>/dev/null); do
        echo "    http://$ip:8080"
    done
    echo "log: ~/logs/dashboard.log"
    echo "Next: press 'Test everything' on the page."
else
    echo "dashboard FAILED to start. Last log lines:"
    tail -n 8 "$HOME/logs/dashboard.log" 2>/dev/null || echo "  (no log)"
    exit 1
fi
