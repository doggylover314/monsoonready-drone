#!/bin/bash
# One-shot dashboard / site-map diagnostics. Run ON THE UNO Q, paste the
# whole output back:
#
#     bash ~/monsoonready-drone/uno_q/diag_dashboard.sh
#
# Read-only, no sudo. The only writes: an append to ~/logs/diag_dashboard.log
# and two throwaway files in /tmp. One tile is fetched through the
# dashboard's own proxy (which the dashboard caches anyway) and one straight
# from Esri, so the output separates "server serves the wrong page" /
# "proxy broken" / "board has no internet" / "all fine, problem is in the
# viewing browser". Overridable for off-board testing: DIAG_REPO, DIAG_DATA,
# DIAG_PORT, DIAG_LOG.
set -u
REPO="${DIAG_REPO:-$HOME/monsoonready-drone}"
DATA="${DIAG_DATA:-$HOME/monsoonready_data}"
PORT="${DIAG_PORT:-8080}"
DLOG="${DIAG_LOG:-$HOME/logs/diag_dashboard.log}"
mkdir -p "$(dirname "$DLOG")"

main() {
  echo "===== dashboard diagnostics $(date '+%Y-%m-%d %H:%M:%S') ====="

  echo "== repo =="
  git -C "$REPO" log --oneline -2 2>&1 || echo "no repo at $REPO"

  echo "== dashboard process =="
  if pgrep -af "python.*$REPO/uno_q/basestation/dashboard.py"; then
    for p in $(pgrep -f "python.*$REPO/uno_q/basestation/dashboard.py"); do
      echo "pid $p started $(ps -o lstart= -p "$p" 2>/dev/null), up $(ps -o etime= -p "$p" 2>/dev/null | tr -d ' ')"
    done
  else
    echo "NOT RUNNING"
  fi

  echo "== page the RUNNING server serves =="
  if curl -sf -m 5 "http://localhost:$PORT/" -o /tmp/diag_index.html; then
    grep -o "UI_VER = '[^']*'" /tmp/diag_index.html \
      || echo "page fetched but has NO UI_VER: server is serving a pre-2026-08-16b page"
  else
    echo "FAILED to fetch / from localhost:$PORT (dashboard down?)"
  fi

  echo "== tile through the dashboard proxy (board -> Esri -> served) =="
  curl -s -m 20 -o /tmp/diag_tile.jpg \
    -w "HTTP %{http_code}, %{size_download} bytes\n" \
    "http://localhost:$PORT/api/tile/3/5/3" || echo "curl to the proxy failed"
  printf 'first bytes:'
  head -c 3 /tmp/diag_tile.jpg 2>/dev/null | od -An -tx1 | tr -d '\n'
  echo "  (a JPEG starts ff d8 ff)"

  echo "== Esri DIRECT from the board (internet check) =="
  curl -s -m 20 -o /dev/null -A MonsoonReady-diag \
    -w "HTTP %{http_code}, %{size_download} bytes\n" \
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/3/3/5" \
    || echo "no route to Esri from the board"

  echo "== tile cache on disk =="
  if [ -d "$DATA/tiles" ]; then
    echo "$(find "$DATA/tiles" -name '*.jpg' 2>/dev/null | wc -l) cached tiles under $DATA/tiles"
  else
    echo "no tile cache dir yet ($DATA/tiles): the proxy has never stored a tile"
  fi

  echo "== tile/satellite lines in dashboard.log =="
  grep -in "tile\|satellite" "$HOME/logs/dashboard.log" 2>/dev/null | tail -15 \
    || echo "(none)"

  echo "== dashboard.log tail =="
  tail -20 "$HOME/logs/dashboard.log" 2>/dev/null || echo "(no dashboard.log)"

  echo "===== end ====="
}
main 2>&1 | tee -a "$DLOG"
