#!/bin/bash
# Keep the UNO Q's checkout in step with the repo, polling every few seconds.
#
# WHY THIS EXISTS: until now the board carried loose scp'd copies of a few
# scripts, so "what is on the board" and "what is in git" were different things
# and nobody could say how different. Every debugging session started by
# wondering whether the board had the current file. This makes the board a
# mirror of main and nothing else.
#
# WHAT IT WILL NOT DO: clobber work. It pulls FAST-FORWARD ONLY. If the board's
# checkout has diverged (someone edited or committed on the board) it stops
# syncing, says so loudly in the log, and leaves the files alone rather than
# throwing away a change nobody has a copy of. Fix it by hand on the board and
# the loop resumes on its own.
#
# It also never RUNS anything it pulls. Syncing code and executing code are
# different jobs; a puller that restarts services turns a bad commit into a
# flying aircraft's problem.
#
# Logs go to journald:  journalctl -u monsoonready-sync -f

set -uo pipefail

REPO="${MONSOONREADY_REPO:-$HOME/monsoonready}"
INTERVAL="${MONSOONREADY_SYNC_INTERVAL:-10}"
SELF="$(readlink -f "$0")"

log() { printf '%s %s\n' "$(date '+%H:%M:%S')" "$*"; }

cd "$REPO" 2>/dev/null || { log "FATAL: no repo at $REPO"; exit 1; }
git rev-parse --git-dir >/dev/null 2>&1 || { log "FATAL: $REPO is not a git repo"; exit 1; }

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
log "watching $REPO on '$BRANCH', every ${INTERVAL}s, fast-forward only"
# Remember our own checksum. If a pull updates THIS script, the running copy is
# stale: exit cleanly and let systemd's Restart=always start the new version.
# Without this the board would silently keep running whatever shipped first.
SELF_SUM="$(md5sum "$SELF" | cut -d' ' -f1)"

offline_since=0        # so a flaky link logs once, not every 10 seconds
diverged=0

while true; do
    if ! git fetch --quiet origin "$BRANCH" 2>/dev/null; then
        if [ "$offline_since" -eq 0 ]; then
            offline_since=$SECONDS
            log "fetch failed (no network, or no credentials). Retrying quietly."
        fi
        sleep "$INTERVAL"
        continue
    fi
    if [ "$offline_since" -ne 0 ]; then
        log "fetch working again after $((SECONDS - offline_since))s offline"
        offline_since=0
    fi

    local_sha="$(git rev-parse HEAD)"
    remote_sha="$(git rev-parse "origin/$BRANCH")"

    if [ "$local_sha" = "$remote_sha" ]; then
        diverged=0
        sleep "$INTERVAL"
        continue
    fi

    # Behind is fine and fast-forwards. Diverged means the board holds commits
    # the remote does not, and pulling would either merge or discard them.
    ahead="$(git rev-list --count "origin/$BRANCH..HEAD")"
    if [ "$ahead" -gt 0 ]; then
        if [ "$diverged" -eq 0 ]; then
            diverged=1
            log "STOPPED: this checkout has $ahead commit(s) the remote does not."
            log "  Nothing will be pulled until that is resolved BY HAND on the"
            log "  board, because a fast-forward is impossible and anything else"
            log "  risks losing work that exists nowhere else."
        fi
        sleep "$INTERVAL"
        continue
    fi

    behind="$(git rev-list --count "HEAD..origin/$BRANCH")"
    if git merge --ff-only "origin/$BRANCH" --quiet 2>/dev/null; then
        log "pulled $behind commit(s): ${local_sha:0:7} -> $(git rev-parse --short HEAD)"
        git log --oneline "${local_sha}..HEAD" | sed 's/^/    /'
        if [ "$(md5sum "$SELF" | cut -d' ' -f1)" != "$SELF_SUM" ]; then
            log "this sync script itself changed; exiting so systemd restarts the new one"
            exit 0
        fi
    else
        # Almost always a locally-modified tracked file blocking the merge.
        log "FF merge refused. Local modifications are in the way:"
        git status --porcelain | sed 's/^/    /'
        log "  Resolve on the board (git checkout -- <file>, or commit it)."
    fi
    sleep "$INTERVAL"
done
