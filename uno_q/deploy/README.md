# Board deploy: keeping the UNO Q in step with the repo

Three ways to run `board_sync.sh`. **Pick the first one that works** — they do
the same job and differ only in what privilege they need.

Before any of them, the repo must be cloned once by hand, because the clone is
where SSH host-key acceptance and any credential prompt happen. A service
cannot answer a prompt.

```
git clone git@github.com:doggylover314/monsoonready-drone.git ~/monsoonready
```

---

## 1. No sudo: user service (preferred)

Needs no root at all. This is the right default even when sudo works, because
nothing this service does requires privilege.

```
mkdir -p ~/.config/systemd/user
cp ~/monsoonready/uno_q/deploy/monsoonready-sync.user.service ~/.config/systemd/user/monsoonready-sync.service
systemctl --user daemon-reload
systemctl --user enable --now monsoonready-sync
journalctl --user -u monsoonready-sync -f
```

To survive a reboot without anyone logging in:

```
loginctl enable-linger arduino
```

If that asks for authentication you cannot give, the service still works for
the current session; use option 3 for reboot persistence.

## 2. No sudo, no systemd: user crontab

Needs nothing but a shell. Survives reboot. The `flock` is what stops a second
copy starting if cron fires again while one is already running.

```
( crontab -l 2>/dev/null; echo '@reboot /usr/bin/flock -n /tmp/mr-sync.lock ~/monsoonready/uno_q/deploy/board_sync.sh >> ~/board_sync.log 2>&1' ) | crontab -
```

Start it now without waiting for a reboot:

```
nohup /usr/bin/flock -n /tmp/mr-sync.lock ~/monsoonready/uno_q/deploy/board_sync.sh >> ~/board_sync.log 2>&1 &
tail -f ~/board_sync.log
```

Note the self-update path behaves differently here: the script exits 0 when it
pulls a new copy of itself, and cron will not restart it until the next reboot.
Re-run the `nohup` line after a sync script change.

## 3. With sudo: system service

Only worth it if you want it running before any user logs in and lingering is
unavailable.

```
sudo cp ~/monsoonready/uno_q/deploy/monsoonready-sync.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now monsoonready-sync
journalctl -u monsoonready-sync -f
```

---

## Stopping it

**Do this before going to the field.** On a phone hotspot it fetches every 10
seconds and spends mobile data to discover that nothing changed.

```
systemctl --user stop monsoonready-sync     # option 1
pkill -f board_sync.sh                      # option 2
sudo systemctl stop monsoonready-sync       # option 3
```

## Reading the log

| line | meaning |
|---|---|
| `watching ... every 10s` | started cleanly |
| `pulled N commit(s)` | worked, with the subjects listed below it |
| `fetch failed` | no network, or credentials the service cannot supply |
| `STOPPED: this checkout has N commit(s)...` | someone committed on the board. Nothing will sync until that is resolved by hand. This is deliberate: a fast-forward is impossible and anything else risks losing work that exists nowhere else |
| `FF merge refused` | a locally-modified tracked file is in the way; the log names it |
| `this sync script itself changed` | expected, and only under systemd does it come straight back |

## What it deliberately does not do

It never runs, restarts, or launches anything it pulls. Syncing code and
executing code are different jobs, and a puller that restarts services turns a
bad commit into a flying aircraft's problem.
