# Board deploy

POLICY (user, 2026-08-16): nothing runs automatically on the board. No systemd
services, no timers, no cron jobs. Every program is started by hand, and the
user updates the checkout by running `git pull` themselves. The old auto-sync
units (`monsoonready-sync.service`, `monsoonready-sync.user.service`,
`board_sync.sh`) were deleted under this policy after the 2026-08-15 farm run.

What remains:

- `board_setup.sh` — one-time, run by hand after a fresh flash. Installs the
  venv and Python dependencies and lists the serial devices it can see. It
  does not install or enable anything that runs on its own.
