"""Wi-Fi switching for the board, through NetworkManager's nmcli.

Why (user, 2026-08-16): "Add a small settings window to the dashboard which
allows easy changing of wifi connections." The WISP router plan is dead (the
MR3020 kept failing to associate), so at the field the board joins the phone
hotspot directly, and that switch has to be doable from the dashboard rather
than from an SSH session that the switch itself is about to kill.

Everything here is verified nmcli 1.46 syntax:
  * TYPE reads "802-11-wireless" from `connection show` but "wifi" from
    `device status`: same radio, two spellings, so both are handled.
  * `-t` escapes literal ':' inside values as '\\:', so a naive split(':')
    corrupts any SSID or MAC containing a colon. _split() below splits on
    unescaped colons only.
  * `device wifi list` re-scans and blocks if its cache is stale, so the
    scan path is `rescan` then `list --rescan no`, each under a timeout.
  * Passwords never go in argv. `nmcli device wifi connect ... password X`
    would expose the secret in `ps` to every user on the box. The
    documented safe path, and the one used here, is: create the profile
    without a secret, then `connection up ... passwd-file FILE` with a
    0600 temp file deleted immediately afterwards.

Reads (listing, scanning) need no privileges. Bringing a connection up or
creating a profile is polkit-gated: on a headless SSH/service session the
call can come back "not authorized", so every failure returns nmcli's own
stderr to the caller instead of a generic message.
"""

import os
import re
import subprocess
import tempfile

NMCLI = 'nmcli'
SHORT_TIMEOUT = 12          # listing / status
SCAN_TIMEOUT = 25           # rescan + list
CONNECT_TIMEOUT = 45        # association + DHCP


class WifiError(RuntimeError):
    """nmcli refused or is not usable; the message is nmcli's own text."""


def _run(args, timeout=SHORT_TIMEOUT):
    """Run nmcli, return stdout. Raises WifiError with nmcli's stderr."""
    try:
        p = subprocess.run([NMCLI] + args, capture_output=True, text=True,
                           timeout=timeout)
    except FileNotFoundError:
        raise WifiError(
            'nmcli is not installed on this board, so Wi-Fi cannot be '
            'switched from here (this image may not use NetworkManager)')
    except subprocess.TimeoutExpired:
        raise WifiError(f'nmcli {args[0] if args else ""} timed out after '
                        f'{timeout}s')
    if p.returncode != 0:
        msg = (p.stderr or p.stdout or '').strip() or f'exit {p.returncode}'
        raise WifiError(msg)
    return p.stdout


def _split(line):
    """Split one -t line on unescaped colons, then unescape."""
    parts = re.split(r'(?<!\\):', line)
    return [p.replace('\\:', ':').replace('\\\\', '\\') for p in parts]


def wifi_device():
    """Name of the first Wi-Fi interface, or None if the board has none."""
    for line in _run(['-t', '-f', 'DEVICE,TYPE', 'device', 'status']).splitlines():
        f = _split(line)
        if len(f) >= 2 and f[1] == 'wifi':
            return f[0]
    return None


def status():
    """What the board is on right now, plus its saved Wi-Fi profiles.

    Shape: {'device','connection','state','ips':[..],'saved':[{name,active}]}
    Never raises for the "no wifi hardware" case: the panel has to be able
    to say so plainly.
    """
    out = {'device': None, 'connection': None, 'state': None, 'ips': [],
           'saved': [], 'error': None}
    try:
        dev = wifi_device()
        out['device'] = dev
        if dev:
            got = _run(['-t', '-g', 'GENERAL.CONNECTION,GENERAL.STATE,'
                        'IP4.ADDRESS', 'device', 'show', dev]).splitlines()
            if len(got) >= 1:
                out['connection'] = got[0].strip() or None
                if out['connection'] == '--':
                    out['connection'] = None
            if len(got) >= 2:
                out['state'] = got[1].strip()
            out['ips'] = [g.strip() for g in got[2:] if g.strip()]
        for line in _run(['-t', '-f', 'NAME,TYPE,DEVICE',
                          'connection', 'show']).splitlines():
            f = _split(line)
            if len(f) >= 2 and f[1] == '802-11-wireless':
                out['saved'].append({'name': f[0],
                                     'active': bool(len(f) > 2 and f[2]
                                                    and f[2] != '--')})
    except WifiError as exc:
        out['error'] = str(exc)
    return out


def scan():
    """Visible networks, strongest first: [{ssid, signal, security}]."""
    dev = wifi_device()
    if not dev:
        raise WifiError('this board has no Wi-Fi interface')
    try:
        _run(['device', 'wifi', 'rescan'], timeout=SCAN_TIMEOUT)
    except WifiError:
        # "Scanning not allowed immediately following previous scan" is
        # normal and harmless: the cached list below is still fresh.
        pass
    seen = {}
    for line in _run(['-t', '-f', 'SSID,SIGNAL,SECURITY', 'device', 'wifi',
                      'list', '--rescan', 'no'],
                     timeout=SCAN_TIMEOUT).splitlines():
        f = _split(line)
        if len(f) < 3 or not f[0]:
            continue                      # hidden network: no SSID to offer
        try:
            sig = int(f[1])
        except ValueError:
            sig = 0
        # one row per SSID, keeping the strongest sighting
        if f[0] not in seen or sig > seen[f[0]]['signal']:
            seen[f[0]] = {'ssid': f[0], 'signal': sig,
                          'security': f[2] or 'open'}
    return sorted(seen.values(), key=lambda r: -r['signal'])


def connect_saved(name):
    """Bring up an existing profile by name."""
    _run(['-w', '30', 'connection', 'up', 'id', name],
         timeout=CONNECT_TIMEOUT)
    return f'connected to {name}'


def connect_new(ssid, password, ifname=None):
    """Join a new network. The password never appears in argv.

    Profile is created first without the secret, then brought up with
    `passwd-file`, a 0600 file removed in the finally block.
    """
    dev = ifname or wifi_device()
    if not dev:
        raise WifiError('this board has no Wi-Fi interface')
    existing = {c['name'] for c in status()['saved']}
    if ssid not in existing:
        add = ['connection', 'add', 'type', 'wifi', 'con-name', ssid,
               'ifname', dev, 'ssid', ssid,
               'connection.autoconnect', 'yes']
        if password:
            add += ['wifi-sec.key-mgmt', 'wpa-psk']
        _run(add, timeout=SHORT_TIMEOUT)
    elif password:
        # A profile of this name already exists and may predate any security
        # section. Setting key-mgmt only on creation meant the PSK below was
        # handed to a profile not configured to use one, which fails with a
        # message that blames the password.
        _run(['connection', 'modify', ssid,
              'wifi-sec.key-mgmt', 'wpa-psk'], timeout=SHORT_TIMEOUT)
    if not password:
        return connect_saved(ssid)
    fd, path = tempfile.mkstemp(prefix='mr_wifi_', text=True)
    try:
        os.fchmod(fd, 0o600)
        if '\n' in password or '\r' in password:
            # The keyfile is line-based: a newline here ends the psk value
            # early and everything after it is parsed as further settings.
            raise WifiError('the password contains a line break, which this '
                            'file format cannot carry')
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(f'802-11-wireless-security.psk:{password}\n')
        _run(['-w', '30', 'connection', 'up', 'id', ssid,
              'passwd-file', path], timeout=CONNECT_TIMEOUT)
    finally:
        try:
            os.remove(path)
        except OSError:
            pass
    return f'connected to {ssid}'
