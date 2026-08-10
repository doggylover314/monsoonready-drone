#!/usr/bin/env python3
"""Everything to do with Pixhawk parameters, and nothing else.

    ./python tools/parameters.py get  BATT_AMP_PERVLT
    ./python tools/parameters.py set  BATT_AMP_PERVLT 24
    ./python tools/parameters.py push                  # the project's config
    ./python tools/parameters.py pull                  # dump the whole board
    ./python tools/parameters.py merge [qgc_dump]      # offline file build

Replaces push_params.py, pull_params.py, make_complete_params.py and the
getparam/setparam subcommands that used to sit in bench.py. Parameter logic
was spread across four files with three separate copies of "write a value and
check the echo"; only one of them got the checking right.

RUN WITH QGC CLOSED. It owns the serial port while connected.

USB, NOT THE RADIO, for push: it is many small round trips and every retry
costs radio time. get/set are fine over the SiK link.

THE ECHO IS THE PROOF. ArduPilot silently clamps out-of-range values and
silently ignores writes to parameters gated behind a parent enable, so a
write with no readback is not a write. Values compare at float32, the board's
native storage, because 0.14 does not survive a float64 round trip intact.
"""

import argparse
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pymavlink import mavutil                      # noqa: E402
from mavlink_link import connect                   # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DUMPS = ROOT / "param_dumps"
SETUP = DUMPS / "pixhawk_full_setup.param"
FULL = DUMPS / "pixhawk_every_param.param"
MERGED = DUMPS / "pixhawk_complete.params"

# Parameters this project has confirmed need a REBOOT before the new value
# does anything. Not exhaustive: ArduPilot marks many others the same way and
# there is no way to ask the board which, so treat a silent no-op after a
# successful write as "probably needs a reboot" and re-read after one.
NEEDS_REBOOT = {
    'BATT_MONITOR': 'selects the battery monitor DRIVER, which is built once '
                    'at startup. The value you just wrote is inert until the '
                    'board restarts, so the board is still running whatever '
                    'was set before',
    'BATT_CURR_PIN': 'analog pin assignment is read once at startup',
    'BATT_VOLT_PIN': 'analog pin assignment is read once at startup',
    'PRX1_TYPE': 'proximity driver is instantiated at startup',
    'RNGFND1_TYPE': 'rangefinder driver is instantiated at startup',
    'RNGFND2_TYPE': 'rangefinder driver is instantiated at startup',
    'SERIAL1_PROTOCOL': 'serial port protocol is bound at startup',
    'SERIAL2_PROTOCOL': 'serial port protocol is bound at startup',
    'SERIAL4_PROTOCOL': 'serial port protocol is bound at startup',
    'SERIAL5_PROTOCOL': 'serial port protocol is bound at startup',
}


def f32(x):
    """Board stores float32; compare at that precision, not text or float64."""
    return struct.unpack("f", struct.pack("f", float(x)))[0]


def fmt(v):
    return str(int(v)) if float(v).is_integer() else repr(round(float(v), 6))


def load_param_file(path=SETUP):
    """NAME,VALUE lines, ignoring comments. Returns {name: float}."""
    out = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if "," in line:
            name, val = line.split(",", 1)
            out[name.strip()] = float(val.strip())
    return out


def await_param(m, name, timeout=10.0):
    """The PARAM_VALUE for THIS name, discarding others.

    ArduPilot broadcasts PARAM_VALUE whenever any parameter changes, and a
    second GCS on the same link produces more, so taking "the next one" and
    labelling it with the name you asked for can report a completely
    different parameter's value.
    """
    end = time.time() + timeout
    while time.time() < end:
        p = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if p is None:
            continue
        got = p.param_id if isinstance(p.param_id, str) else p.param_id.decode()
        if got.strip('\x00') == name:
            return p
    return None


def write_param(m, name, value, attempts=3):
    """Write one parameter and confirm the board echoed THAT name back.

    Returns (ok, echoed_value_or_None). ok is False both for a refused write
    and for a clamped one, because a value the board changed on the way in is
    not the value you asked for and must never be reported as success.
    """
    for _ in range(attempts):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        p = await_param(m, name, timeout=2.0)
        if p is not None:
            return f32(p.param_value) == f32(value), p.param_value
    return False, None


def reboot_note(names):
    hits = [n for n in names if n in NEEDS_REBOOT]
    if hits:
        print("\nREBOOT REQUIRED before these take effect:")
        for n in hits:
            print(f"  {n}  ({NEEDS_REBOOT[n]})")


# --- subcommands ------------------------------------------------------------

def cmd_get(args):
    m, _, _ = connect(args.conn, args.baud)
    # ASK MORE THAN ONCE. A single dropped request is indistinguishable from a
    # misspelt parameter, and the old single-shot version accused the user of
    # a typo when a real read had simply been lost (seen 2026-08-10: the same
    # BATT_AMP_PERVLT read failed then succeeded, unchanged). Requests are
    # cheap; a wrong accusation costs a debugging detour.
    for attempt in range(3):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      args.name.encode(), -1)
        p = await_param(m, args.name, timeout=4.0)
        if p is not None:
            print(f"  {args.name} = {fmt(p.param_value)}")
            return
        if attempt < 2:
            print(f"  no reply, retrying ({attempt + 2}/3) ...")
    sys.exit(f"  {args.name} = NO REPLY after 3 requests. Either the name is "
             f"not on this firmware (a wrong name is answered with silence, "
             f"not an error) or the link is dropping packets.")


def cmd_set(args):
    m, _, _ = connect(args.conn, args.baud)
    ok, echoed = write_param(m, args.name, float(args.value))
    if echoed is None:
        print(f"  {args.name}: NO ECHO. Over a radio link a lost packet looks "
              f"exactly like a refused write, so re-read it with `get` before "
              f"believing either.")
        sys.exit(1)
    print(f"  {args.name} = {fmt(echoed)}")
    if not ok:
        print(f"  REFUSED OR CLAMPED: asked {args.value}, board stored "
              f"{fmt(echoed)}. The write did NOT take effect as typed.")
        sys.exit(1)
    reboot_note([args.name])


def cmd_push(args):
    src = Path(args.file) if args.file else SETUP
    overrides = load_param_file(src)
    if not overrides:
        sys.exit(f"{src} yielded no NAME,VALUE lines. Refusing to report "
                 f"success on an empty push (a QGC-saved, tab-separated file "
                 f"parses to nothing here).")
    print(f"pushing {len(overrides)} parameters from {src.name}")
    m, _, _ = connect(args.conn, args.baud)

    ok, failed, missing = [], [], []
    for name, value in sorted(overrides.items()):
        good, echoed = write_param(m, name, value)
        if good:
            ok.append(name)
            print(f"  OK   {name} = {value:g}")
        elif echoed is None:
            missing.append(name)
            print(f"  ???  {name}  (no echo; param may not exist yet)")
        else:
            failed.append(name)
            print(f"  FAIL {name}  wrote {value:g}, board echoed {echoed:g}")

    print(f"\n{len(ok)} ok, {len(failed)} refused, {len(missing)} no-echo")
    if failed:
        print("REFUSED (board rejected the value):\n  " + "\n  ".join(failed))
    if missing:
        print("NO-ECHO (likely gated until a parent enable + reboot; "
              "reboot and rerun):\n  " + "\n  ".join(missing))
    reboot_note(overrides)
    # The verdict has to reach the exit code: this tool is chained in the
    # bench sequence, and a refused safety parameter must stop the chain.
    sys.exit(1 if (failed or missing) else 0)


def cmd_pull(args):
    m, _, _ = connect(args.conn, args.baud)
    m.mav.param_request_list_send(m.target_system, m.target_component)

    params, total, last_rx = {}, None, time.time()
    while True:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
        if msg is None:
            if time.time() - last_rx > 10:
                break
            continue
        last_rx = time.time()
        name = (msg.param_id if isinstance(msg.param_id, str)
                else msg.param_id.decode())
        params[name.strip("\x00")] = msg.param_value
        total = msg.param_count
        if total and len(params) >= total:
            break
        if len(params) % 100 == 0:
            print(f"  {len(params)}/{total or '?'}")

    if total and len(params) < total:
        print(f"got {len(params)}/{total}, retrying stragglers ...")
        m.mav.param_request_list_send(m.target_system, m.target_component)
        deadline = time.time() + 30
        while len(params) < total and time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
            if msg is None:
                continue
            name = (msg.param_id if isinstance(msg.param_id, str)
                    else msg.param_id.decode())
            params[name.strip("\x00")] = msg.param_value
    print(f"fetched {len(params)} parameters (board reports {total})")
    if not params:
        sys.exit("no parameters received; is QGC closed and the board on USB?")
    if total and len(params) < total:
        print(f"WARNING: {total - len(params)} parameters never arrived. This "
              f"dump is INCOMPLETE; do not treat it as a backup.")

    # TIMESTAMPED, so a pull can never destroy the previous backup. The old
    # pull_params.py wrote one fixed filename, so a bad pull (board half
    # awake, link dropping) silently overwrote the last good snapshot with a
    # short one. param_dumps/* is gitignored, so keeping every pull is free.
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    raw = DUMPS / f"board_dump_{stamp}.param"
    raw.write_text(
        f"# Raw full dump, do not edit. Board state at {stamp}.\n"
        f"# {len(params)} parameters"
        + (f" of {total} reported" if total else "") + "\n"
        + "\n".join(f"{k},{fmt(v)}" for k, v in sorted(params.items())) + "\n")

    deltas = {k: fmt(v) for k, v in load_param_file().items()}
    unknown = [k for k in deltas if k not in params]
    lines = ["# EVERY parameter on this firmware, explicitly defined.",
             "# Base = board dump; lines marked  # SETUP  carry the project's",
             "# chosen values from pixhawk_full_setup.param.",
             f"# Regenerate after any calibration: tools/parameters.py pull",
             f"# Generated {stamp} from {raw.name}"]
    for k, v in sorted(params.items()):
        if k in deltas:
            lines.append(f"{k},{deltas[k]}  # SETUP (board had {fmt(v)})")
        else:
            lines.append(f"{k},{fmt(v)}")
    FULL.write_text("\n".join(lines) + "\n")

    print(f"wrote {raw.name} and {FULL.name}")
    if unknown:
        print("WARNING: these setup params do NOT exist on this firmware, "
              "check names:\n  " + "\n  ".join(unknown))


def cmd_merge(args):
    """Offline: apply the project's overrides onto a QGC dump, in QGC format.

    No board involved. Output loads via QGC Tools > Load from file.
    """
    if args.file:
        dump = Path(args.file)
    else:
        cands = sorted(DUMPS.glob("param_dump*"), key=lambda p: p.stat().st_mtime)
        if not cands:
            sys.exit("no param_dump* file in param_dumps/; name one explicitly")
        dump = cands[-1]
    print(f"dump: {dump.name}")

    overrides = {k: fmt(v) for k, v in load_param_file().items()}
    used, header, rows = set(), [], []
    for line in dump.read_text().splitlines():
        if line.startswith("#") or not line.strip():
            header.append(line)
            continue
        parts = line.split("\t")
        if len(parts) != 5:
            print(f"skipping unparseable line: {line!r}")
            continue
        vid, cid, name, value, ptype = parts
        if name in overrides:
            new = overrides[name]
            if f32(new) != f32(value):
                print(f"  override {name}: {value.rstrip('0').rstrip('.')} "
                      f"-> {new}")
            # TRUNCATION BUG, fixed 2026-08-10 (found in the 2026-08-08 audit):
            # this used to write str(int(float(new))) for every non-float type,
            # so an override of 0.14 on a param QGC typed as an int was written
            # to the file as 0. Silent, and exactly the kind of thing that gets
            # loaded and flown. Only integer-ify a value that really is one.
            if ptype == "9" or not float(new).is_integer():
                if ptype != "9":
                    print(f"    NOTE {name}={new} is fractional but the dump "
                          f"types it as {ptype}; writing it in full rather "
                          f"than truncating. Verify the board accepts it.")
                value = f"{float(new):.18f}"
            else:
                value = str(int(float(new)))
            used.add(name)
        rows.append((vid, cid, name, value, ptype))

    missing = sorted(set(overrides) - used)
    MERGED.write_text("\n".join(header) + "\n" +
                      "\n".join("\t".join(r) for r in rows) + "\n")
    print(f"wrote {MERGED.name}: {len(rows)} parameters, "
          f"{len(used)} overridden, {len(missing)} overrides not on board")
    if missing:
        print("NOT FOUND on board (rename or gated sub-param):")
        for name in missing:
            print(f"  {name}")


def main():
    ap = argparse.ArgumentParser(
        description="Pixhawk parameters: read, write, push, pull, merge.")
    ap.add_argument('cmd', choices=['get', 'set', 'push', 'pull', 'merge'])
    ap.add_argument('name', nargs='?', help='parameter name, or a file path '
                                            'for push/merge')
    ap.add_argument('value', nargs='?', help='value, for set')
    ap.add_argument('--conn', default=None,
                    help='serial device; omit to auto-pick when exactly one '
                         'is present')
    ap.add_argument('--baud', type=int, default=None,
                    help='omit to follow the port type: 57600 for a SiK '
                         'radio, 115200 for USB')
    args = ap.parse_args()

    if args.cmd in ('get', 'set') and not args.name:
        sys.exit(f"{args.cmd} needs a parameter name")
    if args.cmd == 'set' and args.value is None:
        sys.exit("set needs a value")
    args.file = args.name if args.cmd in ('push', 'merge') else None

    {'get': cmd_get, 'set': cmd_set, 'push': cmd_push,
     'pull': cmd_pull, 'merge': cmd_merge}[args.cmd](args)


if __name__ == '__main__':
    main()
