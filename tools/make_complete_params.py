#!/usr/bin/env python3
"""Build the loadable every-parameter file in QGC's own format.

Input : a QGC parameter dump (Tools > Save to file) and the project's
        pixhawk_full_setup.param (NAME,VALUE overrides).
Output: pixhawk_complete.params (QGC tab format) = dump with overrides
        applied. Loadable via QGC Tools > Load from file.

Also prints every override whose name does not exist in the dump: those are
either wrongly named (fix the setup file) or gated sub-params that only
appear after their parent enable is written and the board rebooted. Workflow:
load stage 1, reboot, re-dump, run this again for the final complete file.

Usage: python3 make_complete_params.py [dump_file]   (default: newest dump)
"""

import struct
import sys
from pathlib import Path


def f32(x) -> float:
    """Board stores float32; compare at that precision, not text/float64."""
    return struct.unpack("f", struct.pack("f", float(x)))[0]

ROOT = Path(__file__).resolve().parent.parent
SETUP = ROOT / "pixhawk_full_setup.param"
OUT = ROOT / "pixhawk_complete.params"


def load_overrides() -> dict[str, str]:
    out = {}
    for line in SETUP.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if "," in line:
            name, val = line.split(",", 1)
            out[name.strip()] = val.strip()
    return out


def main() -> None:
    if len(sys.argv) > 1:
        dump = Path(sys.argv[1])
    else:
        cands = sorted(ROOT.glob("param_dump*"), key=lambda p: p.stat().st_mtime)
        if not cands:
            sys.exit("no param_dump* file in project root")
        dump = cands[-1]
    print(f"dump: {dump.name}")

    overrides = load_overrides()
    used = set()
    header = []
    rows = []  # (vid, cid, name, value_str, type_str)
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
                print(f"  override {name}: {value.rstrip('0').rstrip('.')} -> {new}")
            # keep the dump's float formatting style for float types
            value = f"{float(new):.18f}" if ptype == "9" else str(int(float(new)))
            used.add(name)
        rows.append((vid, cid, name, value, ptype))

    missing = sorted(set(overrides) - used)
    OUT.write_text("\n".join(header) + "\n" +
                   "\n".join("\t".join(r) for r in rows) + "\n")
    print(f"wrote {OUT.name}: {len(rows)} parameters, "
          f"{len(used)} overridden, {len(missing)} overrides not on board")
    if missing:
        print("NOT FOUND on board (rename or gated sub-param, see docstring):")
        for m in missing:
            print(f"  {m}")


if __name__ == "__main__":
    main()
