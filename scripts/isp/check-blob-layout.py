#!/usr/bin/env python3
"""
Check blob-layout.toml against real tuning files.

Confirms what only bytes can: gate words hold 0 or 1, band arrays are ascending finite floats,
declared band counts fit their payload rows, and the layout holds for all three vendor sensors.

    kernel/scripts/isp/check-blob-layout.py \\
        out/air-gather/vendor-root/usr/usrdata/tunning/*_tuning_preview_fpv.bin

Exit 1 on any failure. Needs blobs, so it runs by hand; tests/test_blob_layout.py runs in CI.
"""

import argparse
import sys
from pathlib import Path

from blob_layout import Blob, Layout

# Two axes: gain runs 1.0 to 2048, trigger scalar 0 to 550.
BAND_MAX = 2048.5


def check(blob: Blob, name: str, verbose: bool) -> list[str]:
    bad: list[str] = []

    layout = blob.layout
    for section in layout:
        if section.end > layout.size:
            bad.append(f"{section.name}: ends at {section.end:#x}, past the file")

    for gate in layout.gates():
        value = blob.u32(gate.name)
        if value not in (0, 1):
            bad.append(f"{gate.name}: gate reads {value:#x}, not 0 or 1")

    for section in layout:
        if section.kind != "bands":
            continue

        values = blob.array(section.name)
        used = [v for v in values if v != 0.0]
        if any(v != v or v < 0.0 or v > BAND_MAX for v in used):
            bad.append(f"{section.name}: band values outside [0, {BAND_MAX}]")

        ascending = [v for v in values if v != 0.0]
        if ascending != sorted(ascending):
            bad.append(f"{section.name}: band edges are not ascending")

    # A declared band count over the payload row count means selection walks into the next stage.
    for section in layout:
        if "count" not in section.fields:
            continue

        declared = blob.field(section.name, "count")
        stage_payloads = [s for s in layout.stage(section.stage) if s.kind == "payload"]
        for payload in stage_payloads:
            if declared > (payload.count or 0):
                bad.append(f"{section.stage}: header says {declared} bands, "
                           f"{payload.name} has {payload.count} rows")

    if verbose and not bad:
        gates_on = [g.name for g in layout.gates() if blob.gate(g.name)]
        print(f"  {len(layout)} sections checked, {len(gates_on)} gates set: "
              f"{', '.join(sorted(gates_on))}")

    return bad


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("blobs", nargs="+", type=Path, help="one or more sensor tuning files")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    layout = Layout.load()
    failures = 0

    for path in args.blobs:
        if not path.exists():
            sys.exit(f"{path}: not found")

        blob = Blob.open(path, layout)
        bad = check(blob, path.name, args.verbose)
        print(f"{path.name}: {'OK' if not bad else str(len(bad)) + ' problems'}")
        for line in bad:
            print(f"  {line}")

        failures += len(bad)

    # Compared per named field, not byte for byte: the values differ between sensors.
    if len(args.blobs) > 1:
        print(f"\nlayout holds for all {len(args.blobs)} sensors: "
              f"{'yes' if not failures else 'NO'}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
