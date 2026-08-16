#!/usr/bin/env python3
"""
Classify the shipped raw_3dnr state from the tuning blob and register captures.

The raw_3dnr stage owns ISP bank 0x4000. Its tuning gate lives at blob offset
0x0b500c, and the shared snapshot list captures the safe first 64 bank words on
both the vendor stack and the open stack. The FPV tuning gate and the streaming
vendor bank head are both zero, which classifies the stage as disabled for this
sensor mode.

    kernel/scripts/isp/check-raw-3dnr.py \\
        --tuning out/air-gather/vendor-root/usr/usrdata/tunning/nt99235_tuning_preview_fpv.bin \\
        --vendor-capture out/au-snapshot/registers.txt \\
        --open-capture out/au-snapshot/ours-registers-live.txt
"""

import argparse
import pathlib
import re
import struct
import sys

from blob_layout import Layout

_LAY = Layout.load()


RAW_3DNR_GATE = _LAY["raw_3dnr_gate"].offset
RAW_3DNR_SECTION = ("isp", 0x4000)
RAW_3DNR_WORDS = 64

SECTION_RE = re.compile(r"^--- ([a-z0-9_]+) \+0x([0-9a-fA-F]+) \((\d+) words\) ---$")
ROW_RE = re.compile(r"^\+0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)$")


def u32(blob: bytes, off: int) -> int:
    return struct.unpack_from("<I", blob, off)[0]


def read_section(path: pathlib.Path, want_block: str,
                 want_offset: int) -> list[int]:
    words: list[int] = []
    active = False
    expected_count = None

    with pathlib.Path(path).open(encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            match = SECTION_RE.match(line)
            if match:
                block = match.group(1)
                offset = int(match.group(2), 16)
                active = (block, offset) == (want_block, want_offset)
                expected_count = int(match.group(3)) if active else None
                if active:
                    words = []

                continue

            if not active:
                continue

            row = ROW_RE.match(line)
            if row:
                words.extend(int(word, 16) for word in row.group(1).split())
                if expected_count is not None and len(words) >= expected_count:
                    return words[:expected_count]

    sys.exit(f"{path}: missing {want_block} +0x{want_offset:04x} section")


def require_zero_words(label: str, words: list[int]) -> None:
    nonzero = [(i, word) for i, word in enumerate(words) if word]
    if nonzero:
        first = ", ".join(f"+0x{i * 4:03x}={word:#010x}" for i, word in nonzero[:8])
        sys.exit(f"{label}: raw_3dnr bank head is active: {first}")

    print(f"{label}: {len(words)} raw_3dnr bank-head words are zero")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tuning", required=True, type=pathlib.Path,
                    help="the NT99235 tuning blob")
    ap.add_argument("--vendor-capture", required=True, type=pathlib.Path,
                    help="stock slot-A register snapshot from au-snapshot-vendor.sh")
    ap.add_argument("--open-capture", type=pathlib.Path,
                    help="open-stack register snapshot using the same window list")
    args = ap.parse_args()

    blob = args.tuning.read_bytes()
    gate = u32(blob, RAW_3DNR_GATE)
    if gate != 0:
        sys.exit(f"raw_3dnr blob gate at {RAW_3DNR_GATE:#x} reads {gate}, expected 0")

    print(f"blob gate {RAW_3DNR_GATE:#x}: disabled")

    require_zero_words(
        "vendor capture",
        read_section(args.vendor_capture, *RAW_3DNR_SECTION)[:RAW_3DNR_WORDS],
    )

    if args.open_capture:
        require_zero_words(
            "open capture",
            read_section(args.open_capture, *RAW_3DNR_SECTION)[:RAW_3DNR_WORDS],
        )

    print("\nraw_3dnr is disabled in the shipped NT99235 FPV path")
    return 0


if __name__ == "__main__":
    sys.exit(main())
