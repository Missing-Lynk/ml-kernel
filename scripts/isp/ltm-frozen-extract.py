#!/usr/bin/env python3
"""
Locate the LTM structures in a frozen ar_lowdelay heap dump (vendor-session stage 8).

Stage 8 of glue/camera/au-vendor-session.sh SIGSTOPs the vendor media process and dumps its
heap, so the dump holds one frame's LTM input and output together. This finds them:

  1. every well-formed LTM output page: 64 tiles of a 128-sample u16 curve at a 0x100 stride,
     each curve monotonic from near zero into the top of a 10-bit range (the same geometry
     check-ltm-page.py trusts);
  2. every 8-byte pointer in the heap referring to each page, which is where the owning
     structure holds it: the algorithm writes the page at ltm_ctx+1464 and the publish path
     double-buffers through slots at ltm_ctx[552]+20056/20072, so a pointer's address is an
     ltm_ctx candidate (plans/done/au-ltm-page-algorithm.md section 3);
  3. a hexdump of the input window before each candidate output, ltm_ctx+1320..+1464, whose
     u32 fields (+32/36/44/48/88/124/128 relative to the input) are the CLAHE parameters the
     fit needs.

The heap base comes from the maps file the harness saves beside the dumps. Pages and the input
windows are written next to the dump as <dump>.page<N>.bin / .ctx<N>.txt for the fit to consume.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

TILES = 64
SAMPLES = 128
STRIDE = 0x100
PAGE_SIZE = TILES * STRIDE
SAMPLE_MAX = 0x3FF

# ltm_ctx offsets, measured in plans/done/au-ltm-page-algorithm.md section 2.
CTX_INPUT_OFF = 1320
CTX_OUTPUT_OFF = 1464
CTX_WINDOW = 0x600          # dumped around each candidate ctx, covers 0..2872 generously


def heap_base_from_maps(maps_text: str) -> int:
    """The [heap] mapping's start address."""
    for line in maps_text.splitlines():
        if line.rstrip().endswith("[heap]"):
            return int(line.split("-", 1)[0], 16)

    sys.exit("no [heap] line in the maps file")


def is_page_at(heap: bytes, off: int) -> bool:
    """The check-ltm-page.py geometry: 64 monotonic curves ending in the 10-bit top."""
    if off + PAGE_SIZE > len(heap):
        return False

    for tile in range(TILES):
        base = off + tile * STRIDE
        curve = struct.unpack_from(f"<{SAMPLES}H", heap, base)

        if curve[0] > 16 or not 900 <= curve[-1] <= SAMPLE_MAX:
            return False

        if any(b < a for a, b in zip(curve, curve[1:], strict=False)):
            return False

    return True


def find_pages(heap: bytes) -> list[int]:
    """Offsets of every well-formed page. Cheap rejects first: a full check per position
    would take minutes over a 20 MB dump."""
    hits: list[int] = []
    off = 0
    limit = len(heap) - PAGE_SIZE

    while off <= limit:
        first, last = struct.unpack_from("<2H", heap, off), None
        if first[0] <= 16:
            last = struct.unpack_from("<H", heap, off + (SAMPLES - 1) * 2)[0]

        if last is not None and 900 <= last <= SAMPLE_MAX and is_page_at(heap, off):
            hits.append(off)
            off += PAGE_SIZE
            continue

        off += 8

    return hits


def find_pointers(heap: bytes, heap_base: int, target_off: int) -> list[int]:
    """Offsets of every aligned u64 holding the virtual address of target_off."""
    want = struct.pack("<Q", heap_base + target_off)
    hits: list[int] = []
    start = 0

    while True:
        i = heap.find(want, start)
        if i < 0:
            return hits

        if i % 8 == 0:
            hits.append(i)

        start = i + 1


def hexdump(data: bytes, base_off: int) -> str:
    lines: list[str] = []
    for row in range(0, len(data), 16):
        words = " ".join(
            f"{struct.unpack_from('<I', data, row + col)[0]:08x}"
            for col in range(0, min(16, len(data) - row), 4)
        )
        lines.append(f"{base_off + row:08x}  {words}")

    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("dump", type=Path, help="ltm-frozen-N.bin from vendor-session stage 8")
    parser.add_argument("--maps", type=Path, required=True,
                        help="ltm-frozen-maps.txt saved beside the dump")
    args = parser.parse_args()

    heap = args.dump.read_bytes()
    base = heap_base_from_maps(args.maps.read_text())
    print(f"heap {len(heap):#x} bytes at {base:#x}")

    pages = find_pages(heap)
    if not pages:
        sys.exit("no well-formed LTM page in the dump: the freeze may have caught the compute "
                 "mid-write, try the next repeat")

    for n, off in enumerate(pages):
        va = base + off
        out = args.dump.with_suffix(f".page{n}.bin")
        out.write_bytes(heap[off:off + PAGE_SIZE])
        print(f"page {n}: offset {off:#x} va {va:#x} -> {out.name}")

        for ptr_off in find_pointers(heap, base, off):
            print(f"  pointer at offset {ptr_off:#x} va {base + ptr_off:#x}")

            # If this slot is ltm_ctx+1464, the input window sits just before it.
            ctx_off = ptr_off - CTX_OUTPUT_OFF
            if ctx_off >= 0 and ctx_off + CTX_WINDOW <= len(heap):
                ctx = args.dump.with_suffix(f".ctx{n}.txt")
                window = heap[ctx_off + CTX_INPUT_OFF:ctx_off + CTX_INPUT_OFF + 160]
                ctx.write_text(
                    f"ltm_ctx candidate at offset {ctx_off:#x} va {base + ctx_off:#x}\n"
                    f"input window ltm_ctx+{CTX_INPUT_OFF}:\n"
                    + hexdump(window, ctx_off + CTX_INPUT_OFF) + "\n")
                print(f"  ltm_ctx candidate {ctx_off:#x} -> {ctx.name}")


if __name__ == "__main__":
    main()
