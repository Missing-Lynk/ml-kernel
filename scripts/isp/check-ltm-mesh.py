#!/usr/bin/env python3
"""
The LTM/gtm2 hardware-programming layer: mesh geometry and its registers.

Our driver publishes an identity LTM page because it cannot compute the real one:
the neo_v2 producer (userspace/ml-ltm/ml-ltm-core.c) needs two clip-shaping tables
the ISP hardware writes, and nothing programs our ISP to produce them. That
programming layer is the vendor's isp_sub_ltm.c / isp_sub_gtm2.c set_format path,
at library 0x18c1d8 with identical copies at 0x189000 and 0x18fb00 (the same source
compiled into the gtm2 and ltm modules).

What that path does, read out of the library and reproduced here:

  1. Map the mesh dimensions to a register field. mesh_w_h_to_reg is a log2:
     a mesh of 4, 8, 16 or 32 becomes 2, 3, 4 or 5, and the value lands in bits
     5..7 of the bank's first register. Anything else falls back to 8.
  2. Stage the frame size: width into bits 16..27 of the second register, height
     into bits 0..11.
  3. Divide the frame into blocks, one per mesh cell, and give the last column and
     the last row the remainder. This is the geometry the per-tile statistics the
     page producer consumes are accumulated over.
  4. Double the mesh and repeat, which is the second pass the vendor logs as
     "22222 block_w=...".

The shipped configuration is an 8x8 mesh, and 8x8 is 64 tiles, which is exactly the
tile count of the LTM page (64 tiles x 128 u16). That agreement is the check that
this is the right code: the page geometry falls out of the mesh geometry rather than
being asserted.

Run from kernel/:  python3 scripts/isp/check-ltm-mesh.py
"""

from __future__ import annotations

import pathlib
import re
import sys

HERE: pathlib.Path = pathlib.Path(__file__).resolve().parent
LIBRARY_HEADER: pathlib.Path = (HERE.parent.parent
                                / "overlay/drivers/media/artosyn/vendor-tables/ar-isp-library.h")

# The ISP bank the module owns, and the offsets the set_format path stages into it.
LTM_BANK: int = 0x2800

# The mesh pass count. set_format runs the block-geometry pass twice and carries a
# counter of 2 then 4; the second value is what a unit that finished both reads.
AR_ISP_LTM_MESH_PASSES: int = 0x2834
MESH_PASS_SECOND: int = 4

# Library addresses, so a reader can check the decode without rerunning a search.
SET_FORMAT: int = 0x18C1D8          # the copy in the gtm2 module
MESH_TO_REG_LOG: int = 0x18C27C     # "w=%d h=%d reg_val=%d"
BLOCK_LOG: int = 0x18C398           # "block_w=%d block_h=%d block_w_last_col=%d ..."

# Frame the shipped FPV mode configures.
FRAME_WIDTH: int = 1920
FRAME_HEIGHT: int = 1080

# The mesh the vendor ships. Both dimensions 8, taken by the equality test at
# 0x18c210/0x18c22c that jumps to the 0x60 case.
MESH_WIDTH: int = 8
MESH_HEIGHT: int = 8

# The LTM page the driver publishes: 64 tiles of 128 u16 samples.
PAGE_TILES: int = 64

# Register field positions in the module's shadow bank, from the read-modify-writes
# at 0x18c2d8 (first register) and 0x18c2f8 (second).
MESH_FIELD_SHIFT: int = 5
MESH_FIELD_MASK: int = 0xFFFFFF1F    # the bits the vendor clears before OR-ing
WIDTH_SHIFT: int = 16
WIDTH_MASK: int = 0xF000FFFF
HEIGHT_MASK: int = 0xFFFFF000

# The mesh sizes with a dedicated case, mapped to the value the field takes.
MESH_TO_REG: dict[int, int] = {4: 2, 8: 3, 16: 4, 32: 5}
MESH_DEFAULT_REG: int = 3            # the csel fallback at 0x18c53c/0x18c550


def mesh_w_h_to_reg(mesh_width: int, mesh_height: int) -> int:
    """
    The field value for one mesh size, as mesh_w_h_to_reg computes it.

    The vendor tests the pairs in order (8,8), (4,4), (16,16), (32,32) and falls
    back to the 8 case, so a mismatched pair is not an error, it is 8.
    """
    if mesh_width != mesh_height:
        return MESH_DEFAULT_REG

    return MESH_TO_REG.get(mesh_width, MESH_DEFAULT_REG)


def block_geometry(size: int, mesh: int) -> tuple[int, int]:
    """
    One axis of the block grid: the block size, and the last block's size.

    sdiv then msub then add, at 0x18c348 and 0x18c364: the last block carries the
    remainder on top of a full block, so the grid covers the frame exactly.
    """
    block: int = size // mesh
    last: int = block + (size - block * mesh)
    return block, last


def mesh_register_words(width: int, height: int, mesh_width: int,
                        mesh_height: int) -> tuple[int, int]:
    """The two register words the layer stages, from a cleared bank."""
    first: int = (0 & MESH_FIELD_MASK) | (mesh_w_h_to_reg(mesh_width, mesh_height)
                                          << MESH_FIELD_SHIFT)
    second: int = (0 & WIDTH_MASK) | (width << WIDTH_SHIFT)
    second = (second & HEIGHT_MASK) | height
    return first, second


# The two slot-A captures the bank diff is read from, relative to the repo root.
VENDOR_CAPTURE: str = "out/au-snapshot/registers.txt"
OPEN_CAPTURE: str = "out/au-snapshot/ours-registers-live.txt"

# Registers whose value cannot match by construction, with why.
BANK_EXEMPT: dict[int, str] = {
    0x2808: "the page address, our own DMA allocation",
    0x280C: "the statistics address, our own DMA allocation",
    0x2838: "hardware-owned, in ar_isp_hw_owned: it changes between two identical "
            "reads with the configuration held still",
}


def capture_bank(path: pathlib.Path) -> list[int]:
    """The 18-word ltm bank out of a register capture, or an empty list."""
    if not path.exists():
        return []

    words: list[int] = []
    collecting: bool = False
    for line in path.read_text(errors="replace").split("\n"):
        if line.startswith(f"--- isp +{LTM_BANK:#06x}"):
            collecting = True
            continue

        if collecting:
            row = re.match(r"\+0x[0-9a-f]+:\s+(.*)", line.strip())
            if row is None:
                break

            words += [int(word, 16) for word in row.group(1).split()]

    return words[:18]


def driver_writes_mesh_passes() -> bool:
    """Whether the driver stages the mesh pass count, which the captures say it must."""
    source: pathlib.Path = (HERE.parent.parent
                            / "overlay/drivers/media/artosyn/ar-isp-tables.c")
    return "AR_ISP_LTM_MESH_PASSES" in source.read_text()


def library_image() -> dict[int, int]:
    """The ltm/gtm2 static register image the driver installs, from ar-isp-library.h."""
    text: str = LIBRARY_HEADER.read_text()
    rows: dict[int, int] = {}
    for off, val in re.findall(r"\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*\}", text):
        offset: int = int(off, 16)
        if LTM_BANK <= offset < LTM_BANK + 0x100:
            rows.setdefault(offset, int(val, 16))

    return rows


def main() -> int:
    failures: list[str] = []

    def check(label: str, got: object, want: object) -> None:
        mark: str = "ok  " if got == want else "FAIL"
        if got != want:
            failures.append(f"{label}: got {got}, want {want}")

        print(f"  {mark} {label}: {got}")

    print(f"mesh_w_h_to_reg (library {MESH_TO_REG_LOG:#x}), the log2 mapping:")
    for mesh, want in ((4, 2), (8, 3), (16, 4), (32, 5)):
        check(f"mesh {mesh}x{mesh}", mesh_w_h_to_reg(mesh, mesh), want)

    check("mesh 8x4 falls back to the 8 case", mesh_w_h_to_reg(8, 4), MESH_DEFAULT_REG)
    check("mesh 5x5 falls back to the 8 case", mesh_w_h_to_reg(5, 5), MESH_DEFAULT_REG)

    print(f"\nthe shipped mesh is {MESH_WIDTH}x{MESH_HEIGHT}, and the LTM page has "
          f"{PAGE_TILES} tiles:")
    check("mesh cells equal the page's tile count",
          MESH_WIDTH * MESH_HEIGHT, PAGE_TILES)

    print(f"\nblock grid over {FRAME_WIDTH}x{FRAME_HEIGHT} at the shipped mesh "
          f"(library {BLOCK_LOG:#x}):")
    block_w, last_col = block_geometry(FRAME_WIDTH, MESH_WIDTH)
    block_h, last_row = block_geometry(FRAME_HEIGHT, MESH_HEIGHT)
    check("block_w", block_w, 240)
    check("block_h", block_h, 135)
    check("block_w_last_col", last_col, 240)
    check("block_h_last_row", last_row, 135)
    check("the grid covers the frame, width",
          block_w * (MESH_WIDTH - 1) + last_col, FRAME_WIDTH)
    check("the grid covers the frame, height",
          block_h * (MESH_HEIGHT - 1) + last_row, FRAME_HEIGHT)

    print("\nthe second pass, at double the mesh (the vendor's \"22222\" log):")
    block_w2, last_col2 = block_geometry(FRAME_WIDTH, MESH_WIDTH * 2)
    block_h2, last_row2 = block_geometry(FRAME_HEIGHT, MESH_HEIGHT * 2)
    check("block_w", block_w2, 120)
    check("block_h", block_h2, 67)
    check("block_w_last_col", last_col2, 120)
    check("block_h_last_row, carrying the remainder", last_row2, 75)
    check("the grid still covers the frame, height",
          block_h2 * (MESH_HEIGHT * 2 - 1) + last_row2, FRAME_HEIGHT)

    print("\nthe two register words the layer stages, from a cleared bank:")
    first, second = mesh_register_words(FRAME_WIDTH, FRAME_HEIGHT,
                                        MESH_WIDTH, MESH_HEIGHT)
    check("bank+0x00, mesh field in bits 5..7", f"{first:#010x}", "0x00000060")
    check("bank+0x04, width in 16..27 and height in 0..11",
          f"{second:#010x}", "0x07800438")

    print(f"\nagainst the bank the driver already installs, ltm entry 39 at "
          f"{LTM_BANK:#06x}:")
    image: dict[int, int] = library_image()
    check(f"{LTM_BANK + 0x04:#06x} is the staged frame size",
          f"{image[LTM_BANK + 0x04]:#010x}", f"{second:#010x}")
    check(f"{LTM_BANK + 0x30:#06x} carries the 0x1f the layer writes",
          image[LTM_BANK + 0x30] & 0x1F, 0x1F)
    template_mesh: int = (image[LTM_BANK] >> MESH_FIELD_SHIFT) & 0x7
    print(f"  note {LTM_BANK:#06x} bits 5..7 read {template_mesh}, a "
          f"{1 << template_mesh}x{1 << template_mesh} mesh: the static template was built for")
    print("       a different mesh than the shipped one, which is what set_format re-stages.")

    root: pathlib.Path = HERE.parent.parent.parent
    vendor: list[int] = capture_bank(root / VENDOR_CAPTURE)
    ours: list[int] = capture_bank(root / OPEN_CAPTURE)
    if vendor and ours:
        print("\nthe whole bank against the streaming vendor, from the slot-A captures.")
        print("These predate the driver write below, so a difference here is the gap as")
        print("it was measured, not the gap as it stands:")
        differing: list[int] = []
        for index, (want, got) in enumerate(zip(vendor, ours, strict=True)):
            register: int = LTM_BANK + 4 * index
            if want == got or register in BANK_EXEMPT:
                continue

            differing.append(register)

        for register in differing:
            index = (register - LTM_BANK) // 4
            print(f"    {register:#06x}  vendor {vendor[index]:#010x}  "
                  f"ours {ours[index]:#010x}")

        check("every difference is a register the driver now writes",
              [f"{register:#06x}" for register in differing
               if register != AR_ISP_LTM_MESH_PASSES], [])
        check(f"the vendor's {AR_ISP_LTM_MESH_PASSES:#06x} is the second-pass count",
              vendor[(AR_ISP_LTM_MESH_PASSES - LTM_BANK) // 4], MESH_PASS_SECOND)

        for register, why in BANK_EXEMPT.items():
            print(f"    {register:#06x} exempt: {why}")
    else:
        print("\nthe slot-A captures are absent, skipping the bank diff")

    print()
    check(f"the driver writes {AR_ISP_LTM_MESH_PASSES:#06x}, the mesh pass count",
          driver_writes_mesh_passes(), True)

    if failures:
        print(f"\n{len(failures)} check(s) failed:")
        for line in failures:
            print(f"    {line}")

        return 1

    print("\nAll checks passed. The mesh and block geometry the LTM statistics are")
    print("accumulated over is derivable from the mode, it lands in the bank the driver")
    print("already owns, and the mesh pass count the vendor leaves in 0x2834 is staged.")
    print("What the driver still cannot do is emit the clip tables the page producer")
    print("consumes: how the hardware learns about those two buffers is untraced, and")
    print("out/au-ltm-hwprog/report.md carries where the search stands.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
