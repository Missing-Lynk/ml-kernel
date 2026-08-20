#!/usr/bin/env python3
"""
Emit the vendor CVISP's static register images as a kernel header, from the
library alone.

CVISP is the block at 0x08e00000 (ISP base + 0x200000). Its register pages are
mostly computed at runtime, but three pages are installed from static images the
vendor service carries in its own data segment, exactly the way the ISP
submodule images are carried.

Provenance, all inside libmpp_service.so, and NOT what get_cvisp_init_config
points at:

  get_cvisp_init_config() (VA 0x242740) returns 0x4aa498, but that is the 3A
  algorithm ops object, not a register image: the words there relocate to
  artosyn_ae_algo_creat, artosyn_awb_algo_creat, artosyn_af_algo_creat and the
  nuc_device_module_* entries (R_AARCH64_ABS64 in .rela.dyn). It carries no
  register data. The cvisp_sub_*_creat constructors (gamma/lsc/rro_stats) only
  bfm_malloc a zeroed struct and register ops function pointers; they hold no
  bank literal and copy no image. So the ISP-side idiom -- a {source,length}
  array reached through get_*_init_config, banks from constructor literals --
  does not describe CVISP.

  The CVISP images instead live in the SAME ISP-init template array that
  gen-isp-library.py reads (the ar9311 array at VMA 0x48c770, selected for this
  SoC). They are the entries that generator leaves unmapped, because no
  isp_sub_* "[memcpy_dump]" string names them: they are installed by the CVISP /
  dvp_scaler path, whose init accessor at VA 0x1f4df0 returns this very array
  (adrp 0x48c000 + 0x770). Three entries carry CVISP register images:

    entry  9  -> CVISP page 0x0000, 40 registers   (arbitration / DMA routing)
    entry  0  -> CVISP page 0x4000, 13 registers   (frame geometry and limits)
    entry 55  -> CVISP page 0x8000, 224 registers  (output-descriptor setup)

  Each entry's {source, length} descriptor is read exactly as on the ISP side
  (arlib.template_entry). The CVISP page each installs at is fixed by the block's
  own layout, not by a constructor literal; it is pinned below and checked
  against the descriptor length.

An image is exactly its descriptor length. The page 0x8000 image is the
pre-enable state: its 0x8000 word is 0x00800800 (armed, output not enabled). The
streaming trace captures the runtime state written on top of it -- the output
enable, the scaled geometry re-stage, the ring buffer plane bases and a handful
of per-descriptor toggles -- so the trace and this image differ exactly on those
runtime registers and agree on every static one. See
out/au-cvisp-library/report.md for the entry-by-entry trace comparison.

Not applied by the driver. Like ar_isp_library, this is the reference set that
says which of the values the driver applies have a vendor origin.

    kernel/scripts/isp/gen-cvisp-library.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        -o overlay/drivers/media/artosyn/vendor-tables/ar-cvisp-library.h
"""

import argparse
import hashlib
import struct
import sys

import arlib

# sha256 of the air-unit libmpp_service.so this map was read from. Same build the
# ISP library map is pinned to; the ar9311 template array and the three CVISP
# images it points at are byte-identical on the slot-A library, so this one
# digest covers both.
LIB_SHA256 = "4cfc8e6cfb42d8c821137993b95b152f1aaad7c53ce425e6a0493c4dd453936c"

REG_BYTES = 4

# CVISP images carried in the ar9311 ISP-init template array. Each row is
# (template entry, name, CVISP page the image installs at, expected register
# count, kind). `kind` is "static" for a genuine vendor register image and
# "geometry" for a page whose every word is frame geometry or a derived limit,
# which a driver computes from the mode rather than copies.
#
# The page and the register count are pinned so a layout change trips the
# self-check instead of emitting silently. Both were established by aligning the
# descriptor's image against the streaming trace: entry 9 agrees 40/40, entry 0
# agrees 13/13, entry 55 agrees on 208 of its 224 registers, the 16 exceptions
# all being runtime state (report.md lists them).
IMAGES = (
    (9, "arbitration", 0x0000, 40, "static"),
    (0, "geometry", 0x4000, 13, "geometry"),
    (55, "output_setup", 0x8000, 224, "static"),
)

# Anchor: the first word of each image, checked before emit. A shifted array or a
# different SoC's array fails here rather than emitting the wrong page.
ANCHORS = {
    9: 0x00006003,
    0: 0x00000780,
    55: 0x00800800,
}

# Frame-geometry half-words: 960, 540, 1920, 1080 and the two output strides the
# vendor pairs them with. A register is geometry when at least one half is one of
# these dimensions and the other half is a dimension or zero -- so 0x00000780
# (960) and 0x021c03c0 (540 x 960) tag, but a plain 0x00000000 does not.
GEOM_DIMS = frozenset((0x03C0, 0x021C, 0x0780, 0x0438, 0x04A4, 0x0840))
GEOM_HALVES = GEOM_DIMS | {0x0000}

# CVISP registers the streaming vendor writes that no template-array image
# covers: the per-channel limit page 0x4200 and the wrap-cadence tick banks at
# 0x4600/0x4700. They are computed or per-frame, not carried as a static image;
# named here so a later reader does not hunt the library for them.
TRACE_ONLY = (
    (0x4200, "per-channel limits (0xf040 / 0x110), 16 registers"),
    (0x4600, "wrap-cadence tick banks, 8 registers"),
    (0x4610, "per-channel tick reload, computed"),
)

DIGEST_CHARS = 32


def is_geometry(value: int) -> bool:
    """True when `value` is a frame dimension, alone or paired, in its halves."""
    lo, hi = value & 0xFFFF, value >> 16
    return lo in GEOM_HALVES and hi in GEOM_HALVES and (lo in GEOM_DIMS
                                                        or hi in GEOM_DIMS)


def images(lib: bytes) -> list[tuple[int, str, int, str, list[int]]]:
    """(entry, name, page, kind, words) for every CVISP image."""
    out = []
    for entry, name, page, expect_regs, kind in IMAGES:
        source, length = arlib.template_entry(lib, entry)
        if not source or not length:
            sys.exit(f"template entry {entry} ({name}) is empty; "
                     f"the array layout has changed")

        if length % REG_BYTES:
            sys.exit(f"template entry {entry} ({name}) is 0x{length:x} bytes, "
                     f"not a whole number of registers")

        regs = length // REG_BYTES
        if regs != expect_regs:
            sys.exit(f"template entry {entry} ({name}) holds {regs} registers, "
                     f"expected {expect_regs}; the array layout has changed")

        raw = arlib.lib_slice(lib, source, length, f"{name} static image")
        words = list(struct.unpack(f"<{regs}I", raw))
        if words[0] != ANCHORS[entry]:
            sys.exit(f"template entry {entry} ({name}) begins 0x{words[0]:08x}, "
                     f"expected 0x{ANCHORS[entry]:08x}; the array layout has changed")

        out.append((entry, name, page, kind, words))

    return out


def emit(handle, digest: str, installed: list) -> None:
    write = handle.write
    total = sum(len(words) for *_, words in installed)

    write(arlib.banner("scripts/isp/gen-cvisp-library.py", [
        "The vendor CVISP static register images.",
        "",
        "Read from the ar9311 ISP-init template array in libmpp_service.so",
        f"(sha256 {digest[:DIGEST_CHARS]}), the same array gen-isp-library.py",
        "reads. These are the entries that generator leaves unmapped: they carry",
        "no isp_sub_* name because the CVISP / dvp_scaler path installs them, not",
        "an ISP submodule. get_cvisp_init_config does NOT point here -- it",
        "returns the 3A algorithm ops object, which holds function pointers, not",
        "register data.",
        "",
        "Three template entries carry a CVISP image:",
        "  entry  9 -> page 0x0000, arbitration / DMA routing",
        "  entry  0 -> page 0x4000, frame geometry and limits (driver-derivable)",
        "  entry 55 -> page 0x8000, output-descriptor setup",
        "",
        "An image is exactly its descriptor length. The page 0x8000 image is the",
        "pre-enable state (0x8000 = 0x00800800); the streaming trace writes the",
        "output enable, the geometry re-stage and the ring buffer plane bases on",
        "top of it, so it and ar_cvisp_setup agree on every static register and",
        "differ only on those runtime ones. Geometry rows carry both frame",
        "dimensions in their two halves and are tagged; a driver derives them",
        "from the mode instead of copying them.",
        "",
        "Not applied by the driver. It is the reference set that says which of",
        "the values the driver does apply have a vendor origin.",
    ]))
    write("\n")

    guard_open, guard_close = arlib.guard("AR_CVISP_LIBRARY_H")
    write(guard_open + "\n")
    write('#include "ar-cvisp-defaults.h"\n\n')

    write("/*\n")
    write(f" * {total} registers across {len(installed)} CVISP images.\n")
    write(" *\n")
    write(" * Each block comment gives the template array entry the image comes\n")
    write(" * from, so a reader can check the binding without rerunning the\n")
    write(" * generator. Rows tagged geom are frame geometry a driver derives\n")
    write(" * from the mode.\n")
    write(" */\n")
    write("static const struct ar_cvisp_reg ar_cvisp_library[] = {\n")
    for entry, name, page, kind, words in installed:
        note = " (frame geometry and limits, driver-derivable)" \
            if kind == "geometry" else ""
        write(f"\t/* {name}: entry {entry}, {len(words)} registers at "
              f"page {page:#06x}{note} */\n")
        for i, value in enumerate(words):
            tag = "\t/* geom */" if is_geometry(value) else ""
            write(f"\t{{ {page + REG_BYTES * i:#06x}, {value:#010x} }},{tag}\n")

    write("};\n\n")

    write("/*\n")
    write(" * CVISP registers the streaming vendor writes that no template-array\n")
    write(" * image covers -- computed or per-frame, not a static image:\n")
    write(" *\n")
    for page, what in TRACE_ONLY:
        write(f" *   page {page:#06x}: {what}\n")

    write(" */\n\n")
    write(guard_close)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lib", required=True, help="vendor libmpp_service.so")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    with open(args.lib, "rb") as handle:
        lib = handle.read()

    digest = hashlib.sha256(lib).hexdigest()
    if digest != LIB_SHA256:
        sys.exit("libmpp_service.so sha256 mismatch: the CVISP image map does not apply")

    installed = images(lib)

    with open(args.output, "w") as handle:
        emit(handle, digest, installed)

    return 0


if __name__ == "__main__":
    sys.exit(main())
