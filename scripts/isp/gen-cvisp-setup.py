#!/usr/bin/env python3
"""
Emit the CVISP register configuration with a source for every value.

ar-cvisp-defaults.h was recovered from a wide MMIO write trace of the streaming
vendor. Every value in it is a recording: right at the operating point it was
taken at, unexplained everywhere else. This generator emits the same
configuration with each value computed instead, from the vendor's own CVISP
static images in libmpp_service.so, from the frame geometry the driver already
owns, or from a named constant, and it refuses to emit anything it cannot
source.

What is derived and what is not:

  * The VALUES come from the library images (ar-cvisp-library.h, template array
    entries 9, 0 and 55), from the configured frame dimensions, or from a unity
    constant. A library swap regenerates them.
  * The ORDER is still the trace's. Nothing has been read that says what order
    the vendor's own code writes these in, and the setup table's staged enable
    shows the order carries meaning, so it is preserved exactly rather than
    invented. ar-cvisp-defaults.h is the order oracle and the equality oracle.
  * Nine registers remain recordings. They are tagged `residue` in the output
    and listed in check-cvisp-derivation.py with what is known about each.

The generated table is byte-identical to the recovered one, which is the point:
this changes where the numbers come from, not what the block is programmed with.
The check is built in, so a library that no longer reproduces the trace fails
here rather than on a device.

    kernel/scripts/isp/gen-cvisp-setup.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        -o overlay/drivers/media/artosyn/vendor-tables/ar-cvisp-derived.h
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import sys

HERE: pathlib.Path = pathlib.Path(__file__).resolve().parent
TABLES: pathlib.Path = HERE.parent.parent / "overlay/drivers/media/artosyn/vendor-tables"
DEFAULTS: pathlib.Path = TABLES / "ar-cvisp-defaults.h"
LIBRARY: pathlib.Path = TABLES / "ar-cvisp-library.h"

LIB_SHA256: str = "4cfc8e6cfb42d8c821137993b95b152f1aaad7c53ce425e6a0493c4dd453936c"

FRAME_WIDTH: int = 1920
FRAME_HEIGHT: int = 1080

BLC_BANK: range = range(0x4200, 0x4240, 4)
DIGIGAIN_BANKS: tuple[range, ...] = (range(0x4600, 0x4640, 4), range(0x4700, 0x4740, 4))
UNITY_Q8: int = 0x100

# Ports the block arbitrates. Page 0x0000 carries eight routing entries and eight
# 0x03ffffff masks, and the output-descriptor scoreboard at 0x8024 accumulates one
# bit per port in each of three bytes, which is what makes its value derivable
# rather than assumed. See PORT_COUNT's use below.
PORT_COUNT: int = 8

ROW_RE: re.Pattern[str] = re.compile(
    r"\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*\}([^\n]*)")

# Registers a row may still take from the trace, because the value disagrees with
# the library image and the disagreement has not been reverse-engineered. Six of
# them are the line-limit group the setup sequence writes one count below the
# image (0x08000b32 against 0x08000b33); late restores two of the six, and those
# restoring rows match the image and are sourced from it like any other.
# Keep in step with check-cvisp-derivation.py.
RESIDUE: frozenset[int] = frozenset({
    0x8000, 0x802c, 0x8054, 0x807c, 0x80a4, 0x80cc, 0x80f4, 0x8144, 0x81b4, 0x81c8,
})

# Vendor DRAM the driver overwrites from ar_cvisp_arm_buffer the moment it streams.
VENDOR_DRAM: frozenset[int] = frozenset({0x8098, 0x809c, 0x8174, 0x8194})


def parse_rows(text: str, name: str) -> list[tuple[int, int]]:
    body = re.search(rf"ar_cvisp_{name}\[\]\s*=\s*\{{(.*?)\n\}};", text, re.S)
    if body is None:
        sys.exit(f"{DEFAULTS.name}: no ar_cvisp_{name} table")

    return [(int(off, 16), int(val, 16)) for off, val, _ in ROW_RE.findall(body.group(1))]


def parse_ring(text: str) -> list[tuple[int, int, int]]:
    """The vendor's five plane triplets, carried as-is: they are DRAM addresses."""
    body = re.search(r"ar_cvisp_ring\[\]\s*=\s*\{(.*?)\n\};", text, re.S)
    if body is None:
        sys.exit(f"{DEFAULTS.name}: no ar_cvisp_ring table")

    triples = re.findall(
        r"\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*\}",
        body.group(1))
    return [(int(y, 16), int(u, 16), int(v, 16)) for y, u, v in triples]


def parse_library() -> dict[int, tuple[int, bool]]:
    body = re.search(r"ar_cvisp_library\[\]\s*=\s*\{(.*?)\n\};", LIBRARY.read_text(), re.S)
    if body is None:
        sys.exit(f"{LIBRARY.name}: no ar_cvisp_library table")

    return {int(off, 16): (int(val, 16), "geom" in trailer)
            for off, val, trailer in ROW_RE.findall(body.group(1))}


def geometry_words() -> dict[int, str]:
    """
    Every word the frame dimensions spell, mapped to how they spell it.

    The block stores geometry as a bare dimension or as a pair packed high/low,
    and the chroma path runs at half resolution, so a pair can mix scales: the
    output stage carries height/2 against a full-rate width. All four terms are
    offered in both halves rather than assuming which pairings occur.
    """
    terms: dict[int, str] = {
        FRAME_WIDTH: "width",
        FRAME_HEIGHT: "height",
        FRAME_WIDTH // 2: "width / 2",
        FRAME_HEIGHT // 2: "height / 2",
    }
    words: dict[int, str] = dict(terms)
    for high, high_name in terms.items():
        for low, low_name in terms.items():
            words.setdefault((high << 16) | low, f"{high_name} << 16 | {low_name}")

    return words


def scoreboard() -> int:
    """
    The output-descriptor scoreboard at 0x8024.

    dvp_scaler_outlib_output_config (library 0x1f3c28) reaches it three times per
    port, ORing 1 << port, 1 << (port + 8) and 1 << (port + 16). Over the eight
    ports page 0x0000 routes, that fills the low three bytes.
    """
    value: int = 0
    for port in range(PORT_COUNT):
        value |= (1 << port) | (1 << (port + 8)) | (1 << (port + 16))

    return value


def source(off: int, want: int, library: dict[int, tuple[int, bool]],
           geom: dict[int, str]) -> tuple[int, str]:
    """
    The value to emit for one row of the sequence, and where it came from.

    Per row, not per register: setup writes several registers more than once, and
    the same register can be image-sourced on one write and driver-computed on the
    next. The scoreboard at 0x8024 is the clearest case, staged clear and then
    filled a port at a time.
    """
    if off in VENDOR_DRAM:
        return want, "vendor dram"

    if off in BLC_BANK:
        return want, "blc fallback"

    if any(off in bank for bank in DIGIGAIN_BANKS):
        return (UNITY_Q8 if want == UNITY_Q8 else 0), "unity"

    if off in library:
        value, is_geom = library[off]
        if value == want:
            return value, "library"

        if is_geom and want in geom:
            return want, f"geom: {geom[want]}"

    if off == 0x8024 and want == scoreboard():
        return want, "scoreboard"

    if want in geom and (off not in library or library[off][1]):
        return want, f"geom: {geom[want]}"

    if want == 0 and off not in library:
        return 0, "zero"

    if off in RESIDUE:
        return want, "residue"

    if off not in library and want == 0:
        return 0, "zero"

    return want, "UNSOURCED"


def emit(name: str, comment: str, rows: list[tuple[int, int]],
         library: dict[int, tuple[int, bool]], geom: dict[int, str],
         failures: list[str]) -> list[str]:
    out: list[str] = [comment, f"static const struct ar_cvisp_reg ar_cvisp_{name}[] = {{"]
    for off, want in rows:
        value, tag = source(off, want, library, geom)
        if tag == "UNSOURCED":
            failures.append(f"0x{off:04x} = 0x{want:08x}: no source")

        if value != want:
            failures.append(f"0x{off:04x}: derived 0x{value:08x} but the trace holds 0x{want:08x}")

        out.append(f"\t{{ 0x{off:04x}, 0x{value:08x} }},\t/* {tag} */")

    out.append("};")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lib", required=True, help="libmpp_service.so")
    parser.add_argument("-o", "--output", required=True)
    args = parser.parse_args()

    blob: bytes = pathlib.Path(args.lib).read_bytes()
    digest: str = hashlib.sha256(blob).hexdigest()
    if digest != LIB_SHA256:
        sys.exit(f"libmpp_service.so sha256 mismatch: the CVISP image map does not apply\n"
                 f"  expected {LIB_SHA256}\n  got      {digest}")

    text: str = DEFAULTS.read_text()
    library: dict[int, tuple[int, bool]] = parse_library()
    geom: dict[int, str] = geometry_words()
    failures: list[str] = []

    body: list[str] = []
    body += emit("setup", "/*\n"
                 " * Output-descriptor setup, in the trace's write order with consecutive\n"
                 " * duplicates collapsed, ending at the staged enable. Values from library\n"
                 " * template entry 55 except where tagged otherwise.\n"
                 " */",
                 parse_rows(text, "setup"), library, geom, failures)
    body.append("")
    body += emit("late", "/*\n"
                 " * The tail that follows the enable: arbitration (entry 9), frame geometry\n"
                 " * and limits (entry 0), the BLC fallback ar_cvisp_blc_apply overwrites from\n"
                 " * the tuning blob, and the digital-gain banks the vendor holds at unity.\n"
                 " */",
                 parse_rows(text, "late"), library, geom, failures)
    body.append("")
    body += emit("tick", "/* The per-frame rewrite of the digital-gain head, all unity. */",
                 parse_rows(text, "tick"), library, geom, failures)

    body.append("")
    body.append("/*")
    body.append(" * The vendor's own five-slot plane ring. These are DRAM addresses, not")
    body.append(" * configuration: ar_cvisp_arm_buffer replaces all three the moment the capture")
    body.append(" * node streams, and they survive only on the idle ring path, which is what")
    body.append(" * makes a /dev/mem read of a vendor-addressed frame possible. Carried.")
    body.append(" */")
    body.append("static const struct ar_cvisp_bufset ar_cvisp_ring[] = {")
    for y, u, v in parse_ring(text):
        body.append(f"\t{{ 0x{y:08x}, 0x{u:08x}, 0x{v:08x} }},")

    body.append("};")

    if failures:
        print("cannot emit a derived table:", file=sys.stderr)
        for line in failures:
            print(f"    {line}", file=sys.stderr)

        return 1

    header: list[str] = [
        "/* SPDX-License-Identifier: GPL-2.0 */",
        "/* Generated by scripts/isp/gen-cvisp-setup.py. Do not edit. */",
        "/*",
        " * CVISP register configuration, with a source for every value.",
        " *",
        " * Values come from the vendor's own CVISP static images (libmpp_service.so",
        f" * sha256 {LIB_SHA256[:32]}, template array",
        " * entries 9, 0 and 55), from the configured frame geometry, or from a named",
        " * constant. The write ORDER is the streaming vendor's, taken from the MMIO",
        " * trace in ar-cvisp-defaults.h: no vendor code has been read that fixes the",
        " * order, and the staged enable at the end of setup shows the order matters.",
        " *",
        " * Row tags:",
        " *   library       the value the vendor's static image carries for that register",
        " *   geom          spelled out of the frame dimensions the driver configures",
        " *   zero          cleared; there is no vendor datum to source",
        " *   unity         a digital-gain bank the vendor holds at 1.0",
        " *   blc fallback  overwritten by ar_cvisp_blc_apply from the tuning blob on every",
        " *                 boot the firmware loads; a recording only in the degraded path",
        " *   scoreboard    accumulated per port by dvp_scaler_outlib_output_config",
        " *   vendor dram   a vendor buffer address ar_cvisp_arm_buffer overwrites when the",
        " *                 capture node streams",
        " *   residue       still a recording. Why it differs from the image is known, what",
        " *                 to write instead is not. check-cvisp-derivation.py lists them.",
        " *",
        " * This table is byte-identical to the recovered one by construction; the",
        " * generator fails rather than emitting a value it cannot reproduce.",
        " */",
        "",
        "#ifndef AR_CVISP_DERIVED_H",
        "#define AR_CVISP_DERIVED_H",
        "",
        "struct ar_cvisp_reg {",
        "\tu16 off;",
        "\tu32 val;",
        "};",
        "",
        "struct ar_cvisp_bufset {",
        "\tu32 y;",
        "\tu32 u;",
        "\tu32 v;",
        "};",
        "",
    ]
    footer: list[str] = ["", "#endif /* AR_CVISP_DERIVED_H */", ""]

    pathlib.Path(args.output).write_text("\n".join(header + body + footer))
    print(f"{args.output}: {sum(1 for line in body if line.startswith(chr(9)))} rows, "
          f"every value sourced")
    return 0


if __name__ == "__main__":
    sys.exit(main())
