#!/usr/bin/env python3
"""
Can the CVISP applied path be composed from vendor-owned sources instead of replayed?

ar-cvisp.c installs ar-cvisp-defaults.h, a table recovered from a wide MMIO write
trace of the streaming vendor. A trace is a recording: correct at the operating
point it was taken at and unexplained everywhere else, which is the same standing
objection audit-provenance.py exists to answer for the main ISP. That audit reads
ar-isp-defaults.h only, so CVISP has never been inside it.

This script asks whether the trace can be replaced. It composes the register state
the driver would reach from vendor-owned sources alone, in the driver's own apply
order, and diffs that against the state the trace path reaches today. Sources, in
the order the composition applies them:

  library image        the value is what the vendor's own CVISP static image
                       carries for that register, per ar-cvisp-library.h
  frame geometry       the value decomposes into the configured frame dimensions,
                       which the driver owns and derives from the mode
  zero write           the register is cleared; there is no vendor datum to source
  unity gain           a digital-gain bank the vendor holds at 1.0, so the value
                       is the unity constant rather than a recorded number
  blc constant         the fallback the BLC pass overwrites from the tuning blob
                       whenever the firmware loads; a recording only in the
                       degraded path, and ar_cvisp_blc_apply runs immediately after
  vendor DRAM address  a vendor buffer address the driver overwrites with its own
                       allocation the moment the capture node streams
  trace residue        the register still differs from the library image and the
                       difference has NOT been reverse-engineered. Why it differs
                       is known for each one below; what value to write instead is
                       not. These stay recordings until somebody reads the writer.
  UNSOURCED            everything else: not even a reason for the difference

The residue is the honest remainder. Switching ar-cvisp.c to the library images
shrinks the replay to those registers; it does not remove it. Exit status is 0 when
the composition reproduces the trace exactly and nothing is UNSOURCED, which is the
condition for making that switch.

Run from kernel/:  python3 scripts/isp/check-cvisp-derivation.py
"""

from __future__ import annotations

import pathlib
import re
import sys

HERE: pathlib.Path = pathlib.Path(__file__).resolve().parent
TABLES: pathlib.Path = HERE.parent.parent / "overlay/drivers/media/artosyn/vendor-tables"
DEFAULTS: pathlib.Path = TABLES / "ar-cvisp-defaults.h"
LIBRARY: pathlib.Path = TABLES / "ar-cvisp-library.h"
ROOT: pathlib.Path = HERE.parent.parent.parent

# Slot-A register captures: the streaming vendor, and our unit at the same operating
# point. The windows they cover are what decides whether a value is merely recorded
# or recorded AND measured equal to the vendor.
VENDOR_CAPTURE: pathlib.Path = ROOT / "out/au-snapshot/registers.txt"
OPEN_CAPTURE: pathlib.Path = ROOT / "out/au-snapshot/ours-registers-live.txt"
CAPTURE_WINDOWS: tuple[str, ...] = ("cvisp +0x8000", "cvisp +0x4000", "cvisp +0x4400",
                                    "cvisp +0x4600", "cvisp +0x4700")

# The mode the recovered configuration was captured at, and the only one the
# driver configures today (ar-cvisp.c AR_CVISP_WIDTH / AR_CVISP_HEIGHT).
FRAME_WIDTH: int = 1920
FRAME_HEIGHT: int = 1080

ROW_RE: re.Pattern[str] = re.compile(
    r"\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*\}([^\n]*)")


def parse_table(text: str, name: str) -> list[tuple[int, int]]:
    """The (offset, value) rows of one ar_cvisp_* table, in write order."""
    body = re.search(rf"ar_cvisp_{name}\[\]\s*=\s*\{{(.*?)\n\}};", text, re.S)
    if body is None:
        sys.exit(f"{DEFAULTS.name}: no ar_cvisp_{name} table")

    return [(int(off, 16), int(val, 16))
            for off, val, _ in ROW_RE.findall(body.group(1))]


def parse_library() -> dict[int, tuple[int, bool]]:
    """Every library register as offset -> (value, is_geometry)."""
    text: str = LIBRARY.read_text()
    body = re.search(r"ar_cvisp_library\[\]\s*=\s*\{(.*?)\n\};", text, re.S)
    if body is None:
        sys.exit(f"{LIBRARY.name}: no ar_cvisp_library table")

    out: dict[int, tuple[int, bool]] = {}
    for off, val, trailer in ROW_RE.findall(body.group(1)):
        out[int(off, 16)] = (int(val, 16), "geom" in trailer)

    return out


def geometry_words() -> set[int]:
    """
    Every 32-bit word the frame dimensions can spell.

    The block stores geometry three ways: a bare dimension, the two dimensions
    packed high/low, and the same at the half resolution the chroma path runs at.
    A value is driver-derivable when it is one of these, which is what lets the
    driver compute it from the mode instead of carrying the recorded number.
    """
    words: set[int] = set()
    for width, height in ((FRAME_WIDTH, FRAME_HEIGHT), (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)):
        words |= {width, height, (height << 16) | width, (width << 16) | height,
                  (width << 16) | width, (height << 16) | height}

    return words


# Vendor DRAM addresses. The image carries the vendor's own allocation; the driver
# overwrites all four from ar_cvisp_arm_buffer as soon as the node streams, so the
# carried value only survives on the idle ring path.
VENDOR_DRAM: dict[int, str] = {
    0x8098: "ring plane base, overwritten by ar_cvisp_arm_buffer",
    0x809c: "ring plane stride, paired with the base above",
    0x8174: "ring plane base, overwritten by ar_cvisp_arm_buffer",
    0x8194: "ring plane base, overwritten by ar_cvisp_arm_buffer",
}

# Registers where the trace and the library image disagree and the disagreement is
# NOT understood well enough to compute the value. Each entry says what is known.
# Reducing this list is the remaining CVISP reverse-engineering work.
# Seven of the nine are field +4 of an eight-element, stride-40 descriptor array
# based at 0x8028, one element per port. Field +0 of the same array is the frame
# geometry, packed by dvp_scaler_outlib_output_config (library 0x1f3c28) at the
# store at 0x1f3e1c, indexed 10 * port. Elements alternate full frame (0x04380780)
# and half (0x021c03c0); element 6 carries a different pair. The +4 field holds
# 0x08000b33 in every image element, and the streaming vendor ends with bit 0
# CLEAR on elements 0, 1, 4 and 5, SET on 2 and 3, and element 7 cleared outright.
# So the delta is a per-element flag in bit 0, not an off-by-one count, and the
# code that touches it is not in the geometry loop.
TRACE_RESIDUE: dict[int, str] = {
    0x8000: "output enable. The image is the pre-enable 0x00800800; setup stages "
            "0x00800802 then 0x00800806. Bits 1 and 2 are undecoded, so the final "
            "word is recorded, not computed",
    0x802c: "descriptor element 0, field +4. Bit 0 clear against the image's set",
    0x8054: "descriptor element 1, field +4. Bit 0 clear against the image's set",
    0x80cc: "descriptor element 4, field +4. Bit 0 clear against the image's set",
    0x80f4: "descriptor element 5, field +4. Bit 0 clear against the image's set",
    0x8144: "descriptor element 7, field +4, cleared outright where the image "
            "carries 0x08000b32. Whether the clear is configuration or teardown "
            "residue is open",
    0x81b4: "off the descriptor stride. The image leaves it clear and the vendor "
            "ends at 0x04000400, which is the scaler's Q10 unity phase in both "
            "halves. No writer read",
    0x81c8: "off the descriptor stride. Low byte only, 0x3a -> 0x30, so bits 1 and "
            "3 are cleared. dvp_scaler_outlib_output_config writes this register at "
            "0x1f3e48 via [x22, #456], but it clears bits at port+16 and port+24, "
            "which cannot reach the low byte, so a second writer exists",
}

# The output-descriptor scoreboard at 0x8024. dvp_scaler_outlib_output_config
# (library 0x1f3c28) reaches it three times per port, ORing 1 << port,
# 1 << (port + 8) and 1 << (port + 16); over the eight ports page 0x0000 routes,
# that fills the low three bytes. Derived, not an assumed field width.
PORT_COUNT: int = 8
PORT_SCOREBOARD: int = sum((1 << p) | (1 << (p + 8)) | (1 << (p + 16))
                           for p in range(PORT_COUNT))

BLC_BANK: range = range(0x4200, 0x4240, 4)
DIGIGAIN_BANKS: tuple[range, ...] = (range(0x4600, 0x4640, 4), range(0x4700, 0x4740, 4))
UNITY_Q8: int = 0x100


def classify(trace: dict[int, int], library: dict[int, tuple[int, bool]],
             geom: set[int]) -> tuple[dict[str, list[int]], list[tuple[int, int, int]]]:
    """
    Sort the block's final register state into provenance classes.

    Returns the classes, offset lists keyed by class name, and every register whose
    composed value disagrees with the trace. audit-provenance.py calls this too, so
    the two never drift into reporting different numbers for the same block.
    """
    classes: dict[str, list[int]] = {}
    mismatch: list[tuple[int, int, int]] = []

    for off in sorted(trace):
        want: int = trace[off]
        source: str
        got: int | None

        if off in VENDOR_DRAM:
            source, got = "vendor DRAM address", want
        elif off in TRACE_RESIDUE:
            source, got = "trace residue", want
        elif off in BLC_BANK:
            source, got = "blc constant", want
        elif any(off in bank for bank in DIGIGAIN_BANKS):
            source = "unity gain"
            got = UNITY_Q8 if want == UNITY_Q8 else 0
        elif off in library:
            value, is_geom = library[off]
            if value == want:
                source, got = "library image", value
            elif is_geom and want in geom:
                source, got = "frame geometry", want
            elif off == 0x8024 and want == PORT_SCOREBOARD:
                source, got = "port scoreboard", want
            else:
                source, got = "UNSOURCED", None
        elif want == 0:
            source, got = "zero write", 0
        elif want in geom:
            source, got = "frame geometry", want
        else:
            source, got = "UNSOURCED", None

        classes.setdefault(source, []).append(off)
        if got != want:
            mismatch.append((off, want, got))

    return classes, mismatch


def capture(path: pathlib.Path) -> dict[int, int]:
    """Every CVISP register a slot-A capture covers, as offset -> value."""
    if not path.exists():
        return {}
    words: dict[int, int] = {}
    text: str = path.read_text(errors="replace")
    for header in CAPTURE_WINDOWS:
        base: int = int(header.split("+")[1], 16)
        collecting: bool = False
        for line in text.split("\n"):
            if line.startswith(f"--- {header}"):
                collecting = True
                continue

            if collecting:
                row = re.match(r"\+0x([0-9a-f]+):\s+(.*)", line.strip())
                if row is None:
                    break

                offset: int = int(row.group(1), 16)
                for index, word in enumerate(row.group(2).split()):
                    words[base + offset + 4 * index] = int(word, 16)

    return words


def measured_section(trace: dict[int, int], residue: list[int]) -> int:
    """
    Report how much of the configuration is measured equal to the streaming vendor.

    Provenance and validation are different questions. A register can be a recording
    by rule 1, because the driver copies it rather than computing it, and still be
    proven right by rule 2, because a capture of the streaming vendor at the matched
    operating point reads the same word. Returns the count of residue registers no
    capture covers, which is the only part still resting on the recording alone.
    """
    vendor: dict[int, int] = capture(VENDOR_CAPTURE)
    ours: dict[int, int] = capture(OPEN_CAPTURE)
    if not vendor or not ours:
        print("slot-A captures absent: no measurement against the vendor available\n")
        return len(residue)

    covered: list[int] = sorted(set(vendor) & set(ours))
    differing: list[int] = [off for off in covered if vendor[off] != ours[off]]
    print(f"measured against the streaming vendor, from the slot-A captures: "
          f"{len(covered)} registers\n")
    for off in differing:
        note: str = (" (our own allocation)" if off in VENDOR_DRAM else "")
        print(f"    {off:#06x}  vendor {vendor[off]:#010x}  ours {ours[off]:#010x}{note}")

    real: list[int] = [off for off in differing if off not in VENDOR_DRAM]
    print(f"    {len(differing)} differ, {len(real)} of them outside the vendor DRAM class\n")

    seen: list[int] = [off for off in residue if off in vendor and off in ours]
    unseen: list[int] = [off for off in residue if off not in seen]
    print(f"    of the {len(residue)} recordings, {len(seen)} are covered by the captures "
          f"and read the same on both units:")

    for off in seen:
        print(f"      {off:#06x} = {vendor[off]:#010x}")

    if unseen:
        print(f"    {len(unseen)} sit outside every captured window, so they rest on the "
              f"recording alone:")
        for off in unseen:
            print(f"      {off:#06x}")

    print()
    return len(unseen)


def main() -> int:
    text: str = DEFAULTS.read_text()
    setup: list[tuple[int, int]] = parse_table(text, "setup")
    late: list[tuple[int, int]] = parse_table(text, "late")
    tick: list[tuple[int, int]] = parse_table(text, "tick")
    library: dict[int, tuple[int, bool]] = parse_library()
    geom: set[int] = geometry_words()

    # The state ar_cvisp_configure leaves behind: setup, then late, last write wins.
    # The tick group is a per-frame rewrite of the digital-gain head, so it lands on
    # top of whatever late put there and belongs in the same final state.
    trace: dict[int, int] = {}
    for off, val in setup + late + tick:
        trace[off] = val

    classes, mismatch = classify(trace, library, geom)

    total: int = len(trace)
    print(f"CVISP registers the driver leaves set: {total}\n")
    order: list[str] = ["library image", "frame geometry", "port scoreboard", "zero write",
                        "unity gain", "blc constant", "vendor DRAM address", "trace residue",
                        "UNSOURCED"]
    for name in order:
        if name in classes:
            print(f"  {len(classes[name]):5}  {name}")

    print()

    residue: list[int] = classes.get("trace residue", [])
    derived: int = (len(classes.get("library image", [])) + len(classes.get("frame geometry", []))
                    + len(classes.get("zero write", [])) + len(classes.get("unity gain", []))
                    + len(classes.get("port scoreboard", [])))
    overwritten: int = (len(classes.get("blc constant", []))
                        + len(classes.get("vendor DRAM address", [])))
    print(f"  {derived:5}  vendor-owned or driver-derived: no recording needed")
    print(f"  {overwritten:5}  carried but overwritten before they matter "
          f"(the blob's BLC pass, the node's own buffers)")
    print(f"  {len(residue):5}  still a recording: why it differs is known, "
          f"what to write instead is not")
    print()

    unmeasured: int = measured_section(trace, residue)

    if residue:
        print("trace residue, the remaining CVISP reverse-engineering work:")
        for off in residue:
            print(f"    0x{off:04x} = 0x{trace[off]:08x}  {TRACE_RESIDUE[off]}")

        print()

    failed: bool = False
    if mismatch:
        failed = True
        print(f"composition differs from the trace at {len(mismatch)} registers:")
        for off, want, got in mismatch:
            shown: str = "no source" if got is None else f"0x{got:08x}"
            print(f"    0x{off:04x}  trace 0x{want:08x}  composed {shown}")
        print()

    if "UNSOURCED" in classes:
        failed = True
        print("UNSOURCED, the value exists only in the trace:")
        for off in classes["UNSOURCED"]:
            print(f"    0x{off:04x} = 0x{trace[off]:08x}")

        print()

    if failed:
        print("The trace is still load-bearing. ar-cvisp.c cannot be switched off it yet.")
        return 1

    print("The composition reproduces the trace exactly, with nothing unsourced.")
    print(f"The CVISP replay is {len(residue)} registers, down from {total}, and of those")
    print(f"{unmeasured} rest on the recording alone; the rest read the same on the streaming")
    print("vendor and on our unit at the matched operating point.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
