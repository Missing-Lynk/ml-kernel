#!/usr/bin/env python3
"""
Prove the gamma and DRC selector bands that the vendor AE state reaches.

Gamma and DRC pages are already generated from the NT99235 tuning blob. The
remaining runtime gap is the selector: the vendor chooses which gamma curve and
which DRC profile to build from one AE scalar. This checker reads the two band
tables from the blob and reports the interval a pure selection of each pair
implies:

  session A: gamma curve 2 and DRC profile 3 share scalar interval [210, 250]
  session B: gamma curve 3 and DRC profile 4 share scalar interval [290, 330]

**Session A is superseded and is kept here only as the band arithmetic.** The
selection is not always pure: check-trigger-scalar.py fits the captured pages as
blends, and the DRC page of the session-A capture (out/au-tone-tables/pre-drc.bin)
is profile 3 blended into profile 4 at weight exactly 0.2500, which is a scalar
of exactly 275, not an interval of [210, 250]. Its gamma page agrees, fitting
curve 2 into curve 3 at 0.832 against the 0.8333 that a scalar of 275 predicts.
So the pair "curve 2 and profile 3" was a nearest-curve reading of a blend.

That correction matters for the candidate rule-out below, which is why the
verdicts there are reported per session rather than as one answer: a candidate
that only ever failed against session A was never ruled out at all.

The scalar's producer is still open work. This script makes the table side
reproducible from the blob, so a later selector formula has a fixed target and
a future blob swap has a quick compatibility check.

    kernel/scripts/isp/check-tone-selector.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        --live-heap out/au-vendor-session/heap-live.bin \\
        --bright-heap out/au-vendor-session/heap-bright.bin \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so
"""

import argparse
import pathlib
import struct
import subprocess
import sys

GAMMA_HEADER = 0x26AFC
GAMMA_COUNT = 0x26B04
GAMMA_BANDS = 0x26B0C
GAMMA_MAX = 5

DRC_HEADER = 0x17A88
DRC_COUNT = 0x17A90
DRC_BANDS = 0x17A9C
DRC_MAX = 6

HEADER_ENABLE = 0x00
HEADER_INTERP = 0x04

EXPECTED = (
    ("session A", 2, 3, (210.0, 250.0)),
    ("session B", 3, 4, (290.0, 330.0)),
)

HEAP_BASE = 0x50C000
STATE_VA = 0xED07A0
STATE_OFF = STATE_VA - HEAP_BASE
STATE_AE = STATE_OFF + 0x4000
STATE_TABLE = STATE_OFF + 0x68

CANDIDATES = (
    ("exp_index", "u32", STATE_AE + 4300),
    ("ae_lux_index", "float", STATE_AE + 4312),
    ("ae_gain_float", "float", STATE_AE + 4316),
    ("ae_gain_cached", "float", STATE_AE + 4320),
    ("ae_frame_period", "float", STATE_AE + 4324),
    ("current_luma", "float", STATE_AE + 4596),
    ("luma_comp_target", "u32", STATE_AE + 4512),
    ("settle_counter", "u16", STATE_AE + 4544),
    ("skip_countdown", "u32", STATE_AE + 4572),
    ("ae_output_gain", "float", STATE_AE + 4776),
    ("ae_output_lines", "u32", STATE_AE + 4780),
    ("ae_output_cached", "float", STATE_AE + 4784),
    ("ae_output_target", "u32", STATE_AE + 4800),
    ("abscissa_x256", "table_gain", STATE_AE + 4300),
    ("abscissa_percent", "table_gain_percent", STATE_AE + 4300),
    ("line_count", "table_lines", STATE_AE + 4300),
)

HEAP_EXPECTED = (
    ("bright heap", "session A", (210.0, 250.0)),
    ("live heap", "session B", (290.0, 330.0)),
)

LIB_CHECKS = (
    ("direct scalar trigger helper",
     0x179530, 0x1796D4,
     ("fcmpe\ts4, s0",
      "fcmpe\ts0, s3",
      "stp\tw4, w3, [x2]",
      "stp\tw0, w0, [x2]",
      "fsub\ts0, s0, s1",
      "fcsel\ts0, s1, s0, le")),
    ("CVISP set-ctl forwards AEC commands",
     0x240FD8, 0x241530,
     ("ldr\tw1, [x20, #4]",
      "mov\tw2, #0x1002",
      "cmp\tw1, w2",
      "ldr\tx4, [x19, #592]",
      "mov\tw1, #0xb16",
      "ldr\tx4, [x4, #480]",
      "blr\tx4")),
    ("CVISP get-config accepts AEC frame command",
     0x242A64, 0x242A88,
     ("cmp\tw0, #0x760",
      "b.eq\t2428c4")),
    ("CVISP get-config forwards accepted AEC commands",
     0x2428C4, 0x2428E0,
     ("ldr\tx0, [x19, #544]",
      "mov\tw3, #0x10",
      "mov\tw1, #0xb16",
      "blr\tx4")),
    ("CVISP trigger snapshot copy",
     0x2411F4, 0x241268,
     ("mov\tx2, #0xa60",
      "add\tx0, x19, #0x3a0",
      "ldr\tx4, [x21], #8",
      "add\tx3, x4, #0x688",
      "bl\t52660 <memcpy@plt>",
      "add\tx19, x19, #0xe00",
      "stp\tx6, x7, [x19]",
      "stp\tx0, x1, [x19, #48]")),
    ("CVISP broadcasts trigger snapshots",
     0x2410A0, 0x2410F8,
     ("mov\tw1, #0xb0d",
      "tbz\tw5, #0",
      "ldr\tx4, [x4, #480]",
      "blr\tx4",
      "ldr\tx4, [x19, #592]",
      "mov\tw1, #0xb0d")),
    ("ISP gamma registers its command callback",
     0x194B20, 0x194B40,
     ("add\tx0, x0, #0x350",
      "str\tx0, [x19, #480]")),
    ("ISP gamma command 0xb0d stores trigger snapshot",
     0x194388, 0x194600,
     ("cmp\tw21, #0xb0d",
      "b.eq\t1945d4",
      "ldr\tx19, [x22]",
      "add\tx0, x20, #0x244",
      "add\tx20, x20, #0x4, lsl #12",
      "mov\tx2, #0x8c",
      "add\tx1, x19, #0x9b4",
      "bl\t52660 <memcpy@plt>",
      "ldr\tw0, [x19, #2308]",
      "str\tw0, [x20, #736]")),
    ("ISP DRC registers its command callback",
     0x1A5E1C, 0x1A5E30,
     ("add\tx0, x0, #0x700",
      "stp\tx1, x0, [x19, #472]")),
    ("ISP DRC command 0xb0d stores trigger snapshot",
     0x1A573C, 0x1A5980,
     ("cmp\tw20, #0xb0d",
      "b.eq\t1a5964",
      "ldr\tx1, [x22]",
      "mov\tx2, #0x8c",
      "add\tx0, x21, #0x264",
      "add\tx1, x1, #0x9b4",
      "bl\t52660 <memcpy@plt>")),
    ("AEC module command 0x760 dispatch",
     0x239D34, 0x239D44,
     ("cmp\tw0, #0x760",
      "b.eq\t2398b4")),
    ("AEC per-frame output handoff",
     0x2398B4, 0x239970,
     ("mov\tw2, #0x4c58",
      "movk\tw2, #0x1, lsl #16",
      "mov\tw2, #0x7c",
      "bl\t514c0 <camera_map_phy_addr@plt>",
      "ldr\tx3, [x3, #24]",
      "blr\tx3",
      "mov\tw1, #0x1d",
      "ldr\tx4, [x4, #32]",
      "blr\tx4")),
    ("AEC control command 29 output readback",
     0x2642E8, 0x264300,
     ("ldr\tx0, [x19, #5200]",
      "ldr\tx1, [x0, #128]",
      "ldr\tx0, [x0, #144]",
      "stp\tx1, x0, [x2]")),
    ("helper copies trigger state",
     0x179880, 0x1798FC,
     ("ldp\tw6, w7, [x1, #28]",
      "ldr\ts0, [x1, #36]",
      "stp\tw4, w5, [x3]",
      "str\ts0, [x4]")),
    ("ISP gamma trigger records",
     0x1955C8, 0x1955E4,
     ("add\tx2, x23, #0x3a, lsl #12",
      "add\tx1, x23, #0x3a, lsl #12",
      "add\tx2, x2, #0xc98",
      "add\tx1, x1, #0x410",
      "bl\t51bc0 <is_aec_trigger_compute_user@plt>")),
    ("ISP gamma direct scalar fallback",
     0x1956A8, 0x1956BC,
     ("fmov\ts0, s8",
      "add\tx1, x1, #0xb0c",
      "bl\t52e70 <is_aec_trigger_compute@plt>")),
    ("ISP DRC trigger records",
     0x1A46B8, 0x1A46D4,
     ("add\tx2, x23, #0x3a, lsl #12",
      "add\tx1, x23, #0x3a, lsl #12",
      "add\tx2, x2, #0xb2c",
      "add\tx1, x1, #0x2a4",
      "bl\t51bc0 <is_aec_trigger_compute_user@plt>")),
    ("ISP DRC direct scalar fallback",
     0x1A47B0, 0x1A47C4,
     ("fmov\ts0, s8",
      "add\tx1, x1, #0xa9c",
      "bl\t52e70 <is_aec_trigger_compute@plt>")),
)

LIB_JUMP_CHECKS = (
    ("AEC control command 29", 0x38A2F0, 29, 0x2636CC, 0x2642E8),
)


def u32(blob: bytes, off: int) -> int:
    return struct.unpack_from("<I", blob, off)[0]


def f32(blob: bytes, off: int) -> float:
    return struct.unpack_from("<f", blob, off)[0]


def bands(blob: bytes, header: int, count_off: int, bands_off: int,
          maximum: int, name: str) -> list[tuple[float, float]]:
    """Read one selector table as [(lo, hi), ...]."""
    enable = u32(blob, header + HEADER_ENABLE)
    interp = u32(blob, header + HEADER_INTERP)
    count = u32(blob, count_off)

    if enable != 1:
        sys.exit(f"{name}: enable word at {header:#x} reads {enable}, expected 1")

    if interp != 1:
        sys.exit(f"{name}: interpolate word at {header + 4:#x} reads {interp}, "
                 "expected 1")

    if count < 1 or count > maximum:
        sys.exit(f"{name}: count word at {count_off:#x} reads {count}, "
                 f"expected 1..{maximum}")

    out: list[tuple[float, float]] = []
    for i in range(count):
        lo = f32(blob, bands_off + i * 8)
        hi = f32(blob, bands_off + i * 8 + 4)
        if hi < lo:
            sys.exit(f"{name}[{i}]: descending band {lo:g}..{hi:g}")

        if i and lo <= out[-1][1]:
            sys.exit(f"{name}[{i}]: band {lo:g}..{hi:g} overlaps previous "
                     f"{out[-1][0]:g}..{out[-1][1]:g}")

        out.append((lo, hi))

    return out


def interval_intersection(a: tuple[float, float],
                          b: tuple[float, float]) -> tuple[float, float] | None:
    lo = max(a[0], b[0])
    hi = min(a[1], b[1])
    if lo > hi:
        return None

    return lo, hi


def fmt_interval(iv: tuple[float, float]) -> str:
    return f"[{iv[0]:.0f}, {iv[1]:.0f}]"


def in_interval(value: float, interval: tuple[float, float]) -> bool:
    return interval[0] <= value <= interval[1]


def candidate_value(heap: bytes, kind: str, off: int) -> float:
    if kind == "u32":
        return float(u32(heap, off))

    if kind == "u16":
        return float(struct.unpack_from("<H", heap, off)[0])

    if kind == "float":
        return f32(heap, off)

    if kind in {"table_gain", "table_gain_percent", "table_lines"}:
        index = u32(heap, off)
        table = STATE_TABLE + index * 8
        gain = u32(heap, table)
        lines = u32(heap, table + 4)

        if kind == "table_gain":
            return float(gain)

        if kind == "table_gain_percent":
            return gain * 100.0 / 256.0

        return float(lines)

    raise ValueError(kind)


def check_heap_candidates(bright_path: pathlib.Path,
                          live_path: pathlib.Path) -> None:
    heaps = []
    for label, expected_name, interval, path in (
        (*HEAP_EXPECTED[0], bright_path),
        (*HEAP_EXPECTED[1], live_path),
    ):
        heap_path = pathlib.Path(path)
        if not heap_path.exists():
            sys.exit(f"{heap_path}: not found")

        heaps.append((label, expected_name, interval, heap_path.read_bytes()))

    print("\nmeasured AE-state scalar candidates:")
    any_match = False
    standing = []
    for name, kind, off in CANDIDATES:
        values = []
        ok = True
        for label, expected_name, interval, heap in heaps:
            value = candidate_value(heap, kind, off)
            values.append(f"{label} {value:.6g} against {expected_name} "
                          f"{fmt_interval(interval)}")
            ok = ok and in_interval(value, interval)

        # Per session, not as one verdict. Session A's interval is superseded
        # by the blend measurement in check-trigger-scalar.py, so a candidate
        # that fits session B and misses only session A is still standing.
        per_session = [in_interval(candidate_value(heap, kind, off), interval)
                       for _, _, interval, heap in heaps]
        ok = all(per_session)
        if ok:
            status = "fits"
        elif per_session[1]:
            status = "STILL STANDING, fails only the superseded session A"
        else:
            status = "ruled out"

        print(f"  {name:<17} {status}: " + "; ".join(values))
        any_match = any_match or ok
        if per_session[1]:
            standing.append(name)

    if any_match:
        sys.exit("one listed AE field fits both tone-selector intervals; "
                 "promote it only after tracing is_aec_trigger_compute_user")

    if standing:
        print(f"\nstill standing against session B, the interval the blend "
              f"measurement did not overturn: {', '.join(standing)}")
        print("Session A cannot rule anything out until a capture pairs its "
              "AE state with its own pages; check-trigger-scalar.py measures "
              "the scalar and the gain from one register sweep instead, which "
              "needs no pairing.")
    else:
        print("\nnone of the measured obvious AE fields is the tone selector "
              "scalar")


def objdump(lib: pathlib.Path, start: int, stop: int) -> str:
    try:
        return subprocess.check_output(
            ["aarch64-linux-gnu-objdump", "-d",
             f"--start-address={start:#x}",
             f"--stop-address={stop:#x}", str(lib)],
            text=True,
            stderr=subprocess.STDOUT,
        )
    except FileNotFoundError:
        sys.exit("aarch64-linux-gnu-objdump: not found")
    except subprocess.CalledProcessError as exc:
        sys.exit(exc.output)


def elf_vma_offset(blob: bytes, vma: int) -> int:
    if blob[:4] != b"\x7fELF" or blob[4] != 2 or blob[5] != 1:
        sys.exit("vendor library must be a little-endian ELF64 file")

    e_phoff = struct.unpack_from("<Q", blob, 32)[0]
    e_phentsize = struct.unpack_from("<H", blob, 54)[0]
    e_phnum = struct.unpack_from("<H", blob, 56)[0]
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        p_type = struct.unpack_from("<I", blob, off)[0]
        if p_type != 1:
            continue

        p_offset = struct.unpack_from("<Q", blob, off + 8)[0]
        p_vaddr = struct.unpack_from("<Q", blob, off + 16)[0]
        p_filesz = struct.unpack_from("<Q", blob, off + 32)[0]
        if p_vaddr <= vma < p_vaddr + p_filesz:
            return p_offset + vma - p_vaddr

    sys.exit(f"{vma:#x}: no file-backed LOAD segment")


def check_library(lib_path: pathlib.Path) -> None:
    lib = pathlib.Path(lib_path)
    if not lib.exists():
        sys.exit(f"{lib}: not found")

    blob = lib.read_bytes()

    print("\nvendor library tone-selector call sites:")
    for name, table_vma, index, anchor, expected in LIB_JUMP_CHECKS:
        off = elf_vma_offset(blob, table_vma + index * 2)
        jump = struct.unpack_from("<h", blob, off)[0]
        target = anchor + (jump << 2)
        if target != expected:
            sys.exit(f"{name}: jump table entry {index} targets {target:#x}, "
                     f"expected {expected:#x}")

        print(f"  {name}: jump table entry {index} targets {target:#x}")

    for name, start, stop, needles in LIB_CHECKS:
        text = objdump(lib, start, stop)
        missing = [needle for needle in needles if needle not in text]
        if missing:
            sys.exit(f"{name}: missing expected instruction(s): "
                     + ", ".join(missing))

        print(f"  {name}: {start:#x}..{stop:#x} matches")

    print("  AEC module command 0x760 dispatches to the frame handoff")
    print("  CVISP forwards 0xb16/0x760 payloads to the AEC command slot")
    print("  CVISP command 0xb0d snapshots and broadcasts trigger state")
    print("  ISP gamma and DRC command callbacks store the 0xb0d snapshot")
    print("  AEC frame handoff maps 0x14c58-byte input and 0x7c-byte "
          "output, calls the process slot, then calls control command 29")
    print("  direct scalar helper maps s0 plus blob bands to low/high indices")
    print("  ISP gamma current record: stage record +0x3a410")
    print("  ISP gamma cached record:  stage record +0x3ac98")
    print("  ISP DRC current record:   stage record +0x3a2a4")
    print("  ISP DRC cached record:    stage record +0x3ab2c")
    print("  selected low/high indices live at record +0x1c/+0x20; "
          "blend weight at +0x24")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tuning", required=True,
                    help="the sensor tuning blob the vendor ships")
    ap.add_argument("--live-heap",
                    help="vendor heap dump for the darker/live operating point")
    ap.add_argument("--bright-heap",
                    help="vendor heap dump for the brighter operating point")
    ap.add_argument("--lib", type=pathlib.Path,
                    help="vendor libmpp_service.so for helper/call-site checks")
    args = ap.parse_args()

    path = pathlib.Path(args.tuning)
    if not path.exists():
        sys.exit(f"{path}: not found. The tuning blob is a capture artifact "
                 "and is deliberately not in the tree.")

    blob = path.read_bytes()
    gamma = bands(blob, GAMMA_HEADER, GAMMA_COUNT, GAMMA_BANDS, GAMMA_MAX,
                  "gamma")
    drc = bands(blob, DRC_HEADER, DRC_COUNT, DRC_BANDS, DRC_MAX, "drc")
    failures = []

    print("gamma selector bands:")
    for i, iv in enumerate(gamma):
        print(f"  curve {i}: {fmt_interval(iv)}")

    print("\nDRC selector bands:")
    for i, iv in enumerate(drc):
        print(f"  profile {i}: {fmt_interval(iv)}")

    print()
    for name, gamma_index, drc_index, expected in EXPECTED:
        got = interval_intersection(gamma[gamma_index], drc[drc_index])
        if got is None:
            failures.append(f"{name}: gamma {gamma_index} and DRC {drc_index} "
                            "do not share a scalar interval")
            continue

        print(f"{name}: gamma curve {gamma_index} {fmt_interval(gamma[gamma_index])} "
              f"+ DRC profile {drc_index} {fmt_interval(drc[drc_index])} "
              f"=> {fmt_interval(got)}")
        if got != expected:
            failures.append(f"{name}: expected {fmt_interval(expected)}, got "
                            f"{fmt_interval(got)}")

    if failures:
        print()
        for line in failures:
            print(f"FAIL: {line}")

        return 1

    print("\ntone selector bands reproduce the two known vendor operating intervals")

    if bool(args.live_heap) != bool(args.bright_heap):
        sys.exit("--live-heap and --bright-heap must be passed together")

    if args.live_heap:
        check_heap_candidates(args.bright_heap, args.live_heap)

    if args.lib:
        check_library(args.lib)

    return 0


if __name__ == "__main__":
    sys.exit(main())
