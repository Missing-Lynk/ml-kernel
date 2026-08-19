#!/usr/bin/env python3
"""Compare the tone pages a running unit published against the ones the builders produce.

gamma and DRC are the only configured stages with no register readback: they are DMA tables, so
`ladder_banks` cannot reach them and the published bytes are otherwise unobservable. That leaves
one link in the chain untested on hardware, which is whether a rebuilt page actually lands in the
buffer the ISP fetches when the trigger scalar moves.

The `gamma_page` and `drc_page` debugfs nodes close it. Pull both, together with the scalar the
unit was running, and this rebuilds the same pages host-side with `tone-dump.c` (the shipped
selector and packers, compiled unmodified out of the driver directory) and compares byte for byte.

    # on the unit
    cat /sys/module/ar_isp/parameters/tone_scalar          # the scalar to pass below
    cat /sys/kernel/debug/ar-isp/gamma_page > gamma.bin
    cat /sys/kernel/debug/ar-isp/drc_page   > drc.bin

    check-tone-page-live.py --tuning nt99235.bin --scalar-q8 74496 \\
        --gamma gamma.bin --drc drc.bin

A pinned unit (`tone_scalar` -1) publishes from `gamma_curve`/`drc_profile` instead, so pass
`--scalar-q8` the value those pins correspond to, or run the unit with the scalar set.

The DRC page is the strong half. Its dynamic banks carry the tuning file's own 20-bit samples with
no transform in between, so a byte-exact match proves the selector, the blend weight, the packer
and the publish path together. Gamma decimates and carries a small residual of its own against a
captured vendor page; against our own builder it is expected to be exact, because both sides are
the same code.
"""

import argparse
import os
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
DRIVERS = HERE.parents[1] / "overlay" / "drivers" / "media" / "artosyn"

# The coherent allocations the driver publishes, from ar-isp-codec.h.
GAMMA_SIZE = 0x4000
DRC_SIZE = 0x2000


def build_tone_dump(out: pathlib.Path) -> pathlib.Path:
    """Compile tone-dump.c, the shipped selector and page builders host-side."""
    cc = os.environ.get("CC", "cc")
    cmd = [cc, "-O2", "-Wall", "-Wextra", "-I", str(DRIVERS), "-o", str(out),
           str(HERE / "tone-dump.c")]

    try:
        subprocess.run(cmd, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        sys.exit(f"building tone-dump failed: {exc}")

    return out


def run_tone_dump(binary: pathlib.Path, tuning: pathlib.Path, scalar_q8: int,
                  tmp: str) -> tuple[bytes, bytes, list[int]]:
    """Both pages the driver would build at one scalar, plus its selection."""
    gamma = os.path.join(tmp, "gamma.bin")
    drc = os.path.join(tmp, "drc.bin")
    proc = subprocess.run([str(binary), str(tuning), str(scalar_q8), gamma, drc],
                          check=True, capture_output=True, text=True)
    pick = [int(v) for v in proc.stdout.split()]

    return (pathlib.Path(gamma).read_bytes(),
            pathlib.Path(drc).read_bytes(), pick)


def compare(name: str, live: bytes, want: bytes, expect_size: int) -> int:
    """Report one page. Returns the number of differing bytes."""
    if len(live) != expect_size:
        print(f"{name}: device page is {len(live)} bytes, expected {expect_size}. "
              f"A short read means the node was truncated, not that the page is wrong.")

        return max(len(live), expect_size)

    # tone-dump writes only the region it builds; the rest of the allocation is carried.
    n = min(len(live), len(want))
    diff = [i for i in range(n) if live[i] != want[i]]

    if not diff:
        print(f"{name}: {n} bytes compared, byte-exact")

        return 0

    first = diff[0]
    print(f"{name}: {len(diff)} of {n} bytes differ, first at 0x{first:04x} "
          f"(device 0x{live[first]:02x}, built 0x{want[first]:02x})")

    runs: list[tuple[int, int]] = []

    for i in diff:
        if runs and i == runs[-1][1] + 1:
            runs[-1] = (runs[-1][0], i)
        else:
            runs.append((i, i))

    print(f"          {len(runs)} run(s); first few: "
          + ", ".join(f"0x{a:04x}..0x{b:04x}" for a, b in runs[:6]))

    return len(diff)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tuning", required=True, type=pathlib.Path)
    ap.add_argument("--scalar-q8", required=True, type=int,
                    help="the tone_scalar the unit was running, in Q8")
    ap.add_argument("--gamma", type=pathlib.Path, help="device gamma_page dump")
    ap.add_argument("--drc", type=pathlib.Path, help="device drc_page dump")
    args = ap.parse_args()

    if not args.gamma and not args.drc:
        sys.exit("nothing to compare: pass --gamma, --drc or both")

    if args.scalar_q8 < 0:
        sys.exit("the unit was pinned (tone_scalar -1); pass the scalar its pins correspond to")

    with tempfile.TemporaryDirectory() as tmp:
        binary = build_tone_dump(pathlib.Path(tmp) / "tone-dump")
        built_gamma, built_drc, pick = run_tone_dump(binary, args.tuning,
                                                     args.scalar_q8, tmp)

    print(f"scalar {args.scalar_q8 >> 8} (q8 {args.scalar_q8}), selection {pick}\n")

    bad = 0

    if args.gamma:
        bad += compare("gamma", args.gamma.read_bytes(), built_gamma, GAMMA_SIZE)

    if args.drc:
        bad += compare("drc  ", args.drc.read_bytes(), built_drc, DRC_SIZE)

    print()
    print("VERDICT: " + ("the published pages are the ones the builders produce"
                         if not bad else
                         "a published page differs from the builder's, read above"))

    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
