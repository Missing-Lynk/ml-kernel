#!/usr/bin/env python3
"""
Prove the shipped C gain-ladder headers against the Python models beside them.

Every check-*-ladder.py restates one stage's arithmetic in Python and proves
that restatement against captured register state. That proves the recovery.
It does not prove the driver: the kernel runs the C in
overlay/drivers/media/artosyn/ar-isp-*.h, and a divergence between the two is
invisible to a check that only ever runs the Python.

This closes that gap. ladder-dump.c includes the five stage headers unmodified,
runs each stage's from_blob at one abscissa and prints what the applier would
write; this script builds it with the host compiler, runs it beside the Python
models at a spread of abscissae, and refuses to pass on any disagreement.

The abscissae are chosen to exercise the shapes the arithmetic can differ in
rather than to sample evenly: the cold band, a verbatim band with no blend, the
blended interior where the Q24 fraction is non-zero, and both clamps.

lnr is compared from a zero seed. It packs fields into a bank the applier read
back first, so its output is only defined relative to a seed; both sides use the
same one, which still exercises every field it writes.

The tuning file is proprietary and is not in the repository.

    kernel/scripts/isp/check-ladder-c.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import importlib.util
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVERS = os.path.normpath(os.path.join(HERE, "..", "..", "overlay",
                                        "drivers", "media", "artosyn"))

# Cold band, verbatim interior, blended interior, and past both ends of every
# ladder so the clamps are compared too.
ABSCISSAE = (1, 256, 512, 700, 1024, 2048, 3200, 4096, 8192, 65535)


def load(name):
    """Import a check script by file name; the hyphens rule out a plain import."""
    path = os.path.join(HERE, name)
    spec = importlib.util.spec_from_file_location(name.replace("-", "_")[:-3],
                                                  path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def build(out):
    """Compile ladder-dump.c against the driver headers with the host compiler."""
    cc = os.environ.get("CC", "cc")
    cmd = [cc, "-O2", "-Wall", "-Wextra", "-I", DRIVERS, "-o", out,
           os.path.join(HERE, "ladder-dump.c")]

    try:
        subprocess.run(cmd, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        sys.exit(f"building ladder-dump failed: {exc}")

    return out


def run(binary, tuning, gain):
    """The C output for one abscissa, as {stage: [value, ...]} in emit order."""
    proc = subprocess.run([binary, tuning, str(gain)], check=True,
                          capture_output=True, text=True)
    out = {}

    for line in proc.stdout.splitlines():
        stage, index, value = line.split()
        out.setdefault(stage, []).append(int(value, 16))

        if len(out[stage]) != int(index) + 1:
            sys.exit(f"{stage}: out-of-order index {index} from ladder-dump")

    return out


def de3d_masks(module):
    """The per-register masks the de3d applier writes under."""
    return [m for _, m in module.REGS]


def compare(gain, stage, c_values, py_values, failures):
    """Record every position where the C and the Python disagree."""
    if len(c_values) != len(py_values):
        failures.append(f"gain {gain} {stage}: C emitted {len(c_values)} "
                        f"registers, Python {len(py_values)}")
        return

    for i, (c, py) in enumerate(zip(c_values, py_values)):
        if c != py & 0xFFFFFFFF:
            failures.append(f"gain {gain} {stage}[{i}]: C {c:#010x} != "
                            f"Python {py & 0xFFFFFFFF:#010x}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tuning", required=True, help="sensor tuning file")
    args = ap.parse_args()

    with open(args.tuning, "rb") as f:
        blob = f.read()

    rnr = load("check-rnr-ladder.py")
    lnr = load("check-lnr-ladder.py")
    de3d = load("check-de3d-ladder.py")
    cfa = load("check-cfa-ladder.py")
    cnf = load("check-cnf-ladder.py")

    failures = []
    compared = 0

    with tempfile.TemporaryDirectory() as tmp:
        binary = build(os.path.join(tmp, "ladder-dump"))

        for gain in ABSCISSAE:
            q16 = gain << 8
            got = run(binary, args.tuning, gain)

            compare(gain, "rnr", got["rnr"], rnr.rnr_from_blob(blob, q16),
                    failures)
            compare(gain, "rnr_tail", got["rnr_tail"],
                    rnr.tail_from_blob(blob, q16), failures)

            seed = [0] * len(got["lnr"])
            compare(gain, "lnr", got["lnr"],
                    lnr.lnr_from_blob(blob, seed, q16)[0], failures)

            compare(gain, "de3d", got["de3d"],
                    [v & m for v, m in zip(de3d.de3d_from_blob(blob, q16),
                                           de3d_masks(de3d))],
                    failures)

            compare(gain, "cfa", got["cfa"],
                    [v for _, v in cfa.cfa_from_blob(blob, q16)], failures)

            strength = cnf.strength(blob, q16)
            compare(gain, "cnf_strength", got["cnf_strength"], [strength],
                    failures)
            compare(gain, "cnf", got["cnf"],
                    [cnf.pack(strength),
                     cnf.norm_pack(strength) | cnf.NORM_A_BIT,
                     cnf.norm_pack(cnf.NORM_CONST_B)], failures)

            compared = sum(len(v) for v in got.values())

    for line in failures:
        print(line)

    if failures:
        return 1

    print(f"the shipped C headers agree with the Python models at all "
          f"{len(ABSCISSAE)} abscissae, {compared} registers each")

    return 0


if __name__ == "__main__":
    sys.exit(main())
