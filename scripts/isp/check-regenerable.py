#!/usr/bin/env python3
"""
Regenerate every generated vendor table and diff it against the tree.

The provenance audit answers where a value came from. This answers a different
question: whether the header in the tree is still what its generator produces
from the vendor library and the tuning file today. A header that was generated
once and then hand-edited reads as sourced in the audit while no longer being
reproducible, and nothing else in the tree would notice.

Generators that need an MMIO trace are excluded and named in the report: their
output is a recording of a running device by construction, which is the thing
the derived tables exist to replace.

Usage: kernel/scripts/isp/check-regenerable.py [--lib PATH] [--blob PATH]
"""

import argparse
import difflib
import pathlib
import subprocess
import sys
import tempfile

HERE: pathlib.Path = pathlib.Path(__file__).resolve().parent
TABLES: pathlib.Path = HERE.parent.parent / "overlay/drivers/media/artosyn/vendor-tables"
ROOT: pathlib.Path = HERE.parent.parent.parent

DEFAULT_LIB: pathlib.Path = ROOT / "out/air-gather/vendor-root/usr/lib/libmpp_service.so"
DEFAULT_BLOB: pathlib.Path = ROOT / "out/air-gather/camera/nt99235_tuning_preview_fpv.bin"

# header -> (generator, argument style, extra arguments)
#   "stdout"  the generator prints the header
#   "output"  the generator takes -o
#   "check"   the generator compares against the tree itself
GENERATED: dict[str, tuple[str, str, list[str]]] = {
    "ar-isp-blob.h":       ("gen-blob-header.py",   "check",  []),
    "ar-isp-rgb2yuv.h":    ("gen-rgb2yuv.py",       "check",  ["--lib"]),
    "ar-isp-ccm-init.h":   ("gen-ccm.py",           "stdout", ["--lib", "--blob"]),
    "ar-isp-compander.h":  ("gen-compander.py",     "stdout", ["--lib"]),
    "ar-isp-drc-tail.h":   ("gen-drc-tail.py",      "stdout", ["--lib"]),
    "ar-isp-gamma-page1.h": ("gen-gamma-page1.py",  "stdout", ["--lib"]),
    "ar-isp-library.h":    ("gen-isp-library.py",   "output", ["--lib"]),
    "ar-cvisp-library.h":  ("gen-cvisp-library.py", "output", ["--lib"]),
    "ar-cvisp-derived.h":  ("gen-cvisp-setup.py",   "output", ["--lib"]),
    "ar-isp-defaults.h":   ("gen-isp-defaults.py",  "prefix", ["--lib", "--trace"]),
}

# The MMIO write trace gen-isp-defaults.py needs. Which trace is used changes the
# output, so the one the checked-in header was generated from is named here rather
# than left to whichever file a glob happens to find first.
ISP_TRACE: pathlib.Path = ROOT / "out/au-mmiotrace/mmio-combined.log"

# ar-isp-defaults.h carries one block the generator does not emit: ar_isp_vendor_trim,
# a correction pass measured on hardware and added by hand, documented as measured in
# the header itself. The tree's copy must differ from the generator's output by exactly
# one contiguous insertion, and that insertion must be this block. Requiring a single
# region keeps the hand edit bounded and visible rather than letting further drift hide
# behind it.
INSERTED_BLOCK: str = "ar_isp_vendor_trim"

# Generated from an MMIO write trace and no longer included by any source file: the
# derived table replaced it and only comments still name it.
TRACE_SOURCED: dict[str, str] = {
    "ar-cvisp-defaults.h": "gen-cvisp-defaults.py, trace-sourced, no longer compiled in",
}

# Written by hand, with no generator. Listed so the count adds up.
HAND_WRITTEN: dict[str, str] = {
    "ar-isp-gates.h": "stage gates, recovered per bit and written by hand",
}


def run(args: list[str]) -> tuple[int, str]:
    done = subprocess.run(args, capture_output=True, text=True)
    return done.returncode, done.stdout


def check_one(header: str, gen: str, style: str, extra: list[str],
              lib: pathlib.Path, blob: pathlib.Path) -> tuple[str, str]:
    """Returns (verdict, detail). Verdict is MATCH, DRIFT, or ERROR."""
    script: pathlib.Path = HERE / gen
    if not script.exists():
        return "ERROR", f"{gen} is missing"

    args: list[str] = [sys.executable, str(script)]
    for flag in extra:
        if flag == "--lib":
            args += [flag, str(lib)]
        elif flag == "--trace":
            args += [flag, str(ISP_TRACE)]
        else:
            args += [flag, str(blob)]

    if style == "check":
        args.append("--check")
        code, out = run(args)
        return ("MATCH", "generator's own --check passed") if code == 0 else \
               ("DRIFT", (out.strip().split("\n") or [""])[-1][:120])

    target: pathlib.Path = TABLES / header
    if not target.exists():
        return "ERROR", f"{header} is not in the tree"

    if style == "prefix":
        if not ISP_TRACE.exists():
            return "ERROR", f"trace missing: {ISP_TRACE}"

        with tempfile.NamedTemporaryFile(suffix=".h", delete=False) as tmp:
            path = pathlib.Path(tmp.name)

        code, _ = run(args + ["-o", str(path)])
        produced = path.read_text() if path.exists() else ""
        path.unlink(missing_ok=True)
        if code != 0:
            return "ERROR", f"{gen} exited {code}"

        generated: list[str] = produced.splitlines()
        tree: list[str] = target.read_text().splitlines()
        edits = [op for op in difflib.SequenceMatcher(None, generated, tree,
                                                      autojunk=False).get_opcodes()
                 if op[0] != "equal"]
        if len(edits) != 1 or edits[0][0] != "insert":
            return "DRIFT", (f"{len(edits)} differing regions against the generator, "
                             f"expected one insertion")

        _, _, _, lo, hi = edits[0]
        block: list[str] = tree[lo:hi]
        if not any(INSERTED_BLOCK in line for line in block):
            return "DRIFT", f"the inserted block is not {INSERTED_BLOCK}"

        rows: int = sum(1 for line in block if "{ 0x" in line)
        return "PREFIX", (f"identical but for one inserted block, {rows} measured "
                          f"{INSERTED_BLOCK} rows")

    if style == "output":
        with tempfile.NamedTemporaryFile(suffix=".h", delete=False) as tmp:
            path = pathlib.Path(tmp.name)

        code, _ = run(args + ["-o", str(path)])
        produced: str = path.read_text() if path.exists() else ""
        path.unlink(missing_ok=True)
    else:
        code, produced = run(args)

    if code != 0:
        return "ERROR", f"{gen} exited {code}"

    if produced == target.read_text():
        return "MATCH", f"byte-identical, {len(produced.splitlines())} lines"

    a: list[str] = target.read_text().splitlines()
    b: list[str] = produced.splitlines()
    differing: int = sum(1 for x, y in zip(a, b) if x != y) + abs(len(a) - len(b))
    return "DRIFT", f"{differing} of {max(len(a), len(b))} lines differ"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lib", type=pathlib.Path, default=DEFAULT_LIB)
    parser.add_argument("--blob", type=pathlib.Path, default=DEFAULT_BLOB)
    args = parser.parse_args()

    for path, what in ((args.lib, "vendor library"), (args.blob, "tuning file")):
        if not path.exists():
            print(f"{what} missing: {path}")
            return 2

    print(f"library {args.lib.name}, tuning {args.blob.name}\n")

    results: list[tuple[str, str, str]] = []
    for header, (gen, style, extra) in sorted(GENERATED.items()):
        verdict, detail = check_one(header, gen, style, extra, args.lib, args.blob)
        results.append((header, verdict, detail))
        print(f"  {verdict:6}  {header:24} {detail}")

    print()

    for header, why in sorted(TRACE_SOURCED.items()):
        print(f"  TRACE   {header:24} {why}")

    for header, why in sorted(HAND_WRITTEN.items()):
        print(f"  HAND    {header:24} {why}")

    prefix: list[tuple[str, str, str]] = [r for r in results if r[1] == "PREFIX"]
    drift: list[tuple[str, str, str]] = [r for r in results if r[1] == "DRIFT"]
    error: list[tuple[str, str, str]] = [r for r in results if r[1] == "ERROR"]
    match: list[tuple[str, str, str]] = [r for r in results if r[1] == "MATCH"]

    print(f"\n  {len(match)} of {len(results)} generated headers reproduce byte-identically")
    for header, _, detail in prefix:
        print(f"  {header} reproduces except for a bounded hand-appended block: {detail}")

    print(f"  {len(TRACE_SOURCED)} trace-sourced and no longer compiled in")
    print(f"  {len(HAND_WRITTEN)} hand-written with no generator")

    if error:
        print("\nthe generator could not be run, so the header is unverified:")
        for header, _, detail in error:
            print(f"    {header}: {detail}")

    if drift:
        print("\nthe tree no longer matches what the generator produces:")
        for header, _, detail in drift:
            print(f"    {header}: {detail}")

        print("\nA header that drifted still carries sourced values, but it is no longer")
        print("reproducible: regenerating it would change the driver. Re-derive or record")
        print("the edit in the generator.")

        return 1

    return 1 if error else 0


if __name__ == "__main__":
    sys.exit(main())
