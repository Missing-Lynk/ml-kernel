#!/usr/bin/env python3
"""
Check the ISP curve closed forms against the vendor library's own tables.

gen-gamma-page1.py and gen-drc-tail.py compute their headers from isp_curves.py rather
than copying bytes, which means a mistyped constant can produce a wrong header where a
byte copy could not. This is the check that closes that gap: it unpacks the library's
prepacked tables and compares them sample by sample against the same functions the
generators call. It is a precondition for those generators, not an optional extra.

The compander is carried rather than computed. Its four decoded subregions are checked
exactly here, so the shapes recorded in ar-isp-compander.h stay backed by something that
runs. The whole page remains carried: several samples are still unexplained corrections
rather than recovered producer arithmetic.

The unpacking below is the vendor's, mirrored from ar-isp-codec.h. It lives here because
the check is the only thing that needs it: the generators emit decoded samples and the
driver packs them again on the way to the hardware.

The library is proprietary and not in the repository; supply it with --lib.

    kernel/scripts/isp/check-table-closed-forms.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so
"""

import argparse
import pathlib
import struct
import sys
from collections.abc import Sequence

import arlib
import isp_curves

ROOT: pathlib.Path = pathlib.Path(__file__).resolve().parent.parent.parent.parent
DEFAULT_LIB: pathlib.Path = ROOT / "out/air-gather/vendor-root/usr/lib/libmpp_service.so"

# Gamma page 1: two identical 0x800 pages, 128 records of four twelve-bit samples.
GAMMA_VMA = 0x47E1A0
GAMMA_PAGE = 0x800
GAMMA_RECORDS = 128
GAMMA_SAMPLE_MASK = 0xFFF
GAMMA_SAMPLE_SHIFTS = ((0, 0), (0, 12), (1, 16), (2, 8))
GAMMA_FORWARD_SHIFT = 20

# DRC: the 0x2000 template the ISP-init descriptor array loads, of which the tail is the
# second half. 128 records of three overlapping 20-bit lanes, the middle one duplicated.
DRC_VMA = 0x467460
DRC_LEN = 0x2000
DRC_BANK_OFFSETS = (0x1000, 0x1800)
DRC_RECORDS = 128

# Compander: entry 6 of the ISP-init template array. The curve region holds 16-bit
# samples as overlapping pairs, one pair per word, so 256 words carry 257 samples.
COMPANDER_ENTRY = 6
COMPANDER_SIZE = 0x7800
COMPANDER_HEAD_OFF = 0x000
COMPANDER_HEAD_RECORDS = 16
COMPANDER_CURVE_OFF = 0x400
COMPANDER_CURVE_WORDS = 256
COMPANDER_RAMP_OFF = 0x800
COMPANDER_RAMP_WORDS = 32
COMPANDER_MID_OFF = 0x1000


def unpack_gamma_page(page: bytes) -> tuple[list[int], int]:
    """The page's 512 samples and the forward field the last record mirrors."""
    samples: list[int] = []
    for record in range(GAMMA_RECORDS):
        words = struct.unpack_from("<4I", page, record * 16)
        samples.extend((words[w] >> shift) & GAMMA_SAMPLE_MASK
                       for w, shift in GAMMA_SAMPLE_SHIFTS)

    last = struct.unpack_from("<I", page, (GAMMA_RECORDS - 1) * 16 + 8)[0]

    return samples, (last >> GAMMA_FORWARD_SHIFT) & GAMMA_SAMPLE_MASK


def unpack_triples(data: bytes, offset: int, records: int, name: str) -> list[int]:
    """
    A 20-bit overlapping-triple table: one DRC bank, or the compander's mid span.

    Records overlap by one sample and carry a second copy of their middle lane. Both
    redundancies are checked: they are the cheapest evidence that the layout still holds.
    """
    samples: list[int] = []
    for record in range(records):
        at = offset + record * 16
        word0, word1 = struct.unpack_from("<II", data, at)
        half = struct.unpack_from("<H", data, at + 8)[0]
        lane0 = word0 & isp_curves.DRC_LANE_MAX
        lane1 = ((word0 >> 20) & 0xFFF) | ((word1 & 0xFF) << 12)
        lane2 = ((word1 >> 28) & 0xF) | (half << 4)
        duplicate = (word1 >> 8) & isp_curves.DRC_LANE_MAX

        if lane1 != duplicate:
            sys.exit(f"{name} record {record}: overlap 0x{lane1:x} != 0x{duplicate:x}")

        if record == 0:
            samples.extend((lane0, lane1, lane2))
            continue

        if lane0 != samples[-1]:
            sys.exit(f"{name} record {record}: curve break 0x{lane0:x} "
                     f"!= 0x{samples[-1]:x}")

        samples.extend((lane1, lane2))

    return samples


def unpack_compander_pairs(page: bytes, offset: int, words_len: int,
                           name: str) -> list[int]:
    """A compander 16-bit overlapping-pair table."""
    words = struct.unpack_from(f"<{words_len}I", page, offset)
    samples: list[int] = [words[0] & 0xFFFF]
    for index, word in enumerate(words):
        if (word & 0xFFFF) != samples[-1]:
            sys.exit(f"{name} word {index}: overlap 0x{word & 0xFFFF:x} "
                     f"!= 0x{samples[-1]:x}")

        samples.append((word >> 16) & 0xFFFF)

    return samples


def compare(name: str, vendor: Sequence[int], model: Sequence[int],
            tolerance: int = 0) -> int:
    """Report one table and return how many samples miss by more than `tolerance`."""
    if len(vendor) != len(model):
        print(f"{name:<16} FAIL  {len(vendor)} vendor samples, {len(model)} computed")
        return max(len(vendor), len(model))

    deviations = [abs(a - b) for a, b in zip(vendor, model, strict=True)]
    over = [i for i, d in enumerate(deviations) if d > tolerance]
    exact = sum(1 for d in deviations if d == 0)
    worst = max(deviations) if deviations else 0
    verdict = "FAIL" if over else "ok"
    note = f", within {tolerance}" if tolerance else ""
    print(f"{name:<16} {verdict:<5} {exact}/{len(vendor)} exact, "
          f"max deviation {worst}{note}")
    for i in over[:8]:
        print(f"  sample {i}: vendor {vendor[i]}, computed {model[i]}")

    return len(over)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lib", default=str(DEFAULT_LIB), help="vendor libmpp_service.so")
    args = ap.parse_args()

    path = pathlib.Path(args.lib)
    if not path.exists():
        sys.exit(f"{path}: not found; pass --lib")

    lib = path.read_bytes()
    bad = 0

    block = arlib.lib_slice(lib, GAMMA_VMA, 2 * GAMMA_PAGE, f"{path}: gamma page 1")
    if block[:GAMMA_PAGE] != block[GAMMA_PAGE:]:
        sys.exit(f"{path}: the two halves at 0x{GAMMA_VMA:x} differ; the layout has changed")

    samples, forward = unpack_gamma_page(block[:GAMMA_PAGE])
    bad += compare("gamma page1", samples, isp_curves.gamma_page1_samples())
    tail = isp_curves.gamma_page1_tail()
    if forward == tail:
        print(f"{'gamma tail':<16} {'ok':<5} {forward}, the transfer function at full scale")
    else:
        print(f"{'gamma tail':<16} {'FAIL':<5} vendor {forward}, computed {tail}")
        bad += 1

    template = arlib.lib_slice(lib, DRC_VMA, DRC_LEN, f"{path}: DRC template")
    models = (isp_curves.drc_tail_bank0(), isp_curves.drc_tail_bank1())
    for bank, (offset, model) in enumerate(zip(DRC_BANK_OFFSETS, models, strict=True)):
        bad += compare(f"drc tail bank{bank}",
                       unpack_triples(template, offset, DRC_RECORDS,
                                      f"drc tail bank{bank}"), model)

    page = arlib.template_payload(lib, COMPANDER_ENTRY, COMPANDER_SIZE, str(path))
    bad += compare("compander head", unpack_triples(
                       page, COMPANDER_HEAD_OFF, COMPANDER_HEAD_RECORDS,
                       "compander head"),
                   isp_curves.compander_head_ramp())
    bad += compare("compander curve", unpack_compander_pairs(
                       page, COMPANDER_CURVE_OFF, COMPANDER_CURVE_WORDS,
                       "compander curve"),
                   isp_curves.compander_curve())
    bad += compare("compander ramp", unpack_compander_pairs(
                       page, COMPANDER_RAMP_OFF, COMPANDER_RAMP_WORDS,
                       "compander ramp"),
                   isp_curves.compander_ramp())
    bad += compare("compander mid", unpack_triples(
                       page, COMPANDER_MID_OFF, DRC_RECORDS, "compander mid"),
                   isp_curves.compander_mid())

    if bad:
        print(f"\nFAIL: {bad} samples disagree with the vendor library")
        return 1

    print("\nevery computed sample matches the vendor library")
    return 0


if __name__ == "__main__":
    sys.exit(main())
