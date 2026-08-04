#!/usr/bin/env python3
"""Extract the Artosyn ISP per-submodule default register blocks from the vendor
libmpp_service.so and emit them as a kernel header.

The vendor library carries the static half of the ISP configuration as
per-submodule default register blocks in its data segment, each present as
several byte-identical copies. Blocks are located by matching the first-pass
values of a register page, taken from an MMIO write trace of the streaming
vendor, as an ordered run.

The library is proprietary and is not in the repository. Supply it with --lib.
The generated header is checked in, so this script is only rerun when the block
map changes.

Blocks are cross-checked: a register is emitted only inside the run where every
copy of the block agrees. Registers outside the traced span but inside that run
are recovered values, which the trace could not show because the vendor pushes
its shadow image with a write-only-if-changed primitive (isp_memcpy_bycmp).
"""

import argparse
import hashlib
import re
import struct
import sys
from collections.abc import Iterator, Sequence

# Page number -> file offsets of each copy of that page's default block. The
# offset is that of the first register the trace wrote on the page, which is
# register 0 of the page for every block found so far.
BLOCKS = {
    0x08: (0x4571a0, 0x4712f0, 0x48b780),
    0x0c: (0x45a260, 0x4743b0, 0x491840),
    0x28: (0x4646b0,),
    0x2e: (0x4635d0, 0x47da70),
    0x30: (0x459460, 0x4735b0),
    0x34: (0x457100, 0x471250, 0x48b6e0),
    0x38: (0x4570b0, 0x471200, 0x48b690),
    0x3d: (0x440368, 0x463548, 0x47d9e8),
    0x50: (0x4594d0, 0x473620, 0x48fd60),
    0x58: (0x44afe0, 0x46e130, 0x4885c0),
    0x60: (0x457400, 0x471550, 0x48b9e0),
    0x65: (0x457368, 0x4714b8, 0x48b948),
    0x6c: (0x459510, 0x473660, 0x48fda0),
    0x6d: (0x459610, 0x473760, 0x48fea0),
    0x74: (0x459bc0, 0x473d10, 0x490480),
    0x75: (0x459cc0, 0x473e10, 0x490580),
    0x76: (0x462b90, 0x47d030),
}

# The output-stage pair the setup table corrects only past entry 1773, which is
# beyond the prefix the camera harness applies. Emitted as their own table so the
# driver can write them unconditionally. Values come from the trace; the choice of
# these two registers is a hardware bisect, recorded in the emitted comment.
OUTPUT_FIX_REGS = (0x2E2C, 0x2E30)

# sha256 of the air-unit libmpp_service.so the block map was derived from.
LIB_SHA256 = '4cfc8e6cfb42d8c821137993b95b152f1aaad7c53ce425e6a0493c4dd453936c'

REGS_PER_PAGE = 64
REG_BYTES = 4
PAGE_BYTES = REGS_PER_PAGE * REG_BYTES
PAGE_SHIFT = PAGE_BYTES.bit_length() - 1

ISP_BASE = 0x08C00000

# Span of the ISP window the block map was derived against. The trace prints every
# physical address as eight zero-padded hex digits, so this is exactly the set whose
# text form starts "0x08c". The block itself is 2 MiB.
ISP_TRACE_SPAN = 0x100000

# Register offset within the block. Every page in BLOCKS is well inside 16 bits.
REG_OFF_MASK = 0xFFFF

# A trace line is "wNNNNNN wWW pa=0xADDR val=0xVAL"; anything shorter is not a write record.
TRACE_FIELDS = 4

# Capture sections holding ISP registers are named "isp-<page>".
ISP_SECTION = 'isp-'
ISP_SECTION_LEN = len(ISP_SECTION)

# Characters of the library sha256 recorded in the generated header.
DIGEST_CHARS = 32


def isp_writes(path: str, stop: int) -> Iterator[tuple[int, int]]:
    """(offset, value) for every ISP write of the setup phase, in trace order."""
    for line in open(path):
        fields = line.split()
        if len(fields) < TRACE_FIELDS:
            continue

        # Traces taken with MMIOTRACE_READS also carry r-prefixed load lines. They share
        # the sequence counter, so they must be skipped rather than parsed as writes.
        if fields[0][0] != 'w':
            continue

        if int(fields[0][1:], 16) > stop:
            return

        phys = int(fields[2].split('=')[1], 16)
        if not ISP_BASE <= phys < ISP_BASE + ISP_TRACE_SPAN:
            continue

        yield phys & REG_OFF_MASK, int(fields[3].split('=')[1], 16)


def load_trace(path: str, stop: int) -> dict[int, int]:
    """First-pass value per ISP register offset during the setup phase."""
    first = {}
    for off, val in isp_writes(path, stop):
        first.setdefault(off, val)

    return first


def load_setup_writes(path: str, stop: int) -> list[tuple[int, int]]:
    """Every ISP write of the setup phase, in order, with runs collapsed.

    Order is preserved because the block carries an enable ladder and several
    arm-then-load sequences whose result depends on it. Consecutive writes of
    the same value to the same register are dropped; distinct values are not,
    since a rewrite is how the vendor arms some registers.
    """
    writes = []
    for off, val in isp_writes(path, stop):
        if writes and writes[-1] == (off, val):
            continue

        writes.append((off, val))

    return writes


def load_window(path: str) -> dict[int, int]:
    """ISP registers from an au-chain-capture.sh capture."""
    regs, section = {}, None
    for line in open(path):
        hit = re.match(r'SECTION (\S+)', line)
        if hit:
            section = hit.group(1)
            continue

        hit = re.match(r'\+0x([0-9a-f]{4}): (.*)', line)
        if hit and section and section.startswith(ISP_SECTION):
            page = int(section[ISP_SECTION_LEN:], 16)
            row_base = (page << PAGE_SHIFT) + int(hit.group(1), 16)
            for i, word in enumerate(hit.group(2).split()):
                if re.fullmatch(r'[0-9a-f]{8}', word):
                    regs[row_base + REG_BYTES * i] = int(word, 16)

    return regs


def load_trim(vendor_path: str | None,
              our_path: str | None) -> list[tuple[int, int]]:
    """Registers whose live value on the working device differs from ours.

    Live against live, which is the only valid comparison: our intended value is
    not a baseline, and a register can legitimately read back something else.
    """
    if not vendor_path or not our_path:
        return []

    vendor = load_window(vendor_path)
    ours = load_window(our_path)
    return [(off, vendor[off]) for off in sorted(set(vendor) & set(ours))
            if vendor[off] != ours[off]]


def agree_span(lib: bytes, bases: Sequence[int], lo: int,
               hi: int) -> tuple[int, int, bool]:
    """Widest [start, end) around the traced span over which all copies agree.

    With a single copy there is nothing to cross-check, so the whole page is
    returned and the caller marks it unverified.
    """
    if len(bases) == 1:
        return 0, REGS_PER_PAGE, False

    def same(i: int) -> bool:
        ref = lib[bases[0] + REG_BYTES * i:bases[0] + REG_BYTES * (i + 1)]
        return all(lib[other + REG_BYTES * i:other + REG_BYTES * (i + 1)] == ref
                   for other in bases[1:])

    start = lo
    while start > 0 and same(start - 1):
        start -= 1

    end = hi + 1
    while end < REGS_PER_PAGE and same(end):
        end += 1

    return start, end, True


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--lib', required=True, help='vendor libmpp_service.so')
    ap.add_argument('--trace', required=True, help='MMIO write trace')
    ap.add_argument('--stop', default='0xc50',
                    help='trace index at which the setup phase ends')
    ap.add_argument('--vendor-window',
                    help='au-chain-capture.sh capture from the streaming vendor. '
                         'Emits a trim table forcing our final state to match the '
                         'values the working device actually holds.')
    ap.add_argument('--our-window',
                    help='the matching capture from our stack, so the trim covers '
                         'only registers whose live state actually differs')
    ap.add_argument('-o', '--output', required=True)
    args = ap.parse_args()

    lib = open(args.lib, 'rb').read()
    digest = hashlib.sha256(lib).hexdigest()
    if LIB_SHA256 and digest != LIB_SHA256:
        sys.exit('libmpp_service.so sha256 mismatch: block offsets do not apply')

    first_write = load_trace(args.trace, int(args.stop, 16))
    trim = load_trim(args.vendor_window, args.our_window)
    setup = load_setup_writes(args.trace, int(args.stop, 16))

    output_fix = []
    for reg in OUTPUT_FIX_REGS:
        seen = [val for off, val in setup if off == reg]
        if not seen:
            sys.exit(f"output-fix register 0x{reg:04x} is absent from the setup table")

        output_fix.append((reg, seen[-1]))

    pages = []
    disagree = []
    n_traced = n_recovered = n_unverified = 0
    for page, bases in sorted(BLOCKS.items()):
        page_base = page << PAGE_SHIFT
        traced = sorted(off for off in first_write if (off >> PAGE_SHIFT) == page)
        lo = (traced[0] - page_base) // REG_BYTES
        hi = (traced[-1] - page_base) // REG_BYTES

        # Rebase each copy onto register 0 of the page.
        rebased = [base - REG_BYTES * lo for base in bases]
        start, end, verified = agree_span(lib, rebased, lo, hi)

        regs = []
        for i in range(start, end):
            off = page_base + REG_BYTES * i
            val = struct.unpack_from('<I', lib, rebased[0] + REG_BYTES * i)[0]
            seen = off in first_write
            if seen and first_write[off] != val:
                # The static default and the vendor's first write disagree.
                # Not fatal: the setup table writes the traced value after
                # the defaults, so the traced value wins. Reported because a
                # large count would mean the block map is wrong.
                disagree.append((off, val, first_write[off]))

            regs.append((off, val, seen))
            if seen:
                n_traced += 1
            elif verified:
                n_recovered += 1
            else:
                n_unverified += 1

        pages.append((page, len(bases), verified, regs))

    with open(args.output, 'w') as header:
        emit = header.write
        emit('/* SPDX-License-Identifier: GPL-2.0 */\n')
        emit('/*\n')
        emit(' * Artosyn ISP default register values.\n')
        emit(' *\n')
        emit(' * Generated by scripts/gen-isp-defaults.py. Do not edit.\n')
        emit(' *\n')
        emit(' * Extracted from the per-submodule default blocks in the vendor\n')
        emit(' * libmpp_service.so (sha256 %s).\n' % digest[:DIGEST_CHARS])
        emit(' *\n')
        emit(' * Entries marked "recovered" were not present in the vendor MMIO write\n')
        emit(' * trace. The vendor pushes its shadow image with a write-only-if-changed\n')
        emit(' * primitive, so a register already holding its target value is never\n')
        emit(' * written and cannot appear in a trace. Their values come from the block\n')
        emit(' * alone, confirmed identical across every copy of it.\n')
        emit(' *\n')
        emit(' * Page 0x28 has a single known copy, so its values cannot be\n')
        emit(' * cross-checked and are marked unverified.\n')
        emit(' */\n\n')
        emit('#ifndef _AR_ISP_DEFAULTS_H\n#define _AR_ISP_DEFAULTS_H\n\n')
        emit('#include <linux/compiler_attributes.h>\n')
        emit('#include <linux/types.h>\n\n')
        emit('struct ar_isp_reg {\n\tu16 off;\n\tu32 val;\n};\n\n')
        emit('/*\n')
        emit(' * The complete static default set. Not applied by the driver, which\n')
        emit(' * applies ar_isp_recovered followed by ar_isp_setup_1080p60 and reaches\n')
        emit(' * the same state in fewer writes. Kept because it is the mode-independent\n')
        emit(' * half of the configuration, and a driver that supports a second sensor\n')
        emit(' * mode needs it rather than the 1080p60 table.\n')
        emit(' */\n')
        emit('static const struct ar_isp_reg ar_isp_defaults[] __maybe_unused = {\n')
        for page, ncopies, verified, regs in pages:
            seen = sum(1 for _, _, traced in regs if traced)
            emit('\t/* page 0x%02x: %d registers, %d traced, %d %s, %d block cop%s */\n'
                 % (page, len(regs), seen, len(regs) - seen,
                    'recovered' if verified else 'unverified',
                    ncopies, 'y' if ncopies == 1 else 'ies'))

            for off, val, traced in regs:
                emit('\t{ 0x%04x, 0x%08x },%s\n'
                     % (off, val, '' if traced else
                        '\t/* recovered */' if verified else '\t/* unverified */'))

        emit('};\n\n')

        emit('/*\n')
        emit(' * Registers with a static default that the setup phase never writes.\n')
        emit(' * Apply these first, then ar_isp_setup_1080p60 in order.\n')
        emit(' */\n')
        emit('static const struct ar_isp_reg ar_isp_recovered[] = {\n')
        for _page, _ncopies, _verified, regs in pages:
            for off, val, seen in regs:
                if not seen:
                    emit('\t{ 0x%04x, 0x%08x },\n' % (off, val))

        emit('};\n\n')

        emit('/*\n')
        emit(' * The vendor setup phase for the 2-lane 1080p60 sensor mode, in write\n')
        emit(' * order. Order is load bearing: the block has a staged master enable and\n')
        emit(' * several arm-then-load registers. Consecutive duplicate writes are\n')
        emit(' * collapsed; repeated writes of differing values are kept.\n')
        emit(' *\n')
        emit(' * This is a static init table, not a timing replay. It carries no delays\n')
        emit(' * and no dependence on when each write happened.\n')
        emit(' */\n')
        emit('static const struct ar_isp_reg ar_isp_setup_1080p60[] = {\n')
        for off, val in setup:
            emit('\t{ 0x%04x, 0x%08x },\n' % (off, val))

        emit('};\n\n')

        if trim:
            emit('/*\n')
            emit(' * Final correction pass, measured rather than derived.\n')
            emit(' *\n')
            emit(' * Every entry is a register whose live value on the streaming\n')
            emit(' * vendor differs from the live value our stack reaches after the\n')
            emit(' * tables above. Live against live: our intended value is not a\n')
            emit(' * valid baseline, since a register can legitimately read back\n')
            emit(' * something other than what was written.\n')
            emit(' *\n')
            emit(' * Some entries are certainly counters and status words that\n')
            emit(' * ignore writes. Writing them is harmless and they cannot be\n')
            emit(' * told apart from configuration with a single sample each.\n')
            emit(' */\n')
            emit('static const struct ar_isp_reg ar_isp_vendor_trim[] = {\n')
            for off, val in trim:
                emit('\t{ 0x%04x, 0x%08x },\n' % (off, val))

            emit('};\n\n')

        emit('/*\n')
        emit(' * The output-stage pair that decides whether the image is crushed.\n')
        emit(' *\n')
        emit(' * ar_isp_setup_1080p60 carries both an early value and a later correction for\n')
        emit(' * each of these, at entries 572/573 and 1773/1774. Anything that applies only a\n')
        emit(' * prefix of that table stops before the correction and leaves the placeholder,\n')
        emit(' * which crushes 57% of the frame below luma 32.\n')
        emit(' *\n')
        emit(' * Bisected on hardware one register at a time, cumulative, luma mean of the\n')
        emit(' * same scene in one bring-up:\n')
        emit(' *\n')
        emit(' *\tbaseline 86.5   ...   0x2e20 91.1   0x2e2c 12.2   0x2e30 187.2\n')
        emit(' *\n')
        emit(' * The two are a PAIR and must be written together: 0x2e2c on its own takes the\n')
        emit(' * mean to 12, far worse than not writing either. They are kept in their own\n')
        emit(' * table for that reason, so a caller cannot apply half of them.\n')
        emit(' *\n')
        emit(' * Only these two. Every other register in the page was measured and none moved\n')
        emit(' * the image: the 0x2ebc-0x2efc LUT that our tables hold flat at 0x10101010\n')
        emit(' * changed the mean by 1.3 counts across all eighteen registers, and 0x2e00\'s\n')
        emit(' * enable nibble by 21. One of the measured writes, 0x2ea0, made it worse (187\n')
        emit(' * to 164), which is why this table is the proven pair rather than every\n')
        emit(' * difference against the vendor.\n')
        emit(' */\n')
        emit('static const struct ar_isp_reg ar_isp_output_fix[] = {\n')
        for off, val in output_fix:
            emit('\t{ 0x%04x, 0x%08x },\n' % (off, val))

        emit('};\n\n')
        emit('#endif /* _AR_ISP_DEFAULTS_H */\n')

    total = n_traced + n_recovered + n_unverified
    print('%s: %d registers over %d pages' % (args.output, total, len(pages)))
    print('  %d traced, %d recovered, %d unverified'
          % (n_traced, n_recovered, n_unverified))
    print('  setup table: %d ordered writes over %d registers'
          % (len(setup), len(set(off for off, _ in setup))))

    if trim:
        print('  trim table: %d registers corrected to the streaming vendor'
              % len(trim))

    if disagree:
        print('  %d registers where the static default and the vendor first '
              'write disagree (traced value wins)' % len(disagree))


if __name__ == '__main__':
    main()
