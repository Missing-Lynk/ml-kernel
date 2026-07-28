#!/usr/bin/env python3
"""Extract the Artosyn ISP per-submodule default register blocks from the vendor
libmpp_service.so and emit them as a kernel header.

The vendor library carries the static half of the ISP configuration as
per-submodule default register blocks in its data segment, each present as
several byte-identical copies. Blocks are located by matching the first-pass
values of a register page, taken from an MMIO write trace of the streaming
vendor, as an ordered run. See archive/re/notes/nt99235/isp-dsp-and-tuning-blob.md.

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

# sha256 of the air-unit libmpp_service.so the block map was derived from.
LIB_SHA256 = '4cfc8e6cfb42d8c821137993b95b152f1aaad7c53ce425e6a0493c4dd453936c'

REGS_PER_PAGE = 64
ISP_BASE = 0x08C00000


def load_trace(path, stop):
	"""First-pass value per ISP register offset during the setup phase."""
	first = {}
	for line in open(path):
		f = line.split()
		if len(f) < 4:
			continue
		# Traces taken with MMIOTRACE_READS also carry r-prefixed load lines. They share
		# the sequence counter, so they must be skipped rather than parsed as writes.
		if f[0][0] != 'w':
			continue
		if int(f[0][1:], 16) > stop:
			break
		pa = f[2].split('=')[1]
		if not pa.startswith('0x08c'):
			continue
		off = int(pa, 16) & 0xFFFF
		first.setdefault(off, int(f[3].split('=')[1], 16))
	return first


def load_setup_writes(path, stop):
	"""Every ISP write of the setup phase, in order, with runs collapsed.

	Order is preserved because the block carries an enable ladder and several
	arm-then-load sequences whose result depends on it. Consecutive writes of
	the same value to the same register are dropped; distinct values are not,
	since a rewrite is how the vendor arms some registers.
	"""
	writes = []
	for line in open(path):
		f = line.split()
		if len(f) < 4:
			continue
		# Traces taken with MMIOTRACE_READS also carry r-prefixed load lines. They share
		# the sequence counter, so they must be skipped rather than parsed as writes.
		if f[0][0] != 'w':
			continue
		if int(f[0][1:], 16) > stop:
			break
		pa = f[2].split('=')[1]
		if not pa.startswith('0x08c'):
			continue
		off = int(pa, 16) & 0xFFFF
		val = int(f[3].split('=')[1], 16)
		if writes and writes[-1] == (off, val):
			continue
		writes.append((off, val))
	return writes


def load_window(path):
	"""ISP registers from an au-chain-capture.sh capture."""
	out, section = {}, None
	for line in open(path):
		m = re.match(r'SECTION (\S+)', line)
		if m:
			section = m.group(1)
			continue
		m = re.match(r'\+0x([0-9a-f]{4}): (.*)', line)
		if m and section and section.startswith('isp-'):
			page = int(section[4:], 16)
			off = (page << 8) + int(m.group(1), 16)
			for i, w in enumerate(m.group(2).split()):
				if re.fullmatch(r'[0-9a-f]{8}', w):
					out[off + 4 * i] = int(w, 16)
	return out


def load_trim(vendor_path, our_path):
	"""Registers whose live value on the working device differs from ours.

	Live against live, which is the only valid comparison: our intended value is
	not a baseline, and a register can legitimately read back something else.
	"""
	if not vendor_path or not our_path:
		return []
	v = load_window(vendor_path)
	o = load_window(our_path)
	return [(k, v[k]) for k in sorted(set(v) & set(o)) if v[k] != o[k]]


def agree_span(data, bases, lo, hi):
	"""Widest [a, b) around the traced span over which all copies agree.

	With a single copy there is nothing to cross-check, so the whole page is
	returned and the caller marks it unverified.
	"""
	if len(bases) == 1:
		return 0, REGS_PER_PAGE, False

	def same(i):
		w = data[bases[0] + 4 * i:bases[0] + 4 * (i + 1)]
		return all(data[b + 4 * i:b + 4 * (i + 1)] == w for b in bases[1:])

	a = lo
	while a > 0 and same(a - 1):
		a -= 1
	b = hi + 1
	while b < REGS_PER_PAGE and same(b):
		b += 1
	return a, b, True


def main():
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

	data = open(args.lib, 'rb').read()
	digest = hashlib.sha256(data).hexdigest()
	if LIB_SHA256 and digest != LIB_SHA256:
		sys.exit('libmpp_service.so sha256 mismatch: block offsets do not apply')

	first = load_trace(args.trace, int(args.stop, 16))
	trim = load_trim(args.vendor_window, args.our_window)
	setup = load_setup_writes(args.trace, int(args.stop, 16))

	pages = []
	disagree = []
	n_traced = n_recovered = n_unverified = 0
	for pg, bases in sorted(BLOCKS.items()):
		pagebase = pg << 8
		traced = sorted(o for o in first if (o >> 8) == pg)
		lo = (traced[0] - pagebase) // 4
		hi = (traced[-1] - pagebase) // 4
		# Rebase each copy onto register 0 of the page.
		zero = [b - 4 * lo for b in bases]
		a, b, verified = agree_span(data, zero, lo, hi)

		regs = []
		for i in range(a, b):
			off = pagebase + 4 * i
			val = struct.unpack_from('<I', data, zero[0] + 4 * i)[0]
			seen = off in first
			if seen and first[off] != val:
				# The static default and the vendor's first write disagree.
				# Not fatal: the setup table writes the traced value after
				# the defaults, so the traced value wins. Reported because a
				# large count would mean the block map is wrong.
				disagree.append((off, val, first[off]))
			regs.append((off, val, seen))
			if seen:
				n_traced += 1
			elif verified:
				n_recovered += 1
			else:
				n_unverified += 1
		pages.append((pg, len(bases), verified, regs))

	with open(args.output, 'w') as out:
		w = out.write
		w('/* SPDX-License-Identifier: GPL-2.0 */\n')
		w('/*\n')
		w(' * Artosyn ISP default register values.\n')
		w(' *\n')
		w(' * Generated by scripts/gen-isp-defaults.py. Do not edit.\n')
		w(' *\n')
		w(' * Extracted from the per-submodule default blocks in the vendor\n')
		w(' * libmpp_service.so (sha256 %s).\n' % digest[:32])
		w(' *\n')
		w(' * Entries marked "recovered" were not present in the vendor MMIO write\n')
		w(' * trace. The vendor pushes its shadow image with a write-only-if-changed\n')
		w(' * primitive, so a register already holding its target value is never\n')
		w(' * written and cannot appear in a trace. Their values come from the block\n')
		w(' * alone, confirmed identical across every copy of it.\n')
		w(' *\n')
		w(' * Page 0x28 has a single known copy, so its values cannot be\n')
		w(' * cross-checked and are marked unverified.\n')
		w(' */\n\n')
		w('#ifndef _AR_ISP_DEFAULTS_H\n#define _AR_ISP_DEFAULTS_H\n\n')
		w('#include <linux/compiler_attributes.h>\n')
		w('#include <linux/types.h>\n\n')
		w('struct ar_isp_reg {\n\tu16 off;\n\tu32 val;\n};\n\n')
		w('/*\n')
		w(' * The complete static default set. Not applied by the driver, which\n')
		w(' * applies ar_isp_recovered followed by ar_isp_setup_1080p60 and reaches\n')
		w(' * the same state in fewer writes. Kept because it is the mode-independent\n')
		w(' * half of the configuration, and a driver that supports a second sensor\n')
		w(' * mode needs it rather than the 1080p60 table.\n')
		w(' */\n')
		w('static const struct ar_isp_reg ar_isp_defaults[] __maybe_unused = {\n')
		for pg, ncopies, verified, regs in pages:
			seen = sum(1 for _, _, s in regs if s)
			w('\t/* page 0x%02x: %d registers, %d traced, %d %s, %d block cop%s */\n'
			  % (pg, len(regs), seen, len(regs) - seen,
			     'recovered' if verified else 'unverified',
			     ncopies, 'y' if ncopies == 1 else 'ies'))
			for off, val, s in regs:
				w('\t{ 0x%04x, 0x%08x },%s\n'
				  % (off, val, '' if s else
				     '\t/* recovered */' if verified else '\t/* unverified */'))
		w('};\n\n')

		w('/*\n')
		w(' * Registers with a static default that the setup phase never writes.\n')
		w(' * Apply these first, then ar_isp_setup_1080p60 in order.\n')
		w(' */\n')
		w('static const struct ar_isp_reg ar_isp_recovered[] = {\n')
		for pg, ncopies, verified, regs in pages:
			for off, val, seen in regs:
				if not seen:
					w('\t{ 0x%04x, 0x%08x },\n' % (off, val))
		w('};\n\n')

		w('/*\n')
		w(' * The vendor setup phase for the 2-lane 1080p60 sensor mode, in write\n')
		w(' * order. Order is load bearing: the block has a staged master enable and\n')
		w(' * several arm-then-load registers. Consecutive duplicate writes are\n')
		w(' * collapsed; repeated writes of differing values are kept.\n')
		w(' *\n')
		w(' * This is a static init table, not a timing replay. It carries no delays\n')
		w(' * and no dependence on when each write happened.\n')
		w(' */\n')
		w('static const struct ar_isp_reg ar_isp_setup_1080p60[] = {\n')
		for off, val in setup:
			w('\t{ 0x%04x, 0x%08x },\n' % (off, val))
		w('};\n\n')

		if trim:
			w('/*\n')
			w(' * Final correction pass, measured rather than derived.\n')
			w(' *\n')
			w(' * Every entry is a register whose live value on the streaming\n')
			w(' * vendor differs from the live value our stack reaches after the\n')
			w(' * tables above. Live against live: our intended value is not a\n')
			w(' * valid baseline, since a register can legitimately read back\n')
			w(' * something other than what was written.\n')
			w(' *\n')
			w(' * Some entries are certainly counters and status words that\n')
			w(' * ignore writes. Writing them is harmless and they cannot be\n')
			w(' * told apart from configuration with a single sample each.\n')
			w(' */\n')
			w('static const struct ar_isp_reg ar_isp_vendor_trim[] = {\n')
			for off, val in trim:
				w('\t{ 0x%04x, 0x%08x },\n' % (off, val))
			w('};\n\n')
		w('#endif /* _AR_ISP_DEFAULTS_H */\n')

	total = n_traced + n_recovered + n_unverified
	print('%s: %d registers over %d pages' % (args.output, total, len(pages)))
	print('  %d traced, %d recovered, %d unverified'
	      % (n_traced, n_recovered, n_unverified))
	print('  setup table: %d ordered writes over %d registers'
	      % (len(setup), len(set(o for o, _ in setup))))
	if trim:
		print('  trim table: %d registers corrected to the streaming vendor'
		      % len(trim))
	if disagree:
		print('  %d registers where the static default and the vendor first '
		      'write disagree (traced value wins)' % len(disagree))


if __name__ == '__main__':
	main()
