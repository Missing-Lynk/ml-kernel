#!/usr/bin/env python3
"""Extract the CVISP configuration from a wide MMIO write trace of the streaming vendor
and emit it as a kernel header.

CVISP is the block at 0x08e00000, ISP base + 0x200000, immediately past the ISP's 2 MiB
reg range. It is absent from the vendor device tree. The name comes from libmpp_service.so,
which is unstripped and exports a complete cvisp_* stack (device, input, output, filter,
stats, gamma, LSC, plus cvisp_outlib_*, cvisp_device_irq_process and cvisp_dispatch_irq).
It is NOT the DTS scaler@08840000 or gdc@08848000, which are different addresses.

Why this exists: every earlier trace narrowed the tracer window to a single block, while the
vendor maps all 256 MiB of register space in one call and drives every block through it. So
writes to CVISP happened throughout and were never recorded. Nothing about the ISP work was
wrong; it was scoped to windows we chose ourselves. CVISP, not the ISP's page 0x2e, is what
writes frames to DRAM in the vendor's design.

Four tables come out, because the vendor drives the block at different cadences:

  setup    once, ending at the output enable 0x8000 = 0x00800806
  late     once, a tail of 101 registers the vendor writes just after the enable
  ring     five buffer sets, one Y/U/V triplet per frame, round robin
  tick     eight registers, written once per ring wrap rather than once per frame

The setup boundary is the enable write, not the first per-frame VIF acknowledge that the ISP
tables use. The VIF acknowledge lands several frames later, so cutting there pulls ring
rotations into the setup table.

The block therefore is not fully configured when it is enabled: the late tail carries the
arbitration table on page 0x0000 and the channel geometry on page 0x4000, and it lands with
the first frames already in flight. Whether that ordering is required or merely what the
vendor's threading produced is not established, so the two tables stay separate rather than
being merged into one.

Usage:
    kernel/scripts/gen-cvisp-defaults.py --trace out/au-mmiotrace/wide-sweep.log \\
        > kernel/overlay/drivers/media/artosyn/ar-cvisp-defaults.h
"""

import argparse
import collections
import hashlib
import sys

BLK_BASE = 0x08E00000
BLK_SIZE = 0x10000

# Output enable. The vendor stages this register 0x00800800 -> 0x00800802 -> 0x00800806 and
# then never writes it again, so the final value marks the end of setup.
CVISP_CONTROL = 0x8000
CVISP_ENABLE_DONE = 0x00800806

# Output plane bases, Y/U/V, rewritten in lockstep once per frame.
CVISP_PLANE = (0x8098, 0x8174, 0x8194)


def load(path):
	"""Ordered (phys, value) writes from the trace."""
	writes = []
	for line in open(path):
		f = line.split()
		if len(f) < 4 or f[0][0] != 'w':
			continue
		pa = int(f[2].split('=')[1], 16)
		val = int(f[3].split('=')[1], 16)
		writes.append((pa, val))
	return writes


def block_writes(writes):
	return [(pa - BLK_BASE, v) for pa, v in writes
		if BLK_BASE <= pa < BLK_BASE + BLK_SIZE]


def split_at_enable(seq):
	"""Index just past the final output-enable write."""
	for i in range(len(seq) - 1, -1, -1):
		if seq[i] == (CVISP_CONTROL, CVISP_ENABLE_DONE):
			return i + 1
	sys.exit("no output-enable write (0x%04x = 0x%08x) in the trace"
		 % (CVISP_CONTROL, CVISP_ENABLE_DONE))


def collapse(seq):
	"""Drop consecutive duplicate writes to the same register.

	Order is kept: the block has a staged enable whose result depends on the sequence,
	exactly as the ISP does. Only runs of the identical write to the identical register
	are removed.
	"""
	out = []
	for off, val in seq:
		if out and out[-1] == (off, val):
			continue
		out.append((off, val))
	return out


def extract_ring(steady):
	"""The distinct Y/U/V buffer sets, in the order the vendor first queues them."""
	sets, seen = [], set()
	cur = {}
	for off, val in steady:
		if off not in CVISP_PLANE:
			continue
		cur[off] = val
		if len(cur) == len(CVISP_PLANE):
			triplet = tuple(cur[o] for o in CVISP_PLANE)
			if triplet not in seen:
				seen.add(triplet)
				sets.append(triplet)
			cur = {}
	return sets


def emit(name, rows, comment):
	print(f"\n/*\n{comment}\n */")
	print(f"static const struct ar_cvisp_reg {name}[] = {{")
	for off, val in rows:
		print(f"\t{{ 0x{off:04x}, 0x{val:08x} }},")
	print("};")


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument('--trace', required=True,
			help='wide MMIO write trace covering 0x08e00000')
	args = ap.parse_args()

	digest = hashlib.sha256(open(args.trace, 'rb').read()).hexdigest()

	seq = block_writes(load(args.trace))
	if not seq:
		sys.exit(f"{args.trace}: no writes to 0x{BLK_BASE:08x}; "
			 "was the trace window wide enough?")

	cut = split_at_enable(seq)
	setup = collapse(seq[:cut])
	steady = seq[cut:]

	ring = extract_ring(steady)

	rate = collections.Counter(off for off, _ in steady)

	# The late tail: registers the vendor writes exactly once after the enable. They form a
	# contiguous run at the head of the steady state, interleaved with the first frames'
	# plane and tick writes, which is why they are selected by write count rather than by a
	# position cut.
	late = [(off, val) for off, val in steady if rate[off] == 1]

	# The tick group: the wrap-cadence registers, one write per five frames. The threshold
	# separates them from a handful of registers the vendor rewrites only a few times
	# across the whole capture, which are reported but not tabled.
	tick_val, odd = {}, {}
	for off, val in steady:
		if off in CVISP_PLANE or rate[off] == 1:
			continue
		(tick_val if rate[off] > 100 else odd).setdefault(off, val)
	tick = sorted(tick_val.items())

	# Self-check: replaying the setup table in order must reproduce the vendor's final
	# setup value for every register it touches.
	final = {}
	for off, val in seq[:cut]:
		final[off] = val
	replay = {}
	for off, val in setup:
		replay[off] = val
	bad = [o for o in final if replay.get(o) != final[o]]

	print("/* Generated by kernel/scripts/gen-cvisp-defaults.py. Do not edit. */")
	print("/*")
	print(" * CVISP register configuration, recovered from a wide MMIO write trace of the")
	print(" * streaming vendor. CVISP is the block at 0x08e00000; the name is from the")
	print(" * cvisp_* stack in the vendor's unstripped libmpp_service.so. It is absent from")
	print(" * the vendor device tree.")
	print(" *")
	print(" * In the vendor's design this block, not the ISP, writes frames to DRAM. The")
	print(" * ISP feeds it and CVISP owns the output queue.")
	print(" *")
	print(" * Geometry registers carry both 0x04380780 (1080 x 1920) and 0x021c03c0")
	print(" * (540 x 960); 0x8008 is written with the smaller value during setup and ends at")
	print(" * the larger one, so which stage is scaled is NOT established here. 0x021c03c0 is")
	print(" * also what ISP 0x7080 reads on the streaming vendor.")
	print(" *")
	print(" * The trace was captured writes-only, so the absence of reads below says nothing")
	print(" * about whether the vendor polls this block.")
	print(" *")
	print(f" * Trace: {args.trace}")
	print(f" * sha256: {digest}")
	print(" */")
	print()
	print("struct ar_cvisp_reg {")
	print("\tu16 off;")
	print("\tu32 val;")
	print("};")
	print()
	print("struct ar_cvisp_bufset {")
	print("\tu32 y;")
	print("\tu32 u;")
	print("\tu32 v;")
	print("};")

	emit('ar_cvisp_setup', setup,
	     " * Setup, in write order with consecutive duplicates collapsed, ending at the\n"
	     " * output enable. The last entries stage 0x8000 through 0x00800800, 0x00800802,\n"
	     " * 0x00800806; bits 1 and 2 are the launch candidates and their individual\n"
	     " * meanings are not decoded. The plane bases this table leaves behind are ring\n"
	     " * set 0, so applying it alone arms the first frame.")

	emit('ar_cvisp_late', late,
	     " * The tail the vendor writes just after the enable, in order: the arbitration\n"
	     " * table on page 0x0000, then per-channel geometry and limits on page 0x4000,\n"
	     " * then the upper halves of the two tick banks. Page 0x4000 carries 0x780 x 0x438\n"
	     " * (1920 x 1080), so this stage is not the scaled one.\n"
	     " *\n"
	     " * The vendor issues these with frames already in flight. Whether that ordering\n"
	     " * matters or is just what its threading produced is not established.")

	print("\n/*\n * The output queue: five Y/U/V buffer sets, one triplet written per frame in\n"
	      " * round robin. The vendor's own DRAM addresses; a driver that allocates its own\n"
	      " * buffers replaces these and must keep the relative plane spacing.\n */")
	print("static const struct ar_cvisp_bufset ar_cvisp_ring[] = {")
	for y, u, v in ring:
		print(f"\t{{ 0x{y:08x}, 0x{u:08x}, 0x{v:08x} }},")
	print("};")

	emit('ar_cvisp_tick', tick,
	     " * Written once per ring wrap, not once per frame: the vendor issues this group\n"
	     " * after every fifth triplet. Two banks of four registers, all taking 0x00000100.\n"
	     " * Purpose undecoded; an acknowledge or a queue re-arm are both consistent with\n"
	     " * the cadence.")

	print(f"\n/* setup: {len(setup)} ordered writes over {len(final)} registers", end='')
	print(", reconstruction exact" if not bad
	      else f", {len(bad)} reconstruction mismatches", end='')
	print(" */")
	print(f"/* late: {len(late)} registers */")
	print(f"/* ring: {len(ring)} buffer sets */")
	if tick:
		wraps = rate[tick[0][0]]
		print(f"/* tick: {len(tick)} registers, {wraps} wraps over "
		      f"{rate[CVISP_PLANE[0]]} frames */")
	if odd:
		print("/* not tabled, rewritten a few times aperiodically: "
		      + ", ".join(f"0x{o:04x}={v:#010x} ({rate[o]}x)"
				  for o, v in sorted(odd.items())) + " */")

	print(f"setup: {len(setup)} writes over {len(final)} registers", file=sys.stderr)
	print(f"late: {len(late)} registers", file=sys.stderr)
	print(f"ring: {len(ring)} buffer sets", file=sys.stderr)
	print(f"tick: {len(tick)} registers", file=sys.stderr)
	if odd:
		print("aperiodic, not tabled: "
		      + ", ".join(f"0x{o:04x}" for o in sorted(odd)), file=sys.stderr)
	if bad:
		print(f"WARNING: {len(bad)} registers do not reconstruct: "
		      + ", ".join(f"0x{o:04x}" for o in bad[:8]), file=sys.stderr)
	else:
		print("reconstruction exact", file=sys.stderr)


if __name__ == '__main__':
	main()
