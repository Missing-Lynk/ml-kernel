#!/usr/bin/env python3
"""
Prove the submodule image map in ar-isp-library.h against the vendor's own writes.

The map is recovered from code: which template array entry a submodule installs
comes from the get_isp_init_config call site that loads it, and the bank it
installs at comes from a literal in the module's constructor. Neither reads a
capture. What a capture can still do is falsify the result, because if an image
is placed at the wrong bank then the values the vendor writes there will not be
the values in the image.

So this script is a placement oracle, not a source of values. It replays the
vendor's MMIO write trace, takes the first value written to each ISP register,
and checks that every register an image covers and the vendor wrote holds that
image's value. Registers the vendor never wrote are counted and reported, not
required: the install path is compare-then-write, so a register already holding
its image value never appears in a trace.

It also measures what the earlier page-block map in gen-isp-defaults.py emits
past the end of each image. A page block is a fixed 64 registers, an image is as
long as its descriptor says, and for every image shorter than a page the
difference is emitted as register data when it is really whatever the linker
placed after the payload. Where those past-the-end values are exactly the bytes
following the payload, that is not an inference.

Refuses to pass unless:

  every register an image covers that the vendor also wrote holds the image's
  value, for every image in the map;

  no two images claim the same register;

  every image lies inside the ISP register window.

The library and the trace are not in the repository.

    kernel/scripts/isp/check-isp-library.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        --trace out/au-mmiotrace/mmio-combined.log
"""

import argparse
import importlib.util
import struct
import sys

import arlib

ISP_BASE = 0x08C00000
ISP_SPAN = 0x100000
REG_OFF_MASK = 0xFFFF
REG_BYTES = 4

# Trace index at which the vendor's setup phase ends, the same bound
# gen-isp-defaults.py uses.
SETUP_STOP = 0xC50

# The page the earlier block map emits per located block, for the overrun
# measurement only.
REGS_PER_PAGE = 64

# Registers an image covers where the vendor's first write is not the image
# value. Both hold zero in the image and are written non-zero, so the module
# does not carry them: it computes them at install and the image slot is a
# placeholder. Named here rather than absorbed into a tolerance, because a
# tolerance would also hide a misplaced image.
NOT_CARRIED = {
    0x1CF0: 'hdr, image holds 0 and the module writes 1',
    0x1D18: 'hdr, image holds 0 and the module writes 4',
}


def load_module_map() -> tuple[tuple, tuple]:
    """MODULES and UNINSTALLED from the generator, so there is one map."""
    spec = importlib.util.spec_from_file_location(
        'gen_isp_library', __file__.replace('check-isp-library.py',
                                            'gen-isp-library.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.MODULES, mod.UNINSTALLED


def first_writes(path: str) -> dict[int, int]:
    """First value the vendor wrote to each ISP register during setup."""
    first: dict[int, int] = {}
    with open(path) as trace:
        for line in trace:
            fields = line.split()
            if len(fields) < 4 or fields[0][0] != 'w':
                continue

            if int(fields[0][1:], 16) > SETUP_STOP:
                break

            phys = int(fields[2].split('=')[1], 16)
            if ISP_BASE <= phys < ISP_BASE + ISP_SPAN:
                first.setdefault(phys & REG_OFF_MASK,
                                 int(fields[3].split('=')[1], 16))

    return first


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--lib', required=True, help='vendor libmpp_service.so')
    ap.add_argument('--trace', required=True, help='vendor MMIO write trace')
    args = ap.parse_args()

    with open(args.lib, 'rb') as handle:
        lib = handle.read()

    modules, _uninstalled = load_module_map()
    first = first_writes(args.trace)
    print(f'{len(first)} ISP registers written during the vendor setup phase\n')

    owner: dict[int, str] = {}
    bad = 0
    covered = matched = unwritten = 0
    overrun = 0
    print(f'{"module":<20}{"entry":>6}{"regs":>6}{"written":>9}{"agree":>7}'
          f'{"":4}past the page block')
    for entry, module, bank, _site in modules:
        source, length = arlib.template_entry(lib, entry)
        words = struct.unpack(f'<{length // REG_BYTES}I',
                              arlib.lib_slice(lib, source, length, module))

        if bank + length > ISP_SPAN:
            sys.exit(f'{module}: image at {bank:#06x} runs past the ISP window')

        seen = agree = 0
        for i, value in enumerate(words):
            off = bank + REG_BYTES * i
            if off in owner:
                print(f'  overlap: 0x{off:04x} is claimed by both '
                      f'{owner[off]} and {module}')
                bad += 1

            owner[off] = module
            if off in first:
                seen += 1
                if first[off] == value:
                    agree += 1
                elif off not in NOT_CARRIED:
                    print(f'  0x{off:04x} ({module}): image {value:#010x}, '
                          f'vendor wrote {first[off]:#010x}')
                    bad += 1

        # what a fixed 64-register page block emits past the end of this image
        spill = max(0, REGS_PER_PAGE - len(words))
        overrun += spill

        covered += len(words)
        matched += agree
        unwritten += len(words) - seen
        note = f'{spill:>4}' if spill > 0 else '   0'
        print(f'{module:<20}{entry:>6}{len(words):>6}{seen:>9}{agree:>7}'
              f'{"":4}{note}')

    print(f'\n{covered} registers covered by an image, {matched} of them written '
          f'by the vendor and holding the image value, {unwritten} never written')
    print(f'{len(NOT_CARRIED)} the module writes a value of its own instead:')
    for off, why in sorted(NOT_CARRIED.items()):
        print(f'  0x{off:04x}  {why}')
    print(f'{overrun} registers a fixed 64-register page block would emit past '
          f'the end of these images')

    if bad:
        print(f'\nFAIL: {bad} placements disagree with the vendor trace')
        return 1

    print('\nevery register an image covers and the vendor wrote holds the '
          'image value')
    return 0


if __name__ == '__main__':
    sys.exit(main())
