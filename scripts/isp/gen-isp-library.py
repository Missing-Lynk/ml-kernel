#!/usr/bin/env python3
"""
Emit the vendor ISP's per-submodule static register images as a kernel header.

The vendor service keeps one static register image per ISP submodule in its own
data segment, and installs it verbatim at init. The images are reachable from
code alone, so every value this script emits is library data with a name on it
rather than a value recognised in a recording.

How the chain is read, all of it inside libmpp_service.so:

  get_isp_init_config() resolves the registered ops object named
  "ar9311_isp_init_hw_opt" and tail-calls its first method, which returns a
  fixed address in the library's own data. That address is the ISP-init
  template array: 76 {u64 source, u64 length} descriptors, immediately followed
  by the ops object itself. Its siblings ar9341_ and ar9411_ hold the same
  shape for the other two SoCs in this family and are not used here.

  Each submodule's config function calls get_isp_init_config(), loads one
  descriptor's source and length at a fixed byte offset, and passes both to
  isp_memcpy with the module's own register window as the destination. The
  offset divided by 16 is the entry index, which is what binds an entry to a
  module. Each of those call sites also carries an ar_log_func format string of
  the form "[memcpy_dump]isp_sub_<module>_reg=%d <module>_table_size=%d", which
  names the module outright. isp_sub_cm.c at 0x19fe28 is the clearest example.

  The destination is the module's register window, set up in its constructor as
  ar_dev_pa2va(isp_pa) + bank, with the bank offset a literal in the vendor's
  code. isp_sub_acm.c at 0x207318 stores ar_dev_pa2va(...) + 0x7600, and 0x7600
  is the acm bank. So the bank is a code fact too, not an alignment guess.

The result is that an image is a contiguous register run: entry length bytes
starting at the module's bank. That bound matters. The earlier block map in
gen-isp-defaults.py located blocks by matching a page of traced values and then
emitted a whole 64-register page from each, which reads past the end of every
image shorter than a page and emits whatever the linker placed next as if it
were register data. check-isp-library.py measures that overrun.

The library is proprietary and is not in the repository. The generated header is
checked in, so this script is only rerun when the module map changes.

    kernel/scripts/isp/gen-isp-library.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        -o overlay/drivers/media/artosyn/vendor-tables/ar-isp-library.h
"""

import argparse
import hashlib
import struct
import sys

import arlib

# sha256 of the air-unit libmpp_service.so this map was read from. The slot-A
# library is a different build of the same version; its ISP-init template array
# and every payload the array points at are byte-identical, so the map holds for
# both and only this digest is pinned.
LIB_SHA256 = '4cfc8e6cfb42d8c821137993b95b152f1aaad7c53ce425e6a0493c4dd453936c'

REG_BYTES = 4

# entry, module, bank, address of the get_isp_init_config call site that binds
# the entry to the module. Sorted by bank, which is the order the header emits.
#
# Three rows carry no call site. Entry 56 is installed by isp_input.c, whose
# bank 0x7000 is a constructor literal, but no call site loading entry 56 was
# found, so the entry-to-module binding there rests on the base alone. Entries
# 67 to 69 are all loaded by the isp_sub_hdr.c call site at 0x197a84; 0x1fc0 and
# 0x1fe0 are constructor literals and 0x1f98 is the only base that tiles the
# three images into the run ending at hdr_lsc's second window at 0x1ffc.
MODULES = (
    (31, 'cfa', 0x0800, 0x1b17cc),
    (8, 'dpc', 0x0c00, 0x1c88e8),
    (10, 'rnr', 0x1800, 0x19a8b8),
    (3, 'hdr', 0x1c00, 0x197a84),
    (27, 'hdr_rro_0_stats', 0x1d20, 0x1fae5c),
    (28, 'hdr_rro_1_stats', 0x1d78, 0x1fbedc),
    (13, 'hdr_lsc', 0x1dd0, 0x1b9a38),
    (18, 'hdr_awbs_stats', 0x1e44, 0x1f9dbc),
    (27, 'hdr_rro_face_stats', 0x1f40, 0x1fcf54),
    (68, 'hdr', 0x1f98, None),
    (67, 'hdr', 0x1fc0, None),
    (69, 'hdr', 0x1fe0, None),
    (39, 'ltm', 0x2800, 0x18e4b0),
    (54, 'de3d', 0x2e00, 0x1c5278),
    (21, 'drc', 0x3000, 0x1a5828),
    (33, 'ccm1', 0x3400, 0x187654),
    (34, 'ccm2', 0x3800, 0x1af450),
    (46, 'rgb2yuv', 0x3c00, 0x19e8f4),
    (50, 'cnf', 0x3c64, 0x1a2800),
    (48, 'lnr', 0x3cc8, 0x1bd430),
    (53, 'cm2', 0x4800, 0x1a14ec),
    (47, 'cm', 0x4834, 0x19fe28),
    (11, 'lsc', 0x4c00, 0x1b6884),
    (20, 'wb', 0x5000, 0x1ae32c),
    (41, 'lut3d', 0x5800, 0x1c2dcc),
    (23, 'raw_hist_stats', 0x6000, 0x1fecac),
    (25, 'rro_stats', 0x6400, 0x201fc8),
    (26, 'rro_face_stats', 0x64c8, 0x1f891c),
    (19, 'awbs_stats', 0x6c00, 0x1f7634),
    (56, 'isp_input', 0x7000, None),
    (58, 'nuc_dpc', 0x7200, 0x1e3b9c),
    (60, 'qgg', 0x7230, 0x1e6da0),
    (61, 'lms', 0x7264, 0x1e8968),
    (63, 'ir_gtm', 0x7278, 0x1ea630),
    (16, 'af_stats', 0x7400, 0x1f579c),
    (57, 'acm', 0x7600, 0x2076d4),
)

# Modules whose bank is a constructor literal but whose template entry the
# vendor never installs in this pipeline: the setup phase writes no register in
# any of these banks. Recorded so a later reader does not hunt for them again.
UNINSTALLED = (
    (0, 'tg', 0x4000),
    (2, 'blc', 0x4200),
    (5, 'digigain1', 0x4600),
    (7, 'digigain2', 0x4700),
    (30, 'gib', 0x2400),
    (32, 'rgb_max_stats', 0x5400),
    (35, 'rgb_hist_stats', 0x5c00),
)

DIGEST_CHARS = 32
REGS_PER_LINE = 1


def images(lib: bytes) -> list[tuple[int, str, int, int | None, list[int]]]:
    """(entry, module, bank, call site, words) for every installed image."""
    out = []
    for entry, module, bank, site in MODULES:
        source, length = arlib.template_entry(lib, entry)
        if not source or not length:
            sys.exit(f'template entry {entry} ({module}) is empty; '
                     f'the array layout has changed')

        if length % REG_BYTES:
            sys.exit(f'template entry {entry} ({module}) is 0x{length:x} bytes, '
                     f'not a whole number of registers')

        raw = arlib.lib_slice(lib, source, length, f'{module} static image')
        out.append((entry, module, bank, site,
                    list(struct.unpack(f'<{length // REG_BYTES}I', raw))))

    return out


def check_no_overlap(installed: list) -> None:
    """No two images may claim the same register."""
    owner: dict[int, str] = {}
    for _entry, module, bank, _site, words in installed:
        for i in range(len(words)):
            off = bank + REG_BYTES * i
            if off in owner:
                sys.exit(f'register 0x{off:04x} is claimed by both '
                         f'{owner[off]} and {module}')

            owner[off] = module


def emit(handle, lib: bytes, digest: str, installed: list) -> None:
    write = handle.write
    total = sum(len(words) for *_, words in installed)

    write(arlib.banner('scripts/isp/gen-isp-library.py', [
        'The vendor ISP submodule static register images.',
        '',
        'Read from the ISP-init template array in libmpp_service.so (sha256',
        f'{digest[:DIGEST_CHARS]}). Each entry of that array is a {{source,',
        'length} descriptor; the submodule that installs it is named by the',
        'ar_log_func format string at the get_isp_init_config call site that',
        'loads it, and the bank it installs at is a literal in that module\'s',
        'constructor, added to the mapped ISP base. Both are code, so no',
        'recording is involved in placing a value.',
        '',
        'An image is exactly its descriptor length. Registers past that bound',
        'are not part of it, whatever the library data segment holds there.',
        '',
        'The vendor installs these with a compare-then-write primitive, so a',
        'register already holding its image value is never written and cannot',
        'appear in an MMIO trace. That is why the image, not the trace, is the',
        'source: the trace is a subset by construction.',
        '',
        'Not applied by the driver. It is the reference set that says which of',
        'the values the driver does apply have a vendor origin.',
    ]))
    write('\n')

    guard_open, guard_close = arlib.guard('AR_ISP_LIBRARY_H')
    write(guard_open + '\n')
    write('#include <linux/compiler_attributes.h>\n\n')
    write('#include "ar-isp-defaults.h"\n\n')

    write('/*\n')
    write(f' * {total} registers across {len(installed)} submodule images.\n')
    write(' *\n')
    write(' * Each block comment gives the template array entry the image comes\n')
    write(' * from and the address of the call site that binds the entry to the\n')
    write(' * module, so a reader can check the binding without rerunning the\n')
    write(' * generator.\n')
    write(' */\n')
    write('static const struct ar_isp_reg ar_isp_library[] __maybe_unused = {\n')
    for entry, module, bank, site, words in installed:
        where = f'call site {site:#08x}' if site else 'no call site located'
        write(f'\t/* {module}: entry {entry}, {len(words)} registers at '
              f'{bank:#06x}, {where} */\n')
        for i, value in enumerate(words):
            write(f'\t{{ {bank + REG_BYTES * i:#06x}, {value:#010x} }},\n')

    write('};\n\n')

    write('/*\n')
    write(' * Submodules whose bank is a constructor literal and whose template\n')
    write(' * entry the vendor never installs on this pipeline: the setup phase\n')
    write(' * writes no register in any of these banks.\n')
    write(' *\n')
    for entry, module, bank in UNINSTALLED:
        write(f' *   {module} at {bank:#06x}, entry {entry}\n')

    write(' */\n\n')
    write(guard_close)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--lib', required=True, help='vendor libmpp_service.so')
    ap.add_argument('-o', '--output', required=True)
    args = ap.parse_args()

    with open(args.lib, 'rb') as handle:
        lib = handle.read()

    digest = hashlib.sha256(lib).hexdigest()
    if digest != LIB_SHA256:
        sys.exit('libmpp_service.so sha256 mismatch: the module map does not apply')

    installed = images(lib)
    check_no_overlap(installed)

    with open(args.output, 'w') as handle:
        emit(handle, lib, digest, installed)

    return 0


if __name__ == '__main__':
    sys.exit(main())
