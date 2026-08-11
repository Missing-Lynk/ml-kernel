#!/usr/bin/env python3
"""
Prove the seven hdr registers the vendor moves away from its own image.

`isp_sub_hdr` is the one bank here with no RAM shadow: its map handler at
0x196900 stores the mapped bank VA at priv+568, and all seven registers are
written by a direct `str` through it. Two things nearby look like the bank and
are not. `[priv+544] + 0x3194` is a DRAM stand-in used only when the work mode
is 3, at the same offsets; and the tuning record at `tuning_mgr[544] +
idx*0x3b1e8 + 0x20` is 1688 bytes reached with bank-sized offsets, which is
what a naive offset search finds first.

The seven fall into three groups.

**The exposure ratios**, bank+0x1c and bank+0x38. Command 0xb16 subcommand
0x2403 takes two floats from its payload and converts each with `fcvtzu ...,
#8`, so they are Q8.8. The module's own CLI names them: `--exp_ration` sets
`man_ration_l_s` and `man_ration_m_s`, and `--get_exp_ration` prints both
scaled by 1e4 with a "10000 as 1" note. Both read 1.0 here, which is what a
sensor delivering one exposure gives.

**The line buffer**, bank+0x7c and bank+0x8c for the address and bank+0x88 and
bank+0x98 for the stride. The address is written twice from one pointer, which
is why the pair is identical:

	bl   get_camera_server
	ldr  x0, [x0, #1584]
	str  w0, [x19, #124]      bank+0x7c
	bl   get_camera_server
	ldr  x0, [x0, #1584]
	str  w0, [x19, #140]      bank+0x8c

and that pointer is a fixed reserved physical address the vendor hard-codes at
0x145ef4 when `get_start_opt()->[40]` is set. **It is the vendor's carveout,
not ours.** The stride is `align(width * bit_depth / 8, 256)`, built at
0x197e48 with the bit depth coming from `convert_stream_format_to_isp_format`.

**A constant**, bank+0x114, stored immediately after the template install at
0x197b30, so it is a deliberate override of the image's zero rather than part
of it.

Needs the vendor library, which is not in the tree, and aarch64 objdump:

    kernel/scripts/isp/check-hdr-bank.py \\
        --library out/air-gather/vendor-root/usr/lib/libmpp_service.so
"""

import argparse
import importlib.util
import pathlib
import re
import shutil
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
OBJDUMP = 'aarch64-linux-gnu-objdump'

BANK = 0x1C00

# The frame and the sensor's raw bit depth, which the driver configures.
FRAME_W = 1920
BIT_DEPTH = 12
STRIDE_ALIGN = 256

# Q8.8, from the `fcvtzu ..., #8` at 0x1982c4 and 0x1982c8.
RATIO_SHIFT = 8
RATIO_REGS = {0x1C: 'middle-short', 0x38: 'long-short'}

# The line-buffer address pair, and the site that builds the reserved physical
# address they both carry.
ADDRESS_REGS = (0x7C, 0x8C)
RESERVED_PA = 0x02000000
RESERVED_SITE = (0x145EC0, 0x145EFC)

STRIDE_REGS = (0x88, 0x98)

# Written straight after the template install, so it overrides the image.
CONSTANT_REG = 0x114
CONSTANT_VALUE = 0x100
CONSTANT_SITE = (0x197B14, 0x197B3C)


def load_audit():
    """The driver's register tables, via audit-provenance.py."""
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def window(library, start, stop):
    if not shutil.which(OBJDUMP):
        sys.exit(f'{OBJDUMP} not found. It reads the vendor library, which is '
                 f'the only source for where these values are built.')

    out = subprocess.run(
        [OBJDUMP, '-d', '--no-show-raw-insn',
         f'--start-address={start:#x}', f'--stop-address={stop:#x}',
         str(library)],
        capture_output=True, text=True)
    if out.returncode:
        sys.exit(f'{OBJDUMP} failed on {library}: {out.stderr.strip()}')

    return out.stdout


def builds(text, value):
    """Whether a span builds this constant with a mov, in x or w form."""
    return bool(re.search(r'\bmov\s+[wx]\d+, #(0x0*%x|%d)\b'
                          % (value, value), text))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--library', required=True,
                    help='the vendor libmpp_service.so')
    args = ap.parse_args()

    library = pathlib.Path(args.library)
    if not library.exists():
        sys.exit(f'{library}: not found. The vendor library is a capture '
                 f'artifact and is deliberately not in the tree.')

    audit = load_audit()
    _image, final, _origin = audit.load_tables()

    wanted = (*RATIO_REGS, *ADDRESS_REGS, *STRIDE_REGS, CONSTANT_REG)
    missing = [f'{BANK + off:#06x}' for off in wanted if BANK + off not in final]
    if missing:
        sys.exit(f'the driver writes no value at {", ".join(missing)}, so '
                 f'there is nothing to prove the handlers against')

    failures = []

    print(f'exposure ratios, Q8.{RATIO_SHIFT}:\n')
    for off, what in RATIO_REGS.items():
        got = final[BANK + off]
        print(f'  {BANK + off:#06x} = {got:#010x}  '
              f'{got / (1 << RATIO_SHIFT):g}  the {what} ratio')
        if got != 1 << RATIO_SHIFT:
            failures.append(
                f'{BANK + off:#06x}: reads {got / (1 << RATIO_SHIFT):g}, not '
                f'the 1.0 a sensor delivering one exposure gives; if the '
                f'sensor now delivers two this is right and this check is not')

    stride = -(-(FRAME_W * BIT_DEPTH // 8) // STRIDE_ALIGN) * STRIDE_ALIGN
    print(f'\nline buffer:\n')
    print(f'  stride  align({FRAME_W} * {BIT_DEPTH} / 8, {STRIDE_ALIGN}) = '
          f'align({FRAME_W * BIT_DEPTH // 8}, {STRIDE_ALIGN}) = {stride:#x}')
    for off in STRIDE_REGS:
        got = final[BANK + off]
        print(f'  {BANK + off:#06x} = {got:#010x}')
        if got != stride:
            failures.append(f'{BANK + off:#06x}: the packer gives {stride:#x} '
                            f'and the driver installs {got:#010x}')

    reserved = window(library, *RESERVED_SITE)
    for off in ADDRESS_REGS:
        got = final[BANK + off]
        print(f'  {BANK + off:#06x} = {got:#010x}  the vendor\'s reserved '
              f'carveout, written twice from one pointer')
        if got != RESERVED_PA:
            failures.append(
                f'{BANK + off:#06x}: reads {got:#010x} where the vendor '
                f'hard-codes {RESERVED_PA:#010x}')

    if final[BANK + ADDRESS_REGS[0]] != final[BANK + ADDRESS_REGS[1]]:
        failures.append(
            f'{BANK + ADDRESS_REGS[0]:#06x} and {BANK + ADDRESS_REGS[1]:#06x} '
            f'differ, but the vendor writes both from one pointer')

    if not builds(reserved, RESERVED_PA):
        failures.append(
            f'no mov in {RESERVED_SITE[0]:#x}..{RESERVED_SITE[1]:#x} builds '
            f'{RESERVED_PA:#010x}, so this address is not the one the vendor '
            f'hard-codes')

    got = final[BANK + CONSTANT_REG]
    print(f'\n  {BANK + CONSTANT_REG:#06x} = {got:#010x}  constant, stored '
          f'immediately after the template install at {CONSTANT_SITE[0]:#x}')
    if got != CONSTANT_VALUE:
        failures.append(f'{BANK + CONSTANT_REG:#06x}: the handler stores '
                        f'{CONSTANT_VALUE:#x} and the driver installs '
                        f'{got:#010x}')
    elif not builds(window(library, *CONSTANT_SITE), CONSTANT_VALUE):
        failures.append(
            f'no mov in {CONSTANT_SITE[0]:#x}..{CONSTANT_SITE[1]:#x} builds '
            f'{CONSTANT_VALUE:#x}, so the post-install override is not there')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print(f'\nall seven follow from the frame, the sensor exposure count and '
          f'the vendor\'s reserved region\n')
    print(f'note: {BANK + ADDRESS_REGS[0]:#06x} and '
          f'{BANK + ADDRESS_REGS[1]:#06x} carry a physical address the vendor '
          f'reserves, not one this driver allocates. The stage does not run on '
          f'this camera module, so nothing reads them. Enabling it means '
          f'allocating a line buffer and writing that address instead.')

    return 0


if __name__ == '__main__':
    sys.exit(main())
