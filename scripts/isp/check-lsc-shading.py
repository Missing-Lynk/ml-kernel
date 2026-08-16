#!/usr/bin/env python3
"""
Prove lsc's shading enable and strength word at bank+0x40.

This register resisted longer than any other, for two reasons worth recording.

It sits past the end of the lsc ISP-init template image, which covers 0x4c00 to
0x4c3c only, so no vendor image backs it. And the store that fills its shadow
word uses a split base:

    mov x3, #0x3400
    add x0, x19, x3
    str w1, [x0, #196]        shadow container + 13508

so a search for a store at immediate 13508 finds nothing. The immediate is 196.

The word is a shading enable in bit 16 and a strength in the low byte, and the
vendor writes it from two branches of one routine:

    enabled     0x1b4e58  orr w1, w1, #0x10000       bit 16 set
                0x1b4e5c  fcvtzu w2, s0, #7          strength in Q7
                0x1b4e64  csel w2, w2, w3, ls        clamped to 128
    disabled    0x1b4c20  and w1, w1, #0xfffeffff    bit 16 clear
                0x1b4c24  mov w2, #0x80              the low byte forced to 128

both falling into the same mask-and-store at 0x1b4c2c. That is also what
explains the one confusing observation in the capture: the hardware read-back
of this bank reads 0x00000080 here, which is the disabled branch's output, and
the value pushed afterwards is the enabled branch's, with no read in between.

Both inputs come from the lens-shading record in the tuning file, which the
tuning manager copies at 0x174b10: `isp_get_tuning_manager()`, then the
per-instance record, then its blob pointer at +24, then 0xe9f8 bytes from blob
+0x9090. The two fields sit at the head of that region. `libmpi_vin.so`'s
public `AR_MPI_ISP_SetMeshShadingAttr` corroborates both sizes independently:
it copies 0xe9f8 for the manual table and 0x678 for the auto table, which is
the record stride.

Needs the tuning blob, which is a capture artifact and not in the tree:

    kernel/scripts/isp/check-lsc-shading.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import importlib.util
import pathlib
import struct
import sys

from blob_layout import Layout

_LAY = Layout.load()


HERE = pathlib.Path(__file__).resolve().parent

REGISTER = 0x4C40

# The lens-shading region in the tuning file, and the two fields at its head.
REGION = _LAY["lsc_gate"].offset
STRENGTH_AT, ENABLE_AT = _LAY["lsc_strength"].offset, _LAY["lsc_enable"].offset

# `fcvtzu ..., #7` at 0x1b4e5c, clamped against the 128 at 0x1b4e54.
STRENGTH_SHIFT = 7
STRENGTH_MAX = 128
ENABLE_BIT = 16

# What the disabled branch produces, which is what the bank read-back holds.
DISABLED_VALUE = 0x80


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


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tuning', required=True,
                    help='the sensor tuning blob the vendor ships')
    args = ap.parse_args()

    path = pathlib.Path(args.tuning)
    if not path.exists():
        sys.exit(f'{path}: not found. The tuning blob is a capture artifact '
                 f'and is deliberately not in the tree.')

    blob = path.read_bytes()
    audit = load_audit()
    image, final, _origin = audit.load_tables()

    if REGISTER not in final:
        sys.exit(f'the driver writes no value at {REGISTER:#06x}, so there is '
                 f'nothing to prove the packer against')

    failures = []
    strength = struct.unpack_from('<f', blob, STRENGTH_AT)[0]
    enable = struct.unpack_from('<I', blob, ENABLE_AT)[0]
    got = final[REGISTER]

    print(f'lens-shading region at blob+{REGION:#x}\n')
    print(f'  strength  blob+{STRENGTH_AT:#x} = {strength:g}')
    print(f'  enable    blob+{ENABLE_AT:#x} = {enable}\n')

    # fcvtzu truncates toward zero, and the strength is positive.
    quantised = min(int(strength * (1 << STRENGTH_SHIFT)), STRENGTH_MAX)
    expected = (bool(enable) << ENABLE_BIT) | quantised

    print(f'  Q{STRENGTH_SHIFT} of {strength:g} is {quantised}, capped at '
          f'{STRENGTH_MAX}, with the enable in bit {ENABLE_BIT}')
    print(f'  {REGISTER:#06x}  ({enable} << {ENABLE_BIT}) | {quantised:#x} = '
          f'{expected:#010x}  driver {got:#010x}')

    if got != expected:
        failures.append(f'{REGISTER:#06x}: the packer gives {expected:#010x} '
                        f'and the driver installs {got:#010x}')

    # No vendor image reaches this offset, which is why the value had to come
    # from the packer rather than from a template.
    if image.get(REGISTER) is not None:
        failures.append(
            f'{REGISTER:#06x}: a vendor image now covers this offset, so the '
            f'lsc template is longer than the 0x4c00 to 0x4c3c this assumes '
            f'and the register may have a static source after all')

    print(f'\n  the disabled branch would give {DISABLED_VALUE:#010x} instead, '
          f'which is what the bank read-back holds')
    if expected == DISABLED_VALUE:
        failures.append(
            f'{REGISTER:#06x}: the enabled and disabled branches now produce '
            f'the same value, so this no longer distinguishes them')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print('\nthe shading enable and strength both come from the tuning file, '
          'through the branch that has shading on')

    return 0


if __name__ == '__main__':
    sys.exit(main())
