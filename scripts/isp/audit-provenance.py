#!/usr/bin/env python3
"""
Where every ISP register value the driver writes comes from.

The standing rule: a configured value is parity when it is computed at runtime
from the vendor's tuning file, or carried from static data in the vendor's own
libmpp_service.so the way the vendor itself carries it. A value that exists only
because we recorded the vendor writing it is a recording, not a recovery, and it
is correct only at the operating point it was recorded at.

This script classifies every register the driver installs and reports what is
still unexplained, so the remaining recovery work is a list rather than a
feeling. It reads only checked-in sources, needs no device and no capture, and
is meant to be re-run as each stage lands.

Classes, in the order they are tested:

  derived from the blob   a stage in the driver computes it from the tuning
                          file at runtime
  library image           the value is the one the vendor's own submodule static
                          image carries for that register, per ar-isp-library.h
  explained               hand-recovered by reading the vendor packer, with the
                          finding recorded in EXPLAINED below
  stage gate              every bit the driver writes is a stage enable or
                          bypass recovered from the library, per
                          vendor-tables/ar-isp-gates.h
  zero write              the register is cleared; there is no vendor datum to
                          source
  frame geometry / grid   the value decomposes into the configured frame or
                          statistics-grid dimensions, which the driver owns
  vendor DMA address      a vendor buffer address the driver overwrites with
                          its own allocation at runtime
  UNEXPLAINED             everything else: the value exists only in the MMIO
                          write trace or in a live capture diff

The unexplained set is reported in two halves, because they are different work.
A register a submodule image covers but whose value the vendor moved away from
is one the module computes, so its owner and its packer are already known and
what is left is the arithmetic. A register no image covers has neither.

Exits non-zero when the unexplained count exceeds BASELINE, so the number
ratchets downward and a regression is a failure rather than a footnote.

    kernel/scripts/isp/audit-provenance.py
"""

import pathlib
import re
import sys
from collections import Counter, defaultdict

HERE = pathlib.Path(__file__).resolve().parent
DRIVERS = HERE.parent.parent / 'overlay' / 'drivers' / 'media' / 'artosyn'
DEFAULTS = DRIVERS / 'vendor-tables' / 'ar-isp-defaults.h'
LIBRARY = DRIVERS / 'vendor-tables' / 'ar-isp-library.h'
GATES = DRIVERS / 'vendor-tables' / 'ar-isp-gates.h'
MAIN = DRIVERS / 'ar-isp-main.c'
REGDIFF = HERE / 'isp-regdiff.py'

# The count of unexplained registers this tree is known to have. Lower it as
# stages are recovered; never raise it without saying why in the commit.
BASELINE = 136

# ar_isp_recovered is generated but no longer applied: every one of its entries
# is past the end of a submodule image, so none has a vendor value behind it.
# Reported so the dead table is visible until the generator drops it.
DEAD_TABLE = 'ar_isp_recovered'

# The frame and statistics-grid dimensions the driver configures. A register
# whose value is one of these, or a pair of them packed into halfwords, is
# carrying geometry rather than tuning data.
GEOMETRY = {1920, 1080, 960, 540, 36, 16, 1088, 1092}

# The vendor's MMZ physical range. Addresses here are vendor allocations that
# the driver replaces with its own at arm time.
DMA_LO, DMA_HI = 0x2A000000, 0x2C000000

# What a bank does to the shipped image, from the enable states in
# plans/isp-vendor-parity.md. This is what ranks the remaining work: a value on
# an enabled stage changes the picture, one on a quiescent stage cannot, and a
# statistics bank configures an accumulator the driver already owns.
IMAGE_PATH = {'cfa', 'dpc', 'rnr', 'ltm', 'de3d', 'drc', 'ccm1', 'rgb2yuv',
              'cnf', 'lnr', 'cm', 'cm2', 'lsc', 'wb', 'qgg', 'acm', 'blc'}
STATISTICS = {'rro_stats', 'rro_face_stats', 'raw_hist_stats', 'rgb_hist_stats',
              'rgb_max_stats', 'awbs_stats', 'af_stats', 'derolling_stats',
              'hdr_rro_0_stats', 'hdr_rro_1_stats', 'hdr_rro_face_stats',
              'hdr_awbs_stats'}


def bank_class(name: str) -> str:
    if name in IMAGE_PATH:
        return 'enabled image-path stage'

    if name in STATISTICS:
        return 'statistics accumulator'

    if name == 'base':
        return 'top-level control'

    return 'quiescent or disabled stage'


# Registers recovered by reading the vendor's packer rather than by class, with
# the finding. Each entry is a claim that has to survive review on its own.
EXPLAINED = {
    0x082C: 'cfa frame geometry: (width << 15) | (height << 2) | mode, which '
            'reassembles the measured value exactly at 1920 x 1080 mode 0',
    0x0858: 'cfa mode, bits 1:0, written by the same subcommand as 0x082c',
    0x0834: 'cfa hardware-written: a known value written here reads back as '
            'something else on two independent boots, and the packer stores '
            'to no such offset',
    0x08A8: 'cfa hardware-written, same evidence as 0x0834',
    0x3C74: 'cnf second strength copy, bit 0 is that copy\'s own enable',
}


def reg_arrays(path: pathlib.Path) -> dict[str, list[tuple[int, int]]]:
    """Every struct ar_isp_reg table in a header, by name."""
    arrays: dict[str, list[tuple[int, int]]] = {}
    current = None
    for line in path.read_text().splitlines():
        hit = re.match(r'static const struct ar_isp_reg (\w+)\[\]', line)
        if hit:
            current = hit.group(1)
            arrays[current] = []
            continue

        hit = re.match(r'\s*\{ (0x[0-9a-f]+), (0x[0-9a-f]+) \},', line)
        if hit and current:
            arrays[current].append((int(hit.group(1), 16),
                                    int(hit.group(2), 16)))

    return arrays


def load_tables() -> tuple[dict[int, int], dict[int, int], dict[int, str]]:
    """The library images, the driver's final value, and which table won."""
    arrays = reg_arrays(DEFAULTS) | reg_arrays(MAIN)
    library = dict(reg_arrays(LIBRARY)['ar_isp_library'])

    final: dict[int, int] = {}
    origin: dict[int, str] = {}
    # Applied in this order by ar_isp_configure; the last write wins.
    for name in ('ar_isp_kept', 'ar_isp_setup_1080p60', 'ar_isp_vendor_trim'):
        for off, val in arrays[name]:
            final[off] = val
            origin[off] = name

    return library, final, origin


def defines(path: pathlib.Path) -> dict[str, int]:
    return {m.group(1): int(m.group(2), 0) for m in
            re.finditer(r'#define\s+(\w+)\s+(0x[0-9a-fA-F]+|\d+)',
                        path.read_text())}


def table_body(path: pathlib.Path, name: str) -> str:
    hit = re.search(rf'{name}\[[^\]]*\]\s*=\s*\{{(.*?)\}};',
                    path.read_text(), re.S)
    if not hit:
        sys.exit(f'{path.name}: cannot find {name}[]')

    return hit.group(1)


def derived_registers() -> dict[int, str]:
    """Every register a driver stage recomputes from the tuning file."""
    out: dict[int, str] = {}

    def add(regs, stage):
        for reg in regs:
            out[reg] = stage

    rnr = defines(DRIVERS / 'ar-isp-rnr.h')
    add((rnr['AR_ISP_RNR_BANK'] + rnr['AR_ISP_RNR_LADDER'] + 4 * i
         for i in range(rnr['AR_ISP_RNR_REGS'])), 'rnr')
    add((rnr['AR_ISP_RNR_BANK'] + rnr['AR_ISP_RNR_TAIL'] + 4 * i
         for i in range(rnr['AR_ISP_RNR_TAIL_REGS'])), 'rnr')

    lnr = defines(DRIVERS / 'ar-isp-lnr.h')
    skipped = {0x3D10, 0x3D14}
    add((a for a in (lnr['AR_ISP_LNR_BANK'] + 4 * i
                     for i in range(lnr['AR_ISP_LNR_REGS']))
         if a not in skipped), 'lnr')

    de3d = defines(DRIVERS / 'ar-isp-de3d.h')
    body = table_body(DRIVERS / 'ar-isp-de3d.h', 'ar_isp_de3d_regs')
    add((de3d['AR_ISP_DE3D_BANK'] + int(m, 16)
         for m in re.findall(r'\{\s*(0x[0-9a-f]+),', body)), 'de3d')

    cfa = defines(DRIVERS / 'ar-isp-cfa.h')
    body = table_body(DRIVERS / 'ar-isp-cfa.h', 'ar_isp_cfa_runs')
    for reg, _word, count in re.findall(
            r'\{\s*(0x[0-9a-f]+),\s*(0x[0-9a-f]+),\s*(\d+)\s*\}', body):
        add((cfa['AR_ISP_CFA_BANK'] + int(reg, 16) + 4 * k
             for k in range(int(count))), 'cfa')

    cnf = defines(DRIVERS / 'ar-isp-cnf.h')
    add((cnf['AR_ISP_CNF_STRENGTH_REG'], cnf['AR_ISP_CNF_NORM_REG_A'],
         cnf['AR_ISP_CNF_NORM_REG_B']), 'cnf')
    add((cnf['AR_ISP_CNF_STATIC_REG'] + 4 * i
         for i in range(cnf['AR_ISP_CNF_STATIC_REGS'])), 'cnf')

    colour = defines(DRIVERS / 'ar-isp-colour.h')
    ccm_init = DRIVERS / 'vendor-tables' / 'ar-isp-ccm-init.h'
    for bank, name in ((colour['AR_ISP_CCM1_BANK'], 'ar_isp_ccm1_init'),
                       (colour['AR_ISP_CCM2_BANK'], 'ar_isp_ccm2_init')):
        words = len(re.findall(r'0x[0-9a-f]{8}', table_body(ccm_init, name)))
        add((bank + 4 * i for i in range(words)), 'ccm')

    add((int(m, 16) for m in re.findall(
        r'\{\s*(0x[0-9a-f]+),\s*0x[0-9a-f]+\s*\}',
        (DRIVERS / 'ar-isp-dpc.h').read_text())), 'dpc')

    # rgb2yuv packs one of the library's four CSC matrices rather than reading
    # the tuning file, the same provenance as the CCM init blocks above.
    csc = defines(DRIVERS / 'vendor-tables' / 'ar-isp-rgb2yuv.h')
    add((csc['AR_ISP_RGB2YUV_BANK'] + 4 * i
         for i in range(csc['AR_ISP_RGB2YUV_REGS'])), 'rgb2yuv')

    return out


def gate_masks() -> dict[int, int]:
    """
    Register to the mask of bits the recovered stage gates account for.

    From vendor-tables/ar-isp-gates.h, which isp-gates.py generates out of the
    library's own read-modify-writes. A register is only reclassified when the
    gates cover every bit the driver writes to it: the top-level word holds one
    or two bits for each of ten stages plus bit 1, which no module was seen to
    write, so it stays unexplained and the covered bits are reported instead of
    being claimed as the whole word.
    """
    out: defaultdict[int, int] = defaultdict(int)
    body = GATES.read_text()

    for reg, setm, clrm in re.findall(
            r'\{\s*(0x[0-9a-f]{4}),\s*(0x[0-9a-f]{8}),\s*(0x[0-9a-f]{8}),',
            body):
        out[int(reg, 16)] |= int(setm, 16) | int(clrm, 16)

    return dict(out)


def bank_lookup() -> list[tuple[int, str]]:
    banks = [(int(base, 16), name) for base, name in re.findall(
        r'\(0x([0-9A-Fa-f]+),\s*"(\w+)"\)', REGDIFF.read_text())]
    return sorted(banks, reverse=True)


def is_geometry(value: int) -> bool:
    if value in GEOMETRY:
        return True

    high, low = value >> 16, value & 0xFFFF
    if high in GEOMETRY and low in GEOMETRY:
        return True

    return (high in GEOMETRY and not low) or (low in GEOMETRY and not high)


def main() -> int:
    library, final, origin = load_tables()
    derived = derived_registers()
    gates = gate_masks()
    banks = bank_lookup()

    def bank_of(off: int) -> str:
        for base, name in banks:
            if off >= base:
                return name

        return 'base'

    def classify(off: int) -> str:
        value = final[off]
        if off in derived:
            return 'derived from the blob'

        if library.get(off) == value:
            return 'library image'

        if off in EXPLAINED:
            return 'explained'

        # After the zero test below would be wrong for a gate that happens to
        # read zero, but claiming one is weaker than calling it a zero write:
        # a cleared register has no vendor datum behind it either way, so the
        # conservative class keeps it.
        if value and off in gates and value & ~gates[off] == 0:
            return 'stage gate'

        if not value:
            return 'zero write'

        if DMA_LO <= value < DMA_HI:
            return 'vendor DMA address'

        if is_geometry(value):
            return 'frame geometry / grid'

        return 'UNEXPLAINED'

    tally: Counter[str] = Counter()
    open_regs: defaultdict[str, list[int]] = defaultdict(list)
    overridden: list[int] = []
    for off in final:
        kind = classify(off)
        tally[kind] += 1
        if kind == 'UNEXPLAINED':
            open_regs[bank_of(off)].append(off)
            if off in library:
                overridden.append(off)

    order = ['derived from the blob', 'library image', 'explained',
             'stage gate', 'zero write', 'frame geometry / grid',
             'vendor DMA address', 'UNEXPLAINED']
    print(f'ISP registers the driver writes: {len(final)}\n')
    for kind in order:
        print(f'  {tally[kind]:5}  {kind}')

    unexplained = tally['UNEXPLAINED']
    backed = len(final) - unexplained
    print(f'\n  {backed:5}  traceable to the tuning file, the library, or the '
          f'driver\'s own configuration')
    print(f'  {unexplained:5}  UNEXPLAINED: the value exists only in a '
          f'recording')
    print(f'           {len(overridden):5}  of them inside a submodule image the '
          f'vendor moved away from, so the owning module is known')
    print(f'           {unexplained - len(overridden):5}  of them outside every '
          f'image located so far')

    groups: defaultdict[str, list[tuple[str, list[int]]]] = defaultdict(list)
    for name, regs in open_regs.items():
        groups[bank_class(name)].append((name, regs))

    print('\nunexplained, grouped by what the bank does to the image:')
    for kind in ('enabled image-path stage', 'statistics accumulator',
                 'quiescent or disabled stage', 'top-level control'):
        rows = sorted(groups.get(kind, []), key=lambda kv: -len(kv[1]))
        count = sum(len(regs) for _name, regs in rows)
        print(f'\n  {kind}: {count}')
        for name, regs in rows:
            listed = ' '.join(f'{r:#06x}' for r in sorted(regs)[:8])
            more = ' ...' if len(regs) > 8 else ''
            print(f'    {name:<20}{len(regs):4}   {listed}{more}')

    dead = len(reg_arrays(DEFAULTS)[DEAD_TABLE])
    print(f'\n{DEAD_TABLE} still holds {dead} registers and is no longer '
          f'applied: every one lies past the end of a submodule image')

    if unexplained > BASELINE:
        print(f'\nregression: {unexplained} unexplained against a baseline of '
              f'{BASELINE}')
        return 1

    if unexplained < BASELINE:
        print(f'\nimproved: {unexplained} against a baseline of {BASELINE}; '
              f'lower BASELINE to lock it in')

    return 0


if __name__ == '__main__':
    sys.exit(main())
