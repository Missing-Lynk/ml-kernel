#!/usr/bin/env python3
"""
The ISP pipeline as the driver configures it: every stage, in order, running or
not, and where its register values come from.

Three checked-in sources are joined here. `vendor-tables/ar-isp-gates.h` says
which register and which bits gate each stage, recovered from the vendor
library's own read-modify-writes. The driver's register tables say what those
registers end up holding. `audit-provenance.py` says where each stage's values
come from. Evaluating the gate against the final value is what makes the
running column a measurement of the shipped configuration rather than a note
someone kept up to date by hand.

A stage that is gated off still has its registers written, and several here do:
the driver reproduces the vendor's whole register state, and the vendor
installs tables for stages it then leaves disabled. Carrying a table and
running a stage are separate questions and this prints both.

    kernel/scripts/isp/isp-pipeline.py
    kernel/scripts/isp/isp-pipeline.py --markdown
"""

import argparse
import importlib.util
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
DRIVERS = HERE.parent.parent / 'overlay' / 'drivers' / 'media' / 'artosyn'
GATES = DRIVERS / 'vendor-tables' / 'ar-isp-gates.h'

# The top-level control word, which gates ten stages between them. A stage
# carrying this as its bank keeps its values in a DMA page instead.
BASE_BANK = 0x0000

# Pipeline order, and what each stage does to the image. The order is the
# vendor's own, read off the submodule list in libmpp_service.so; the
# descriptions are what the stage computes, not what it is named after.
#
# Stages the driver never writes a register for are absent: this is the shipped
# pipeline, not the block's full submodule catalogue.
PIPELINE = [
    ('blc', 'sensor correction',
     'subtracts the per-channel black level, on CVISP bank 0x4200 rather '
     'than in the ISP register file'),
    ('gib', 'sensor correction',
     'green imbalance between the two Bayer greens'),
    ('dpc', 'sensor correction',
     'replaces defective pixels'),
    ('lsc', 'sensor correction',
     'lens shading: a 10x10 gain grid that lifts the corners'),
    ('digigain1', 'sensor correction',
     'digital gain ahead of the noise stages'),
    ('compander', 'sensor correction',
     'companding curve between the sensor range and the pipe'),
    ('decompander', 'sensor correction',
     'the inverse curve'),
    ('rnr', 'noise reduction',
     'radial noise reduction, strength rising toward the corners'),
    ('birnr', 'noise reduction',
     'bilateral noise reduction in the Bayer domain'),
    ('lee_lnr', 'noise reduction',
     'luma noise reduction, bank 0x3cc8, which the register map calls lnr'),
    ('de3d', 'noise reduction',
     'temporal noise reduction across frames, the motion-sensitive stage'),
    ('raw_3dnr', 'noise reduction',
     'raw-domain temporal filter'),
    ('cfa', 'demosaic',
     'Bayer to RGB, the point where the image gains three channels'),
    ('wb', 'colour',
     'per-channel white balance gains'),
    ('ccm1', 'colour',
     'the 3x3 colour correction matrix'),
    ('ccm2', 'colour',
     'a second colour matrix'),
    ('cm', 'colour', 'colour manipulation'),
    ('cm2', 'colour', 'a second colour manipulation block'),
    ('acm', 'colour', 'adaptive colour manipulation'),
    ('cnf', 'colour', 'chroma noise filter'),
    ('lut3d', 'colour', 'a 3D colour lookup table in four DMA banks'),
    ('qgg', 'colour', 'quadratic green gain'),
    ('lms', 'colour', 'long/medium/short colour space conversion'),
    ('gamma', 'tone', 'the gamma transfer curve, fetched as a DMA page'),
    ('drc', 'tone',
     'dynamic range compression, fetched as a DMA page'),
    ('ltm', 'tone',
     'local tone mapping: 64 per-tile transfer curves, recomputed per frame'),
    ('gtm2', 'tone',
     'global tone mapping, sharing ltm bank 0x2800'),
    ('rgb2yuv', 'colour space',
     'RGB to YUV, the point where the image becomes luma and chroma'),
    ('binning_filter', 'geometry', 'binning ahead of the scaler'),
    ('rro_stats', 'statistics',
     'the 36x16 zone grid AE meters from'),
    ('face_rro_stats', 'statistics',
     'a second zone grid over a smaller window'),
    ('raw_his_stats', 'statistics',
     'the Bayer histogram, 128 bins by 4 lanes'),
    ('awbs_stats', 'statistics', 'the white-balance accumulator'),
    ('af_stats', 'statistics', 'the autofocus accumulator'),
    ('derolling_stats', 'statistics', 'rolling-shutter statistics'),
    ('rgb_his_stats', 'statistics', 'the RGB histogram'),
    ('rgb_max_stats', 'statistics', 'per-channel maxima'),
]

# The hdr and ir families take a second sensor exposure and an infrared
# channel. Neither exists on this camera module, so their banks are named here
# only so a reader who meets them in a register dump knows what they are.
ABSENT_FAMILIES = {
    'hdr': 'the second-exposure HDR path, which needs a sensor exposure this '
           'module does not produce',
    'ir': 'the infrared path, which needs an IR channel this module does not '
          'have',
}


def load_audit():
    """audit-provenance.py's tables, reused rather than reimplemented."""
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def gate_table():
    """
    Stage name to (bank, [(reg, set_mask, kind, polarity)]), from
    ar-isp-gates.h.

    The polarity is the trailing comment isp-gates.py writes on each gate, and
    it is how far the recovery got. `code` means the library's own branch said
    which side means running. `paired` means it was inferred from the tuning
    flag and a live register agreeing, which is weaker: an inverted polarity
    turns a verdict into its opposite, so the two are kept apart rather than
    averaged into one confident-looking answer.
    """
    text = GATES.read_text()
    arrays = {}
    for hit in re.finditer(
            r'static const struct ar_isp_gate (\w+)\[\] = \{(.*?)\n\};',
            text, re.S):
        arrays[hit.group(1)] = [
            (int(reg, 16), int(setm, 16), kind.replace('AR_ISP_GATE_', ''),
             polarity)
            for reg, setm, _clr, kind, polarity in re.findall(
                r'\{\s*(0x[0-9a-f]{4}),\s*(0x[0-9a-f]{8}),'
                r'\s*(0x[0-9a-f]{8}),\s*(AR_ISP_GATE_\w+)[^/]*/\*\s*(\w+)',
                hit.group(2))]

    stages = {}
    for name, bank, blob, array, _n, _flags in re.findall(
            r'\{ "(\w+)", (0x[0-9a-f]+), (0x[0-9a-f]+), (\w+), (\d+), '
            r'([^}]*)\}', text):
        stages[name] = (int(bank, 16), arrays[array], int(blob, 16))

    return stages


def blob_states(path):
    """
    Each stage's enable flag as the sensor tuning file stores it.

    This is the second, independent reading of whether a stage runs. The
    register verdict comes from the value the driver installs; this one comes
    from the file the vendor configured the stage out of, and the two are
    recovered from different places. Where they agree, an inferred gate
    polarity is corroborated by something that cannot depend on it. Stages
    carrying AR_ISP_STAGE_NO_BLOB read no flag and are absent here.
    """
    blob = pathlib.Path(path).read_bytes()
    out = {}
    for name, (_bank, _gates, off) in gate_table().items():
        if off and off + 4 <= len(blob):
            out[name] = 'runs' if int.from_bytes(
                blob[off:off + 4], 'little') else 'off'

    return out


def stage_state(gates, final):
    """
    Whether a stage runs, from its gates against the driver's final values.

    A bit-enable gate runs when its bits are set and a bit-bypass gate runs
    when they are clear; a word gate stops the stage at zero. Gates whose kind
    the library did not settle, and gates on a register the driver never
    writes, carry no verdict. A stage whose resolvable gates disagree carries
    none either, because the disagreement is the finding.

    A verdict resting on any inferred polarity is marked, because inverting a
    polarity inverts the verdict.
    """
    verdicts = set()
    inferred = False
    for reg, mask, kind, polarity in gates:
        value = final.get(reg)
        if value is None or kind == 'UNKNOWN':
            continue

        if kind == 'BIT_ENABLE':
            verdicts.add('runs' if value & mask else 'off')
        elif kind == 'BIT_BYPASS':
            verdicts.add('off' if value & mask else 'runs')
        elif kind == 'WORD':
            verdicts.add('off' if value == 0 else 'runs')

        inferred = inferred or polarity == 'paired'

    if len(verdicts) == 1:
        return verdicts.pop(), inferred

    return ('undecided' if verdicts else 'no gate recovered'), False


def reconcile(verdict, inferred, blob_state):
    """
    One verdict out of the register gate and the tuning file's own flag.

    An inferred polarity that the tuning file agrees with is corroborated by a
    source that cannot depend on it, so it stops being a caveat. One the file
    contradicts is reported as the conflict it is rather than resolved here.
    """
    if blob_state is None:
        return verdict + (' (inferred)' if inferred else '')

    # No register gate was recovered, so the file is the only reading there is.
    if verdict == 'no gate recovered':
        return f'{blob_state} (tuning file only)'

    # The register gates contradicted each other. The file is a third reading,
    # reported beside them rather than used to declare a winner.
    if verdict == 'undecided':
        return f'undecided, tuning file says {blob_state}'

    if blob_state != verdict:
        return f'{verdict}, tuning file says {blob_state}'

    return verdict + (' (file agrees)' if inferred else '')


def provenance(audit, library, final, derived, masks, bank, upper):
    """
    Where a stage's register values come from, as the audit classifies them.

    Reported as the class covering most of the stage's registers, with the
    unexplained count beside it, so a stage that is mostly derived but has a
    handful of recordings left reads as exactly that.
    """
    kinds = {}
    unexplained = 0
    for off, value in final.items():
        if not bank <= off < upper:
            continue

        if off in derived:
            kind = 'derived from the tuning file'
        elif library.get(off) == value:
            kind = 'the vendor library image'
        elif off in audit.EXPLAINED:
            kind = 'read out of the vendor packer'
        elif value and off in masks and value & ~masks[off] == 0:
            kind = 'a stage gate'
        elif not value:
            kind = 'cleared'
        elif audit.DMA_LO <= value < audit.DMA_HI:
            kind = 'a driver DMA allocation'
        elif audit.is_geometry(value):
            kind = 'the frame geometry'
        else:
            kind = 'a recording of the vendor'
            unexplained += 1

        kinds[kind] = kinds.get(kind, 0) + 1

    if not kinds:
        return 'no ISP register written', 0, 0

    total = sum(kinds.values())
    best = max(kinds, key=lambda k: kinds[k])

    return best, unexplained, total


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tuning',
                    help='the sensor tuning blob, to cross-check each gate '
                         'against the flag the vendor configured it from')
    ap.add_argument('--markdown', action='store_true',
                    help='emit the table as markdown for docs/camera-stack.md')
    args = ap.parse_args()

    audit = load_audit()
    library, final, _origin = audit.load_tables()
    derived = audit.derived_registers()
    masks = audit.gate_masks()
    banks = audit.bank_lookup()
    gates = gate_table()
    blob = blob_states(args.tuning) if args.tuning else {}

    bases = sorted({base for base, _name in banks})
    by_name = {name: base for base, name in banks}

    def span(bank):
        after = [b for b in bases if b > bank]
        return bank, after[0] if after else 0x10000

    rows = []
    for name, group, what in PIPELINE:
        # The bank map is preferred over the gate table's bank, because several
        # stages are gated from the top-level control word at 0x0000 and carry
        # that as their bank. Their registers live elsewhere, or in a DMA page
        # with no bank at all.
        if name in gates:
            gate_bank, stage_gates, _blob = gates[name]
            bank = by_name.get(name, gate_bank)
        elif name in by_name:
            # A stage the library gates from the tuning file rather than from
            # a register, so isp-gates.py had no read-modify-write to recover.
            # It still has a bank and a provenance, which is most of the row.
            bank, stage_gates = by_name[name], []
        else:
            sys.exit(f'{name}: no stage of that name in {GATES.name} or in the '
                     f'register bank map. The pipeline list and its sources '
                     f'have diverged, and a silent skip here would print a '
                     f'pipeline missing a stage.')

        verdict, inferred = stage_state(stage_gates, final)
        state = reconcile(verdict, inferred, blob.get(name))
        # Bank 0x0000 is the top-level control word, which every stage shares.
        if bank != BASE_BANK:
            lo, hi = span(bank)
            source, open_regs, total = provenance(audit, library, final,
                                                  derived, masks, lo, hi)
        else:
            source, open_regs, total = 'a DMA page', 0, 0
        rows.append((name, group, state, what, source, open_regs, total))

    if args.markdown:
        print('| Stage | Runs | What it does | Where its values come from |')
        print('|---|---|---|---|')
        group = None
        for name, grp, state, what, source, open_regs, total in rows:
            if grp != group:
                group = grp
                print(f'| **{grp}** | | | |')

            note = f' ({open_regs} still recorded)' if open_regs else ''
            print(f'| `{name}` | {state} | {what} | {source}{note} |')

        return 0

    width = max(len(r[0]) for r in rows)
    group = None
    for name, grp, state, what, source, open_regs, total in rows:
        if grp != group:
            group = grp
            print(f'\n{grp}')

        note = f'  [{open_regs}/{total} still a recording]' if open_regs else ''
        print(f'  {name:<{width}}  {state:<34}{source}{note}')
        print(f'  {"":<{width}}  {what}')

    print()
    for family, why in ABSENT_FAMILIES.items():
        print(f'{family}: absent from the shipped pipeline, {why}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
