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
  stage switched off      the register belongs to a stage the gate and the
                          tuning file independently agree is disabled, so it
                          has no operating point to be wrong at; see
                          DISABLED_STAGES, and note this reverts to a recording
                          the day the stage is enabled
  hardware-owned          no vendor code writes it, so the value the driver
                          carries is whatever the block had put there when the
                          state was captured; see HARDWARE_OWNED. This is the
                          one class where copying the vendor is probably wrong
                          rather than right, and none of it is confirmed
                          against the device yet
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
BASELINE = 1

# The frame and statistics-grid dimensions the driver configures. A register
# whose value is one of these, or a pair of them packed into halfwords, is
# carrying geometry rather than tuning data.
# 1928 is the frame width padded by 8, which de3d's spatial filter needs for
# its border, and 241 and 135 are that padded frame divided by 8, its block
# grid: 1928/8 = 241 exactly and 1080/8 = 135 exactly.
GEOMETRY = {1920, 1080, 960, 540, 36, 16, 1088, 1092, 1928, 241, 135}

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

    # Descriptor lengths, in units of 32 bytes, sitting one word above the
    # pointer they describe. Each reproduces the fetch the driver itself sets
    # up, so the value follows from the allocation rather than from the trace.
    0x0034: 'gamma descriptor 0 length: 0x80 units of 32 bytes is the 0x1000 '
            'the block fetches, which is what ar-isp-tables.c builds',
    0x0044: 'gamma descriptor 1 length, the same 0x1000 fetch; the three gamma '
            'slots are channel aliases on one buffer',
    0x0054: 'gamma descriptor 2 length, the same 0x1000 fetch',
    0x0064: 'DRC descriptor length: 0x100 units of 32 bytes is the 0x2000 DRC '
            'page, the size of the vendor template ar-isp-tables.c rebuilds',

    # Module-local descriptor valid bits. Each sits at a fixed offset from the
    # pointer its own module publishes, and the publish site is recovered.
    0x1C60: 'HDR page descriptor valid bit, set when 0x1c6c is published',
    0x1E40: 'hdr_lsc descriptor valid bit, set when 0x1e38 is published',
    0x5818: 'lut3d bank 0 descriptor valid bit, set when 0x5810 is published',
    0x5830: 'lut3d bank 1 descriptor valid bit, set when 0x5828 is published',
    0x5848: 'lut3d bank 2 descriptor valid bit, set when 0x5840 is published',
    0x5860: 'lut3d bank 3 descriptor valid bit, set when 0x5858 is published',
}

# The rro zone-metering engine, instantiated five times. Its layout holds
# across all five, the three HDR instances carrying it eight bytes further into
# their bank, and check-rro-engine.py proves both the layout and the relations
# below. rro_face_stats meters a different window, so it satisfies them with a
# different zone and they are a measurement rather than a restatement.
RRO_INSTANCES = (('rro_stats', 0x6400, 0), ('rro_face_stats', 0x64C8, 0),
                 ('hdr_rro_0_stats', 0x1D20, 8),
                 ('hdr_rro_1_stats', 0x1D78, 8),
                 ('hdr_rro_face_stats', 0x1F40, 8))

# Engine-relative offset to what recovers it. The zone dimensions at 0x24 and
# 0x28 are deliberately absent: where they come from is not recovered, and only
# the fields that follow from them are claimed here.
RRO_FIELDS = {
    0x30: 'rro engine: the zone width written a second time, tracking '
          'engine+0x24',
    0x38: 'rro engine: the zone height written a second time, tracking '
          'engine+0x28',
    0x3c: 'rro engine: the saturation threshold, 0xff on every instance',
    0x48: 'rro engine: the accumulator enable',
}

for _name, _bank, _shift in RRO_INSTANCES:
    for _off, _why in RRO_FIELDS.items():
        EXPLAINED.setdefault(_bank + _shift + _off, _why)

# ltm's reciprocal tile areas. The packer at 0x18c418 divides 2^26 by each tile
# area with sdiv, so the block can normalise a tile histogram by multiplying.
# check-ltm-tiles.py reproduces all eight, and solves the tile grid they imply:
# exactly one grid up to 32 by 32 fits, and its tile count is the 64 curves the
# coefficient page independently holds.
LTM_BANK = 0x2800
LTM_RECIPROCALS = {
    0x10: 'the tile area', 0x14: 'the tile area doubled',
    0x18: 'the tile area quadrupled', 0x1c: 'the last column',
    0x20: 'the last column doubled', 0x24: 'the last row',
    0x28: 'the last row doubled', 0x2c: 'the corner tile',
}

for _off, _what in LTM_RECIPROCALS.items():
    EXPLAINED.setdefault(
        LTM_BANK + _off,
        f'ltm: 2^26 divided by {_what}, precomputed so the block normalises a '
        f'tile histogram by multiplying')

# cm2's two clamp windows and their reciprocals, all six from one 24-byte
# record of a ladder in the tuning blob. conver_cm2_tuning_pra_to_snr1_reg at
# 0x1a0578 stores the four bounds verbatim as a single 16-byte q0, then divides
# 1024 by each window width with sdiv and stores the pair three words along.
# check-cm2-ladder.py reproduces the bounds from the blob and the reciprocals
# from the bounds.
#
# The ladder is indexed by the AEC trigger, its interpolation gate is set, and
# the second low bound is an interpolation between rows rather than any row's
# value. So bank+0x24 and bank+0x30 are an operating point, not a constant:
# they move with the scene, and 1006 is what the capture caught.
CM2_BANK = 0x4800
CM2_LADDER = {
    0x1C: 'the first window\'s low bound, constant down the ladder',
    0x20: 'the first window\'s high bound, constant down the ladder',
    0x24: 'the second window\'s low bound, interpolated between ladder rows '
          'by the AEC trigger, so this value is one operating point',
    0x28: 'the second window\'s high bound, constant down the ladder',
    0x2C: '1024 divided by the first window\'s width, sdiv truncating',
    0x30: '1024 divided by the second window\'s width, sdiv truncating, so it '
          'moves with bank+0x24',
}

for _off, _why in CM2_LADDER.items():
    EXPLAINED.setdefault(CM2_BANK + _off, f'cm2: {_why}')

# The ISP top-level bank, which has no submodule and so no static image behind
# it. The ISP open path writes it directly, reaching it as g_hw_info+4 through
# ar_dev_pa2va. That base is the discriminator: the same offsets exist on VIF
# at g_hw_info+12, and most writers of these offsets in the library are VIF
# ones. check-isp-base.py proves the geometry against the configured frame and
# rebuilds every constant from the mov/movk pair in the library that builds it.
EXPLAINED.update({
    0x0004: 'ISP top level: {mode[27:26], height[25:13], width[12:0]}, the '
            'layout the packing site at 0x1d1fdc builds field by field. '
            'Decodes to the frame padded by four in each dimension, which is '
            'the input the VIF measures',
    0x0008: 'ISP top level: the active frame under the layout proven on '
            '0x0004, which it reproduces exactly at the configured size and '
            'with which it shares an untouched top nibble. No pointer to '
            'base+0x8 is formed anywhere in the library; it is committed from '
            'the register shadow at 0x1d4670 and its producer is not found, '
            'so the layout is carried from its neighbour rather than proven '
            'here',
    0x000C: 'ISP top level: a 3-bit selector cleared and bit 5 set, by the '
            'read-modify-write pair at 0x1c9614. Bit 5 moves in lockstep with '
            '0x4404 bit 0 and 0x4834 bit 2',
    0x0090: 'ISP top level: a bare constant stored by the open path at '
            '0x1d3a54, with no derivation behind it',
    0x00B8: 'ISP top level: the open path writes all ones at 0x1d3924, reads '
            'back, and clears bit 14',
    0x00C4: 'ISP top level: a bare constant stored at 0x1d397c',
    0x00C8: 'ISP top level: the first interrupt-enable mask, a constant on the '
            'branch get_start_opt()->[12308] selects; the other branch is a '
            'debug one enabling everything',
    0x00D0: 'ISP top level: the second interrupt-enable mask, stored by the '
            'same branch at 0x1d3e0c',
})

# af_stats. The metering window is a region of interest, a four-float constant
# in the library at 0x36ddd0 read as fractions of half the frame, and the three
# geometry registers are that region expressed four ways. The mode word comes
# from a tuning-blob ladder indexed by the AEC trigger. check-af-stats.py
# rebuilds all of it, and fixes the ladder alignment by rebuilding 0x7404,
# which the vendor image independently carries.
EXPLAINED.update({
    0x7408: 'af_stats: six bitfields from the tuning ladder at blob+0xd5bd0, '
            'packed by isp_sub_process_reg_compute at 0x1f6178',
    0x740C: 'af_stats: the metering offset, (width/2 * roi[0]) and '
            '(height/2 * roi[1]) in two 13-bit fields',
    0x7410: 'af_stats: the skip, the metering region over 16 and 9, in two '
            '9-bit fields',
    0x7414: 'af_stats: the block size, the metering region over 17 and 10, in '
            'two 10-bit fields',
    0x7570: 'af_stats: a constant stored by the per-frame re-arm at 0x1f55bc, '
            'one phase of an A/B toggle on priv+824 whose other phase stores '
            'zero here',
})

# hdr. The one bank with no RAM shadow: its map handler stores the mapped bank
# VA at priv+568 and the handlers write it directly. check-hdr-bank.py proves
# all seven.
#
# 0x1c7c and 0x1c8c hold a physical address the VENDOR reserves, hard-coded at
# 0x145ef4, not one this driver allocates. **They are parity only because the
# stage does not run on this camera module, which has one exposure.** Enabling
# HDR means allocating a line buffer and writing that address instead; copying
# the vendor's would point the block at memory this system has not carved out.
EXPLAINED.update({
    0x1C1C: 'hdr: the middle-short exposure ratio in Q8.8, from the 0xb16 '
            'subcommand 0x2403 payload; 1.0 for a single-exposure sensor',
    0x1C38: 'hdr: the long-short exposure ratio in Q8.8, the sibling store at '
            '0x1982d8; 1.0 for a single-exposure sensor',
    0x1C7C: 'hdr: the line-buffer physical address, the vendor\'s own reserved '
            'carveout from get_camera_server()+1584, hard-coded at 0x145ef4. '
            'Inert only because the stage does not run here',
    0x1C8C: 'hdr: the same line-buffer address, written a second time from one '
            'pointer, which is why the pair is identical',
    0x1C88: 'hdr: the line-buffer stride, align(width * bit_depth / 8, 256), '
            'built at 0x197e48',
    0x1C98: 'hdr: the same stride, written a second time',
    0x1D14: 'hdr: a constant stored immediately after the template install at '
            '0x197b30, so it overrides the image rather than being part of it',
})

# The statistics engines write their banks directly through the mapped VA,
# with no shadow. check-rro-engine.py derives the zones these sit alongside.
EXPLAINED.update({
    0x64DC: 'rro_face_stats: the metering x offset, the frame times the 0.3 '
            'quadruple at library 0x36e3c0, rounded to even at 0x1f8324',
    0x64E0: 'rro_face_stats: the metering y offset, the same 0.3 applied to '
            'the height',
    0x64F0: 'rro_face_stats: the y skip, the metered height over the 36-row '
            'grid, ceilinged then rounded to even at 0x1f8378',
    0x6400: 'rro_stats: the image value with bit 1 set by the read-modify-'
            'write at 0x201b10, gated on get_start_opt()->[12268]; the other '
            'branch clears the same bit',
    0x6000: 'raw_hist_stats: the same bit and the same gate, at 0x1fef64',
    0x1D84: 'hdr_rro_1_stats: a constant stored at 0x1fba48. The two non-HDR '
            'instances write 1 in the same engine field, so this is the HDR '
            'instance\'s mode value',
})

# drc's three coefficient groups and its strength. The selector at 0x1a45e0 is
# the cm2 pattern again: a ladder in the tuning blob at 0xd17b1c, stride 0xc8c,
# six rows with AEC abscissas at 0x17a9c, and four paths through it. The packer
# at 0x1a4200 writes the shadow, which is pushed to the bank with isp_memcpy.
#
# The three groups are 15 taps packed one per byte, and all three read the same
# delta kernel out of the blob: seven zeros, 255, seven zeros. That is why
# three different image ladders collapse to one value, and why the value is
# 0xff000000 rather than anything saturating.
EXPLAINED.update({
    0x3004: 'drc: byte 3 of the first coefficient group, the 255 of a delta '
            'kernel at tuning record+0x80c, packed at 0x1a42b0',
    0x3014: 'drc: byte 3 of the second group, the same delta kernel at '
            'record+0x848, packed at 0x1a4368',
    0x3024: 'drc: byte 3 of the third group, the same delta kernel at '
            'record+0x884, packed at 0x1a4424',
    0x3060: 'drc: bits 8:0 are the strength from tuning record+0x808, which '
            'reads 255, 255, 255, 200, 150, 100 down the six ladder rows; the '
            'installed 150 is row 4, and the upper half is the image value '
            'the mask at 0x1a4484 preserves',
})

# ltm's control word, which the configure and enable paths build from the
# image by read-modify-write rather than writing whole.
EXPLAINED.update({
    0x2800: 'ltm: the image 0x000207af with the 8x8 tile grid at 0x18c2ec, '
            'then 0x20000, 0x40000, the low nibble cleared, and the two stage '
            'enables at 0x18d484 and 0x18d490, giving exactly the installed '
            'value',
    0x2830: 'ltm: a constant, stored 0x1f at 0x18c350 and then re-read and '
            'or-ed with 0x1f00 at 0x18c370, so the image value cannot survive',
})

# The rest of the ISP top-level bank. Entry 49 of the ar9311 ISP-init template
# array is the bank's own static image, covering 0x0004 to 0x0068, installed by
# an isp_memcpy at 0x25aa4c on the cvisp side rather than by any submodule.
# ar-isp-library.h does not carry it because that file holds submodule images
# only, which is why these read as uncovered.
#
# The gamma, drc, raw_crop, ltm and ccm2 sites that also reference entry 49
# only restore their own four-word DMA descriptor out of it before overriding,
# which is what made the entry look like it belonged to two unrelated modules.
EXPLAINED.update({
    0x0000: 'ISP top level: the enable word. A base 0x90000000 stored whole at '
            '0x25aa58, planted into the template structure at runtime by the '
            'SoC accessor at 0x1f4de0, then one or-ed bit per module that came '
            'up: dpc_v1, decompander, wb, raw_crop, ccm1 and ltm_v1/gamma',
    0x0010: 'ISP top level: the template entry 49 image word verbatim',
    0x0014: 'ISP top level: the template image word. Bits 6:0 are per-stage '
            'commit bits the hardware clears after consuming, which is why '
            'this sits in the trim table rather than the ordered one',
    0x0018: 'ISP top level: the template image word with bit 16 or-ed in by '
            'raw_crop at 0x1a5560 and cleared again at 0x1a5930, so it '
            'alternates with that stage',
    0x0068: 'ISP top level: a constant the raw_crop enable path stores at '
            '0x1a5548, over an image that holds zero; the disable path writes '
            'zero back',
})

# The colour and noise stages. wb, cm and cm2 all pack a gain into a field of
# their first words, each from its own ladder in the tuning file, and the three
# agree on one AE operating point: cm2's interpolation fraction is pinned by
# the lo2 bound check-cm2-ladder.py already proves, and the abscissa that
# implies lands cm on the row its own value needs.
EXPLAINED.update({
    0x5004: 'wb: a red gain in Q8 masked to 12 bits, stored at 0x1adf74. The '
            'AWB gate at blob+0xbbe98 reads 0, the same flag awbs_stats is '
            'disabled by, so the setter takes the branch at 0x1ae584 that '
            'loads 1.0 into all three channels',
    0x500C: 'wb: the blue gain, the sibling store at 0x1adf7c, 1.0 for the '
            'same reason',
    0x483C: 'cm: floor(32 * gain) in a 7-bit field, packed at 0x19f23c over an '
            'image whose upper bits are zero. The gain is the cm ladder at '
            'blob+0x89d70, 5 AEC rows by 7 CT columns; the installed 33 needs '
            'a gain in [1.03125, 1.0625) and row 1 holds 1.05 exactly',
    0x4804: 'cm2: floor(32 * gain) in a 7-bit field over the image\'s upper '
            'bits, packed at 0x1a0658, with the saturation multiplier at '
            'priv+808 reading exactly 1.0. The gain interpolates between rows '
            '1 and 2 of the ladder at blob+0xa1378, and the blend fraction is '
            'pinned by the lo2 bound at 0x4824: the whole interval that bound '
            'allows gives floor(32 * gain) = 30, which is the installed field',
    0x1800: 'rnr: bit 3 is (payload word 0 > 1), from the blob ladder at '
            '0x7a6c, cleared at the driver\'s band where the image was built '
            'at a higher one. Bits 0 to 2 come from the command handlers and '
            'bits 5 to 7 from the image',
    0x1890: 'rnr: (line length - width + 500) with bit 16 set, built at '
            '0x19aab4 and stored in two parts at 0x19ab18 and 0x19ab28. At '
            '1080p60 the line length is 2200, so 2200 - 1920 + 500 = 0x30c',
    0x4C30: 'lsc: a constant stored at 0x1b6518, the same 52 that also reaches '
            'bank+0x24 and bank+0x28. The alternate branch at 0x1b6608 writes '
            'zero here instead, selected by the toggle at priv+1560, so the '
            'register alternates across table re-arms',
})

# Registers whose derivation is recovered but whose shipped value does not
# follow from it at the operating point this driver configures. The provenance
# question is answered; the value is still a replay, and a stale one.
MISMATCHED = {
    0x3D14: 'lnr: two 9-bit fields packed at 0x1bbc38 from payload words 0x88 '
            'and 0x8c, each scaled by the strength level at priv+752 through '
            'the formula at 0x1bef08. Strength 55 reproduces both measured '
            'vendor captures bit-exactly and is the only integer that fits '
            'both. **At the abscissa this driver configures, 3938/256, that '
            'gives 0x00490049 and not the 0x004a004a shipped**, which needs a '
            'band-3 blend around gain 6.4 to 7.2. The shipped constant is a '
            'replay from a different capture and should be computed instead',
}
EXPLAINED.update(MISMATCHED)


# Stages both the register gate and the vendor's own tuning file agree are
# switched off. A register on a stage that does not run cannot be
# operating-point dependent, which is the whole risk the UNEXPLAINED class
# exists to flag: there is no operating point. Reproducing the vendor's value
# is parity, because the vendor had the stage off with that value too.
#
# This is weaker than a derivation and is deliberately its own class rather
# than folded into EXPLAINED. **The moment a stage here is enabled its
# registers become recordings again**, and for awbs_stats that is a live
# prospect: it feeds AWB, which is still to be implemented.
#
# Two independent sources are required. The register gate comes from the
# library's own read-modify-writes via ar-isp-gates.h, evaluated against the
# value the driver installs. The tuning-file flag comes from the blob offset
# that same table records, read with scripts/isp/isp-pipeline.py --tuning. They
# are recovered from different places, so one cannot prop up the other.
DISABLED_STAGES = {
    'awbs_stats': (0x6C00, 0x7200,
                   'gate 0x6c00 bit 0 clear in the installed 0x02, and the '
                   'tuning flag at blob+0x0bbe98 reads 0'),
    'hdr_awbs_stats': (0x1E44, 0x1F40,
                       'gate 0x1e4c is a word gate reading 0, and it shares '
                       'the tuning flag at blob+0x0bbe98, which reads 0'),
}


# Registers no vendor code writes. Each was searched for the same way: every
# store in the owning module enumerated, the bank shown to be mapped exactly
# once in the whole library so no other module can reach it, the value searched
# for as an instruction immediate, and ar_mpp_drv.ko checked. The value in the
# driver came from a state capture, so what it captured is whatever the block
# had put there.
#
# **This class is not parity, and it is the one place where reproducing the
# vendor is probably the wrong thing to do.** Writing a recorded value into a
# register the block owns is a defect waiting to happen; the right fix is
# usually to stop writing it. Each entry says what the evidence is, because
# "read but never written" is much stronger than "never referenced at all",
# and neither has been confirmed against the device yet.
HARDWARE_OWNED = {
    0x6060: 'read at 0x1ff268 and 0x1ff308, never written; the value is '
            'multiplied by the stats queue slot index and added to the buffer '
            'base, and the second read is a load whose result is discarded',
    0x6478: 'read at 0x202658 and 0x2027dc, never written; same slot-index use '
            'and same discarded second read',
    0x6514: 'read at 0x1f8f80 and 0x1f8ff4, never written; same shape, and '
            'ar-isp-main.c already flags this one as unmeasured',
    0x647C: 'neither read nor written anywhere in the library, so the evidence '
            'is absence only',
    0x6518: 'neither read nor written anywhere in the library, so the evidence '
            'is absence only',
    0x651C: 'neither read nor written anywhere in the library, so the evidence '
            'is absence only',
    0x2834: 'no store at this offset in ltm, ltm_stats or the reciprocal '
            'packer, and the ISP-init image carries zero. Four device sweeps '
            'read 4, 4, 0 and 4, none of them the value the trim table '
            'replays, and the neighbour 0x2838 is already in ar_isp_hw_owned',
    0x00EC: 'no writer and no reader on the ISP base anywhere in the library, '
            'and the vendor never touches it in any capture: no access to '
            '0x00e8, 0x00ec or 0x00f0 in 115554 traced writes, nor in the '
            'read traces. The offsets the vendor does touch below 0x100 are '
            '0x00 to 0x68 contiguous plus 0x90, 0xb8 and 0xc4 to 0xdc. It is '
            'outside template entry 49, and the value exists nowhere in the '
            'library as a dword',
}


def disabled_span(off: int) -> str | None:
    """The stage covering this register, when that stage is switched off."""
    for name, (lo, hi, _why) in DISABLED_STAGES.items():
        if lo <= off < hi:
            return name

    return None


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

    # Registers the driver declines to replay because the hardware owns them.
    # They stay in the generated table, which records what was measured, so the
    # skip list in ar-isp-main.c is the authority on what is actually written.
    body = re.search(r'ar_isp_hw_owned\[\]\s*=\s*\{([^}]*)\}', MAIN.read_text())
    if not body:
        sys.exit(f'{MAIN.name}: no ar_isp_hw_owned array. Without it this audit '
                 'counts registers the driver does not write, which reads as '
                 'coverage it does not have.')

    for off in re.findall(r'0x[0-9a-f]+', body.group(1)):
        final.pop(int(off, 16), None)
        origin.pop(int(off, 16), None)

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

    # The half of de3d's bank that is derived from the frame geometry and the
    # sensor line length rather than from the gain ladder. Proved by
    # check-de3d-geometry.py, which reproduces the live vendor bank exactly.
    body = table_body(DRIVERS / 'ar-isp-de3d-geom.h', 'ar_isp_de3d_geom_regs')
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

        if disabled_span(off):
            return 'stage switched off'

        if off in HARDWARE_OWNED:
            return 'hardware-owned'

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
             'vendor DMA address', 'stage switched off', 'hardware-owned',
             'UNEXPLAINED']
    print(f'ISP registers the driver writes: {len(final)}\n')
    for kind in order:
        print(f'  {tally[kind]:5}  {kind}')

    # A class the tally does not list would vanish silently into the traceable
    # total, which is how the count stops meaning anything.
    unlisted = sorted(set(tally) - set(order))
    if unlisted:
        sys.exit(f'\nclassify() returns {", ".join(unlisted)}, which the tally '
                 f'does not list, so those registers are counted as traceable '
                 f'without ever being shown')

    if MISMATCHED:
        print(f'\n  {len(MISMATCHED):5}  of the explained are a stale replay: '
              f'the derivation is recovered and the shipped value does not '
              f'follow from it at the operating point this driver configures')
        for off in sorted(MISMATCHED):
            print(f'         {off:#06x}  {final[off]:#010x}')

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
