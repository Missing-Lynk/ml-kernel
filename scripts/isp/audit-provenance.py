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

import importlib.util
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
BASELINE = 0

# The count of registers that still need a register read off a running unit.
# This is the stricter bar: a value can have a known provenance and still be a
# recording. Ratchets the same way.
DEVICE_BASELINE = 2

# The frame and statistics-grid dimensions the driver configures. A register
# whose value is one of these, or a pair of them packed into halfwords, is
# carrying geometry rather than tuning data.
# 1928 is the frame width padded by 8, which de3d's spatial filter needs for
# its border, and 241 and 135 are that padded frame divided by 8, its block
# grid: 1928/8 = 241 exactly and 1080/8 = 135 exactly.
GEOMETRY = {1920, 1080, 960, 540, 36, 16, 1088, 1092, 1928, 241, 135}

# The vendor's MMZ physical range.
#
# Values in this range are physical addresses. The driver replaces active
# coefficient and statistics descriptors with its own allocations. The list below
# is a regression watch list for disabled-stage descriptors that used to survive
# the setup replay: HDR statistics, LUT3D, AWB statistics and AF statistics.
# They should be absent from the final register set. If one of those stages is
# enabled later, its driver path must allocate and publish a buffer it owns.
DMA_NOT_OVERWRITTEN = (0x1D68, 0x1DC0, 0x1E5C, 0x1F88, 0x5810, 0x5828, 0x5840,
                       0x5858, 0x6C90, 0x6D38, 0x7574, 0x758C, 0x75A0, 0x75BC)
DMA_LO, DMA_HI = 0x2A000000, 0x2C000000

# What a bank does to the shipped image, from the enable states in
# plans/isp-vendor-parity.md. This is what ranks the remaining work: a value on
# an enabled stage changes the picture, one on a quiescent stage cannot, and a
# statistics bank configures an accumulator the driver already owns.
IMAGE_PATH = {'cfa', 'dpc', 'rnr', 'ltm', 'de3d', 'drc', 'ccm1', 'rgb2yuv',
              'cnf', 'lnr', 'cm', 'cm2', 'lsc', 'wb', 'qgg', 'blc'}
STATISTICS = {'rro_stats', 'rro_face_stats', 'raw_hist_stats', 'rgb_hist_stats',
              'rgb_max_stats', 'awbs_stats', 'af_stats', 'derolling_stats',
              'hdr_rro_0_stats', 'hdr_rro_1_stats', 'hdr_rro_face_stats',
              'hdr_awbs_stats'}


def bank_class(name: str) -> str:
    if name in IMAGE_PATH:
        return 'enabled image-path stage'

    if name in STATISTICS:
        return 'statistics accumulator'

    if name in ('base', 'isp_input'):
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

    # Descriptor lengths, in units of 32 bytes, sitting one word above the
    # pointer they describe. Each reproduces the fetch the driver itself sets
    # up, so the value follows from the allocation rather than from the trace.
    0x0034: 'gamma descriptor 0 length, in 16-byte records: 0x80 records is '
            'the 0x800 gamma page. The granule is 16 and not 32: no shift '
            'exists in the library, and the three descriptors whose flush size '
            'is a constant in the same function all give 16, compander 0x780 '
            'against a 0x7800 flush, lut3d 0x280 against 0x2800, HDR 0x80 '
            'against 0x800. ar-isp-colour.h already states it for lut3d',
    0x0038: 'gamma descriptor 0 valid/apply word, set beside the page 0 '
            'descriptor after the generated page is published',
    0x0044: 'gamma descriptor 1 length, the same 0x800 page; the three gamma '
            'slots are channel aliases on one buffer',
    0x0048: 'gamma descriptor 1 valid/apply word, set beside the page 1 '
            'descriptor after the generated page is published',
    0x0054: 'gamma descriptor 2 length, the same 0x800 page',
    0x0058: 'gamma descriptor 2 valid/apply word, set beside the page 2 '
            'descriptor after the generated page is published',
    0x0064: 'DRC descriptor length: 0x100 records of 16 bytes is 0x1000. The '
            'DRC flush covers the whole 0x2000 allocation rather than the '
            'content, which is what made 32 look right',

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
}

# The accumulator enable, on the four instances that have one. rro_stats does
# not: its configure routine stores nothing at engine+0x48, its enable is bit 1
# of engine+0x00 which EXPLAINED[0x6400] covers, and engine+0x48 there holds
# the library image rather than a boolean.
RRO_ACCUMULATOR_ENABLE = 0x48
RRO_NO_ACCUMULATOR_FIELD = {'rro_stats'}

# The zone the engine meters, which check-rro-engine.py derives from the grid
# and the metered window. Recovering these retired an earlier note saying they
# were read out of the configuration rather than derived; the vendor's own log
# format string calls them x_skip and y_skip.
RRO_ZONE = {
    0x24: 'the zone width, the metered window over the 16-column grid',
    0x28: 'the zone height, the metered window over the 36-row grid',
}

for _name, _bank, _shift in RRO_INSTANCES:
    for _off, _why in RRO_FIELDS.items():
        EXPLAINED.setdefault(_bank + _shift + _off, _why)

    if _name not in RRO_NO_ACCUMULATOR_FIELD:
        EXPLAINED.setdefault(_bank + _shift + RRO_ACCUMULATOR_ENABLE,
                             'rro engine: the accumulator enable')

    for _off, _why in RRO_ZONE.items():
        EXPLAINED.setdefault(_bank + _shift + _off, f'rro engine: {_why}')

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
    0x0008: 'ISP top level: the template entry 49 image word verbatim. The '
            'image carries 0x54870780 at both +0 and +4, so 0x0004 and this '
            'start identical and only 0x0004 is then patched. There is no '
            'producer to find: its decode to the active frame is the image\'s '
            'own operating point, not a derivation, and it would read the '
            'same at any other frame size',
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
    0x00D4: 'ISP top level: the second status/command word. The output-arm '
            'sequence writes the same 0x100 event the vendor repeats in its '
            'per-frame cycle',
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
    0x7558: 'af_stats: the per-frame A/B phase selector on priv+824. The '
            'fixed-focus sensor leaves the AF engine disabled',
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
    0x1D84: 'hdr_rro_1_stats: a constant built at 0x1fba38 and stored at '
            '0x1fba48. The field is per-instance, not per-HDR: the other two '
            'HDR instances store zero at the same site, 0x1fa9c8 and '
            '0x1fcac0, and the trace shows 0x1d2c holding zero throughout',
})

# drc's three coefficient groups and its strength. The selector at 0x1a45e0 is
# the cm2 pattern again: a ladder in the tuning blob at 0x17b1c, stride 0xc8c,
# six rows with AEC abscissas at 0x17a9c, and four paths through it. The packer
# at 0x1a4200 writes the shadow, which is pushed to the bank with isp_memcpy.
#
# The three groups are 15 taps packed one per byte, and all three read the same
# delta kernel out of the blob: seven zeros, 255, seven zeros. That is why
# three different image ladders collapse to one value, and why the value is
# 0xff000000 rather than anything saturating.
EXPLAINED.update({
    0x3004: 'drc: byte 3 of the first coefficient group, the 255 of a delta '
            'kernel at tuning record+0x80c whose 255 sits at record+0x828, '
            'packed at 0x1a42b0',
    0x3014: 'drc: byte 3 of the second group, the same delta kernel at '
            'record+0x848 whose 255 sits at record+0x864, packed at 0x1a4368',
    0x3024: 'drc: byte 3 of the third group, the same delta kernel at '
            'record+0x884 whose 255 sits at record+0x8a0, packed at 0x1a4424',
    0x3060: 'drc: bits 8:0 are the strength from tuning record+0x808, which '
            'reads 255, 255, 255, 200, 150, 100 down the six ladder rows. The '
            'same selector blends it, at 0x1a488c, so this is an operating '
            'point and not a row: the traces hold 16 distinct values here and '
            'settle on 160, with the installed 150 appearing once in the whole '
            'corpus. The upper half is the image value the mask at 0x1a4484 '
            'preserves',
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

# The isp_input page's vsync monitor, written by isp_hw_module_set_ctl at
# 0x1d3448. Two NEON quadword loads at 0x1d3a9c and 0x1d3aa0 copy one 32-byte
# .rodata constant at 0x367370, {0, 3, 8, 14, 15, 23, 22, 31}, into the ISP
# object at +2432, and each register then takes one slot into a 5-bit mux_sel
# field by read-modify-write.
#
# The vendor's own debug command names the block and the field: the help text
# at 0x367f10 is "ISP hardware info debug command", the printf at 0x368360 is
# "mux_sel=%d module : %s : hcnt %d vcnt %d ro_blank_h=%d", and the %s indexes
# a 32-pointer name table at 0x412510. So the eight values are a tap routing
# map along the pipeline, and 0x7050 reports the counters for the selected tap.
ISP_INPUT_MUX = {
    0x7058: (0, 'vsync_in'),
    0x7088: (3, 'debug_hdr_vysnc'),
    0x708C: (8, 'rnr_vsync'),
    0x7090: (14, 'dpp2ccml_vysnc'),
    0x7094: (15, 'nr3d_vsync_o'),
    0x7098: (23, 'defog_vsync'),
    0x709C: (22, 'cnf_vsync'),
    0x70A0: (31, 'debug_scaler_y_vsync'),
}

for _reg, (_index, _tap) in ISP_INPUT_MUX.items():
    EXPLAINED[_reg] = (
        f'isp_input vsync monitor: mux_sel = {_index} in bits 4:0, selecting '
        f'the {_tap} tap, from the 0x367370 table copied at 0x1d3a9c')

EXPLAINED[0x7058] += ('. Bits 14 and 19 are hardware state the vendor preserves '
                      'by reading before writing; this driver installs them as '
                      'a flat captured constant instead')

# The colour and noise stages. wb is fixed by the AWB-off branch. cm and cm2
# both pack a gain into a field of their first words, each from its own ladder
# in the tuning file, and agree on one AE operating point: cm2's interpolation
# fraction is pinned by the lo2 bound check-cm2-ladder.py already proves, and
# the abscissa that implies lands cm on the row its own value needs.
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
    0x1800: 'rnr: bit 3 is packed at 0x19b250 by birnr, not by rnr, from word '
            '1 of the birnr ladder at blob+0xb35e4. That ladder is all zeros '
            'in this blob, so the bit is clear at every abscissa, which is the '
            'whole difference from the image. Bits 0 to 2 come from the '
            'command handlers and bits 5 to 7 from the image',
    0x1890: 'rnr: (line length - width + 500) with bit 16 set, built at '
            '0x19aab4 and stored in two parts at 0x19ab18 and 0x19ab28. At '
            '1080p60 the line length is 2200, so 2200 - 1920 + 500 = 0x30c',
    0x4C40: 'lsc: the shading enable in bit 16 and the strength in Q7 in '
            'the low byte, both from the lens-shading record the tuning '
            'manager copies from blob+0x9090 at 0x174b10. The enabled branch '
            'at 0x1b4e58 sets the bit and quantises the strength, the '
            'disabled one at 0x1b4c20 clears it and forces 0x80, and both '
            'fall into the same store. The shadow write uses a split base, '
            'str at immediate 196 off x19+0x3400, which is why a search at '
            '13508 found nothing',
    0x0024: 'compander descriptor length, in 16-byte records, copied '
            'verbatim by the vendor at 0x1b03ac. Structurally the same '
            'descriptor as gamma 0x0030/0x0034 and DRC 0x0060/0x0064, and '
            'ar-isp-regs.h already names it a length; it had been passing as '
            'frame geometry only because 0x780 equals 1920',
    0x3C74: 'cnf: the library image 0x000a0d25 with bit 0 cleared by the '
            'read-modify-write at 0x1a25c8, in the same basic block that sets '
            'bit 0 of 0x3c64 at 0x1a25bc. No instruction anywhere sets this '
            'bit, and the cnf packer never writes this offset, so it is '
            'neither an enable nor a copy of the strength',
    # hdr_awbs_stats' mesh grid, computed by isp_awb_mesh_grid_stats_set_format
    # at 0x1f93f0 and stored as four separate words at 0x1f9550 to 0x1f9560.
    # The frame over the ROI quadruple {0,0,1,1} gives 1920>>6 = 30 and
    # 1080/36 = 30, halved by 1 << ([ctx+812] + 1) to 15, then the HDR-only
    # tail ((n + 1) & ~1) - 2 gives 14. The vendor image carries 15 because the
    # non-HDR twin at 0x1f6c90 has the identical prologue and no tail, and the
    # ISP-init template carries that page's value in this bank.
    0x1E7C: 'hdr_awbs_stats: the mesh grid, 14 from the frame over the ROI '
            'with the HDR-only correction at 0x1f9534',
    0x1E80: 'hdr_awbs_stats: the same, stored at 0x1f9558',
    0x1E84: 'hdr_awbs_stats: the same, stored at 0x1f955c',
    0x1E88: 'hdr_awbs_stats: the same, stored at 0x1f9560',

    0x0C10: 'dpc: a per-frame read-modify-write at 0x1c8704, setting bit 1 '
            'then clearing bit 0, so the image 1 becomes 3 and then 2. Bit 0 '
            'is the enable and bit 1 the bypass, and the branch is an AE and '
            'tuning decision re-evaluated every frame from tuning +0x6ce8 and '
            '+0x6cfc, not a constant. 2 means DPC bypassed; a capture shows it '
            'flipping to enabled mid-run, so this driver bakes in one phase',
    0x0D08: 'dpc: 0x1ff with bit 8 cleared at 0x1c89e8, once, immediately '
            'after the defect LUT is written and its window at 0x0c0c closed. '
            'No image covers it because the dpc template is 0x108 bytes, '
            'ending one word below. What bit 8 selects, and whether 0x1ff is '
            'the reset value, are not established: no capture reads this '
            'register before the read-modify-write',
    0x6C34: 'awbs_stats: the vendor\'s reserved carveout base, from '
            'get_camera_server()+1584 at 0x1f7820, which is the literal '
            '0x02000000 hard-coded at 0x145ef4 alongside a 32 MiB size. The '
            'same constant the HDR line buffer uses, so it is a code constant '
            'rather than a runtime allocation',
    0x6C38: 'awbs_stats: the same carveout base, fetched a second time at '
            '0x1f782c and stored at 0x1f783c, which is why the pair is equal',
    0x6C00: 'awbs_stats: the image 1 with bit 0 cleared and bit 1 set. Bit 0 '
            'is the stage enable, cleared at 0x1f78c8 because the tuning flag '
            'at blob+0xbbe98 reads zero. Bit 1 is set by the read-modify-write '
            'at 0x1f6cf0, on the get_start_opt()->[12268] branch, which is the '
            'same gate that sets bit 1 of 0x6400 and 0x6000',
    0x1E50: 'hdr_awbs_stats: a constant 1 stored at 0x1f9454, on the same '
            'get_start_opt()->[12268] branch; the other arm takes a different '
            'path entirely',
    0x4C24: 'lsc: the same 52 the enable path writes to bank+0x30, masked '
            'into the low byte here at 0x1b6504 to 0x1b6514. It is not three '
            'gate bits: 0x34 passes the gate test only because it equals '
            '0x04|0x10|0x20 against clear masks of 0xff, which any low byte '
            'would pass',
    0x4C30: 'lsc: a constant stored at 0x1b6518, the same 52 that also reaches '
            'bank+0x24 and bank+0x28. The alternate branch at 0x1b6608 writes '
            'zero here instead, selected by the toggle at priv+1560, so the '
            'register alternates across table re-arms',
    0x4C3C: 'lsc descriptor valid/apply bit, set after the driver-owned LSC '
            'page is published',
})

EXPLAINED.update({
    0x2834: 'ltm: the vendor writes zero here, from its own template image, '
            'which ar-isp-library.h carries and the trace shows installed in '
            'the contiguous 0x2800 to 0x2844 block. The later live value is '
            'hardware state, so the trim table excludes it.',
})


# Stages both the register gate and the vendor's own tuning file agree are
# switched off. A register on a stage that does not run cannot be
# operating-point dependent, which is the whole risk the UNEXPLAINED class
# exists to flag: there is no operating point. Reproducing the vendor's value
# is parity, because the vendor had the stage off with that value too.
#
# This is weaker than a derivation and is deliberately its own class rather
# than folded into EXPLAINED. **The moment a stage here is enabled its
# registers become recordings again**. On this unit awbs_stats feeds an AWB
# path the vendor configured off, so enabling it would be beyond-vendor work.
#
# **The two readings are not independent, and the earlier wording here claiming
# they were was wrong.** The library derives the register bit from the blob
# flag: isp_sub_awbs_stats command 0xb10 at 0x1f7780 reaches blob+0xbbe98
# through the tuning manager and branches on it, setting bit 0 of 0x6c00 at
# 0x1f77e8 or clearing it at 0x1f7924. The installed 0x02 having bit 0 clear is
# caused by that flag reading zero. One blob byte underwrites this class, and
# also underwrites EXPLAINED[0x5004] and [0x500c].
#
# The span must be the stage's real image extent. It was 0x6c00 to 0x7200,
# which swallowed the whole isp_input page at 0x7000 and excused ten registers
# on a stage that does run.
DISABLED_STAGES = {
    'awbs_stats': (0x6C00, 0x6DA4,
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
# "read but never written" is much stronger than "never referenced at all".
#
# **Seven of these are no longer written.** The trim A/B in out/au-trim-ab/,
# read by check-trim-effect.py, settled them, and gen-isp-defaults.py's
# TRIM_EXCLUDED carries the verdict for each. They stay listed here because the
# class is what they are, not what the driver does with them; 0x6518 and 0x651c
# are the two the driver still writes, and they are the two the audit reports as
# needing a device.
HARDWARE_OWNED = {
    # Comparing the installed values against five device captures, 0x6060,
    # 0x6478, 0x647c and 0x6514 never read back what this driver writes, while
    # 0x6518, 0x651c and 0x705c always do, which is why ar_isp_kept holds the
    # first two.
    #
    # **That comparison is weaker than the A/B, which has since been run.** The
    # A/B toggles the trim parameter and re-reads, which separates "the write
    # did nothing" from "a later applier overwrote it"; a snapshot cannot. It
    # confirms the four counters and 0x00ec never take effect, shows 0x7054's
    # low half never taking the written value while its high half agrees only
    # with the pass on, and shows 0x705c's write genuinely landing. 0x6514 stays
    # unmeasured: no sweep layout has dumped its page yet.
    #
    # These four are counters, and the evidence is now positive rather than an
    # absence. Each sits exactly one word past the end of its own module's
    # static image, is read and multiplied by a queue-node field to locate the
    # statistics buffer, and reads a DIFFERENT value in every capture: 6, 7, 5,
    # 15 and 3 across five sweeps, never the 10 or 11 this driver installs.
    # A symbolic scan that propagates the bank pointer through split-base,
    # indexed, stp, stur and NEON forms finds no store at any of them.
    0x6060: 'raw_hist_stats: a hardware counter one word past its image, read '
            'at 0x1ff268 and multiplied by a queue-node field at 0x1ff288 to '
            'locate the buffer. Five captures read five different values',
    0x6478: 'rro_stats: the same, read at 0x202658 and multiplied at 0x202684',
    0x647C: 'rro_stats: tracks 0x6478 in every capture',
    0x6514: 'rro_face_stats: the same, read at 0x1f8f80 and multiplied at '
            '0x1f8fa0; it tracks the rro_stats pair except in one capture, '
            'the same off-by-one this driver\'s own recording caught',
    0x6518: 'rro_face_stats: no writer, and unlike the counters above it reads '
            '1 on all five captures including the vendor\'s, so the value we '
            'carry is at least stable',
    0x651C: 'rro_face_stats: the same, stable at 1 across every capture',
    0x7054: 'isp_input vsync monitor: a live counter. No 0x7054 immediate '
            'exists in the library, split-base and indexed forms were '
            'searched, and the offset appears zero times across all nine MMIO '
            'traces, which do record reads. Five device captures read five '
            'different values: 0x000b1f22, 0x000b0187, 0x00000202, 0x000c0211 '
            'and 0x000b00ff',
    0x705C: 'isp_input vsync monitor: hardware status. Zero trace accesses, '
            'and identical across all five captures. Bits 31:16 are named '
            'ro_blank_h by the vendor\'s own printf at 0x368360; the low half '
            'is not established',
    0x00EC: 'MEASURED on the device: the trim A/B reads 0x56008600 with the '
            'pass on and off alike, and never the 0x6008400 this driver '
            'writes, so that write provably never takes effect. No writer and '
            'no reader on the ISP base anywhere in the library, '
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


def check_generator_exclusions(arrays: dict[str, list[tuple[int, int]]]) -> None:
    """
    The generated header must agree with the generator's own exclusion lists.

    ar-isp-defaults.h says "do not edit" and is not regenerated on every change,
    so an exclusion that lives only in the emitted file is a hand edit that the
    next regeneration silently undoes. Every one of them is a decision with a
    reason recorded in gen-isp-defaults.py; this asserts that the file in the
    tree is the file that generator would emit, for those decisions at least.
    """
    spec = importlib.util.spec_from_file_location(
        'ar_isp_gen', HERE / 'gen-isp-defaults.py')
    gen = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = gen
    spec.loader.exec_module(gen)

    setup = arrays['ar_isp_setup_1080p60']
    bad = [(off, val) for off, val in setup
           if off in gen.SETUP_EXCLUDED_DISABLED_WRITES]
    bad += [(off, val) for off, val in setup
            if off in gen.SETUP_DISABLED_ENABLES and val]
    bad += [(off, val) for off, val in arrays['ar_isp_vendor_trim']
            if off in gen.TRIM_EXCLUDED]
    if bad:
        listed = ', '.join(f'{off:#06x}={val:#x}' for off, val in sorted(bad))
        sys.exit(f'{DEFAULTS.name} carries writes gen-isp-defaults.py excludes: '
                 f'{listed}. Either the header predates the exclusion or the '
                 f'exclusion was never encoded, and a regeneration would '
                 f'disagree with the tree either way.')


def load_tables() -> tuple[dict[int, int], dict[int, int], dict[int, str]]:
    """The library images, the driver's final value, and which table won."""
    arrays = reg_arrays(DEFAULTS) | reg_arrays(MAIN)
    check_generator_exclusions(arrays)
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


# Stages that reach derived_registers() without reading the tuning file. Their
# provenance is real but it is not the blob, and calling it blob-derived
# overstated the strongest class in this file.
#
#   ccm, rgb2yuv   a static block lifted from the vendor's ISP-init template,
#                  per ar-isp-ccm-init.h and ar-isp-rgb2yuv.h
#   dpc            read from a device, per ar-isp-dpc.h, which records the
#                  unit and the date. 63 of its 67 registers turn out to equal
#                  the vendor's library image and two more are the frame, so
#                  only the remaining two are a capture with no other source
VENDOR_STATIC = {'ccm', 'rgb2yuv'}
DEVICE_CAPTURE = {'dpc'}


def derived_registers() -> dict[int, str]:
    """Every register a driver stage computes or carries, by stage."""
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
    skipped = {0x3D10}
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


# CVISP is a separate block at 0x08e00000 with its own configuration path, so it
# has its own classifier. The audit reports its bottom line here rather than
# leaving it outside the headline, which is what let a 331-register replay sit
# beside a zero-unexplained result for the main ISP.
CVISP_BASELINE: int = 8


def cvisp_section() -> int:
    """Print the CVISP block's own provenance line. Returns the recorded count."""
    checker = HERE / 'check-cvisp-derivation.py'
    spec = importlib.util.spec_from_file_location('cvisp_check', checker)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    body = module.DEFAULTS.read_text()
    state: dict[int, int] = {}
    for name in ('setup', 'late', 'tick'):
        for off, val in module.parse_table(body, name):
            state[off] = val

    classes, _mismatch = module.classify(state, module.parse_library(),
                                         module.geometry_words())

    print(f'\nCVISP, the block at 0x08e00000, configured from '
          f'vendor-tables/ar-cvisp-derived.h: {len(state)}\n')
    for name in ('library image', 'frame geometry', 'port scoreboard', 'zero write',
                 'unity gain', 'blc constant', 'vendor DRAM address', 'trace residue',
                 'UNSOURCED'):
        if classes.get(name):
            print(f'    {len(classes[name]):5}  {name}')
    print('\n  run scripts/isp/check-cvisp-derivation.py for which ones and why')
    return len(classes.get('trace residue', [])) + len(classes.get('UNSOURCED', []))


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
            stage = derived[off]
            if stage in VENDOR_STATIC:
                return 'vendor static block'

            # A device-captured stage is only a recording where the capture is
            # the only source. Most of dpc turns out to equal the vendor's own
            # library image, and two of its registers are the frame, so those
            # are held to the stronger class rather than excused by the stage
            # they belong to.
            if stage in DEVICE_CAPTURE:
                if library.get(off) == value:
                    return 'library image'

                if is_geometry(value):
                    return 'frame geometry / grid'

                # A hand-recovered mechanism outranks the stage it belongs to.
                if off in EXPLAINED:
                    return 'explained'

                return 'device capture, no other source'

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
             'vendor static block', 'device capture, no other source',
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

    # The second axis, and the one that matters for rebuilding this driver on
    # a bench with no camera attached: can the value be regenerated from
    # checked-in sources plus the two capture artifacts that are themselves
    # files, the vendor library and the tuning blob? A class that can is
    # device-independent. A class that cannot needs a register read off a
    # running unit, which means it is correct only at the operating point and
    # on the silicon it was read from.
    device_sourced = {
        'device capture, no other source':
            'read off a device, with no vendor image or code behind it',
        'hardware-owned':
            'the block writes it; the value we carry is what it happened to '
            'hold when the state was captured',
        'stage switched off':
            'excused because the stage does not run, which is not a '
            'derivation; it reverts to a recording the day it is enabled',
    }
    # A union, not a sum: a register can be both a vendor DMA address the
    # driver never overwrites and on a stage that does not run, and counting
    # it twice would overstate the problem.
    #
    # The DMA class splits. The driver replaces most of these with its own
    # allocation at arm time, which is device-independent; the ones in
    # DMA_NOT_OVERWRITTEN are published as the vendor read them.
    # Stale replay values are driver bugs rather than device-sourced facts.
    # Keep them out of this axis; audit must report none separately.
    sourced = {off for off in final if classify(off) in device_sourced}
    sourced |= {off for off in DMA_NOT_OVERWRITTEN if off in final}
    needs_device = len(sourced)

    print(f'\n  {len(final) - needs_device:5}  regenerable without a device, '
          f'from the library, the tuning blob and the driver\'s own '
          f'configuration')
    print(f'  {needs_device:5}  needs a register read off a running unit')
    counted: set[int] = set()
    for kind, why in device_sourced.items():
        regs = {off for off in sourced if classify(off) == kind} - counted
        counted |= regs
        if regs:
            print(f'         {len(regs):5}  {kind}: {why}')

    regs = {o for o in DMA_NOT_OVERWRITTEN if o in final} - counted
    counted |= regs
    if regs:
        print(f'         {len(regs):5}  vendor DMA the driver never overwrites: '
              f'we publish the vendor\'s carveout address')

    actions = []
    if any(classify(off) == 'hardware-owned' for off in sourced):
        actions.append((
            'hardware-owned',
            'widen the sweep to the pages the A/B missed, then rerun '
            'check-trim-effect.py. 0x6518 and 0x651c read a stable 1 on every '
            'capture, so a sweep that dumps page 0x65 with the trim pass off '
            'settles whether that 1 is theirs or ours. Do not decide this from '
            'snapshot comparison: it cannot tell a dead write from one a later '
            'applier overwrites'))
    if any(classify(off) == 'stage switched off' for off in sourced):
        actions.append((
            'stage switched off',
            'stop writing them while the stage is off, and derive them from '
            'its packer on the day it is enabled'))
    if regs:
        actions.append((
            'vendor DMA',
            'drop disabled-stage vendor addresses from ar_isp_setup_1080p60 '
            'or publish driver-owned allocations before enabling the stage'))

    if actions:
        print('\n  what would make each device-sourced group independent:')
        for group, fix in actions:
            print(f'    {group}: {fix}')

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


    cvisp_residue: int = cvisp_section()

    if cvisp_residue > CVISP_BASELINE:
        print(f'\nregression: {cvisp_residue} CVISP registers are still '
              f'recordings against a baseline of {CVISP_BASELINE}')
        return 1

    if cvisp_residue < CVISP_BASELINE:
        print(f'\nimproved: {cvisp_residue} CVISP recordings against a baseline of '
              f'{CVISP_BASELINE}; lower CVISP_BASELINE to lock it in')

    if needs_device > DEVICE_BASELINE:
        print(f'\nregression: {needs_device} registers need a device read '
              f'against a baseline of {DEVICE_BASELINE}')
        return 1

    if needs_device < DEVICE_BASELINE:
        print(f'\nimproved: {needs_device} need a device read against a '
              f'baseline of {DEVICE_BASELINE}; lower DEVICE_BASELINE to '
              f'{needs_device}')

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
