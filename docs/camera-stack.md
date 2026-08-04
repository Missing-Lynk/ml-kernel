# Camera stack (NT99235 / CSI-2 / VIF / ISP / CVISP)

The air unit (BetaFPV VR04, Proxima-9311) captures from an onboard NT99235 image sensor over MIPI CSI-2. The vendor firmware drives that path entirely from userspace; reproducing the vendor's sequence in open drivers is the goal.

This document keeps two things separate: **how the vendor drives the hardware**, recovered from a live MMIO write trace of the streaming stock unit, and **what the open stack implements and has validated on hardware**. Every statement is backed by a capture, a disassembly or a hardware observation; anything not yet established is listed under "Not known".

Drivers: `overlay/drivers/media/artosyn/` (`nt99235.c`, `ar-csi2.c`, `ar-vif.c`, `ar-isp.c`, `ar-cvisp.c`). Device tree: `devices/betafpv-vr04-air/proxima-9311-air.dts`.

## Blocks

| Block | Base | Notes |
|---|---|---|
| VIF | `0x08870000` (64 KiB), IRQ SPI 62 | front end (path0) plus 8 capture views |
| MIPI CSI wrapper | `0x08880000` (64 KiB), IRQ SPI 60/61 | 8 receiver instances in 4 pairs; each pair `0x1000`: Artosyn glue at pair base, DesignWare host core 0 at `+0x400`, core 1 at `+0x800` |
| ISP | `0x08c00000` (2 MiB), IRQ SPI 71 | Bayer processing; feeds CVISP, does not write frames itself |
| CVISP | `0x08e00000` (64 KiB), IRQ unknown | the output stage: owns the frame queue and writes to DRAM |
| CGU | `0x0a100000` / `0x0a104000` / `0x0a108000` | clock controller |

This board uses CSI pair 0, core 0 at `0x08880400`. The DesignWare core version register reads v1.20, checked at probe.

**All four blocks are inside the SoC**, in its media subsystem. The NT99235 is a plain Bayer sensor: a pixel array, its own PLL, exposure and analogue gain, and a MIPI CSI-2 transmitter, nothing else. Every stage that turns its raw output into a picture runs on the Proxima-9311, so nothing in this document lives on the camera module.

The chain is: sensor sends CSI-2 packets over the MIPI lanes; the **CSI-2 receiver** decodes the link and produces a pixel stream on its IPI output; the **VIF** is the SoC's capture front end, which accepts that pixel stream and routes it, either straight to DDR through its bypass views or onward into the ISP; the **ISP** converts Bayer to YUV; and **CVISP** takes the ISP's output and writes frames to DDR.

**CVISP is the writer**: the ISP converts and hands off, CVISP owns the frame queue and the DRAM writes.

`VIF` is the vendor's own name for the block, used throughout `libmpp_service.so` (`vif_*`, 118 functions) and in the interrupt it registers (`ar_irq_reg_with_name(irq, handler, dev, "vif")`). It is the video interface between the receiver and the rest of the media block: it sees pixels, where the receiver sees packets. It also measures the incoming video timing, which is what makes `0x1f0` the front end's ground-truth register.

The vendor kernel does no camera hardware programming. `cam_hardware_power_on` calls `ar_pwr_ctrl_on(1)`, which on the 9311 dispatches through an uninstalled pointer and returns -1. All VIF, CSI, ISP and CGU register writes on the stock unit come from userspace (`libmpp_service.so`, called by `ar_lowdelay`) through `/dev/mem`.

## How the vendor drives the pipeline

### The trace

48183 register writes from one boot of the stock firmware, captured with `native/mmiotrace.c` (an `LD_PRELOAD` shim under `ar_lowdelay`, installed by `glue/camera/au-slotA-mmiotrace.sh`) over the window `0x08860000`-`0x08c0ffff`. Lines are `wNNNNNN wWW pa=0xADDR val=0xVAL` in program order. The log itself is a capture artifact and is not in the tree; re-take it with the harness.

Distribution: ISP 34790 writes, VIF 12639, CSI 754. Distinct registers touched: ISP 1267 (spanning `0x08c00000`-`0x08c076d8`), VIF 74, CSI 52.

Two properties of this trace constrain how it may be used:

- It captures **stores only**. The shim keeps the application's mapping `PROT_READ` and decodes the faulting store; loads pass through untrapped and are never logged. Nothing in this trace shows how the vendor polls status, reads interrupt flags, or decides a frame is complete.
- It captures **`ar_lowdelay`'s userspace writes only**. Writes issued by any other process or by the on-chip DSP core are invisible to it.

### Phase order

The trace has a setup phase (writes 1 to about 3150) and then a per-frame steady-state loop that repeats for the rest of the capture.

Setup runs in this order:

1. **ISP access port touched first** (writes 0-2): `0x08c0000c` = `0x82`, `0x08c000d4` = `0x06008000`, `0x08c0000c` = `0x02`.
2. **VIF front end** (writes 3 onward): the input-format and geometry block at `0x0c0`-`0x0f0`, then `0x13c`, `0x190`, `0x1c0`-`0x1d0`, `0x328`, the four coefficient pairs at `0x140`-`0x15c`, the `0x2b0` pair (`0x013ffffe` then `0x003ffffe`), and a full interrupt-status clear (`0x080`, `0x084`, `0x17c`, `0x184`, `0x194`, `0x424`, `0x5c0` written all-ones or zero).
3. **CSI receivers** (writes 134 onward): all eight receiver instances are zeroed at `+0x440`, `+0x444`, `+0x408` before pair 0 is configured, then VIF and CSI writes interleave through the D-PHY and IPI bring-up.
4. **ISP full configuration** (writes 719 onward): the bulk of the trace. 1267 distinct registers across roughly 34 banks of `0x100`, from `0x0000` to `0x76d8`.
5. **ISP staged enable** on the master register `0x08c00000`, in this exact progression:

   `0x90000000` -> `0x90000002` -> `0x90000012` -> `0x90000052` -> `0x90080052` -> `0x90280052` -> `0xb0280052`

   The final value `0xb0280052` is the streaming state and is re-written unchanged several times later in setup.

Geometry appears in the VIF front end as `0x088700d4` = `0x088700d8` = `0x07800438`, that is 1920 x 1080.

### The per-frame loop

From write ~3150 to the end of the trace the vendor repeats a fixed 20-write cycle, 2060 times. One cycle, in order:

| # | Register | Value |
|---|---|---|
| 1 | ISP `0x08c075a0` | buffer address |
| 2 | ISP `0x08c075bc` | buffer address |
| 3 | ISP `0x08c06440` | buffer address |
| 4 | ISP `0x08c06474` | buffer address |
| 5 | ISP `0x08c0600c` | buffer address |
| 6 | ISP `0x08c0280c` | buffer address |
| 7 | ISP `0x08c06508` | buffer address |
| 8 | ISP `0x08c02808` | buffer address |
| 9 | VIF `0x0887017c` | `0x01000000` |
| 10 | VIF `0x08870184` | `0x00000000` |
| 11-12 | VIF `0x08870194` | `0x00000000`, twice |
| 13-14 | VIF `0x08870294` | `0x00000000`, twice |
| 15 | ISP `0x08c000cc` | `0x04001550` |
| 16 | ISP `0x08c000d4` | `0x003a2000` |
| 17 | ISP `0x08c000cc` | `0x00000000` |
| 18 | ISP `0x08c000d4` | `0x10000200` |
| 19 | ISP `0x08c000cc` | `0x00000000` |
| 20 | ISP `0x08c000d4` | `0x00000100` |

Observations that hold across all 2060 iterations:

- **The eight address registers ping-pong between exactly two value sets.** Each register takes one of two addresses and alternates every frame. This is a two-buffer rotation.
- **All eight addresses fall in `0x2a65f200`-`0x2b378c00`**, inside a single reserved region. The `isp_cma` reservation in the air DTS (`0x2a000000`, 32 MiB) covers this range so the vendor's addresses are safe to write on the open stack.
- **The addresses are per-boot allocations.** An earlier separate trace shows the same structure at different addresses. The layout is stable; the values change per boot.
- **`0x08c000cc` / `0x08c000d4` behave as an indirect access port**, written as a pair. Three transactions run per frame, with the same values every frame (`0x04001550`/`0x003a2000`, then `0x0`/`0x10000200`, then `0x0`/`0x00000100`).
- **VIF `0x0887017c` = `0x01000000` is the path0 frame-start interrupt acknowledge** (`0x17c` bit24). It is the only VIF interrupt the vendor acknowledges per frame. `0x184`, `0x194` and `0x294` are cleared alongside it. Our ISR services `0x17c`, `0x184` and `0x1b0` where the vendor services `0x17c`, `0x184` and `0x194`: `0x1b0` handling is load-bearing for the polling path and fixed a real interrupt storm, and `0x194` reads zero mid-stream so the vendor's read-modify-write of it is inert. The deviation is therefore documented rather than removed; what `0x1b0` is in the vendor's map, and whether `0x194` is ever non-zero mid-stream, are open parity questions of low priority.
- **The ISP master register is not touched in the loop.** It is left at `0xb0280052`.

So per frame the vendor: writes eight buffer addresses, acknowledges the VIF frame-start interrupt, and runs three indirect-port transactions. Nothing else.

### The VIF views serve the bypass path

The VIF exposes a bypass-to-DDR path (`vif_bp*`) that writes "views" straight to DRAM with their own stride, FIFO, DDR size and crop, and an into-ISP path (`vif_isp*`). There are 8 views.

In the trace the vendor configures view 0 during setup, writes the view address registers `0x020`/`0x040`/`0x060` a total of 12 times, and then **sets the per-view reset** (`0x088702bc` = `0x00000002`, write 689) and never writes a view address again. All 2060 subsequent frames are captured with the view path held in reset.

This is corroborated on hardware: the open driver's VIF steady state (bypass-or register `0x01000000`, interrupt `0`, `0x184` `0`) is byte-identical to the running vendor's, and the open driver's view DMA writes nothing to DRAM.

The capture DMA is the ISP block's, driven by the eight per-frame address registers above.

### VIF ISP-path status registers

`0x104` through `0x138` are read-only: the vendor writes nothing in that range across all 48183 traced writes. Their use is recovered from `vif_ispintr_process` (`libmpp_service.so` `0x223a70`, 468 bytes), the VIF ISP-path interrupt handler. Its second argument is the interrupt status word (VIF `0x17c`), and it dispatches on the bits:

| Status bit | Registers read |
|---|---|
| 25, 26 | `0x104` |
| 28, 29 | `0x108` |
| 24 (ISP frame event) | `0x0c0`, `0x0ec`, then `0x10c` and `0x110` |

On bit 24 the handler reads `0x0c0` and tests bit 9, reads `0x0ec` and tests mask `0x40`, reads `0x10c` and `0x110` and discards both values, then conditionally clears the low 8 or low 16 bits of `0x0ec` and writes it back. Reading and discarding is consistent with clear-on-read status, but whether these registers latch status or count lines or pixels is **not established**; the values are never compared against anything in the handler.

These are the only visibility into whether pixels cross from the VIF front end into the ISP. Frame-start delimiters (`0x17c` bit24 on path0) are a header-level event and do not prove pixel traversal.

### VIF debug counters: `0x32c` selects, `0x330`-`0x33c` report

`0x330`, `0x334`, `0x338` and `0x33c` are **read-only debug counters behind a mux**. A vendor dump routine at `0x22ee80` (a local function, between the exported `vif_getmipitype_bits` and `vif_device_module_creat`) drives them: it read-modify-writes a channel index into `0x32c` bits [19:16], calls `ar_delay(100)`, then reads each counter and logs it as **two independent 16-bit fields**, high half and low half, through `ar_log_func_raw`. It loops, incrementing the channel index.

Two consequences for anyone diffing a live VIF window:

- **The values are only comparable when `0x32c` matches on both units**, because the mux decides what the counters are counting. On the paired slot A and slot B captures it reads `0x00000fff` on both.
- **These are free-running measurements, so the two units may read differently.** Measured on a streaming vendor unit against the open stack with the same mux setting: `0x330` low half is 6640 on both, `0x33c` high half is 9869 on both, and `0x338` agrees within 0.2% in both halves. That level of agreement across independent units is positive evidence the front end is seeing the same video, and it is the correct way to read these registers.

The same caution applies to the CSI wrapper's `0x030`: a single sample of a free-running counter carries no bit-level meaning.

`vif_ispcrc_config_path` (`0x226de8`) programs a CRC over the ISP path: after a dummy read of `0x134` it writes a `(width << 16) | height` pair to `0x118`, or to `0x114` for the other path. The vendor never writes either register in the trace, so this block is unexercised on stock; the result register has not been located.

### What the ISP is made of

The vendor library names every stage. `libmpp_service.so` exports 65 `isp_sub_<name>_creat` / `_delete` pairs, one per submodule, each allocating a module struct carrying an id at `+444` (`isp_sub_blc` is `0x1703`, the ISP block itself `0x1701`) and registering it. Register values are pushed per submodule through `isp_memcpy_bycmp(hw, shadow, prev, len)`, which writes a word only when it differs from the previous image, from 226 call sites spread across the per-submodule code.

Read in pipeline order, the list is a conventional ISP:

| Stage | Submodules |
|---|---|
| Sensor correction, Bayer domain | `blc`, `gib`, `fpn`, `dpc`, `dpc_v1`, `nuc_dpc`, `lsc`, `lsc_v1`, `hdr_lsc`, `hdr_lsc_v1`, `digigain1`, `digigain2`, `compander`, `decompander` |
| Noise reduction, Bayer domain | `raw_3dnr`, `birnr`, `rnr`, `lnr`, `de3d`, `de3d_v1` |
| Demosaic | `cfa`, the point where Bayer becomes RGB |
| Colour | `wb`, `ccm1`, `ccm2`, `acm`, `cm`, `cm2`, `lut3d`, `qgg`, `cnf`, `lms` |
| Tone | `gamma`, `gamma_v1`, `drc`, `drc_v1`, `ltm`, `ltm_v1`, `gtm2`, `gtm2_algo`, `hdr` |
| Colour space | `rgb2yuv`, the point where RGB becomes YUV |
| Geometry | `raw_crop`, `binning`, plus the scaler configuration on page `0x70` |
| Infrared | `ir_gtm`, `ir_nbbc`, `ir_rnr`, `ir_lms_horz`, `ir_stats` |
| Timing | `tg` |
| Statistics, no pixel output | `af_stats`, `awbs_stats`, `hdr_awbs_stats`, `drc_stats`, `ltm_stats`, `ltm_stats_v1`, `ltm_stats_v2`, `raw_hist_stats`, `rgb_hist_stats`, `rgb_max_stats`, `rro_stats`, `rro_face_stats`, `hdr_rro_0_stats`, `hdr_rro_1_stats`, `hdr_rro_face_stats`, `derolling_stats` |

The statistics family is what the per-frame loop's seven buffer addresses feed. They measure the frame for the 3A algorithms and are re-armed every frame, which is why they ping-pong while the picture planes, programmed once during setup, do not.

The `_v1` and `_v2` suffixes are alternate implementations of the same stage. Many submodules are never enabled in this configuration: their static defaults exist in the library but the vendor never pushes them, and their registers read back zero on hardware.

The eight per-frame ISP addresses are statistics buffers and the LTM coefficient page; each is assigned to its module in the statistics section below. Frame output is CVISP's, planar YUV at stride 2048.

### The output stage is CVISP

The block at `0x08e00000` is what writes frames to DRAM. The ISP feeds it. It is absent from the vendor device tree, so block-windowed traces never saw it; only the wide sweep did.

**The sweep.** A second capture, window `0x08000000`-`0x0a1fffff` skipping `0x08820000`-`0x0885ffff` (encoder and DSI, excluded because a previous wide attempt corrupted them), vendor streaming throughout, 330630 writes.

| Block | Writes | Identity |
|---|---|---|
| `0x0a080000` | 182058 | `h26x` encoder (vendor DTS) |
| `0x08c00000` | 60622 | ISP |
| `0x08800000` | 60228 | `axi_dma` (vendor DTS) |
| `0x08870000` | 15129 | VIF |
| `0x08e00000` | 11782 | **CVISP** |
| `0x08880000` | 738 | CSI-2 |
| `0x0a100000` | 66 | CGU |

The sweep is writes-only too, so it says nothing about what the vendor reads.

**The name** comes from `libmpp_service.so`, which is unstripped and exports a complete `cvisp_*` stack: device, input, output, filter, statistics, gamma and LSC, plus `cvisp_outlib_*`, `cvisp_device_irq_process` and `cvisp_dispatch_irq`. It is distinct from the device tree's `scaler@08840000` and `gdc@08848000`, which are different addresses.

**Cadences.** The vendor drives it at four:

| Table | When | Contents |
|---|---|---|
| setup | once | 255 ordered writes over 224 registers, ending at the staged output enable |
| late | once, just after the enable | 101 registers: the arbitration table on page `0x0000`, channel geometry on page `0x4000` |
| ring | once per frame | one Y/U/V triplet, round robin over five buffer sets |
| tick | once per ring wrap | eight registers at `0x4600`-`0x460c` and `0x4700`-`0x470c`, all `0x00000100` |

The tick group going once per *wrap* rather than once per frame is measured: 496 writes to each tick register against 2477 to each plane register, and the interleaving in the trace shows five triplets between consecutive tick groups.

**The enable** is a staged write to `0x8000`: `0x00800800` then `0x00800802` then `0x00800806`, five times and then twice, after which the register is never written again. Bits 1 and 2 are the launch candidates; their individual meanings are not decoded.

**The ring** is five buffer sets:

```
Y  0x28014000 -> 0x2834c000 -> 0x28684000 -> 0x289bc000 -> 0x28cf4000 -> repeat
U  0x28232000    0x2856a000    0x288a2000    0x28bda000    0x28f12000
V  0x282bb000    0x285f3000    0x2892b000    0x28c63000    0x28f9b000
```

Round robin, all three planes in lockstep, no deviation across 496 wraps. The range `0x28014000`-`0x2902c000` is reserved as `cvisp_cma` in the device tree; `vif_cma` moved to `0x2c000000` because the ring's last slot runs past where it used to start.

**Geometry is not settled.** `0x8028` carries `0x04380780` (1080 x 1920) and `0x8008` is written `0x021c03c0` (540 x 960) during setup but ends at `0x04380780`, while page `0x4000` carries 1920 x 1080 throughout. `0x021c03c0` is also what ISP `0x7080` reads on the streaming vendor. Which stage, if any, is scaled is open.

**Clock.** `cgu_rsz_clk`, `0x0a104014` low half, gate bit 12, device tree index 9. This is from the vendor clock table: the trace contains **no CGU write for this block at all**, and `0x0a104014` does not appear in the live vendor CGU snapshot either. So the vendor streams with whatever state the boot leaves that leaf in, and enabling it is an assumption. It is the load-bearing one in `ar-cvisp.c`, which is why that driver reads no register at probe: if the leaf is wrong, the first access hangs the SoC, and a probe-time read would make that unavoidable on every boot.

**No reset and no interrupt are declared.** No CVISP reset write appears anywhere in the trace and no reset leaf has been identified. The block does have its own completion path (`cvisp_device_irq_process` at `0x2424b8` dispatches through `cvisp_dispatch_irq` at `0x242390`, routing status bits 1 and 5 to output events `0x1001`/`0x1002` and bit 3 to `0x2c02`), which is good evidence that frame completion is serviced from CVISP rather than from VIF SPI 62 alone. But the hardware IRQ number and its acknowledge register are still behind the vendor's generic camera-module event layer, so asserting either in the device tree would be inventing it.

Extractor: `scripts/isp/gen-cvisp-defaults.py` -> `overlay/drivers/media/artosyn/vendor-tables/ar-cvisp-defaults.h`. The setup table self-checks: replaying it in order reproduces the vendor's final value for every register it touches.

### CVISP on the open stack, validated

CVISP writes YUV planes to DRAM on the open stack, sustained at 60 fps with one ring slot armed per VIF frame start (the vendor's cadence). Established facts:

- **The arm triggers the write.** A fully configured, enabled CVISP with no slot armed writes nothing.
- **The clock is inherited.** No clock request exists in the vendor's CVISP path; the boot firmware leaves `0x0a104014` gate bit 12 set and everyone inherits it. `ar-cvisp` does the same (`assert_clk` param, default off). The leaf is gate-modelled, so a `clk_disable_unprepare` on rmmod clears a gate the boot set; do not take ownership.
- **Line stride is 2048 for a 1920-wide plane**, measured. The 128 bytes of per-line padding hold stale DDR content that CVISP does not write: histogram or dump the active columns only, and dump `stride * height`, never `width * height`.
- **Completion path** (for the parked IRQ-driven driver, not needed for capture): CVISP registers its own raw IRQ handler (`0x241ed0`) through the vendor's generic IRQ service; the handler W1C-acknowledges a status word at `base + 0x34` and the service separately re-enables the IRQ per event. There are two CVISP registrations and the hwirq numbers live only in runtime `g_hw_info`, so no IRQ is asserted in our device tree. **Do not W1C `0x08e00034`**: in the trace that address is a written-once entry in the page `0x0000` arbitration table, so the IRQ status base is not the CVISP register base.

### Exposure, gain, and the tone response

Exposure and gain are implemented and validated. Both are SMIA addresses, recovered from the vendor's AE commit list in `libsns_nt99235.so`: integration time is a 16-bit big-endian line count at `0x0202`/`0x0203` clamped to the mode's frame length minus two, analogue gain is a code written to both `0x0206` and `0x0207`, and the pair must be bracketed by `0x0104` = 1 then `0x0104` = 0 so no frame sees a half-updated commit. The 26 registers at `0x8250`-`0x826c` and `0x8550`-`0x855c` that drift at runtime on the vendor are **not** exposure: they are sensor-MCU lens shading tables driven by the AWB path and reloaded through `0x8201`.

Gain is an index into a 97-entry table, and the mapping is closed form. The table was extracted from `libsns_nt99235.so` at virtual address `0x1a130`, file offset `0xa130` through the second `PT_LOAD`, 97 `u32` entries with 1024 meaning 1x, and

	gain = 2^(code >> 4) * (16 + (code & 0xf)) / 16

reproduces **all 97 entries exactly**: a four-bit mantissa with a four-bit binary exponent, 1x at code `0x00` to 64x at `0x60`. The formula reproduces the table exactly.

**The sensor is not SMIA-compliant, only SMIA-addressed.** Reading the SMIA identification and capability blocks on hardware gives 3 of 20 registers non-zero: `model_id` `0x9235` and `revision_number` `0x0b` are real, but `manufacturer_id` is 0, `smia_version` reads `0xff`, and the whole analogue-gain capability block `0x0080`-`0x0092` and integration-time block `0x1000`-`0x1006` are zero. So the functional registers follow SMIA addresses and SMIA semantics, confirmed by behaviour rather than by a compliance claim, and nothing is discoverable from the sensor: every limit has to come from the vendor library or from measurement. `nt99235.ko dump_smia=1` reproduces this.

The vendor ships three tuning blobs, `nt99235`, `sc2210` and `sc231`, all exactly 879,704 bytes: the format is a fixed-layout struct with constant offsets, which is what makes every "at `raw + offset`" reference in this document stable.

### DRC upload and strength control (recovered)

Dynamic-range control (DRC) is another ISP DMA payload. For this vendor `libmpp_service.so`, its immutable 8 KiB initial template is embedded at service-image VMA `0x467460` (file offset `0x457460`). The runtime configuration table at VMA `0x472760` supplies the exact pair `(source=0x467460, length=0x2000)`. The module's initial `0xb09` handler copies those bytes to its DMA allocation, flushes all `0x2000` bytes, stores the physical address in its descriptor and sets the descriptor apply bit (`0x1a5828..0x1a5940`). This is a firmware-build-specific constant; do not assume it is the same in another vendor build.

During normal operation the service overwrites only the first `0x1000` bytes of that page (`0x1a4200`). They are two `0x800` banks, each containing 128 16-byte records. A record packs three unsigned 20-bit values in its first ten bytes; the second value is duplicated, and adjacent records overlap. Thus one bank represents a 257-sample curve. The active DRC tuning profile starts at raw tuning offset `0x17b1c + index*0xc8c`; its first two 257-word curves are packed into the two banks. At neutral strength 50, a retained vendor DRC page decodes byte-for-byte to profile 3 of the NT99235 FPV blob.

Strength is implemented in ARM software before packing, around neutral 50. For a requested strength `s`, the service calculates `q = floor(abs(s - 50) * 4096 / 50)` and blends every curve word in Q12 with one of two fixed 514-word service-image curves: the high curve at VMA `0x35f080` when `s > 50`, or the low curve at `0x35fc90` when `s < 50`. It then performs the 20-bit packing; the ISP receives only the resulting DMA page. The final `0x1000` bytes remain the initial template. This establishes the vendor-equivalent DRC payload construction without a DMA capture, while the semantic meaning of the four page banks still needs hardware validation.

### Static table array and GTM2/LTM activation (recovered)

The service's ISP-init configuration is a contiguous array of `{u64 source, u64 length}` descriptors at VMA `0x472600..0x472a40` (56 non-null entries). It is the source of the initial payloads. The module setup handlers now give several entries unambiguous names:

| Config offset | Source / length | Consumer |
| --- | --- | --- |
| `+0x0b0`, `+0x0c0` | `0x46a0f0` / `0x40`, `0x469db0` / `0x340` | LSC control and DMA payload |
| `+0x150`, `+0x160` | `0x469460` / `0x64`, `0x467460` / `0x2000` | DRC control and page |
| `+0x240` | `0x463050` / `0x4000` | gamma initial page |
| `+0x270` | `0x451570` / `0x3c` | shared GTM2/LTM control template |
| `+0x290`, `+0x2a0..+0x2d0` | `0x45afe0` / `0x6c`, then four `0x2680` sources | LUT3D control and four DMA banks |

The LSC and LUT3D handlers copy precisely those descriptor fields into their DMA allocations. This resolves their payload provenance. The unlabelled entries must remain unlabelled until a setup handler accesses them; size and pipeline position alone are not proof of a module mapping.

The vendor modules `gtm2` and `ltm` named here share register bank `0x2800` and publish to it at `+0x08`. They are distinct from the HDR page (`0x1c6c`, `isp_sub_hdr`) and LSC (`0x4c34`, `isp_sub_lsc`); the driver owns their descriptors `0x2808`/`0x280c` with the identity page.

GTM2 and LTM are **enabled** in the NT99235 FPV preview configuration. Their separate setup handlers (`0x18ab38` for GTM2 and `0x18e2c4` for LTM) read the same raw control word at `raw + 0x7abd8`; it is `1` in the vendor blob. The enabled paths each set bits 4 (`0x10`) and 11 (`0x800`) of their module control word, prepare a `0x4000` DMA page from the selected `0x9c` profile and flush it. Their disabled paths clear exactly those two bits. This is enough to rule out "the blocks are off" as a reason to skip their packers. The physical ISP register represented by that module-control word is still unmapped, so these are proven service-side enable bits rather than a claim about a named MMIO bit.

### 3A execution (proven)

`raw_stats_filter_port` creates the AEC, AWB and AF modules in sequence (`creat_aec_algo_module` at `0x15db40`, `creat_awb_algo_module` at `0x15dbb0`, and `creat_af_algo_module` at `0x15dc20`).  The AEC module selects an in-process algorithm through `get_aec_algo_lib`; the vendor's `artosyn_ae_algo_creat`, AWB and AF implementations are all in `libmpp_service.so`.

The only DSP remote-call sites are `AR_DSP_AiISP_PreProcess`/`PostProcess`, an optional CNN/NPU enhancement path: it allocates NPU buffers and starts NPU/DSP pre/post threads, not 3A modules.  The FPV rootfs contains no DSP program image, so that path cannot run there.  The ARM MMIO trace therefore has no hidden DSP ISP-programming participant.

### Not known

Still open, not to be assumed:

- **Whether the late table has to follow the enable.** The vendor issues it with frames already in flight; whether that is required or is just what its threading produced is untested.
- **What the tick group does.** An acknowledge, a queue re-arm and a five-credit refill are all consistent with a once-per-wrap cadence.
- **`0x4100` and `0x4108`**, written `0x00000000` nine times aperiodically, paired with `0x4104`/`0x410c` which hold 1920 and 1080. A crop origin is the obvious reading; it is not confirmed.
- **What the three indirect-port transactions do.**

Resolved elsewhere: frame completion is the VIF ISR in the vendor's interrupt mode, the eight per-frame addresses are assigned in the statistics section, and the ISP module map from constructors through banks to descriptors is complete.

## Open stack

### Sensor: `nt99235.c`

V4L2 sensor subdev on I2C-0, address `0x1a`. 16-bit register address, 8-bit data, raw 3-byte I2C messages (no SMBus).

- Mode tables are opaque register sequences transcribed verbatim from the vendor `libsns_nt99235.so`, roughly 200 writes each. Modes: (0) 1920x1080p60 2-lane, the vendor's mode; (1) 960x540p120 2-lane; (2) 1920x1080p60 4-lane; (3) 1280x720p90 4-lane. Lane count is set inside each table by SMIA register `0x0114` (`0x01` = 2 lanes, `0x03` = 4 lanes). Resolution, format, PLL, and line/frame length are all inside the table.
- MCLK is 24 MHz on `cgu_sensor_mclk0` (DT clock name "mclk") and must be running before reset is released.
- Power-on, verbatim from the vendor `nt99235_cmos_power_on`: assert enable (gpio104) and reset (gpio107), both active low; settle 10 ms; enable MCLK; release reset; settle 10 ms. The enable line stays asserted for the whole session.
- `s_stream(1)`: runtime-resume, write the mode's full register table, `msleep(20)` for PLL and MCU settle, then `MODE_SELECT` (`0x0100`) = `0x01`. Stream-off writes `0x0100` = `0x00`.
- Link frequency and pixel rate are exported as read-only V4L2 controls so the receiver can pick its D-PHY range. The DT endpoint lane count selects and filters the modes.

#### Sensor configuration is verified equivalent to the vendor

Established two independent ways, one static and one on hardware. This is the only stage of the capture chain verified to this standard.

**Static, against the decompiled vendor library.** The vendor's 2-lane 1080p60 mode init is `FUN_00103440` in `libsns_nt99235.so`, 193 I2C writes issued through `FUN_00101508(ctx, reg, val)`. `nt99235_regs_1920x1080p60[]` is identical to it: same registers, same values, same order, duplicate writes included. This is a sequence comparison, not a set comparison. No register the vendor writes is missing, and no value differs.

One deviation exists. The open driver additionally writes `0x0383` = `0x01` at position 118, and the vendor library writes register `0x0383` nowhere, in any mode. In SMIA-style register maps `0x0383` is `X_ODD_INC` and `1` is the no-subsampling value, so it is very likely inert, but it is unexplained and is the single divergence from the vendor sequence.

**Live, on hardware.** 184 sensor registers read back over I2C from the streaming vendor on slot A and from the open stack on slot B, both mid-stream with the VIF front-end gate confirmed at `0x0784043c`. **Zero differences**, outside 26 registers in `0x8250`-`0x826c` and `0x8550`-`0x855c`.

Those 26 drift at runtime on the vendor and are expected to differ. Measured on slot A: 156 of 183 registers still hold exactly the value the vendor library programmed, and every one of the 27 that moved falls in those two ranges. The vendor's 3A layer writes them through the callbacks the sensor library registers (`AR_MPI_AE_SensorRegCallBack`, `AR_MPI_AWB_SensorRegCallBack`). The open driver programs the same initial values and never updates them, so a difference there is the vendor adapting.

**Exposure and gain are left unset.** Neither the vendor mode table nor the open driver sets them. The vendor drives them from the 3A layer at runtime, so after mode init the open stack leaves the sensor at its power-on defaults. A correct capture may therefore be very dark or black. **Judge a capture by whether the DMA wrote, not by image brightness**: pre-fill the target buffer with a marker and check whether the marker was overwritten. A buffer full of zeros is indistinguishable from a DMA that never ran.

Method: `glue/camera/au-chain-capture.sh` with `SLOT=a` then `SLOT=b`, diffed with `glue/camera/au-chain-diff.py`. Capture slot A first: it is the reference, and both captures are only meaningful if the front-end gate reads `0x0784043c`.

### Receiver: `ar-csi2.c`

V4L2 bridge subdev driving the DesignWare CSI-2 host core plus the Artosyn glue wrapper.

- Clocks: "csi" (`cgu_mipi_csi_0_clk`) and "pcs" (`cgu_mipi_pcs_clk`), both enabled at stream-on.
- D-PHY bring-up (`ar_csi2_phy_power_on_core`): hold PHY shutdown and reset, clear the test interface, run a dummy test cycle, write the fixed setup registers, write the lane registers, write the frequency-range code, then release shutdown and reset.
- Lane config: wrapper `NUM_LANES` and core `N_LANES` both = lanes - 1. A link wider than 2 lanes merges both cores via the wrapper `SCENARIO` and `LANE_MERGE` registers.
- Frequency-range code: `rate_mbps = 2 * source_link_freq`, mapped through 8 coarse bins. The vendor mode (900 Mbps/lane, 2 lanes) maps to range code 5; a 4-lane 456 Mbps/lane link maps to code 3. Overridable with the `phy_range` module param.
- IPI runs in camera-timing mode, cut-through, VCID 0, data type `0x2c` (RAW12).
- Error and interrupt registers are a 10-entry mask table written twice, masked during bring-up and unmasked after. Per-PHY free-running HS activity counters in the wrapper serve as a lane-routing probe.
- **HS-settle (hardware-confirmed, load bearing).** D-PHY internal registers `0x03` and `0x0a` must be written with `0x03` for the vendor's 2-lane 1080p60 mode, after the frequency-range write and before the deskew writes. The PHY reset default samples the HS burst at the wrong instant. Left unwritten, the link produces SoT sync errors on every lane (`INT_ST_PHY` bits 0 and 1) plus double-ECC and frame-boundary errors, the VIF front end measures no geometry, and no frame starts arrive. Written, every error bank reads zero in steady state exactly like the vendor, the front end measures 1924x1084, and frame starts arrive at frame rate. The value is the vendor's own, decoded from its D-PHY test-interface writes in the MMIO trace. This is per-rate: `dphy_freq_conf_get` in `libmpp_service.so` is a table keyed on the per-lane rate in 10 Mbps steps, so other modes need their own settle byte.
- Clock-lane re-acquisition fix (hardware-confirmed): D-PHY internal register `0x3d`/`0x45` bit5 is set during PHY init and must be cleared at the end of configure. Left set, the receiver re-acquires the clock lane every frame and corrupts each frame-start header.
- The full D-PHY internal register set the vendor writes, in order: frequency range `0x00` = `0x05` (900 Mbps/lane), setup `0x4d`/`0x4e`/`0x4f`/`0x50` = `0x00`/`0x08`/`0x00`/`0x08`, lane `0x25`/`0x26` = `0x07`/`0x07`, HS-settle `0x03`/`0x0a` = `0x03`, deskew `0x3d`/`0x45` = `0x20`. Recovered by decoding the paired address/data writes to `PHY_TEST_CTRL0`/`CTRL1` (core `+0x050`/`+0x054`); seven of the nine match constants the driver already carried, which validates the decode.
- `s_stream(1)`: enable csi_clk, enable pcs_clk, configure, then call the sensor's `s_stream(1)` last.

### Capture: `ar-vif.c`

V4L2 video node plus VIF front-end and view-engine programming. Constants are captured from the vendor's live 1920x1080 RAW12 2-lane stream; only geometry and stride are computed.

The front-end half of this driver is validated. The view-engine half implements the bypass view DMA, which the section above shows is not the vendor's capture path.

- Probe claims the reserved capture pool, ioremaps `0x08870000`, gets the DT clock "axi" (`cgu_vif_axi_clk`), sets up a vb2 dma-contig queue and the V4L2/media devices, and registers an async notifier for the CSI source. The completion IRQ is requested only when the `use_irq` param is set; the default is polling.
- The block loses all state when the axi clock gates.
- `ar_vif_global_init`: replays the vendor `vif_reg_init` 28-pair table, a soft-reset pulse, the RGB2YUV coefficient and clip bank (BT.709 limited), input-pipe transient writes, a per-view output-FIFO reset, AXI master enable and run, frame-boundary and DDR-clip config, and the block-enable release. Runs once per session with the clock already on.
- `ar_vif_configure`: front-end input format, geometry, blanking, stride strobe and value, view control, per-view FIFO partition, view mux/enable/threshold, crop window covering the full frame, frame-end check, and input-format select.
- Raw Bayer moves 3 pixels per group, so the crop and frame-end width fields count groups of three.
- A `test_pattern` module param feeds view 0 from the path-0 pattern generator (`0x0f4` bit0).

### Stream-on order

The media graph is sensor -> ar-csi2 -> ar-vif. Stream-on propagates from sink to source:

1. `STREAMON` on the VIF video node runs `ar_vif_start_streaming`: axi clock on, global_init, configure, arm, input path enabled.
2. The VIF calls ar-csi2 `s_stream(1)`.
3. ar-csi2 enables csi_clk and pcs_clk, configures the D-PHY and IPI, then calls the sensor `s_stream(1)`.
4. The sensor powers on, writes its mode table, settles, and writes `0x0100` = `0x01`.

CSI-2 here is a one-way push with no handshake. The sensor streams autonomously once its mode table and `0x0100` = `0x01` are written; the receiver's D-PHY range is chosen from the sensor's reported link frequency rather than negotiated; and the IPI is a FIFO with no acknowledge crossing it. The only coordination is ordering: the sink is armed before the source streams.

## Clocks and power

Leaf clocks enabled through the Linux clk framework, all DT-defined:

| Clock | DT name | Provider index | Enabled by |
|---|---|---|---|
| `cgu_sensor_mclk0` (24 MHz) | mclk | pixclk 35 | `nt99235_power_on` |
| `cgu_mipi_csi_0_clk` | csi | pixclk 13 | ar-csi2 stream-on |
| `cgu_mipi_pcs_clk` | pcs | pixclk 15 | ar-csi2 stream-on |
| `cgu_vif_axi_clk` | axi | pixclk 7 | ar-vif start-streaming |

Source selects are done by a manual `/dev/mem` prologue poked before insmod in the test scripts, not modeled by the drivers. These mirror the vendor `cam_hw_*_power_on` sequence, which runs before any CSI or VIF register is written:

- `0x0a104010` low half = `0x1300`: `cgu_vif_axi_clk` parent select 3 (fix_pll_clk333) plus gate. The vendor runs VIF AXI at 333 MHz.
- `0x0a10400c` = `0x13001300`: `cgu_isp_clk` (low half) and `cgu_isp_hdr_clk` (high half), both select 3 and gated.
- `0x0a104020` low half = `0x1103`.

The open CGU provider models these as gate-only leaves, so `clk_prepare_enable` sets the gate bit but not the parent-select mux. The vendor prologue additionally sets the parent selects and enables `cgu_isp_hdr_clk`, which the open stack does not.

### What the vendor programs, measured

Read live from a streaming stock unit and cross-checked against a read+write trace of the CGU window, which contains only 66 writes for the whole camera bringup. Reference copy: `glue/camera/cgu-vendor-streaming.txt`.

| Register | Vendor value | Open stack |
|---|---|---|
| `0x0a10400c` | `13001300` | set outright, matches |
| `0x0a104010` | `02001300` | low half set, high half preserved |
| `0x0a104020` | `10001103` | low half set, high half preserved |
| `0x0a104018` | `04011300` | not programmed |
| `0x0a10401c` | `02011201` | not programmed |
| `0x0a104044` | `01001000` | not programmed |
| `0x0a102004` | `f7ffffdf` | not programmed |
| `0x0a102014` | `e7ffdfff` | not programmed |
| `0x0a102008`, `0x0a102018`, `0x0a102028` | `00000000` | not programmed |
| `0x0a106008`, `0x0a106200` | `88922000` | not programmed |
| `0x0a106204` | `00a05f5f` | not programmed |
| `0x0a106208` | `00005b5b` | not programmed |
| `0x0a108098` | `0000002c` | not programmed |
| `0x0a108410` | `07684bda` | not programmed |


`0x0a102004` and `0x0a102014` are reset control and are written twice each, in an assert-configure-release order: first with a bit clear (`f3ffffdf`, `e7ffdffb`), then the `0x0a106xxx` block is configured, then with that bit set (`f7ffffdf`, `e7ffdfff`). A block held in reset accepts register writes and reads them back correctly while its datapath does nothing.

`0x0a104018` is the register the vendor polls, 17,434 reads in a 34-second trace, far more than any other register in any window.

## Status

The full chain, sensor through CVISP, captures processed colour frames on the open stack, validated by marker count and by rendered images. Per-block state against the streaming vendor:

- **Sensor.** Configuration verified equivalent: mode table matches the vendor library write for write in order, and 184 live registers are identical outside the 26 the vendor's 3A adapts at runtime. Exposure and gain are driven explicitly (see above); the vendor drives them from AE.
- **CSI-2 link.** Every `INT_ST_*` bank reads zero on the steady-state second read, matching the vendor. The banks are clear-on-read: always read twice.
- **VIF front end.** Measures the incoming timing correctly (`0x1f0` = `0x0784043c`) and matches the vendor's live values; frame starts fire at frame rate. The four registers the vendor's end state differs in (`0x080`, `0x08c`, `0x140`, `0x2bc`) were set to vendor values on hardware with no effect.
- **ISP.** Register state vendor-identical over the modeled range; every DMA descriptor points at driver-generated memory (see "What the driver owns").
- **CVISP.** Configured, armed per frame start, sustains 60 fps.
- **CGU.** The open stack programs three of the fourteen registers the vendor programs; the rest, including two reset-control writes, are inherited from boot state. Unexplained but measured harmless in the working chain.

Two trace-reading facts that stay relevant: the vendor never spin-waits (no consecutive-read run on any block), and the vendor brings the pipeline up, tears it down, and brings it up again, so the head of a trace shows first-bringup behaviour, not the working path.

The cold-boot dark frame is clean (see "The cold-boot dark-frame blobs"); the edge notch on synthetic bars is the sensor's own test-pattern generator, not an ISP stage: edge-aligned averaging over 3,400 real-scene edges shows no trace of it at optical edges, so it never affects live imagery.

### Gain-keyed stages: parity by derivation

Three ISP stages are recomputed by the vendor from gain-band ladders in the tuning file: `rnr`, `lnr` and `de3d`. All three share one blob layout (a header of enable, interpolate, band count and abscissa selector; sixteen `[low, high]` float32 band slots; one payload record per band) and one law: inside a band the record applies verbatim, between bands the fields blend linearly with truncation toward zero. The driver implements the shared band selection and blend in `ar-isp-ladder.h` and each stage in its own `ar-isp-rnr.h`, `ar-isp-lnr.h` and `ar-isp-de3d.h`, deriving every register from the tuning file at a caller-supplied abscissa (`rnr_gain`, `lnr_gain`, `de3d_gain`, Q8 module parameters until an AE loop produces the abscissa per frame). The proofs are `scripts/isp/check-rnr-ladder.py`, `check-lnr-ladder.py` and `check-de3d-ladder.py`: the same integer arithmetic in Python, refusing to pass unless every capture-covered register reproduces bit-exactly from the blob at both measured vendor operating points.

Registers in these banks that are not ladder outputs are classified, not assumed: lnr `0x3d10` is never written by the vendor and `0x3d14` keeps a replayed value (unresolved pre-pack bias); de3d `0x2e98` is the block's hardware line-time latch and is never written; the de3d buffer addresses are driver-owned; the per-stage user-strength laws are the identity at the vendor's default of 50 and nothing in this stack sets a strength, so they are not carried. The de3d control and strength words (`0x2e00` enables, `0x2e1c`, `0x2e20`, `0x2e90`, `0x2eb4`) are packer outputs and derive with the rest; running them at the replayed pre-streaming state instead is what destroyed moving regions even with the threshold registers exact.

The census over the whole tuning blob and bank map is closed: the powers-of-two ladder signature exists for exactly `blc`, `rnr`, `lnr` and `de3d` among enabled stages (plus disabled hits owned by `gib` and two unused tables). Checked and found static: `dpc` (init_config block), `rgb2yuv`, `qgg`, `lms`, `digigain1/2`. Two further enabled stages recompute on the AEC trigger through their own record blends rather than the shared ladder layout and still run replayed state here: `cfa` and `cnf`; their transforms are the next recovery. The AWB-triggered family (`ccm1`, `cm`, `cm2`, `acm`, `wb`, and LSC's selection among its four illuminant groups) and the tone tables (`gamma`, `drc`, `ltm`) recompute from their own 3A inputs and wait on the AWB and AE loops.

### Scene-adaptive state still frozen

Parity above is at a fixed operating point. The vendor's 3A loops move state this stack holds constant: the sensor exposure and gain (AE, including its anti-flicker snap of the integration time to the mains half-period), the ladder abscissas (an AE product), the gamma table (regenerated continuously from 3A; ours is generated once), the AE-selected tone-table index over the gamma and DRC profiles, the LTM tile curves (recomputed per frame; ours ships an identity page), the AWB-driven colour state (ccm1, white-balance gains, and the sensor-MCU lens-shading tables behind `0x8201`), and possibly a scene-adaptive LSC region after the static grid (unconfirmed; ours ships zeros there). These are the remaining functional gaps, and they are loops to implement, not registers to fix.

## Source map

| Concern | Files |
|---|---|
| Sensor init, MCLK, stream-on | `overlay/drivers/media/artosyn/nt99235.c` |
| CSI-2 D-PHY, lanes, IPI, deskew | `overlay/drivers/media/artosyn/ar-csi2.c` |
| VIF front end, view arm, DMA | `overlay/drivers/media/artosyn/ar-vif.c` |
| ISP configuration | `overlay/drivers/media/artosyn/ar-isp.c`, tables `vendor-tables/ar-isp-defaults.h` from `scripts/isp/gen-isp-defaults.py` |
| CVISP output stage and queue | `overlay/drivers/media/artosyn/ar-cvisp.c`, tables `vendor-tables/ar-cvisp-defaults.h` from `scripts/isp/gen-cvisp-defaults.py` |
| DT nodes (camera, CSI, VIF, ISP, CVISP, clocks, carveouts) | `devices/betafpv-vr04-air/proxima-9311-air.dts` |
| Vendor MMIO write trace, per block and wide sweep | capture harness `glue/camera/au-slotA-mmiotrace.sh`, shim `native/mmiotrace.c`; the logs are capture artifacts, not in the tree |
| CVISP first-light experiment | `glue/camera/au-cvisp-firstlight.sh` |

Precedence between sources: the trace is the authority for what the vendor writes, the disassembly is the authority for what a register means, and hand-written RE notes are leads to verify. Notes checked against the trace have been wrong on `0x080`, `0x32c` and `0x0d0`.

### Judging a capture: the marker count, never the image

`ml-isploop` writes 24 marker words (`0xa5a5a5a5`, every `0x1000` bytes) into each output plane before its capture window opens, and reports how many survived. **A capture is only ours if those markers were overwritten.** `0/24` means nothing in our stack wrote that memory and the dump is whatever was already in DRAM.

DDR survives a RAM-boot, so the vendor's last frame from slot A persists at the same addresses our ring uses, and reads as a plausible healthy capture. A residual vendor frame is also useful deliberately, as a same-sensor same-scene control through the vendor pipeline; dump it before our own stack runs.

### CVISP writes only on the first bring-up after a RAM-boot

The first camera bring-up of a boot writes frames to DRAM. Every later bring-up in the same boot arms normally, reports full frame-start and slot-arm counts, and writes nothing, leaving the first bring-up's frame in place.

Measured: first bring-up 24/24 markers overwritten on all three planes; second and third `0/24`, each with 241 frame starts and 241 slots armed in 4 s and CVISP control `0x00800806` correct throughout.

This is not simply cold-versus-warm. Slot B is RAM-booted from slot A with the vendor pipeline already streaming, so the first bring-up **inherits hardware state the vendor established**, and the register replay does not recreate it once our own teardown has destroyed it. A boot therefore yields exactly one trustworthy capture; `RUNS=` in `glue/camera/au-prove-camera.sh` selects which one to spend it on.

### Within a working bring-up, capture sustains

All five ring slots hold distinct frames after a one-second window, differing by 10 to 16 per cent of sampled pixels. The pipeline streams at 60 fps; it does not produce a single frame and stop.

### The shadow crush was two output-stage registers

The crush was `0x2e2c` and `0x2e30`, whose corrected values sit past the prefix the replay applies; writing them recovers the whole shadow range (57% of pixels under luma 32 becomes 0.0%). See `ar_isp_output_fix` in `vendor-tables/ar-isp-defaults.h`.

A related trap, worth keeping: the vendor's coefficient pages survive in DRAM across a RAM-boot at the addresses the replay arms, so a pipeline that does not generate its own tables runs correctly on **inherited** content and only fails on a cold boot. `ar-isp.c` generates every byte the hardware fetches (gamma page 0 and the DRC dynamic half from the tuning file; gamma page 1 and the DRC tail carried as decoded curves via `scripts/isp/gen-gamma-page1.py` and `scripts/isp/gen-drc-tail.py`). Packed multi-lane table formats are neither mostly zero nor monotonic read as flat `u32`, so zero-fraction and monotonicity heuristics score the packing, not the content.

## What the driver owns

Validated on hardware with seeding off, so nothing in the "ours" rows came from the vendor's residual DRAM.

**Coefficient tables the block fetches:**

| Table | Descriptor | Source | Status |
|---|---|---|---|
| gamma | `0x0030`/`0x0040`/`0x0050` | tuning file + carried page 1 | every fetched byte ours |
| DRC | `0x0060` | tuning file + carried tail | every fetched byte ours |
| compander | `0x0020` | library template entry 6, verbatim | every fetched byte ours, byte-exact |
| HDR page | `0x1c6c` | nothing to generate | ours; zero over its whole extent, matching the vendor |
| LSC | `0x4c34` | tuning file, two float32 arrays | every fetched byte ours, exact; the fetch is `0x340`, see below |
| hdr_lsc | `0x1e38` | zero page | parity: the stage ends disabled and the vendor's own page is stale heap |
| CCM | registers `0x3400`/`0x3800`, no DMA | tuning file, packed Q8 sign-magnitude | ours; all six words match the trace exactly |
| BLC | CVISP registers `0x4200`, no DMA | tuning file, gain-blended calibration entries | ours; reproduces the traced registers exactly |
| rnr | registers `0x1808`-`0x1834`, no DMA | tuning file, gain-ladder blend (`ar-isp-rnr.h`) | ours; band 0 is byte-identical to the replayed cold bank, and the vendor's live bank reproduces at abscissa 13.6-14.2 (`scripts/isp/check-rnr-ladder.py`) |

**Statistics buffers the hardware writes**, allocated and published by the driver, confirmed on silicon:

| Buffer | Register | Size | Validation |
|---|---|---|---|
| zone grid (`rro_stats`) | `0x6440`, `0x6474` | `0x4800` | count 826, matching the vendor exactly |
| second grid (`rro_face_stats`) | `0x6508` | `0x4800` | count 90, matching the vendor exactly |
| Bayer histogram (`raw_hist_stats`) | `0x600c` | `0x1000` | lanes exactly 518400 / 1036800 / 518400 / 0 |

`de3d`'s three working buffers are allocated and published the same way, at `0x2e3c`/`0x2e44`, `0x2e58`/`0x2e60` and `0x2e80`/`0x2e88`. These are the one case where the driver does not reproduce a vendor value but substitutes storage of its own, and the sizes `0x1db000`, `0xef000` and `0x1db000` are bounds derived from the gaps between the vendor's allocations rather than measured extents. A bring-up with `de3d=1` delivers all 24 markers on all three planes and a frame with no visible artefact, which establishes that the hardware accepts the substitution; a single still frame cannot rule out a temporal artefact, so the bound-derived sizes remain the weakest evidence in this section.

Those histogram lanes are precisely a quarter, a half and a quarter of `1920 x 1080`, with the fourth lane exactly zero, read from a buffer this driver allocated. That is the Bayer population, and it is the strongest single confirmation that the layouts in `ar-isp-stats.h` are right. **AE has a working input.**

**BLC is the first stage that recomputes with gain, which makes it an AE dependency rather than a table.** Sixteen registers on CVISP bank `0x4200`, filled by a verbatim 64-byte copy. The payload is five calibration entries in the tuning file at `0xb4`, selected by a ladder of five float pairs at `0x34`. A gain inside a pair's own range uses that entry alone; a gain between one pair's second bound and the next pair's first bound blends the two across that band:

	t   = (gain - ladder[lo].second) / (ladder[hi].first - ladder[lo].second)
	out = entry[hi] * t + entry[lo] * (1 - t)

with the first group of four shifted left by 6 on its way to the registers and the second group unshifted. Recovered from the selection at `0x1bfef8` and the blend at `0x1c0048`.

The vendor's operating point is pinned exactly: at gain **187** the band is 130 to 510, `t` is 0.150000, and all four lanes come out at 961 and 272, which are the `0xf040` and `0x110` the trace holds. The blend reproduces both registers on all four lanes from the tuning file.

**In practice BLC does not track gain at all.** Measured on a streaming vendor unit with the sensor dark and AE saturated: sensor gain code `0x5f` (62x) with the BLC bank still holding 961 and 272 on all four lanes, the same values as at the traced 14x point. The blend exists in code but the vendor runs one operating point of it, so the driver's fixed 187 is measured parity, not an approximation. BLC is also lane-uniform by construction (one level for all four lanes at every calibration entry), which excludes it as a source of any coloured field.

**BLC is not the only stage that recomputes with gain, and the rest are still frozen at the operating point the capture happened to sit at.** Every stage below reaches the AE trigger and converts between integer and float inside its own code, which is what distinguishes a gain-indexed recomputation from static configuration. Auto-exposure moving gain therefore walks the picture away from the only point those stages are correct for, which is a prerequisite for AE being any good rather than a completeness item.

| Stage | Table | Status |
|---|---|---|
| `rnr` | `0x7a6c`, stride `0x160`, 12 entries | implemented: the driver derives the twelve registers from the ladder (`ar-isp-rnr.h`), validated against both measured points |
| `lnr` | `0x89f18`, stride `0x428`, 11 entries | table pinned, source fields known, destination registers not established |
| `de3d` | `0x963ac`, stride `0x2f8`, 12 entries | as `lnr` |
| `acm`, `cfa`, `cnf` | not located | carry a recomputation path that did not fire during this capture |

`rnr`, `lnr` and `de3d` all blend between two adjacent entries by an AE-supplied weight, exactly as BLC does, and truncate on the way to the registers. The weight's abscissa is a linear gain multiplier with unity at 1.0 (rnr's band edges are powers of two), but it is not the sensor analog multiplier: the vendor's live rnr bank demands 13.6-14.2 where the analog gain was 62, the same unresolved scale error BLC shows at 2.6x. The driver therefore takes the abscissa as the `rnr_gain` module parameter (Q8, default 1.0, which is the replayed cold bank exactly) rather than deriving it from the gain code; one vendor capture of the sensor gain register and ISP `0x1808` together pins the unit.

**lnr additionally lost its static curves to the prefix cut.** The applied 1475-entry prefix truncates lnr's four 64-entry strength curves (`0x3d60` onward, stride `0x40`, normalised to `0x40`) mid-block, leaving the tail zero, which is gain zero rather than neutral. `ar_isp_lnr_fix[]` in `ar-isp.c` restores the 35 affected registers with the streaming vendor's live values, applied in both configure paths after the setup replay. Found by the register-state diff (`scripts/isp/isp-regdiff.py`), which classifies every live difference against the driver's own tables; the sweep files carry window-relative row offsets, so that tool is the only sanctioned way to read them.

**The hdr-path descriptors are owned and quiescent.** `hdr_lsc` (bank `0x1dd0`, length `0x1e2c`, address `0x1e38`, valid `0x1e40`) gets a driver-owned zero page: the vendor's own live page at `0x2b2e8c00` is stale heap (the fill path never runs in the FPV configuration) and the stage's control word ends disabled on both sides, so zero is parity, not a placeholder. Both module-local valid bits (`0x1e40` and HDR's `0x1c60`) are cleared at the end of the output arm, matching the vendor's measured steady state of arm, fetch, de-validate; the driver previously left them set after publishing.

**`acm`, `cfa` and `cnf` are the subtle case.** Their registers are constant across the whole trace and are replayed with the vendor's values. What is not established is that they *cannot* move: each reaches the AE trigger and carries float conversion, so they are correct at the capture's operating point rather than correct by construction.

**CCM was not merely unowned, it was inactive.** The register replay does carry the vendor's runtime colour matrix, but at entry 1718 of the setup table, while the camera harness applies a 1475-entry prefix. Every bring-up before this one therefore ran with ccm1 holding the identity that earlier entries wrote, so colour correction was switched off and it was not obvious, because an identity CCM yields a plausible picture rather than a broken one. The driver now installs the matrix itself, after the prefix. Generated from the tuning file, the six words reproduce the traced vendor registers exactly.

Without AWB there is nothing to interpolate between, so one illuminant bank is installed verbatim; bank 0 is the one the vendor was traced writing, which makes this reproduce the traced state rather than approximate it. `ccm_bank=` selects another. ccm2 is installed from its static init block and never moves, because its tuning gate reads 0.

Publishing has an ordering requirement that is easy to get wrong and was got wrong once. The setup table carries the vendor's own statistics addresses at entries around 1417 to 1425, so publishing only from `ar_isp_tables_apply` is overwritten by the table or by any prefix past that point. The driver therefore publishes **twice**, the second time from `ar_isp_arm_output`, after the table has run. The symptom of getting this wrong is a buffer that stays all zero while everything else looks healthy.

**Not owned, and why:**

| Item | Why not |
|---|---|
| the `gib` bypass bit | vendor writes ISP `0x2408` bit 30, this driver does not; measured incapable of the resolved blobs, parity write queued |
| the `0x1c6c` payload past `0x800` | runtime state, no stored source |
| `af_stats` buffer | extent known (`0x1200`), autofocus is off and nothing writes it |
| 18 further stages | register files |

**The cold-boot dark-frame blobs were LSC fetching the vendor's dead page; resolved by the descriptor republish.** The setup table carries the vendor's own LSC descriptor write, so the table-time publish was overwritten and the block fetched shading from the vendor's decayed DRAM on every cold boot. Junk per-Bayer-channel gains are multiplicative, which is how the blobs were nearly pure chroma-axis excursions that could push a channel below its background (no additive source can): position-stable across power cycles because DRAM decay patterns are physically stable, amplitude and mix drifting with power-off time, absent on warm boots because the vendor's genuine page was still resident at the fetched address, and invisible on the test pattern because its bars are saturated where multiplicative errors clip. Fixed by republishing the LSC descriptor after the register replay (see the ordering note above); validated on two independent cold boots: dark frames neutral (every blob mask under 1.3 counts) with the vendor's radial shading shape (corners 34-37 against centre 23, vendor 37-40 against 23.2).

Background facts that remain true and useful: the cold sensor's floor sits below the pipeline's black clip while a warm sensor's dark current lifts it clear, so dark-frame judgments need the floor off the clip; no configuration state crosses a RAM boot (warm and cold sweeps differ only in counters); `libsns_nt99235.so` performs no dark or FPN calibration, and the sensor init is at parity to one register. Remaining dark-frame artifacts, both open: the ~16-column left-edge ramp (geometry words, see the register diff) and the hard ring contours with periodic dashes against the vendor's smoother floor (the gain-keyed noise stages run at a replayed operating point; the dashes are not dpc, measured by the live-disable test).

**The LTM descriptors are owned.** The driver allocates and publishes both `0x2808` (coefficient page, `0x4000`) and `0x280c` (statistics buffer, `0x80000`) under the `ltm=` parameter, default on. The page carries an identity curve, 64 tiles each holding the linear ramp `i * 1003 / 127` over 128 `u16` samples; the statistics buffer is zeroed at prepare. Before this the descriptors still held vendor DRAM addresses, and they were the last DMA fetch the driver did not source: on a cold boot (no prior slot-A streaming) that DRAM is junk and every frame came out marbled and posterized regardless of driver version. With the identity page a cold boot with `seed=0` produces a clean frame. Replacing the identity curve with a per-frame recompute from `ltm_stats` is the remaining open item; the vendor's adaptation magnitude is about 2% of range.

**LTM is computed per frame by the vendor, but the adaptation is small.** The page is 64 tiles of a 128-sample `u16` curve at `0x100` stride, monotonic 0 to about 1003. Exactly one of the three captured pages is well formed; the other two are noise and must not be used as oracles, and it deviates from the identity ramp `i * 8` by -14 to +34 counts of 1016, growing down the tile grid. The identity page is therefore within the vendor's own adaptation envelope; the real computation (CLAHE, see the LTM section below) is queued, not required for a usable image.

`ltm_stats` fills its **entire `0x80000`**, measured at 99.8% nonzero right to the last byte, so the half-megabyte figure is a real fill extent and not just an allocation.

Descriptor ownership by vendor module: `0x4c34` is `isp_sub_lsc` (bank `0x4c00`), `0x1c6c` is `isp_sub_hdr` (bank `0x1c00`), and `0x2808`/`0x280c` are the shared `gtm2`/`ltm` pair (bank `0x2800`). A descriptor's name is worth nothing until its bank is attributed to a module; the attribution method is in the shading and colour section below.

Compander needs no generator: the `0x7800` page is installed verbatim at ISP init from entry 6 of the descriptor array at VMA `0x472600` (`{u64 source, u64 length}` pairs), whose body at VMA `0x46a3b0` is byte-identical to the page captured off a streaming vendor unit. `scripts/isp/gen-compander.py` extracts it and `ar_isp_compander_fill` rebuilds it.

Three quarters of the page is one 16-byte unity record repeated 1536 times and a further `0x700` bytes are zero, so only `0x900` bytes at the start and `0x800` at `0x1000` are carried: 4352 bytes rather than 30720. The generator script checks that structure against the library and refuses to emit if it has changed.

### LSC: the lens-shading grid is in the tuning file

LSC's fetch is `0x340`: 52 sixteen-byte records, 50 of grid and two zero. The block reads nothing past that. The page sits in a `0x600` arena slot (`init_submodule_lut` carves one arena; `isp_sub_lsc` takes index `0xb` at `arena + 0x8400`, the next slot starts `0x600` above), and the `0x2c0` remainder of the slot is unfetched stale heap. Three independent proofs of the fetch length: the vendor flushes exactly 832 bytes at the publish site, the length register `0x4c28` holds `0x34` sixteen-byte records, and the neighbouring descriptor's page sits `0x600` above so a longer record unit would overlap it.

	0x000..0x33f   10x10 lens-shading grid, generated from the tuning file
	0x340..0x5ff   unfetched remainder of the arena slot, stale heap, no meaning

**Region A is two 100-entry float32 arrays in the tuning file**, stored back to back at `raw + 0x910c` and `raw + 0x929c`, which is `0x7c` and `0x20c` past the LSC enable gate at `0x9090`. Each value is a gain, unity at the frame centre and rising to about 3.9 at the corners; the grid is a proper 10x10 bowl with an off-centre, anisotropic falloff. The table value is `floor(f * 2048)`, and truncation is measured rather than assumed: rounding matches 55 of 100 entries against a captured page, truncation matches all 100. Grid points pack two to a 16-byte record as `(x, x, y)` triplets, 50 records of data then two zero records. `ar_isp_lsc_from_blob` reproduces all 832 bytes exactly against two independent captures.

The blob stores the grids unpacked as `float32`; searches for the packed `u16` form find nothing, which once produced a wrong "no stored source" conclusion. The scene-to-scene byte drift once attributed to a "region B" is heap churn in the unfetched slot remainder and means nothing.

### The coefficient pages overlap in DRAM

Measured, zero differing bytes in both directions:

	HDR page   0x2b2e0200   fetches 0x1000, holds 0xa00 of content
	compander  0x2b2e0c00 = HDR page + 0xa00
	LSC        0x2b2e8600 = compander + 0x7a00

So the HDR page's `0xa00..0xfff` **is** the compander table's first `0x600` bytes, read because it over-fetches past its own content, and a `0x8000` dump of the compander runs into the LSC page. Its real payload is only the 512 bytes at `0x800..0x9ff`; `0x000..0x7ff` is zero.

`ar-isp.c` reproduces this rather than working around it: the HDR page and the compander share **one allocation**, the HDR page at offset 0 and the compander at `+0xa00`, and the two descriptors are published into the same block. That reproduces the fetched bytes for both without copying the shared `0x600` twice. The compander span is the `0xf000` its length field at `0x0024` implies rather than the `0x7800` the table occupies: in gamma's proven 32-byte units that is a fetch the vendor cannot satisfy either, since `0xf000` past its compander runs into the gamma page, so the excess is ignored by the block and allocating it only keeps the DMA inside memory we own.

### The HDR page needs nothing generated

Its `0x1000` fetch is `0x800` of zeros, then `0x200` of payload, then `0x600` of compander. The payload has no stored source: absent from the tuning file, from the service library, and from all 53 non-null entries of the ISP-init template array, and a float-correlation scan of the kind that located the LSC shading grid finds nothing above `r = 0.24` at 256, 128 or 64-value windows.

Its bytes drift between captures and carry the same heap fingerprint as the unfetched LSC slot remainder (entropy 5.50 against 5.53, bit 7 set in 78% of bytes in both): one cause, stale service heap, not two similar algorithms.

Zeroing it was measured on hardware to move 6.3% of pixels by more than eight levels, against a 94.5% frame-to-frame floor from scene motion alone, so the driver leaves it zero. **Do not spend further effort searching for its source.**

### Committing a coefficient table

Each table has a descriptor register holding a physical address, and a bit in `0x0014` that makes the block fetch it. The commit is **write-to-trigger, not a set-then-clear pulse**: clearing the bit afterwards cancels the fetch, and a fill only takes effect if it precedes the write. Bit 16 is set alongside every commit. From the vendor trace, one table at a time:

	0x0020 = 0x2b2e0c00                     0x0014 = 0x00010001    compander
	0x0060 = 0x2b2e9200                     0x0014 = 0x00010010    DRC
	0x0030 / 0x0040 / 0x0050 = 0x2b2ec600   0x0014 = 0x0001000e    gamma

Gamma has three descriptors and the vendor points all three at one buffer. The sequence is not a one-time setup: the vendor reissues it on every AE update, which is what makes republishing our own addresses after the replay an ordinary operation rather than a special case.

**Each descriptor also has a length, and it is not the allocation size.** `0x0034`, `0x0044` and `0x0054` hold the transfer length in units of 32 bytes. The vendor writes `0x200` during setup, describing the whole `0x4000` allocation, and then **`0x80` immediately before the commit**, describing `0x1000`. The three slots carry the same base and the same length with no offset or stride field between them, so they are channel aliases fetching the same first `0x1000` bytes, not three slices of the buffer. That is also why the three captured gamma dumps are byte-identical.

Two consequences. The `0x4000` memcpy in the vendor handler is the size of its software allocation and must not be read as a DMA length. And **the tail from `0x1000` to `0x3fff` is never fetched**: it is per-record software state that rides along in the same copy, which is why it decodes as high-entropy nonsense and why it differs between captures without meaning anything.

Our replay already programs the streaming length: `0x80` appears at `ar_isp_setup_1080p60` indices 470, 472 and 474, inside the 1475-entry prefix the harness applies. Only pages 0 and 1 need to be correct.

### The HDR page and LSC do not use the 0x0014 commit at all

They are module-local descriptor records, not entries in the global table selector. There is no `0x0014` bit for either; the only global commits are compander bit 0, DRC bit 4 and gamma bits 1 to 3.

| | pointer | length | valid |
|---|---|---|---|
| HDR page | `0x1c6c` = `0x2b2e0200` | `0x1c74` = `0x80`, so `0x1000` fetched | `0x1c60` |
| LSC | `0x4c34` = `0x2b2e8600` | `0x4c28` = `0x34` sixteen-byte records, so `0x340` fetched | `0x4c3c` |

Recovered from the vendor write trace and independently confirmed against a live register read of a streaming vendor unit; the pointers agree exactly. Both valid bits read `0` mid-stream, so like the `0x0014` commit they appear to self-clear after the fetch.

The length fields matter for the same reason gamma's did: a flush size is the software allocation, not the fetch, and the record unit differs per descriptor (HDR's `0x80` is 32-byte units, LSC's `0x34` is 16-byte records). The LSC handler is `isp_sub_lsc`'s at `0x1b6944`: publishes `+0x34`, valid `+0x3c`, length `0x34`, flush `0x340`.

A captured `0x1c6c` page truncated to `0x1000` and a captured `0x4c34` page truncated to `0x340` are exact oracles for the two pages.

### The AE selector is a threshold table in the tuning file

Each module carries a table of float thresholds giving every entry an active band. Gamma's is at blob `0x26b0c`, ten floats, with the curve count `5` stored just before it at `0x26b04`. DRC's is at `0x17a9c`, twelve floats.

```
gamma, 5 curves            drc, 6 profiles
  0:    0 ..  40             0:    0 ..  80
  1:   80 .. 130             1:  100 .. 130
  2:  150 .. 250             2:  150 .. 180
  3:  280 .. 330             3:  210 .. 270
  4:  360 .. 450             4:  290 .. 380
                             5:  410 .. 500
```

The gaps between bands are the interpolation regions: inside a band one entry is used, between bands a Q12-weighted mix of the pair either side.

The model was checked against something it was not fitted to. The bands come from the blob and the indices come from decoding captures, independently. One scalar drives both modules, so the bands our captures decode to must intersect, and they do:

	session A   gamma curve 2 [150,250]  n  drc profile 3 [210,270]  =  [210,250]
	session B   gamma curve 3 [280,330]  n  drc profile 4 [290,380]  =  [290,330]

**The units of that scalar are not established.** It spans 0 to 500, and both a light level in lux and a total gain in percent fit the evidence, with opposite physical meanings but the same self-consistency. Do not record either as fact. Settling it means tracing what value reaches `is_aec_trigger_compute_user`.

### Shading and colour: LSC, LUT3D, CCM (recovered)

Recovered by static analysis of the module code in `libmpp_service.so` plus the vendor MMIO trace; nothing here required a hardware run. Each `isp_sub_*` module registers three handlers from its `_creat`; the second maps the module's register bank (an `ar_dev_pa2va` call pair with the bank offset as an immediate) and the third is the command handler that fills it. The bank constants attribute the register map to modules:

| Module | Register bank | Descriptor / payload |
| --- | --- | --- |
| `isp_sub_ccm1` | `0x3400` | register file, no DMA |
| `isp_sub_ccm2` | `0x3800` | register file, no DMA |
| `isp_sub_lsc` | `0x4c00` | descriptor `0x4c34`, valid `0x4c3c`, length `0x4c28` = `0x34` records, the `0x340` fetch |
| `isp_sub_lut3d` | `0x5800` | four descriptors at `0x5810`/`0x5828`/`0x5840`/`0x5858` |
| `isp_sub_gtm2`, `isp_sub_ltm` | `0x2800` | descriptors `0x2808`/`0x280c`, both publish sites verified at `+0x08` |
| `isp_sub_digigain2` | CVISP `+0x4700` | register file, no DMA |

`isp_sub_lsc`'s command handler publishes the DMA address to bank `+0x34`, sets valid at `+0x3c` and writes length `0x34`, exactly the `0x4c34` record, and its tuning path reads the enable at `raw + 0x9090`.

The float region past the LSC gate holds sixteen 10x10 grids, not one pair: groups at `raw + 0x910c`, `0x9784`, `0x9dfc`, `0xa474`, stride `0x678`, four back-to-back `0x190` grids per group behind a `0x38` header. The runtime path copies one whole `0x678` group verbatim (`memcpy` at `0x1b4ac8`); what selects the group index is untraced. The shipped pair is byte-exact against two captures. The fourth grid of each group (`+0x4b0`) has no located consumer.

**LUT3D is present, armed, and disabled on the streaming vendor.** The init handler copies ISP-init template entries 42 to 45 verbatim (`0x458960`, `0x4562e0`, `0x453c60`, `0x4515e0`, `0x2680` each, four distinct banks of 16-byte records with nine content bytes) into four DMA banks, writes per-descriptor length `0x280` in 16-byte records (an over-fetch, flush is `0x2800`), publishes the four addresses and valid bits, and never reads the tuning file for payload. The tuning gate at `raw + 0x7b634` only drives module control `0x5800` bit 0 through the apply-tuning command. The working bringup's last write to `0x5800` is 0 and the whole bank reads zero on the streaming unit, so the module is off and the driver reproduces the vendor by leaving it off. The four banks are deliberately not carried in-tree; `ar-isp-colour.h` records the register layout and the template VMAs to extract them if the stage is ever enabled.

**CCM lands in registers, not a DMA page.** Both banks are `0x50` bytes: a packed 3x3 matrix at `+0x00`, a second copy at `+0x20`, a zero tail at `+0x40`. A matrix is six words, `{c0|c1<<16, c2, c3|c4<<16, c5, c6|c7<<16, c8}`, each coefficient 16-bit sign-magnitude Q8 with the magnitude truncated toward zero (`fcvtzs #8`; a negative v is encoded `0x8000 + |v|`). At init ccm1 gets an identity pair (template entry 33) and ccm2 gets the vendor's fixed matrix pair (entry 34, word-identical to the traced registers; its tuning gate at `raw + 0x2595c` reads 0, so it never moves). At runtime the AWB path packs an interpolated tuning matrix into ccm1's first copy only: gate `raw + 0x253fc` reads 1, the illuminant ladder is eight kelvins at `raw + 0x25438`, and four of eight matrix slots at `raw + 0x25470` (stride `0x24`, nine float32 row-major) hold data, matching the bank count in the selector block. The traced runtime write is bank 0 packed verbatim, which is the byte-for-byte proof: `scripts/isp/gen-ccm.py` re-packs it and refuses to emit `vendor-tables/ar-isp-ccm-init.h` on any mismatch. The packing functions are in `ar-isp-colour.h`. CCM enable is not a bank register: ccm2's enabled path clears bits 25 and 27 of the ISP global control word at `+0x0000` (both already 0 in the live streaming value `0xb0280052`).

### Statistics: the eight per-frame addresses, and the AE input (recovered)

Recovered by static analysis plus the mid-stream captures; no hardware run. The bank-attribution method from the shading and colour work settles the assignment: each `isp_sub_*_stats_creat` registers three handlers, and the second maps the module's bank with the bank offset as an immediate beside its `ar_dev_pa2va` calls.

| # | Register | Module | Bank | What it is |
| --- | --- | --- | --- | --- |
| 1, 2 | `0x75a0`, `0x75bc` | `af_stats` | `0x7400` | autofocus, module disabled |
| 3, 4 | `0x6440`, `0x6474` | `rro_stats` engines 0 and 1 | `0x6400` | **the AE zone grid** |
| 5 | `0x600c` | `raw_hist_stats` | `0x6000` | **the Bayer histogram** |
| 6 | `0x280c` | `ltm_stats` | `0x2800` | LTM statistics output |
| 7 | `0x6508` | `rro_face_stats` | `0x64c8` | a second zone grid, smaller window |
| 8 | `0x2808` | `ltm` / `gtm2` | `0x2800` | coefficient input, not statistics |

Bank `0x6400` holds **two** independent engines at a `0x34` stride, and `rro_face` at `0x64c8` is a third instance of the same block. The vendor programs all three to the same geometry and points both `0x6400` engines at one buffer, which is why captures 3 and 4 are identical bytes.

**The metering grid is in the hardware registers, not just in the algorithm.** Each engine takes columns at bank`+0x1c`, rows at `+0x20`, frame width at `+0x2c`, height at `+0x34` and the buffer address at `+0x40`. The vendor writes `36`, `16`, `1920`, `1080`. That is the same 36 by 16 the AE code loads, established independently, so `rro_stats` is the AE metering producer.

**The zone buffer is `0x4800` bytes, column major, with the divisor stored inline.** 36 columns of `0x200`: a `0x100` count block then a `0x100` sum block, each 16 zones by 4 channels of `u32`. Every word of a count block holds the same value, the pixels accumulated per zone per channel. Dividing sums by it gives means spanning 4.2 to exactly 255.0, with the maximum exactly `255 * count`, which is what establishes the sums as 8-bit samples rather than the sensor's 10 bits. Channels 1 and 2 track each other across the whole grid while 0 and 3 sit either side, identifying 1 and 2 as the two greens; **which of 0 and 3 is red is not established.** In the reference capture the count is 826 for `rro_stats` and 90 for `rro_face_stats`, so the two engines meter different window sizes. The geometry that yields 826 is not derived here, so read the count from the buffer rather than computing it.

**The raw histogram is 128 bins by 4 lanes, and the lane assignment is exact.** The buffer is `0x1000` with only the first `0x800` written. Lanes 0, 1 and 2 sum to exactly one quarter, one half and one quarter of `1920 * 1080`, which is the Bayer population and identifies them as red, green and blue; lane 3 is always zero. All three together account for every pixel in the frame. Bin width is not pinned down: the capture cannot separate a 10-bit input binned by 8 from any other range leaving the populated bins where they are.

**Three registers are answered negatively, which is still an answer.** `af_stats` bank `0x7400` reads 0 after its last write, so autofocus is off and registers 1 and 2 address a buffer nothing writes; their captures are uninitialised DRAM, as expected for a fixed-focus FPV lens. Register 8 is not statistics at all: `ltm` and `gtm2` both publish their coefficient page to bank`+0x08`, and `ltm_stats` publishes its output to bank`+0x0c` (`str w2, [x21, #12]` with `x21` the bank base), so `0x2808` is the coefficient input and `0x280c` the statistics output, one of each. `awbs_stats` is named (bank `0x6c00`, enabled, geometry registers `36` by `64` at `+0x50`/`+0x54`, buffers `0x6c90` and `0x6d38`) but **its capture shows no statistics structure at any element width tested, so its format is not established**; it feeds AWB, not AE. `rgb_hist_stats` (`0x5c00`) and `rgb_max_stats` (`0x5400`) are never written at all.

**Two vendor modules register under a duplicate name.** `isp_sub_af_stats_creat` stores the name string `isp_sub_rro_stats`, the same pointer `isp_sub_rro_stats_creat` uses, and `isp_sub_derolling_stats_creat` stores `isp_sub_raw_hist_stats`. Bank attribution is unaffected because it comes from the attach handler, but any name-keyed lookup over this module set will collide.

`ar-isp-stats.h` holds the indexing and `scripts/isp/check-stats-layout.py` is the proof: it re-derives both layouts from the captures and fails if the structure, the 255 ceiling or the Bayer populations stop holding.

### LTM: the coefficient page is computed, not stored (recovered)

Static analysis plus the captured page; no hardware run. Bank `0x2800` carries a module control word at `+0x00`, frame dimensions at `+0x04`, the coefficient page at `+0x08` and the `ltm_stats` output at `+0x0c`.

**The page is 64 tiles of a 128-sample transfer curve.** Each curve is `u16` in a `0x100` slot, so the page is exactly `0x4000`. Every curve starts at zero and rises monotonically to just under 1024, a 10-bit output range, and all 64 are distinct. The extent is measured three ways that agree: the captured page holds 64 well-formed curves and turns to unrelated data at exactly `0x4000`, the publish site flushes `0x4000`, and the producer loop writes its tile count times `0x100` into the same buffer. **How the 64 tiles map onto the frame is not established**; an 8x8 grid is the obvious reading and nothing here proves it.

**It has no stored source.** The SIMD loops at `0x18a4f8` (gtm2 family) and `0x18dbb8` (ltm, byte-identical) are the init-time identity ramp generators: no data input, they write `out[tile][i] = i * 8`. The real per-frame computation is CLAHE, in the algorithm object `ltm` and `gtm2` share (`isp_sub_gtm2_algo_creat` at `0x18bd34`; the per-frame compute is vtable slot `+56` = `0x28a098`). Stages, layout, the double-buffered page with a moving descriptor, and the worker-thread-versus-inline mode flag (`get_start_opt()->[12220]`) are recovered; the final curve-to-page stage is the remaining hole. The vendor's captured page deviates from the identity ramp by at most 34 counts of 1016, which is what makes the driver's identity page a faithful static stand-in.

That makes LTM a 3A-class stage. Reproducing it is a computation over `ltm_stats`, not a table lookup, and it belongs with the exposure and white-balance work rather than with the coefficient generators.

**One producer double buffers it, rather than two modules sharing it.** `ltm` and `gtm2` both publish to `+0x08`, but the trace shows `0x2808` alternating strictly between `0x2b2f8c00` and `0x2b378c00` across all 4571 writes, with `0x280c` written 4573 times over the same span. A strict two-address alternation at one write per frame is double buffering; two independent modules publishing to one descriptor would not produce it.

**The enable bits check out.** The control word at bank`+0x00` settles on `0x00060f70` and is rewritten with that value 917 times. Bits 4 and 11 are both set in it, and the value it passes through on the way, `0x00060770`, has bit 4 but not bit 11. That confirms the earlier claim about bits 4 and 11 against the trace rather than against the module code alone.

**Buffer extents, from the vendor's allocation layout.** Each descriptor alternates between two addresses and the gap between them is the allocation: the LTM page and `ltm_stats` are both `0x80000`, `rro_stats` and `rro_face_stats` `0x8000`, `raw_hist_stats` `0x1000`, and `af_stats` `0x1200`. These are what the vendor allocates, not what it fills; the LTM page fills `0x4000` of its `0x80000` and the histogram fills `0x800` of its `0x1000`.

`ar-isp-ltm.h` holds the geometry, the extents and an identity-page filler; `scripts/isp/check-ltm-page.py` proves the geometry against the capture and reports the template divergence.

### ml-isploop flags

`--cvisp` rotates the CVISP ring once per frame start and does **not** drive the per-frame ISP cycle. That is the combination recorded above as sustaining. The cycle is opt-in behind `--isp-cycle`; briefly folding it into `--cvisp` silently changed the behaviour of every existing caller.

`cycle[]` follows the order the wide sweep shows for the target registers: statistics buffers, then the VIF clears, then the three indirect transactions on `0x0cc`/`0x0d4`. Those two are an indirect access port, not acknowledgements, so their pairs must stay adjacent and ordered; the real interrupt acknowledgement is the VIF `0x17c` write.

### Never touch VIF without a live pixel domain

Reading VIF with its clock gated hard-hangs the SoC into a watchdog reset to slot A. The frame-start liveness check cannot prevent this, because the check is itself a VIF read: it catches a stopped stream but not a gated clock. `ml-isploop` now reads the camera gate bit in CGU `0x0a104014` first, which is safe at any time, and refuses before touching VIF.

"The modules are still loaded" is not the same as "the pipeline is live". Scripts kill their grabber on exit, which puts the sensor into standby and gates the pixel domain, so a later standalone invocation hangs. Run captures only through a gated harness.

## The capture node advertises raw Bayer but has never delivered a frame

`/dev/video2` advertises `V4L2_PIX_FMT_SRGGB12` and is wired to the VIF bypass view: 1920x1080, `RG12`, 3840 bytes per line, 4147200 bytes per frame, so 16-bit padded samples. That is what the node claims. **No frame has ever come out of it, on any run.**

Read mid-stream, with a grabber holding the stream open, the view is armed and running and still signals nothing:

	0x17c  view-done W1C status    0x00000000    never signalled
	0x184  second W1C status       0x00000000
	0x1b0  block status            0x00000000
	0x020  armed view address      0x2c000000    a vb2 buffer is armed
	0x000  view control            0x0000c068    the configured value
	0x190  AXI config              0x40000101    RUN and ENABLE both set

The vendor does not use this path either. It configures view 0's geometry, then holds the view in reset (`VIF+0x2bc = 0x00000002`) and streams everything through the ISP path; afterwards it never writes a view address again. So there is no vendor sequence to copy, and bringing the view DMA up means working from the hardware specification.

Two things that look like the cause are not. The `crop_v 0xffff` / `crop_h 0xffff` / `frame_end 0x05000780` values in our init log are reset defaults printed before the write: `ar_vif_configure()` writes `+0x3a0 = 0x84380280`, `+0x3c0 = 0x00000438`, `+0x3c4 = 0x0000027f`, matching the vendor's live window exactly. And `+0x17c` bit 24, which the vendor acknowledges in its per-frame loop, is an ISP-path frame event rather than bypass-view completion, so a poll loop waiting on it is watching the wrong register.

**Consequence for judging images: there is no raw reference.** Whether the ISP is destroying information has to be answered from the live register diff against the vendor or from a same-scene vendor capture. Do not plan on a raw-against-ISP histogram comparison.

`ml-v4l2grab` writes its output file on the first and last frame of the requested count. A caller that passes a large `-n` and kills the process, which is what a streaming harness does, gets the first frame. Before that behaviour existed, such callers got no file at all.

## The sensor is unpowered outside a live stream

i2c reads of the sensor fail unless the pipeline is streaming, because the driver powers the sensor down when the stream stops. Any sensor register comparison has to run inside a bring-up, between the streaming gate and teardown.

## Liveness: a running grabber is not a delivering pipeline

Checking that `ml-v4l2grab` is still alive does not establish that frames are arriving. It blocks indefinitely waiting for a buffer that never completes, staying alive the whole time, which is its normal state on `/dev/video2`. Gate on a delivered artifact, not on the process.

Do not gate on the grabber's log either: its stdout is fully buffered when redirected, so the log stays empty until it exits and an empty log proves nothing.

The same distinction applies to VIF frame starts: `ml-isploop` counts frame starts, which continue on a failing bring-up, while no view DMA completes. Frame starts are not frames.

## Sensor registers that differ between stacks without meaning anything

A live-against-live comparison of all 3328 readable sensor registers, ours mid-stream on slot B against the vendor's mid-stream on slot A, leaves twelve differences. None of them is configuration. Four groups, all accounted for:

`0x0005` is a status byte that changes continuously between consecutive reads.

`0x0206`/`0x0207` is the committed analog gain code, and `0x0250`-`0x0253` is its shadow, exactly `gain << 4`. The difference was our old default of `0x2f` against the vendor's `0x3c`; the driver now defaults to `0x3c`.

`0x32bc`-`0x32bf` is written by no code path in the vendor library. It is an output of the sensor's own MCU, in the same class as the gain shadow, and exists on the vendor unit only because its AE loop has run.

`0x0138`-`0x013a` is the SMIA temperature sensor: `0x0138 = 0x01` enables it on both stacks and `0x013a` is `TEMP_SENSE_OUTPUT`. The value drifts upward as the unit warms, and it drifts *within* one stack, so it is not a stack difference at all. Five dumps, two stacks:

	ours   0x0139 = 0x23  0x013a = 0x74
	ours   0x0139 = 0x24  0x013a = 0x75
	vendor 0x0139 = 0x25  0x013a = 0x77
	vendor 0x0139 = 0x26  0x013a = 0x78
	vendor 0x0139 = 0x2a  0x013a = 0x7e

The two bytes track each other with a near-constant offset of about `0x52`, so they are one rising quantity read out in two places. The vendor's own dumps span `0x25` to `0x2a`, a wider spread than the gap to ours.

`0x0340` read `0x08` on the vendor during the first snapshot. That was contamination from an accidental i2c write, not a vendor value; a clean stack reads `0x04`, which is what we write.

The 271 registers our mode table never writes are power-on defaults: they read identically on both stacks.

## Sensor clamps are ours, not the chip's

Exposure is clamped to `vts - 2` and gain to `NT99235_AGAIN_MAX` (`0x60`) by the driver. Requesting exposure 2000 reads back as 1123 from the sensor, so the chip never sees the larger value and its own limit is unknown. Gain values above `0x60` all behave identically for the same reason. Both clamps are correct for a fixed frame rate, but they are decisions, not measurements.
