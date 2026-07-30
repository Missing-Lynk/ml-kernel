# Camera stack (NT99235 / CSI-2 / VIF / ISP / CVISP)

The air unit (BetaFPV VR04, Proxima-9311) captures from an onboard NT99235 image sensor over MIPI CSI-2. The vendor firmware drives that path entirely from userspace; reproducing the vendor's sequence in open drivers is the goal.

This document records two things and keeps them separate: **how the vendor drives the hardware**, recovered from a live MMIO write trace of the streaming stock unit, and **what the open stack currently implements and has validated on hardware**. Everything stated here is backed by a named artifact or a hardware observation. Anything not yet established is listed under "Not known".

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

**All four blocks are inside the SoC**, in its media subsystem. The NT99235 is a plain Bayer sensor: it holds a pixel array, its own PLL and exposure and analogue gain, and a MIPI CSI-2 transmitter, and nothing else. Every stage that turns its raw output into a picture runs on the Proxima-9311. Nothing in this document lives on the camera module.

The chain is: sensor sends CSI-2 packets over the MIPI lanes; the **CSI-2 receiver** decodes the link and produces a pixel stream on its IPI output; the **VIF** is the SoC's capture front end, which accepts that pixel stream and routes it, either straight to DDR through its bypass views or onward into the ISP; the **ISP** converts Bayer to YUV; and **CVISP** takes the ISP's output and writes frames to DDR.

**The ISP is not the writer.** That was assumed for most of this investigation and it is wrong. The correction is in "The output stage is CVISP" below, and it explains the standing contradiction: an ISP configured to match the vendor register for register, measurably receiving pixels, still produced no frames, because the stage that writes them was one we had never touched.

`VIF` is the vendor's own name for the block, used throughout `libmpp_service.so` (`vif_*`, 118 functions) and in the interrupt it registers (`ar_irq_reg_with_name(irq, handler, dev, "vif")`). It is the video interface between the receiver and the rest of the media block. It is not a MIPI or CSI-2 block itself: it sees pixels, not packets, and it also measures the incoming video timing, which is what makes `0x1f0` the front end's ground-truth register.

The vendor kernel does no camera hardware programming. `cam_hardware_power_on` calls `ar_pwr_ctrl_on(1)`, which on the 9311 dispatches through an uninstalled pointer and returns -1. All VIF, CSI, ISP and CGU register writes on the stock unit come from userspace (`libmpp_service.so`, called by `ar_lowdelay`) through `/dev/mem`.

## How the vendor drives the pipeline

Most of this section was written before CVISP was found, from traces windowed on one block at a time. It is accurate about the blocks it describes, but where it treats the ISP as the stage that writes frames to memory it is describing the wrong stage. Read "The output stage is CVISP" below first; it says what the ISP's per-frame loop and plane registers are not.

### The trace

`out/au-mmiotrace/mmio-combined.log`: 48183 register writes from one boot of the stock firmware, captured with `native/mmiotrace.c` (an `LD_PRELOAD` shim under `ar_lowdelay`, installed by `glue/camera/au-slotA-mmiotrace.sh`) over the window `0x08860000`-`0x08c0ffff`. Lines are `wNNNNNN wWW pa=0xADDR val=0xVAL` in program order.

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

- **The eight address registers ping-pong between exactly two value sets.** Each register takes one of two addresses and alternates every frame. This is a two-buffer rotation, not a descriptor ring walked over many slots.
- **All eight addresses fall in `0x2a65f200`-`0x2b378c00`**, inside a single reserved region. The `isp_cma` reservation in the air DTS (`0x2a000000`, 32 MiB) covers this range so the vendor's addresses are safe to write on the open stack.
- **The addresses are per-boot allocations.** An earlier separate trace (`out/au-mmiotrace/isp-arm-sequence.txt`) shows the same structure at different addresses. Only the layout is stable, not the values.
- **`0x08c000cc` / `0x08c000d4` behave as an indirect access port**, written as a pair. Three transactions run per frame, with the same values every frame (`0x04001550`/`0x003a2000`, then `0x0`/`0x10000200`, then `0x0`/`0x00000100`).
- **VIF `0x0887017c` = `0x01000000` is the path0 frame-start interrupt acknowledge** (`0x17c` bit24). It is the only VIF interrupt the vendor acknowledges per frame. `0x184`, `0x194` and `0x294` are cleared alongside it.
- **The ISP master register is not touched in the loop.** It is left at `0xb0280052`.

So per frame the vendor: writes eight buffer addresses, acknowledges the VIF frame-start interrupt, and runs three indirect-port transactions. Nothing else.

### The VIF views are not the capture path

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

`0x330`, `0x334`, `0x338` and `0x33c` are **read-only debug counters behind a mux**, not configuration. A vendor dump routine at `0x22ee80` (a local function, between the exported `vif_getmipitype_bits` and `vif_device_module_creat`) drives them: it read-modify-writes a channel index into `0x32c` bits [19:16], calls `ar_delay(100)`, then reads each counter and logs it as **two independent 16-bit fields**, high half and low half, through `ar_log_func_raw`. It loops, incrementing the channel index.

Two consequences for anyone diffing a live VIF window:

- **The values are only comparable when `0x32c` matches on both units**, because the mux decides what the counters are counting. On the captures in `out/au-chain/` it reads `0x00000fff` on both.
- **A difference here is not a fault.** These are free-running measurements. Measured on a streaming vendor unit against the open stack with the same mux setting: `0x330` low half is 6640 on both, `0x33c` high half is 9869 on both, and `0x338` agrees within 0.2% in both halves. That level of agreement across independent units is positive evidence the front end is seeing the same video, and it is the correct way to read these registers.

The same caution applies here as to the CSI wrapper's `0x030`: a single sample of a free-running counter carries no bit-level meaning, and reading one as though it did has produced retracted conclusions in this project before.

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

The `_v1` and `_v2` suffixes are alternate implementations of the same stage, not extra stages. Many submodules are never enabled in this configuration: their static defaults exist in the library but the vendor never pushes them, and their registers read back zero on hardware.

**The output pixel format is not established.** The buffer-spacing argument below is now better explained: these ISP addresses are not the frame output. CVISP's are. The paragraph stands as a record of what the ISP's own address registers look like. The library's format vocabulary distinguishes `STREAM_FORMAT_YUV420_8BIT_Plannar` (three planes) from `STREAM_FORMAT_YUV420_8BIT_semiPlannar` (two planes), and offers 420, 422 and 444 at 8, 10 and 12 bits. Three plane-address registers per buffer set point at planar rather than semi-planar. Against that, the observed buffer spacings do not fit a 1920x1080 planar layout: `0x2b439200` to `0x2b614200` is 1945600 bytes while a 1920x1080 luma plane is 2073600, so the gap is smaller than the luma plane it would have to hold. Either those three addresses are not the Y, U and V of one surface, or the surface at that stage is not full 1080p. Page `0x70` holds both `0x04380780` (1920x1080) and `0x021c03c0` (960x540), which is suggestive but untied. Settle this before describing a V4L2 format: a wrong guess makes a correct capture look like a broken one.

### The output stage is CVISP

The block at `0x08e00000` is what writes frames to DRAM. The ISP feeds it. This was found late and it invalidates the framing of everything above it: the ISP was audited for months as though it were the writer.

**How it was missed.** Every trace before the wide sweep narrowed `MMIOTRACE_LO`/`HI` to one block, chosen from the vendor device tree. The vendor maps all 256 MiB of register space in a single `/dev/mem` call and drives every block through it, so writes outside the chosen window were never recorded. CVISP is absent from the vendor device tree, so it was never a candidate window. The earlier claims "register state matches the vendor everywhere" and "coverage is 1267 of 1267" were true, but only of the ISP block, inside windows we picked ourselves.

**The sweep.** `out/au-mmiotrace/wide-sweep.log`, window `0x08000000`-`0x0a1fffff` skipping `0x08820000`-`0x0885ffff` (encoder and DSI, excluded because a previous wide attempt corrupted them), vendor streaming throughout, 330630 writes.

| Block | Writes | Identity |
|---|---|---|
| `0x0a080000` | 182058 | `h26x` encoder (vendor DTS) |
| `0x08c00000` | 60622 | ISP |
| `0x08800000` | 60228 | `axi_dma` (vendor DTS) |
| `0x08870000` | 15129 | VIF |
| `0x08e00000` | 11782 | **CVISP** |
| `0x08880000` | 738 | CSI-2 |
| `0x0a100000` | 66 | CGU |

The sweep is writes-only, so it says nothing about what the vendor reads from any of these.

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

**The ring** is five buffer sets, not the two an earlier reading suggested:

```
Y  0x28014000 -> 0x2834c000 -> 0x28684000 -> 0x289bc000 -> 0x28cf4000 -> repeat
U  0x28232000    0x2856a000    0x288a2000    0x28bda000    0x28f12000
V  0x282bb000    0x285f3000    0x2892b000    0x28c63000    0x28f9b000
```

Round robin, all three planes in lockstep, no deviation across 496 wraps. The range `0x28014000`-`0x2902c000` is reserved as `cvisp_cma` in the device tree; `vif_cma` moved to `0x2c000000` because the ring's last slot runs past where it used to start.

**Geometry is not settled.** `0x8028` carries `0x04380780` (1080 x 1920) and `0x8008` is written `0x021c03c0` (540 x 960) during setup but ends at `0x04380780`, while page `0x4000` carries 1920 x 1080 throughout. `0x021c03c0` is also what ISP `0x7080` reads on the streaming vendor. Which stage, if any, is scaled is open.

**Clock.** `cgu_rsz_clk`, `0x0a104014` low half, gate bit 12, device tree index 9. This is from the vendor clock table, not the trace: the trace contains **no CGU write for this block at all**, and `0x0a104014` does not appear in the live vendor CGU snapshot either. So the vendor streams with whatever state the boot leaves that leaf in, and enabling it is an assumption. It is the load-bearing one in `ar-cvisp.c`, which is why that driver reads no register at probe: if the leaf is wrong, the first access hangs the SoC, and a probe-time read would make that unavoidable on every boot.

**No reset and no interrupt are declared.** No CVISP reset write appears anywhere in the trace and no reset leaf has been identified. The block does have its own completion path (`cvisp_device_irq_process` at `0x2424b8` dispatches through `cvisp_dispatch_irq` at `0x242390`, routing status bits 1 and 5 to output events `0x1001`/`0x1002` and bit 3 to `0x2c02`), which is good evidence that frame completion is serviced from CVISP rather than from VIF SPI 62 alone. But the hardware IRQ number and its acknowledge register are still behind the vendor's generic camera-module event layer, so asserting either in the device tree would be inventing it.

Extractor: `scripts/gen-cvisp-defaults.py` -> `overlay/drivers/media/artosyn/ar-cvisp-defaults.h`. The setup table self-checks: replaying it in order reproduces the vendor's final value for every register it touches.

### CVISP first light, validated on hardware

Slot B RAM-boot, 2026-07-29. CVISP writes YUV planes to DRAM.

`glue/camera/au-cvisp-firstlight.sh`, CVISP configured against a control run with it left alone, same addresses and same everything else:

| Run | Plane `0x28014000` | `0x28232000` | `0x282bb000` |
|---|---|---|---|
| configured | 24/24 overwritten, `0x01010101` | 24/24, `0x7f7f7f7f` | 24/24, `0x7f7f7f7f` |
| control | 0/24 | 0/24 | 0/24 |

Frame starts ran at 60.0/s in both, so the front end was equally live in the negative case. Luma `0x01` with both chroma planes at neutral `0x7f` is a black frame, not a fill pattern: a memset would not produce that pairing. Sensor exposure and gain are still unset on the open stack, so a black frame is the expected picture.

**The clock is inherited, not asserted.** There is no clock request anywhere in the CVISP path in `libmpp_service.so`, and `0x0a104014` reads `0x02011102` on slot B with gate bit 12 already set, so the boot firmware leaves it on and the vendor inherits it. `ar-cvisp` does the same; asserting it is behind an off-by-default `assert_clk` param. Taking ownership was harmful, not merely unnecessary: the leaf is gate-modelled, so `clk_disable_unprepare` on `rmmod` cleared a gate the boot had set. Stock-A baselines read `0x12011100` at the same register, differing in both halves but agreeing on bit 12.

**The write is triggered by the arm, not by the configuration.** The control run above did not reset the block, and its opening `regs` dump still showed `control=0x00800806` with set 0 armed and geometry intact. So CVISP sat fully configured and enabled through ten seconds of 60 fps input and wrote nothing.

**It sustains.** `glue/camera/au-cvisp-framelock.sh` arms one ring slot per VIF frame start, the vendor's cadence: 480 slots armed over 96 wraps in 8 seconds at 60.0/s, and ring slots 1, 2 and 3 were all overwritten.

An earlier burst-driven test (`au-cvisp-rotate.sh`, `echo 5 > queue` in a loop) saw only slot 1 written and looked like a stall. It was not: five rotations complete in microseconds, so slots 2 to 4 were armed and superseded long before a 60 Hz frame could land in them. The apparent stall was the test.

**The frame is uniformly black.** The active region of a dumped luma plane is 100% `0x01` across 1,941,016 bytes, with chroma at neutral `0x7f`. CVISP writes; there is no picture. Sensor exposure and gain are unset on the open stack, so this is the expected next problem rather than a CVISP failure. A perfectly uniform non-zero luma is consistent with a black-level clamp, that is, the ISP processing correctly with nothing above black arriving.

**Line stride is 2048 for a 1920-wide plane**, measured: the gap between bright runs in a dumped plane is exactly 2048 and every sub-gap pair sums to it. A full plane is therefore 2048 x 1080, not 1920 x 1080.

**Trap, hit once already.** The 128 bytes of per-line padding hold high-entropy stale DDR content that CVISP does not write. Histogramming a whole dumped plane counts it and reports a confidently "structured" frame that is in fact flat black, and dumping `width * height` rather than `stride * height` truncates at 1012 of 1080 rows and shears every row, which renders as diagonal scanlines that look like a picture. The markers cannot catch either, because at 4096-byte spacing they only sample column 0 of every other row and never touch the padding. Analyse the active columns only.

The wide sweep captured every write `ar_lowdelay` made to the block, and there are only four things in it: setup, late, the per-frame plane triplet and the per-wrap tick group. We do all four. So whatever sustains the vendor is not an MMIO write to CVISP.

A read-to-clear acknowledge was the obvious candidate, and it is **not supported**. `cvisp_device_irq_process` (`0x2424b8`) takes its status from an event structure, not from hardware:

```
status    = *(u32 *)(event + 0x08);
module_id = *(u32 *)(event + 0x1c);
if (status)
        cvisp_dispatch_irq(device->private, module_id, status);
```

`cvisp_dispatch_irq` (`0x242390`) reads only heap-resident device and module objects and calls their `set_ctl` vtables. It never maps `0x08e00000` and never dereferences a CVISP register base. The same holds for the `cvisp_outlib_set_ctl` event paths. The status arrives from the generic camera event producer. **Do not poke candidate status registers in the `0x08e` map on the strength of this**: an invisible read below the userspace layer is still possible, but nothing here gives a safe offset to test, and a blind read of the wrong register hangs the SoC.

The stronger candidate is the vendor's generic IRQ service protocol: the kernel handler disables the IRQ and queues an event, userspace handles it and then explicitly **re-enables** that IRQ. A missed re-enable wedges further completions without any CVISP register operation at all. That fits a block which produces a frame or two and then stops, and it means the thing to recover is the event producer and the IRQ registration, not a CVISP offset.

There is a confirmed buffer-done path, and it is software only. For status bit 1 the dispatcher emits event `0x1001` to the selected output module; `cvisp_outlib_set_ctl` handles it at `0x240620`, rotates the software buffer entries under its lock, and invokes the registered upper callback (`0x2406d8`-`0x2406ec`). It issues no CVISP address-register write. So a slot return is real but lives above the register interface.

Resolved as `hdf-20260729-002`.

**The completion path, recovered (`hdf-20260729-003`).** The event is built in CVISP's registered raw IRQ handler, not in `cvisp_device_irq_process`. `cvisp_device_set_ctl` takes two `(hwirq, base)` pairs from runtime hardware info (`g_hw_info + 0x50`/`+0x54` for CVISP0, `+0x58`/`+0x5c` for CVISP1), maps each base, and registers both through `ar_irq_reg_with_name`. The handler at `0x241ed0` reads a status word at `base + 0x34` and writes the identical value back, which is a W1C acknowledge, then places that status at `event + 0x08` and the IRQ identity at `event + 0x1c` before invoking the device callback. Full path:

```
/dev/ar_mpp_ctl kernel IRQ queue -> ar_irq worker -> CVISP raw handler (0x241ed0, W1C at base+0x34)
    -> CVISP event callback -> cvisp_device_irq_process
```

The generic worker waits on ioctl `0x80104d04`, registration is `0x40184d00`, and the re-enable is `ar_irq_enable`, ioctl `0x40044d03` with `hwirq - 32`. So each completion is acknowledged twice: a register W1C in the raw handler, and a GIC unmask through the generic service. **There are two CVISP registrations**, so completion is not simply the ISP's SPI 71. The numbers live in `g_hw_info` at runtime and are not in static code, so they are not recoverable without logging the `ar_irq_reg_with_name` arguments on a stock run. No IRQ is asserted in our device tree.

**Do not W1C `0x08e00034` on the strength of that.** Taking `base` to be `0x08e00000` conflicts with the trace: that address is written exactly **once**, with `0x0000801a`, and its neighbours `0x30` and `0x38` hold `0x8013` and `0x801b`. It is an entry in the ascending arbitration table on page `0x0000`, not a per-frame status. A real per-frame W1C would appear about 2477 times, like the plane registers do. Either the IRQ status base is not the CVISP register base, or the acknowledge happens through a mapping the shim never trapped. Writing that offset as a W1C would corrupt a configuration entry.

None of this is on the critical path: frame-locked arming sustains at 60 fps with no acknowledge of any kind. It matters for a completion-driven driver, not for capture.

### Exposure, gain, and the tone response

Exposure and gain are implemented and validated. Both are SMIA addresses, recovered from the vendor's AE commit list in `libsns_nt99235.so`: integration time is a 16-bit big-endian line count at `0x0202`/`0x0203` clamped to the mode's frame length minus two, analogue gain is a code written to both `0x0206` and `0x0207`, and the pair must be bracketed by `0x0104` = 1 then `0x0104` = 0 so no frame sees a half-updated commit. The 26 registers at `0x8250`-`0x826c` and `0x8550`-`0x855c` that drift at runtime on the vendor are **not** exposure: they are sensor-MCU lens shading tables driven by the AWB path and reloaded through `0x8201`.

Gain is an index into a 97-entry table, and the mapping is closed form. The table was extracted from `libsns_nt99235.so` at virtual address `0x1a130`, file offset `0xa130` through the second `PT_LOAD`, 97 `u32` entries with 1024 meaning 1x, and

	gain = 2^(code >> 4) * (16 + (code & 0xf)) / 16

reproduces **all 97 entries exactly**: a four-bit mantissa with a four-bit binary exponent, 1x at code `0x00` to 64x at `0x60`. It is not an approximation of the table.

**The sensor is not SMIA-compliant, only SMIA-addressed.** Reading the SMIA identification and capability blocks on hardware gives 3 of 20 registers non-zero: `model_id` `0x9235` and `revision_number` `0x0b` are real, but `manufacturer_id` is 0, `smia_version` reads `0xff`, and the whole analogue-gain capability block `0x0080`-`0x0092` and integration-time block `0x1000`-`0x1006` are zero. So the functional registers follow SMIA addresses and SMIA semantics, confirmed by behaviour rather than by a compliance claim, and nothing is discoverable from the sensor: every limit has to come from the vendor library or from measurement. `nt99235.ko dump_smia=1` reproduces this.

**The tone response is bimodal, and that is the remaining defect.** Swept live on hardware at gain `0x2f`:

| exposure (lines) | mean luma | at luma 0-1 | over 200 |
|---|---|---|---|
| 64 | 1.1 | 99.9% | 0.0% |
| 256 | 1.1 | 99.9% | 0.0% |
| 512 | 4.0 | 92.1% | 0.0% |
| 1123 | 25.7 | 85.9% | 9.4% |

The response **accelerates**: four times the integration from 64 to 256 changes nothing, while 2.2 times from 512 to 1123 multiplies the mean by 6.4. A linear sensor through a normal display gamma compresses at the top and does the opposite. The full distribution at 1123 is hollow in the middle: 85.91% at luma 0-1, then 1.06%, 1.32%, 1.24% and 1.10% across the bands to 199, then 9.35% at 200-239 and essentially nothing above. That is a threshold, not a curve, and it does not clip at white.

Two candidates, not yet separated: a black-level pedestal far too large, clamping everything below it, or the missing gamma LUT. Handed to Codex as `hdf-20260729-007`.

**What is missing is a 16 KiB gamma LUT upload, not a register.** `isp_sub_gamma_creat` allocates a `0x42e8` object whose handler at `libmpp_service.so:0x194350` calls `isp_memcpy` with length `0x4000` into an ISP-resident aperture, then flushes and programs the gamma control words. Our own trace corroborates that this never reached the register replay: across 60,622 ISP writes over 1,276 distinct offsets, the highest offset written anywhere is `0x76d8` and there are **zero** writes at or above `0x8000`, with no tracer bail-out. A 4096-word upload would be unmistakable.

The colour-space conversion is **not** the problem and should not be touched first: `0x08c03c00`/`0x08c03c04` hold the Rec.601 Y row `306, 601, 116` in Q10, and a linear matrix with that row cannot map five colours to the same near-black while leaving yellow bright.

The vendor ships three tuning blobs, `nt99235`, `sc2210` and `sc231`, all **exactly 879,704 bytes**, so the format is a fixed-layout struct. Across all three, 97,880 bytes differ over 46,738 regions with a largest contiguous differing region of only 1200 bytes, so there is no 16 KiB block that varies by sensor and the gamma data is likely shared. Differential analysis therefore cannot locate it.

### DRC upload and strength control (recovered)

Dynamic-range control (DRC) is another ISP DMA payload, not a single register.
For this vendor `libmpp_service.so`, its immutable 8 KiB initial template is
embedded at service-image VMA `0x467460` (file offset `0x457460`). The
runtime configuration table at VMA `0x472760` supplies the exact pair
`(source=0x467460, length=0x2000)`. The module's initial `0xb09` handler
copies those bytes to its DMA allocation, flushes all `0x2000` bytes, stores
the physical address in its descriptor and sets the descriptor apply bit
(`0x1a5828..0x1a5940`). This is a firmware-build-specific constant; do not
assume it is the same in another vendor build.

During normal operation the service overwrites only the first `0x1000` bytes
of that page (`0x1a4200`). They are two `0x800` banks, each containing 128
16-byte records. A record packs three unsigned 20-bit values in its first ten
bytes; the second value is duplicated, and adjacent records overlap. Thus one
bank represents a 257-sample curve, not 384 independent values. The active
DRC tuning profile starts at raw tuning offset `0x17b1c + index*0xc8c`; its
first two 257-word curves are packed into the two banks. At neutral strength
50, a retained vendor DRC page decodes byte-for-byte to profile 3 of the
NT99235 FPV blob.

Strength is implemented in ARM software before packing, around neutral 50.
For a requested strength `s`, the service calculates
`q = floor(abs(s - 50) * 4096 / 50)` and blends every curve word in Q12 with
one of two fixed 514-word service-image curves: the high curve at VMA
`0x35f080` when `s > 50`, or the low curve at `0x35fc90` when `s < 50`.
It then performs the 20-bit packing; the ISP receives only the resulting DMA
page. The final `0x1000` bytes remain the initial template. This establishes
the vendor-equivalent DRC payload construction without a DMA capture, while
the semantic meaning of the four page banks still needs hardware validation.

### Static table array and GTM2/LTM activation (recovered)

The service's ISP-init configuration is a contiguous array of `{u64 source,
u64 length}` descriptors at VMA `0x472600..0x472a40` (56 non-null entries).
It is the source of the initial payloads; it is not a second tuning blob. The
module setup handlers now give several entries unambiguous names:

| Config offset | Source / length | Consumer |
| --- | --- | --- |
| `+0x0b0`, `+0x0c0` | `0x46a0f0` / `0x40`, `0x469db0` / `0x340` | LSC control and DMA payload |
| `+0x150`, `+0x160` | `0x469460` / `0x64`, `0x467460` / `0x2000` | DRC control and page |
| `+0x240` | `0x463050` / `0x4000` | gamma initial page |
| `+0x270` | `0x451570` / `0x3c` | shared GTM2/LTM control template |
| `+0x290`, `+0x2a0..+0x2d0` | `0x45afe0` / `0x6c`, then four `0x2680` sources | LUT3D control and four DMA banks |

The LSC and LUT3D handlers copy precisely those descriptor fields into their
DMA allocations. This resolves their payload provenance. The unlabelled
entries must remain unlabelled until a setup handler accesses them; size and
pipeline position alone are not proof of a module mapping.

The vendor modules `gtm2` and `ltm` named here share register bank `0x2800` and
publish to it at `+0x08`. They are **not** the descriptors this document calls
GTM2 (`0x1c6c`) and LSC (`0x4c34`); see the naming note under the ownership
table below. Their own descriptors are still unowned.

GTM2 and LTM are **enabled** in the NT99235 FPV preview configuration. Their
separate setup handlers (`0x18ab38` for GTM2 and `0x18e2c4` for LTM) read the
same raw control word at `raw + 0x7abd8`; it is `1` in the vendor blob. The
enabled paths each set bits 4 (`0x10`) and 11 (`0x800`) of their module control
word, prepare a `0x4000` DMA page from the selected `0x9c` profile and flush
it. Their disabled paths clear exactly those two bits. This is enough to rule
out "the blocks are off" as a reason to skip their packers. The physical ISP
register represented by that module-control word is still unmapped, so these
are proven service-side enable bits rather than a claim about a named MMIO
bit.

### 3A execution (proven)

`raw_stats_filter_port` creates the AEC, AWB and AF modules in sequence
(`creat_aec_algo_module` at `0x15db40`, `creat_awb_algo_module` at
`0x15dbb0`, and `creat_af_algo_module` at `0x15dc20`).  The AEC module
selects an in-process algorithm through `get_aec_algo_lib`; the vendor's
`artosyn_ae_algo_creat`, AWB and AF implementations are all in
`libmpp_service.so`.

The only DSP remote-call sites are `AR_DSP_AiISP_PreProcess`/`PostProcess`, an
optional CNN/NPU enhancement path: it allocates NPU buffers and starts
NPU/DSP pre/post threads, not 3A modules.  The FPV rootfs contains no DSP
program image, so that path cannot run there.  The ARM MMIO trace therefore
has no hidden DSP ISP-programming participant.

### Not known

These are open and are not to be assumed while building the ISP driver:

- **Why CVISP stops after the first armed buffers.** See above; the frame-locked run is the next step and may resolve it without further analysis.
- **Whether the late table has to follow the enable.** The vendor issues it with frames already in flight; whether that is required or is just what its threading produced is untested. `configure` takes 2 to apply setup alone.
- **What the tick group does.** An acknowledge, a queue re-arm and a five-credit refill are all consistent with a once-per-wrap cadence.
- **The output format.** `0x01`/`0x7f`/`0x7f` is consistent with 8-bit planar YUV but does not prove the plane geometry. `dd` on `/dev/mem` fails with `EFAULT` for reads as well as writes on this device, so dumping a plane needs `mmap`; `ml-isploop --dump` does that and is unrun.
- **`0x4100` and `0x4108`**, written `0x00000000` nine times aperiodically, paired with `0x4104`/`0x410c` which hold 1920 and 1080. A crop origin is the obvious reading; it is not confirmed, and they are not tabled.

- **How the vendor detects frame completion.** The trace has no reads. Whether the loop is driven by the VIF frame-start IRQ, an ISP IRQ (SPI 71), or a poll is undetermined.
- **What each of the eight address registers points at.** Plane, statistics buffer, or intermediate surface is unassigned. The two-value alternation is established; the meaning of each register is not.
- **The pixel format in those buffers.** Whether the ISP output is raw Bayer or processed YUV has not been read back and decoded.
- **What the three indirect-port transactions do.**
- **The function of the ~1267 ISP registers.** They are captured verbatim; they are not decoded.

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

Those 26 drift at runtime on the vendor and are expected to differ. Measured on slot A: 156 of 183 registers still hold exactly the value the vendor library programmed, and every one of the 27 that moved falls in those two ranges. The vendor's 3A layer writes them through the callbacks the sensor library registers (`AR_MPI_AE_SensorRegCallBack`, `AR_MPI_AWB_SensorRegCallBack`). The open driver programs the same initial values and never updates them, so a difference there is the vendor adapting, not a fault.

**Not covered: exposure and gain.** Neither the vendor mode table nor the open driver sets them. The vendor drives them from the 3A layer at runtime, so after mode init the open stack leaves the sensor at its power-on defaults. A correct capture may therefore be very dark or black. **Judge a capture by whether the DMA wrote, not by image brightness**: pre-fill the target buffer with a marker and check whether the marker was overwritten. A buffer full of zeros is indistinguishable from a DMA that never ran.

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

An earlier revision of this file gave `0x0a10401c` as `0x1102`. The measured value is `0x02011201`.

`0x0a102004` and `0x0a102014` are reset control and are written twice each, in an assert-configure-release order: first with a bit clear (`f3ffffdf`, `e7ffdffb`), then the `0x0a106xxx` block is configured, then with that bit set (`f7ffffdf`, `e7ffdfff`). A block held in reset accepts register writes and reads them back correctly while its datapath does nothing.

`0x0a104018` is the register the vendor polls, 17,434 reads in a 34-second trace, far more than any other register in any window.

## Status

Validated on hardware, sensor through VIF front end:

- **Clocks.** The CGU prologue reads back the vendor's values and the downstream blocks come alive.
- **Sensor.** `nt99235` powers on and streams, and its configuration is **verified equivalent to the vendor's**: the mode table matches the vendor library write for write in order, and 184 registers read back live from both a streaming vendor unit and the open stack are identical outside the 26 the vendor's 3A adapts at runtime. See "Sensor configuration is verified equivalent to the vendor". Exposure and gain are the exception: nothing sets them on the open stack, so the sensor runs at power-on defaults.
- **CSI-2 link.** Every `INT_ST_*` bank reads zero on the steady-state (second) read, matching a streaming vendor unit read the same way. The banks are clear-on-read, so a single read is meaningless: it returns the start transient too. Omitting the HS-settle write is a hard negative control and puts errors in four banks.
- **VIF front end (path0).** Measures the incoming video timing correctly: `0x1f0` reads `0x0784043c` (1924 x 1084), and `0x1f4` and `0x1f8`/`0x1fc` match the streaming vendor's live values. `path0_frame_start` (`0x17c` bit24) fires at frame rate, 720 events per 1510 polls over 12 seconds.
- **VIF-to-ISP hop.** The ISP-path status registers `0x10c` and `0x110` are live and advance with the vendor's value structure, so data crosses out of the front end into the ISP path.

No frames are produced by either path, and ISP register state is not the reason: the stage that writes frames is CVISP, and until now it had never been programmed. `ar-cvisp.c` exists and builds; nothing about it has been run on hardware.

Read+write traces of a streaming vendor unit now cover every programmable block. For each, the vendor's end state was reconstructed as last-write-per-register, validated against a live register snapshot of the same streaming unit, and compared with the open stack only where model and snapshot agree:

| Block | Model validation | Differences vs open stack |
|---|---|---|
| Sensor | n/a, compared register by register | 0 |
| CSI-2 | 34/34 | 0 |
| ISP | 1081/1202 | 0 |
| VIF | 60/64 | 4, all set to vendor values on hardware with no effect |

The ISP's one apparent difference, `0x00cc`, is a per-frame interrupt status register the vendor never writes. The VIF's four are `0x080`, `0x08c`, `0x140` and `0x2bc`.

So the chain reaches vendor-identical register state and delivers nothing, in both the ISP path and the VIF bypass view.

**That table is scoped to blocks we chose to look at, and the choice was wrong.** All four rows were measured inside tracer windows aimed at blocks in the vendor device tree. CVISP is not in it, was never in a window, and is the stage that writes frames. "Vendor-identical register state" therefore means the ISP, CSI-2, VIF and sensor are vendor-identical while a fifth block sits entirely unprogrammed. The wide sweep is the fix, and the CVISP section above is what it found.

A CGU divergence also remains: the open stack programs three of the fourteen registers the vendor programs, and two of the eleven it omits are reset control. That was the leading hypothesis before the sweep. It is still unexplained, but it is no longer needed to explain the absence of frames.

Two further facts from the traces bear on how to read them:

- The vendor never spin-waits. There is not one consecutive-read run on any block, so a missing poll or wait state is not the mechanism.
- The vendor brings the pipeline up, tears it down, and brings it up again. Both the VIF and CSI traces show this. Reading only the head of a trace attributes first-bringup behaviour to the working path.

The VIF bypass view DMA the open driver arms is not the vendor's path: the vendor streams with view 0 held in reset. It is front-end scaffolding, not the capture mechanism.

Investigation history and next steps live in `plans/air-camera-first-light.md`; this file tracks mechanism only.

## Source map

| Concern | Files |
|---|---|
| Sensor init, MCLK, stream-on | `overlay/drivers/media/artosyn/nt99235.c` |
| CSI-2 D-PHY, lanes, IPI, deskew | `overlay/drivers/media/artosyn/ar-csi2.c` |
| VIF front end, view arm, DMA | `overlay/drivers/media/artosyn/ar-vif.c` |
| ISP configuration | `overlay/drivers/media/artosyn/ar-isp.c`, tables `ar-isp-defaults.h` from `scripts/gen-isp-defaults.py` |
| CVISP output stage and queue | `overlay/drivers/media/artosyn/ar-cvisp.c`, tables `ar-cvisp-defaults.h` from `scripts/gen-cvisp-defaults.py` |
| DT nodes (camera, CSI, VIF, ISP, CVISP, clocks, carveouts) | `devices/betafpv-vr04-air/proxima-9311-air.dts` |
| Vendor MMIO write trace, per block | `out/au-mmiotrace/mmio-combined.log`, capture harness `glue/camera/au-slotA-mmiotrace.sh`, shim `native/mmiotrace.c` |
| Vendor MMIO write trace, all blocks | `out/au-mmiotrace/wide-sweep.log` (this is the one that found CVISP) |
| CVISP first-light experiment | `glue/camera/au-cvisp-firstlight.sh` |
| Vendor RE cross-reference | `archive/re/notes/nt99235/` (see the caution below) |

Caution on the RE notes: they were checked against the trace and are unreliable in detail. They record the vendor's `0x080` as `0x76543210` (the trace shows `0xffffffff` and `0xfffffff8`), state that the open driver never writes `0x32c` when it does, and give `0x0d0` as both `0x2c` and `0xaaaaaaaa` in different files. Treat them as leads to verify. The trace is the authority for what the vendor writes; the disassembly is the authority for what a register means.

### Judging a capture: the marker count, never the image

`ml-isploop` writes 24 marker words (`0xa5a5a5a5`, every `0x1000` bytes) into each output plane before its capture window opens, and reports how many survived. **A capture is only ours if those markers were overwritten.** `0/24` means nothing in our stack wrote that memory and the dump is whatever was already in DRAM.

This is not a theoretical caution. DDR survives a RAM-boot, so the vendor's last frame from slot A persists at the same addresses our ring uses. One such frame was read as our own output and treated as proof the pipeline was healthy, which then required inventing a regression to explain why every later frame looked worse. Three separate theories were built and refuted against a phenomenon that never existed. The marker count was in every log throughout.

The vendor's residual frame is now useful as a **control**, being the same sensor, lens and scene through a correctly configured ISP:

| | dark 0-31 | mid 32-223 | blown 224+ | mean |
|---|---|---|---|---|
| ours | 71.2% | 10.0% | 18.8% | 61.5 |
| vendor residual | 0.0% | 97.6% | 2.4% | 95.7 |

### CVISP writes only on the first bring-up after a RAM-boot

The first camera bring-up of a boot writes frames to DRAM. Every later bring-up in the same boot arms normally, reports full frame-start and slot-arm counts, and writes nothing, leaving the first bring-up's frame in place.

Measured: first bring-up 24/24 markers overwritten on all three planes; second and third `0/24`, each with 241 frame starts and 241 slots armed in 4 s and CVISP control `0x00800806` correct throughout.

This is not simply cold-versus-warm. Slot B is RAM-booted from slot A with the vendor pipeline already streaming, so the first bring-up **inherits hardware state the vendor established**, and the register replay does not recreate it once our own teardown has destroyed it. A boot therefore yields exactly one trustworthy capture; `RUNS=` in `glue/camera/au-prove-camera.sh` selects which one to spend it on.

### Within a working bring-up, capture sustains

All five ring slots hold distinct frames after a one-second window, differing by 10 to 16 per cent of sampled pixels. The pipeline streams at 60 fps; it does not produce a single frame and stop.

### The tone tables were never the crush, and they were never garbage either

The crush was two output-stage registers, `0x2e2c` and `0x2e30`, whose corrected values sit past the prefix the replay applies. Writing them recovers the whole shadow range: 57% of pixels under luma 32 becomes 0.0%. See `ar_isp_output_fix` in `ar-isp-defaults.h`. That is the whole explanation, and it was reproducible across all five ring slots, at 1 s and 4 s capture windows, and on boots with and without a preceding vendor stream, so it was never scene-, time- or slot-dependent.

**Correction, 2026-07-30.** An earlier version of this section stated the DMA-fetched tuning tables were unfilled and that the ISP was fetching undefined data with the enable bits set, on the strength of a residency check that scored each page by its zero fraction and monotonicity:

	gamma      0x2b2ec600  0x4000   zero  6.2%  non-decreasing 51.4%
	compander  0x2b2e0c00  0x7800   zero 50.6%  non-decreasing 75.1%
	drc        0x2b2e9200  0x2000   zero 33.2%  non-decreasing 47.7%

Those are not garbage. Every row is the vendor's own correct table:

- The compander figures are **byte-identical** to the prepacked `0x7800` constant at service VMA `0x46a3b0`, and reproduce it to the digit, `50.6%` and `75.1%`.
- The DRC page decodes exactly as tuning-file profile 3 through the packing in `ar-isp-codec.h`, across all 514 samples of its dynamic half.
- The gamma page decodes to tuning-file curve 2, within 6 counts of 4095.

The heuristic was wrong, not the pages. These are packed multi-lane formats: a correct table is neither mostly zero nor monotonic when read as a flat `u32` array, so both scores measured the packing rather than the content.

**What is actually true is worse in one way and better in another.** The tables are present because slot B is RAM-booted from a slot A whose camera was streaming, so the vendor's pages survive in DRAM at the addresses the replay arms. The pipeline has therefore been running on **inherited** tables, not on nothing, which is why the image looked right; and a cold slot-B boot, with no vendor stream first, would find those pages undefined. That has never been tested.

`ar-isp.c` now allocates and publishes its own gamma and DRC buffers, and produces every byte the hardware fetches from either. Gamma page 0 and the dynamic half of the DRC page come from the tuning file; gamma page 1 and the static half of the DRC page are not in that file in any form and are carried as decoded curves, extracted from the service library by `scripts/gen-gamma-page1.py` and `scripts/gen-drc-tail.py`. Neither table inherits anything.

**Current ownership of the tone and shading path**, validated on hardware with seeding off, so nothing came from the vendor's residual DRAM:

| Table | Descriptor | Source | Status |
|---|---|---|---|
| gamma | `0x0030`/`0x0040`/`0x0050` | tuning file + carried page 1 | every fetched byte ours |
| DRC | `0x0060` | tuning file + carried tail | every fetched byte ours |
| compander | `0x0020` | library template, verbatim | every fetched byte ours |
| GTM2 | `0x1c6c` | nothing to generate | ours; payload is runtime state, left zero |
| LSC region A | `0x4c34` | tuning file, two float32 arrays | every byte ours, exact |
| LSC region B | `0x4c34` | none exists | scene-adaptive, left to the seed path |

The only unowned bytes here are LSC's region B and GTM2's payload, and neither is recoverable from any file. Reproducing them means running the vendor's algorithms against live ISP statistics, which is 3A work.

Two of these names are this driver's, not the vendor's, and the distinction matters because the vendor has its own modules by those names on other banks. `0x4c34` is owned by `isp_sub_lsc` on bank `0x4c00`, so the driver calls it LSC; it was called LTM until the bank attribution below settled it. `0x1c6c` sits on bank `0x1c00`, which has no attributed owner, and keeps the name GTM2 for continuity with the captures and the harness. The vendor's own `gtm2` and `ltm` modules share bank `0x2800` and their descriptors `0x2808`/`0x280c` are still unowned.

Compander is now generated too, and it needed no generator at all. **It has no runtime producer and no tuning-file source**: the `0x7800` page is installed verbatim at ISP init from entry 6 of the descriptor array at VMA `0x472600`, a list of `{u64 source, u64 length}` pairs, whose body at VMA `0x46a3b0` is byte-identical to the page captured off a streaming vendor unit and to the page resident in DRAM on a RAM-booted unit. `scripts/gen-compander.py` extracts it and `ar_isp_compander_fill` rebuilds it. The bilinear at `0x186920` that an earlier note named as its generator produces something else; the `0x7800` size match was a coincidence.

Three quarters of the page is one 16-byte unity record repeated 1536 times and a further `0x700` bytes are zero, so only `0x900` bytes at the start and `0x800` at `0x1000` are carried: 4352 bytes rather than 30720. The generator script checks that structure against the library and refuses to emit if it has changed.

### LSC: the lens-shading grid is in the tuning file

LSC's `0x680` fetch is three parts, and the first is now generated:

	0x000..0x33f   10x10 lens-shading grid, generated from the tuning file
	0x340..0x5ff   scene-adaptive runtime state, no stored source
	0x600..0x67f   zero

**Region A is two 100-entry float32 arrays in the tuning file**, stored back to back at `raw + 0x910c` and `raw + 0x929c`, which is `0x7c` and `0x20c` past the LSC enable gate at `0x9090`. Each value is a gain, unity at the frame centre and rising to about 3.9 at the corners; the grid is a proper 10x10 bowl with an off-centre, anisotropic falloff. The table value is `floor(f * 2048)`, and truncation is measured rather than assumed: rounding matches 55 of 100 entries against a captured page, truncation matches all 100. Grid points pack two to a 16-byte record as `(x, x, y)` triplets, 50 records of data then two zero records. `ar_isp_lsc_from_blob` reproduces all 832 bytes exactly against two independent captures.

An earlier note here said no part of this page had a stored source. That was wrong, and the reason is worth recording: the search that produced it looked for the packed `u16` table, and the data is stored unpacked as `float32`. It was found by correlating every 100-value float window in the blob against the decoded grid, which matched at r = 1.0000.

**Region B has no stored source and is not worth searching for.** Between two captures of the same unit in different scenes, 43 of its 44 records differ, with 212 of 704 bytes changing: small deltas spread through nearly the whole region rather than a static part plus a dynamic part. It is computed by vendor userspace from ISP statistics delivered by event, so reproducing it is 3A work, not table work. It follows the driver's seed path, which means `lsc=1 seed=0` runs the block on shading alone.

### The coefficient pages overlap in DRAM

Measured, zero differing bytes in both directions:

	GTM2       0x2b2e0200   fetches 0x1000, holds 0xa00 of content
	compander  0x2b2e0c00 = GTM2 + 0xa00
	LSC        0x2b2e8600 = compander + 0x7a00

So GTM2's `0xa00..0xfff` **is** the compander table's first `0x600` bytes, read because GTM2 over-fetches past its own content, and a `0x8000` dump of the compander runs into the LSC page. GTM2's real payload is only the 512 bytes at `0x800..0x9ff`; `0x000..0x7ff` is zero.

`ar-isp.c` reproduces this rather than working around it: GTM2 and the compander share **one allocation**, GTM2 at offset 0 and the compander at `+0xa00`, and the two descriptors are published into the same block. That reproduces the fetched bytes for both without copying the shared `0x600` twice. The compander span is the `0xf000` its length field at `0x0024` implies rather than the `0x7800` the table occupies: in gamma's proven 32-byte units that is a fetch the vendor cannot satisfy either, since `0xf000` past its compander runs into the gamma page, so the excess is ignored by the block and allocating it only keeps the DMA inside memory we own.

### GTM2 needs nothing generated

Its `0x1000` fetch is `0x800` of zeros, then `0x200` of payload, then `0x600` of compander. The payload has no stored source: absent from the tuning file, from the service library, and from all 53 non-null entries of the ISP-init template array, and a float-correlation scan of the kind that located the LSC shading grid finds nothing above `r = 0.24` at 256, 128 or 64-value windows.

It is **scene-varying runtime state**, and that is measured rather than inferred: two captures of the same unit in different scenes differ by 117 of its 512 bytes, the same proportion as LSC's region B at 212 of 704. Its byte fingerprint matches region B's closely, entropy 5.50 against 5.53 with bit 7 set in 78% of bytes in both.

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

### GTM2 and LSC do not use the 0x0014 commit at all

They are module-local descriptor records, not entries in the global table selector. There is no `0x0014` bit for either; the only global commits are compander bit 0, DRC bit 4 and gamma bits 1 to 3.

| | pointer | length | valid |
|---|---|---|---|
| GTM2 | `0x1c6c` = `0x2b2e0200` | `0x1c74` = `0x80`, so `0x1000` fetched | `0x1c60` |
| LSC | `0x4c34` = `0x2b2e8600` | `0x4c28` = `0x34`, so `0x680` fetched | `0x4c3c` |

Recovered from the vendor write trace and independently confirmed against a live register read of a streaming vendor unit; the pointers agree exactly. Both valid bits read `0` mid-stream, so like the `0x0014` commit they appear to self-clear after the fetch.

The length fields matter for the same reason gamma's did: a flush size is the software allocation, not the fetch. **LSC fetches only `0x680`.**

An earlier version of this table attributed `0x4c34` to a handler at `0x18e2c4`, in the vendor's `isp_sub_ltm` code. That attribution was wrong. Both `0x18ad54` and `0x18e54c` flush `0x4000` and then publish to their bank `+0x08`, not `+0x34`, so neither is this descriptor; the handler that writes `0x4c34` is `isp_sub_lsc`'s at `0x1b6944`, which publishes `+0x34`, sets valid at `+0x3c`, writes length `0x34` units and flushes `0x340`. The descriptor values in the table above came from the trace and from live register reads, so they are unaffected; only the code attribution changes.

That makes `out/au-snapshot/tbl_isp_0x1c6c.bin` truncated to `0x1000` and `tbl_isp_0x4c34.bin` truncated to `0x680` exact oracles for the two pages.

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
| `isp_sub_lsc` | `0x4c00` | descriptor `0x4c34`, valid `0x4c3c`, length `0x4c28` = `0x34` units, the `0x680` fetch |
| `isp_sub_lut3d` | `0x5800` | four descriptors at `0x5810`/`0x5828`/`0x5840`/`0x5858` |
| `isp_sub_gtm2`, `isp_sub_ltm` | `0x2800` | descriptors `0x2808`/`0x280c`, both publish sites verified at `+0x08` |
| `isp_sub_digigain2` | CVISP `+0x4700` | register file, no DMA |

**The page this document calls LTM belongs to the vendor's `isp_sub_lsc`.** Its command handler publishes the DMA address to bank `+0x34`, sets valid at `+0x3c` and writes length `0x34`, exactly the known `0x4c34` record, and its tuning path reads the enable at `raw + 0x9090`, the known LSC gate. So the lens-shading grid already generated by `ar_isp_lsc_from_blob` is the LSC stage's table, and this handoff item was closed by the tone path before it was opened. The labels GTM2 and LTM used elsewhere in this document name the hardware descriptors, not the vendor modules: the modules the vendor calls `gtm2` and `ltm` sit together on bank `0x2800` (descriptors `0x2808`/`0x280c`), and the owner of bank `0x1c00` (descriptor `0x1c6c`) is unattributed. The descriptor-level facts recorded elsewhere are unaffected; only the vendor-side names move.

The float region past the LSC gate holds sixteen 10x10 grids, not one pair: groups at `raw + 0x910c`, `0x9784`, `0x9dfc`, `0xa474`, stride `0x678`, four back-to-back `0x190` grids per group behind a `0x38` header. Every grid correlates with the shipped pair at r > 0.997. The shipped pair is byte-exact against two captures, so the static bring-up choice is proven; the other grids are per-illuminant or per-channel variants for the runtime path (inference, not measured). The `0x2c0` region B stays scene-varying runtime state: it does not decode as the grid's triplet format, correlates with nothing in the tuning file at any window size, and carries the established runtime signature (top bytes clustering on `0xc3`/`0xc6`).

**LUT3D is present, armed, and disabled on the streaming vendor.** The init handler copies ISP-init template entries 42 to 45 verbatim (`0x458960`, `0x4562e0`, `0x453c60`, `0x4515e0`, `0x2680` each, four distinct banks of 16-byte records with nine content bytes) into four DMA banks, writes per-descriptor length `0x280` in 16-byte records (an over-fetch, flush is `0x2800`), publishes the four addresses and valid bits, and never reads the tuning file for payload. The tuning gate at `raw + 0x7b634` only drives module control `0x5800` bit 0 through the apply-tuning command. The working bringup's last write to `0x5800` is 0 and the whole bank reads zero on the streaming unit, so the module is off and the driver reproduces the vendor by leaving it off. The four banks are deliberately not carried in-tree; `ar-isp-colour.h` records the register layout and the template VMAs to extract them if the stage is ever enabled.

**CCM lands in registers, not a DMA page.** Both banks are `0x50` bytes: a packed 3x3 matrix at `+0x00`, a second copy at `+0x20`, a zero tail at `+0x40`. A matrix is six words, `{c0|c1<<16, c2, c3|c4<<16, c5, c6|c7<<16, c8}`, each coefficient 16-bit sign-magnitude Q8 with the magnitude truncated toward zero (`fcvtzs #8`; a negative v is encoded `0x8000 + |v|`). At init ccm1 gets an identity pair (template entry 33) and ccm2 gets the vendor's fixed matrix pair (entry 34, word-identical to the traced registers; its tuning gate at `raw + 0x2595c` reads 0, so it never moves). At runtime the AWB path packs an interpolated tuning matrix into ccm1's first copy only: gate `raw + 0x253fc` reads 1, the illuminant ladder is eight kelvins at `raw + 0x25438`, and four of eight matrix slots at `raw + 0x25470` (stride `0x24`, nine float32 row-major) hold data, matching the bank count in the selector block. The traced runtime write is bank 0 packed verbatim, which is the byte-for-byte proof: `scripts/gen-ccm.py` re-packs it and refuses to emit `ar-isp-ccm-init.h` on any mismatch. The packing functions are in `ar-isp-colour.h`. CCM enable is not a bank register: ccm2's enabled path clears bits 25 and 27 of the ISP global control word at `+0x0000` (both already 0 in the live streaming value `0xb0280052`).

### ml-isploop flags

`--cvisp` rotates the CVISP ring once per frame start and does **not** drive the per-frame ISP cycle. That is the combination recorded above as sustaining. The cycle is opt-in behind `--isp-cycle`; briefly folding it into `--cvisp` silently changed the behaviour of every existing caller.

`cycle[]` follows the order the wide sweep shows for the target registers: statistics buffers, then the VIF clears, then the three indirect transactions on `0x0cc`/`0x0d4`. Those two are an indirect access port, not acknowledgements, so their pairs must stay adjacent and ordered; the real interrupt acknowledgement is the VIF `0x17c` write.

### Never touch VIF without a live pixel domain

Reading VIF with its clock gated hard-hangs the SoC into a watchdog reset to slot A. The frame-start liveness check cannot prevent this, because the check is itself a VIF read: it catches a stopped stream but not a gated clock. `ml-isploop` now reads the camera gate bit in CGU `0x0a104014` first, which is safe at any time, and refuses before touching VIF.

"The modules are still loaded" is not the same as "the pipeline is live". Scripts kill their grabber on exit, which puts the sensor into standby and gates the pixel domain, so a later standalone invocation hangs. Run captures only through a gated harness.

## The capture node advertises raw Bayer but has never delivered a frame

`/dev/video2` advertises `V4L2_PIX_FMT_SRGGB12` and is wired to the VIF bypass view: 1920x1080,
`RG12`, 3840 bytes per line, 4147200 bytes per frame, so 16-bit padded samples. That is what the
node claims. **No frame has ever come out of it, on any run.**

Read mid-stream, with a grabber holding the stream open, the view is armed and running and still
signals nothing:

	0x17c  view-done W1C status    0x00000000    never signalled
	0x184  second W1C status       0x00000000
	0x1b0  block status            0x00000000
	0x020  armed view address      0x2c000000    a vb2 buffer is armed
	0x000  view control            0x0000c068    the configured value
	0x190  AXI config              0x40000101    RUN and ENABLE both set

The vendor does not use this path either. It configures view 0's geometry, then holds the view in
reset (`VIF+0x2bc = 0x00000002`) and streams everything through the ISP path; afterwards it never
writes a view address again. So there is no vendor sequence to copy, and bringing the view DMA up
means working from the hardware specification.

Two things that look like the cause are not. The `crop_v 0xffff` / `crop_h 0xffff` /
`frame_end 0x05000780` values in our init log are reset defaults printed before the write:
`ar_vif_configure()` writes `+0x3a0 = 0x84380280`, `+0x3c0 = 0x00000438`, `+0x3c4 = 0x0000027f`,
matching the vendor's live window exactly. And `+0x17c` bit 24, which the vendor acknowledges in
its per-frame loop, is an ISP-path frame event rather than bypass-view completion, so a poll loop
waiting on it is watching the wrong register.

**Consequence for judging images: there is no raw reference.** Whether the ISP is destroying
information has to be answered from the live register diff against the vendor or from a
same-scene vendor capture. Do not plan on a raw-against-ISP histogram comparison.

`ml-v4l2grab` writes its output file on the first and last frame of the requested count. A caller
that passes a large `-n` and kills the process, which is what a streaming harness does, gets the
first frame. Before that behaviour existed, such callers got no file at all.

## The sensor is unpowered outside a live stream

i2c reads of the sensor fail unless the pipeline is streaming, because the driver powers the
sensor down when the stream stops. Any sensor register comparison has to run inside a bring-up,
between the streaming gate and teardown.

## Liveness: a running grabber is not a delivering pipeline

Checking that `ml-v4l2grab` is still alive does not establish that frames are arriving. It blocks
indefinitely waiting for a buffer that never completes, staying alive the whole time, which is
its normal state on `/dev/video2`. Gate on a delivered artifact, not on the process.

Do not gate on the grabber's log either: its stdout is fully buffered when redirected, so the log
stays empty until it exits and an empty log proves nothing.

The same distinction applies to VIF frame starts: `ml-isploop` counts frame starts, which
continue on a failing bring-up, while no view DMA completes. Frame starts are not frames.

## Sensor registers that differ between stacks without meaning anything

A live-against-live comparison of all 3328 readable sensor registers, ours mid-stream on slot B
against the vendor's mid-stream on slot A, leaves twelve differences. None of them is
configuration. Four groups, all accounted for:

`0x0005` is a status byte that changes continuously between consecutive reads.

`0x0206`/`0x0207` is the committed analog gain code, and `0x0250`-`0x0253` is its shadow, exactly
`gain << 4`. The difference was our old default of `0x2f` against the vendor's `0x3c`; the driver
now defaults to `0x3c`.

`0x32bc`-`0x32bf` is written by no code path in the vendor library. It is an output of the
sensor's own MCU, in the same class as the gain shadow, and exists on the vendor unit only
because its AE loop has run.

`0x0138`-`0x013a` is the SMIA temperature sensor: `0x0138 = 0x01` enables it on both stacks and
`0x013a` is `TEMP_SENSE_OUTPUT`. The value drifts upward as the unit warms, and it drifts *within*
one stack, so it is not a stack difference at all. Five dumps, two stacks:

	ours   0x0139 = 0x23  0x013a = 0x74
	ours   0x0139 = 0x24  0x013a = 0x75
	vendor 0x0139 = 0x25  0x013a = 0x77
	vendor 0x0139 = 0x26  0x013a = 0x78
	vendor 0x0139 = 0x2a  0x013a = 0x7e

The two bytes track each other with a near-constant offset of about `0x52`, so they are one
rising quantity read out in two places. The vendor's own dumps span `0x25` to `0x2a`, a wider
spread than the gap to ours.

`0x0340` read `0x08` on the vendor during the first snapshot. That was contamination from an
accidental i2c write, not a vendor value; a clean stack reads `0x04`, which is what we write.

The 271 registers our mode table never writes are power-on defaults: they read identically on
both stacks.

## Sensor clamps are ours, not the chip's

Exposure is clamped to `vts - 2` and gain to `NT99235_AGAIN_MAX` (`0x60`) by the driver.
Requesting exposure 2000 reads back as 1123 from the sensor, so the chip never sees the larger
value and its own limit is unknown. Gain values above `0x60` all behave identically for the same
reason. Both clamps are correct for a fixed frame rate, but they are decisions, not measurements.
