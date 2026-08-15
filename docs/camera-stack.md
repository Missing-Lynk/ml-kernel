# Camera stack (NT99235 / CSI-2 / VIF / ISP / CVISP)

The air unit (BetaFPV VR04, Proxima-9311) captures from an NT99235 image sensor, on a camera module connected to the SoC board, over MIPI CSI-2. The vendor firmware drives that path entirely from userspace; reproducing the vendor's sequence in open drivers is the goal.

This is the readable account: what the hardware is, what the pipeline does to a frame, which stages run, and what the open drivers do. The register-level record behind it, what the vendor writes and where every value comes from, is `camera-isp-recovery.md`.

Drivers: `overlay/drivers/media/artosyn/` (`nt99235.c`, `ar-csi2.c`, `ar-vif.c`, `ar-isp-main.c` + `ar-isp-tables.c`, `ar-cvisp.c`). Device tree: `devices/betafpv-vr04-air/proxima-9311-air.dts`.

## Blocks

| Block | Base | Notes |
|---|---|---|
| VIF | `0x08870000` (64 KiB), IRQ SPI 62 | front end (path0) plus 8 capture views |
| MIPI CSI wrapper | `0x08880000` (64 KiB), IRQ SPI 60/61 | 8 receiver instances in 4 pairs; each pair `0x1000`: Artosyn glue at pair base, DesignWare host core 0 at `+0x400`, core 1 at `+0x800` |
| ISP | `0x08c00000` (2 MiB), IRQ SPI 71 | Bayer processing; feeds CVISP, does not write frames itself |
| CVISP | `0x08e00000` (64 KiB), IRQ unknown | the output stage: owns the frame queue and writes to DRAM |
| CGU | `0x0a100000` / `0x0a104000` / `0x0a108000` | clock controller |

This board uses CSI pair 0, core 0 at `0x08880400`. The DesignWare core version register reads v1.20, checked at probe.

**All four blocks are inside the SoC**, in its media subsystem. The NT99235 is not: it sits on a separate camera module, a small board carrying the sensor and its optics, connected to the SoC board over the MIPI CSI-2 lanes and an I2C control bus. Everything with a base address in the table above is on the Proxima-9311; everything reached by an I2C register address is across that connector on the module.

The split matters for reading the rest of this document. A register written with `writel` to one of those bases is the SoC's. A register written over I2C, addressed in the SMIA space (`0x0100` streaming, `0x0202`/`0x0203` exposure, `0x0206`/`0x0207` gain) or in Novatek's own `0x3xxx`/`0x8xxx` ranges, is the module's, and no amount of ISP configuration reaches it.

The module is not a bare pixel array. Besides the array, its PLL, exposure and analogue gain, and the CSI-2 transmitter, it applies its **own lens shading correction** from tables at `0x8250`-`0x826d` and `0x8550`-`0x855c`, committed with `0x8201`. That correction happens on the module before a single pixel reaches the SoC, so it sits upstream of every stage described here and is invisible to the ISP's own `lsc` bank.

The chain is: sensor sends CSI-2 packets over the MIPI lanes; the **CSI-2 receiver** decodes the link and produces a pixel stream on its IPI output; the **VIF** is the SoC's capture front end, which accepts that pixel stream and routes it, either straight to DDR through its bypass views (a path that has never been proven to deliver a frame, on either stack) or onward into the ISP; the **ISP** converts Bayer to YUV; and **CVISP** takes the ISP's output, owns the frame queue, and writes frames to DDR.

`VIF` is the vendor's own name for the block, used throughout `libmpp_service.so` (`vif_*`, 118 functions) and in the interrupt it registers (`ar_irq_reg_with_name(irq, handler, dev, "vif")`). It is the video interface between the receiver and the rest of the media block: it sees pixels, where the receiver sees packets. It also measures the incoming video timing, which is what makes `0x1f0` the front end's ground-truth register.

The vendor kernel does no camera hardware programming. `cam_hardware_power_on` calls `ar_pwr_ctrl_on(1)`, which on the 9311 dispatches through an uninstalled pointer and returns -1. All VIF, CSI, ISP and CGU register writes on the stock unit come from userspace (`libmpp_service.so`, called by `ar_lowdelay`) through `/dev/mem`.

## The ISP pipeline

The ISP is a chain of small fixed-function stages, each with its own register bank and its own enable. The vendor library names all 65 of them; this configuration writes registers for the 38 below, and runs a subset of those.

The table is generated, not maintained: `scripts/isp/isp-pipeline.py` reads which register gates each stage out of `vendor-tables/ar-isp-gates.h`, evaluates it against the values the driver actually installs, and asks `scripts/isp/audit-provenance.py` where those values come from. Re-run it rather than trusting this copy.

**Runs** answers one question: does this sub-block touch the pixels? These are hardware stages inside the ISP, each with its own register bank and its own enable bit, not Linux modules. *off* means the block is disabled or bypassed and the frame passes through it unaltered. The ISP as a whole is running regardless; a stage being off says nothing about the block.

Three things the column does not mean. It is read from the register values this driver installs, so it describes the shipped configuration and not a live readback. A stage that is off still has its registers written, because the driver reproduces the vendor's whole register state and the vendor installs tables for stages it then leaves disabled. And the verdict is per stage, not per function: `blc` reads *off* here while black-level subtraction happens anyway, on CVISP bank `0x4200`, which this table does not cover.

The verdict is cross-checked against a second, independent source: the enable flag the vendor's tuning file stores for that stage, which is what the vendor configured the stage out of and cannot depend on how the register gate was read. *(file agrees)* marks a gate whose polarity was inferred rather than read out of the library's own branch, and which the tuning file then corroborated. *(tuning file only)* marks a stage with no recovered register gate, where the file is the only reading there is. Where the two sources disagree the row says so and leaves it open, because the disagreement is the finding: `birnr` reads as running while the file has it off, and it shares gate `0x1800` bit 0 with `rnr`, which the file has on, so one of those two gate assignments is wrong. *undecided* means the stage's own register gates contradict each other.


**Where its values come from** is the provenance of most of that bank. *derived from the tuning file* means the driver recomputes it at runtime from the sensor's tuning blob, the way the vendor does. *the vendor library image* means it is the static default the vendor's own submodule carries. *a recording of the vendor* means the value exists because the value was observed, and is only known correct at the operating point it was observed at; the count beside it is how many registers in that bank are still in that state.

| Stage | Runs | What it does | Where its values come from |
|---|---|---|---|
| **sensor correction** | | | |
| `blc` | off | subtracts the per-channel black level, on CVISP bank 0x4200 rather than in the ISP register file | no ISP register written |
| `gib` | gate unwritten | green imbalance between the two Bayer greens | no ISP register written |
| `dpc` | undecided | replaces defective pixels | derived from the tuning file |
| `lsc` | runs (inferred) | lens shading: a 10x10 gain grid that lifts the corners | the vendor library image |
| `digigain1` | off (inferred) | digital gain ahead of the noise stages | no ISP register written |
| `compander` | off (inferred) | companding curve between the sensor range and the pipe | a DMA page |
| `decompander` | off (inferred) | the inverse curve | a DMA page |
| **noise reduction** | | | |
| `rnr` | runs | radial noise reduction, strength rising toward the corners | derived from the tuning file |
| `birnr` | runs | bilateral noise reduction in the Bayer domain | derived from the tuning file |
| `lee_lnr` | runs | luma noise reduction, bank 0x3cc8, which the register map calls lnr | derived from the tuning file |
| `de3d` | no gate recovered | temporal noise reduction across frames, the motion-sensitive stage | derived from the tuning file |
| `raw_3dnr` | disabled | raw-domain temporal filter | tuning gate clear; vendor and open bank heads are zero |
| **demosaic** | | | |
| `cfa` | no gate recovered | Bayer to RGB, the point where the image gains three channels | derived from the tuning file |
| **colour** | | | |
| `wb` | runs | per-channel white balance gains | the vendor library image |
| `ccm1` | runs | the 3x3 colour correction matrix | derived from the tuning file |
| `ccm2` | no gate recovered | a second colour matrix | derived from the tuning file |
| `cm` | runs | colour manipulation | derived from the tuning file at the pinned trigger scalar |
| `cm2` | runs | a second colour manipulation block | derived from the tuning file at the pinned trigger scalar |
| `acm` | off | adaptive colour manipulation | the vendor library image |
| `cnf` | runs | chroma noise filter | derived from the tuning file |
| `lut3d` | off | a 3D colour lookup table in four DMA banks | the vendor library image |
| `qgg` | no gate recovered | quadratic green gain | the vendor library image |
| `lms` | off | long/medium/short colour space conversion | the vendor library image |
| **tone** | | | |
| `gamma` | runs (inferred) | the gamma transfer curve, fetched as a DMA page | tuning file + carried page |
| `drc` | runs (inferred) | dynamic range compression, fetched as a DMA page | tuning file + carried tail |
| `ltm` | no gate recovered | local tone mapping: 64 per-tile transfer curves, recomputed per frame | read out of the vendor packer |
| `gtm2` | runs | global tone mapping, sharing ltm bank 0x2800 | read out of the vendor packer |
| **colour space** | | | |
| `rgb2yuv` | no gate recovered | RGB to YUV, the point where the image becomes luma and chroma | derived from the tuning file |
| **geometry** | | | |
| `binning_filter` | gate unwritten | binning ahead of the scaler | no ISP register written |
| **statistics** | | | |
| `rro_stats` | runs | the 36x16 zone grid AE meters from | the vendor library image |
| `face_rro_stats` | runs | a second zone grid over a smaller window | the vendor library image (2 still recorded) |
| `raw_his_stats` | runs | the Bayer histogram, 128 bins by 4 lanes | the vendor library image |
| `awbs_stats` | off | the white-balance accumulator | the vendor library image |
| `af_stats` | off | the autofocus accumulator | the vendor library image |
| `derolling_stats` | gate unwritten | rolling-shutter statistics | no ISP register written |
| `rgb_his_stats` | gate unwritten | the RGB histogram | no ISP register written |
| `rgb_max_stats` | gate unwritten | per-channel maxima | no ISP register written |
The `hdr` and `ir` families have banks in the register map and are absent from the pipeline: `hdr` needs a second sensor exposure and `ir` an infrared channel, and this camera module produces neither.

Seven stages recompute continuously as the scene changes rather than sitting at a configured value: `rnr`, `lee_lnr` and `de3d` all interpolate between gain bands in the tuning file. `cfa`, `cnf`, `cm` and `cm2` do the same through their own record layouts. All seven take one abscissa, and current `ml-3a` drives it; the shared gain-keyed gate boot is the remaining hardware proof.

## The open drivers

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

**Live, on hardware.** 184 sensor registers read back over I2C from the streaming vendor on slot A and from the open stack on slot B, both mid-stream with the VIF front-end gate confirmed at `0x0784043c`. **Zero differences.** The 26 that used to differ, in `0x8250`-`0x826c` and `0x8550`-`0x855c`, were closed by `nt99235_shading_regs` and re-measured: the two sensor states are now bit-identical.

Those 26 do **not** drift at runtime, and 3A does not write them. They are a step in the vendor's bring-up that the open driver was missing.

The sensor object `libsns_nt99235.so` exports carries a lens shading entry point (the function at `0x6e80`, reached through the pointer at data `0x1b128`) which the MPP layer calls after the mode table, with a selector: 0 installs a static table, 1 installs a flat `0xff`/`0x00` one. Selector 0 reproduces all 31 of the streaming vendor's live values exactly, `0x826c` = `0x80` included, and finishes with the `0x8201` = `0x0f` commit.

The point of it is that it is **mode-independent**. Three of the four mode tables already carry these values, but the one both stacks run here, `1920x1080p60` at two lanes, carries its own set; the install overwrites them. The open driver programmed the mode table and stopped, which is the entire 26-register difference. `nt99235_shading_regs` replays the install after the mode table, and against the slot A / slot B pair that lands on the vendor's value for all 26.

Two earlier readings of this were wrong and are recorded here so they are not re-derived. The attribution to AWB was wrong: the AWB sensor callback is a single function that memsets an eight-byte struct and fills it from two static `u16` arrays, so it reports capability and writes no sensor register at all. A later attempt to shrink the count to six was also wrong, from diffing against the `1920x1080p60-4lane` table, which is not the mode either stack runs (`0x0114` reads `0x01` on both).

One register is hardware-owned rather than a difference: `0x8205` is written `0x03` by the mode table and reads back `0x02` on **both** slots.

**Exposure and gain are set outside the mode table.** The vendor drives them from its 3A layer at runtime; the mode table does not touch them. The open driver commits explicit defaults before stream-on (integration 1123 lines, gain code `0x3c`, the vendor's own operating point) through the writable `exposure` and `gain` module parameters, and `ml-3a` updates them live (see "Scene-adaptive state"). Brightness therefore reflects AE state, not pipeline health; judge a capture by the marker count (see "Judging a capture").

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

The front-end half of this driver is validated. The view-engine half implements the bypass view DMA, but that path is **unvalidated: no bypass view has ever delivered a frame, on either stack**. The vendor holds the views in reset and streams through the ISP path instead (`camera-isp-recovery.md`, "The VIF views serve the bypass path"), so the view-engine code is written from the register map without a working reference; treat every view-engine claim as untested (see "The capture node advertises raw Bayer but has never delivered a frame" under "Working on it").

- Probe claims the reserved capture pool, ioremaps `0x08870000`, gets the DT clock "axi" (`cgu_vif_axi_clk`), sets up a vb2 dma-contig queue and the V4L2/media devices, and registers an async notifier for the CSI source. Completion is interrupt-driven by default; `use_irq=0` falls back to polling.
- The block loses all state when the axi clock gates.
- `ar_vif_global_init`: replays the vendor `vif_reg_init` 28-pair table, a soft-reset pulse, the RGB2YUV coefficient and clip bank (BT.709 limited), input-pipe transient writes, a per-view output-FIFO reset, AXI master enable and run, frame-boundary and DDR-clip config, and the block-enable release. Runs once per session with the clock already on.
- `ar_vif_configure`: front-end input format, geometry, blanking, stride strobe and value, view control, per-view FIFO partition, view mux/enable/threshold, crop window covering the full frame, frame-end check, and input-format select.
- Raw Bayer moves 3 pixels per group, so the crop and frame-end width fields count groups of three.
- A `test_pattern` module param feeds view 0 from the path-0 pattern generator (`0x0f4` bit0).

### Processing: `ar-isp-main.c`, `ar-isp-tables.c`

Configures the stage banks of the table above and generates the DMA payloads the block fetches.

- `ar-isp-main.c` replays the vendor's setup order and then applies the stages that compute their own registers, so a stage whose transform is recovered overwrites the replayed value rather than sitting beside it.
- `ar-isp-tables.c` builds every byte the hardware fetches: the gamma page, the DRC page, the compander page, the LSC shading grid, the LTM coefficient page, and the statistics buffers the block writes into. Each is published to a descriptor register and committed. The generators are the `scripts/isp/gen-*.py` set, each of which self-checks against the vendor's own data and refuses to emit on a mismatch.
- The stages that recompute from the tuning file live in one header each, `ar-isp-rnr.h`, `ar-isp-lnr.h`, `ar-isp-de3d.h`, `ar-isp-cfa.h`, `ar-isp-cnf.h`, `ar-isp-cm.h` and `ar-isp-cm2.h`, over the shared band-selection and blend law in `ar-isp-ladder.h`. `scripts/isp/check-*-ladder.py` proves each against the vendor's live registers, and `check-ladder-c.py` compiles the shipped headers host-side and runs them beside the Python so the kernel's C is what was proved.

The reproducibility boundary is source-based. A new `nt99235_tuning_preview_fpv.bin` drives the blob-fed runtime stages directly: gamma page 0, DRC dynamic profiles, LSC, BLC, and the RNR/LNR/DE3D/CFA/CNF/CM/CM2 ladders. A new `libmpp_service.so` drives the checked-in ISP static headers under `vendor-tables/` through `gen-isp-library.py`, `isp-gates.py`, `gen-ccm.py`, `gen-gamma-page1.py`, `gen-drc-tail.py`, `gen-compander.py` and `gen-rgb2yuv.py`. The CVISP defaults come from a wide MMIO trace via `gen-cvisp-defaults.py`. `audit-provenance.py` is the regression gate after either ISP input swap: the current target is zero unexplained replay values and zero device-capture-only values, with the two hardware-owned readbacks kept isolated.

### Output: `ar-cvisp.c`

The V4L2 capture node, and the block that writes frames to DRAM. With `chain=true`, the default, `STREAMON` brings the whole chain up.

- Frames are planar YUV at line stride 2048 for a 1920-wide plane. The 128 bytes of per-line padding hold whatever was in DRAM; dump `stride * height` and read the active columns only.
- One ring slot is armed per VIF frame start, the vendor's own cadence. The arm is what triggers the write: a fully configured, enabled CVISP with no slot armed writes nothing.
- The clock is inherited rather than requested. The boot firmware leaves the `cgu_rsz_clk` gate set and the vendor never touches it, so `ar-cvisp` does the same and takes no ownership.
- The driver reads no register at probe. The clock leaf identity is an assumption, and if it is wrong the first access hangs the SoC; a probe-time read would make that unavoidable on every boot.

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

Read live from a streaming stock unit and cross-checked against a read+write trace of the CGU window, which contains only 66 writes for the whole camera bringup.

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

- **Sensor.** Configuration verified equivalent: mode table matches the vendor library write for write in order, and all 184 live registers read back bit-identical to the streaming vendor mid-stream. Exposure and gain are driven explicitly; the vendor drives them from AE.
- **CSI-2 link.** Every `INT_ST_*` bank reads zero on the steady-state second read, matching the vendor. The banks are clear-on-read: always read twice.
- **VIF front end.** Measures the incoming timing correctly (`0x1f0` = `0x0784043c`) and matches the vendor's live values; frame starts fire at frame rate. The four registers the vendor's end state differs in (`0x080`, `0x08c`, `0x140`, `0x2bc`) were set to vendor values on hardware with no effect.
- **ISP.** Register state is vendor-identical over the modeled range. `scripts/isp/audit-provenance.py` currently audits 1260 driver-written ISP registers: 1258 regenerate from the NT99235 tuning blob, `libmpp_service.so` static images, or driver-owned geometry/DMA state, and 2 are hardware-owned readbacks. The audit reports zero unexplained replay values and zero device-capture-only values. Open ISP image-parity work is now runtime selection and hardware validation: CFA/CNF/CM/CM2 gate proof, gamma/DRC tone selection and LTM/CLAHE. `raw_3dnr` is classified disabled by `scripts/isp/check-raw-3dnr.py`.
- **CVISP.** Configured, armed per frame start, sustains 60 fps.
- **CGU.** The open stack programs three of the fourteen registers the vendor programs; the rest, including two reset-control writes, are inherited from boot state. Unexplained but measured harmless in the working chain.

Two trace-reading facts that stay relevant: the vendor never spin-waits (no consecutive-read run on any block), and the vendor brings the pipeline up, tears it down, and brings it up again, so the head of a trace shows first-bringup behaviour, not the working path.

The cold-boot dark frame is clean (`camera-isp-recovery.md`, "The cold-boot dark-frame blobs"); the edge notch on synthetic bars is the sensor's own test-pattern generator, not an ISP stage: edge-aligned averaging over 3,400 real-scene edges shows no trace of it at optical edges, so it never affects live imagery.

### Scene-adaptive state: AE closed, the rest still frozen

Parity above is at a fixed operating point. The AE loop is implemented and hardware-validated: `native/ml-3a.c` meters from the `rro_stats` zone grid and drives sensor exposure, sensor gain and the gain-keyed ISP ladder abscissa from one exposure-table index, through `/sys/module/nt99235/parameters/{exposure,gain}`, the ISP `*_gain` Q8 parameters and the debugfs `ladders` re-arm. The validated runs covered the AE law and the original RNR/LNR/DE3D ladder actuation; the current seven-ladder hook adds CFA/CNF/CM/CM2 and still needs its gate-validation boot.

Still frozen, held constant where the vendor's 3A moves them: the vendor AE's anti-flicker snap of the integration time to the mains half-period, the gamma table (regenerated continuously from 3A; ours is generated once), the AE-selected tone-table index over the gamma and DRC profiles, and the LTM tile curves (recomputed per frame; ours ships an identity page). The shipped tuning gates AWB off, so `wb` and the traced `ccm1` matrix are static vendor state on this unit. These are loops or selectors to implement, not registers to fix.

## Working on it

### Judging a capture: the marker count, never the image

`ml-isploop` writes 24 marker words (`0xa5a5a5a5`, every `0x1000` bytes) into each output plane before its capture window opens, and reports how many survived. **A capture is only ours if those markers were overwritten.** `0/24` means nothing in our stack wrote that memory and the dump is whatever was already in DRAM.

DDR survives a RAM-boot, so the vendor's last frame from slot A persists at the same addresses our ring uses, and reads as a plausible healthy capture. A residual vendor frame is also useful deliberately, as a same-sensor same-scene control through the vendor pipeline; dump it before our own stack runs.

### CVISP writes only on the first bring-up after a RAM-boot

The first camera bring-up of a boot writes frames to DRAM. Every later bring-up in the same boot arms normally, reports full frame-start and slot-arm counts, and writes nothing, leaving the first bring-up's frame in place.

Measured: first bring-up 24/24 markers overwritten on all three planes; second and third `0/24`, each with 241 frame starts and 241 slots armed in 4 s and CVISP control `0x00800806` correct throughout.

This is not simply cold-versus-warm. Slot B is RAM-booted from slot A with the vendor pipeline already streaming, so the first bring-up **inherits hardware state the vendor established**, and the register replay does not recreate it once our own teardown has destroyed it. A boot therefore yields exactly one trustworthy capture; `RUNS=` in `glue/camera/au-prove-camera.sh` selects which one to spend it on.

### Within a working bring-up, capture sustains

All five ring slots hold distinct frames after a one-second window, differing by 10 to 16 per cent of sampled pixels. The pipeline streams at 60 fps; it does not produce a single frame and stop.

### ml-isploop flags

`--cvisp` rotates the CVISP ring once per frame start and does **not** drive the per-frame ISP cycle. That is the combination recorded above as sustaining. The cycle is opt-in behind `--isp-cycle`; briefly folding it into `--cvisp` silently changed the behaviour of every existing caller.

`cycle[]` follows the order the wide sweep shows for the target registers: statistics buffers, then the VIF clears, then the three indirect transactions on `0x0cc`/`0x0d4`. Those two are an indirect access port, not acknowledgements, so their pairs must stay adjacent and ordered; the real interrupt acknowledgement is the VIF `0x17c` write.

### Never touch VIF without a live pixel domain

Reading VIF with its clock gated hard-hangs the SoC into a watchdog reset to slot A. The frame-start liveness check cannot prevent this, because the check is itself a VIF read: it catches a stopped stream but not a gated clock. `ml-isploop` now reads the camera gate bit in CGU `0x0a104014` first, which is safe at any time, and refuses before touching VIF.

"The modules are still loaded" is not the same as "the pipeline is live". Scripts kill their grabber on exit, which puts the sensor into standby and gates the pixel domain, so a later standalone invocation hangs. Run captures only through a gated harness.

### The capture node advertises raw Bayer but has never delivered a frame

This section is about the VIF bypass view node only. The working capture path is `ar-cvisp`'s V4L2 node, which with `chain=true` (the default) brings the whole chain up at STREAMON.

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

### The sensor is unpowered outside a live stream

i2c reads of the sensor fail unless the pipeline is streaming, because the driver powers the module down when the stream stops. Any sensor register comparison has to run inside a bring-up, between the streaming gate and teardown.

### Liveness: a running grabber is not a delivering pipeline

Checking that `ml-v4l2grab` is still alive does not establish that frames are arriving. It blocks indefinitely waiting for a buffer that never completes, staying alive the whole time, which is its normal state on `/dev/video2`. Gate on a delivered artifact, not on the process.

Do not gate on the grabber's log either: its stdout is fully buffered when redirected, so the log stays empty until it exits and an empty log proves nothing.

The same distinction applies to VIF frame starts: `ml-isploop` counts frame starts, which continue on a failing bring-up, while no view DMA completes. Frame starts are not frames.

## Source map

| Concern | Files |
|---|---|
| Sensor init, MCLK, stream-on | `overlay/drivers/media/artosyn/nt99235.c` |
| CSI-2 D-PHY, lanes, IPI, deskew | `overlay/drivers/media/artosyn/ar-csi2.c` |
| VIF front end, view arm, DMA | `overlay/drivers/media/artosyn/ar-vif.c` |
| ISP configuration | `overlay/drivers/media/artosyn/ar-isp-main.c`, fetched buffers `ar-isp-tables.c`, tables `vendor-tables/ar-isp-defaults.h` from `scripts/isp/gen-isp-defaults.py` |
| CVISP output stage and queue | `overlay/drivers/media/artosyn/ar-cvisp.c`, tables `vendor-tables/ar-cvisp-defaults.h` from `scripts/isp/gen-cvisp-defaults.py` |
| DT nodes (camera, CSI, VIF, ISP, CVISP, clocks, carveouts) | `devices/betafpv-vr04-air/proxima-9311-air.dts` |
| Vendor MMIO write trace, per block and wide sweep | capture harness `glue/camera/au-slotA-mmiotrace.sh`, shim `native/mmiotrace.c`; the logs are capture artifacts, not in the tree |
| The pipeline table above | `scripts/isp/isp-pipeline.py`, over `vendor-tables/ar-isp-gates.h` and `scripts/isp/audit-provenance.py` |
| What the vendor writes, and where each value came from | `camera-isp-recovery.md` |
