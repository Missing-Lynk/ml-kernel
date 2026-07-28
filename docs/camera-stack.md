# Camera stack (NT99235 / CSI-2 / VIF / ISP)

The air unit (BetaFPV VR04, Proxima-9311) captures from an onboard NT99235 image sensor over MIPI CSI-2. The vendor firmware drives that path entirely from userspace; reproducing the vendor's sequence in open drivers is the goal.

This document records two things and keeps them separate: **how the vendor drives the hardware**, recovered from a live MMIO write trace of the streaming stock unit, and **what the open stack currently implements and has validated on hardware**. Everything stated here is backed by a named artifact or a hardware observation. Anything not yet established is listed under "Not known".

Drivers: `overlay/drivers/media/artosyn/` (`nt99235.c`, `ar-csi2.c`, `ar-vif.c`). Device tree: `devices/betafpv-vr04-air/proxima-9311-air.dts`.

## Blocks

| Block | Base | Notes |
|---|---|---|
| VIF | `0x08870000` (64 KiB), IRQ SPI 62 | front end (path0) plus 8 capture views |
| MIPI CSI wrapper | `0x08880000` (64 KiB), IRQ SPI 60/61 | 8 receiver instances in 4 pairs; each pair `0x1000`: Artosyn glue at pair base, DesignWare host core 0 at `+0x400`, core 1 at `+0x800` |
| ISP | `0x08c00000` (2 MiB), IRQ SPI 71 | frame-capture DMA; no open driver binds it |
| CGU | `0x0a100000` / `0x0a104000` / `0x0a108000` | clock controller |

This board uses CSI pair 0, core 0 at `0x08880400`. The DesignWare core version register reads v1.20, checked at probe.

The vendor kernel does no camera hardware programming. `cam_hardware_power_on` calls `ar_pwr_ctrl_on(1)`, which on the 9311 dispatches through an uninstalled pointer and returns -1. All VIF, CSI, ISP and CGU register writes on the stock unit come from userspace (`libmpp_service.so`, called by `ar_lowdelay`) through `/dev/mem`.

## How the vendor drives the pipeline

### The trace

`out/au-mmiotrace/mmio-combined.log`: 48183 register writes from one boot of the stock firmware, captured with `native/mmiotrace.c` (an `LD_PRELOAD` shim under `ar_lowdelay`, installed by `glue/dev/au-slotA-mmiotrace.sh`) over the window `0x08860000`-`0x08c0ffff`. Lines are `wNNNNNN wWW pa=0xADDR val=0xVAL` in program order.

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

### Not known

These are open and are not to be assumed while building the ISP driver:

- **How the vendor detects frame completion.** The trace has no reads. Whether the loop is driven by the VIF frame-start IRQ, an ISP IRQ (SPI 71), or a poll is undetermined.
- **What each of the eight address registers points at.** Plane, statistics buffer, or intermediate surface is unassigned. The two-value alternation is established; the meaning of each register is not.
- **The pixel format in those buffers.** Whether the ISP output is raw Bayer or processed YUV has not been read back and decoded.
- **What the three indirect-port transactions do.**
- **Whether the on-chip DSP participates.** The DSP runs AiISP 3A tuning (AE/AWB/AF) over RPC and `libhal_dsp.so` is present on the unit. Whether it also writes ISP datapath registers cannot be answered from this trace, because DSP-core writes are not visible to the shim. The whole ISP configuration and the whole per-frame loop observed above come from userspace MMIO, so at minimum the datapath is userspace-driven; whether that is sufficient is untested.
- **The function of the ~1267 ISP registers.** They are captured verbatim; they are not decoded.

## Open stack

### Sensor: `nt99235.c`

V4L2 sensor subdev on I2C-0, address `0x1a`. 16-bit register address, 8-bit data, raw 3-byte I2C messages (no SMBus).

- Mode tables are opaque register sequences transcribed verbatim from the vendor `libsns_nt99235.so`, roughly 200 writes each. Modes: (0) 1920x1080p60 2-lane, the vendor's mode; (1) 960x540p120 2-lane; (2) 1920x1080p60 4-lane; (3) 1280x720p90 4-lane. Lane count is set inside each table by SMIA register `0x0114` (`0x01` = 2 lanes, `0x03` = 4 lanes). Resolution, format, PLL, and line/frame length are all inside the table.
- MCLK is 24 MHz on `cgu_sensor_mclk0` (DT clock name "mclk") and must be running before reset is released.
- Power-on, verbatim from the vendor `nt99235_cmos_power_on`: assert enable (gpio104) and reset (gpio107), both active low; settle 10 ms; enable MCLK; release reset; settle 10 ms. The enable line stays asserted for the whole session.
- `s_stream(1)`: runtime-resume, write the mode's full register table, `msleep(20)` for PLL and MCU settle, then `MODE_SELECT` (`0x0100`) = `0x01`. Stream-off writes `0x0100` = `0x00`.
- Link frequency and pixel rate are exported as read-only V4L2 controls so the receiver can pick its D-PHY range. The DT endpoint lane count selects and filters the modes.

### Receiver: `ar-csi2.c`

V4L2 bridge subdev driving the DesignWare CSI-2 host core plus the Artosyn glue wrapper.

- Clocks: "csi" (`cgu_mipi_csi_0_clk`) and "pcs" (`cgu_mipi_pcs_clk`), both enabled at stream-on.
- D-PHY bring-up (`ar_csi2_phy_power_on_core`): hold PHY shutdown and reset, clear the test interface, run a dummy test cycle, write the fixed setup registers, write the lane registers, write the frequency-range code, then release shutdown and reset.
- Lane config: wrapper `NUM_LANES` and core `N_LANES` both = lanes - 1. A link wider than 2 lanes merges both cores via the wrapper `SCENARIO` and `LANE_MERGE` registers.
- Frequency-range code: `rate_mbps = 2 * source_link_freq`, mapped through 8 coarse bins. The vendor mode (900 Mbps/lane, 2 lanes) maps to range code 5; a 4-lane 456 Mbps/lane link maps to code 3. Overridable with the `phy_range` module param.
- IPI runs in camera-timing mode, cut-through, VCID 0, data type `0x2c` (RAW12).
- Error and interrupt registers are a 10-entry mask table written twice, masked during bring-up and unmasked after. Per-PHY free-running HS activity counters in the wrapper serve as a lane-routing probe.
- Clock-lane re-acquisition fix (hardware-confirmed): D-PHY internal register `0x3d`/`0x45` bit5 is set during PHY init and must be cleared at the end of configure. Left set, the receiver re-acquires the clock lane every frame and corrupts each frame-start header.
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
- `0x0a10401c` = `0x1102`: undecoded low bits on `cgu_mipi_csi_0_clk`.

The open CGU provider models these as gate-only leaves, so `clk_prepare_enable` sets the gate bit but not the parent-select mux. The vendor prologue additionally sets the parent selects and enables `cgu_isp_hdr_clk`, which the open stack does not.

## Status

Validated on hardware, sensor through VIF front end:

- **Clocks.** The CGU prologue reads back the vendor's values and the downstream blocks come alive.
- **Sensor.** `nt99235` powers on and streams; the receiver sees its data.
- **CSI-2 link.** Clean at the driver's D-PHY settings. A `phy_range`/`hs_settle` sweep read all error banks (`PHY_FATAL`, `PKT_FATAL`, `FRAME_FATAL`, `INT_ST_IPI`) as zero, with a positive control (settle `0x08` did raise ECC and frame errors), so the zero is real. The clock-lane re-acquisition fix holds.
- **VIF front end (path0).** Accepts frame-start delimiters at frame rate. `path0_frame_start` (`0x17c` bit24) fires continuously and the steady-state interrupt-ack pattern is byte-identical to the running vendor.

Not built:

- **The ISP capture path**, which is the vendor's frame-to-DRAM mechanism. No open driver binds `0x08c00000`. This is the remaining work: a driver that reproduces the vendor's ISP configuration and per-frame loop.
- The VIF bypass view DMA the open driver currently arms produces no frames and is not the vendor's path. It is retained as front-end scaffolding, not as the capture mechanism.

Investigation history and next steps live in `plans/air-camera-first-light.md`; this file tracks mechanism only.

## Source map

| Concern | Files |
|---|---|
| Sensor init, MCLK, stream-on | `overlay/drivers/media/artosyn/nt99235.c` |
| CSI-2 D-PHY, lanes, IPI, deskew | `overlay/drivers/media/artosyn/ar-csi2.c` |
| VIF front end, view arm, DMA | `overlay/drivers/media/artosyn/ar-vif.c` |
| DT nodes (camera, CSI, VIF, clocks, `isp_cma`) | `devices/betafpv-vr04-air/proxima-9311-air.dts` |
| Vendor MMIO write trace | `out/au-mmiotrace/mmio-combined.log`, capture harness `glue/dev/au-slotA-mmiotrace.sh`, shim `native/mmiotrace.c` |
| Vendor RE cross-reference | `archive/re/notes/nt99235/` |
