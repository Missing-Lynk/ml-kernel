# Clocks (AR9311 CGU: read-only tree + settable CPU and pixel clocks)

How the open kernel provides the AR9311 clock generation unit (CGU) as a common-clock-framework (CCF) provider. One built-in driver, `../overlay/drivers/clk/clk-ar9311-cgu.c`, registers the whole clock tree **read-only** and exposes exactly two settable clocks: the CPU clock (for cpufreq/DVFS) and the DC pixel PLL (for display frame pacing). Each fact is tagged **[confirmed]** (direct evidence in the code, DT, or on hardware), **[inferred]**, **[open]**.

## Architecture

```
artosyn,ar9311-cgu (clock-controller@a100000, DT label `pixclk`, 3 MMIO windows)
  read-only:  49 leaf clocks (3-bit parent mux + gate) over the fixed source PLLs
  settable 1: cgu_cpu_clk   @0x0A100008 (+ SPL mirror 0x0A100408)  -> cpufreq/OPP
  settable 2: pixel PLL     @0x0A108000 +0x410 (period word)        -> VO pclk pacing
```

The whole tree is registered with get_parent/is_enabled only (every clock `CLK_IGNORE_UNUSED`), so `clk_disable_unused` never touches anything and no clock is reparented or re-rated behind the vendor's back. Two write paths are the deliberate exceptions.

## Vendor baseline [confirmed]

The vendor kernel's own clk driver is minimal: it registers the fixed source PLLs plus a single composite (the A53 CPU mux+divider) in a small provider whose only populated slots are the CPU, and it exists solely to feed cpufreq. Every other clock (display, codec, camera, MMC taps) is driven from closed userspace (`libmpp_service.so` via `/dev/mem`) or left at its SPL default. So vendor-kernel parity is just the CPU clock; the read-only tree and any gating the open driver adds exceed the vendor deliberately.

## The provider [confirmed]

`compatible = "artosyn,ar9311-cgu"`, node `clock-controller@a100000` (DT label `pixclk`, kept for phandle stability), `clocks = <&clk24m>`, `#clock-cells = <1>` (`../dts/proxima-9311.dts`). Three reg windows, each mapped with `devm_ioremap` (non-exclusive, because the ADC and protemp drivers own parts of `0x0A108000`):

| Bank | Base | Size | Holds |
|------|------|------|-------|
| 0 | `0x0A100000` | `0x20` | `cgu_cpu_clk` (+0x08), `cgu_cs_dbg_clk` (+0x10) |
| 1 | `0x0A104000` | `0x210` | the 49 leaf mux/gate bank |
| 2 | `0x0A108000` | `0x420` | pixel PLL (power +0x00, range +0x98, feedback word +0x410) |

The provider publishes `ARRAY_SIZE(leaves) + 3` clocks: slot 0 = the DC pixel leaf `cgu_dvp_sub_1_2x_pix_clk` (the VO's `pclk`), slot 1 = the `pixel_pll` (debug), slot 2 = `cgu_cpu_clk`, slots 3.. = the 49 leaves. The fixed source taps (`fix_pll_clk*`, `adc_pll_clk*`) and rate-0 placeholders for the sd/emmc/audio taps model the tree the SPL set up.

### Read-only leaf model [confirmed]

A leaf is a 3-bit parent **mux** with a gate, no divider (leaf rate = the selected parent's rate). The low half uses sel bits[10:8] / gate bit12; the high half uses sel bits[26:24] / gate bit28; the dual-bank leaves (axi/sys/npu/cpu) use both halves with bit31 selecting the active bank. `get_parent` and `is_enabled` decode these; nothing is written.

## CPU clock (cpufreq/DVFS) [confirmed]

`cgu_cpu_clk` at `0x0A100008` is a settable composite mirroring the vendor `artosyn_composite_set_rate`: register `0x0A100008` and its SPL mirror `0x0A100408` are kept in lockstep, and a rate change is a glitch-free two-write sequence (program the idle bank including its gate, barrier, then flip the active-bank/bypass bits). bit30=0 selects the 24 MHz bypass; the divider is a 7-bit field+1 (max 128). It carries `CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED`. No cpufreq platform device is registered by hand: `cpufreq-dt-platdev` auto-registers it, and the CPU DT nodes carry `clocks = <&pixclk 2>` + `operating-points-v2` (the vendor OPP table verbatim). Config fragment `../configs/cpufreq.config`.

## Pixel PLL (display frame pacing) [confirmed on hardware]

The DC pixel clock is steered to pace the panel refresh. `artosyn_vo.c` holds the pixel leaf as `pclk` and calls `clk_set_rate` on it; the rate propagates through the read-only mux to the PLL. The rate model is a **period word**, not a multiplier: `rate = parent_rate * KNUM / word` (bigger word = slower clock), with `KNUM` calibrated so the leaf rate is the true pixel clock through the /4 tap. The stock feedback word `0x0592eab1` back-computes to exactly 148.5 MHz (60.000 Hz), which confirms the whole model.

Two safety mechanisms wrap the one writable register (the feedback word at `0x0A108000 + 0x410`):

- **Clamp** to +/-5% of the boot-time word.
- **Slew limiter**: steps of word/2048 (~0.05%) one per ~16 ms (a frame interval). A single step >= ~0.2% fires a DSI DPI-FIFO error (a transient the DSI `INT_ST1` bit7 latch catches; the panel can visibly drop sync); <=0.05% steps are transient-free. Only the PLL word is ever written; there is no relock kick.

The pixel leaf itself is a mux+gate registered with `clk_mux_ro_ops` (read-only) at `0x0A104000 + 0x4c`; the fixed-factor `pixel_pll_clkN` taps are /1, /2, /4, /8 (only /4 is validated). A debugfs knob `/sys/kernel/debug/ar9311_pixclk_rate` reads the current leaf rate and writes a `clk_set_rate` (mainline's writable clk debugfs is compiled out on this kernel). Measurement tool: `../test_tools/vblank_rate`.

## Consumers

- **VO** `vo@8810000`: `clocks = <&pixclk 0>` (the DC pixel leaf). `artosyn_vo.c` enables it and steers scanout with `clk_set_rate`.
- **CPU** nodes: `clocks = <&pixclk 2>` (`cgu_cpu_clk`) + the OPP table.
- **DSI** still uses a `clk150m` fixed-clock stand-in for its APB pclk, not the CGU (a standing TODO).

### SDMMC gates: not yet routed through the CGU [open]

`../modules/dw_mci-artosyn.c` enables the SDMMC source-clock gates by poking `0x0A104024` (bit 22) and `0x0A104028` (bit 23) directly, because it predates this provider (its comment says "our open kernel has no CGU clock driver"). The CGU driver already models those same registers as leaf gates, but with different bit positions (12/28, the low/high-half gate bits), so the two do not agree and there is no CCF handoff between them. Reconciling this (SD consuming the CGU clocks instead of poking gates) is the natural consumer-migration follow-up.

## Source

`../overlay/drivers/clk/clk-ar9311-cgu.c` (the whole tree, CPU and pixel write paths), the DT node in `../dts/proxima-9311.dts`. The register map was recovered from the vendor `libmpp_service.so`.
