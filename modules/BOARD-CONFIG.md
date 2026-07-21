# Board configuration (open Artosyn modules)

These modules and their bring-up tools target the **BetaFPV VR04 HD** goggle (Artosyn **Proxima-9311** SoC + **AR8030** RF link). The settings below are **board-specific**: another product built on the same SoC and RF module may wire them differently. The most board-variable item is the **AR8030 reset GPIO**, whichever line the board designer routed to the chip's reset pin; on the VR04 it is GPIO23, and driving it correctly is what brings the RF chip out of reset (without it the chip is SDIO-silent).

## Where these live

- **Runtime source of truth = the Device Tree.** The modules read their register bases, IRQs, the MMZ carveout, etc. from DT nodes, not from hardcoded constants, so retargeting a board is (mostly) a `.dts` change, not a code change.
- Everything needed to port a board lives in the table below and the DT.

## The settings

| Setting | VR04 / Proxima-9311 value | Production home (DT) | Notes |
|---|---|---|---|
| MMZ media carveout | phys `0x29200000`, size `0x06E00000` | reserved-memory `mmz@29200000` (nomap) | the wave5 codec pool (bound per-device via of_reserved_mem, page-granular allocator) |
| GPIO controller | `artosyn,gpio` @`0x0A10A000`, 7 banks (sizes 23/22/26/6/6/11/16) | gpio node `artosyn,gpio` + `arto,gpio-port` children | per-bank reg block at base+`0xBC`+N*`0xC`; SET@`+0` / DIRIN@`+4` (1=input) / DAT@`+8`. The bank base is `0xBC`, not `0xC0`; see `../docs/artosyn-gpio.md`. |
| **AR8030 reset** | **GPIO23 = bank1 line0**, active-low (drive output low->high to release) | `reset-gpios` driven before the SDIO scan | **most board-variable**; if not driven, the chip stays held in reset and never answers on SDIO |
| SDIO host (AR8030) | `dw_mmc` @`0x01B00000`, IRQ GIC-SPI 48 | mmc node `dwmmc0` / `artosyn,proxima-dw-mshc`, `cap-sdio-irq`, `broken-cd` | the RF bus; SDIO id `4152:8030` (ROM) / `4152:8031` (firmware running) |
| SDMMC clock | mmc0 SEL @`0x0A108088` / CFG @`0x0A10808C`; mmc1 (SD) SEL @`0x0A1080C0` / CFG @`0x0A1080C4` | `dw_mci-artosyn` glue (`clk_sel`/`clk_cfg` debug params) | the hardware-validated working pair is `clk_sel=0x87`/`clk_cfg=0x02` (135/2), passed by the shipped modprobe config; the values originated from misread slot-A register dumps, and whether to bake them in as the driver default is unsettled. CGU source-clock gates @`0x0A104024` bit22 / `0x0A104028` bit23. SEL tap table, phase encoding, and the never-write register rules: `../docs/sd-card.md`. |
| RF firmware | `bb_demo_gnd_d.img` + `bb_config_gnd.json` | `artosyn_sdio` `fw_name`/`cfg_name` params | the RF baseband blob (stays binary; NOT `chagall.bin`, the Wave5 codec firmware, see `../STATUS.md`); open `artosyn_sdio` uploads it via the ROM loader -> chip re-enumerates `0x8030`->`0x8031` and `sdio0` comes up |
| Framebuffer | 1920x1080 ARGB4444, stride 4096 | (panel + overlay) | display geometry |
| Scaler | regs @`0x08840000`, ctrl @`0x0A100000`, IRQ 107 | `scaler@8840000` | |
| MPP engines | @`0x08870000`, IRQs GIC-SPI 68/69/79 | `ar_mpp@8870000` | h26x/jpeg/ge2d |

## RF firmware deployment

`artosyn_sdio` pulls `fw_name`/`cfg_name` via `request_firmware`. The ubifs rootfs `/lib/firmware` is full, so stage the blobs on tmpfs and point the firmware search path at them: `mkdir -p /run/ml/fw; cp bb_demo_gnd_d.img bb_config_gnd.json /run/ml/fw/; echo /run/ml/fw > /sys/module/firmware_class/parameters/path`. Full bring-up sequence and validation signals (device-ID `0x8031`, `sdio0`, board current +130 mA) are in `HW-BRINGUP.md` Phase 6.

## Porting to another Artosyn board

1. Update the board `.dts`: the reserved-memory carveout, the gpio/mmc/clock/scaler/mpp nodes, and the `reset-gpios` for the AR8030 (the table above lists every board constant to retarget).
2. The module C code should not need changes - if it does, that constant was wrongly hardcoded and belongs in DT (and the table above).
