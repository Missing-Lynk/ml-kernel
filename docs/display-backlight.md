# Display and backlight

How the goggle's screen and backlight work, and how to bring them up on the open Linux 6.18 kernel as a DRM/KMS stack. Tags: **[confirmed]** (direct evidence), **[inferred]**, **[open]**.

## Architecture

```
framebuffer -> VO display controller (CRTC, 0x8810000, IRQ 102) -> DesignWare MIPI DSI host (0x8850000) -> QY45043A0 panel (1920x1080@60)
PWM backlight (pwmchip0 ch1, 0x1000000) -- lights the panel
```

The picture path is MIPI DSI. The DSI host is a **Synopsys DesignWare MIPI DSI** (mainline `dw-mipi-dsi`); the VO display controller is custom Artosyn. On stock these are driven from userspace (`ar_lowdelay` via `/dev/mem` + `libmpp_service.so` / `AR_MPI_VO_*`), with no vendor kernel driver.

## Backlight (PWM) [confirmed]

The backlight is `pwmchip0` channel 1 on the `artosyn,ar9301-pwm` controller. `customerHmLcdEnable(level)` @ `0x46da50` writes sysfs:

- `/sys/class/pwm/pwmchip0/pwm1/period` = `500000` ns (2 kHz)
- `/sys/class/pwm/pwmchip0/pwm1/duty_cycle` = `level*40000 + 100000` ns
- `/sys/class/pwm/pwmchip0/pwm1/enable` = `1`

`level` is 1..10 -> duty 140000..500000 ns = **28%..100%** (a hard ~28% floor). `customerHmLcdDisable` writes `duty_cycle=0`, leaves `enable=1`.

### PWM controller register map [confirmed]

recovered from the vendor kernel `artosyn_pwm_apply` / `artosyn_pwm_get_state`. Two controllers: `pwm@08000000` (reg `0x1000000`, `pwmchip0`) and `pwm@08040000` (reg `0x1002000`, `pwmchip8`), `compatible = "artosyn,ar9301-pwm"`, `#pwm-cells = 2`, 8 channels each, functional clock `host_ref` @ 150 MHz. Per channel N:

| field | offset | meaning |
|-------|--------|---------|
| OFF count | `N*0x14 + 0x00` | low time = (period - duty) in clk cycles |
| CTRL | `N*0x14 + 0x08` | enable = set bits 0,1,3 (mask `0xb`) |
| ON count | `0xb0 + N*0x04` | high time = duty in clk cycles |

count = `round(time_ns * clk_rate / 1e9)`; period = OFF+ON. Open driver: `modules/artosyn_pwm.c` (modern `.apply`/`.get_state`), DT `pwm@1000000` + a `pwm-backlight` node on `&pwm0 1 500000` with `brightness-levels = <28 36 ... 100>` (the vendor's `level*8+20` %). Config `display.config` (`PWM`, `BACKLIGHT_PWM`).

## Panel: QY45043A0 (MIPI DSI) [confirmed]

`customerHmLcdDetect` @ `0x46e2b8`, gated on HW-config string `mipi_dsi_lcd1` -> QY45043A0. Flow: `AR_MPI_VO_SetPubAttr` / `SetDevFrameRate(60)` / `VO_Enable` / `Dsi_SetAttr` (1920x1080@60; a value 1488 appears, likely DSI bit-rate Mbps), then `QY45043A0_power_on` (GPIOs), then a **519-entry DCS init table** sent via `vo_dsi_short_cmd`, then a panel-ID readback (DSI read reg `0x98` == `0x96`).

- **Init table** at `firmware/bin/ar_lowdelay` rodata `0x4c7a10`..`0x4c822c`, 4-byte tuples `{datatype, cmd, param, delay_ms}`: `0x15` = DCS short write 1-param, `0x05` = 0-param. Ends with `05 11 00 20` (Sleep-Out, 32 ms) and `05 29 00 28` (Display-On, 40 ms). Dump verbatim for the panel driver's `init_sequence`.
- Other board variants exist (`vo_mipi_dsi_device_init` @ `0x442938`): `mipi_dsi_lcd` -> `gm8773c` + `LT9711` (a MIPI bridge), `nd043fhdm30p`. Our unit is QY45043A0.

### Panel GPIOs [confirmed]

`QY45043A0_power_on` @ `0x442830` drives two lines on `a10a000.gpio` (the `artosyn_gpio` controller we already have):

- Global GPIO **43** = gpiochip1 line 20 (bank base 23), reset/enable (dropped on power-off).
- Global GPIO **95** = gpiochip6 line 1 (bank base 94), second rail/enable.
- Sequence: `gpio95=0; gpio43=0; 10ms; gpio43=1; 10ms; gpio95=1; 10ms`. Power-off: `gpio43=0`.

(GPIO 100 = gpiochip6 line 6 is also `out hi` at boot but not touched here; it is the panel-power line whose rising edge samples the scan-direction straps, see below.) The open `artosyn_gpio` driver must register banks in the same order/sizes (23/22/26/6/6/11/16) so globals 43/95 map correctly.

### Panel orientation (180 flip) [confirmed]

The panel comes up rotated 180 unless two power-on straps are driven low: **gpio 42** (horizontal/source scan) and **gpio 107** (vertical/gate scan), sampled when panel-power (gpio 100) rises. It is not a DCS command or a DC register, and the DC does not rotate (stock's scanout path is pure identity). Stock drives both straps low; the open kernel left them floating, so both axes came up reversed = the full 180. Fix: two `output-low` gpio-hogs (`panel-scan-h-hog` gpio 42, `panel-scan-v-hog` gpio 107) placed before `panel-power-hog` in `dts/proxima-9311.dts`. Proven on hardware.

## DSI host = DesignWare [confirmed]

`libmpp_service.so` `dsi_short_cmd_1pra` @ `0x2f6e80` writes the DCS command header to DSI register offset **`0x6c`** = DesignWare `GEN_HDR`; payload at `0x70` = `GEN_PLD_DATA`. So the DSI host @ `0x8850000` is a **Synopsys DesignWare MIPI DSI host** -> use the mainline `dw-mipi-dsi` bridge driver + a small Artosyn glue (base, D-PHY ops, clocks, lane count). The D-PHY is configured by `dphy_freq_conf_*` / `dphy_get_config_value` in `libmpp_service.so`.

## VO display controller (CRTC) [confirmed]

The VO @ `0x8810000` (IRQ 102 = GIC_SPI 70) is the custom display controller (scanout/timing/format). On stock it is driven only by `AR_MPI_VO_*` / `libmpp_service.so` via `/dev/mem`, with no mainline driver. Its register map (framebuffer base, 1920x1080@60 timing, pixel format, scanout enable, vsync IRQ) was recovered from `libmpp_service.so` and is implemented as a real DRM CRTC/KMS in `../overlay/drivers/gpu/drm/artosyn/artosyn_vo.c`.

## Open-kernel DRM/KMS stack [confirmed]

The whole pipeline runs on the goggle as a real DRM/KMS stack, hardware-validated. Sources in `../overlay/drivers/gpu/drm/artosyn/` (`Kconfig`, `Makefile`); see `../PERIPHERALS.md`.

- **Backlight**: `../modules/artosyn_pwm.c` + DT + `../configs/display.config`, driven via `pwm-backlight`.
- **DSI host**: mainline `dw-mipi-dsi` + Artosyn glue `artosyn_dsi.c` (D-PHY from `dphy_freq_conf`).
- **Panel**: `panel-qy45043a0.c` (`panel-qy45043a0.h`), GPIO 43/95 power sequence + the 519-cmd init table.
- **VO CRTC**: `artosyn_vo.c`, a DRM CRTC (VO register map recovered from `libmpp_service.so`) driving primary + overlay planes over the standard DRM/KMS ABI.

The register-level sections above are the RE record of how each block was recovered.

## Beyond a faithful reimplementation

Two places where the open pipeline does more than the stock userspace path. On stock the kernel is deliberately dumb: a userspace CUSE compositor (`ar_overlay`) plus userspace panel/VO bring-up (`ar_lowdelay_rx_vo`) mean the panel only lights once binder, servicemanager, and the VO daemon are running, so there is no early Linux console. The open stack moves both into the kernel.

### Kernel framebuffer [confirmed]

The in-kernel display pipeline (`artosyn_vo` CRTC, `artosyn_dsi` glue, `panel-qy45043a0`; see `../PERIPHERALS.md`) lights the panel and scans out independently of any userspace VO daemon. The VO DRM driver exports fbdev emulation (`DRM_FBDEV_DMA_DRIVER_OPS` + `select DRM_FBDEV_DMA`, XRGB8888 = Vivante X8R8G8B8 format 5; `VO_1080P60_FB_CONFIG` in `artosyn_vo.c`). fb0 reaches the panel through DRM, and a Linux VT/console and panic-on-screen render via fbcon over the DRM fbdev, with no dependency on the closed compositor.

There is no u-boot-provided framebuffer: u-boot does not set up the panel, so nothing is on screen until the DRM driver probes during kernel init. There is no pre-kernel boot splash; the earliest the panel can show anything is once `artosyn_vo` has come up.

The separate `../modules/ar_framebuffer.c` is an open reimplementation of the vendor OSD framebuffer (allocates the `fb_mmz` buffer, registers fbdev, forwards flips to the vendor `ar_overlay` CUSE compositor via the stubbed `ar_overlay_xfer()`). Because that transport is a stub, its `/dev/fb0` reaches nothing on the open stack: it has no scanout path. It is reference only - compile-checked, not shipped or loaded at boot (`../modules/README.md`).

### Vsync from hardware, not a poll [confirmed]

Vsync comes from the VO frame-done interrupt (IRQ 102 = GIC_SPI 70), not the stock `ar_mpp_drv` `nr7` 100ms MMIO poll. `artosyn_vo.c` hooks `enable_vblank`/`disable_vblank` to `VO_INT_ENABLE` (`0x1480`, `VO_INT_VSYNC` = BIT(0)); its `ar_vo_irq` handler acks the status and calls `drm_crtc_handle_vblank`, feeding standard DRM vblank events and page-flip completion.

## Source

Disassembly of the vendor `ar_lowdelay` and `libmpp_service.so`, the reconstructed vendor kernel, the vendor DT, and live captures.
