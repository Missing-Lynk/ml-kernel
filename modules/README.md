# Open Artosyn kernel modules

Out-of-tree modules for the open 6.18.36 kernel. The RF driver and the board-peripheral glue live here; display (DRM) and the codec (wave5) are in-tree (`../overlay/`, `../patches/`).

The MPP-stack reimplementations (`ar_osal`, `ar_vb`, `ar_sys`, `ar_sysctl`, `ar_mpp_drv`, `ar_mpp_proc_ctrl`, `ar_scaler`, `ar_mpp_overlay`) are **reference only**: compile-checked, not shipped in the rootfs, not loaded at boot (full rationale: the pivot note in `VERIFICATION.md`; per-module detail: the "Reference" section below).

## Build (out-of-tree, dev iteration)

```sh
modules/build.sh         # compiles all *.ko against the built 6.18 tree
modules/build.sh clean
```

Needs `scripts/build.sh` to have produced `$BUILD_DIR/linux` first. The script seeds `Module.symvers` from the tree's `vmlinux.symvers` (MODVERSIONS is off, so a copy is exact) and stages the shippable set (these modules + the in-tree DRM/wave5/v4l2 `.ko`) under `$BUILD_DIR/ml-modules/rootfs/`.

Companion docs:
- **`HW-BRINGUP.md`** - the RF bring-up procedure (reset release, firmware upload, association) and the serial-capture/build-match safety rules.
- **`BOARD-CONFIG.md`** - the VR04 board constants (GPIO map, SDIO clock taps, MMZ carveout, RF firmware) and how to port to another Artosyn board.
- **`KERNEL-REQUIREMENTS.md`** - what the kernel config/DT must provide for these modules to load and be match-buildable.

## Modules

| module | /dev or interface | notes |
|---|---|---|
| artosyn_sdio | `/dev/artosyn_sdio` + `sdio0` netdev | the AR8030 RF link: firmware uploader (`fw_name=`/`cfg_name=` via request_firmware; ROM `0x8030` -> running `0x8031`), bb_ioctl passthrough (`'v'`), RX reassembly with the split-header/desync fix, TX IP-header compression. NEVER warm-reload (hangs the device). `../docs/artosyn-sdio.md` |
| dw_mci-artosyn | mmc hosts | DesignWare MMC glue for the Proxima clock-tap/phase registers; binds the DTS `mmc@1b00000`/`mmc@1c00000` nodes param-less (defaults SEL 0x80/phase 0, the stock-faithful register-diff values; `clk_sel`/`clk_cfg`/`bus_hz` are debug knobs). `../docs/sd-card.md` |
| artosyn_gpio | gpiochips | the 7-bank GPIO controller (bank reg base `0xBC`); provides the AR8030 reset line and the panel reset (display defers without it). `../docs/artosyn-gpio.md` |
| artosyn_adc | IIO | SAR ADC for the button ladder + battery voltage (`adc-keys` consumes it). `../docs/artosyn-adc.md` |
| artosyn_protemp | IIO | SoC temperature sensor (shares the ADC MMIO window). `../docs/artosyn-protemp.md` |
| artosyn_pwm | pwmchips | PWM controller: LCD backlight (via mainline `pwm-backlight`) + buzzer. `../docs/display-backlight.md`, `../docs/buzzer.md` |
| artosyn_mmc | mmc host | standalone DesignWare MMC host reimplementation (alternate to the dw_mci glue path) |
| ml_dmablit | `/dev/ml-dmablit` | char shim exposing the built-in dw-axi-dmac copy engine as batched dmabuf memcpy - the off-CPU two-tile compositor (REQUIRED for 60 fps composite). Engine register protocol recovered from the vendor driver; test: `../test_tools/ml_dmablit_test.c` |

## artosyn_sdio design notes

- The `chagall`-class baseband firmware (`bb_demo_gnd_d.img`) stays a closed blob - genuinely irreducible vendor IP. It is NOT `chagall.bin` (the Wave5 codec firmware; see `../STATUS.md`).
- The in-kernel video-packet classification (`video_packet_header_t` parse, TX IP-header compression) is link-specific framing the vendor put in-kernel for throughput; kept byte-compatible.
- The `'v'` bb_ioctl (opcode/arg/resp passthrough, register peek-poke, mailbox) is a vendor control ABI, not 802.11 - mac80211 does not apply.

## Reference: the MPP-stack reimplementations (not built into the shipped rootfs)

Kept in-tree after the pivot (see `VERIFICATION.md`'s pivot note) - complete, reviewed reimplementations of the vendor MPP kernel ABI, available if this stack is ever integrated properly. Load order and parameters: `load.sh`.

| module | /dev node | notes |
|---|---|---|
| ar_scaler | arscaler | 'Z' ABI + **full register choreography recovered**: real 1024B LUT, exact Q16 ratio/delta math, descriptor-at-regbase+0x1C map, clock sequence, freq-divider table. DT node `artosyn,scaler@8840000`. Byte-pinned from the `.ko`; oracle for per-bit semantics: /proc/arscaler/state. |
| ar_vb | ar_vb | pool bookkeeping over MMZ. Per-command nrs + field offsets + the `(pool_id<<16)\|index` handle encoding byte-exact from libhal_vb.so/ar_vb.ko. |
| ar_mpp_proc_ctrl | ar_mpp_proc_ctl | /proc/umap shuttle. 144/280/24-byte structs pinned; the WRITE<->show data path is real. |
| ar_mpp_overlay | - (no /dev; runtime DT overlay) | injects the `ahb_dma`(8ch)/`axi_dma`(3ch) child nodes `ar_mpp_drv` expects under `ar_mpp` at load time, since closed `libmpp_service`'s `SYS_Init` asserts exact IRQ counts. A reflash-free bring-up helper, ahead of a real DT fix; load before `ar_mpp_drv`. Not one of the 9 vendor-`.ko` reimplementations below. |
| ar_osal | mmz_userdev | MMZ allocator + 'm/r/c/d/i/t' ioctls + mmap + /proc/media-mem; exports the hil_* API. WC carveout done; cached fast-path still TODO. |
| ar_sys | ar_sys | PTS/timezone/GPS/flush ('y'/'p') - faithful to §E. loglevel subset stubbed. |
| ar_sysctl | ar_sysctl | pure-SW 7-ioctl priority/suspend arbiter. |
| ar_mpp_drv | ar_mpp_ctl | engine-agnostic GIC-IRQ forwarder. DT node `artosyn,ar_mpp@8870000`. Must load `ar_mpp_overlay` first (above) so the `ahb_dma`/`axi_dma` IRQ-count children it expects exist. |
| ar_cipher | - | DROPPED, unused at runtime/OTA (parent-project RE decision). |

All 8 vendor-`.ko` reimplementations (plus the `ar_mpp_overlay` bring-up helper) compile clean (zero warnings, warnings-as-errors), link to `.ko`, all undefined symbols resolve against vmlinux + ar_osal, and the inter-module `depends=` (ar_vb/ar_sys/ar_scaler -> ar_osal) are recorded. DTB compiles. (`ar_framebuffer`, the vendor OSD fbdev whose CUSE transport was a permanent stub, is removed: the HUD renders on the DRM overlay plane instead.)
