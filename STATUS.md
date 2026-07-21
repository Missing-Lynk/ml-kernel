# Kernel status: the single progress table

The one place kernel-side progress is tracked. Every other doc under `kernel/` keeps its own explanatory/how-it-works content (architecture, register-level RE, bring-up recipes) but defers to this table for current status - update progress here, not there.

Tags used below: **DONE** (working, hardware-validated) - **PARTIAL** (works, with a known real gap) - **NOT DONE** (open work item).

## SoC peripherals

| Item | Status | Notes |
|---|---|---|
| Status LED | DONE | `docs/status-led.md`, `test_tools/led_test.c` |
| Front-panel buttons | DONE | `docs/artosyn-adc.md`, `test_tools/button_test.c` |
| SoC temperature sensor | DONE | absolute-accuracy cross-check vs stock still pending; `docs/artosyn-protemp.md`, `test_tools/temp_read.sh` |
| Buzzer + LCD backlight | DONE | `docs/buzzer.md`, `docs/display-backlight.md`, `test_tools/buzzer_test.c` |
| Display controller (VO + MIPI-DSI + panel) | DONE | `docs/display-backlight.md`, `test_tools/{display_test,display_bounce,overlay_test,display_demo}.c` |
| RTC (DS1307 on i2c0) | DONE | mainline-only path (`snps,designware-i2c` + `dallas,ds1307`, verbatim from the vendor DTS minus its `resets` phandle; `configs/i2c-rtc.config`); this is the vendor's own RTC path (it never binds the SoC-internal `rtc@0a10c000`). Registers rtc0, hctosys sets the clock at boot, keeps and ticks wall time. Caveat: the backup battery is currently removed from this unit, so time resets on power-off (hardware, not driver) |
| SD card (microSD, `mmc1`) | DONE | reads and writes validated (`test_tools/sd_rwtest.c`); ~21-22 MB/s R/W = the SD-High-Speed 50 MHz/4-bit bus limit (UHS needs 1.8 V, vqmmc is fixed 3.3 V, same mode as stock); exFAT built in (`configs/exfat.config`) and validated under a DVR-style workload. `docs/sd-card.md` |

## RF chip (AR8030)

| Item | Status | Notes |
|---|---|---|
| SDIO enumeration, firmware upload, RF-link association | DONE | device flips `0x8030`->`0x8031`, `sdio0` up, associates with a paired air unit; `docs/artosyn-sdio.md`. ABI-level module status: see `artosyn_sdio` below. |
| Full-rate video downlink | DONE | the air unit streams H.265 to the open slot-B stack, **~1.5 MB/s sustained** on `sdio0` `:10001`; the gating chain (poll cadence + TX power `type:8` + the UDP `:10000` params handshake) is the product story in `../docs/reference/rf-video-downlink.md`. |
| Live RF video through the kernel UDP stack (end-to-end) | DONE | the live stream runs `sdio0` -> IP -> UDP at rate; ~1.6 MB/s clean (InAddrErrors 0, ReasmFails 0) after the `artosyn_sdio` RX resync fix (`docs/artosyn-sdio.md`), and the full gated chain puts 60 fps on the panel. |

## RF module (`modules/`)

| Module | Status | Notes |
|---|---|---|
| artosyn_sdio | DONE | see "RF chip" above - core link and full-rate video DONE. Instrumented (`frame_log`, `tx_window`; the `tx_window` gate defaults off). RX resync fix landed (split-header carry + header plausibility check + multi-run back-parse); mechanism in `docs/artosyn-sdio.md` |

The MPP-stack vendor-`.ko` reimplementations (`ar_osal`, `ar_sys`, `ar_sysctl`, `ar_mpp_drv`, `ar_scaler`, `ar_vb`, `ar_mpp_proc_ctrl`, `ar_mpp_overlay`) are **reference only** since the 2026-07-12 pivot: in-tree and compile-checked, but not shipped in the rootfs or loaded at boot. Rationale: the pivot note in `modules/VERIFICATION.md`; per-module detail: `modules/README.md`.

## Kernel-level

| Item | Status | Notes |
|---|---|---|
| Reproducible build | DONE | `README.md` |
| Boot, both A53 cores (SMP) | DONE | `ar-spin-table`; `README.md` "SMP" |
| CPU PMU / perf counters | DONE | `arm,cortex-a53-pmu` DT node (per-core overflow SPIs 109/110, from the vendor DTS); mainline ARM_PMUV3 driver. Verified by `test_tools/pmu_test.c`: exact instruction counting, zero multiplexing, per-core overflow-IRQ routing clean. Measured core clock **800 MHz on both cores** = the vendor's top OPP |
| Clock controller (CGU), read-only tree | DONE | `drivers/clk/clk-ar9311-cgu.c`: all 49 leaves + sources registered read-only, pixel PLL keeps its validated set_rate; clk_summary matches the register decode on all 49 rows, `clk_disable_unused` provably touched nothing, and display/panel/wave5/SD/RF-SDIO are all healthy on top of it. `docs/clocks.md` |
| DVFS/cpufreq | DONE (benchmark pending) | settable `cgu_cpu_clk` + the vendor OPP table verbatim + `cpufreq-dt` (`configs/cpufreq.config`); all 8 OPPs PMU-verified on both cores, default governor performance = vendor parity. Remaining: the ROADMAP benchmark-then-decide pass (idle heat) and selective gating of unused clock domains. `docs/clocks.md` |
| Hardware watchdog | DONE | mainline `dw_wdt`: probes, kernel auto-feed (`WATCHDOG_HANDLE_BOOT_ENABLED`), userspace supervision via `/dev/watchdog` (open/ping/magic-close round-trip), and expiry resets the box. The DT pins `snps,watchdog-tops` to the proven standard `2^(16+i)` values (the block is synthesized without fixed TOPs, so this silences the cosmetic warning without changing behavior) |
| Rootfs slim + flavors | DONE | `/tmp` is a 32 MB tmpfs (`rootfs/skeleton/etc/fstab`), fixing the fills-persistent-flash problem, and the build grew `FLAVOR=dev\|slim` (`rootfs/build.sh`): base is 5 packages (alpine-base busybox openrc dropbear iproute2; busybox applets replace util-linux/less), `dev` layers on scp/sftp + util-linux + strace/tcpdump/htop, `slim` ships the base only. QoL in both flavors: `ml-info` login banner + slot-aware prompt (`/etc/profile.d/10-ml.sh`), a getty on the debug UART (`/etc/inittab`), and an offline-safe one-shot NTP sync (`/etc/init.d/ntp-oneshot`). Booted on HW (audited boot: UBI/UBIFS on userapp1, gadget + SSH up) |
| Boot dmesg audit (kernel-adaptations sign-off) | DONE | full review of the boot log: no unexplained error/warning; every flagged line explained benign. UBI reports 2 bad PEBs (normal NAND, 18 reserved), watch only if it grows. |
| Flash current kernel to slot B (cold-boot default) | NOT DONE | RAM-boot proven, not yet flashed; see `README.md` "Flash" |

## Kernel parity vs the vendor kernel

Everything the goggle exercises is at parity or better on the open kernel. Hardware blocks the vendor kernel serves that the open kernel does not drive (none affect current function):

| Block | State | Consequence |
|---|---|---|
| Scaler (crop/resize engine) | not driven (reference driver `modules/ar_scaler.c`) | no 720p->1080p upscale or digital zoom; unneeded at the native-1080p link mode (compositing runs on the DMA engine) |
| JPEG codec (Chips&Media JPU, `0x08830000`, SPI 69) | not driven, scoped | standalone single-instance JPU, register-driven, no firmware; no mainline driver fits, open path = small custom driver from the open `jpuapi` choreography (ml-kernel#1) |
| USB gadget MTP function | not configured (open gadget is CDC-ECM only; vendor is rndis+mtp+uart) | DVR files not pullable over USB; SD card removal or scp instead |
| Audio codec (`artosyn,audio_codec`, GIC 79) | not driven | none - disabled in this unit's stock config (`AudioEnable:0`) |

Vendor behaviors replaced by different (mainline) means, not missing: fb0-composited-over-video -> DRM ARGB4444 overlay plane; the MPP MMZ/VB kernel plumbing -> per-device coherent pool + vb2/dma-buf.

## Open codec (V4L2, the open in-kernel codec)

The vendor codec is a **Chips&Media WAVE521C** (H.264/H.265 encode+decode) plus a CODA9/JPU (JPEG), byte-proven from the vendor `libmpp_service.so` disassembly. The open path adapts the mainline `wave5` V4L2 driver rather than reversing a bespoke codec. This is the only codec path; the vendor-bin MPP port is retired (see above).

| Item | Status | Notes |
|---|---|---|
| Codec IP identification | DONE | Chips&Media WAVE521C / CODA9, byte-proven from `libmpp_service.so` |
| Firmware blob extraction (VCPU ucode) | DONE | it's `/usr/bin/chagall.bin` (the codec fw, NOT RF - RF is `bb_demo_gnd_d.img`), 999616 B; the wave5 driver loads it from `/lib/firmware/` |
| Config fragment (`MEDIA_SUPPORT`+`V4L2_MEM2MEM`+`VIDEO_WAVE_VPU`) | DONE | `configs/codec.config`: media core `=y`, `VIDEO_WAVE_VPU=m` (module, so the Image fits the 6 MiB slot); registered in `container-build.sh` after `trim` |
| DT node for wave5 (`0x0a080000`+SRAM, SPI 68) | DONE | `devices/betafpv-vr04-goggle/proxima-9311.dts`: `video-codec@a080000` (`ti,j721s2-wave521c`) + `mmio-sram@1f0000` |
| Dedicated codec memory pool (the DTS mmz reserved-memory node) | DONE | `memory-region` on the codec node + `of_reserved_mem_device_init` in probe, so codec buffers stop fragmenting the 64 MiB CMA (the display shares it) |
| Artosyn power-on glue (venc power domain + clocks) in `probe()` | DONE | Recovered and validated: CRG `0x0a106000` (gate/PMU) + isolation `0x0a102000`; the sequence lives in `patches/0008` |
| Reset-path fix (backbone bus-idle `0x3f` for this WAVE521C) | DONE | `wave5-hw.c` overlay; mainline `0x00ff1f3f` hung reset on this silicon |
| **Driver probes + registers on hardware** | **DONE** | cold boot: `Product Code 0x521c`, `Firmware Revision 329715`, caps ENCODE+DECODE; `/dev/video0` (decoder) + `/dev/video1` (encoder) |
| Functional decode - HEVC decodes | DONE | on HW: source-change + GIC-100 IRQ + DEC_PIC complete; the enabling fixes (power sequencing, keep-awake PM, decoder-capability enable) are `patches/0004`..`0008` (`patches/README.md`) |
| Functional decode - clean linear (WTL) output | DONE | vb2 CAPTURE frames are **pixel-perfect**; the WAVE521C fixes (sec-AXI forced off, `reorder_enable=FALSE`, FBC recon height `ALIGN(h,64)`) are in the patches (`patches/README.md`) |
| Decode output - byte-exact | DONE | **bit-exact vs ffmpeg** (mad 0.000 on all 3 planes, multiple resolutions, stable across repeated decoder instances) |
| Decoder capability matrix (codecs / resolutions / rates) | DONE | **HEVC** at 640x360, 1280x720, 1920x1080 and **H.264** at 1280x720, all bit-exact; **323 fps @ 720p / 165 fps @ 1080p**. Full capability reference incl. the decoded GET_VPU_INFO bits, the 8-bit-only feature set, and the 1080p memory budget: `docs/wave5-codec-capabilities.md` |
| Functional encode (WAVE521C -> HEVC/H.264) | DONE | all 4 combos verified by host-ffmpeg decode + PSNR vs source: **HEVC and H.264 at 1280x720 AND 1920x1080, 42-48 dB PSNR every plane**; **171 fps @ 720p / 108 fps @ 1080p**. The two encode fixes (sec-AXI off, `finish_encode` error-path job-finish) and the userspace buffer contract: `docs/wave5-codec-capabilities.md`. DVR integration DONE (concurrent-fit row below); RTSP: future work |
| Flash-prep: convert codec to module (`=m`), fw on rootfs | DONE | `VIDEO_WAVE_VPU=m`, embedded fw dropped; Image fits the 6 MiB slot (96.9%). `wave5.ko` + its v4l2/videobuf2 deps staged by `modules/build.sh`; `chagall.bin` installed on the rootfs at `/lib/firmware/cnm/` by `rootfs/build.sh`. Validated: `modprobe wave5` brings the codec up + decodes on HW |
| Decode -> display (dma-buf to DRM YUV plane) | DONE | validated end to end via GStreamer: H.264/H.265 file -> wave5 decode -> dma-buf export -> PRIME import -> DRM I420 primary, **1080p60 at measured 60.00 fps / 0 drops, zero-copy**; no `ar_scaler` hop needed at native res (the DC scans the decoder's 64-aligned luma stride; `artosyn_vo` dumb-pitch is 64-px for 8-bpp). Tooling + full gotcha list: `userspace/gstreamer/README.md` |
| Concurrent 2x decode + 1080p60 encode fit (DVR) | DONE | the vendor-parity load (two RF-tile decoders + the 1080p60 H.264 DVR encoder) fits the MMZ pool via `patches/0011-dma-coherent-page-granular.patch` (page-granular first-fit instead of power-of-2 rounding). HW-validated 2026-07-12: 60 fps composite, 0 drops, playable 1920x1080@60 MP4. `dec_cap_bufs` must stay 0 (`docs/wave5-codec-capabilities.md`). Kernel1 flash pending |
