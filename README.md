# Modern kernel for Artosyn Proxima-9311 devices (reproducible build)

Linux 6.18.36 for the Artosyn Proxima-9311 SoC + AR8030 RF link: an open, modern kernel driving the display (DRM/KMS), codec (wave5 V4L2), RF link, and board peripherals with no vendor userspace. Naming: `AR9311` in driver names is this same SoC (the Proxima-9311), e.g. `clk-ar9311-cgu`; `AR9301` names its QSPI-NAND controller IP (`spi-ar9301`). Device-neutral: the same SoC/RF chip pair shows up across goggle, VRx, VTx, and air-unit products; this repo's hardware validation happens to be on a goggle (BetaFPV VR04 HD), but nothing here is goggle-specific except where noted (e.g. the panel/backlight/button peripherals a display-and-keypad unit has and a camera-only air unit would not).

## Reproducible build

Everything is pinned and the build runs in a hermetic container, so the output `Image` is bit-reproducible across machines:
- **Source + toolchain pinned + sha256-verified** in `pin.env` (linux 6.18.36; kernel.org crosstool gcc 14.2.0).
- **Fixed build metadata** (`SOURCE_DATE_EPOCH`, `KBUILD_BUILD_USER/HOST/TIMESTAMP`) so banners and timestamps are deterministic.
- **Hermetic container** (`scripts/Dockerfile`) provides the host-side build tools at fixed versions; the cross toolchain is the pinned crosstool, not the host's.

```sh
scripts/build.sh           # fetch+verify, configure, build -> Image + Image.sha256
scripts/build.sh verify    # build twice in separate trees, assert identical Image sha256
```

The build tree lives outside the repo (so it never pollutes git); set its location with `BUILD_DIR=`. The container runs `--network none` for the compile, all inputs are pre-fetched and verified on the host first.

Environment knobs:
- `BUILD_DIR` - where the (large) kernel tree is unpacked and built.
- `JOBS` - parallelism (default `nproc`).
- `MINIMAL=1` - pure `arm64 defconfig`, skips all config fragments.
- `NOTRIM=1` - skip `trim.config` (no size trimming).
- `DEBUGSDIO=1` - append `debug-sdio.config` (throwaway diagnostics).
- `FAST=1` - reuse the existing tree for an incremental build (dev loop). NOT bit-reproducible; do a clean build before flashing.

To bump the kernel or toolchain, edit `scripts/pin.env` (URL + sha256) only. For full base-image reproducibility, pin the `scripts/Dockerfile` `FROM` to a digest (noted there).

`scripts/build.sh` runs the actual build steps via `scripts/container-build.sh` inside the container; read that file for the exact configure/overlay/compile sequence.

## Layout

- `scripts/`: the kernel/module **build** scripts, kept out of the root so it doesn't clutter the layout below.
  - `pin.env`: pinned versions + hashes + deterministic metadata.
  - `Dockerfile`: hermetic build environment.
  - `build.sh`: fetch/verify, run the container build, `verify` mode.
  - `container-build.sh`: the in-container half (defconfig, fragment merge, driver overlays, compile).
- The device-interaction **glue** lives in the sibling `../glue/` tree (host-side scripts that talk to the device):
  - `../glue/dev/kdev.sh`: chains build + RAM-boot into one command (see "Build and test pipeline" below).
  - `../glue/flash/flash-kernel-b.sh`: writes a RAM-boot-proven `Image`+dtb to slot B only (see Flash below).
  - `../glue/dev/push.sh`: copy a file/dir to the device's slot-B tmpfs over SSH (no scp/sftp on-device).
- `configs/`: the shared config-fragment **files** merged onto `arm64 defconfig` (see Configuration fragments below). Which fragments a given board merges is listed per-board in `devices/<name>/fragments`, not here.
- `devices/`: one dir per supported device (e.g. `betafpv-vr04-goggle/`), holding that board's device tree (`*.dts`) and its config-fragment list (`fragments`). `BOARD=<name>` (default `betafpv-vr04-goggle`) selects the dir; the DTS basename sets the `.dtb` name.
- `overlay/`: complete driver sources we own (no mainline counterpart), copied into the kernel tree at build time. Currently `drivers/clk/clk-ar9311-cgu.c`, `drivers/spi/spi-ar9301.c` (QSPI-NAND controller), and the `drivers/gpu/drm/artosyn/` display driver. See `overlay/README.md`.
- `patches/`: unified diffs (`*.patch` + `series`) against pinned mainline `6.18.36` files we only tweak, applied with `patch -p1`. Currently the `ar-spin-table` SMP enable-method (`0001`/`0002`), the wave5 codec fixes (`0003`..`0010`), and the page-granular per-device coherent-pool allocator (`0011`, required for the concurrent decode+encode fit). A patch that stops applying on a kernel bump flags an upstream change. See `patches/README.md`.
- `modules/`: out-of-tree Artosyn kernel modules, built separately by `modules/build.sh`.
- `initramfs/`: minimal static-busybox initramfs for bare-kernel boot testing. Not used in the normal slot-B cold-boot path.
- `test_tools/`: on-device smoke tests exercising each driver through its real userspace ABI (LED, buzzer, buttons, display, overlay).
- `STATUS.md`: the single progress table for everything under `kernel/` - update progress there, not in the docs below.
- `PERIPHERALS.md`: per-peripheral architecture - what works via a stock/mainline driver vs. what needed a custom one.
- `docs/`: curated register-level reference behind the per-peripheral drivers (`artosyn-adc.md`, `artosyn-protemp.md`, `display-backlight.md`, `status-led.md`, `buzzer.md`, `artosyn-gpio.md`, `clocks.md` (AR9311 CGU), `artosyn-sdio.md` (AR8030 RF link), `sd-card.md` (microSD), `wave5-codec-capabilities.md`) - the current-state "why" behind the code in `patches/` and `modules/`.
- `ROADMAP.md`: why open reimplementation instead of the vendor `.ko`.

## Configuration fragments

The merge is a **universal base** followed by a **per-board list**, then `make olddefconfig`. Fragments later in the list override earlier ones (so trim disables broadly, then the feature fragments re-enable what that board needs). The universal base (`defconfig` -> `artosyn.config` -> `trim.config`) is applied by `container-build.sh`; the re-enables after trim come from `devices/$BOARD/fragments` (one fragment basename per line, in order), so the config composition is per-board and lives with the board. `DEBUGSDIO=1` appends `debug-sdio.config` last.

Universal base (every board):

| Order | Fragment | Purpose |
|---|---|---|
| 1 | arm64 `defconfig` | upstream baseline |
| 2 | `configs/artosyn.config` | platform base: UART console, devmem, FUSE/CUSE/binder, USB ECM/RNDIS gadget (dwc2), MTD/UBI/squashfs (rootfs + NAND), dw_mmc (SD), crash-recovery detectors |
| 3 | `configs/trim.config` | size trim: removes components we do not use (other vendors' SoC/board support, unused subsystems and drivers) so the LZ4-packed Image fits the 6 MiB kernel slot. Skip with `NOTRIM=1` |

Then the board's `devices/<name>/fragments` list. For `betafpv-vr04-goggle` that is, in order:

| Fragment | Purpose |
|---|---|
| `configs/modules.config` | re-enables `CONFIG_FB` (backs the DRM fbdev console), `CONFIG_MMC` (SD + RF SDIO), `CONFIG_IKCONFIG` (self-describing running config) |
| `configs/input.config` | IIO core + `adc-keys` + evdev (front-panel buttons), built-in |
| `configs/spi.config` | DesignWare SPI master + spidev (status LED), built-in |
| `configs/display.config` | PWM core + `pwm-backlight` + fbdev (built-in), plus the DRM/KMS path (`DRM`, `dw-mipi-dsi`, the `DRM_ARTOSYN` driver) built as **modules** to keep the DRM mass out of the Image |
| `configs/codec.config` | the open codec: `MEDIA_SUPPORT` + V4L2 M2M + `VIDEO_WAVE_VPU=m` (needs `patches/0003` to lift the arch gate) |
| `configs/exfat.config` | exFAT (the SD card / DVR filesystem), built-in |
| `configs/usb-mtp.config` | MTP over USB (FunctionFS) for the DVR-recordings gadget |
| `configs/dma.config` | the dw-axi DMA engine (built-in; the `ml_dmablit` compositor path) + `DMATEST=m` |
| `configs/cpufreq.config` | cpufreq-dt + governors for DVFS on `cgu_cpu_clk` (vendor OPP table in the DT) |
| `configs/i2c-rtc.config` | DesignWare I2C + `rtc-ds1307` (external DS1307 @0x68) |

Opt-in, appended last: `configs/debug-sdio.config` (`DYNAMIC_DEBUG` + `MMC_DEBUG` for SDIO bring-up diagnostics; only when `DEBUGSDIO=1`; throwaway, enlarges the Image, may violate slot-fit).

`MINIMAL=1` skips the fragment merge entirely and produces a pure `arm64 defconfig` kernel.

## Built-in vs out-of-tree modules

**Built-in (`=y`, compiled into the Image):** mainline SPI DesignWare (`spi_dw_mmio`) + spidev, IIO core, `adc-keys`, evdev, PWM core, `pwm-backlight`, fbdev, plus our own built-ins:
- `clk-ar9311-cgu` (AR9311 clock provider) and `spi-ar9301` (Artosyn QSPI-NAND controller): full-file drivers from `overlay/`, wired `obj-y`.
- `ar-spin-table`: the patched `smp_spin_table.c` / `cpu_ops.c` implementing the secondary-core wakeup, from `patches/` (see SMP below).

**Out-of-tree / loadable modules (`=m`, loaded from the rootfs):**
- Artosyn modules in `modules/` (built by `modules/build.sh`):
  - Peripheral controllers: `artosyn_adc` (the `adc-keys` IIO provider), `artosyn_pwm` (PWM chip for backlight/buzzer), `artosyn_gpio`.
  - SD/SDIO: `artosyn_sdio`, `dw_mci-artosyn`, `artosyn_mmc` (diagnostic).
  - The MPP reimplementations (`ar_osal`, `ar_vb`, `ar_sys`, `ar_sysctl`, `ar_mpp_drv`, `ar_mpp_proc_ctrl`, `ar_scaler`, `ar_mpp_overlay`) are reference only: compile-checked, not staged into the rootfs or loaded at boot (`modules/README.md`).
- The DRM/KMS stack (`drm`, `drm_kms_helper`, `dw-mipi-dsi`, and `DRM_ARTOSYN` = `artosyn_vo` CRTC + `artosyn_dsi` glue + `panel-qy45043a0`) is `=m`, in-tree from `overlay/`, built by `make modules` in the kernel tree, then loaded from the rootfs alongside the Artosyn modules.

The built-in `adc-keys` and `pwm-backlight` deferred-probe: `adc-keys` binds once `artosyn_adc` registers its IIO device, `pwm-backlight` once `artosyn_pwm` registers its PWM chip.

## Slot fit and size trim

The `kernel1` partition is 6 MiB and SPL Falcon decompresses with LZ4 only, so the `Image` must fit that partition LZ4-packed. With `trim.config` applied it currently packs to roughly 5.5 MiB (about 500 KB of margin). The main reductions: all non-Proxima SoC platform families, the unused ZSTD library, PHYLIB and unused USB host controllers, ~10 dead-weight subsystems arm64 defconfig enables but that have no DT node here, plus size-optimized compile (`-Os`) and link-time dead-code elimination. Building the DRM stack as modules (above) keeps it out of the Image entirely.

To pack the `Image` into the OTRA + legacy-uImage(lz4) container that SPL/U-Boot require:
```sh
glue/flash/mkkernel.py pack <Image> <out.bin> --otra-template <kernelN partition bin or file>
```
`--otra-template` supplies the OTRA header (`ram-boot.sh` pulls it read-only from the live `kernel1` automatically). The LZ4 frame must use independent blocks (`FLG=0x64 BD=0x70`); `mkkernel.py` matches the vendor frame exactly. Linked blocks cause a `-93` (`-EPROTONOSUPPORT`) error in U-Boot.

`debug-sdio.config` adds debug tables that can push the packed size past 6 MiB; it is for throwaway diagnostic builds only.

## SMP

Both A53 cores come online via the `ar-spin-table` enable-method (`patches/000{1,2}-arm64-*.patch` over `arch/arm64/kernel/{cpu_ops,smp_spin_table}.c`, referenced from the DT `cpu` nodes). The vendor SPL parks the secondary core with a Proxima-9311-specific release protocol that upstream `spin-table` does not implement; the patched files do the correct wakeup sequence. (cpufreq/DVFS status: see "Status" below.)

## U-Boot / boot constraints

Handled by `glue/flash/mkkernel.py` and `glue/boot/ram-boot.sh`:
- This U-Boot rejects a raw `booti` of an arm64 `Image` ("magic error!"). The Image must be wrapped in the OTRA + legacy-uImage(lz4) container and `bootm`'d.
- The LZ4 frame must use independent blocks (linked blocks fail with `-93`).
- RAM is 256 MiB (`0x20000000`-`0x30000000`). The container loads at `0x24000000`; `bootm` decompresses the Image to `0x200a0000`.
- `bootm` does not supply `mtdparts` (SPL does on a flash boot), so the RAM-boot bootargs must carry the full `mtdparts=` string or the kernel panics at rootfs mount.

## RAM-boot (test without flashing)

`glue/boot/ram-boot.sh <Image> <dtb>` runs the whole test sequence: pack the Image, drop the device to U-Boot, `loady` the dtb + container over the serial bridge, `bootm`, and confirm the kernel came up. Nothing is written to flash and slot B stays active, so a power cycle returns to the flashed slot-B kernel. Once the device is already at the U-Boot prompt, `glue/boot/ramboot-at-uboot.sh <container> <dtb>` does just the `loady`+`bootm` half.

Preconditions: the device is on the open slot-B Alpine and reachable over the network, and the Pico UART serial bridge is connected. Device-access and serial setup are in `docs/guides/`.

## Flash (commit a RAM-boot-proven kernel to slot B)

Only after `glue/boot/ram-boot.sh` has proven the candidate `Image`+dtb boot end to end with **slot A still active** (previous section). `glue/flash/flash-kernel-b.sh <Image> <dtb>` writes **only** `kernel1`/`dtb1` (slot B): it refuses to run unless the device currently answers as slot A, resolves the partitions by name (never a hardcoded mtd number) and refuses if they'd alias slot A's, packs the `Image` into the OTRA container itself, and verifies the write by reading the flashed bytes back and comparing sha256 - all before you've touched the active-slot pointer. The active slot is **still A** when the script finishes.

```sh
glue/flash/flash-kernel-b.sh <Image> <dtb>              # writes kernel1 + dtb1 only, verifies by readback
ROOT_PASS=artosyn glue/boot/ram-boot-flashed-b.sh        # gold standard: RAM-boot the ACTUAL flashed bytes
glue/boot/flip-slot.sh b                                  # only after that succeeds - makes B the active slot
```

This is the same untainted-A ladder every A/B write in this project follows - `glue/docs/flash-and-verify-slots.md` has the full method (why each step exists, the general recovery story) if you want the background; the three commands above are the kernel-specific instance of it. See the **HARD RULES** banner at the top of `../CLAUDE.md` before doing any of this on real hardware - flipping to an unproven slot B once bricked a unit.

## Status

Current progress (build, boot, per-peripheral, per-module, known gaps) is tracked in one place: **`STATUS.md`**. Architecture/how-it-works detail for peripherals lives in `PERIPHERALS.md`, for the MPP/RF modules in `modules/README.md`, and forward-looking planning in `ROADMAP.md` - none of those restate current status, they point here.

## Build and test pipeline

| # | step | script | output |
|---|---|---|---|
| 1 | kernel + DT | `scripts/build.sh` (pinned, hermetic, reproducible) | `<build>/linux/arch/arm64/boot/{Image,proxima-9311.dtb}` |
| 2 | out-of-tree + `=m` modules | `modules/build.sh` (host cross-gcc against the built tree) | `<build>/ml-modules/` (incl. the staged `rootfs/lib/modules/`) |
| 3 | static busybox (bare-kernel test) | `initramfs/build-busybox.sh` | `initramfs/build/busybox-aarch64` |
| 4 | initramfs (bare-kernel test) | `initramfs/build.sh` | `initramfs/build/initramfs.cpio.gz` |
| 5 | RAM-boot test | `glue/boot/ram-boot.sh <Image> <dtb>` | boots the new kernel from RAM; slot B unchanged |

`glue/dev/kdev.sh` chains these into one command with composable flags: `--build` (full reproducible) or `--build-fast` (incremental dev loop) builds the kernel + modules, and `--ramboot` RAM-boots whatever is currently built. For example `kdev.sh --build-fast --ramboot`, or `kdev.sh --ramboot` to boot the current build without rebuilding.

To exercise a kernel/dtb without a rootfs (boot straight to a busybox shell on the UART instead of mounting the flashed rootfs), build the initramfs (steps 3-4) and add `--initramfs`: `kdev.sh --ramboot --initramfs`. This RAM-boots the current Image + dtb with `initramfs/build/initramfs.cpio.gz` as the root; the flashed slot is untouched, so a power-cycle returns to it. Under the hood `ram-boot.sh` honours `INITRAMFS=<cpio.gz>` (loaded to `RDADDR`, passed to `bootm` as `addr:size`); `--initramfs` only ever applies with `--ramboot`, and a plain `--ramboot` never picks up a stale artifact from `build/`.

Host toolchains: docker (kernel Image), `gcc-aarch64-linux-gnu` (modules + busybox). The serial scripts auto-detect the Pico bridge by USB id (override with `$ML_SERIAL`).

## Support

This is unpaid nights-and-weekends work: reverse engineering, bricked-and-recovered hardware, and serial-console archaeology. Everything here is free and open, but if it saved you time or got video flowing off your goggles, you can [buy me a coffee](https://buymeacoffee.com/stylesuxx) - it genuinely helps keep work like this going.
