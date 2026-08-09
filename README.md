# Modern kernel for Artosyn Proxima-9311 devices (reproducible build)

An open Linux 6.18.36 kernel for the Artosyn Proxima-9311 SoC + AR8030 RF link, running the hardware with no vendor userspace.

Drives:
- Display (DRM/KMS)
- Video codec (wave5, V4L2)
- AR8030 RF link
- Board peripherals (SD/SDIO, buttons, status LED, buzzer, backlight, ADC, RTC)

The same SoC/RF chip pair shows up across goggle, VRx, VTx, and air-unit products, and nothing here is device-specific except where noted, for example the panel/backlight/button peripherals a display-and-keypad unit has and a camera-only air unit would not. This repo's hardware validation happens on a goggle, the BetaFPV VR04 HD.

Naming: `AR9311` in driver names is this same SoC, the Proxima-9311, as in `clk-ar9311-cgu`. `AR9301` names its QSPI-NAND controller IP, `spi-ar9301`.

## Reproducible build

Everything is pinned and the build runs in a hermetic container, so the output `Image` is bit-reproducible across machines:
- **Source + toolchain pinned + sha256-verified** in `pin.env` (linux 6.18.36; kernel.org crosstool gcc 14.2.0).
- **Fixed build metadata** (`SOURCE_DATE_EPOCH`, `KBUILD_BUILD_USER/HOST/TIMESTAMP`) so banners and timestamps are deterministic.
- **Hermetic container** (`scripts/Dockerfile`) provides the host-side build tools at fixed versions; the cross toolchain is the pinned crosstool, not the host's.

```sh
scripts/build.sh           # fetch+verify, configure, build -> Image, plus Image.sha256 to compare against
scripts/build.sh verify    # build twice in separate trees, assert identical Image sha256
```

The build tree lives outside the repo so it never pollutes git. The container runs `--network none` for the compile, all inputs are pre-fetched and verified on the host first.

Environment knobs:
- `BUILD_DIR` - where the kernel tree is unpacked and built. It is large.
- `JOBS` - parallelism (default `nproc`).
- `MINIMAL=1` - pure `arm64 defconfig`, skips all config fragments.
- `NOTRIM=1` - skip `trim.config` (no size trimming).
- `LAX_FRAGMENTS=1` - downgrade the post-`olddefconfig` fragment check from fatal to a warning, for bisecting a kernel bump that renames symbols.
- `FAST=1` - reuse the existing tree for an incremental build (dev loop). NOT bit-reproducible; do a clean build before flashing.

To bump the kernel or toolchain, edit `scripts/pin.env` (URL + sha256) only.

`scripts/build.sh` runs the actual build steps via `scripts/container-build.sh` inside the container; read that file for the exact configure/overlay/compile sequence.

Note: for full base-image reproducibility, pin the `scripts/Dockerfile` `FROM` to a digest. This is noted in the Dockerfile itself.

## Layout

- `scripts/`: the kernel/module **build** scripts.
  - `pin.env`: pinned versions + hashes + deterministic metadata.
  - `Dockerfile`: hermetic build environment.
  - `build.sh`: fetch/verify, run the container build, `verify` mode.
  - `container-build.sh`: the in-container half (defconfig, fragment merge, overlay copy, compile).
- `configs/`: the shared config-fragment **files** merged onto `arm64 defconfig`. Which fragments a given board merges is listed per-board in `devices/<name>/fragments`, not here. See "Configuration fragments" below.
- `devices/`: one dir per supported device, for example `betafpv-vr04-goggle/`, holding that board's device tree (`*.dts`) and its config-fragment list (`fragments`). `BOARD=<name>` (default `betafpv-vr04-goggle`) selects the dir; the DTS basename sets the `.dtb` name.
- `overlay/`: complete driver sources we own, with no mainline counterpart, laid out in kernel-tree paths and copied into the tree at build time. The clock provider, the QSPI-NAND controller, and the display driver live here. See `overlay/README.md`.
- `patches/`: unified diffs (`*.patch` + `series`) against pinned mainline `6.18.36` files we only tweak, applied with `patch -p1`. They cover the SMP enable-method, the wave5 codec fixes, and a page-granular per-device coherent-pool allocator. A patch that stops applying on a kernel bump flags an upstream change. The current set and what each one does is in `patches/README.md`.
- `modules/`: out-of-tree Artosyn kernel modules, built separately by `modules/build.sh`.
- `initramfs/`: minimal static-busybox initramfs for bare-kernel boot testing. Not used in the normal slot-B cold-boot path.
- `test_tools/`: on-device smoke tests exercising each driver through its real userspace ABI (LED, buzzer, buttons, display, overlay).
- `STATUS.md`: the single progress table for everything under `kernel/` - update progress there, not in the docs below.
- `PERIPHERALS.md`: per-peripheral architecture - what works via a stock/mainline driver vs. what needed a custom one.
- `docs/`: curated register-level reference, one file per peripheral - the current-state "why" behind the code in `overlay/`, `patches/`, and `modules/`.
- `ROADMAP.md`: why open reimplementation instead of the vendor `.ko`.

Nothing here talks to a device. Everything host-side that does - the build + RAM-boot inner loop, the slot-B flashers, serial and U-Boot access, recovery - lives in the sibling `../glue/` tree and is documented in `../glue/README.md`.

This tree produces a kernel, not a system. The `Image` boots the open Alpine rootfs built by the sibling `../rootfs/` tree, and that is where `modules/build.sh` stages the loadable modules for `modprobe` to find at runtime.

## Configuration fragments

The merge is a **universal base** followed by a **per-board list**, then `make olddefconfig`. Fragments later in the merge override earlier ones, so the trim disables broadly and the feature fragments re-enable what that board needs.

```
arm64 defconfig
      |
      v
configs/artosyn.config       universal base, applied by container-build.sh
      |
      v
configs/trim.config
      |
      v
devices/$BOARD/fragments     per-board re-enables, one basename per line, in order
      |
      v
make olddefconfig
```

Because the re-enables live in `devices/$BOARD/fragments`, the config composition is per-board and lives with the board. That file is annotated line by line, so it, and not this README, is where a board's composition is stated.

Universal base (every board):

| Order | Fragment | Purpose |
|---|---|---|
| 1 | arm64 `defconfig` | upstream baseline |
| 2 | `configs/artosyn.config` | platform base: UART console, devmem, FUSE/CUSE/binder, USB ECM/RNDIS gadget (dwc2), MTD/UBI/squashfs (rootfs + NAND), dw_mmc (SD), crash-recovery detectors |
| 3 | `configs/trim.config` | size trim: removes components we do not use (other vendors' SoC/board support, unused subsystems and drivers) so the LZ4-packed `Image` fits the kernel slot, see "Slot fit and size trim". Skip with `NOTRIM=1` |

The per-board lists then diverge along the obvious line, what the hardware physically has. `betafpv-vr04-goggle` merges the display, input, and SPI fragments for its panel, button ladder, and status LED. `betafpv-vr04-air` drops all three, having none of that hardware, and adds the camera, IIO, and DMA-heap fragments for its sensor and encoder path. Both merge the shared codec, storage, MTP, DMA, cpufreq, and RTC fragments. Read `devices/<name>/fragments` for the exact list and the reason on each line.

`MINIMAL=1` skips the fragment merge entirely and produces a pure `arm64 defconfig` kernel.

## Built-in vs loadable drivers

The default is **built-in** (`=y`), and three things move a driver to **loadable** (`=m`).

- **Boot path forces built-in.** Anything needed to reach the rootfs has to be in the `Image`, because there is no filesystem to load a module from yet: the clock provider, the QSPI-NAND controller and MTD/UBI, the console, the SMP enable-method. These can never be modules.
- **Size forces loadable.** The `Image` must fit a 6 MiB slot LZ4-packed (see "Slot fit and size trim"), so anything large that is not on the boot path is a module. The DRM/KMS stack is the main case: it is bigger than the remaining margin, and nothing before the rootfs mounts draws to a display.
- **Out-of-tree is always loadable.** The Artosyn modules under `modules/` are built against an already-built kernel tree, so they cannot be linked into the `Image` by construction.

Small mainline drivers off the boot path stay built-in, because modularizing them would add packaging and load-ordering work for no size gain.

The split has one consequence worth knowing: a built-in driver can depend on a loadable one, and then it sits in deferred probe until that module loads from the rootfs. That is the normal state on this board for the input and backlight drivers, which wait on their loadable IIO and PWM providers.

The exact set is the config, not this file: `configs/*.config` carries the `=y`/`=m` choice per driver, `modules/README.md` lists the out-of-tree modules, and `overlay/README.md` the drivers we own.

## Slot fit and size trim

The `kernel1` partition is 6 MiB and SPL Falcon decompresses with LZ4 only, so the `Image` must fit that partition LZ4-packed. With `trim.config` applied it currently packs to roughly 5.5 MiB, about 500 KB of margin. The main reductions: all non-Proxima SoC platform families, the unused ZSTD library, PHYLIB and unused USB host controllers, ~10 dead-weight subsystems arm64 defconfig enables but that have no device-tree node here, plus size-optimized compile (`-Os`) and link-time dead-code elimination. Building the DRM stack as modules keeps it out of the `Image` entirely.

To pack the `Image` into the OTRA + legacy-uImage(lz4) container that SPL/U-Boot require:
```sh
glue/flash/mkkernel.py pack <Image> <out.bin> --otra-template <kernelN partition bin or file>
```
`--otra-template` supplies the OTRA header. `ram-boot.sh` pulls it read-only from the live `kernel1` automatically. The LZ4 frame must use independent blocks (`FLG=0x64 BD=0x70`); `mkkernel.py` matches the vendor frame exactly. Linked blocks cause a `-93` (`-EPROTONOSUPPORT`) error in U-Boot.

## SMP

Both A53 cores come online via an `ar-spin-table` enable-method, referenced from the device tree `cpu` nodes. The vendor SPL parks the secondary core with a Proxima-9311-specific release protocol that upstream `spin-table` does not implement, so `arch/arm64/kernel/cpu_ops.c` and `smp_spin_table.c` are patched to add the enable-method and do the correct wakeup sequence. This is a patch rather than a driver because the enable-method table lives inside those core files; there is no out-of-tree hook for it. See `patches/README.md`.

For cpufreq/DVFS status see "Status" below.

## U-Boot / boot constraints

Handled by `glue/flash/mkkernel.py` and `glue/boot/ram-boot.sh`:
- This U-Boot rejects a raw `booti` of an arm64 `Image` ("magic error!"). The `Image` must be wrapped in the OTRA + legacy-uImage(lz4) container and `bootm`'d.
- The LZ4 frame must use independent blocks (linked blocks fail with `-93`).
- RAM is 256 MiB (`0x20000000`-`0x30000000`). The container loads at `0x24000000`; `bootm` decompresses the `Image` to `0x200a0000`.
- `bootm` does not supply `mtdparts` (SPL does on a flash boot), so the RAM-boot bootargs must carry the full `mtdparts=` string or the kernel panics at rootfs mount.

## RAM-boot (test without flashing)

`glue/boot/ram-boot.sh <Image> <dtb>` runs the whole test sequence: pack the `Image`, drop the device to U-Boot, `loady` the device tree blob + container over the serial bridge, `bootm`, and confirm the kernel came up. Nothing is written to flash and slot B stays active, so a power cycle returns to the flashed slot-B kernel. Once the device is already at the U-Boot prompt, `glue/boot/ramboot-at-uboot.sh <container> <dtb>` does just the `loady`+`bootm` half.

Preconditions: the device is on the open slot-B Alpine and reachable over the network, and the Pico UART serial bridge is connected. Device-access and serial setup are in `docs/guides/`.

## Flash (commit a RAM-boot-proven kernel to slot B)

Only after the previous section's `glue/boot/ram-boot.sh` has proven the candidate `Image` + device tree blob boot end to end with **slot A still active**. `glue/flash/flash-kernel-b.sh <Image> <dtb>` writes **only** `kernel1`/`dtb1` (slot B): it refuses to run unless the device currently answers as slot A, resolves the partitions by name (never a hardcoded mtd number) and refuses if they'd alias slot A's, packs the `Image` into the OTRA container itself, and verifies the write by reading the flashed bytes back and comparing sha256 - all before you've touched the active-slot pointer. The active slot is **still A** when the script finishes.

```sh
glue/flash/flash-kernel-b.sh <Image> <dtb>              # writes kernel1 + dtb1 only, verifies by readback
ROOT_PASS=artosyn glue/boot/ram-boot-flashed-b.sh        # gold standard: RAM-boot the ACTUAL flashed bytes
glue/boot/flip-slot.sh b                                  # only after that succeeds - makes B the active slot
```

This is the same untainted-A ladder every A/B write in this project follows - `glue/docs/flash-and-verify-slots.md` has the full method (why each step exists, the general recovery story) if you want the background; the three commands above are the kernel-specific instance of it. See the **HARD RULES** banner at the top of `../CLAUDE.md` before doing any of this on real hardware - flipping to an unproven slot B once bricked a unit.

## Status

Build, boot, per-peripheral, per-module state and known gaps are tracked in one place, **`STATUS.md`**. No other document here restates current status; they point at it.

## Build and test pipeline

| # | step | script | output |
|---|---|---|---|
| 1 | kernel + device tree | `scripts/build.sh` (pinned, hermetic, reproducible) | `<build>/linux/arch/arm64/boot/{Image,proxima-9311.dtb}` |
| 2 | out-of-tree + `=m` modules | `modules/build.sh` (host cross-gcc against the built tree) | `<build>/ml-modules/` (incl. the staged `rootfs/lib/modules/`) |
| 3 | static busybox (bare-kernel test) | `initramfs/build-busybox.sh` | `initramfs/build/busybox-aarch64` |
| 4 | initramfs (bare-kernel test) | `initramfs/build.sh` | `initramfs/build/initramfs.cpio.gz` |
| 5 | RAM-boot test | `glue/boot/ram-boot.sh <Image> <dtb>` | boots the new kernel from RAM; slot B unchanged |

`glue/dev/kdev.sh` chains these into one command with composable flags: `--build` (full reproducible) or `--build-fast` (incremental dev loop) builds the kernel + modules, and `--ramboot` RAM-boots whatever is currently built. For example `kdev.sh --build-fast --ramboot`, or `kdev.sh --ramboot` to boot the current build without rebuilding.

To exercise a kernel and device tree without a rootfs, booting straight to a busybox shell on the UART instead of mounting the flashed rootfs, build the initramfs (steps 3-4) and add `--initramfs`: `kdev.sh --ramboot --initramfs`. This RAM-boots the current `Image` + device tree blob with `initramfs/build/initramfs.cpio.gz` as the root; the flashed slot is untouched, so a power-cycle returns to it. Under the hood `ram-boot.sh` honours `INITRAMFS=<cpio.gz>`, loaded to `RDADDR` and passed to `bootm` as `addr:size`. `--initramfs` only ever applies with `--ramboot`, and a plain `--ramboot` never picks up a stale artifact from `build/`.

Host toolchains: docker (kernel `Image`), `gcc-aarch64-linux-gnu` (modules + busybox). The serial scripts auto-detect the Pico bridge by USB id (override with `$ML_SERIAL`).

## Support

Everything here is free and open. The work behind it is unpaid nights and weekends: reverse engineering, bricked and recovered hardware, and a lot of time on a serial console. If it saved you some of your own, you can [buy me a coffee](https://buymeacoffee.com/stylesuxx).

Not bought the hardware yet? The [project README](https://github.com/Missing-Lynk/MissingLynk#support-this-project) has affiliate links that support the work at no extra cost to you.
