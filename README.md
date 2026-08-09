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

Artifacts, all under `$BUILD_DIR` except the initramfs pair:

| what | script | output |
|---|---|---|
| kernel + device tree | `scripts/build.sh` (pinned, hermetic, reproducible) | `<build>/linux/arch/arm64/boot/{Image,<board>.dtb}` |
| out-of-tree + `=m` modules | `modules/build.sh` (host cross-gcc against the built tree) | `<build>/ml-modules/`, incl. the staged `rootfs/lib/modules/` |
| static busybox (bare-kernel test) | `initramfs/build-busybox.sh` | `initramfs/build/busybox-aarch64` |
| initramfs (bare-kernel test) | `initramfs/build.sh` | `initramfs/build/initramfs.cpio.gz` |

Host toolchains: docker (kernel `Image`), `gcc-aarch64-linux-gnu` (modules + busybox).

The build tree lives outside the repo so it never pollutes git. The container runs `--network none` for the compile, all inputs are pre-fetched and verified on the host first.

Environment knobs:
- `BUILD_DIR` - where the kernel tree is unpacked and built. It is large.
- `JOBS` - parallelism (default `nproc`).
- `MINIMAL=1` - pure `arm64 defconfig`, skips all config fragments.
- `NOTRIM=1` - skip `trim.config` (no size trimming).
- `LAX_FRAGMENTS=1` - downgrade the post-`olddefconfig` fragment check from fatal to a warning, for bisecting a kernel bump that renames symbols.
- `FAST=1` - reuse the existing tree for an incremental build (dev loop). NOT bit-reproducible; do a clean build before flashing. Declined automatically, with a log line, when an input that only reaches a tree one way has changed since it was extracted (an edited patch, a deleted overlay source, a changed Kconfig/Makefile hook); see `inputs_sha` in `scripts/build.sh`.

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
| 3 | `configs/trim.config` | size trim: removes components we do not use (other vendors' SoC/board support, unused subsystems and drivers) so the LZ4-packed `Image` fits the kernel slot, see "Boot container and slot fit". Skip with `NOTRIM=1` |

The per-board lists then diverge along the obvious line, what the hardware physically has. `betafpv-vr04-goggle` merges the display, input, and SPI fragments for its panel, button ladder, and status LED. `betafpv-vr04-air` drops all three, having none of that hardware, and adds the camera, IIO, and DMA-heap fragments for its sensor and encoder path. Both merge the shared codec, storage, MTP, DMA, cpufreq, and RTC fragments. Read `devices/<name>/fragments` for the exact list and the reason on each line.

`MINIMAL=1` skips the fragment merge entirely and produces a pure `arm64 defconfig` kernel.

## Built-in vs loadable drivers

The default is **built-in** (`=y`), and three things move a driver to **loadable** (`=m`).

- **Boot path forces built-in.** Anything needed to reach the rootfs has to be in the `Image`, because there is no filesystem to load a module from yet: the clock provider, the QSPI-NAND controller and MTD/UBI, the console, the SMP enable-method. These can never be modules.
- **Size forces loadable.** The `Image` must fit a 6 MiB slot LZ4-packed (see "Boot container and slot fit"), so anything large that is not on the boot path is a module. The DRM/KMS stack is the main case: it is bigger than the remaining margin, and nothing before the rootfs mounts draws to a display.
- **Out-of-tree is always loadable.** The Artosyn modules under `modules/` are built against an already-built kernel tree, so they cannot be linked into the `Image` by construction.

Small mainline drivers off the boot path stay built-in, because modularizing them would add packaging and load-ordering work for no size gain.

The split has one consequence worth knowing: a built-in driver can depend on a loadable one, and then it sits in deferred probe until that module loads from the rootfs. That is the normal state on this board for the input and backlight drivers, which wait on their loadable IIO and PWM providers.

The exact set is the config, not this file: `configs/*.config` carries the `=y`/`=m` choice per driver, `modules/README.md` lists the out-of-tree modules, and `overlay/README.md` the drivers we own.

## Boot container and slot fit

The `kernel1` partition is 6 MiB and SPL Falcon decompresses with LZ4 only, so the `Image` has to fit that partition packed - and it has to be packed in a form this U-Boot accepts:

- A raw arm64 `Image` will not `booti` here ("magic error!"). It must be wrapped in the OTRA + legacy-uImage(lz4) container and `bootm`'d.
- The LZ4 frame must use independent blocks (`FLG=0x64 BD=0x70`). Linked blocks fail with `-93` (`-EPROTONOSUPPORT`).
- RAM is 256 MiB (`0x20000000`-`0x30000000`). The container loads at `0x24000000`; `bootm` decompresses the `Image` to `0x200a0000`.
- `bootm` does not supply `mtdparts` (SPL does on a flash boot), so a RAM-boot's bootargs must carry the full `mtdparts=` string or the kernel panics at rootfs mount.

`glue/flash/mkkernel.py` produces that container and matches the vendor frame exactly:

```sh
glue/flash/mkkernel.py pack <Image> <out.bin> --otra-template <kernelN partition bin or file>
glue/flash/mkkernel.py size <Image>     # packed size against the 6 MiB slot
```

`--otra-template` supplies the OTRA header; `glue/boot/ram-boot.sh` pulls it read-only from the live `kernel1` automatically.

**The margin is thin: run `mkkernel.py size` before flashing rather than trusting a number here.** As of the 6.18.36 air-unit build it packed to 96.5% of the slot, around 212 KiB free. What buys that fit is `trim.config` dropping all non-Proxima SoC platform families, the unused ZSTD library, unused USB host controllers and the subsystems arm64 defconfig enables that have no device-tree node here, plus `-Os`. The other half of the answer is the `=m` split described under "Built-in vs loadable drivers", which keeps the DRM stack out of the `Image` entirely.

Two things that read like size levers here and are not:

- `LD_DEAD_CODE_DATA_ELIMINATION` is unavailable on arm64, which does not select `HAVE_LD_DEAD_CODE_DATA_ELIMINATION`. It never did anything.
- Deleting `# CONFIG_PHYLIB is not set` makes the `Image` **larger**. The line cannot force a `select`ed symbol off, but it does hold `PHYLIB` at `=m` instead of `=y`, which keeps it and its PHY drivers out of the `Image`. See the fragment-check notes in `scripts/container-build.sh`.

## SMP

Both A53 cores come online via an `ar-spin-table` enable-method, referenced from the device tree `cpu` nodes. The vendor SPL parks the secondary core with a Proxima-9311-specific release protocol that upstream `spin-table` does not implement, so `arch/arm64/kernel/cpu_ops.c` and `smp_spin_table.c` are patched to add the enable-method and do the correct wakeup sequence. This is a patch rather than a driver because the enable-method table lives inside those core files; there is no out-of-tree hook for it. See `patches/README.md`.

For cpufreq/DVFS status see "Status" below.

## Testing a kernel on hardware

Nothing in this tree talks to a device. The sequence is RAM-boot the candidate to prove it, flash slot B, then flip - all of it in `../glue/`, documented in `../glue/README.md`, wrapped as `make` targets in the top-level `../README.md`, with the authoritative ladder in `../glue/docs/flash-and-verify-slots.md`.

`glue/dev/kdev.sh` is the inner loop: `--build` (full reproducible) or `--build-fast` (incremental) builds the kernel + modules, `--ramboot` boots whatever is currently built, and `--initramfs` boots to a busybox shell on the UART instead of mounting the flashed rootfs. For example `kdev.sh --build-fast --ramboot`.

Two properties of that sequence are worth stating here, because misreading either costs a device:

- **A RAM boot writes nothing to flash.** It swaps the kernel only. A power cycle returns to the active slot, and the **rootfs stays the flashed one** - a new kernel does not bring new userspace with it.
- **The active slot never flips on its own.** Flashing writes `kernel1`/`dtb1` and verifies by readback while the active slot is unchanged. Making B active is a separate, explicit step, taken only after a RAM boot of the *actual flashed bytes* has succeeded.

The slot the device must be on differs between the two, which is the easiest thing here to misread. The **flasher** refuses to run unless the device currently answers as **slot A**, so A stays untainted while B is written. The **dev loop** RAM-boots from whatever is running, which day to day is the open **slot B** Alpine. Both are "prove it in RAM first"; they simply start from different places.

Read the **HARD RULES** banner at the top of `../AGENTS.md` before doing any of this on real hardware - flipping to an unproven slot B once bricked a unit.

## Status

Build, boot, per-peripheral, per-module state and known gaps are tracked in one place, **`STATUS.md`**. No other document here restates current status; they point at it.

## Support

Everything here is free and open. The work behind it is unpaid nights and weekends: reverse engineering, bricked and recovered hardware, and a lot of time on a serial console. If it saved you some of your own, you can [buy me a coffee](https://buymeacoffee.com/stylesuxx).

Not bought the hardware yet? The [project README](https://github.com/Missing-Lynk/MissingLynk#support-this-project) has affiliate links that support the work at no extra cost to you.
