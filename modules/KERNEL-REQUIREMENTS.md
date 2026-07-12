# Kernel requirements for the open out-of-tree modules

**Audience: project contributors and agents working on the slot-B kernel build**, not
end users of the shipped modules - this is the internal record of what the kernel
config/DT must provide for `modules/` to load and be match-buildable, why, and
what's done vs. still open. External users building/flashing the modules should start
at `README.md`.

## TL;DR
1. Merge **`configs/modules.config`** into the kernel build (it's now
   appended automatically by `scripts/build.sh`, last, so it overrides `trim.config`).
   It enables **IKCONFIG**, **FB**, and **MMC/SDIO**.
2. The **`mmc1` host-controller** DT node + driver for the AR8030 SDIO link is DONE
   (`dw_mci-artosyn` + `artosyn_gpio`; see §4) - but the `ar_dtbo_sdio` overlay that
   provides the node itself is a known build-tracking gap, also §4.
3. Rebuild + reflash slot B, then build the modules against that exact kernel (via
   `/proc/config.gz`) and resume hardware bring-up.

## 1. The problem this fixes (why modules panicked on load)

`ar_osal` insmod panicked the goggle instantly:
`Kernel panic: stack-protector: stack is corrupted in ar_osal_init`.
It is **NOT a code bug** (confirmed by disassembly - the init has no stack buffer to
overflow; serial console caught the panic). Root cause:

- The kernel is built with `CONFIG_MODVERSIONS=n` + `CONFIG_STACKPROTECTOR_PER_TASK=y`.
- With MODVERSIONS off, a module built against a *different config* still loads (the
  vermagic string still matches "6.18.36 SMP preempt … aarch64"), but its compiled-in
  `task_struct` layout differs -> it reads the stack canary from the wrong offset ->
  the canary check fails on init return -> panic.
- The device ran a **trimmed** kernel (38,882 kallsyms, no aarch32/VT); the modules had
  been built against a **differently-configured** 6.18.36 tree. Mismatch.

**Fix = build modules against the EXACT kernel that's flashed.** The device had no
`CONFIG_IKCONFIG_PROC`, so its config wasn't extractable and couldn't be matched.
Enabling IKCONFIG (below) makes this self-serve and permanent.

## 2. Config changes - done in this repo (`configs/modules.config`)

| symbol | for | why |
|---|---|---|
| `CONFIG_IKCONFIG=y` + `CONFIG_IKCONFIG_PROC=y` | all | `zcat /proc/config.gz` -> build modules against the exact running config. **The key fix.** |
| `CONFIG_FB=y`, `CONFIG_FB_DEVICE=y`, `CONFIG_FB_CFB_{FILLRECT,COPYAREA,IMAGEBLIT}=y` | `ar_framebuffer` | fbdev core + `/dev/fbN` + the `cfb_*` blit helpers the driver calls. |
| `CONFIG_MMC=y` (+ `CONFIG_MMC_DW=y`) | `artosyn_sdio` | MMC/SDIO core symbols (`sdio_register_driver`, `sdio_readb/writeb`, `sdio_memcpy_toio`, `mmc_wait_for_cmd`, …). |

`scripts/build.sh` appends `modules.config` after `trim.config` automatically, so
these win. `trim.config` no longer disables FB/MMC; the fragment keeps what the modules
need isolated/reviewable.

The other 6 modules (`ar_osal`, `ar_vb`, `ar_sys`, `ar_sysctl`, `ar_mpp_drv`,
`ar_mpp_proc_ctrl`) need **only core symbols** (already present) - unaffected by the above.

## 3. Size tradeoff

`trim.config` exists to fit the Image in the 6 MB `kernel0/1` partition. FB + MMC
built-in grow the Image. Options, in order of preference:
- Build **FB/MMC as modules (`=m`)** instead of `=y` to keep the Image small - then
  `fbmem.ko`/`mmc_core.ko` must ship in the rootfs and load before our modules (and our
  out-of-tree build needs their `Module.symvers`). Cleanest for size.
- Keep `=y` and re-trim elsewhere to stay under 6 MB.
- Accept a larger Image (only matters for *flashing*; the RAM `bootm` path doesn't care).
Check the resulting Image size against the partition before flashing.

## 4. Device tree - status

- **Already present + confirmed on the running device:** `scaler@8840000`
  (`artosyn,scaler`), `ar_mpp@8870000` (`artosyn,ar_mpp`), `ar_mpp_proc`, and the MMZ
  `reserved-memory` carveout `mmz@29400000` (nomap). `ar_mpp_drv`/`ar_scaler` probe fine.
- **DONE (AR8030 host-controller + reset):** the `ar_dtbo_sdio` DT overlay
  adds the `mmc@1b00000` node (`non-removable`, `cap-sdio-irq`), the open
  `dw_mci-artosyn` glue drives the Proxima-9311 DesignWare mmc IP, and `artosyn_gpio`
  releases the GPIO23 reset. With those, the AR8030 enumerates as a full SDIO card
  (device `0x8030` = ROM mode) and binds by VID/PID `4152:8030/8031`. The original
  SDIO-silence blocker was the `artosyn_gpio.c` bank-base off-by-one (0xC0 -> 0xBC);
  see `../docs/artosyn-gpio.md`. Recipe: `HW-BRINGUP.md` (Phase 6).
  **Known gap:** `ar_dtbo_sdio` is not in `modules/Kbuild` and has no `.c` source
  in this tree - it was hand-built from the bring-up overlay source
  (`mmc-sdio-overlay.dts`) during the bring-up session. Needs wiring into the tracked build (e.g. the same way
  `ar_mpp_overlay.c` now is) before this recipe is reproducible from a clean checkout.
- **DONE (status LED) - it is NOT a GPIO.** RE of `ar_lowdelay` (`customerHmLed*`) showed the front status LED is an addressable RGB LED (WS2812 / SK6812 family) driven over **SPI** via `/dev/spidev32765.0`, not a `a10a000.gpio` pin and not a PWM. The SPI master is the DesignWare `snps,dw-apb-ssi` at `0x1102000` (mainline `dw_spi_mmio`), exclusive to the LED (the display panel uses DSI + GPIO + PWM, not this bus); the vendor's worker thread sends ~72-byte `SPI_IOC_MESSAGE` frames at 6.25 MHz and blinks in software. Open-kernel side is implemented: `CONFIG_SPI_DESIGNWARE`/`CONFIG_SPI_DW_MMIO`/`CONFIG_SPI_SPIDEV` (`configs/spi.config`), the `spi@1102000` controller + spidev child in the DT, and `test_tools/led_test.c` drives WS2812 frames over the resulting `/dev/spidevN.0` (byte-identical encoding to the vendor's). Remaining: wiring real product logic (colour/pattern reflecting actual link state) instead of `led_test`'s manual rainbow/solid-colour/off controls. Full writeup: `../docs/status-led.md`.
- **DONE (buttons) - they are an ADC voltage ladder, not GPIO keys.** RE of the reconstructed vendor kernel showed the buttons are a resistor ladder on ADC channel 0 of the `artosyn,adc` SAR ADC (`0x0a108000`), decoded by the in-kernel `adc-keys` driver into evdev events on `/dev/input/event0` (NOT `gpio-keys`). Implemented for the open kernel: the `artosyn_adc.c` IIO provider (this dir), `configs/input.config` (IIO + adc-keys + evdev as modules), and the `adc` + `adc-keys` DT nodes in `dts/proxima-9311.dts`. Register protocol + ladder: `../docs/artosyn-adc.md`. On-device validation tool: `test_tools/button_test.c` (reads `/dev/input/event0`, prints press/release per button) - run it to confirm rather than generic `evtest`.

## 5. Matching the build

With IKCONFIG on, build-matching is self-serve:
1. `zcat /proc/config.gz` from the device = the authoritative config.
2. Build a kernel tree with that config (deterministic - same `pin.env` metadata) and
   build the modules against it.
3. Sanity-check WITHOUT crashing: `comm` the device `/proc/kallsyms` names against the
   tree's `System.map` - must match exactly (KASLR slides addresses, not names).
4. Load dynamically (insmod from `/run`, never persistent), with a serial capture
   (`cat /dev/ttyACM1 | tee /tmp/ml-hw-logs/serial-$(date +%s).log &`) running so any
   panic is logged even if SSH hangs.

## 6. ar_osal MMZ mapping

`ar_osal` maps MMZ blocks **on demand** (init does not ioremap the whole 108 MiB
carveout). `memremap(MEMREMAP_WC)` on the nomap `mmz@29400000` region is
hardware-validated (Tier 0).

## 7. SDIO CMD tracing (diagnostic build)

For SDIO diagnosis, build with `DEBUGSDIO=1 ./scripts/build.sh` (merges
`configs/debug-sdio.config` last, gated on that env var; verify
`CONFIG_DYNAMIC_DEBUG=y` via `/sys/kernel/debug/dynamic_debug/control`), then enable
the traces on the device:
```
mount -t debugfs none /sys/kernel/debug    # if needed
echo 'module dw_mmc_core +p' > /sys/kernel/debug/dynamic_debug/control
echo 'module mmc_core +p'    > /sys/kernel/debug/dynamic_debug/control
insmod dw_mci-artosyn.ko    # triggers the scan; dmesg shows the CMD trace
```
The resolved AR8030 enumeration root cause (gpio bank-base off-by-one) is owned by
`../docs/artosyn-gpio.md`.
