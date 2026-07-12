# GPIO controller (`artosyn,gpio`)

How the SoC's GPIO controller works at the register level and how the open kernel drives it. There is no mainline driver, so the open stack registers an out-of-tree gpiochip (`../modules/artosyn_gpio.c`) that the AR8030 RF reset, the panel straps/rails, and the SD I/O rail all depend on. Each fact is tagged **[confirmed]** (direct evidence in the disassembly/code/DT), **[inferred]**, **[open]**.

## Architecture

```
artosyn,gpio (gpio@a10a000, one node, 7 banks, lines 0-109 vendor global numbering)
  per-bank block @ base + 0xBC + N*0xC:  SET @+0 (output) / DIRIN @+4 (1=input) / DAT @+8 (input)
  direction + output latch are software-shadowed (both registers are write-only/unreadable)
```

The controller is a single MMIO node at `0x0A10A000` presenting seven banks; the driver registers one gpiochip per bank and a custom `of_xlate` so DT references use the vendor global line numbers. It is get/set/direction only, no IRQ support.

## Why a driver is needed [confirmed]

Ad-hoc `/dev/mem` pokes do not work: the direction register (DIRIN) is effectively write-only, so you cannot read direction back, and a naive read-modify-write of the SET latch corrupts other pins (a SET readback mixes in the live levels of input pins). A proper gpiochip with software shadows is required to drive the AR8030 reset (GPIO23) and the panel lines safely.

## Register model [confirmed]

MMIO base `0x0A10A000`, DT `reg = <0 0x0a10a000 0 0x2000>`. Per bank N the register block is at `base + 0xBC + N*0xC` (`AR_BANK_REG_BASE = 0xBC`, `AR_BANK_STRIDE = 0xC`):

| Offset | Name | Access | Meaning |
|--------|------|--------|---------|
| `+0x0` | SET | W (shadowed) | output-value latch |
| `+0x4` | DIRIN | W (shadowed) | direction, `1` = input |
| `+0x8` | DAT | R | input read |

The base is `0xBC`, **not** `0xC0`. A `0xC0` base is off by one 32-bit register, so a SET write lands on DIRIN and driving any pin (for example the AR8030 reset) flips its direction to input instead of driving the level. The pinmux region (`0x00..0x3c`) is owned by the bootloader and is not touched.

### Software shadows [confirmed]

Because DIRIN and SET cannot be safely read back, `struct ar_gpio_bank` mirrors both:

- **`dir_shadow`** - DIRIN is write-only, so direction lives in software. Seeded to all-input (`0xffffffff`, the safe default) at probe; `get_direction` reports from it; `direction_input`/`direction_output` set/clear the bit and write DIRIN.
- **`out_shadow`** - `set` drives purely from the shadow and never read-modify-writes SET (that would corrupt input pins). Seeded from the current SET latch at probe, so the first write preserves whatever the bootloader/regulators already drive.

## Bank layout and `of_xlate` [confirmed]

Seven banks, `{base, ngpio}` = `{0,23} {23,22} {45,26} {71,6} {77,6} {83,11} {94,16}` = lines 0-109, matching the vendor global numbering. Probe sets each gpiochip's `base`/`ngpio` and logs "registered 7 gpio banks (0-109)".

Because all banks share one DT node, a custom `of_xlate` (`#gpio-cells = 2`) takes the vendor **global** line number in `<&gpio N flags>`: gpiolib tries each bank until the one whose `[base, base+ngpio)` covers N returns `N - base`. This lets the panel/RF DT nodes use the same numbers as the vendor scripts.

## Consumers [confirmed]

| Global GPIO | Bank/line | Function | How driven |
|-------------|-----------|----------|------------|
| 23 | bank1 line0 | AR8030 RF reset, active-low (drive low->high to release) | via gpiolib during SDIO/RF bring-up (`artosyn-sdio.md`) |
| 42 | bank1 line19 | panel source/horizontal scan-direction strap | `gpio-hog` `output-low` (`panel_scan_h_hog`), before panel-power |
| 43 | bank1 line20 | panel reset/enable | `reset-gpios = <&gpio 43 ...>` on the panel node |
| 95 | bank6 line1 | panel second rail/enable | `enable-gpios = <&gpio 95 ...>` on the panel node |
| 100 | bank6 line6 | panel-power rail (its rising edge samples the scan straps); also the SD **vqmmc** I/O rail (fixed 3.3 V) | `gpio-hog` `output-high` (`panel_power_hog`); the SD side is currently hand-driven high before the card scan (`sd-card.md`) |
| 107 | bank6 line13 | panel gate/vertical scan-direction strap | `gpio-hog` `output-low` (`panel_scan_v_hog`), before panel-power |

Ordering matters: straps 42 and 107 must be low before the panel-power line (100) rises. gpio-42 is in bank1 (registered before bank6); gpio-107 shares bank6 with gpio-100 and its hog is listed first so it applies before the power hog. The panel power-on sequence (`display-backlight.md`) is `gpio95=0; gpio43=0; 10ms; gpio43=1; 10ms; gpio95=1; 10ms`, power-off `gpio43=0`.

## Driver ops [confirmed]

The gpiochip wires `get`, `set`, `direction_input`, `direction_output`, `get_direction`, and `of_xlate`; `can_sleep = false`. There are no `request`/`free` callbacks and **no IRQ support** (no irqchip, no `to_irq`, no `interrupt-controller`) - get/set/direction only. Registered via `devm_gpiochip_add_data`, matched on `"artosyn,gpio"`, driver name `"artosyn-gpio"`, built as a module.

## Status [confirmed on hardware]

Working on hardware. There is no standalone GPIO status row, but every peripheral it underpins is validated: the display/panel (the scan-direction strap fix on GPIO 42/107 is proven on hardware), the SD card (vqmmc = GPIO 100), and the live RF video path through the AR8030 (which needs the GPIO23 reset). The controller is also why `/dev/dri/card0` only appears once `artosyn_gpio` is loaded (the panel reset line is on this controller, else the panel/DSI/VO defer). See `../STATUS.md` and `../PERIPHERALS.md`.

## Source

`../modules/artosyn_gpio.c`; the DT nodes in `../dts/proxima-9311.dts` (the `gpio@a10a000` node plus the panel hogs and `reset-gpios`/`enable-gpios`) and the duplicate in `../modules/ar_dtbo_sdio.dts`; cross-refs `../modules/BOARD-CONFIG.md`, `display-backlight.md`, `sd-card.md`, `artosyn-sdio.md`.
