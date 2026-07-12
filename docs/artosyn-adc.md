# Artosyn ADC (SAR ADC + button voltage ladder)

How the `artosyn,adc` SoC ADC works at the register level, recovered so the open kernel can drive the buttons. The buttons are a resistor ladder on a single ADC channel read by the in-kernel `adc-keys` driver; the ADC itself has no mainline driver, so the open stack needs an out-of-tree IIO provider (`modules/artosyn_adc.c`). Each fact below is tagged **[confirmed]** (direct evidence in the disassembly), **[inferred]** (consistent but not proven on hardware), **[open]** (still to be determined).

## Summary

The ADC is a SAR ADC at MMIO base `0x0a108000` (DT `reg = <0xa108000 0x21c>`), 8 channels, 10-bit (full-scale code `0x3ff`) - this is a fixed property of the SoC silicon, the same on every Proxima-9311 device. The same block also backs the `artosyn,protemp` temperature sensor (DT `temperature@0a108000`, `reg = <0xa108000 0xd4>`). The driver exposes IIO channels; `adc-keys` polls channel 0 every 10 ms and maps voltage bands to key codes.

The **eight buttons** (back/record/bind/up/down/left/right/enter) are a **BetaFPV VR04-specific board fact**, not a SoC one: they are how this goggle's keypad happens to be wired onto one ADC channel as a resistor ladder (see the `adc-keys` node in the vendor DTS, codes 65-87, and the "Button ladder" section below).

A different Artosyn product (a VRx with fewer/more buttons, an air unit with just a bind button, or a goggle that wires its keypad differently) needs **zero driver-code changes** - `artosyn_adc.c` just exposes the 8 raw ADC channels as a generic IIO provider and knows nothing about buttons. Porting is entirely a **device-tree change**: edit that other board's `adc-keys` node (`press-threshold-microvolt` / `linux,code` lists - fewer or more entries, different thresholds) and, if the ladder sits on a different physical channel, the `io-channels` reference. This mirrors the general board-porting story in `modules/BOARD-CONFIG.md`: the ADC's own register base/IRQ could in principle differ too (also a DT-only change), though on Proxima-9311 parts it is likely fixed at `0x0a108000` since it is on-die SoC IP, not a discrete board component like the AR8030 reset GPIO.

## Register map [confirmed]

Offsets are from the MMIO base `0x0a108000`. The DATA and reset-register offsets are chosen by a SoC variant check (below); the values here are for the **9311 variant**.

| Offset | Name | Access | Meaning |
|--------|------|--------|---------|
| `0x00` | CTRL/ENABLE | RMW | `adc_probe` sets bit7 (`0x80`) to enable/power the ADC. |
| `0x18` | SAMPLE_EN | W | Write `0` to begin a sample burst, `1` when the burst is done (also written `1` once at probe). Acts as a start/hold gate around the data reads. |
| `0x24` | CHAN_SEL | W | Write the channel index (0..7, masked `0xff`) before sampling. `adc_read_raw` writes this from `chan->channel`. |
| `0x2c` | CALIB_CTRL | RMW | bit3 (`0x8`) = calibration enable. Set only around the per-channel calibration reads in probe; cleared otherwise. |
| `0x200` | DATA_RESET | W | Written `0` at init to clear the data path. (9341 variant: `0x800`.) |
| `0x210` | DATA | R | 16-bit sample output, read once per sample. (9341 variant: `0x810`.) |

All accesses are wrapped in `dsb` barriers in the vendor driver (`dsb st` before writes, `dsb ld` after the data read).

### SoC variant select [confirmed]

`adc_probe` computes the ioremap window size `size = res.end + 1 - res.start` and picks the register layout: `size > 0x80f` selects the "9341" type (id `0x9341`, DATA `0x810`, reset `0x800`); otherwise the "9311" type (id `0x9311`, DATA `0x210`, reset `0x200`). The id is stored in the driver state and re-checked in `adc_REG_DATA_get_data` and at init. This board's DT window is `0x21c` (`< 0x80f`), so it is the **9311** layout: **DATA at `0x210`, reset at `0x200`**.

## Sample sequence (`adc_get_data`) - the vendor's method [confirmed]

This is what the vendor driver actually does (recovered from disassembly, not a general best-practice writeup); see "Implications for the open driver" below for how much of it the open driver reproduces vs. simplifies. For one conversion of the currently selected channel:

1. `dsb st`, then write `0` to `SAMPLE_EN` (`0x18`).
2. Do 16 reads of `DATA` (`0x210`) and discard them (warm-up / pipeline flush).
3. Read `N = samp-count` samples from `DATA` into a small buffer (DT `samp-count`, default 1).
4. Write `1` to `SAMPLE_EN` (`0x18`).
5. Reduce the buffer to one 16-bit value:
   - `N <= 3`: plain mean (`sum / N`).
   - `N > 3`: sort the buffer (`sort()` with `adc_compare`), drop the lowest and highest eighth, mean the middle (a trimmed mean).

`adc_REG_DATA_get_data` is the single-read helper: it selects DATA at `0x210` (or `0x810` for id `0x9341`), reads the 16-bit value, `dsb ld`.

## read_raw masks [confirmed]

`adc_read_raw(st, chan, *val, *val2, mask)` takes the driver mutex (`st+0x4a8`), writes `chan->channel` to `CHAN_SEL` (`0x24`) for the RAW/PROCESSED paths, then:

- `mask == 0` (IIO_CHAN_INFO_RAW): returns the averaged ADC code from `adc_get_data`.
- `mask == 1` (IIO_CHAN_INFO_PROCESSED): returns a calibrated value in microvolts, computed by interpolating the averaged code against the per-channel 2-point calibration table built in probe.
- `mask == 4` / `5` (CALIBSCALE / CALIBBIAS): per-channel calibration-table access.
- anything else: `-EINVAL`.

## Probe init sequence [confirmed]

`adc_probe`:

1. `devm_iio_device_alloc`, init mutex at `st+0x4a8`.
2. `platform_get_resource(IORESOURCE_MEM, 0)` + `devm_ioremap` -> MMIO base stored in driver state.
3. Variant select from the window size (above); store id (`0x9311` here).
4. `of_property_read_variable_u32_array` for `channels` (channel count) and `samp-count` (sample count, default 1 if absent).
5. Initialise the 8 IIO channel specs and seed each channel's calibration table with reference points (`1024`/`2025` and `900`).
6. Hardware bring-up: write `0` to reset reg (`0x200`); clear CALIB_CTRL bit3; write `1` to `SAMPLE_EN` (`0x18`); set CTRL bit7 (`0x80`) at `0x00`.
7. For each of the 8 channels: enable calibration (set CALIB_CTRL bit3), `adc_get_data`, insert `(code, 900)` into that channel's calibration table, disable calibration. (This self-measures each channel against an on-chip `900` reference; see "Calibration" below.)
8. `dev_set_name`, `iio_device_register`.

## Calibration [inferred]

The per-channel table holds up to 10 sorted `(code, value)` points (`adc_calibration_insert`, 40-byte stride per channel at `st+0x4fc`). Probe seeds reference points (`1024`/`2025`, `900`) and then measures each channel once with calibration enabled, inserting `(measured_code, 900)`. PROCESSED reads interpolate between the nearest two points (the math in `adc_read_raw` uses the constant `0xf4240 = 1000000`, i.e. microvolts per volt). The exact physical meaning of `2025`/`900` (millivolts of two on-chip references) is **[open]**; reproducing the vendor calibration bit-exactly is not required for the buttons.

## Implications for the open driver

For `adc-keys` the only requirement is that channel 0 reports a voltage that lands each button in its DT threshold band. `artosyn_adc.c` (`ar_adc_sample`) keeps the parts of the vendor sequence that are electrically load-bearing - channel-select before sampling, the `SAMPLE_EN` 0-then-1 gate around the burst, and the 16 warm-up reads - since those look like real hardware requirements (pipeline flush), not vendor style choices. It drops the trimmed-mean reduction for `samp-count > 3` (the open driver defaults `samp-count` to 1, so it never engages).

It **does** reproduce the per-channel 2-point calibration (`ar_adc_calibrate` at probe self-measures each channel against the on-chip reference -> `code_ref`; `ar_adc_code_to_mv` interpolates the line `(code_ref, CAL_REF_MV) .. (1024, full-scale)`), exposed as `IIO_CHAN_INFO_PROCESSED` alongside RAW and a gain-only SCALE. This was needed once a reading finer than the button ladder was wanted: the battery gauge reads channel 1's PROCESSED and applies the board divider, and the gain-only SCALE dropped the per-channel offset (a real ~22 mV at the pin, ~0.46 V at the 2S pack), which showed up as a low battery reading. Two constants are bench-corrected from the vendor nominals: full-scale `2025 -> 2113 mV`, and the on-chip reference `900 -> 894 mV` (both from the 3-point bench line `pin_mV = 2.043*raw + 21.74`; the full-scale derivation is in "Possible improvements" below). `ar_adc_calibrate` logs each channel's measured `code_ref` at probe (`dmesg`) so the line stays verifiable. **[confirmed on hardware]**

## Button ladder (from the vendor `adc-keys` node)

Channel 0, 10 ms poll, `keyup-threshold-microvolt = 0`. Press thresholds (microvolts) and key codes:

| Key | code | press-threshold (µV) |
|-----|------|----------------------|
| bind | 73 (`0x49`) | 200000 |
| back | 66 (`0x42`) | 399968 |
| record | 77 (`0x4d`) | 601024 |
| up | 87 (`0x57`) | 800000 (`0xc3500`) |
| left | 65 (`0x41`) | 1000000 |
| right | 68 (`0x44`) | 1200000 |
| enter | 69 (`0x45`) | 1398016 |
| down | 83 (`0x53`) | 1550848 |

(`up` renders as the escaped string `"\0\f5"` in the decompiled DTS; the bytes are `00 0c 35 00` = `0xc3500` = 800000 µV, which makes the ladder a clean ~200 mV-per-step progression.)

## Possible improvements

Where `artosyn_adc.c` could go beyond a faithful-enough reimplementation:

- **The `artosyn,protemp` temperature sensor is ported (`modules/artosyn_protemp.c`).** It is a sibling driver on the shared `0x0a108000` window: a separate engine from the SAR ADC (temperature code at `0xcc`/`0xd0`, linear code-to-Celsius), so extending `artosyn_adc.c` was not the fit. The register-level RE and the code-to-Celsius conversion are written up in `artosyn-protemp.md`. It exposes an `IIO_TEMP` channel whose `in_temp_scale` carries the whole temperature. Working on slot B (registers `iio:deviceN`, `name = temperature`); full register-level writeup and status in `artosyn-protemp.md`.
- **The full-scale reference (`AR_ADC_FULLSCALE_MV`) is a hardcoded C constant, not DT-configurable**, unlike the rest of the driver (`channels`, `samp-count` are both DT properties). A different board with a different reference voltage would need a code edit instead of a DT edit. A `full-scale-millivolt` (or similar) DT property with the default would close this gap. The value was bench-measured on the battery channel (3 points 6.0/7.1/8.4 V through the ~20.7 divider, linear to <16 mV, both ends back-solving to the same value): **2113 mV**, not the nominal 2025. Caveat: full-scale and the board divider trade off against each other from pack-voltage data alone; if the divider is measured directly (meter the divider tap, or read the resistors), re-derive full scale from the same points to split them cleanly.
- **The trimmed-mean reduction for `samp-count > 3` is dormant, not implemented.** `ar_adc_sample()` always computes a plain mean regardless of `N` ("Implications for the open driver" above). The now-ported protemp driver (previous bullet) does implement the real trimmed-mean (sort, drop the lowest/highest eighth, mean the middle) for its `samp-count = 8`; `artosyn_adc.c` still does not, which is only a latent concern for the ADC channels if its DT `samp-count` (currently 4) is relied on for a low-noise reading.

## Source

Disassembly of the vendor ADC driver in the reconstructed vendor kernel, the vendor device tree (the `adc@0a108000` and `adc-keys` nodes), and live captures.
