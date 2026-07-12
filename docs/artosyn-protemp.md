# Artosyn protemp (SoC temperature sensor)

The `artosyn,protemp` SoC temperature sensor, exposed by the open kernel as a standard IIO sensor. It is a small analog block at MMIO base `0x0a108000` (DT `temperature@0a108000`, `reg = <0xa108000 0xd4>`) sharing the SAR ADC's window (`reg = <0xa108000 0x21c>`), but a separate engine: a free-running 9-bit code at `0xcc`/`0xd0`, a linear code-to-Celsius conversion, and none of the ADC's `SAMPLE_EN` gate / warm-up / `DATA@0x210` machinery. The driver (`modules/artosyn_protemp.c`) exposes one `IIO_TEMP` channel. Facts are tagged **[confirmed]** (in the disassembly), **[inferred]**, or **[open]**.

## The two vendor temperature drivers [confirmed]

The vendor `vmlinux` carries two temperature platform drivers, split by `driver.name` / compatible:
- **`artosyn,protemp`** (`driver.name = "protemp"`): the simple free-running variant (`artosynts_probe` @`0x84aed18`, `artosynts_read_raw` @`0x84aec60`, `artosynts_get_raw.isra.2` @`0x84aebf8`). The only one the goggle DT instantiates (`temperature@0a108000`), so it is what `artosyn_protemp.c` reimplements.
- `artosyn,temperature` (`driver.name = "temperature"`): a heavier variant (`artosynts_probe` @`0x84ae450`) with three clocks (`ts_pclk`/`ts_clk`/`ts_bg_clk`), a threaded IRQ, an IIO triggered buffer, a 12-bit multipoint sampler (`DATA@0x20`), a 34-entry table conversion, and a trimmed-mean reduction (`artosynts_partial_average`). No DT node has this compatible, so it never binds: dead code on this board (its different register map points to a separate on-die sensor or SoC revision). Two of its ideas were adopted anyway (see Reduction).

Both ship because this is a shared Proxima-9311 BSP (the DTS is a `Proxima Development Board`): one image across the product line, with each board's DT selecting what binds.

> The BetaFPV HD goggle wires up only the simple sensor.

## Register map [confirmed]

Offsets from base `0x0a108000`:

| Offset | Name | Access | Meaning |
|--------|------|--------|---------|
| `0x00` | CTRL | RMW | probe sets bit0 (enable). |
| `0x38` | CFG | RMW | probe sets bits[6:4] (`0x70`); analog enable / mode. **[inferred]** exact meaning. |
| `0xcc` | DATA_LO | R | low 8 bits of the code. |
| `0xd0` | DATA_HI | R | bit0 = bit8 of the code. |

Each sample is `code = (readl(0xcc) & 0xff) | ((readl(0xd0) & 1) << 8)`, a 9-bit value (`dsb ld` after reads, `dsb st` around probe writes). No gate register: the block free-runs and `0xcc`/`0xd0` always hold the latest code.

## Read and conversion [confirmed]

`artosynts_read_raw` takes the mutex, then per `mask`: RAW (`0`) returns the averaged 9-bit code; SCALE (`2`) returns the converted temperature in whole degrees C; else `-EINVAL`. SCALE is **non-standard IIO**: `in_temp_scale` carries the whole temperature, not a factor (read it directly as degrees C). Averaging reads `samp-count` codes back-to-back (no delay/warm-up/status poll) then reduces them (see Reduction). The device registers as `/sys/bus/iio/devices/iio:deviceN` with `name = temperature`; resolve it by the `name` attribute, not a fixed path (modern IIO does not honour `dev_set_name` for the directory).

Conversion: `temperature_C = (raw*5320 - 1373400) / 10000`, whole degrees, taken bit-exact from the vendor SCALE path. The vendor does the `/10000` as `* 0x346dc5d63886594b >> 75`, and that constant is exactly `ceil(2^75/10000)`, confirming the divisor. Examples: `300 -> 22`, `320 -> 32`, `400 -> 75`. The vendor's unsigned reciprocal underflows below `raw ~260` (< 0 C); `artosyn_protemp.c` uses signed division so it degrades to a small negative instead (**[inferred]**, sub-0 C only).

Channel spec (from const data): `IIO_TEMP`, `channel 0`, `scan_type { u, realbits 12, storagebits 16 }`, `info_mask_separate = RAW | SCALE`. No PROCESSED / OFFSET / SAMP_FREQ.

## Probe and the shared MMIO window [confirmed]

`artosynts_probe`: `devm_iio_device_alloc` + `mutex_init`; `platform_get_resource` + `devm_ioremap`; read `samp-count` (default 1); enable (`CTRL |= bit0`, `CFG |= 0x70`); `dev_set_name("temperature")`; register. No clocks, IRQ, or buffer: the block is already clocked, so the sibling variant's `ts_*`/bandgap clocks are not needed here.

The `devm_ioremap` (not `devm_platform_ioremap_resource`) is deliberate: the protemp window `[base, +0xd4]` overlaps the ADC window `[base, +0x21c]`, which `artosyn_adc.c` already claimed exclusively via `request_mem_region`, so an exclusive protemp map would fail `-EBUSY`. `devm_ioremap` ignores the resource tree, so both coexist. Their register sets do not collide, and although both RMW `0x00`, the ADC sets bit7 and protemp bit0.

## Divergences from artosyn_adc.c

- DATA is a 9-bit code at `0xcc`/`0xd0` (no variant select), vs the ADC's 10-bit `0x210`.
- No sampling gate or warm-up reads; the code free-runs.
- Linear conversion, vs the ADC's SCALE / vendor 2-point table.

## Reduction (trimmed mean) [inferred]

For `samp-count > 3`, `artosyn_protemp.c` takes a trimmed mean (`sort()`, drop the lowest and highest eighth, mean the middle) instead of the plain mean the `artosyn,protemp` binary uses. This is the vendor sibling's `artosynts_partial_average` behavior and rejects outliers better at the DT's `samp-count = 8`. `<= 3` falls back to plain mean. Intentional; reads and conversion are otherwise unchanged.

## Open items

- **Calibration is the vendor's, not ours.** The conversion is bit-exact from the vendor, so for the same raw code we print exactly what stock prints; there is no separate conversion accuracy to verify. Whether the vendor's own `355 -> 51 C` is physically correct is its calibration, which we mirror and cannot independently check or fix.
- **`CFG (0x38)` bits `0x70` not decoded.** Reproduced verbatim; reads are valid with it as-is.

## Source

Disassembly of the vendor temperature drivers in the reconstructed vendor kernel (their `platform_driver` / `of_device_id` const data) and the vendor DT (`temperature@0a108000`).
