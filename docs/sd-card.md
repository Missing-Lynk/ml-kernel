# microSD card (mmc1 over the DesignWare MMC host)

How the microSD slot is brought up on the open kernel. The card runs on the **unmodified mainline `dw_mmc` core** plus a small Artosyn platform-glue module (`../modules/dw_mci-artosyn.c`) that programs the SoC-specific clock tap/phase and CGU gates. The same glue also serves the AR8030 RF link on `mmc@1b00000`; that device is `artosyn-sdio.md`, and only the shared clock/glue facts are repeated here. Each fact is tagged **[confirmed]** (on hardware or in the disassembly), **[inferred]**, **[open]**.

## Architecture

```
microSD card <- SD 4-bit -> mmc@1c00000 (mmc1, DesignWare MMC host, IRQ GIC_SPI 49)
  clock: SEL/CFG taps + CGU gates programmed by dw_mci-artosyn
  core:  mainline dw_mmc (dw_mci_pltfm_register), unmodified
  power: vqmmc I/O rail = GPIO 100 (port6.6, fixed 3.3 V)
```

The card runs on the stock `dw_mmc` core; the glue is a `dw_mci_drv_data` hook in the exact shape of mainline's own `dw_mmc-<soc>.c` glues. Throughput sits at the SD-High-Speed bus ceiling and matches stock; reads, writes, and in-kernel exFAT under a DVR-style workload are all validated.

The bring-up scaffolding is graduated: the mmc nodes live in `../dts/proxima-9311.dts` (the `ar_dtbo_sdio` runtime overlay is retired), the vqmmc rail is held by the DTS `panel_power_hog` (a shared rail, see Power below), and card-detect is native CDETECT (no `broken-cd` poll).

## Host node: `mmc@1c00000` (mmc1) [confirmed]

The SD node lives in `../dts/proxima-9311.dts` (`mmc@1c00000`; graduated from the retired `ar_dtbo_sdio` runtime overlay). Node properties:

- `compatible = "dwmmc1", "artosyn,proxima-dw-mshc"`, `reg = <0 0x1c00000 0 0x1000>`, `interrupts = <0 49 4>` (GIC_SPI 49 = hwirq 81, vendor `0x31`).
- Same DesignWare IP as the RF host `mmc@1b00000`, different instance and clock registers. Flags differ from the RF node: `no-mmc` + `no-sdio` (so the core enumerates an inserted SD card, not an SDIO function), no `cap-sdio-irq`, no `broken-cd` (native card-detect, see below).

On the open 6.18 kernel the card enumerates as `mmcblk1` (or `mmcblk0`/`mmcblk1` depending on whether the RF side is up). Note that stock (vendor 4.9) and some product docs call it `mmcblk2p1`; that is the vendor enumeration order, not the open kernel's.

## Clock glue: `dw_mci-artosyn` [confirmed]

The glue exists because the SoC has no mainline clock driver and U-Boot never used these hosts, so the generic `snps,dw-mshc` binding leaves the bus unclocked. It programs the clock byte-faithfully to the vendor `dw_mci_proxima_init`, then registers on the mainline core with `dw_mci_pltfm_register` (driver name `dwmmc_artosyn`, `.pm = dw_mci_pltfm_pmops`). `ar_dwmmc_parse_dt` selects the register set by the node's physical base: mmc1 (SD) uses `AR_SDMMC1_CLK_SEL = 0x0A1080C0` and `AR_SDMMC1_CLK_CFG = 0x0A1080C4` (mmc0/RF uses `0x0A108088`/`0x0A10808C`).

The clock model [confirmed]:

- **SEL** is the bus_hz divider tap: 25 MHz = `0x00`, 50 = `0x40`, 100 = `0x80`, 200 = `0xC0`. Default `AR_CLK_SEL_VAL = 0x80`.
- **CFG** carries the phase in its low 5 bits: `writel((cfg & ~0x1f) | phase)`, init phase 0. The vendor `ar_config_cm_dll_phase` table is identity.
- `ar_dwmmc_init` reads CFG, writes `(cfg & ~0x1f) | phase`, `dsb(st)`, writes SEL, `udelay(200)`. It writes **only** CFG and SEL.
- **Never write `0x0A108000` or `0x0A108038`.** Forcing `0x0A108000 = 0x81` (a value from unreadable slot-A `/dev/mem` reads) corrupts the clock so the card gets commands but never frames a valid CMD8 response; leaving `0x0A108000` at 0 (its U-Boot value) is what makes the card enumerate.
- The CGU source-clock gates `0x0A104024` (bit 22) / `0x0A104028` (bit 23) are mapped by the glue; whether it actively sets them or relies on U-Boot is **[open]** (see `artosyn-sdio.md`, same registers).

Module params (all `0444`, debug knobs): `clk_sel` (-1 = use the bus_hz mapping), `clk_cfg` (-1, phase masked `& 0x1f`), `bus_hz` (0 = keep the DT value, else override the divider for sweeps). The committed defaults are the mapping above (`0x80`/phase 0, the stock-faithful values from the slot-A register diff); the module loads param-less. The `0x87`/`0x02` pair the early bring-up passed explicitly originated from misread slot-A dumps and is retired from the shipped modprobe config (a restore line is documented in `rootfs/skeleton/etc/modprobe.d/ml.conf` should a host fail to enumerate).

## Power: vqmmc = GPIO 100 [confirmed]

The SD I/O rail (vqmmc) is GPIO 100 (port6.6, a fixed 3.3 V supply) - the SAME line as the panel power rail: one shared 3.3 V rail switch. The DTS `panel_power_hog` drives it high at `artosyn_gpio` probe, which is what powers the SD bus on every boot. It is deliberately NOT described as a `vqmmc-supply` regulator: the hog owns the line (gpiolib refuses a second claim), and the mmc core must never power-cycle a rail the panel shares. Card power (vmmc) may have its own switch GPIO or be hardwired - **[open]**.

## Card detect [confirmed]

The DTS node has no `broken-cd`: the core uses the DesignWare controller's native CDETECT, like stock (bring-up proved the line: CDETECT reads 0 with a card present). The retired overlay node polled at 1 Hz via `broken-cd`; if the native line turns out unreliable in the enclosure, `broken-cd` returns with a comment stating why.

## Performance [confirmed]

SD High-Speed, 50 MHz, 4-bit, 3.3 V signaling, confirmed via debugfs `mmc1/ios`. Throughput is ~21-22 MB/s O_DIRECT both directions (16 MiB O_DIRECT write 781 ms, read-back 762 ms; `dd` 64 MiB O_DIRECT read 22.4 MB/s) = the SD-HS bus ceiling, matching stock. UHS is physically unreachable: it needs a 1.8 V vqmmc and the rail is a fixed 3.3 V GPIO, and stock ships the same `cap-sd-highspeed`-only config, so this is the hardware limit, not a driver gap. The HS/UHS `execute_tuning` glue stub is therefore dormant by design. Whether the core runs IDMAC or PIO on this platform is **[open]** (a one-boot check via the dmesg "Using internal DMA controller" line / `/proc/interrupts`).

exFAT is built in (`../configs/exfat.config`, `CONFIG_EXFAT_FS=y`); a DVR-style workload (paced ~2 MB/s and full-speed 1 GiB writes, every block md5-verified, clean unmount) is validated on slot B with zero exfat/mmc errors.

## `artosyn_mmc.c`: a diagnostic tool, not a driver [confirmed]

`../modules/artosyn_mmc.c` is a standalone Stage-1 DW-MSHC host driver written when the mainline core was still suspect: on insmod it ioremaps the controller (default base `0x01c00000`), runs a polled CMD0 -> CMD8 (arg `0x1aa`, R7) -> ACMD41/CMD2/CMD3 (or CMD5 for SDIO) self-test, reads CDETECT at `0x50`, and returns 0 to stay loaded for register inspection. Nothing binds it (no `platform_driver`, no `of_match`); it never grew a data path. Keep it as the controller-level probe it is (a load-time CMD-sequence self-test), and read it as a diagnostic, not the SD path.

## Source

Module sources `../modules/dw_mci-artosyn.c` (glue), `../dts/proxima-9311.dts` (the `mmc@1c00000` node), `../modules/artosyn_mmc.c` (diagnostic); the validation method `../modules/VERIFICATION.md`; the peripheral map `../PERIPHERALS.md` and board config `../modules/BOARD-CONFIG.md`.
