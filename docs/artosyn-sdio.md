# SDIO RF link (AR8030 baseband over the DesignWare SDIO host)

How the AR8030 RF baseband is brought up on the open kernel as an SDIO device, and how its IP-over-SDIO netdev (`sdio0`) works at the driver level. This is the kernel/transport layer only; the RF association, the `:10000`/`:10001` UDP handshake, and the video downlink protocol are the product story in `../../docs/reference/rf-video-downlink.md`. Each fact is tagged **[confirmed]** (direct evidence in the disassembly, DT, or on hardware), **[inferred]** (consistent but not proven), **[open]** (still to be determined).

## Architecture

```
AR8030 baseband <- SDIO -> mmc@1b00000 (mmc0, DesignWare SDIO host, IRQ GIC_SPI 48)
  reset: GPIO23 (a10a000.gpio, active-low) driven by artosyn_gpio
  clock: SEL/CFG taps + CGU gates programmed by dw_mci-artosyn (mainline dw_mci core)
  data:  artosyn_sdio -> firmware upload, then /dev/artosyn_sdio + sdio0 (IPv4-over-SDIO netdev)
```

Three out-of-tree modules bring it up, in load order [confirmed] (`../modules/HW-BRINGUP.md` Phase 6 used a runtime overlay for the DT nodes, since graduated into the DTS):

1. **`artosyn_gpio`** - binds `gpio@a10a000`, drives the AR8030 active-low reset on GPIO23.
2. **`dw_mci-artosyn`** - the SoC clock glue on the mainline `dw_mci` core; binds both mmc nodes. Shared with the SD card, documented in `sd-card.md`.
3. **`artosyn_sdio`** - matches the enumerated SDIO function, uploads the baseband firmware, then exposes `/dev/artosyn_sdio` and the `sdio0` netdev.

The AR8030 enumerates in two identities: **`4152:8030`** = ROM loader (needs firmware), **`4152:8031`** = firmware running. Probe forks on `func->device`: `0x8030` uploads firmware then returns; the chip resets, re-enumerates as `0x8031`, and the second probe builds the netdev.

## Host node: `mmc@1b00000` (mmc0) [confirmed]

The mmc nodes live in `../dts/proxima-9311.dts` (`mmc@1b00000` + `mmc@1c00000`, graduated from the retired `ar_dtbo_sdio` runtime overlay). Node properties:

- `compatible = "dwmmc0", "artosyn,proxima-dw-mshc"`, `reg = <0 0x1b00000 0 0x1000>`, `interrupts = <0 48 4>` (GIC_SPI 48 = hwirq 80, vendor `0x30`).
- `clock-frequency = <100000000>`, `clock-freq-min-max = <400000 100000000>`, `bus-width = <4>`, `num-slots = <1>`, `card-detect-delay = <200>`.
- Flags: `full-pwr-cycle`, `cap-sd-highspeed`, `cap-mmc-hw-reset`, `disable-wp`, `no-mmc`, `no-sd`, `cap-sdio-irq`, `broken-cd`. The `no-mmc`/`no-sd` pair forces the core to treat this instance as SDIO-only; `broken-cd` because there is no card-detect line for a soldered chip.

The microSD host `mmc@1c00000` (mmc1) is the same DesignWare IP, a different instance with its own clock registers; see `sd-card.md`. Note the enumeration-order cosmetic mismatch: the DT/board docs call the AR8030 host `mmc0`, but at runtime Linux may print it as `mmc1: new SDIO card` depending on probe order. Both are correct (DT instance name vs runtime host index).

## Reset line: `artosyn_gpio` GPIO23 [confirmed]

`artosyn_gpio.c` is the open reimplementation of the vendor `artosyn,gpio` controller at `0x0A10A000`; it is needed because the AR8030 reset must be driven through gpiolib (ad-hoc `/dev/mem` pokes fail: the direction register is write-only/unreadable). GPIO23 = bank1 line0 is the AR8030 active-low reset (drive output low then high to release). The register model, the load-bearing `0xBC` (not `0xC0`) bank base, and the bank layout are `artosyn-gpio.md`.

## Clock glue: `dw_mci-artosyn` (mmc0 taps) [confirmed]

The clock glue is shared with the SD card and documented in full in `sd-card.md` (the SEL tap table, the never-write rules for `0x0A108000`/`0x0A108038`, the CGU source-gate requirement); only the mmc0-specific facts are here. For mmc0 the tap registers are `AR_SDMMC0_CLK_SEL = 0x0A108088` and `AR_SDMMC0_CLK_CFG = 0x0A10808C` (the SD card uses `0x0A1080C0`/`0x0A1080C4`); the default is SEL `0x80` (100 MHz), phase 0 in CFG. The relevant CGU source gates are `0x0A104024` (bit 22) and `0x0A104028` (bit 23); gated, CMD5 times out (`-110`). Whether the glue actively writes the enable bits or relies on U-Boot is **[open]**; a proper CCF driver to own these gates is deferred (the read-only tree exists, `clocks.md`).

## `artosyn_sdio`: firmware upload + `sdio0` netdev

### Firmware [confirmed]

The baseband blob is `bb_demo_gnd_d.img` plus the config `bb_config_gnd.json`, both delivered via `request_firmware` (module params `fw_name`/`cfg_name`). This is the RF baseband firmware.

Upload runs the AR8030 ROM loader: `"SD"`-framed packets (`SD_FRAME_MAGIC = 0x4453`), a 64-byte SPL header written to `0x2f0040`, payload in <= `0xFF4` (4084) byte chunks with a 12-byte packet header, terminated by an addr=0/len=0 finalize; the config JSON is streamed verbatim. On success the current draw jumps ~+130 mA, the device ID flips to `0x8031`, and `sdio0` appears (`../modules/HW-BRINGUP.md` validation signals).

### Host-controller mailbox (CMD52) [confirmed]

The driver talks to the AR8030 SDIO function through CMD52 registers: STATUS `0x13` (busy = bit4), CTRL `0x14`, mailbox TX `0x44`/`0x48`, mailbox RX `0x54`/`0x58`, RX_COUNT `0x5C` (blocks << 9), TX_CREDIT `0x60` (blocks << 9), EVT_STATUS `0x68` (bits 0..3 = mbox0/mbox1/rx/tx). The IRQ handler loops on `0x13`, reads `0x68`, and dispatches. Bring-up pokes: `0x14 <- 0x01`, `0x68 <- 0xF0`.

A misc char device `/dev/artosyn_sdio` exposes seven `'v'`-type ioctls (CMD `0xC0207600`, REG_RD/WR, MSG_GET/POST, CCCR_RD/WR) for the userspace RF control plane.

### `sdio0` netdev and link framing [confirmed]

The data path is an `alloc_netdev_mqs(..., "sdio%d", ...)` interface with flags `IFF_NOARP | IFF_BROADCAST`, **`mtu = 4096`**, `needs_free_netdev = true`. Each SDIO transfer's last 4 bytes of the final 512-byte block are a link trailer `{0x00, type, len_lo, len_hi}`: `0xCC` = IPv4 tunnel (delivered to the stack via `netif_rx`), `0xDD` = command/response, `0xEE` = flow-control ACK (uplink-only, not a video blocker). The IPv4 tunnel ships a 12-byte compressed header (a 20-byte IPv4 header minus 8 leading bytes, carrying only the last octet of each address, since the link is point-to-point `10.0.0.x`); the driver rebuilds the full header from the `sdio0` IP via an inetaddr notifier.

**RX resync** [confirmed]: a compressed header can split across two SDIO reads, and a `0x45` byte inside HEVC payload looks like an IPv4 version/IHL nibble, so a naive walker fabricates ghost frames with random destination octets. The driver therefore carries split headers across reads (`hdr_carry`), plausibility-checks each header (`artosyn_ip_hdr_plausible`: proto whitelist UDP/TCP/ICMP/IGMP, `totlen <= 4352 = 4096 MTU + 256 headroom`), back-parses multi-run transfers from their 4-byte trailers, and counts junk skips in `rx_frame_errors`. It also forces the CMD53 block size to **512 on both `0x8030` and `0x8031`**: the `0x8031` re-enumeration is a fresh `sdio_func`, and without the forced 512 the core default desyncs the FIFO so `sdio0` rx sticks at 0.

## Status [confirmed on hardware]

The full path works on hardware: SDIO enumeration, firmware upload, RF-link association, and H.265 video downlink through the kernel UDP stack (~1.6 MB/s clean on `sdio0:10001`, `InAddrErrors` 0). The three requirements that together make the chip enumerate: the `artosyn_gpio` `0xBC` register base (`artosyn-gpio.md`), the un-gated CGU source clocks (above), and leaving `0x0A108000` at 0 (`sd-card.md`). See `../STATUS.md` and `../PERIPHERALS.md` for the tree-wide status.

Two operational facts worth carrying: **do not warm-reload `artosyn_sdio`** (`rmmod`/`insmod` hangs the whole device; test via a fresh RAM-boot), and **`sdio0 rx_bytes` is the only trustworthy "is it streaming" signal** - the `bb_ioctl` GET side-channel reads identically on a dead link, and an apparent SDIO "instability" once traced to a low air-unit battery brownout, not the driver.

## Source

Module sources `../modules/artosyn_sdio.c`, `../modules/dw_mci-artosyn.c`, `../modules/artosyn_gpio.c`; the DT nodes `../dts/proxima-9311.dts`; the validation method `../modules/VERIFICATION.md`; the peripheral map `../PERIPHERALS.md` and board config `../modules/BOARD-CONFIG.md`. RF protocol above the driver: `../../docs/reference/rf-video-downlink.md`.
