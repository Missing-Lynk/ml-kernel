# Status LED (addressable RGB over SPI)

How the addressable status LED is driven, recovered so the open kernel can reproduce it. It is a WS2812/SK6812-family addressable RGB LED bit-banged over SPI - the SPI master itself is a standard mainline-supported DesignWare part, so the transport is a general SoC fact; how many LEDs are populated and what colours/patterns they show is a board-specific fact, covered separately below. Each fact below is tagged **[confirmed]** (direct evidence in the disassembly/DT), **[inferred]** (consistent but not proven on hardware), **[open]** (still to be determined).

## Summary

The SPI master is a DesignWare APB SSI (`snps,dw-apb-ssi`) at MMIO `0x1102000` - mainline `dw_spi_mmio` binds it, no custom driver needed, only a DT node and a `spidev` child (see `devices/betafpv-vr04-goggle/proxima-9311.dts`). This is a fixed property of the SoC's SPI block, the same on every Proxima-9311 device.

The vendor drives the LED entirely from userspace: `ar_lowdelay` opens the spidev, configures mode / 8 bits per word / 6.25 MHz, then a worker thread sends ~72-byte SPI frames with `SPI_IOC_MESSAGE` and does the blink in software - there is no in-kernel LED driver on the vendor side either, so the open stack's equivalent (`test_tools/led_test.c`) is also a userspace program, not a kernel module.

The **number of LEDs, the colours used, and the idle/link-state pattern** are BetaFPV VR04-specific board facts, not SoC ones: this board populates three physical RGB pixels (confirmed on hardware; the vendor drives all three the same colour, so stock reads as one uniform LED) and only ever drives them orange (solid vs. blinking to indicate link state - see "Colour-to-state map" below), even though the vendor's precomputed colour table also supports four colours. A different Artosyn product needs **zero driver-code changes** to use a different LED count or colour scheme - the SPI transport and the WS2812 frame encoding are unrelated to how many pixels are populated or which colours a given product's firmware chooses to send.

Porting is a **device-tree change** (the SPI controller node + a `spidev` child, generic across the SoC) plus a userspace-side change to how many LED slots are filled in and what colours/patterns are chosen - there is nothing here for a kernel driver to differ on.

## SPI master [confirmed]

Vendor DT `spi@08140000`:

```
compatible = "snps,dw-apb-ssi";
reg = <0x1102000 0x2000>;
num-cs = <4>;
interrupts = <0 0x27 4>;     /* GIC 71 (live: "dw_spi 71") */
clocks = <spi_clk>;
spidev01 { compatible = "spidev"; reg = <0>; spi-max-frequency = <0x1312d00>; };  /* 20 MHz cap */
```

This binds the mainline DesignWare SPI driver (`dw_spi_mmio`, `CONFIG_SPI_DW` + `CONFIG_SPI_DW_MMIO`). A second identical controller exists at `0x1100000` (`spi@08100000`) with no spidev child - unused by anything observed so far. The bus number `32765` in the device name (`/dev/spidev32765.0`) is a dynamically assigned id (the controller has no fixed `bus_num`).

## Control path [confirmed]

IPC command `0x1b` (`LOWDELAY_CTL_SYS_LED_CTRL`, the `AR_LOWDELAY_MID_SYS_LED_Ctrl` client call in `liblowdelay_mid.so`) lands on the server side in `ar_lowdelay`:

| Symbol | Addr | Role |
|--------|------|------|
| `AR_LOWDEALY_RX_SYSCTRL_LED_CTRL` | `0x43a1f8` | 1-instruction tail-call to `customerHmLedCtrl` |
| `AR_LOWDEALY_RX_SYSCTRL_LED_ENABLE` | `0x43a1f0` | tail-call to `customerHmLedEnable` |
| `customerHmLedInit` | `0x46d2b0` | opens + configures the spidev, starts the worker thread |
| `customerHmLedCtrl` | `0x46d048` | maps a mode byte to a pattern, enqueues a frame |
| `customerHmLedEnable` | `0x46cf10` | enqueues an on/off request |
| `customerHmLedDeinit` | `0x46d4f0` | teardown |
| LED worker thread ("ledOnOffThread") | `0x46c930` | pops the queue, does the SPI transfer, blinks |

`RX` = receiver side (this LED needs no RF link to drive - it's local to the device it's mounted on). The parallel `customerHmBuzzer*` and `customerHmLcd*` families sit next to these.

## SPI setup (`customerHmLedInit`) [confirmed]

1. `open("/dev/spidev32765.0", O_RDWR)`.
2. `ioctl(fd, SPI_IOC_WR_MODE, ...)` (`0x40016b01`).
3. `ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, ...)` (`0x40016b03`).
4. `ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &6250000)` (`0x40046b04`) -> 6.25 MHz.
5. Create a command queue + mutex and spawn the worker thread.

The ioctl magic `'k'` (`0x6b`) is the standard `spidev` `SPI_IOC_MAGIC`. Error strings in the binary confirm the path: "Error[%d] open %s", "set spi mode", "set spi bits", "set spi speed", "Spi Init Success, Dev %s, Fd %d".

## Frame format [confirmed]

The worker / renderer performs `ioctl(fd, SPI_IOC_MESSAGE(1), &xfer)` (`0x40206b00`, transfer struct 0x20 bytes) with `bits_per_word = 8`, `len = 0x48` = 72 bytes. The 72 bytes are a WS2812-style bitstream: 3 LED slots x 24 bits, one SPI byte per LED bit, MSB first, where the SPI byte encodes the WS2812 line timing:

- `0xc0` (`0b11000000`) = a WS2812 **"0"** bit (short high pulse).
- `0xfc` (`0b11111100`) = a WS2812 **"1"** bit (long high pulse).

At the 6.25 MHz clock each SPI byte is 1.28 us, matching the ~1.25 us WS2812 bit period. Colour order is **GRB** (8 bits green, 8 bits red, 8 bits blue per LED). The all-off frame is `memset(buf, 0xc0, 72)` (every bit "0" -> all three LEDs black). The frame carries three LED slots, and hardware confirms **this board populates all three** (the open `led_test`/`ml-ledd` light three distinct pixels).

> **Open path uses `0x80`, not the vendor's `0xc0`, for the "0" symbol.** The vendor's `0xc0` (~0.32 us high) is long enough to be misread as a "1" on the three-pixel chain, and the error compounds pixel-to-pixel (an all-red frame renders red/pink/orange down the chain). The open `led_test` and `ml-ledd` encode "0" as `0x80` (`0b10000000`, ~0.16 us high) and keep "1" as `0xfc`, which reads cleanly on all three pixels across red/green/blue/white. `ml-ledd` also appends trailing low bytes after the 72 data bytes to guarantee the reset/latch on continuous refresh.

The lit colours are **precomputed 72-byte frames in a static table** at `0x5103f0` (`.data`), not built at runtime. Each entry is 89 bytes: an id byte at +8, an ASCII name, then the 72-byte frame at +17 (`frame = 0x510409 + id*89`). `customerHmLedCtrl` / the renderer read a selector byte from the config struct (`+115`) and match it against these ids to choose the colour:

| id | name | encoded colour (decoded from the frame, GRB) |
|----|------|----------------------------------------------|
| 0  | red    | G=0x00 R=0xff B=0x00, all 3 LEDs |
| 1  | green  | G=0xff R=0x00 B=0x00, all 3 LEDs |
| 2  | blue   | G=0x00 R=0x00 B=0xff, all 3 LEDs |
| 3  | orange | G/R mix (per the `c0`/`fc` pattern), all 3 LEDs |

Worked example (entry 0 "red", frame bytes from `0x510409`): the 72 bytes decode bit-by-bit (`c0`->0, `fc`->1) to `00 ff 00` x3 = GRB(0,255,0) = red, confirming the GRB / `0xc0`-`0xfc` encoding.

The blink is software: the worker switches on a mode (0..5) and wraps these frames in timing. Modes 0 and 5 send the all-`0xc0` (off) frame once; the colour modes call a renderer (`0x46c618`) that sends a frame then `usleep`s (1,000,000 us steps; a 2,000,000 us = 2 s delay appears in the blink path) and repeats, producing the blink. So blink-vs-solid and which colour is shown are driven by which mode/colour the link-state logic feeds into `SYS_LED_Ctrl`, not by any hardware register.

## Colour-to-state map [confirmed on hardware]

This board's firmware only ever uses one entry from the four-colour table above. Observed on the stock unit: the LED colour never changes; it is always orange (GRB(40,255,0), the renderer's default/fallback colour, id 3) - the three pixels are all driven the same, so it reads as one uniform orange. State is conveyed by blink vs. solid, not by colour:

| State | LED |
|-------|-----|
| Searching / not synced (no link) | **blinking orange** |
| Paired / synced (linked) | **solid orange** |

This is a passive cross-check of the frame decode too: the observed idle colour matches the `orange` table entry's decoded GRB value with no active write needed to confirm it. The red/green/blue entries are unused by this board/firmware's status indicator; the frame format and table support them regardless.

## Implications for the open kernel

The LED needs an SPI path, not a GPIO or PWM path. This is done for this device (see `devices/betafpv-vr04-goggle/proxima-9311.dts` and `test_tools/led_test.c`):

1. Enable the DesignWare SPI master + spidev: `CONFIG_SPI=y`, `CONFIG_SPI_DW`, `CONFIG_SPI_DW_MMIO`, `CONFIG_SPI_SPIDEV`.
2. A `spi@1102000` controller node (`snps,dw-apb-ssi`, IRQ GIC 71, its clock) with a `spidev` child at CS0, so `/dev/spidevB.0` appears.
3. Drive the LED from userspace (matching the device-direct approach used for brightness/buzzer): open the spidev, set 8 bits / 6.25 MHz, and write the WS2812-encoded frames.

## Possible improvements

Where the open LED path could go beyond what's needed for this one board:

- **No standard LED-class interface exists.** The open path is exactly the vendor's shape: a userspace program that owns the spidev and bit-bangs WS2812 frames (`led_test.c` today, matching `ar_lowdelay`'s worker thread).

A `leds-class` driver (`CONFIG_LEDS_CLASS` + a small `leds-ws2812-spi`-style driver, RGB triplet as one "LED", exposing `/sys/class/leds/<name>/{brightness,multi_intensity}` or a `multicolor` class device) would let any userspace (not just a bespoke tool) drive it through a standard interface, and would let the kernel own the framing so userspace just picks a colour. Difficulty: LOW-MED; no vendor userspace depends on the open path's shape here, so this is compatible-now and not constrained by ABI parity.
- **Only one colour is ever used on this board**, but the wire format and the vendor's own table support three LEDs and four colours. A future open feature (distinguishing more link/error states by colour, not just blink/solid) is available for free at the framing level - it only needs the userspace/driver side to pick a different frame, no format changes.
- **The second identical SPI controller (`spi@08100000`, `0x1100000`) has no spidev child and no known consumer.** Worth a quick check on whether any other peripheral is wired to it before assuming it's unused.

## Source

Disassembly of the stripped `firmware/bin/ar_lowdelay` (aarch64) around the `customerHm*Led*` functions, the vendor device tree, and live captures (`1102000.spi` bound by `dw_spi_mmio`).
