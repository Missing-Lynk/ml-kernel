# test_tools

Standalone, statically-linked programs for bringing up and validating the open-kernel
drivers **on the device**. These exercise a driver through its real userspace ABI (chardev,
sysfs, ioctl), so they test the whole path: DT probe -> driver -> class device -> syscall.

Each `.c` is independent. Build all with `make` (binaries land in `build/`, which is
git-ignored; see the `Makefile` for the toolchain override), copy the one you need to the
goggle, and run it over SSH.

## Tools

### `buzzer_test` - artosyn_pwm buzzer
Drives the buzzer (channel 0 of the 2nd artosyn PWM controller, DT `pwm@1002000`) via
`/sys/class/pwm`. Auto-detects which `pwmchipN` the driver bound the controller to (by
matching the `1002000.pwm` platform device), then sweeps or plays a tone.

```sh
make buzzer_test
scp build/buzzer_test root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/buzzer_test        # sweep volume 1..10 (hear the steps)
ssh root@192.168.3.100 /tmp/buzzer_test 7      # single volume 0..10 for ~1s
ssh root@192.168.3.100 /tmp/buzzer_test /sys/class/pwm/pwmchip1 5   # explicit chip
```

What it confirms:
- `pwm@1002000` is in the DT and `artosyn_pwm` probed it (else "no pwmchip for 1002000.pwm").
- **Which `pwmchipN` the buzzer got** - printed on the first line; this is the number the
  board profile (`buzzer_pwm_chip`) needs.
- `ar_pwm_apply` accepts the period/duty and `enable` actually sounds the buzzer.
- Volume steps are audibly different (duty 13000·vol ns, period 260000 ns).

### `display_test` - artosyn_vo DRM display (primary plane)
Renders through the real DRM/KMS path (`/dev/dri/card0`: dumb buffer + `SETCRTC`), validating
the whole `artosyn_vo` primary-plane pipeline end to end: DC AXI fetch-master, layer
format/stride, scanout, DPI, MIPI-DSI, panel.

```sh
make display_test
scp build/display_test root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/display_test bars         # 8 colour bars (the "rainbow")
ssh root@192.168.3.100 /tmp/display_test 0x00ff0000   # solid colour 0x00RRGGBB
```
Confirms the primary plane fetches a coherent CPU-written framebuffer from DDR and scans it
to the panel (the reason the CRG `0x0a106018` fetch-master enable matters).

### `display_bounce` - animated DRM display
A DVD-style colour-cycling box bouncing on the primary plane, redrawing each frame through the
coherent dumb-buffer mapping (per-frame cache clean). Validates live per-frame updates; loops
until killed (`pkill display_bounce`).

### `display_test` / `overlay_test` / `display_demo` - two-plane compositor
- `overlay_test` puts an ARGB4444 box on the **overlay plane** over primary colour bars, via the
  real DRM plane ABI (`GETPLANERESOURCES` + `SETPLANE`). Confirms the overlay plane composites
  with **per-pixel alpha** (the bars show through where the overlay is transparent).
- `display_demo` is the combined sample: a bouncing colour-cycling square on the primary plane
  **plus** a live "FRAME NNNNNN" counter (white text on a semi-transparent bar) on the overlay
  plane, both animating and blended in hardware. Loops until killed (`pkill display_demo`).

```sh
make overlay_test display_demo
scp build/overlay_test build/display_demo root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/overlay_test 0x8fff 490 100   # semi-transparent box, y=490 h=100
ssh root@192.168.3.100 /tmp/display_demo                   # bouncing square + frame counter
```

> All of these use `/dev/fb0` = the DRM fbdev, and fbcon auto-binds to it and draws a console cursor
> (a small stray box). Unbind it first for a clean surface:
> `for v in /sys/class/vtconsole/vtcon*; do grep -q "frame buffer" "$v/name" && echo 0 > "$v/bind"; done`

### `led_test` - status RGB LED (WS2812 over SPI)
Drives the 3-pixel WS2812 status LED on the DesignWare SPI master via `spidev` (see
`../docs/status-led.md`). Each WS2812 bit is one SPI byte at 6.25 MHz (`0xC0` = "0",
`0xFC` = "1"); a frame is 3 pixels × 24 bits GRB = 72 bytes. The spidev node is auto-detected
(`/dev/spidev*.0`; override with `SPIDEV=`).

```sh
make led_test
scp build/led_test root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/led_test              # rainbow cycle (40 ms/step) until Ctrl-C
ssh root@192.168.3.100 /tmp/led_test rainbow 20   # rainbow with a custom step delay (ms)
ssh root@192.168.3.100 /tmp/led_test ff8000       # solid colour 0xRRGGBB
ssh root@192.168.3.100 /tmp/led_test off          # all pixels off
```
Confirms the `spi@1102000` controller + `spidev` child are in the DT and bound, and that
userspace can drive the LED (the frame encoding is byte-identical to the vendor's). The
LED has this SPI bus/CS to itself - the display panel uses DSI + GPIO + PWM, not SPI.

### `button_test` - front-panel buttons (adc-keys evdev)
Reads the button input device and prints each press/release. The buttons are a resistor ladder
on ADC channel 0, decoded by the in-kernel `adc-keys` driver (fed by the `artosyn_adc` IIO
provider) into evdev events on `/dev/input/event0`. See `../docs/artosyn-adc.md`.

```sh
make button_test
scp build/button_test root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/button_test              # auto-detects the adc-keys device
ssh root@192.168.3.100 /tmp/button_test /dev/input/event0   # explicit node
```
Prints e.g. `up       code=87 (0x57)  press`. Keymap: bind/back/record/up/down/left/right/enter.
Requires `artosyn_adc` loaded, otherwise `adc-keys` stays in deferred-probe and no input node
exists (`insmod artosyn_adc.ko`; IIO core is built-in, so no other module is needed).

### `temp_read.sh` - SoC temperature (artosyn_protemp IIO)
A shell script (not a compiled tool - it only polls sysfs), continuously printing the SoC
temperature from the `artosyn_protemp` IIO device (`temperature@a108000`, shares the ADC MMIO
window). It finds the IIO device by its `name` (`temperature`), since the sysfs dir is
`iio:deviceN`. See `../docs/artosyn-protemp.md`.

```sh
scp temp_read.sh root@192.168.3.100:/tmp/
ssh root@192.168.3.100 sh /tmp/temp_read.sh        # ~1 s interval, until Ctrl-C
ssh root@192.168.3.100 sh /tmp/temp_read.sh 0.2    # custom interval (seconds)
```
Prints e.g. `raw=342  in_temp_scale=44 C  (derived=44 C)`. Note the driver's non-standard IIO
semantics: `in_temp_scale` carries the whole temperature in degrees C (not a scale factor) and
`in_temp_raw` is the averaged 9-bit code; the script re-derives Celsius from the raw code as a
cross-check. Requires `artosyn_protemp` loaded (`insmod artosyn_protemp.ko`; IIO core is
built-in). Confirms `temperature@a108000` is in the DT, the driver probed it, and the
code->Celsius conversion is live.

### `mmztest` - ar_osal (MMZ) + ar_sys
The Tier-0 smoke test for the MMZ allocator and the PTS path: allocates a 1 MiB block from
`/dev/mmz_userdev`, mmaps it, verifies a written pattern reads back correctly, frees it, then
exercises `/dev/ar_sys`'s PTS ioctls and checks the clock is monotonic. Uses the byte-exact
ABI in `../modules/ar_uapi.h`.

```sh
make mmztest
scp build/mmztest root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/mmztest
```
Requires `ar_osal.ko` and `ar_sys.ko` loaded first (see `../modules/load.sh`). Does not
exercise the codec/scaler engines - only the foundation ioctl/mmap contracts.

### `scalertest` - ar_scaler
Allocates a src + dst buffer from MMZ and issues one `CropResize` (1280x720 -> 640x360) on
`/dev/arscaler`. Not a pixel-correctness check (no golden image) - the point is whether the
op *completes*: `rc==0` means the clock bring-up + completion IRQ path work; `-ETIMEDOUT`
means the register packing or clock sequence is wrong and the IRQ never fired.

```sh
make scalertest
scp build/scalertest root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/scalertest
```
Requires `ar_osal.ko` and `ar_scaler.ko` loaded first. After the call, `cat
/proc/arscaler/state` on the device to inspect the programmed registers.

### `sd_rwtest` - SD card block reads/writes (dw_mci-artosyn), hang-safe
Validates SD-card block I/O through the real block layer (`/dev/mmcblk1` -> mmcblk -> mmc core ->
`dw_mci-artosyn`), designed around the historical `mmc_blk_rw_wait` write hang: every I/O phase runs in a forked child under a timeout, so a D-state hang is
*reported* (with the child's `/proc/<pid>/wchan`) instead of wedging the shell. The read test is
non-destructive (4 MiB at offset 0, read twice O_DIRECT, compared). The write test rewrites a
scratch region near the end of the device **in place**: it backs the region up to a file first,
writes a per-sector tagged pattern, verifies it via O_DIRECT read-back, then restores and
re-verifies the original bytes. If a phase hangs, the backup file survives for a post-reboot
`restore`.

```sh
make sd_rwtest
scp build/sd_rwtest root@192.168.3.100:/tmp/   # or push the binary over SSH however you prefer
ssh root@192.168.3.100 /tmp/sd_rwtest read              # non-destructive read check only
ssh root@192.168.3.100 /tmp/sd_rwtest all               # read + 64 KiB O_DIRECT write test
ssh root@192.168.3.100 /tmp/sd_rwtest -n $((4<<20)) -B write   # 4 MiB buffered+fsync (dd path)
ssh root@192.168.3.100 /tmp/sd_rwtest restore /tmp/sd_rwtest.bak   # after a hang + power cycle
```
Exit codes: 0 = PASS, 1 = mismatch/I/O error, 2 = a phase HUNG (power-cycle before trusting any
further SD I/O - the mmc queue is wedged and the hung child is unkillable), 3 = usage/setup.
Requires the SD stack up first (HW-BRINGUP.md: `ar_dtbo_sdio.ko` + `artosyn_gpio.ko` +
`dw_mci-artosyn.ko clk_sel=135 clk_cfg=2` - the hardware-validated clock pair 0x87/0x02,
see `../docs/sd-card.md`; the card enumerates as `mmcblk1`).

### `gpio_pulse` / `gpio_hold` / `gpio_reset` / `gpio_verify` - artosyn_gpio
Four small probes over the standard GPIO chardev v2 ABI (`/dev/gpiochipN`, found by label -
no `/dev/mem`), useful for bringing up any line on the `artosyn_gpio` driver, not just the
AR8030 reset line they were written for:
- `gpio_pulse <chip-label> <line>` - drive a line low then high (a reset-release pulse).
  `gpio_pulse ar-gpio1 0` releases the AR8030 reset (GPIO23 = bank1 line 0).
- `gpio_hold <chip-label> <line> <value> <seconds>` - drive a line to a fixed value and hold
  it for manual probing/measurement.
- `gpio_reset <chip-label> <line> [dat-phys] [bit]` - replicates the vendor's exact reset
  sequence (assert, deassert, then release the line to input/high-Z, matching
  `start_ar813x.sh`'s pulse+unexport), optionally reading back the pad-level register.
- `gpio_verify <chip-label> <line> <dat-phys> <bit>` - drives a line 0 then 1 and reads the
  controller's pad-level (DAT) register after each, confirming the physical pad actually
  follows the driven value (proves the pinmux/line mapping, e.g. that GPIO23 really is the
  AR8030 reset pad).

```sh
make gpio_pulse gpio_hold gpio_reset gpio_verify
scp build/gpio_pulse build/gpio_hold build/gpio_reset build/gpio_verify root@192.168.3.100:/tmp/
ssh root@192.168.3.100 /tmp/gpio_pulse ar-gpio1 0
ssh root@192.168.3.100 /tmp/gpio_verify ar-gpio1 0 0x0A10A0D4 0
```
Requires `artosyn_gpio.ko` loaded (registers the labelled gpiochips the tools search for).
