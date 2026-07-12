# Buzzer (PWM tone)

How the goggle's buzzer is driven on the open kernel. It is a PWM tone on channel 0 of the second Artosyn PWM controller, driven by the same `artosyn_pwm` provider as the LCD backlight; the shared PWM controller register map is documented in `display-backlight.md` and referenced (not repeated) here. Each fact is tagged **[confirmed]** (direct evidence in the disassembly/DT/on hardware), **[inferred]**, **[open]**.

## Architecture

```
buzzer = pwm@1002000 channel 0 (artosyn,ar9301-pwm, 8 ch, host_ref 150 MHz)
  driven by artosyn_pwm.ko -> exposes a pwmchip; the tone policy lives in userspace (the menu) via /sys/class/pwm
```

The buzzer and the LCD backlight are the two consumers of the `artosyn_pwm` driver: the backlight is channel 1 of the first controller (`pwm@1000000`), the buzzer is channel 0 of the second (`pwm@1002000`). The driver only exposes the `pwmchip`; the beep policy (tone period + volume-as-duty) is set from userspace, the same provider-registers-interface / userspace-drives-it shape as the status LED.

## The buzzer channel [confirmed]

DT node `pwm@1002000` (vendor `pwm@08040000`, reg `0x1002000`), `compatible = "artosyn,ar9301-pwm"`, 8 channels, functional clock `host_ref` 150 MHz (`../dts/proxima-9311.dts`). The buzzer is channel 0. The per-channel register model (OFF/CTRL/ON counts, the enable mask, the `count = round(time_ns * clk / 1e9)` math) is the shared `artosyn_pwm` map in `display-backlight.md`.

### Tone and volume [confirmed]

- **Tone period**: `260000` ns (~3.8 kHz), fixed.
- **Loudness = duty**: `13000 * volume` ns for volume 1..10 (0 = silent). Volume 10 = `130000` ns = 50% duty, the firmware maximum (the hardware itself would go to 100% = `260000` ns).

These are RE-confirmed from the stock menu (`customerHmBuzzerEnable`; the duty math in the on/off worker; volume from the config byte `u8BuzzerVolume`), and reproduced by `../test_tools/buzzer_test.c` (`PERIOD_NS 260000`, `DUTY_STEP_NS 13000`).

## Driver shape [confirmed]

`../modules/artosyn_pwm.c` is a plain mainline pwmchip provider: it binds both `pwm@1000000` (backlight) and `pwm@1002000` (buzzer), allocates an 8-channel chip with `devm_pwmchip_alloc`, and implements `apply`/`get_state` over the clock framework. It exposes only the `pwmchip`; userspace drives the buzzer through raw `/sys/class/pwm`.

No `pwm-beeper` input node exists, and it was deliberately rejected: mainline `pwm-beeper` fixes the duty at 50% and has no volume concept, which would regress the stock volume-1..10 feature (duty-based loudness is this board's only volume mechanism). So the buzzer stays userspace-driven rather than an `EV_SND` DT beeper.

## pwmchip discovery caveat [open]

The stock menu hardcodes `pwmchip8` - the vendor 4.9 kernel's global PWM base number. The open 6.18 kernel's modern pwmchip-id allocator enumerates controller 0 (backlight) as `pwmchip0` and controller 1 (the buzzer) as `pwmchip1` (and even that is probe-order dependent; the PWM class has no DT-alias mechanism).

The driver side needs no change (it is already a textbook mainline pwmchip driver). The fix is userspace: resolve the chip directory at runtime by scanning `/sys/class/pwm/pwmchip*/device` for the platform-device name `1002000.pwm` (the scan `buzzer_test.c` already implements), rather than hardcoding a number. The same sysfs-discovery approach applies to the temperature sensor's sysfs path and the backlight (same `pwmchip0` fragility). This only manifests once the menu ports off the vendor kernel; on the vendor 4.9 kernel `pwmchip8` is correct and the buzzer works.

## Status [confirmed on hardware]

Working on hardware: `pwmchip1/pwm0` enumerates, `apply`/`get_state` round-trip exactly, and `buzzer_test` sweeps volume 1..10 with audibly rising loudness. Validation tool `../test_tools/buzzer_test.c` (exports the channel, sets period, sweeps duty, auto-detecting the chip by the `1002000.pwm` platform-device name). See `../STATUS.md`, `../PERIPHERALS.md`, `../modules/VERIFICATION.md`.

## Source

`../modules/artosyn_pwm.c` (the shared provider; register map in `display-backlight.md`), the `pwm@1002000` node in `../dts/proxima-9311.dts`, `../test_tools/buzzer_test.c`.
