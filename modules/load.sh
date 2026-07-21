#!/bin/sh
# Load the open Artosyn MPP/RF kernel modules in dependency order, with the same
# module params the vendor used (the vendor's own load order; ar_cipher dropped).
#
# Run on-device (the open 6.18 kernel) from wherever the .ko live (default /mod).
# ar_osal MUST load first (it exports the hil_* MMZ API the others link).
#
# NOT YET HARDWARE-VALIDATED - see README.md "Residual hardware-validation risks".
set -e
MOD="${MOD:-/mod}"

# Foundation: MMZ allocator. The anonymous zone comes from the DTB's
# reserved-memory mmz node (single declaration of the carveout); mmz= only
# adds the on-chip sram zone, which has no reserved-memory node.
# Then the VB pool, sys/PTS, and SW arbiter.
insmod "$MOD/ar_osal.ko" \
	mmz=sram,0,0x00100000,0x00100000 \
	mmz_allocator=hisi anony=1
insmod "$MOD/ar_vb.ko"
insmod "$MOD/ar_sys.ko"

# MPP engine plumbing (platform drivers; auto-probe on the DT nodes).
insmod "$MOD/ar_mpp_drv.ko"
insmod "$MOD/ar_mpp_proc_ctrl.ko"
insmod "$MOD/ar_scaler.ko"

insmod "$MOD/ar_sysctl.ko"

# RF air-link last (needs the baseband firmware blob present where the kernel
# firmware loader looks, e.g. /lib/firmware or /usrdata/ar813x).
insmod "$MOD/artosyn_sdio.ko" \
	fw_name=bb_demo_gnd_d.img cfg_name=bb_config_gnd.json.usr_cfg.json

# Front-panel buttons: IIO core -> our SAR ADC provider -> adc-keys (DT) -> evdev.
# Load the ADC provider before adc-keys so the io-channel is present (else adc-keys
# just EPROBE_DEFERs until it appears). industrialio/adc-keys/evdev are in-tree
# modules (CONFIG_IIO=m, CONFIG_KEYBOARD_ADC=m, CONFIG_INPUT_EVDEV=m); modprobe pulls
# them from /lib/modules. The driver-model match on the DT "adc-keys" node binds once
# both the provider and adc-keys are loaded; evdev then exposes /dev/input/event0.
modprobe industrialio 2>/dev/null || insmod "$MOD/industrialio.ko" 2>/dev/null || true
insmod "$MOD/artosyn_adc.ko" 2>/dev/null || true
modprobe adc-keys 2>/dev/null || insmod "$MOD/adc-keys.ko" 2>/dev/null || true
modprobe evdev 2>/dev/null || insmod "$MOD/evdev.ko" 2>/dev/null || true

# SoC temperature sensor (shares the ADC MMIO window). IIO_TEMP provider for the
# menu's temperature readout. Independent of adc-keys; also needs industrialio.
insmod "$MOD/artosyn_protemp.ko" 2>/dev/null || true

# LCD backlight: our artosyn PWM provider; the mainline pwm-backlight (DT "backlight"
# node, =y) binds once pwm0 registers, exposing /sys/class/backlight/.
insmod "$MOD/artosyn_pwm.ko" 2>/dev/null || true

echo "loaded. nodes:"
ls -l /dev/mmz_userdev /dev/ar_vb /dev/ar_sys /dev/ar_sysctl \
	/dev/ar_mpp_ctl /dev/ar_mpp_proc_ctl /dev/arscaler \
	/dev/artosyn_sdio /dev/input/event0 2>/dev/null || true
