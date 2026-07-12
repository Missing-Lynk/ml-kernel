#!/bin/sh
# temp_read.sh - continuously read and print the SoC temperature from the
# artosyn_protemp IIO device (modules/artosyn_protemp.c). Runs on-device;
# it only reads sysfs, so there is nothing to compile (unlike the C tools here).
#
#   temp_read.sh          # ~1 s interval, until Ctrl-C
#   temp_read.sh 0.2      # custom interval in seconds
#
# The protemp driver uses NON-STANDARD IIO semantics recovered from the vendor:
# in_temp_scale carries the whole converted temperature in degrees C (NOT a scale
# factor), while in_temp_raw is the averaged 9-bit code. We print both, and also
# re-derive Celsius from the raw code via the documented conversion
# ((raw*5320 - 1373400)/10000) as an independent cross-check. See
# ../docs/artosyn-protemp.md.

interval="${1:-1}"

# Locate the IIO device whose "name" is "temperature" (the protemp driver sets
# indio_dev->name = "temperature"; the sysfs dir itself is iio:deviceN).
dev=""
for d in /sys/bus/iio/devices/iio:device*; do
	[ -r "$d/name" ] || continue
	if [ "$(cat "$d/name")" = "temperature" ]; then
		dev="$d"
		break
	fi
done

if [ -z "$dev" ]; then
	echo "no IIO device named 'temperature' found" >&2
	echo "is artosyn_protemp loaded? (insmod artosyn_protemp.ko)" >&2
	echo "present IIO devices:" >&2
	for d in /sys/bus/iio/devices/iio:device*; do
		[ -r "$d/name" ] && echo "  $d -> $(cat "$d/name")" >&2
	done
	exit 1
fi

echo "reading $dev (name=temperature) every ${interval}s; Ctrl-C to stop"

while :; do
	raw="$(cat "$dev/in_temp_raw" 2>/dev/null)"
	scale="$(cat "$dev/in_temp_scale" 2>/dev/null)"

	# Cross-check Celsius derived from the raw code. Integer math to keep this
	# busybox-portable; prints whole degrees like the driver's own conversion.
	if [ -n "$raw" ]; then
		derived=$(( (raw * 5320 - 1373400) / 10000 ))
	else
		derived="?"
	fi

	echo "raw=${raw:-?}  in_temp_scale=${scale:-?} C  (derived=${derived} C)"
	sleep "$interval"
done
