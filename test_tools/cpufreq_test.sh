#!/bin/sh
# cpufreq_test.sh - walk every OPP with the userspace governor and verify each
# one with the PMU (pmu_test's cycles/wall-time measurement is the ground truth
# for the real core clock).
#
# Run on the goggle with /tmp/pmu_test staged (test_tools/pmu_test.c).
# Restores the previous governor on exit.
set -e

CF=/sys/devices/system/cpu/cpu0/cpufreq
PMU=${PMU:-/tmp/pmu_test}

[ -d "$CF" ] || { echo "FAIL: no cpufreq policy (driver did not bind)"; exit 1; }
[ -x "$PMU" ] || { echo "FAIL: $PMU missing"; exit 1; }

old_gov="$(cat "$CF/scaling_governor")"
restore() { echo "$old_gov" > "$CF/scaling_governor" 2>/dev/null; }
trap restore EXIT

echo "driver=$(cat "$CF/scaling_driver") governor=$old_gov"
echo "available: $(cat "$CF/scaling_available_frequencies")"
echo userspace > "$CF/scaling_governor"

fail=0
for khz in $(cat "$CF/scaling_available_frequencies"); do
	echo "$khz" > "$CF/scaling_setspeed"
	sleep 1
	cur="$(cat "$CF/scaling_cur_freq")"

	# pmu_test prints "INFO: cpuN measured core clock: X.X MHz"; average the
	# per-core numbers and compare against the requested kHz within 2 percent.
	mhz="$($PMU 2>/dev/null | sed -n 's/.*measured core clock: \([0-9.]*\) MHz.*/\1/p')"
	set -- $mhz
	[ $# -eq 2 ] || { echo "FAIL @$khz: pmu_test gave no per-core clocks"; fail=1; continue; }
	ok=1
	for m in "$@"; do
		# integer permille comparison, busybox-safe (truncate decimals)
		got=${m%%.*}
		want=$((khz / 1000))
		dev=$(( (got - want) * 1000 / want ))
		[ $dev -lt 0 ] && dev=$((-dev))
		[ $dev -le 20 ] || ok=0
	done

	if [ $ok -eq 1 ]; then
		echo "PASS @${khz}kHz: cur=$cur PMU cpu0/cpu1 = $1 / $2 MHz"
	else
		echo "FAIL @${khz}kHz: cur=$cur PMU cpu0/cpu1 = $1 / $2 MHz"
		fail=1
	fi
done

[ $fail -eq 0 ] && echo "PASS: all OPPs PMU-verified" || echo "FAIL ($fail)"
exit $fail
