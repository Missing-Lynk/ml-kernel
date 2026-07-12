#!/bin/sh
# dmatest-axi-dma.sh - on-device bring-up validator for the mainline dw-axi-dmac driver
# bound to the Artosyn axi_dma engine (artosyn,axi_dma @ 0x08800000, 3 channels).
#
# Proves the driver probed AND that each channel does a real, correct hardware phys-to-phys
# memcpy - the same DMA_MEMCPY path the ml-dmablit compositor shim drives. Uses the kernel's
# own dmatest client, so a PASS here means device_prep_dma_memcpy works end-to-end on this
# silicon.
#
# Run on the goggle after RAM-booting the new Image+dtb (A still active - RAM-boot only):
#   sh /path/dmatest-axi-dma.sh
# Needs dmatest.ko staged (modules/build.sh whitelists it) + modules.dep (depmod).
#
# Exit 0 = all channels PASS, 1 = probe/setup failure, 2 = a channel reported copy failures.
set -u

SYS=/sys/module/dmatest/parameters
BUF=${BUF:-131072}        # 128 KiB: > one LLI block (4096<<2 = 16 KiB) so it exercises the
                          # descriptor-chain path, but small enough to kmalloc safely.
ITER=${ITER:-100}
TIMEOUT_MS=${TIMEOUT_MS:-3000}

say() { echo "[dmatest-axi] $*"; }
fail() { say "FAIL: $*"; exit 1; }

# 1. Did dw-axi-dmac bind the block?
say "checking dw-axi-dmac probe ..."
if dmesg | grep -qi 'dw-axi-dmac.*8800000.*probed\|8800000.*dma-controller.*DesignWare'; then
	dmesg | grep -i '8800000.*dma-controller\|dw-axi-dmac' | tail -3
else
	say "no clean probe line in dmesg; checking for a bind failure ..."
	dmesg | grep -i '8800000\|dw-axi-dmac\|axi-dma' | tail -8
	fail "dw-axi-dmac did not probe 8800000.dma-controller (see dmesg above)"
fi

# 2. Are the channels registered with the dmaengine core?
if [ ! -d /sys/class/dma ]; then
	fail "/sys/class/dma missing (CONFIG_DMA_ENGINE not built in?)"
fi

CHANS=$(ls -1 /sys/class/dma/ 2>/dev/null | grep '^dma[0-9]*chan[0-9]*$' || true)
[ -n "$CHANS" ] || fail "no dmaNchanN channels registered under /sys/class/dma"
say "registered channels:"
for c in $CHANS; do
	printf '    %s -> %s\n' "$c" "$(cat /sys/class/dma/$c/device/../of_node/name 2>/dev/null || echo '?')"
done

# 3. Load the dmatest client (idempotent).
if [ ! -d "$SYS" ]; then
	say "loading dmatest.ko ..."
	modprobe dmatest 2>/dev/null || insmod "$(find /lib/modules -name dmatest.ko 2>/dev/null | head -1)" \
		|| fail "cannot load dmatest (is dmatest.ko staged? modules/build.sh)"
fi
[ -d "$SYS" ] || fail "dmatest parameters dir absent after load"

# 4. Configure, then bind every channel (channel writes MUST come after all other params -
#    kernel >=5.0 requirement), then run. Results land in dmesg per channel.
say "config: test_buf_size=$BUF iterations=$ITER timeout=${TIMEOUT_MS}ms"
echo "$TIMEOUT_MS" > "$SYS/timeout"
echo "$ITER"       > "$SYS/iterations"
echo "$BUF"        > "$SYS/test_buf_size" 2>/dev/null || say "note: test_buf_size not settable while loaded; using module default"
for c in $CHANS; do
	echo "$c" > "$SYS/channel" 2>/dev/null && say "  armed $c"
done

# Mark a fence in the log so we only parse THIS run's summaries.
FENCE="dmatest-axi-fence-$$"
echo "$FENCE" > /dev/kmsg 2>/dev/null || true

say "running ..."
echo 1 > "$SYS/run"

# 5. Wait for completion (run flips back to 0), bounded.
i=0
while [ "$(cat "$SYS/run" 2>/dev/null)" = "Y" ] || [ "$(cat "$SYS/run" 2>/dev/null)" = "1" ]; do
	i=$((i+1))
	[ "$i" -gt 60 ] && { say "timeout waiting for dmatest to finish"; break; }
	sleep 1
done

# 6. Parse the per-channel summaries emitted after the fence.
say "results:"
SUMS=$(dmesg | sed -n "/$FENCE/,\$p" | grep -i 'dmatest' | grep -iE 'summary|failures|error')
if [ -z "$SUMS" ]; then
	# Fence may have scrolled off a small dmesg ring; fall back to the tail.
	SUMS=$(dmesg | grep -i 'dmatest' | grep -iE 'summary|failures|error' | tail -6)
fi
echo "$SUMS" | sed 's/^/    /'

# A passing dmatest line reads "... summary N tests, 0 failures ...". Any nonzero-failure
# line, or an error line, is a fail.
if echo "$SUMS" | grep -qiE '[1-9][0-9]* failures|error'; then
	say "FAIL: at least one channel reported copy failures/errors"
	exit 2
fi

if echo "$SUMS" | grep -qi '0 failures'; then
	say "PASS: dw-axi-dmac memcpy verified on hardware (0 failures)"
	exit 0
fi

say "INCONCLUSIVE: no summary lines found - check 'dmesg | grep dmatest' manually"
exit 1
