#!/usr/bin/env bash
# Out-of-tree dev build for the open Artosyn kernel modules.
#
# Compiles every module in modules/ against the already-built 6.18.36 tree
# (the one scripts/build.sh produces) using the same pinned crosstool, on the host
# (no container needed - the container is only for the *reproducible* Image; module
# dev iteration just needs a configured tree + the cross gcc).
#
#   modules/build.sh            # build all *.ko
#   modules/build.sh -v         # ...streaming full make output (default: OK/last-40-lines)
#   modules/build.sh clean
#
# Override BUILD_DIR= to point at a different kernel tree. Pass -v/--verbose (or VERBOSE=1)
# to stream the full `make` output instead of the default "OK, or last 40 lines on failure".
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
source "$REPO/scripts/pin.env"

VERBOSE="${VERBOSE:-0}"
args=()
for a in "$@"; do
  case "$a" in
    -v|--verbose) VERBOSE=1 ;;
    *) args+=("$a") ;;
  esac
done
set -- "${args[@]+"${args[@]}"}"

# run <label> <cmd...>: quiet by default (log to a temp file, print "label OK" or the last 40
# lines on failure); VERBOSE=1/-v streams the command's output live instead.
run() {
  local label="$1"; shift
  if [ "$VERBOSE" = 1 ]; then
    "$@"
    return
  fi

  local log; log="$(mktemp)"
  if "$@" >"$log" 2>&1; then
    echo "  $label OK"
    rm -f "$log"
  else
    echo "  $label FAILED, last 40 lines:" >&2
    tail -n 40 "$log" >&2
    rm -f "$log"
    return 1
  fi
}

BUILD_DIR="${BUILD_DIR:-$KERNEL_BUILD_DEFAULT}"
KTREE="$BUILD_DIR/linux"
# Build OUT OF SOURCE. A Kbuild M= build drops .o/.ko/.mod*/.cmd/Module.symvers right
# beside the sources, which clutters the tracked source dir. We copy the sources to
# BUILD_OUT (outside the repo) and build there, so modules/ stays clean. The
# .ko land in BUILD_OUT. Override BUILD_OUT= to relocate.
BUILD_OUT="${BUILD_OUT:-$BUILD_DIR/ml-modules}"
TC="$(find "$BUILD_DIR/toolchain" -name "${CROSS_COMPILE_PREFIX}gcc" | head -1)"
TC="${TC%${CROSS_COMPILE_PREFIX}gcc}${CROSS_COMPILE_PREFIX}"

[ -f "$KTREE/.config" ] || { echo "no configured kernel tree at $KTREE - run scripts/build.sh first"; exit 1; }
# modpost needs Module.symvers for core symbol resolution; the Image build only
# leaves vmlinux.symvers, so seed it. MODVERSIONS is off (configs/modules.config),
# so vmlinux.symvers carries zero CRCs and the kernel checks only vermagic - this
# plain copy still hands modpost exactly what the tree exports.
[ -f "$KTREE/Module.symvers" ] || cp "$KTREE/vmlinux.symvers" "$KTREE/Module.symvers"

# The reproducible Image is built in a container whose glibc is NEWER than this host's,
# so it leaves the kbuild host tools (scripts/basic/fixdep, scripts/mod/modpost,
# usr/gen_init_cpio, ...) as container-glibc ELFs that will not exec here. A host module
# build invokes fixdep per object and modpost at the end, and any config change (e.g.
# flipping MODVERSIONS) forces a rebuild that trips them: "fixdep: not found" / "GLIBC_2.x
# not found". Drop any host tool that is not runnable on this host so `make` rebuilds it
# with HOSTCC (the host's own gcc) before it is needed. Cheap and idempotent: a tool that
# already runs is kept, so this only rebuilds after a container Image build.
for t in scripts/basic/fixdep scripts/mod/modpost usr/gen_init_cpio \
	 scripts/kallsyms scripts/sorttable scripts/recordmcount ; do
	f="$KTREE/$t"
	[ -x "$f" ] || continue
	# Probe whether the tool can exec on this host. `|| rc=$?` keeps the failing
	# exec from tripping `set -e` (the non-runnable case is the whole point here).
	rc=0
	"$f" --version >/dev/null 2>&1 </dev/null || "$f" >/dev/null 2>&1 </dev/null || rc=$?
	# exit 126/127 == cannot exec (bad interpreter / glibc); anything else == it ran.
	case $rc in 126|127) echo "  dropping non-host host-tool $t (rebuilt host-native)"; rm -f "$f" ;; esac
done

# Out-of-tree .ko finalize needs scripts/module.lds (the module linker script). A
# kernel built with just `make Image dtbs` does NOT generate it, so the .ko link
# fails with "No rule to make target '<mod>.ko', needed by __modfinal". Prepare the
# tree's module infra if that script is missing (this also rebuilds any host tool we
# just dropped, via scripts_basic, before the module compile below).
[ -f "$KTREE/scripts/module.lds" ] || \
  run "modules_prepare" make -C "$KTREE" ARCH=arm64 CROSS_COMPILE="$TC" modules_prepare

if [ "${1:-build}" = clean ]; then
  rm -rf "$BUILD_OUT"
  echo "removed $BUILD_OUT"
  exit 0
fi

KVER="$(cat "$KTREE/include/config/kernel.release" 2>/dev/null || make -s -C "$KTREE" kernelrelease)"

# Build the in-tree =m modules (`scripts/build.sh` builds only Image+dtbs). This must be a
# full `make modules`: a trailing-slash subtree target (`make drivers/gpu/drm/`) compiles the
# objects but never runs modpost, so no .ko is linked and the staging below silently ships an
# incomplete set on a fresh tree (DRM stack + wave5 missing -> broken image). The defconfig
# is trimmed, so the =m set is small; the first build on a fresh tree pays the full cost and
# later runs are incremental. Symbol resolution stays intact: everything the out-of-tree
# Artosyn modules import (INPUT/IIO/SPI cores, artosyn_adc's devm_iio_device_*) is built-in
# =y - its symbols are already in vmlinux.symvers (seeded into Module.symvers above).
echo "=== building in-tree modules ==="
run "in-tree modules" make -C "$KTREE" ARCH=arm64 CROSS_COMPILE="$TC" -j"$(nproc)" modules

# stage sources out of the repo, then build there (keeps modules/ clean)
rm -rf "$BUILD_OUT"
mkdir -p "$BUILD_OUT"
cp "$HERE"/*.c "$HERE"/*.h "$HERE"/Kbuild "$BUILD_OUT"/
run "out-of-tree modules" make -C "$KTREE" M="$BUILD_OUT" ARCH=arm64 CROSS_COMPILE="$TC" modules
echo "=== built out-of-tree (repo source tree stays clean) ==="
ls -1 "$BUILD_OUT"/*.ko

# Stage ONLY the modules we actually ship and load: the out-of-tree Artosyn modules plus the
# in-tree DRM stack (drivers/gpu/drm = drm core + helpers + dw-mipi-dsi + our DRM_ARTOSYN).
# `make modules` above still compiles the whole defconfig =m set (Kbuild has no per-module
# build), but installing all of it with `modules_install` would dump ~350 .ko we never load
# into the rootfs, and userapp1 is tight. So we copy a whitelist instead of modules_install.
# (IIO/input/SPI are built-in =y now; if any go back to =m, add their .ko to the whitelist.)
STAGE="$BUILD_OUT/rootfs"
MODDIR="$STAGE/lib/modules/$KVER/kernel"
rm -rf "$STAGE"; mkdir -p "$MODDIR"
find "$KTREE/drivers/gpu/drm" -name '*.ko' -exec cp -t "$MODDIR" {} +   # in-tree DRM stack
# Open codec: the wave5 V4L2 driver (=m, CONFIG_VIDEO_WAVE_VPU in codec.config) plus the
# V4L2 M2M + videobuf2 modules it depends on (these are select-only symbols the =m driver
# forces to =m; videodev itself is built-in =y). depmod below writes modules.dep so
# `modprobe wave5` pulls the chain. Needs the firmware on the rootfs at
# /lib/firmware/cnm/wave521c_k3_codec_fw.bin (chagall.bin).
find "$KTREE/drivers/media/v4l2-core" "$KTREE/drivers/media/common/videobuf2" \
     "$KTREE/drivers/media/platform/chips-media/wave5" \
     -name '*.ko' -exec cp -t "$MODDIR" {} + 2>/dev/null || true
cp "$BUILD_OUT"/*.ko "$MODDIR"/ 2>/dev/null || true                     # out-of-tree Artosyn
# Reference-only MPP-stack modules: compile-checked above
# so they do not rot, but never shipped - nothing on the open stack loads them.
rm -f "$MODDIR"/ar_osal.ko "$MODDIR"/ar_vb.ko "$MODDIR"/ar_sys.ko "$MODDIR"/ar_sysctl.ko \
      "$MODDIR"/ar_mpp_drv.ko "$MODDIR"/ar_mpp_proc_ctrl.ko "$MODDIR"/ar_mpp_overlay.ko \
      "$MODDIR"/ar_scaler.ko
# dmatest (=m, CONFIG_DMATEST in dma.config): the bring-up validator for the dw-axi-dmac
# copy engine (test_tools/dmatest-axi-dma.sh). The engine itself is built-in (=y);
# this is only the test client. Drop it once the ml-dmablit path is proven on HW.
find "$KTREE/drivers/dma" -name 'dmatest.ko' -exec cp -t "$MODDIR" {} + 2>/dev/null || true
# Ship the built-in module manifests (modules.builtin + modules.builtin.modinfo) from the
# kernel build. They are inputs to depmod, not products of it: without modules.builtin in
# the tree, depmod cannot see which drivers are =y and writes EMPTY modules.builtin.bin /
# modules.builtin.alias.bin (the "could not open modules.builtin" warning below). Staging
# them here means the build-time depmod produces correct binary indexes AND the plain-text
# files remain in the image, so an on-device `depmod` (busybox refuses to run without the
# text modules.builtin) works without a stub. modules.order is intentionally NOT copied: it
# lists the whole =m set we do not ship, and depmod tolerates its absence.
cp "$KTREE/modules.builtin" "$KTREE/modules.builtin.modinfo" "$STAGE/lib/modules/$KVER/" 2>/dev/null || true
# depmod lives in /sbin (kmod), which is NOT on a normal user's PATH, so `depmod` alone is
# "command not found" and modules.dep silently never gets written (modprobe deps then fail).
# Resolve the real path. Any residual modules.order warning is harmless (we ship a subset).
# Fail loudly if the critical in-tree modules did not make it: an image staged without the
# codec or the display stack boots to a black screen with no video and the cause is invisible.
for critical in wave5.ko artosyn_vo.ko; do
	[ -f "$MODDIR/$critical" ] || { echo "FATAL: $critical missing from stage (in-tree modules build incomplete)"; exit 1; }
done
DEPMOD="$(command -v depmod || echo /sbin/depmod)"; [ -x "$DEPMOD" ] || DEPMOD=/usr/sbin/depmod
"$DEPMOD" -b "$STAGE" "$KVER" 2>&1 | grep -v 'WARNING: could not open modules\.' || true
[ -f "$STAGE/lib/modules/$KVER/modules.dep" ] || echo "  WARN: depmod produced no modules.dep (modprobe deps will fail)"
echo "=== staged /lib/modules/$KVER (whitelist: Artosyn + DRM stack + wave5 codec) at: $STAGE ==="
echo "  staged .ko: $(find "$MODDIR" -name '*.ko' | wc -l)  (the rest of the =m set is built but not shipped)"
