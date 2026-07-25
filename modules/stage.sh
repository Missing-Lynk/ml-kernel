#!/usr/bin/env bash
# Build and stage the kernel modules the image ships, against an already-configured, already-
# Image-built kernel tree. Shared by two callers so the logic lives in one place:
#   scripts/container-build.sh - the shipping path: runs this inside the hermetic container
#                                right after the Image, so the .ko are as reproducible as the
#                                Image and carry its exact toolchain + vermagic.
#   modules/build.sh           - the host-side dev fast-path for iterating on a single module.
#
# It builds the in-tree =m set (drm core + helpers + dw-mipi-dsi + DRM_ARTOSYN, plus the wave5
# codec and its v4l2/videobuf2 deps) and the out-of-tree Artosyn modules, then copies ONLY the
# whitelist we actually load into $OUT/rootfs and runs depmod there. `make modules` compiles
# the whole =m set (Kbuild has no per-module target); installing all of it with modules_install
# would dump ~350 .ko we never load into the tight userapp1 partition, so we copy a whitelist.
#
# Inputs (environment):
#   KTREE  configured + Image-built kernel tree (has .config, vmlinux.symvers)
#   CROSS  CROSS_COMPILE prefix (e.g. /tc/bin/aarch64-linux- in-container, a host path on host)
#   MODSRC directory holding the out-of-tree module sources (*.c *.h Kbuild)
#   OUT    output base: out-of-tree objects land in $OUT/build, the staged image in $OUT/rootfs
#   JOBS   parallelism (default: nproc)
set -eu

: "${KTREE:?stage.sh: KTREE not set}"
: "${CROSS:?stage.sh: CROSS not set}"
: "${MODSRC:?stage.sh: MODSRC not set}"
: "${OUT:?stage.sh: OUT not set}"
JOBS="${JOBS:-$(nproc)}"

KVER="$(cat "$KTREE/include/config/kernel.release" 2>/dev/null || make -s -C "$KTREE" kernelrelease)"

# modpost needs Module.symvers for core symbol resolution; a tree built with only `make Image`
# leaves just vmlinux.symvers. MODVERSIONS is off, so vmlinux.symvers carries zero CRCs and the
# kernel checks only vermagic - this plain copy hands modpost exactly what the tree exports.
[ -f "$KTREE/Module.symvers" ] || cp "$KTREE/vmlinux.symvers" "$KTREE/Module.symvers"

# Out-of-tree .ko finalize needs scripts/module.lds (the module linker script), which a
# `make Image dtbs` tree does not generate. Prepare the module infra if it is missing.
[ -f "$KTREE/scripts/module.lds" ] || make -C "$KTREE" ARCH=arm64 CROSS_COMPILE="$CROSS" modules_prepare

echo "=== building in-tree modules ==="
make -C "$KTREE" ARCH=arm64 CROSS_COMPILE="$CROSS" -j"$JOBS" modules

echo "=== building out-of-tree Artosyn modules ==="
BUILD_OUT="$OUT/build"
rm -rf "$BUILD_OUT"
mkdir -p "$BUILD_OUT"
cp "$MODSRC"/*.c "$MODSRC"/*.h "$MODSRC"/Kbuild "$BUILD_OUT"/
make -C "$KTREE" M="$BUILD_OUT" ARCH=arm64 CROSS_COMPILE="$CROSS" -j"$JOBS" modules

# Stage ONLY the modules we ship and load: the in-tree DRM stack (drm core + helpers +
# dw-mipi-dsi + DRM_ARTOSYN), the wave5 codec + its v4l2/videobuf2 deps, the out-of-tree
# Artosyn modules, and the dmatest bring-up client.
STAGE="$OUT/rootfs"
MODDIR="$STAGE/lib/modules/$KVER/kernel"
rm -rf "$STAGE"
mkdir -p "$MODDIR"

find "$KTREE/drivers/gpu/drm" -name '*.ko' -exec cp -t "$MODDIR" {} +
find "$KTREE/drivers/media/v4l2-core" "$KTREE/drivers/media/common/videobuf2" \
     "$KTREE/drivers/media/platform/chips-media/wave5" \
     -name '*.ko' -exec cp -t "$MODDIR" {} + 2>/dev/null || true
cp "$BUILD_OUT"/*.ko "$MODDIR"/ 2>/dev/null || true

# Reference-only MPP-stack modules: compile-checked so they do not rot, but never shipped -
# nothing on the open stack loads them.
rm -f "$MODDIR"/ar_osal.ko "$MODDIR"/ar_vb.ko "$MODDIR"/ar_sys.ko "$MODDIR"/ar_sysctl.ko \
      "$MODDIR"/ar_mpp_drv.ko "$MODDIR"/ar_mpp_proc_ctrl.ko "$MODDIR"/ar_mpp_overlay.ko
find "$KTREE/drivers/dma" -name 'dmatest.ko' -exec cp -t "$MODDIR" {} + 2>/dev/null || true

# Ship the built-in module manifests (modules.builtin + modules.builtin.modinfo) so depmod can
# see which drivers are =y (without them it writes empty modules.builtin.bin) and an on-device
# depmod works (busybox refuses to run without the text modules.builtin). modules.order is
# intentionally NOT copied: it lists the whole =m set we do not ship, and depmod tolerates it.
cp "$KTREE/modules.builtin" "$KTREE/modules.builtin.modinfo" "$STAGE/lib/modules/$KVER/" 2>/dev/null || true

# Fail loudly if a module we must ship did not make it: an image staged without the codec or
# the display stack boots to a black screen with no video and the cause is invisible. wave5 is
# required on every device; artosyn_vo (the DRM display controller) only when the kernel was
# configured with a display (CONFIG_DRM_ARTOSYN) - the air unit has no panel.
CRITICAL="wave5.ko"
grep -q '^CONFIG_DRM_ARTOSYN=' "$KTREE/.config" && CRITICAL="$CRITICAL artosyn_vo.ko"
for critical in $CRITICAL; do
  [ -f "$MODDIR/$critical" ] || { echo "FATAL: $critical missing from stage (in-tree modules build incomplete)"; exit 1; }
done

DEPMOD="$(command -v depmod || echo /sbin/depmod)"
[ -x "$DEPMOD" ] || DEPMOD=/usr/sbin/depmod
"$DEPMOD" -b "$STAGE" "$KVER" 2>&1 | grep -v 'WARNING: could not open modules\.' || true
[ -f "$STAGE/lib/modules/$KVER/modules.dep" ] || echo "  WARN: depmod produced no modules.dep (modprobe deps will fail)"

echo "=== staged $(find "$MODDIR" -name '*.ko' | wc -l) .ko at $STAGE (whitelist: Artosyn + DRM stack + wave5 codec) ==="
