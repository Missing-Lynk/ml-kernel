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
# dw-mipi-dsi + DRM_ARTOSYN), the wave5 codec + its v4l2/videobuf2 deps, the Artosyn camera
# capture drivers, the out-of-tree Artosyn modules, and the dmatest bring-up client.
STAGE="$OUT/rootfs"
MODDIR="$STAGE/lib/modules/$KVER/kernel"
rm -rf "$STAGE"
mkdir -p "$MODDIR"

# copy_kos <dir>...: copy every .ko under each dir that exists. A source dir legitimately
# absent for this board's config is skipped; a cp that fails is not, because a partially
# staged module set produces an image that boots and is missing a driver.
copy_kos(){
  local d
  for d in "$@"; do
    [ -d "$d" ] || continue
    find "$d" -name '*.ko' -exec cp -t "$MODDIR" {} +
  done
}

copy_kos "$KTREE/drivers/gpu/drm"
copy_kos "$KTREE/drivers/media/v4l2-core" "$KTREE/drivers/media/common/videobuf2" \
         "$KTREE/drivers/media/platform/chips-media/wave5" \
         "$KTREE/drivers/media/artosyn"

# Persistent store: ramoops keeps the console across a watchdog reset, and PSTORE_RAM hard-selects
# reed_solomon, so modprobe needs both staged to resolve encode_rs8 through modules.dep. Built only
# where the board's fragments turn pstore on, and copy_kos skips a directory that does not exist.
copy_kos "$KTREE/fs/pstore" "$KTREE/lib/reed_solomon"

# The out-of-tree build above is unconditional, so an empty $BUILD_OUT means it produced
# nothing and the whitelist below would silently ship without the Artosyn modules.
compgen -G "$BUILD_OUT/*.ko" >/dev/null \
  || { echo "FATAL: no out-of-tree .ko in $BUILD_OUT (the M= build produced nothing)"; exit 1; }
cp "$BUILD_OUT"/*.ko "$MODDIR"/

# Reference-only MPP-stack modules: compile-checked so they do not rot, but never shipped -
# nothing on the open stack loads them.
rm -f "$MODDIR"/ar_osal.ko "$MODDIR"/ar_vb.ko "$MODDIR"/ar_sys.ko "$MODDIR"/ar_sysctl.ko \
      "$MODDIR"/ar_mpp_drv.ko "$MODDIR"/ar_mpp_proc_ctrl.ko "$MODDIR"/ar_mpp_overlay.ko
# dmatest is a bring-up client, built only when CONFIG_DMATEST=m, so its absence is normal.
find "$KTREE/drivers/dma" -name 'dmatest.ko' -exec cp -t "$MODDIR" {} +

# Ship the built-in module manifests (modules.builtin + modules.builtin.modinfo) so depmod can
# see which drivers are =y (without them it writes empty modules.builtin.bin) and an on-device
# depmod works (busybox refuses to run without the text modules.builtin). modules.order is
# intentionally NOT copied: it lists the whole =m set we do not ship, and depmod tolerates it.
cp "$KTREE/modules.builtin" "$KTREE/modules.builtin.modinfo" "$STAGE/lib/modules/$KVER/"

# Fail loudly if a module we must ship did not make it: an image staged without the codec or
# the display stack boots to a black screen with no video and the cause is invisible. wave5 is
# required on every device; artosyn_vo (the DRM display controller) only when the kernel was
# configured with a display (CONFIG_DRM_ARTOSYN) - the air unit has no panel.
CRITICAL="wave5.ko"
grep -q '^CONFIG_DRM_ARTOSYN=' "$KTREE/.config" && CRITICAL="$CRITICAL artosyn_vo.ko"
# Only where the board asked for pstore: an image whose modules-load.d force-loads ramoops but
# ships no ramoops.ko boots fine and silently keeps no post-mortem record at all.
grep -q '^CONFIG_PSTORE_RAM=m' "$KTREE/.config" && CRITICAL="$CRITICAL ramoops.ko reed_solomon.ko"
for critical in $CRITICAL; do
  [ -f "$MODDIR/$critical" ] || { echo "FATAL: $critical missing from stage (in-tree modules build incomplete)"; exit 1; }
done

# depmod, hard-failed at every step: a stage whose modules.dep is missing, truncated, or stale
# still looks like a successful build, and the breakage only appears on the device as a module
# that will not load. depmod is not always on PATH (it lives in sbin), so probe the usual paths.
DEPMOD="$(command -v depmod || true)"
if [ -z "$DEPMOD" ]; then
  for candidate in /sbin/depmod /usr/sbin/depmod; do
    if [ -x "$candidate" ]; then
      DEPMOD="$candidate"
      break
    fi
  done
fi
[ -n "$DEPMOD" ] || { echo "FATAL: no depmod found on PATH, /sbin, or /usr/sbin (install kmod)"; exit 1; }

# The output is captured rather than piped so depmod's own exit status survives (a pipeline
# reports only its last command, which is what used to swallow the failure). Only the
# modules.order/modules.builtin warning is filtered - that file is deliberately not staged.
DEPMOD_LOG="$(mktemp)"
trap 'rm -f "$DEPMOD_LOG"' EXIT
if ! "$DEPMOD" -b "$STAGE" "$KVER" >"$DEPMOD_LOG" 2>&1; then
  grep -v 'WARNING: could not open modules\.' "$DEPMOD_LOG" >&2 || true
  echo "FATAL: depmod failed for $KVER (stage at $STAGE)"
  exit 1
fi
grep -v 'WARNING: could not open modules\.' "$DEPMOD_LOG" || true

# A depmod that exits 0 over an empty or half-copied tree still writes nothing usable, so assert
# the result: modules.dep exists and carries a line for every module the image must ship.
MODULES_DEP="$STAGE/lib/modules/$KVER/modules.dep"
[ -f "$MODULES_DEP" ] || { echo "FATAL: depmod wrote no modules.dep at $MODULES_DEP (modprobe deps would fail on-device)"; exit 1; }
for critical in $CRITICAL; do
  grep -q "/$critical:" "$MODULES_DEP" \
    || { echo "FATAL: $critical is staged but absent from modules.dep - the dependency tree is stale or partial"; exit 1; }
done

echo "=== staged $(find "$MODDIR" -name '*.ko' | wc -l) .ko at $STAGE (whitelist: Artosyn + DRM stack + wave5 codec) ==="
