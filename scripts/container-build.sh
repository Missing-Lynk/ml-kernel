#!/usr/bin/env bash
# container-build.sh - the in-container half of scripts/build.sh. It runs inside the hermetic
# docker image (build.sh does the `docker run`), with the kernel tree at /src (the workdir),
# the cross toolchain at /tc, and this repo read-only at /repo. It is a separate file so the
# build steps are readable instead of inlined in build.sh's `docker run`.
#
# All inputs arrive via the environment that build.sh sets on the `docker run`:
#   ARCH CROSS_COMPILE JOBS MINIMAL NOTRIM DEBUGSDIO KBUILD_BUILD_USER KBUILD_BUILD_HOST
#   KBUILD_BUILD_TIMESTAMP SOURCE_DATE_EPOCH VERBOSE BOARD
set -eu

# Which device we are building for: selects devices/$BOARD/ (its DTS + its config-fragment
# list). Default = the goggle, so a bare `build.sh` is unchanged. build.sh passes this through.
BOARD="${BOARD:-betafpv-vr04-goggle}"

# build_step <label> <cmd...>: quiet by default (log to a temp file, print "label OK" or the
# last 60 lines on failure); VERBOSE=1 streams the command's output live instead.
build_step() {
  label="$1"; shift
  if [ "${VERBOSE:-0}" = 1 ]; then
    "$@"
    return
  fi

  log="$(mktemp)"
  if "$@" >"$log" 2>&1; then
    echo "[$label] OK"
    rm -f "$log"
  else
    echo "[$label] FAILED, last 60 lines:" >&2
    tail -n 60 "$log" >&2
    rm -f "$log"
    exit 1
  fi
}

build_step defconfig make -j"$JOBS" defconfig

# Two ways our source reaches the tree, both BEFORE the config-fragment merge below so any
# Kconfig they add/change is visible to olddefconfig (overlay/README.md,
# patches/README.md explain the split):
#   overlay/ - full-file drop-in of drivers we own (no mainline counterpart): cp, idempotent.
#   patches/ - unified diffs against pinned mainline files we merely tweak: patch -p1, stamped
#              so a reused FAST tree is not double-applied.

# 1. Overlay our own drivers into the tree, then wire their Kconfig/Makefile hooks. The DRM
#    driver must register its Kconfig here so DRM_ARTOSYN exists and its `select`s of the
#    select-only DRM helper symbols (DRM_KMS_HELPER/DRM_GEM_DMA_HELPER/DRM_DW_MIPI_DSI/...)
#    take effect during olddefconfig. clk/spi are obj-y with no Kconfig symbol.
if [ -d /repo/overlay ]; then
  # Copy the mirrored tree subdirs (drivers/, ...) into place; skip overlay/README.md so it
  # does not land on the kernel's own top-level README.
  for d in /repo/overlay/*/; do
    cp -r "$d" ./
  done

  grep -q "drm/artosyn/Kconfig" drivers/gpu/drm/Kconfig || \
    echo "source \"drivers/gpu/drm/artosyn/Kconfig\"" >> drivers/gpu/drm/Kconfig
  grep -q "obj-y += artosyn/" drivers/gpu/drm/Makefile || \
    echo "obj-y += artosyn/" >> drivers/gpu/drm/Makefile

  grep -q "clk-ar9311-cgu.o" drivers/clk/Makefile || echo "obj-y += clk-ar9311-cgu.o" >> drivers/clk/Makefile
  grep -q "spi-ar9301.o" drivers/spi/Makefile || echo "obj-y += spi-ar9301.o" >> drivers/spi/Makefile
fi

# 2. Apply our downstream patches (arm64 ar-spin-table SMP method; Artosyn Proxima wave5 codec
#    fixes) in series order. One .patch per upstream file, so a kernel bump that touches the
#    same file makes patch fail loudly here instead of silently clobbering upstream. The wave5
#    Kconfig patch relaxes the arch gate that codec.config relies on, so this must precede the
#    merge. Stamp-guarded: a FAST-reused tree already carries the patches, do not re-apply.
if [ -d /repo/patches ] && [ ! -f .ml-patches-applied ]; then
  while read -r p; do
    [ -n "$p" ] || continue
    build_step "patch $p" patch -p1 --no-backup-if-mismatch -i "/repo/patches/$p"
  done < /repo/patches/series
  touch .ml-patches-applied
fi

# Merge our config fragments onto defconfig, IN ORDER (MINIMAL=1 skips this -> pure defconfig).
# Order matters: start from the platform config, then trim.config disables a lot to shrink the
# Image, then the per-board fragments re-enable the specific drivers that board needs - because
# they merge after trim, they override its disables. The per-board list lives in
# devices/$BOARD/fragments (kernel config stays in kernel land); the fragment FILES stay shared
# in configs/. Universal base (artosyn + trim) is applied here; DEBUGSDIO is appended last.
if [ -z "$MINIMAL" ] && [ -f /repo/configs/artosyn.config ]; then
  # Platform base: Artosyn Proxima SoC support (UART, USB gadget, SD, SPI-NAND, binder, ...).
  frags=/repo/configs/artosyn.config

  # trim.config strips kernel components we do not use - other vendors' SoC/board support,
  # plus unused subsystems and drivers - so the compressed Image fits the 6 MB kernel slot.
  # Skip the trimming with NOTRIM=1.
  [ -z "$NOTRIM" ] && [ -f /repo/configs/trim.config ] && frags="$frags /repo/configs/trim.config"

  # Per-board re-enables: devices/$BOARD/fragments lists fragment basenames, one per line, in
  # order (inline '# ...' comments and blank lines ignored). Each resolves to configs/<name>.config.
  bfrags="/repo/devices/$BOARD/fragments"
  if [ -f "$bfrags" ]; then
    while read -r name _rest; do
      case "$name" in ''|\#*) continue ;; esac
      f="/repo/configs/$name.config"
      [ -f "$f" ] || { echo "board $BOARD: fragments lists '$name' but configs/$name.config is missing" >&2; exit 1; }
      frags="$frags $f"
    done < "$bfrags"
  else
    echo "board $BOARD: no devices/$BOARD/fragments (device dir missing?)" >&2
    exit 1
  fi

  # Throwaway SDIO/MMC diagnostic fragment, opt-in via DEBUGSDIO=1, merged LAST.
  [ -n "$DEBUGSDIO" ] && [ -f /repo/configs/debug-sdio.config ] && frags="$frags /repo/configs/debug-sdio.config"
  # -Q silences the (expected) "redefined by fragment" notices: defconfig enables many drivers
  # as =m and our fragments turn them off, so the override warning would fire on every build.
  ./scripts/kconfig/merge_config.sh -m -Q .config $frags

  build_step olddefconfig make olddefconfig
fi

build_step "Image+dtbs" make -j"$JOBS" Image dtbs

# Build the selected board's DT standalone (cpp for the dt-bindings includes, then the
# freshly-built host dtc), output beside Image. The board's DTS lives in devices/$BOARD/;
# its basename sets the .dtb name (e.g. proxima-9311.dts -> proxima-9311.dtb).
for dts in /repo/devices/"$BOARD"/*.dts; do
  [ -e "$dts" ] || continue
  name="$(basename "$dts" .dts)"
  build_step "dtb $name (cpp)" cpp -nostdinc -undef -D__DTS__ -x assembler-with-cpp -I include "$dts" -o "$name.dts.i"
  build_step "dtb $name (dtc)" scripts/dtc/dtc -I dts -O dtb -o "arch/arm64/boot/$name.dtb" "$name.dts.i"
done
