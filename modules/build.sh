#!/usr/bin/env bash
# Host-side dev fast-path for the kernel modules.
#
# The SHIPPING build (`make kernel`) builds and stages modules inside the hermetic container
# (scripts/container-build.sh -> modules/stage.sh), so the .ko are as reproducible as the Image
# and carry its exact toolchain + vermagic. This script does the same build+stage on the HOST
# against the already-built 6.18.36 tree (the one scripts/build.sh produces) using the pinned
# crosstool, so a single-module edit-rebuild loop does not need a container spin-up. It writes
# the same stage layout ($BUILD_OUT/rootfs) that rootfs/build.sh reads. The actual build + stage
# logic lives in modules/stage.sh (shared with the container path); this script only adds the
# host-specific prep (see below) before invoking it.
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
# shellcheck source=../scripts/pin.env
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
TC="${TC%"${CROSS_COMPILE_PREFIX}"gcc}${CROSS_COMPILE_PREFIX}"

[ -f "$KTREE/.config" ] || { echo "no configured kernel tree at $KTREE - run scripts/build.sh first"; exit 1; }

# The reproducible Image is built in a container whose glibc is NEWER than this host's,
# so it leaves the kbuild host tools (scripts/basic/fixdep, scripts/mod/modpost,
# usr/gen_init_cpio, ...) as container-glibc ELFs that will not exec here. A host module
# build invokes fixdep per object and modpost at the end, and any config change (e.g.
# flipping MODVERSIONS) forces a rebuild that trips them: "fixdep: not found" / "GLIBC_2.x
# not found". Drop any host tool that is not runnable on this host so `make` rebuilds it
# with HOSTCC (the host's own gcc) before it is needed. Cheap and idempotent: a tool that
# already runs is kept, so this only rebuilds after a container Image build.
# usr/gen_init_cpio is deliberately NOT in this list: no module build invokes it, so deleting it
# here would never be followed by a rebuild, and initramfs/build.sh would find no generator.
# It heals itself there instead.
for t in scripts/basic/fixdep scripts/mod/modpost \
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

if [ "${1:-build}" = clean ]; then
  rm -rf "$BUILD_OUT"
  echo "removed $BUILD_OUT"
  exit 0
fi

# Build + stage the modules via the shared script (also used by the container path). The host
# prep above (dropping non-host host-tools) has already run, so stage.sh's modules_prepare and
# compile use host-native tools. stage.sh writes $BUILD_OUT/build (out-of-tree objects) and
# $BUILD_OUT/rootfs (the whitelisted stage rootfs/build.sh reads), and fails loudly if a module
# we must ship (wave5, or artosyn_vo on a display kernel) did not make it.
run "modules+stage" \
  env KTREE="$KTREE" CROSS="$TC" MODSRC="$HERE" OUT="$BUILD_OUT" JOBS="$(nproc)" \
  bash "$HERE/stage.sh"
echo "=== staged /lib/modules (whitelist: Artosyn + DRM stack + wave5 codec) at: $BUILD_OUT/rootfs ==="
echo "  staged .ko: $(find "$BUILD_OUT/rootfs" -name '*.ko' 2>/dev/null | wc -l)  (the rest of the =m set is built but not shipped)"
