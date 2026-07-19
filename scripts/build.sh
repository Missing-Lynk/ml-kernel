#!/usr/bin/env bash
# Reproducible modern-kernel build for the Artosyn goggle.
# Pinned source + toolchain (sha256-verified) + fixed build metadata + a hermetic
# container -> a bit-reproducible arm64 Image. Re-running yields the same Image sha256.
#
#   scripts/build.sh            # build (defconfig + our fragment if present)
#   scripts/build.sh verify     # build twice in separate trees, compare Image sha256
#   scripts/build.sh -v         # ...streaming full make/docker output (default: quiet,
#                                      # log tailed only on failure)
#
# Override BUILD_DIR=/path to relocate the (large) build tree. Default lives outside
# the repo so it never pollutes git. FAST=1 reuses the tree for an incremental make
# (dev loop; NOT reproducible - clean-build before flashing a slot). BOARD=<name> selects
# the device (devices/<name>/: its DTS + config-fragment list); default betafpv-vr04-goggle,
# so a bare build.sh is the goggle exactly as before.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

# shellcheck disable=SC1091
source "$HERE/pin.env"

VERBOSE="${VERBOSE:-0}"
args=()
for a in "$@"; do
  case "$a" in
    -v|--verbose) VERBOSE=1 ;;
    *) args+=("$a") ;;
  esac
done
set -- "${args[@]+"${args[@]}"}"

BUILD_DIR="${BUILD_DIR:-$KERNEL_BUILD_DEFAULT}"
JOBS="${JOBS:-$(nproc)}"
BOARD="${BOARD:-betafpv-vr04-goggle}"   # device to build (devices/<name>/); see header

# Everything except the final Image sha256 goes to stderr, so `verify` can capture
# do_build's stdout (the hash line) cleanly.
log(){ echo "[$(date -u +%H:%M:%S)] $*" >&2; }

fetch(){  # url sha256 outfile
  local url="$1" sha="$2" out="$3"
  if [ -f "$out" ] && echo "$sha  $out" | sha256sum -c - >/dev/null 2>&1; then
    return
  fi

  log "fetch $(basename "$out")"
  curl -fSL "$url" -o "$out.tmp"
  if ! echo "$sha  $out.tmp" | sha256sum -c - >/dev/null; then
    echo "SHA256 MISMATCH: $out"
    exit 1
  fi

  mv "$out.tmp" "$out"
}

prepare_image(){
  if docker image inspect "$BUILD_IMAGE" >/dev/null 2>&1; then
    return
  fi

  log "build hermetic image $BUILD_IMAGE"
  if [ "$VERBOSE" = 1 ]; then
    docker build -t "$BUILD_IMAGE" "$HERE" >&2
    return
  fi

  local dlog; dlog="$(mktemp)"
  if docker build -t "$BUILD_IMAGE" "$HERE" >"$dlog" 2>&1; then
    rm -f "$dlog"
  else
    echo "docker build FAILED, last 60 lines:" >&2
    tail -n 60 "$dlog" >&2
    rm -f "$dlog"
    exit 1
  fi
}

do_build(){  # tree-dir
  local tree="$1"
  mkdir -p "$BUILD_DIR/dl"
  fetch "$TOOLCHAIN_URL" "$TOOLCHAIN_SHA256" "$BUILD_DIR/dl/toolchain.tar.xz"
  fetch "$KERNEL_URL"    "$KERNEL_SHA256"    "$BUILD_DIR/dl/linux.tar.xz"

  if [ ! -d "$BUILD_DIR/toolchain" ]; then
    log "extract toolchain"
    mkdir -p "$BUILD_DIR/toolchain"
    tar -C "$BUILD_DIR/toolchain" --strip-components=1 -xf "$BUILD_DIR/dl/toolchain.tar.xz"
  fi

  local cc_bin tc_root
  cc_bin="$(dirname "$(find "$BUILD_DIR/toolchain" -name "${CROSS_COMPILE_PREFIX}gcc" | head -1)")"
  tc_root="$(dirname "$cc_bin")"   # the toolchain install root (has bin/ lib/ libexec/ sysroot)
  if [ -n "${FAST:-}" ] && [ -e "$tree/Makefile" ]; then
    # FAST: reuse the existing tree so make is incremental (dev loop). NOT bit-reproducible
    # (that is what the clean re-extract guarantees), and a deleted/renamed source can leave a
    # stale object behind. Do a clean build (unset FAST) before flashing a slot.
    log "FAST: reusing $(basename "$tree") (incremental; NOT reproducible - clean-build before flashing)"
  else
    rm -rf "$tree"
    mkdir -p "$tree"
    log "extract kernel -> $(basename "$tree")"
    tar -C "$tree" --strip-components=1 -xf "$BUILD_DIR/dl/linux.tar.xz"
  fi

  prepare_image
  log "configure + build (container, -j$JOBS)"
  docker run --rm --network none \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -v "$tree":/src \
    -v "$tc_root":/tc:ro \
    -v "$REPO":/repo:ro \
    -e ARCH=arm64 \
    -e CROSS_COMPILE="/tc/bin/${CROSS_COMPILE_PREFIX}" \
    -e KBUILD_BUILD_USER="$KBUILD_BUILD_USER" \
    -e KBUILD_BUILD_HOST="$KBUILD_BUILD_HOST" \
    -e SOURCE_DATE_EPOCH="$SOURCE_DATE_EPOCH" \
    -e MINIMAL="${MINIMAL:-}" \
    -e NOTRIM="${NOTRIM:-}" \
    -e DEBUGSDIO="${DEBUGSDIO:-}" \
    -e BOARD="$BOARD" \
    -e JOBS="$JOBS" \
    -e VERBOSE="$VERBOSE" \
    -e KBUILD_BUILD_TIMESTAMP="$(date -u -d "@$SOURCE_DATE_EPOCH")" \
    -w /src "$BUILD_IMAGE" \
    bash /repo/scripts/container-build.sh 1>&2

  # stdout = just the hashes (verify takes the Image line); Image + every built board dtb.
  ( cd "$tree" && sha256sum arch/arm64/boot/Image arch/arm64/boot/*.dtb )
}

# Pull just the Image hash out of do_build's (multi-line) stdout, robustly.
image_hash(){ grep 'boot/Image$' | awk '{print $1}'; }

# Create the build dir up front: the `build` pipeline's tee opens Image.sha256 inside it
# before do_build runs its own mkdir, so on a fresh checkout the dir must already exist.
mkdir -p "$BUILD_DIR"

case "${1:-build}" in
  build)
    do_build "$BUILD_DIR/linux" | tee "$BUILD_DIR/Image.sha256"
    log "DONE -> $BUILD_DIR/linux/arch/arm64/boot/{Image,<board>.dtb}"
    ;;
  verify)
    A="$(do_build "$BUILD_DIR/linux-A" | image_hash)"
    B="$(do_build "$BUILD_DIR/linux-B" | image_hash)"
    echo "A=$A"
    echo "B=$B"
    if [ -n "$A" ] && [ "$A" = "$B" ]; then
      echo "REPRODUCIBLE: identical Image sha256"
    else
      echo "NOT reproducible"
      exit 1
    fi
    ;;
  *)
    echo "usage: build.sh [build|verify]"
    exit 1
    ;;
esac
