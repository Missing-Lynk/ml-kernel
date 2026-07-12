#!/usr/bin/env bash
# Build the static aarch64 busybox for the bare-kernel boot-test initramfs.
# Pinned source + a host aarch64 glibc cross-gcc -> a self-contained static binary
# (no runtime libc dep) that runs on the open kernel. Output: ./build/busybox-aarch64.
#
#   [CROSS_COMPILE=aarch64-linux-gnu-] initramfs/build-busybox.sh
#
# Toolchain: any aarch64 glibc cross prefix; on Debian/Ubuntu: `apt install gcc-aarch64-linux-gnu`.
# Other deps: make, curl, bzip2, file.
set -euo pipefail

# busybox version + its source-tarball sha256 (bump together)
VER=1.36.1
SHA=b8cc24c9574d809e7279c3be349795c5d5ceb6fdf19ca709f80cde50e47de314

URL="https://busybox.net/downloads/busybox-${VER}.tar.bz2"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CROSS_COMPILE="${CROSS_COMPILE:-aarch64-linux-gnu-}"
command -v "${CROSS_COMPILE}gcc" >/dev/null || { echo "missing ${CROSS_COMPILE}gcc (apt install gcc-aarch64-linux-gnu, or set CROSS_COMPILE)"; exit 1; }

W="$(mktemp -d)"; trap 'rm -rf "$W"' EXIT
echo "[busybox] fetch ${VER} + verify"
curl -fSL "$URL" -o "$W/bb.tar.bz2"
echo "$SHA  $W/bb.tar.bz2" | sha256sum -c - >/dev/null
tar -C "$W" -xf "$W/bb.tar.bz2"
cd "$W/busybox-${VER}"
make defconfig >/dev/null

# static link + drop applets that don't build against modern kernel headers
sed -i 's/# CONFIG_STATIC is not set/CONFIG_STATIC=y/' .config
sed -i 's/CONFIG_TC=y/# CONFIG_TC is not set/' .config
sed -i 's/CONFIG_FEATURE_WTMP=y/# CONFIG_FEATURE_WTMP is not set/' .config
yes "" | make oldconfig >/dev/null 2>&1
echo "[busybox] build (CROSS_COMPILE=$CROSS_COMPILE)"

make CROSS_COMPILE="$CROSS_COMPILE" -j"$(nproc)" >/dev/null
file busybox | grep -q 'aarch64.*statically linked' || { echo "ERROR: not a static aarch64 binary"; exit 1; }
OUTBB="$HERE/build/busybox-aarch64"

mkdir -p "$HERE/build"
cp busybox "$OUTBB"
echo "[busybox] -> $OUTBB ($(stat -c%s "$OUTBB") bytes, sha256 $(sha256sum "$OUTBB" | cut -c1-12))"
