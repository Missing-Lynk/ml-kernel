#!/usr/bin/env bash
# Assemble the bare-kernel boot-test initramfs (static aarch64 busybox + init +
# /dev/console) into a gzipped cpio for `booti <kernel> <initrd> <dtb>`. Uses the
# kernel's gen_init_cpio so the device nodes need no root.
#
#   [GEN=<linux>/usr/gen_init_cpio] [BB=<static-busybox>] initramfs/build.sh [out.cpio.gz]
#
# GEN defaults to the built kernel's gen_init_cpio (discovered via scripts/pin.env +
# BUILD_DIR, like the rest of the kernel tooling); the static aarch64 busybox is built by
# ./build-busybox.sh to ./build/busybox-aarch64, the default BB. Artifacts live under
# ./build/ (git-ignored).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${GEN:-}" ]; then
  # shellcheck disable=SC1091
  source "$HERE/../scripts/pin.env"
  BUILD_DIR="${BUILD_DIR:-$KERNEL_BUILD_DEFAULT}"
  GEN="$BUILD_DIR/linux/usr/gen_init_cpio"
fi

BB="${BB:-$HERE/build/busybox-aarch64}"
OUT="${1:-$HERE/build/initramfs.cpio.gz}"

[ -x "$GEN" ] || { echo "no gen_init_cpio at $GEN - build the kernel first (scripts/build.sh) or set GEN="; exit 1; }
[ -f "$BB" ] || { echo "no busybox at $BB - run ./build-busybox.sh (or set BB=)"; exit 1; }
file "$BB" | grep -q 'aarch64.*statically linked' || { echo "BB is not a static aarch64 binary"; exit 1; }

mkdir -p "$(dirname "$OUT")"
spec="$(mktemp)"

cat > "$spec" <<EOF
dir /dev 755 0 0
nod /dev/console 600 0 0 c 5 1
nod /dev/null 666 0 0 c 1 3
dir /proc 755 0 0
dir /sys 755 0 0
dir /bin 755 0 0
file /bin/busybox $BB 755 0 0
slink /bin/sh /bin/busybox 777 0 0
file /init $HERE/init 755 0 0
file /recover $HERE/recover 755 0 0
EOF

"$GEN" "$spec" | gzip -9 > "$OUT"
rm -f "$spec"
echo "initramfs -> $OUT ($(wc -c < "$OUT") bytes)"
