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

GEN_DEFAULTED=0
if [ -z "${GEN:-}" ]; then
  # shellcheck disable=SC1091
  source "$HERE/../scripts/pin.env"
  BUILD_DIR="${BUILD_DIR:-$KERNEL_BUILD_DEFAULT}"
  KTREE="$BUILD_DIR/linux"
  GEN="$KTREE/usr/gen_init_cpio"
  GEN_DEFAULTED=1
fi

BB="${BB:-$HERE/build/busybox-aarch64}"
OUT="${1:-$HERE/build/initramfs.cpio.gz}"

# gen_usable <path>: true if the file exists and can actually exec on this host. Existence is
# not enough - scripts/build.sh builds the Image in a container whose glibc is newer than the
# host's, so the tree's usr/gen_init_cpio is often an ELF that exits 126/127 here.
gen_usable(){
  [ -x "$1" ] || return 1
  local rc=0
  "$1" >/dev/null 2>&1 </dev/null || rc=$?
  case $rc in 126|127) return 1 ;; *) return 0 ;; esac
}

# Self-heal the default generator rather than sending the caller off to build a whole kernel:
# a container Image build leaves a non-host ELF here, and building the kernel again just
# reproduces it.
#
# Compiled straight from its source, NOT via `make -C "$KTREE" usr/gen_init_cpio`. That target
# drags in kconfig, and on a host without ARCH=arm64 in the environment kconfig decides the
# architecture changed and drops into an interactive "Restart config..." prompt - it hangs, and
# answering it would rewrite the .config the Image was built from. gen_init_cpio.c is a
# single-file libc-only host tool, so ${HOSTCC:-cc} is all it needs.
if [ "$GEN_DEFAULTED" = 1 ] && ! gen_usable "$GEN"; then
  src="$KTREE/usr/gen_init_cpio.c"
  [ -f "$src" ] || {
    echo "no kernel tree at $KTREE - run scripts/build.sh first (or set GEN=)"; exit 1; }
  echo "[initramfs] gen_init_cpio missing or not host-runnable, compiling it for this host" >&2
  rm -f "$GEN"
  "${HOSTCC:-cc}" -O2 -o "$GEN" "$src"
fi

gen_usable "$GEN" || { echo "no usable gen_init_cpio at $GEN - build the kernel first (scripts/build.sh) or set GEN="; exit 1; }
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
