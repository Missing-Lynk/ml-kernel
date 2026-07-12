#!/usr/bin/env bash
# lint.sh - run the kernel's own checkpatch.pl over the in-repo kernel C sources.
#
# Everything in this repo (out-of-tree modules, the in-tree driver overlay, and the
# on-device test tools) is held to kernel coding style, so it reads the same as the tree
# it patches into. checkpatch.pl is the canonical enforcer, and we run the copy that ships
# with the pinned kernel source (scripts/pin.env) so the rules match the kernel we
# actually build against.
#
#   scripts/lint.sh                 # lint every zone (modules overlay test_tools)
#   scripts/lint.sh modules         # one zone
#   scripts/lint.sh path/to/file.c  # explicit file(s)
#   STRICT=1 scripts/lint.sh        # add checkpatch --strict (pedantic extra checks)
#   CHECKPATCH=/path/to/checkpatch.pl ...  # override which checkpatch to use
#
# Exit 0 = clean, 1 = checkpatch reported findings in at least one file, 2 = setup error.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
KDIR="$REPO"

# shellcheck disable=SC1091
source "$HERE/pin.env"
BUILD_DIR="${BUILD_DIR:-$KERNEL_BUILD_DEFAULT}"

log(){ echo "[lint] $*" >&2; }
die(){ echo "[lint] ERROR: $*" >&2; exit 2; }

# Zones = subtrees whose tracked .c/.h are linted. Order is display order.
ZONES=(modules overlay test_tools)

# Generated sources that are not hand-written kernel code (blobs emitted from .dts by the
# module build). Excluded by basename glob; add more here as needed.
EXCLUDE_GLOBS=('*_dtbo.h')

# checkpatch options shared by every invocation. -f = check source files (not patches);
# --no-tree = we live outside a kernel tree; --show-types prints each check's name so a
# fix pass can target or selectively --ignore individual checks; --typedefsfile teaches it
# the libc opaque types the userspace test tools use (else it misparses "DIR *d").
CHECKPATCH_OPTS=(--no-tree -f --show-types --max-line-length=100 --typedefsfile="$HERE/checkpatch-typedefs")
[ -n "${STRICT:-}" ] && CHECKPATCH_OPTS+=(--strict)

# Checks that assume kernel-internal APIs and so are false positives on the userspace test
# tools (kernel zones keep them). VOLATILE: `volatile` is the correct idiom for a userspace
# /dev/mem MMIO mapping. STRNCPY: strscpy is kernel-only. ARRAY_SIZE / PREFER_DEFINED_
# ATTRIBUTE_MACRO: ARRAY_SIZE() and `noinline` are kernel macros, absent from libc.
USERSPACE_IGNORE=VOLATILE,STRNCPY,ARRAY_SIZE,PREFER_DEFINED_ATTRIBUTE_MACRO

# --- locate checkpatch.pl ----------------------------------------------------------------
# Prefer an explicit override, then a full build tree, then a lint-only extract of just
# scripts/ from the pinned tarball (so linting needs no multi-GB kernel build). Whichever
# we pick, we run it in place so its sibling spelling.txt / const_structs.checkpatch resolve.
resolve_checkpatch(){
  if [ -n "${CHECKPATCH:-}" ]; then
    [ -x "$CHECKPATCH" ] || die "CHECKPATCH=$CHECKPATCH not executable"
    return
  fi

  local from_build="$BUILD_DIR/linux/scripts/checkpatch.pl"
  if [ -f "$from_build" ]; then
    CHECKPATCH="$from_build"
    return
  fi

  # Lint-only cache: extract just scripts/ from the pinned, sha-verified kernel tarball.
  local lint_tree="$BUILD_DIR/lint/linux-${KERNEL_VERSION}"
  local cached="$lint_tree/scripts/checkpatch.pl"
  if [ ! -f "$cached" ]; then
    local tarball="$BUILD_DIR/dl/linux.tar.xz"
    if [ ! -f "$tarball" ] || ! echo "$KERNEL_SHA256  $tarball" | sha256sum -c - >/dev/null 2>&1; then
      log "fetch kernel source for checkpatch (${KERNEL_VERSION})"
      mkdir -p "$BUILD_DIR/dl"
      curl -fSL "$KERNEL_URL" -o "$tarball.tmp"
      echo "$KERNEL_SHA256  $tarball.tmp" | sha256sum -c - >/dev/null || die "kernel tarball sha256 mismatch"
      mv "$tarball.tmp" "$tarball"
    fi
    log "extract scripts/ -> $lint_tree"
    mkdir -p "$lint_tree"
    tar -C "$lint_tree" --strip-components=1 -xf "$tarball" "linux-${KERNEL_VERSION}/scripts"
  fi
  CHECKPATCH="$cached"
}

# --- collect the files for a zone --------------------------------------------------------
zone_files(){  # zone -> tracked .c/.h, minus excludes, one per line (paths relative to KDIR)
  local zone="$1" f keep
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    keep=1
    for g in "${EXCLUDE_GLOBS[@]}"; do
      # shellcheck disable=SC2053
      [[ "$(basename "$f")" == $g ]] && { keep=0; break; }
    done
    [ "$keep" = 1 ] && echo "$f"
  done < <(git -C "$KDIR" ls-files "$zone" | grep -E '\.(c|h)$' || true)
}

resolve_checkpatch
command -v perl >/dev/null || die "perl not found (checkpatch.pl needs it)"
log "using $CHECKPATCH"

# Build the work list: explicit path args, else the default/selected zones.
declare -a targets=()
declare -a labels=()
if [ "$#" -gt 0 ]; then
  for a in "$@"; do
    if printf '%s\n' "${ZONES[@]}" | grep -qx "$a"; then
      targets+=("$a"); labels+=("zone:$a")
    elif [ -f "$KDIR/$a" ]; then
      targets+=("$a"); labels+=("file:$a")
    elif [ -f "$a" ]; then
      # absolute or cwd-relative path: make it relative to KDIR for checkpatch's cwd
      targets+=("$(realpath --relative-to="$KDIR" "$a")"); labels+=("file:$a")
    else
      die "unknown zone or missing file: $a"
    fi
  done
else
  targets=("${ZONES[@]}")
  for z in "${ZONES[@]}"; do labels+=("zone:$z"); done
fi

# --- run ---------------------------------------------------------------------------------
rc=0
for i in "${!targets[@]}"; do
  t="${targets[$i]}"; label="${labels[$i]}"
  # Expand a zone to its file list; a file target is itself.
  declare -a files=()
  if [[ "$label" == zone:* ]]; then
    mapfile -t files < <(zone_files "$t")
  else
    files=("$t")
  fi
  if [ "${#files[@]}" -eq 0 ]; then
    log "$label: no lintable sources, skipping"
    continue
  fi

  # Userspace test tools skip the kernel-API-only checks (see USERSPACE_IGNORE).
  declare -a extra=()
  if [ "$label" = "zone:test_tools" ] || [[ "$t" == test_tools/* ]]; then
    extra=(--ignore "$USERSPACE_IGNORE")
  fi

  echo "-------- ${label}  (${#files[@]} file(s)) --------"
  # Run from KDIR so relative paths and any kernel/.checkpatch.conf resolve. checkpatch
  # returns non-zero when a file has findings; remember that but keep going.
  ( cd "$KDIR" && perl "$CHECKPATCH" "${CHECKPATCH_OPTS[@]}" "${extra[@]}" "${files[@]}" ) || rc=1
done

if [ "$rc" -eq 0 ]; then
  log "clean: no checkpatch findings"
else
  log "checkpatch reported findings (see above)"
fi
exit "$rc"
