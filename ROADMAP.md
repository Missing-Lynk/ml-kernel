# Kernel: why open reimplementation

Companion to `README.md` (how to build/test/flash), `PERIPHERALS.md` (per-peripheral architecture), and `STATUS.md` (the current-progress table this doc does not duplicate).

## Why open reimplementation, not the vendor `.ko` or a BSP port

- No BSP source exists: Artosyn ships NDA-only, no public kernel/BSP/DT for the Proxima-9311.
- The vendor `.ko` cannot load on another kernel: `CONFIG_MODVERSIONS` + vermagic-locked, and a stock 4.9.38 build matches only ~49% of their symbol CRCs (the vendor patched core structs). Force-loading bypasses the version check but not the ABI.
- All vendor `.ko` declare `license=GPL`/`GPL v2`, so reverse-engineering and open reimplementation are on clean legal ground.
- The kernel side is thin on this hardware: the heavy proprietary logic lives in userspace blobs (retired) and device firmware (`bb_demo_gnd_d.img` RF baseband, `chagall.bin` codec microcode), both kernel-version-agnostic.
