# overlay - full-file drivers we own

Complete source files for built-in drivers we wrote that have **no mainline counterpart**. `scripts/container-build.sh` copies them onto the pinned kernel tree (`cp`) and wires their obj/Kconfig hooks, before the config merge. One `.c`/`.h` at its kernel-relative path; grep-guarded obj/Kconfig lines go in the overlay block of the build script.

For diffs to *existing* mainline files, see `patches/` instead.

## Contents
- `drivers/clk/clk-ar9311-cgu.c` - AR9311 (Proxima) CGU common-clock provider (`obj-y`). See `../docs/clocks.md`.
- `drivers/spi/spi-ar9301.c` - AR9301 QSPI controller for the SPI-NAND (`obj-y`). See `../../docs/reference/datasheets/carrier-board.md`.
- `drivers/gpu/drm/artosyn/` - DRM/KMS display stack (VO CRTC + dw-mipi-dsi glue + QY45043A0 panel). Built as a module (`CONFIG_DRM_ARTOSYN=m`), but must be in-tree rather than out-of-tree because its Kconfig `select`s select-only DRM helper symbols. See `../docs/display-backlight.md`.
