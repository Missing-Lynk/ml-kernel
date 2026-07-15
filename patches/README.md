# patches - downstream diffs against pinned mainline

Unified diffs against **existing** mainline `6.18.36` files that we only tweak (the version is pinned in `scripts/pin.env`). `scripts/container-build.sh` applies them with `patch -p1` (in `series` order) onto the freshly-extracted tree, before the config-fragment merge. One `.patch` per upstream file, so a kernel bump that touches the same file makes `patch` fail loudly instead of silently clobbering upstream. Every patch starts with a `Why:` preamble (ignored by `patch`) stating exactly why the change is necessary against upstream.

For drivers we wrote from scratch (no mainline counterpart), see `overlay/` instead.

## Series
One `.patch` per upstream file (path-based names), listed in `series`:
- `0001-arm64-cpu_ops.patch`, `0002-arm64-smp_spin_table.patch` - add the `ar-spin-table` SMP enable-method that matches the vendor SPL's secondary-A53 release protocol (recovered from the vendor SPL/kernel disassembly).
- `0003-wave5-Kconfig.patch` - drop the `ARCH_K3 || COMPILE_TEST` arch gate so `VIDEO_WAVE_VPU` is selectable on the pure-DT arm64 Proxima. Must precede the merge (`codec.config` relies on it).
- `0004`..`0010` - the Artosyn Proxima-9311 WAVE521C fixes to the wave5 codec (dedicated mmz pool routing, non-interruptible close-path locks that stop the pool leak, sizeimage clamps, power sequencing). Every change is tagged `ML (Artosyn)` / `Artosyn` in-context.
- `0011-dma-coherent-page-granular.patch` - the mainline per-device dma-coherent pool allocator (`bitmap_find_free_region`) rounds every allocation up to a power-of-2 page order and places it order-aligned, so an allocation can occupy up to ~2x its size (e.g. 4.8 MiB costs 8 MiB) and the pool returns ENOMEM well below its nominal capacity. Replaced with page-granular first-fit: `bitmap_find_next_zero_area` on allocate, plus a per-allocation page-count map so release does not depend on the caller's `get_order(size)`. Only wave5 binds an rmem coherent pool on this system.
- `0012-dwc2-gadget-buffer-dma.patch` - `params.c` auto-enables gadget descriptor DMA whenever the core advertises it (`p->g_dma_desc = hw->dma_desc_enable`), which the Proxima core does. Under a sustained bulk OUT transfer concurrent with video decode the DDMA OUT ring wedges and `dwc2_hsotg_ep_stop_xfr` times out on `GOUTNAKEFF`/`EPDisable`, dropping and re-enumerating the gadget (and freezing the picture as the recovery busy-waits starve the pipeline). Force `g_dma_desc = false` (buffer DMA, `g_dma` stays on) to match the vendor's Linux-4.9 BSP, which predates gadget DDMA and ran buffer DMA on this same core/DT/FIFO with no failure.

## Regenerating / editing a patch
Patches are the source of truth; there is no checked-in copy of the patched file. To change one:
1. Extract the pinned pristine file: `tar -xf <linux.tar.xz> linux-6.18.36/<path>`.
2. `patch -p1 < 000N-...patch` to reach the current downstream state, edit, then `diff -u --label a/<path> --label b/<path> <pristine> <edited> > 000N-...patch`.

## On a kernel bump
Bump `pin.env`, rebuild. If a patch fails to apply, upstream changed that file - inspect the reject, re-base the hunk against the new pristine source, regenerate. Do not paper over it by reverting to a full-file copy.
