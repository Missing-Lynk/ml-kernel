# patches - downstream diffs against pinned mainline

Unified diffs against **existing** mainline `6.18.36` files that we only tweak (the version is pinned in `scripts/pin.env`). `scripts/container-build.sh` applies them with `patch -p1` (in `series` order) onto the freshly-extracted tree, before the config-fragment merge. One `.patch` per upstream file, so a kernel bump that touches the same file makes `patch` fail loudly instead of silently clobbering upstream. Every patch starts with a `Why:` preamble (ignored by `patch`) stating exactly why the change is necessary against upstream.

For drivers we wrote from scratch (no mainline counterpart), see `overlay/` instead.

## Numbering

Three digits, grouped by subsystem, spaced by 10 so a new patch drops into its range without renaming anything:

| range | subsystem |
|---|---|
| `01xx` | arm64 SMP bring-up |
| `02xx` | wave5 codec |
| `03xx` | core kernel |
| `04xx` | usb |

Numbers are stable identifiers, not a sequence. They are cited from `kernel/STATUS.md`, `kernel/docs/`, and from `userspace/`, which is a separate repository, so a rename never moves atomically with its references. **Gaps are expected; do not renumber to close one.**

Apply order does not matter, because every patch owns exactly one upstream file and no patch edits another's added lines. `series` is sorted for readability. The single build-order constraint is external: `0200-wave5-Kconfig.patch` must precede the config-fragment merge, which `container-build.sh` already guarantees.

## Series

- `0100-arm64-cpu_ops.patch`, `0110-arm64-smp_spin_table.patch` - add the `ar-spin-table` SMP enable-method that matches the vendor SPL's secondary-A53 release protocol (recovered from the vendor SPL/kernel disassembly).
- `0200-wave5-Kconfig.patch` - drop the `ARCH_K3 || COMPILE_TEST` arch gate so `VIDEO_WAVE_VPU` is selectable on the pure-DT arm64 Proxima. Must precede the merge (`codec.config` relies on it).
- `0210`..`0280` - the Artosyn Proxima-9311 WAVE521C fixes to the wave5 codec (dedicated mmz pool routing, non-interruptible close-path locks that stop the pool leak, sizeimage clamps, power sequencing). Every change is tagged `ML (Artosyn)` / `Artosyn` in-context. Two device-local assumptions to NOT cherry-pick onto a general kernel: `0220` hardcodes `gdi_status_check_value = 0x3f` (exact match) and forces the decoder set with sec-AXI disabled, which breaks every non-Artosyn WAVE521C (TI K3 reads 0x00ff1f3f and would time out); `0240` sets `reorder_enable = FALSE` globally, correct for this device's no-B-frame streams but temporally scrambles any stream that actually uses reordering.
- `0210-wave5-helper.patch` and `0240-wave5-vpuapi.patch` together implement the close-path contract: a close that cannot quiesce the firmware instance (siblings live, or the safety reset failed) marks it `keep_dma_bufs`, and `wave5_cleanup_instance` then leaks the FBC/DPB framebuffers and bitstream ring rather than freeing memory the firmware still DMAs into (freeing it hard-hangs the SoC).
- `0250-wave5-vpuapi-h.patch` - the `struct vpu_instance` fields the above needs: `keep_dma_bufs`, and `active_enc_src_idx` (the OUTPUT buffer the encoder firmware currently owns, consumed by `0280`'s result-error path).
- **Live encoder parameter changes** span four of the patches above, one per file, rather than sitting in a patch of their own: `0220` implements `wave5_vpu_enc_change_param()` (`W5_ENC_SET_PARAM` with `OPT_CHANGE_PARAM`, plus the `GET_RESULT` drain the firmware's leftover report requires), `0255` declares it, `0240` marks changes pending and applies them between pictures, `0250` carries the pending mask and the forced-keyframe state, and `0280` wires the V4L2 controls. Bitrate, VBV, frame rate and forced keyframe all ride the one firmware command. HW-validated: exact bitrate at five steps, and a forced IDR that recovers a receiver which joined mid-stream. Do NOT re-try `W5_CMD_ENC_PIC_PIC_PARAM` for the keyframe - every plausible force-picture-type layout was swept on this firmware and produced no IDR.
- `0280-wave5-vpu-enc.patch` carries **every** downstream change to `wave5-vpu-enc.c`, including what were once separate patches for OUTPUT-plane `data_offset`, the wider-than-picture source stride, the coded height, and the `VLC_BUF_FULL` result-error recovery. They were collapsed because each was editing the previous one's added lines: a patch against our own patch means a kernel bump breaks several files' worth of context at once instead of failing loudly in the one place. Its `Why:` preamble enumerates the six changes.
- `0300-dma-coherent-page-granular.patch` - the mainline per-device dma-coherent pool allocator (`bitmap_find_free_region`) rounds every allocation up to a power-of-2 page order and places it order-aligned, so an allocation can occupy up to ~2x its size (e.g. 4.8 MiB costs 8 MiB) and the pool returns ENOMEM well below its nominal capacity. Replaced with page-granular first-fit: `bitmap_find_next_zero_area` on allocate, plus a per-allocation page-count map so release does not depend on the caller's `get_order(size)`. wave5 and ml_mmzheap share the single rmem coherent pool on this system. Caveat: this deliberately drops the DMA-API natural-alignment guarantee for every per-device coherent pool (neither tenant masks address bits; order-aligned placement is where the waste comes from) - a future pool user with address-bit-masked descriptor rings must not rely on `dma_alloc_coherent` alignment here. Also adds a read-only debugfs view per rmem-backed pool (`/sys/kernel/debug/dma_coherent/<rmem-name>`): base, pages used/free, largest free run, and the live allocation list - the shared wave5+ml_mmzheap pool is otherwise a black box and every fragmentation question becomes a live experiment.
- `0400-dwc2-gadget-buffer-dma.patch` - `params.c` auto-enables gadget descriptor DMA whenever the core advertises it (`p->g_dma_desc = hw->dma_desc_enable`), which the Proxima core does. Under a sustained bulk OUT transfer concurrent with video decode the DDMA OUT ring wedges and `dwc2_hsotg_ep_stop_xfr` times out on `GOUTNAKEFF`/`EPDisable`, dropping and re-enumerating the gadget (and freezing the picture as the recovery busy-waits starve the pipeline). Force `g_dma_desc = false` (buffer DMA, `g_dma` stays on) to match the vendor's Linux-4.9 BSP, which predates gadget DDMA and ran buffer DMA on this same core/DT/FIFO with no failure.

## Regenerating / editing a patch
Patches are the source of truth; there is no checked-in copy of the patched file. To change one:
1. Extract the pinned pristine file: `tar -xf <linux.tar.xz> linux-6.18.36/<path>`.
2. `patch -p1 < 0NNN-...patch` to reach the current downstream state, edit, then `diff -u --label a/<path> --label b/<path> <pristine> <edited> > 0NNN-...patch`.

Adding a change to a file that already has a patch means editing that patch, never adding a second one against the same file.

## On a kernel bump
Bump `pin.env`, rebuild. If a patch fails to apply, upstream changed that file - inspect the reject, re-base the hunk against the new pristine source, regenerate. Do not paper over it by reverting to a full-file copy.
