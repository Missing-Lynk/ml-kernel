# patches - downstream diffs against pinned mainline

Unified diffs against **existing** mainline `6.18.36` files that we only tweak (the version is pinned in `scripts/pin.env`). `scripts/container-build.sh` applies them with `patch -p1` (in `series` order) onto the freshly-extracted tree, before the config-fragment merge. One `.patch` per upstream file, so a kernel bump that touches the same file makes `patch` fail loudly instead of silently clobbering upstream. Every patch starts with a `Why:` preamble (ignored by `patch`) stating the upstream behaviour, the failure it causes here, and the change. The preambles carry the detail; this file is the index.

For drivers we wrote from scratch (no mainline counterpart), see `overlay/` instead. Whether upstream would take any of this is tracked separately in `plans/upstream-patches.md`.

## Numbering

Three digits, grouped by subsystem, spaced by 10 so a new patch drops into its range without renaming anything:

| range | subsystem |
|---|---|
| `01xx` | arm64 SMP bring-up |
| `02xx` | wave5 codec |
| `03xx` | core kernel |
| `04xx` | usb |
| `05xx` | dmaengine |

Numbers are stable identifiers, not a sequence. They are cited from `kernel/STATUS.md`, `kernel/docs/`, and from `userspace/`, which is a separate repository, so a rename never moves atomically with its references. **Gaps are expected; do not renumber to close one.**

Apply order does not matter: every patch owns exactly one upstream file and no patch edits another's added lines. `series` is sorted for readability. The single build-order constraint is external, `0200-wave5-Kconfig.patch` must precede the config-fragment merge, which `container-build.sh` already guarantees.

## Series

### arm64 SMP bring-up

- `0100-arm64-cpu_ops.patch` - registers an `ar-spin-table` `cpu_operations`. Upstream resolves the DT `enable-method` against a fixed list that knows only `spin-table` and `psci`, so the DT cannot select the method `0110` implements.
- `0110-arm64-smp_spin_table.patch` - implements `ar-spin-table`. Mainline spin-table releases a secondary CPU by writing the entry point to `cpu-release-addr` and issuing SEV; the vendor SPL parking loop also polls a per-CPU handshake slot above `cpu-release-addr` and branches only once two magic words are present, so the mainline write alone never releases CPU1. Protocol recovered from the vendor SPL/kernel disassembly. Stock `spin-table` is kept unchanged alongside.

### wave5 codec

- `0200-wave5-Kconfig.patch` - drops `depends on ARCH_K3 || COMPILE_TEST` from `VIDEO_WAVE_VPU`. This is a pure-DT arm64 build that defines no `ARCH_*` symbol, so the gate makes the driver unselectable. Functional depends/selects unchanged.
- `0210-wave5-helper.patch` - instance teardown. Upstream takes interruptible locks and returns early on close failure; teardown runs from file release, where a SIGKILL leaves a fatal signal pending, so the lock returns `-EINTR` at once and the instance's FBC framebuffers, bitstream ring and work/task leak from the fixed mmz pool until power cycle. Locks made uninterruptible, every path reaches cleanup, and `wave5_cleanup_instance` honours `keep_dma_bufs` instead of unconditionally freeing.
- `0220-wave5-hw.patch` - four WAVE521C defects plus the live parameter-change command:
  - sec-AXI sizing is a mainline TODO for WAVE521C, so the driver enables sec-AXI into the 64 KiB SRAM with zero line-buffer sizes (`USE_SEC_AXI=0x8201`) and corrupts the write-back. Forced to 0, what the vendor writes.
  - `gdi_status_check_value` is `0x00ff1f3f` upstream; this integration reads back `0x3f`, so `wave5_vpu_reset()` times out and probe fails with `-EBUSY`. Exact-match against `0x3f`.
  - with two decode instances on the one VCPU, the DEC_PIC done interrupt can precede the result being readable and the query returns `WAVE5_SYSERR_RESULT_NOT_READY`. Upstream fails the picture; retry the query.
  - `wave5_vpu_enc_register_framebuffer`'s error path frees local `vpu_buf` copies whose dma handles were already published into `p_enc_info->vb_*`, so `wave5_vpu_enc_close()` double-frees them (seen live as 4x WARN in `__dma_release_from_coherent`). Free the `p_enc_info` copies instead.
  - `wave5_vpu_build_up_enc_param` declares the one device-global 64 KiB sec-AXI window to every encoder instance at CREATE_INSTANCE, handing two concurrent instances fully overlapping scratch. The vendor declares none; the writes are dropped.
  - adds `wave5_vpu_enc_change_param()` (see "Live encoder parameter changes" below).
- `0230-wave5-vdi.patch` - pool accounting. The codec allocates from a fixed dedicated pool and upstream gives no visibility into it, so exhaustion surfaces only as an opaque downstream V4L2 error. Adds a running per-allocation tally (`dev_dbg` per allocation, `dev_warn` with the tally on ENOMEM) plus `wave5_vdi_pool_used()` so a caller can report the total at a meaningful point. The per-allocation lines are 17 of a boot's 28 codec messages, which is why the accessor exists rather than the tally simply being logged. This kernel builds with `CONFIG_DYNAMIC_DEBUG` off, so the per-allocation trace is compiled out, not filtered: recovering it needs a rebuild, not a `dyndbg` write.
- `0240-wave5-vpuapi.patch` - dec/enc close paths, same SIGKILL leak shape as `0210` (interruptible device locks plus early returns on firmware finish/timeout failure). Locks made uninterruptible, buffers always freed, VPU stop/reset ordered before the frees. A close that cannot quiesce the firmware marks the instance `keep_dma_bufs`. Also sets `reorder_enable = FALSE` on the decoder (the vendor's DEC_PIC configuration); the low-latency no-B streams need no reorder delay.
- `0250-wave5-vpuapi-h.patch` - the `struct vpu_instance` fields the rest of the series consumes: `keep_dma_bufs`; `active_enc_src_idx`, the one OUTPUT buffer the encoder firmware owns, which `0280`'s result-error path returns directly because `v4l2_m2m_src_buf_remove()` can no longer find it; `enc_rate_change_pending`, the `W5_ENC_CHANGE_*` mask; and `enc_force_idr` / `enc_idr_restore` / `enc_idr_saved_period` for the forced keyframe. The period is saved rather than re-derived because for HEVC the seq-init path falls back to `avc_idr_period` when `intra_period` is zero, so `enc_param` is not the truth.
- `0255-wave5-h.patch` - the prototype for `wave5_vpu_enc_change_param()`. Upstream has no live parameter-change path, so it declares none.
- `0260-wave5-vpu.patch` - (1) the VPU sleep/wake handshake is broken on this silicon (wake after sleep never completes), so the suspend/resume sleep and wake calls are skipped; the vendor never uses them either. (2) Routes all codec DMA (DPB, WTL/display buffers, `vb_task`) to the SoC's dedicated mmz reserved-memory region via `of_reserved_mem` instead of the device default (global CMA). The attachment is released through a devres action, because `of_reserved_mem_device_init` has no devres form and would otherwise survive every probe-failure return and the remove path, leaving a rebind with stale `dma_mem` state.
- `0270-wave5-vpu-dec.patch` - decoder allocation sizing. Upstream derives the bitstream `sizeimage` from the coded resolution (huge at 1080p) and sizes the internal bitstream ring from the negotiated value, all out of the fixed mmz pool; both are clamped here, and pool-allocation failures get explicit diagnostics. Adds a `dec_cap_bufs` module param capping the LINEAR (display) CAPTURE count independently of FBC/DPB. Diagnostic only: capping below the stream's `fbc_buf_count` starves the firmware's display rotation.
- `0280-wave5-vpu-enc.patch` - **every** downstream change to `wave5-vpu-enc.c`. Collapsed into one patch because each change was editing the previous one's added lines, and a patch against our own patch means a kernel bump breaks several files' worth of context instead of failing loudly in one place. The nine changes, enumerated in full in its `Why:` preamble:
  1. `finish_encode` error path finishes the m2m job. Upstream returns without `v4l2_m2m_job_finish`, and the next STREAMOFF then deadlocks in `v4l2_m2m_cancel_job` (D-state holding the device mutex, VPU unusable until power cycle).
  2. result-error buffer ownership: return the firmware-owned OUTPUT buffer from the vb2 queue directly, and keep the CAPTURE buffer on the m2m ready queue. A pure `VLC_BUF_FULL` consumes the input frame with no valid access unit to hand on, and GStreamer reads a zero-sized M2M CAPTURE buffer as end-of-stream.
  3. coded `sizeimage` clamp, mirroring `0270`. The ceiling is the uncompressed frame (`w * h * 3 / 2`), which bounds the worst case from geometry alone; a fixed 1 MiB follows from an average frame and an average does not bound the frame after a total content change.
  4. OUTPUT-plane `data_offset` is honoured. Upstream programs the bare `vb2_dma_contig_plane_dma_addr`, so a DMABUF importer sharing one allocation across planes has every plane fetch from offset 0 and the encoder reads luma as chroma. MMAP buffers carry 0, so the stock path is unchanged.
  5. source stride wider than the picture. Upstream recomputes `bytesperline` from the width and discards the caller's; the CVISP capture node writes 1920 pixels on a 2048-byte stride. The hardware already takes source stride as its own field.
  6. coded height taken from the requested picture size, not the source height after source alignment. The two aligners differ (`W5_ENC_RAW_STEP_HEIGHT` 16, `W5_ENC_CODEC_STEP_HEIGHT` 8), so 1920x1080 coded 1088 rows whose last eight are allocation tail, shown by every decoder as a strip along the bottom edge.
  7. live rate control: `V4L2_CID_MPEG_VIDEO_BITRATE`, `V4L2_CID_MPEG_VIDEO_VBV_SIZE` and `VIDIOC_S_PARM` mark the change pending instead of only storing it. Upstream acts on them at the next seq init, so a change on a running instance is silently ignored.
  8. `V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME`, which upstream does not implement here. The stream carries one IDR at session start, so a receiver that joins later cannot decode.
  9. vendor parity for six open parameters that have no V4L2 control: `hvs_max_delta_qp` 10 -> 4, `rdo_skip` 1 -> 0, `lambda_scaling_enable` 1 -> 0, `rc_weight_buf` 128 -> 1, `tmvp_enable` 0 -> 1, `max_num_merge` 0 -> 2. Recovered field by field in `userspace/docs/air-video-benchmark.md`. `hvs_max_delta_qp` is the one that bounds an intra picture.

#### Changes that span patches

The **close-path contract** is `0210` + `0240` + `0250`: a close that cannot quiesce the firmware instance (siblings live, or the last-instance safety reset failed) marks it `keep_dma_bufs`, and `wave5_cleanup_instance` then leaks the FBC/DPB framebuffers and bitstream ring rather than freeing memory the firmware still DMAs into. Freeing it hard-hangs the SoC.

**Live encoder parameter changes** span four patches, one per file, rather than sitting in a patch of their own: `0220` implements `wave5_vpu_enc_change_param()` (`W5_ENC_SET_PARAM` with `OPT_CHANGE_PARAM`), `0255` declares it, `0240` marks changes pending and applies them from `start_encode` because the firmware only accepts a change between pictures, `0250` carries the pending mask and the forced-keyframe state, `0280` wires the V4L2 controls. Bitrate, VBV, frame rate and forced keyframe all ride the one firmware command. Two constraints found on hardware:

- the firmware leaves one report on the instance report queue for that command, and it must be drained with a `GET_RESULT` query. Left there it sits ahead of every later ENC_PIC report, `finish_encode` consumes it instead of its own frame, and the instance stalls after the first change.
- `W5_CMD_ENC_PIC_PIC_PARAM` does not force a keyframe on this firmware. Every plausible force-picture-type layout was swept and produced no IDR; the working route is intra parameters with `intra_period` 1 for a single picture.

HW-validated: exact bitrate at five steps, and a forced IDR that recovers a receiver which joined mid-stream.

### core kernel

- `0300-dma-coherent-page-granular.patch` - the per-device dma-coherent pool allocator (`bitmap_find_free_region`) rounds every allocation up to a power-of-2 page order and places it order-aligned, so an allocation can occupy up to ~2x its size (4.8 MiB costs 8 MiB) and the pool returns `ENOMEM` well below nominal capacity. Replaced with page-granular first-fit (`bitmap_find_next_zero_area`) plus a per-allocation page-count map, so release frees the exact count instead of relying on the caller's `get_order(size)`. wave5 and `ml_mmzheap` share the single rmem coherent pool on this system. Also adds a read-only debugfs view per rmem-backed pool (`/sys/kernel/debug/dma_coherent/<rmem-name>`): base, pages used/free, largest free run, live allocation list; the shared pool was otherwise a black box and every fragmentation question a live experiment.

  Caveat: this deliberately drops the DMA-API natural-alignment guarantee for **every** per-device coherent pool, because order-aligned placement is where the waste comes from and neither tenant masks address bits. A future pool user with address-bit-masked descriptor rings must not rely on `dma_alloc_coherent` alignment here.

### usb

- `0400-dwc2-gadget-buffer-dma.patch` - `params.c` auto-enables gadget descriptor DMA whenever the core advertises it (`p->g_dma_desc = hw->dma_desc_enable`), which the Proxima core does. Under a sustained bulk OUT transfer concurrent with video decode the DDMA OUT ring wedges, `dwc2_hsotg_ep_stop_xfr` times out on `GOUTNAKEFF`/`EPDisable`, the gadget drops and re-enumerates, and the recovery busy-waits starve the pipeline so the picture freezes. Forces `g_dma_desc = false` (buffer DMA, `g_dma` stays on) to match the vendor's Linux-4.9 BSP, which predates gadget DDMA and ran buffer DMA on this same core/DT/FIFO with no failure. `dwc2_check_params()`'s `CHECK_BOOL` only forces the value false when it is already true, so a false value passes validation untouched.

### dmaengine

- `0500-dw-axi-dmac-h.patch`, `0510-dw-axi-dmac-platform.patch` - serialize the `DMAC_CFG` read-modify-write behind a new `chip->cfg_lock` (`0500` adds the lock, `0510` takes it in the four helpers; none calls another, so there is nothing to nest). That one controller-wide register holds `DMAC_EN` and `INT_EN`, and mainline read-modify-writes it from `axi_chan_block_xfer_start()` and from `dw_axi_dma_interrupt()` with nothing serializing the two: `chan->vc.lock` is per channel and the handler does not hold it. On SMP the handler's `INT_EN` restore is lost, and since only that exit path ever sets `INT_EN` the loss is self-latching. The controller keeps completing transfers and per-channel `INTSTATUS` keeps being set, but no interrupt reaches the GIC again, so every dmaengine client blocks to its timeout, controller-wide, with no driver message.

  Seen twice on the goggle after ~200k interrupts of a 60 fps compose blit: `DMAC_CFG 0x00000001`, `DMAC_INTSTATUS 0x00000003`, IRQ counts frozen, dmesg clean. Writing `INT_EN` back by hand restored the engine on the spot, 4091.6 ms per frame back to 16.6 ms with no restart. `test_tools/dmablit_race_test.c` reproduces the loss on demand.

## Regenerating / editing a patch

Patches are the source of truth; there is no checked-in copy of the patched file. To change one:

1. Extract the pinned pristine file: `tar -xf <linux.tar.xz> linux-6.18.36/<path>`.
2. `patch -p1 < 0NNN-...patch` to reach the current downstream state, edit, then `diff -u --label a/<path> --label b/<path> <pristine> <edited> > 0NNN-...patch`.

Adding a change to a file that already has a patch means editing that patch, never adding a second one against the same file.

## On a kernel bump

Bump `pin.env`, rebuild. If a patch fails to apply, upstream changed that file: inspect the reject, re-base the hunk against the new pristine source, regenerate. Do not paper over it by reverting to a full-file copy.
