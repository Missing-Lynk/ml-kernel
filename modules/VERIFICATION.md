# Verifying the open MPP/RF kernel modules

> **PIVOT NOTE.** The project pivoted during development: instead of running the
> closed vendor media userspace on these reimplemented modules, the open V4L2/DRM/GStreamer
> stack replaced that userspace outright (`../STATUS.md` "Open codec"). The MPP-stack
> reimplementations are therefore **reference only** - kept in-tree, compile-checked, but not
> shipped in the rootfs and not loaded at boot - and the conformance program below is
> **parked, not active**. It remains the plan of record if these modules are ever integrated
> properly in the future. `artosyn_sdio` is NOT parked: it is production and hardware-validated
> end to end (`../STATUS.md` "RF chip").


How we prove the reverse-engineered open modules (`modules/`) are **correct**, not just that they load. Companion to `HW-BRINGUP.md` (which is *how to load*); this doc is *how to know they're right*.

Scope: the 9 open modules reimplementing the vendor MPP/RF `.ko` (full list + per-module detail: `README.md`).

---

## 0. Guiding principle - trust the vendor's artifacts, not our code

These modules exist to satisfy a **closed ABI**, so the strongest verification is
**differential conformance against artifacts we did not write and cannot fudge**.
Byte-exact kernel ABIs are verified with small, purpose-built test tools/harnesses
(`test_tools/`), never by running the vendor's full closed
userspace stack (`ar_lowdelay`, `mpp_service.app`, the CUSE daemons, binder, ~85 shared
libs) on the open kernel. That vendor userspace is retired (replaced by the open
V4L2/DRM/GStreamer stack) and this conformance program is parked; see the pivot note
above.

Two harness shapes cover this:
1. **Self-contained, byte-exact reimplementation** - the harness speaks the recovered
   ABI directly against our `/dev` node, no vendor code at all (`mmztest`, `scalertest`,
   `led_test`, `button_test`, `buzzer_test`, `display_test`/`overlay_test`). Proves our
   module accepts the exact ioctl/struct shape and behaves.
2. **A single vendor `.so` linked into a minimal harness**, only where the vendor's
   closed IP itself is under test (e.g. the codec's per-frame register choreography) -
   the `ml-codec-probe` harness links `libmpi_venc`/`libmpi_vdec` directly (with an
   `LD_PRELOAD` shim to skip unrelated `SYS_Init` subsystems), never the full
   `ar_lowdelay`/`mpp_service.app`/CUSE/binder stack. This is still "the vendor's
   artifact proves us right", just scoped to the one library that matters instead of
   the whole closed runtime.
3. **The stock slot** (vendor 4.9 kernel + vendor `.ko` + the vendor's own unmodified
   userspace), kept bootable as slot A - observed as a **golden reference for A/B
   differential testing**, never run on the open kernel itself.

"It loaded without panicking" proves almost nothing about ABI correctness. A wrong
struct-field offset or ioctl size happily loads and then silently corrupts. The tiers
below escalate from "loads" to "a harness built against the real ABI can't tell the
difference."

---

## 1. Method & safety (applies to every test)

- **Dynamic only.** `insmod` from `/run/ml` (tmpfs); never persist (no `modules-load.d`,
  no boot script, nothing in the rootfs). A reboot must always come up clean. Reboots
  auto-recover and the network self-heals; the goggle only fully dies on dead battery.
- **Serial capture + match-built modules, every session.** The serial-capture recipe,
  the config-mismatch panic mechanism, and the build-matching procedure are owned by
  `KERNEL-REQUIREMENTS.md`.
- **A/B is non-destructive.** Open modules run on the open slot; stock slot A stays
  pristine for reference. Nothing here writes flash.

---

## 2. Prerequisites (gate the tiers)

| prereq | needed for | status |
|---|---|---|
| Match-built modules (config.gz + fingerprint) | everything | do first, each session |
| `mmztest` (cross-built static aarch64) | Tier 0/1 ar_osal/ar_sys | `../test_tools/mmztest.c`, ready |
| `scalertest` (cross-built static aarch64) | Tier 0/1 ar_scaler | `../test_tools/scalertest.c`, ready (completion-only; not pixel-correctness) |
| `ml-codec-probe` + the `libmpi_venc`/`libmpi_vdec` `.so` (copied from a stock dump) | Tier 1 ar_mpp_drv (codec path) | the `ml-codec-probe` harness builds + link-validates; not yet run on hardware (needs stock firmware with `ar_lowdelay` stopped) |
| `led_test` / `button_test` / `buzzer_test` / `display_test` / `overlay_test` | Tier 0/1 peripheral drivers outside this doc's 9-module scope, listed for completeness | `../test_tools/`, ready |
| A harness for ar_vb / ar_sysctl / ar_mpp_proc_ctrl | Tier 1 those modules | **not yet built** - none of these have a component harness yet (self-contained or `.so`-linked); see Tier 1 table |
| AR8030 SDIO host node + driver + reset | artosyn_sdio enumerate | **DONE** - `ar_dtbo_sdio` + `dw_mci-artosyn` + `artosyn_gpio` (gpio 0xBC fix) enumerate the chip (device 0x8030, ROM mode); see `HW-BRINGUP.md` Phase 6. NOTE: `ar_dtbo_sdio` is not yet in `Kbuild`/has no tracked `.c` source - see the gap note in `KERNEL-REQUIREMENTS.md` §4. |
| Air unit transmitting | Tier 3 RF | on demand |

Missing per-module harnesses are the gating item now, not a monolithic closed-userspace
milestone: Tier 0 is doable with just the modules; Tier 1 is doable module-by-module as
each harness gets built, with no dependency on the vendor's full runtime.

---

## 3. Tier 0 - Loads & introspects (modules alone, no closed userspace)

Proves each module initialises, creates its node, and its internal state is sane. Per
module: `insmod` (serial up) -> no panic -> `/dev` node present -> `/proc` oracle sane.

| module | /dev | /proc or sysfs oracle | Tier-0 pass criterion |
|---|---|---|---|
| ar_osal | `/dev/mmz_userdev` | `/proc/media-mem` | zone(s) listed at `0x29400000`, used=0; **no `memremap … failed` in dmesg** (the open question - WC map of the nomap carveout); `mmztest` MMZ test passes (alloc in-zone, mmap, 1 MiB write/read-back matches, free, no leak) |
| ar_sys | `/dev/ar_sys` | - | `mmztest` PTS: `GET_CUR_PTS` monotonic & µs-scale; tz set/get round-trips; GPS blob set/get round-trips |
| ar_sysctl | `/dev/ar_sysctl` | - | register->query_status round-trips; status transitions RUNNING<->SUSPEND |
| ar_vb | `/dev/ar_vb` | `cat /dev/ar_vb` (pool table) | a self-test or hand-driven CRTPL->GETBLK->RLSBLK->DESTPL leaves the pool table + `/proc/media-mem` consistent (no leaked mmb) |
| ar_mpp_drv | `/dev/ar_mpp_ctl` | `/proc/interrupts` | requires `ar_mpp_overlay` loaded first (injects the `ahb_dma`/`axi_dma` IRQ-count children it probes for); probe maps hwirq table (dmesg `engine irq[N]: hwirq …`); REGISTER hwirq 100 then a real engine IRQ wakes WAIT_EVENT with `{hwirq,ktime}`; `/proc/interrupts` line increments |
| ar_mpp_proc_ctrl | `/dev/ar_mpp_proc_ctl` | `/proc/umap/` | CREATE makes a `/proc/umap/<name>`; write to it -> MSG ioctl delivers it; CLOSE removes it |
| ar_scaler | `/dev/arscaler` | `/proc/arscaler/state` | probe OK (core@0x08840000, ctrl@0x0A100000, irq 107 in `/proc/interrupts`); a single CropResize **completes** (no `-ETIMEDOUT`) = clock seq + IRQ correct |
| ar_framebuffer | `/dev/fb0` | `/sys/class/graphics/fb0/*` | `virtual_size`=1920,3240; `stride`=4096; `bits_per_pixel`=16; mmap + write lands in MMZ; pan updates yoffset |
| artosyn_sdio | `/dev/artosyn_sdio`, `sdio0` | `dmesg`, `ip link` | (needs mmc1 host) probe pokes + fw upload succeed; `sdio0` appears; idle clean. Without mmc1: driver registers, binds nothing - must not crash |

Tier-0 failures to watch for specifically:
- `ar_osal`: `memremap(MEMREMAP_WC)` returning NULL on the nomap region -> kernel maps
  unavailable (userspace mmap still works). This is THE unresolved on-HW question.
- `ar_scaler`: CropResize `-ETIMEDOUT` => the recovered clock-init sequence or register
  packing is wrong (IRQ never fires). `/proc/arscaler/state` is the debug oracle.

---

## 4. Tier 1 - Conformance: purpose-built harnesses vs open modules

The real ABI proof, but scoped narrowly: a **small harness per module**, not the
vendor's full closed runtime. Where the harness is self-contained (speaks the recovered
ABI directly, no vendor code), correctness comes from the ABI being recovered right in
the first place - byte-pinned from the `.ko`/`.so` disassembly. Where a harness links a
single vendor `.so` (only for genuinely closed IP, e.g. the codec's register
choreography), that library encodes the correct struct layouts / ioctl numbers / return
semantics, so a clean run against our module proves byte-for-byte conformance for that
path - without pulling in `ar_lowdelay`, `mpp_service.app`, CUSE, or binder.

| open module | exercised by (harness) | what it proves | pass criterion |
|---|---|---|---|
| ar_osal | `mmztest` (self-contained: `../modules/ar_uapi.h`) | `struct mmb_info` (136B) layout, the `'m'` alloc/free/remap/virt2phys ABI, mmap offset==phys | alloc returns a usable phys the tool maps & uses; virt2phys round-trips; free leaves `/proc/media-mem` clean |
| ar_vb | **no harness yet** - TODO, self-contained (mirror `../modules/ar_vb.c`'s structs the way `scalertest` mirrors `ar_scaler.c`) | the `'b'` table nrs, the `(pool_id<<16)\|idx` handle encoding, per-cmd field offsets | CRTPL/GETBLK/RLSBLK/HL2PA/PA2HL/HL2PID all return what's expected (handle decodes, phys resolves) |
| ar_sys | `mmztest` (self-contained, PTS section) | `'y'`/`'p'` (PTS/flush/tz/gps) | `GET_CUR_PTS` monotonic & µs-scale; tz/gps set/get round-trip |
| ar_sysctl | **no harness yet** - TODO, self-contained (pure-SW ioctl register/query, no HW to fake) | the 7-ioctl `'S'` registry + 40B msg | register/unregister/suspend/resume behave |
| ar_scaler | `scalertest` (self-contained, completion-only today) | `'Z'` 64B descriptor + the clock/IRQ path | `AR_MPI_SCALER_CropResize`-equivalent completes (no `-ETIMEDOUT`); **pixel-correctness** needs either extending `scalertest` with a known test image, or linking `libhal_scaler`/`libmpi_scaler` directly the way `ml-codec-probe` links the codec libs (compare output to stock, §5) |
| ar_mpp_drv | `ml-codec-probe` (links `libmpi_venc`/`libmpi_vdec` directly, sole MPP owner - `ar_lowdelay` must be stopped first) | the `'M'` IRQ-forward + WAIT/ENABLE re-arm protocol, exercised via real codec CreateChn/SendStream/GetFrame calls | the decoder's IRQ-wait loop advances frames (no stalls/timeouts) |
| ar_mpp_proc_ctrl | **no harness yet** - TODO, self-contained (CREATE/write/MSG/CLOSE against `/dev/ar_mpp_proc_ctl` directly) | the umap CREATE/MSG/WRITE/CLOSE 144/280/24/8B structs | `/proc/umap/<name>` appears, round-trips a write, and CLOSE removes it |
| ar_framebuffer | `display_test`/`display_bounce`/`overlay_test`/`display_demo` (self-contained, real DRM/KMS ABI) | the fbdev geometry/mmap/pan ABI, and (for scanout) the DRM plane path | the tools render + composite through the real primary + overlay planes; the still-stubbed piece is the *legacy* kernel->CUSE `ar_overlay` transport specifically, not scanout in general (see `../docs/display-backlight.md`) |
| artosyn_sdio | the open `artosyn_sdio` driver itself + a real paired air unit (no vendor code at all) | the `'v'` ioctl, netdev, fw uploader, the on-wire `video_packet` format | `sdio0` associates with the air unit; RX frames pass CRC and reach the stack (hardware-validated) |

**Quick conformance smoke (per device, before building a full harness):** `strace` a
single vendor call against the stock slot (§7) and confirm the ioctl number + arg size
match what our module expects. Cheap early signal before writing the harness.

---

## 5. Tier 2 - A/B differential vs the stock slot (golden reference)

Stock slot A = vendor kernel + vendor `.ko` + the vendor's own unmodified userspace =
ground truth, observed only, never run on the open kernel. Capture golden traces there
(can be done any time the goggle is on stock - see §7), then run the equivalent workload
on the open slot **through the Tier 1 harness** and diff.

Compare, for the same workload:
- **ioctl stream:** `strace -f -e ioctl` of `ar_lowdelay` on stock (and the daemons) vs.
  the Tier 1 harness's own ioctl stream on the open slot - same cmd numbers, same arg
  sizes, same return values, same order. Divergence = ABI mismatch in the named device.
- **MMZ state:** `/proc/media-mem` - same zones, comparable allocation pattern.
- **scaler state:** `/proc/arscaler/state` - src/dst/crop/ctrl fields match for the same
  CropResize.
- **interrupts:** `/proc/interrupts` - the engine IRQs (100/101/107/111) tick on both.
- **output frames:** checksum decoded/scaled frames - bit-identical (or visually identical
  where timing differs) to stock for the same input.

A divergence points straight at the offending module + ioctl. This is the most
discriminating test short of full end-to-end.

---

## 6. Tier 3 - End-to-end functional parity

Our own minimal pipeline, composed from the Tier-1-validated components, not the
vendor's app: **RF link associates -> the codec harness decodes the downlink -> `ar_scaler`
-> `ar_framebuffer`/DRM composites to the panel -> our own RTSP mux.** Run the same
flying/bench session on the open slot and compare against the stock slot's behaviour
(§7); functional parity = done. Then the usual benchmarks (boot time, `free`,
`cyclictest`, `iperf3` over the gadget, camera-to-display latency).

---

## 7. Golden-trace capture (do this on the stock slot, any time)

Grab the reference behaviour while booted on stock slot A. Suggested capture set, pulled
to the host:

```sh
# on the goggle (stock slot A), with the FPV app running (ar_lowdelay -m 2 -t 0):
strace -ttt -f -e trace=ioctl,mmap,openat -p $(pidof ar_lowdelay) 2>/tmp/golden-strace.txt &
sleep 20; kill %1
cat /proc/media-mem        > /tmp/golden-media-mem.txt
cat /proc/arscaler/state   > /tmp/golden-arscaler.txt   2>/dev/null
cat /proc/interrupts       > /tmp/golden-interrupts.txt
cat /proc/umap/vctrl/h26x  > /tmp/golden-umap-h26x.txt  2>/dev/null
# pull them to the host
```

These become the diff target for Tier 2. (strace may need installing on stock, or use
the vendor's available tools / `/proc` snapshots if strace is absent.)

---

## 8. Tracker

Per module, in order. Mark as validated on hardware (`[x]`).

Tier 0 passed on hardware (all 9 modules load with their /dev nodes and oracles,
both named residuals resolved). The boxes below tick only what that run proves against each
per-box criterion; the rest (load-tested only, or criteria needing absent userspace) were
not back-filled.

```
PREREQ
[x] modules match-built to flashed kernel (config.gz + kallsyms fingerprint)
[ ] golden traces captured from stock slot A

TIER 0 (modules alone)
[x] ar_osal      load + /proc/media-mem + mmztest MMZ  (+ memremap-on-nomap OK)
[ ] ar_sys       mmztest PTS + tz/gps round-trip (PTS proven monotonic/µs-scale; tz/gps not recorded)
[ ] ar_sysctl    register/query/suspend/resume (load-tested only)
[ ] ar_vb        CRTPL->GETBLK->RLSBLK->DESTPL, no leak (load-tested only)
[ ] ar_mpp_drv   REGISTER + WAIT_EVENT on real IRQ (needs a real engine completion; not exercised)
[ ] ar_mpp_proc_ctrl  CREATE/write/MSG/CLOSE via /proc/umap (load-tested only)
[x] ar_scaler    CropResize completes (no -ETIMEDOUT)
[ ] ar_framebuffer  fb0 geometry + mmap + pan (geometry/stride proven; mmap+pan not recorded)
[x] AR8030 SDIO   enumerates on open kernel (device 0x8030 ROM mode; ar_dtbo_sdio + dw_mci-artosyn + artosyn_gpio 0xBC fix)
[x] AR8030 RF     firmware upload -> device flips 0x8031 + sdio0 up + link associates + full-rate video downlink (DONE, ../STATUS.md "RF chip")

PREREQ for Tier 1+ (per module, not a single monolithic milestone)
[ ] ar_osal / ar_sys harness      mmztest, ready
[ ] ar_scaler harness             scalertest, ready (completion-only)
[ ] ar_mpp_drv harness            ml-codec-probe, ready (not yet run on HW)
[ ] ar_vb harness                 not built
[ ] ar_sysctl harness             not built
[ ] ar_mpp_proc_ctrl harness      not built
[ ] ar_framebuffer harness        display_test/overlay_test/display_demo, ready

TIER 1 (harness vs open modules)       [ ] per module as above
TIER 2 (A/B differential vs stock)     [ ] ioctl/state/frame diffs clean
TIER 3 (end-to-end)                    [ ] RF->decode->display->DVR->RTSP parity, our own pipeline
```
