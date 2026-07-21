# Hardware bring-up & validation checklist (open MPP/RF modules)

How to validate the open kernel modules on the goggle once hardware is back. They
load on the **open 6.18 kernel** (booted via the OTRA/`bootm` RAM-test flow,
see `../README.md`), so this is **zero-risk**: nothing is flashed, a power-cycle
reverts to the stock slot. SSH/USB stay up independently (start_ssh.sh), so even a
module that oopses doesn't lock you out.

Work the modules in **dependency order** (the same as `load.sh`); validate each
before loading the next. Each module exposes an oracle (`/proc`, `/dev`, dmesg).

## Pre-flight
1. Boot the open kernel to a shell (the RAM-boot flow). Confirm: `uname -r` -> `6.18.36`.
2. **Build modules against the EXACT kernel image on the device, and ALWAYS capture the
   serial console during any insmod/risky op.** The config-mismatch panic mechanism, the
   build-matching procedure (`/proc/config.gz` + kallsyms fingerprint), and the
   serial-capture recipe are owned by `KERNEL-REQUIREMENTS.md` (§1, §5). Note
   `trim.config` no longer disables FB/MMC (`modules.config`, appended automatically by
   the build, carries what the modules need). Then copy fresh `.ko` to e.g. `/run/ml`.
3. `dmesg -n 8` (see all module prints). Keep both the serial AND the SSH log.

## Phase 1 - ar_osal (the keystone; everything allocates here)
```
insmod ar_osal.ko mmz=sram,0,0x00100000,0x00100000 mmz_allocator=hisi anony=1
```
The anonymous zone derives from the DTB's reserved-memory mmz node; `mmz=` only adds the on-chip sram zone (an explicit `mmz=anonymous,...` tuple overrides the DTB for bench work).
Check:
- `dmesg` shows `MMZ new zone <anonymous>` with the DTS carveout geometry and `<sram>`. **No `memremap ... failed`** - if it failed, the no-map reserved region isn't mappable WC; that's the #1 thing to confirm on HW (the whole design assumes `memremap(MEMREMAP_WC)` works on the no-map carveout).
- `ls -l /dev/mmz_userdev` exists (misc, major 10).
- `cat /proc/media-mem` lists both zones, used=0.
- Smoke test alloc/mmap from userspace (write a tiny C tool, or reuse `libhal_sys`): open `/dev/mmz_userdev`, `IOC_MMB_ALLOC` a 1 MiB block -> returns a phys inside the mmz reserved-memory carveout; `mmap(offset=phys)` -> non-NULL; write+read back a pattern (WC: reads must see writes after a barrier); `IOC_MMB_FREE`. Re-check `/proc/media-mem` returns to used=0 (no leak).
- **Cache-coherency reality check** (the documented risk): allocate a block, have an engine (later phases) DMA into it, confirm the CPU sees correct data via the WC mapping. If frames are corrupt, the WC assumption is wrong and we need the cached path (kernel patch exporting `arch_sync_dma_for_*`).

## Phase 2 - ar_vb / ar_sys / ar_sysctl
```
insmod ar_vb.ko ; insmod ar_sys.ko ; insmod ar_sysctl.ko
```
- `/dev/ar_vb`, `/dev/ar_sys`, `/dev/ar_sysctl` exist; `lsmod` shows `ar_vb`/`ar_sys` depend on `ar_osal`.
- ar_vb: `cat /dev/ar_vb` dumps the pool table (empty). Run libhal_vb (or a tool) to CRTPL a pool -> GETBLK -> RLSBLK -> DESTPL; `/proc/media-mem` shows the pool's mmb appear/disappear.
- ar_sys PTS: ioctl `IOC_SYS_INIT_PTS_BASE`, then `IOC_SYS_GET_CUR_PTS` twice - second ≥ first (monotonic, µs). `IOC_SYS_GET_PTS_OFFSET` sane.
- ar_sysctl: register/query_status round-trip (status 0=RUNNING).

## Phase 3 - ar_mpp_drv / ar_mpp_proc_ctrl (IRQ forwarder + proc shuttle)
```
insmod ar_mpp_overlay.ko ; insmod ar_mpp_drv.ko ; insmod ar_mpp_proc_ctrl.ko
```
`ar_mpp_overlay` must load first: it's a runtime DT overlay that injects the
`ahb_dma`/`axi_dma` child nodes under `ar_mpp` that `ar_mpp_drv`'s IRQ-count probe (and
closed `libmpp_service`'s `SYS_Init`) expects - a reflash-free stopgap ahead of a real
DT fix (see `../ROADMAP.md`).
- `/dev/ar_mpp_ctl`, `/dev/ar_mpp_proc_ctl` exist; dmesg shows `engine irq[N]: hwirq 100/101/111 -> virq …` (the DT table resolved). If hwirqs are wrong, fix the `ar_mpp@8870000` interrupts in the DTS.
- IRQ smoke test: REGISTER hwirq 100, then trigger an h26x completion (needs the codec userspace running) and confirm WAIT_EVENT returns `{hwirq=100, ktime}`. Watch `/proc/interrupts` line 100 increment.
- proc shuttle: `ls /proc/umap/` after the MPP service CREATEs entries.

## Phase 4 - ar_scaler - VALIDATED
> **Why `regbase` maps non-exclusively.** The scaler's MMIO (`reg = 0x08840000`) sits
> **inside the VO display controller's register block** - `/proc/iomem`:
> `08810000-0884ffff : 8810000.vo vo@8810000`. The `vo` DRM driver claims that whole window
> exclusively, so an exclusive `request_mem_region` here fails `-EBUSY` ("can't request region
> for [mem 0x08840000-0x08840fff]" -> probe `-16`). The scaler is a functional sub-block *of*
> the VO subsystem (which is also why they share the CGU), so `ar_scaler_probe` maps `regbase`
> with `devm_ioremap` (non-exclusive, like the `control`/CGU window) to coexist with `vo`
> instead of claiming the region.
>
> **SAFETY: do the first probe after any change with the display feed STOPPED.** `ar_scaler`'s
> `control` window (`0x0A100000`) is the **shared CGU** (`cgu@0x0a100000`, `artosyn,ar9311-cgu`)
> that also generates the display/VO pixel clocks. The probe's clock bring-up does RMW pokes
> into it (gate `+0x6010`, unlock magic `+0x6200`, config `+0x6204`, divider `+0x405c`), so a
> mis-recovered value could glitch the display clock. Stop the video first
> (`rc-service ml-video stop`), then load + test; worst case is a panel glitch fixed by reboot
> (nothing is flashed). The scaler is NOT in the open decode->display path (zero-copy DRM), so
> validating it needs no feed.
>
> `ar_scaler` loads **standalone** (no ar_osal): its internal LUT/batch buffers are `dma_alloc_coherent` on its own device. Only `scalertest` (which allocates its src/dst from MMZ via `/dev/mmz_userdev`) still needs ar_osal; `scaler_dmabuf_test` (CMA dma-heap) does not.
```
insmod ar_scaler.ko
```
- `/dev/arscaler` exists; dmesg shows `ar_scaler 8840000.scaler: ready: regs 0x08840000, control 0x0a100000, irq 28`. `/proc/interrupts` has `107 ... arscaler`.
- **`cat /proc/arscaler/state`** = the verification oracle. Run `../test_tools/scalertest`
  (no closed `libhal_scaler` needed): it allocates MMZ src/dst, paints a pattern and runs a
  crop/resize ladder. **A `-ETIMEDOUT` means the clock-init sequence or register packing is
  wrong** (the IRQ never fires); on failure scalertest dumps `/proc/arscaler/state`.
- Pixel correctness: scalertest's T1/T2 identity/crop ops are **bit-exact** (ratio 1.0 = pure
  crop/copy); the T3 2:1 downscale is checked against a software box reference with tolerance.
  **Result: T1/T2 bit-exact, T3 within tolerance, IRQ 107 fires** - the recovered clock
  bring-up, phys/stride/crop packing, Q16 ratio math, LUT, and completion path are all correct.

## Phase 5 - ar_framebuffer
```
insmod ar_framebuffer.ko width=1920 height=1080 format=4
```
- `/dev/fb0` + `/sys/class/graphics/fb0/*` appear. `cat /sys/class/graphics/fb0/{virtual_size,stride,bits_per_pixel}` -> 1920,3240 / 4096 / 16.
- The repo's own consumer works: `missinglynk screenshot` (reads fb0) and the Qt menu (`QT_QPA_PLATFORM=linuxfb:fb=/dev/fb0`). Drawing into the mmap'd buffer should land in MMZ.
- **Scanout**: the OSD only reaches the panel once the `ar_overlay` QBuf handshake is wired (currently shaped but the kernel->CUSE transport is a stub). Until then, fb0 is allocatable/mappable but may not composite. Validate the buffer contents via screenshot first; scanout integration is a follow-up.

## Phase 6 - AR8030 RF link (SDIO enum, firmware load, and RF association all DONE)

The AR8030 enumerates as a full SDIO card on the open 6.18 kernel (slot B), alongside the
microSD, via the **open** stack (`ar_dtbo_sdio` DT overlay + `dw_mci-artosyn` glue +
`artosyn_gpio`), no vendor `.ko`. Root cause + full proof of the original SDIO-silence
blocker (a gpio register off-by-one, `artosyn_gpio.c` `AR_BANK_REG_BASE` 0xC0 -> 0xBC).

> `ar_dtbo_sdio.ko` is produced by the tracked build (`ar_dtbo_sdio.c` + `.dts` +
> generated `_dtbo.h` in `Kbuild`, mirroring `ar_mpp_overlay`); the embedded blob is
> byte-identical to the hand-built one hardware-validated during bring-up (source of
> both: the bring-up overlay source `mmc-sdio-overlay.dts`). The
> overlay remains the reflash-free
> delivery path; the permanent home for the `mmc@1b00000`/`mmc@1c00000` nodes is
> `devices/betafpv-vr04-goggle/proxima-9311.dts`, to be batched with the pending codec/watchdog DT work.

### Reproducible bring-up recipe (open 6.18 kernel; modules in `/run/ml`; `gpio_pulse` from `../test_tools/`)
```
insmod ar_dtbo_sdio.ko          # DT overlay: mmc@1b00000 (AR8030) + mmc@1c00000 (SD) + gpio@a10a000
insmod artosyn_gpio.ko          # binds the gpio node (with the 0xBC fix)
/run/ml/gpio_pulse ar-gpio1 0   # release AR8030 reset = gpio23 (bank1 line0), low->high, held high
                                # verify: reg_peek 0x0A10A0D0 (bank1 DAT) bit0 == 1 (pad high = reset released)
insmod dw_mci-artosyn.ko clk_sel=135 clk_cfg=2   # open dw_mci glue; the validated working pair SEL=0x87 phase=0x02 (originally from misread slot-A dumps; ../docs/sd-card.md)
```
Result: `mmc: new high speed SDIO card at address 0001` -> `/sys/bus/sdio/devices/mmcN:0001:1`.
Verify: `ls /sys/bus/sdio/devices/` shows `mmcN:0001:1`; `cat .../device` == `0x8030` (ROM mode).
The microSD enumerates too (`mmcM:aaaa` / `mmcblkM`).

### Firmware load - DONE (open uploader)
The open `artosyn_sdio` driver uploads the "chagall" firmware into the chip's ROM loader and the
chip runs it - no vendor `.ko`. Recipe (the ubifs rootfs `/lib/firmware` is FULL, so stage the
firmware on tmpfs and point the firmware search path at it):
```
mkdir -p /run/ml/fw
# Stage the firmware image AND the MERGED bound config (NOT the placeholder bb_config_gnd.json).
# The merged file bb_config_gnd.json.usr_cfg.json carries the real bound air unit + power cal +
# channel bitmap; it is auto_merge'd at boot on slot A and can only be fetched from a running A
# (the vendor-blob fetch script pulls it to the firmware stash). The raw bb_config_gnd.json holds
# placeholder MACs (ap.mac=66000000, candidate=aabbccdd) and will NOT properly associate.
# push firmware/bin/slot-a/usr/usrdata/ar813x/bb_demo_gnd_d.img
#      firmware/bin/slot-a/.../bb_config_gnd.json.usr_cfg.json  -> /run/ml/fw/
echo /run/ml/fw > /sys/module/firmware_class/parameters/path
# (AR8030 already enumerated as 0x8030 per the recipe above) then (matches modules/load.sh):
insmod artosyn_sdio.ko fw_name=bb_demo_gnd_d.img cfg_name=bb_config_gnd.json.usr_cfg.json
```
Validation signals: board current draw jumps +130mA (350 -> 480mA); SDIO device-ID flips
`0x8030` (ROM) -> `0x8031` (firmware running), `cat /sys/bus/sdio/devices/*/device`; `sdio0` net
interface appears (`<BROADCAST,NOARP> mtu 4096`).

### RF link association - DONE
`ip link set sdio0 up` + assign `10.0.0.1`, then a paired air unit transmitting associates the
link end to end (enum -> firmware+config upload -> SDIO -> TX credit -> RF associate -> recv
parse 0xCC IP frames -> `netif_rx` -> `sdio0`). Requires an air-unit power-cycle after every
slot-B reboot to re-associate.

The full-rate video downlink on top of this is DONE (poll cadence + TX power + the UDP `:10000` params handshake; `../STATUS.md` "RF chip", canonical record `../../docs/reference/rf-video-downlink.md`). Phases 1-5 above cover the reference MPP modules (see the pivot note in `VERIFICATION.md`); only this phase is production.

## If something breaks
- A module oops doesn't kill SSH (dropbear is independent). `rmmod` it, fix, retry.
- Ultimate fallback: power-cycle -> stock slot (nothing was flashed).
- Per-module recovery + the UART console: `../../docs/guides/serial-and-debug-access.md`.
