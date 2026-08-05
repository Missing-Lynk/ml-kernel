# Wave5 encoder: firmware state survives instance destroy

Creating a wave5 encoder instance after a previous one was destroyed, within one firmware boot, is unreliable on both units: measured first on the air unit, reproduced on the goggle. The defect is in the codec firmware, not the host driver or its callers. Facts are tagged **[confirmed]** (measured on hardware), **[inferred]** or **[open]**. Reproduction and tooling are at the end.

## Symptom [confirmed]

An instance created after a previous one was destroyed lands in one of three states:

- **clean**: bit-identical output to a healthy instance;
- **watchdog**: `WAVE5_SYSERR_WATCHDOG_TIMEOUT` (`0x20000`) kills the first PIC_RUN ~125 ms in. The firmware stays responsive (`GET_RESULT` answers), only the picture run dies, and the driver returns every buffer as `VB2_BUF_STATE_ERROR`;
- **garbage-encode**: the instance runs to completion but emits a corrupt bitstream ~200x the healthy size. The corruption is deterministic, so repeated garbage instances are byte-identical and the result decodes as dense noise instead of failing.

At 60 frames per instance the outcome alternates strictly good/bad, on both units. The first instance after firmware boot has been clean in every experiment; on the goggle the first *test* instance was clean even after ~18 hours of normal decoder/pipeline use in the same boot.

## What selects the outcome [open]

Hidden firmware state. On the air unit the previous instance's frame count matters non-monotonically (25-frame instances chain clean; 5, 35, 50 and 60-frame instances poison the next), the codec matters (h264-after-h264 self-sustains the garbage mode instead of watchdogging), and the state crosses geometry (a good 720p instance poisons a following 1080p one). The goggle measurements below agree that codec and geometry select the outcome but disagree on both details: frame count is irrelevant there (60, 600 and 1500 all alternate) and H.264 self-sustains a clean state rather than a garbage one. No host-visible model fits all of it, and further black-box probing was unproductive.

## Ruled out by measurement [confirmed]

- **The host driver's inputs**: register dumps at PIC_RUN entry, STREAMOFF and failure show `W5_VPU_VPU_INT_STS`, `W5_VPU_VINT_REASON`, `W5_RET_QUEUE_CMD_DONE_INST`, `W5_RET_SEQ_DONE_INSTANCE_INFO`, `W5_RET_QUEUE_STATUS` and `W5_VPU_BUSY_STATUS` all zero at every probe, on good and bad instances alike. No completion or queue state crosses the instance boundary.
- **Buffer placement**: every vdi allocation of every instance lands at byte-identical addresses and sizes, and the pool returns to the same fill level each time.
- **Runtime PM**: the alternation is unchanged with `power/control` pinned `on`. Sleep/wake saves and restores firmware state by design, so it preserves the poison faithfully.
- **STREAMOFF sequencing**: dropping the explicit STREAMOFF pair and closing the fd instead changes nothing.
- **A live sibling instance**: a second (480p, paced) instance streaming across the create/destroy cycle perturbs the pattern (back-to-back failures appear) but does not prevent corruption. Firmware instance count reaching zero is not the trigger.
- **The stored firmware**: `chagall.bin` is uploaded at probe and identical every boot, and the first instance of every boot is clean. The corrupt state is in the running VCPU's memory, not the blob.
- **Anything upstream of the encoder**: the garbage mode reproduces from a fixed synthetic pattern in dma-heap buffers, no capture hardware involved.

## Vendor comparison

- The vendor userspace close path (`VPU_EncClose` in `libmpp_service.so`) is semantically identical to the mainline driver's close: `ProductVpuEncFiniSeq` with a retry on STILL_BUSY, buffer frees, `FreeCodecInstance`. No reset, no sleep/wake, no extra commands. **[confirmed]**
- The vendor air unit creates one encoder instance at boot and never destroys it, so the vendor stack never exercises this path. The defect is present but unreachable there. **[inferred]**
- Mainline wave5 as of 6.18 has no fix in this area; post-6.18 patches address job-abort races and CMD_STOP frame drops only. **[confirmed]**

## The goggle reproduces it, but only in HEVC [confirmed]

Measured with `ml-cam2enc -e` on a goggle mid-boot (ml-video stopped, wave5 refcount 0). The goggle's flashed wave5 build predates the debug patches, so failing instances log nothing and `-g` packed geometry is required (it rejects the 2048 camera stride); judge by the STREAM line alone.

HEVC alternates strictly good/bad, exactly the air-unit pattern. Good instances are byte-identical to each other; bad ones code 1 frame and 0 bytes:

| Config | Result |
|---|---|
| HEVC 1920x1080, 60 / 600 / 1500 frames | alternates at every length; instance length does not select the outcome |
| HEVC 1920x1080, 6 s idle between instances | alternates; elapsed time does not clear the state |
| HEVC 1280x720 | alternates |
| HEVC 640x480 | **6 of 6 clean**, with and without a live decoder pipeline |
| H.264 1280x720 GOP 60 | **6 of 6 clean**, byte-identical, and the output decodes to correct video |
| H.264 1920x1080 GOP 60 | clean and self-sustaining after the first instance |

So two levers, both measured: **the codec** (H.264 chains clean where HEVC alternates) and **a geometry threshold** between 640x480 and 1280x720 in HEVC.

The H.264 result contradicts the air unit, where h264-after-h264 self-sustained the *garbage* mode. On the goggle it self-sustains a *clean* state, proven by decoding the sixth instance's stream rather than by its size. The two units run different wave5 builds, which is the obvious candidate and is untested.

**Rate control changes which failure you get.** With `-R 10000000` (CBR) at 1080p HEVC, the poisoned instances report 60 coded frames and emit 1411202 bytes, identically, run after run: the garbage mode wearing a healthy frame count. A bitrate-controlled caller therefore gets plausible-looking large corrupt files instead of obvious empty ones.

## Why the goggle's DVR works anyway [confirmed, with one gap]

`mlp-record` rebuilds its record bin, encoder element included, on every start/stop, so every recording is a fresh instance. Recordings nevertheless succeed, and the SD card explains why rather than contradicting the defect:

- The one long chain of consecutive recordings on the card (Video087-090, four in a row ~50 s apart, so one boot) is **H.264 720p**, verified by `ffprobe`, and all four decode clean. That is the configuration measured above as chaining clean.
- The newest recordings (Video099-101, 0.7-1.0 GB each) are **HEVC 1080p**, the configuration that alternates. Each is a single long recording, consistent with one per boot, where the first instance of a firmware boot is always clean.
- The card also carries plenty of 0-byte and 663-byte (header-only) files from earlier development, so silent recording failures are not hypothetical on this device.

The gap: driving two consecutive HEVC recordings through the real pipeline in one boot was attempted and is **not** measured. With the air unit off there is no RF video source, so all three attempts produced header-only files for want of input, which proves nothing about the encoder. Repeat it with a live air unit to close this.

The codec DT nodes are identical apart from the memory pool (air 32 MiB at `0x25000000`, goggle 110 MiB at `0x29200000`).

## Consequences for callers

- **One long-lived encoder instance per firmware boot**, which is also the vendor's usage pattern. Recording start/stop must gate the muxer/file branch, not the encoder.
- **The goggle DVR's HEVC default is the exposed case.** `mlp-record` records HEVC unless `ML_DVR_CODEC=h264`, and rebuilds the encoder element per recording. One recording per boot is safe (first instance always clean) and H.264 is measured safe; a second HEVC recording in the same boot is the untested risk. `ML_DVR_CODEC=h264` is the cheap mitigation, and it is also what the vendor DVR records.
- **Judge a recording by decoding it, not by the file existing.** Header-only 663-byte files and, under rate control, plausibly-sized corrupt ones are both failure shapes seen on this hardware.
- **A test harness that opens the encoder repeatedly is measuring this defect, not its subject.** A series of per-run instances produces ~50% corrupt output regardless of what is fed in. This holds on both units.
- The decoder has shown no equivalent behaviour under heavy instance churn.
- Clearing the state requires re-booting the VCPU from firmware, which today means a power cycle: a warm `rmmod`/`insmod` of wave5 fails (`vpu_init_with_bitcode: -16`) and costs the codec for the rest of the boot. An in-driver VPU reset plus firmware re-upload, without module teardown, is a plausible but unbuilt recovery path.

## Reproduction

No camera, no bring-up, ~30 s on the device. Air unit:

```
glue/camera/au-enc-repeat.sh          # RUNS=, FRAMES=, EXTRA= (extra ml-cam2enc flags)
```

Goggle (no harness; the AU scripts assert an air unit): stop `ml-video` first and verify `ml-pipeline` is gone and wave5's refcount is 0, push `native/build/ml-cam2enc` to `/tmp` (tmpfs, so the test leaves no file residue), and loop `/tmp/ml-cam2enc -e -g 1920x1080 -n 60`. The `-g` packed geometry is required on the flashed goggle build, which rejects the 2048 camera stride; the same build also logs nothing on a failed instance, so judge by the STREAM line alone.

Two traps in the goggle restore. Repeated `rc-service ml-video` cycles starve CMA and the pipeline comes back `crashed`; recover with the ordered restart (stop ml-video and ml-hud, `rc-service ml-display restart`, then start **ml-hud before ml-video** - started the other way round the pipeline takes the CMA and the HUD cannot allocate its DRM overlay). Running the tool while the pipeline is live fails at `DMA_HEAP_IOCTL_ALLOC` on 1080p because the live pools own the heap; 640x480 fits. The VCPU's instance state is the one thing no restore clears; only a power cycle does.

`ml-cam2enc -e` feeds the encoder a fixed synthetic pattern from dma-heap buffers and prints `STREAM: <coded> coded, <bytes> bytes, hash <hex>` (FNV-1a over every coded byte), so repeat runs of one build must hash identically. Healthy 60-frame HEVC 1080p, padded geometry on the AU: `16306 bytes, 06b372ee507e0199`; packed geometry on the goggle: `14774 bytes, 23cbb260ce603b47`. Watchdog (and the goggle's silent failure): `1 coded, 0 bytes`. Garbage: megabytes.
