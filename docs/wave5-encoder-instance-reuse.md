# Wave5 encoder: firmware state survives instance destroy

Creating a wave5 encoder instance after a previous one was destroyed, within one firmware boot, is unreliable on the air unit. The defect lives in the codec firmware, not in the host driver or its callers. Each fact below is tagged **[confirmed]** (measured on hardware), **[inferred]**, **[open]**. Reproduction and the measurement tooling are at the end.

## Symptom [confirmed]

An encoder instance created after a previous instance was destroyed lands in one of three states:

- **clean**: bit-identical output to a healthy instance;
- **watchdog**: the firmware's internal watchdog (`WAVE5_SYSERR_WATCHDOG_TIMEOUT`, `0x20000`) kills the instance's first PIC_RUN ~125 ms in. The firmware stays responsive (`GET_RESULT` answers); only the picture run dies. The driver returns every buffer as `VB2_BUF_STATE_ERROR`;
- **garbage-encode**: the instance runs to completion but emits a corrupt bitstream ~200x the healthy size. The corruption is deterministic: repeated garbage instances produce byte-identical output, so it decodes and plays as dense noise rather than failing.

At 60 frames per instance the outcome alternates strictly good/bad. The first instance after firmware boot has been clean in every experiment.

## What selects the outcome [open]

Hidden firmware state. The previous instance's frame count matters non-monotonically (25-frame instances chain clean; 5, 35, 50, 60-frame instances poison the next), the codec matters (h264-after-h264 self-sustains the garbage mode instead of watchdogging), and the state crosses geometry (a good 720p instance poisons a following 1080p one). No host-visible model fits all observations; further black-box probing was abandoned as unproductive.

## What it is not, each ruled out by measurement [confirmed]

- **The host driver's inputs**: instrumented register dumps at PIC_RUN entry, STREAMOFF, and failure show `W5_VPU_VPU_INT_STS`, `W5_VPU_VINT_REASON`, `W5_RET_QUEUE_CMD_DONE_INST`, `W5_RET_SEQ_DONE_INSTANCE_INFO`, `W5_RET_QUEUE_STATUS`, `W5_VPU_BUSY_STATUS` all zero at every probe of good and bad instances alike. No completion or queue state crosses the instance boundary.
- **Buffer placement**: every vdi allocation of every instance lands at byte-identical addresses and sizes; the pool returns to the same fill level each time.
- **Runtime PM**: the alternation is unchanged with `power/control` pinned `on` (no sleep/wake between instances). Note the VPU's sleep/wake saves and restores firmware state by design, so it preserves the poison faithfully.
- **STREAMOFF sequencing**: dropping the explicit STREAMOFF pair and closing the fd instead changes nothing.
- **A live sibling instance**: keeping a second (480p, paced) encoder instance streaming while instances are created and destroyed around it perturbs the pattern (back-to-back failures appear) but does not prevent corruption. Firmware instance count reaching zero is not the trigger.
- **The stored firmware**: `chagall.bin` is uploaded at probe and identical every boot, and the first instance of every boot is clean. The corrupt state is in the running VCPU's memory, not the blob.
- **Anything upstream of the encoder**: the garbage mode reproduces with a fixed synthetic test pattern in dma-heap buffers, no capture hardware involved.

## Vendor comparison

- The vendor userspace close path (`VPU_EncClose` in `libmpp_service.so`) is semantically identical to the mainline driver's close: `ProductVpuEncFiniSeq` with a retry on STILL_BUSY, buffer frees, `FreeCodecInstance`. No reset, no sleep/wake, no extra commands. **[confirmed]**
- The vendor air unit creates one encoder instance at boot and never destroys it, so the vendor stack never exercises this path on this device. The defect is present but unreachable there. **[inferred]**
- Mainline wave5 as of 6.18 has no fix in this area (post-6.18 patches address job-abort races and CMD_STOP frame drops only). **[confirmed]**

## The goggle [open]

The goggle rebuilds its DVR record bin, encoder element included, on every recording start/stop (`userspace/gstreamer/src/ml-pipeline/mlp-record.c`), and repeated recordings work. That is repeated encoder instance create/destroy against the same driver and firmware without visible failures. The codec DT nodes are identical apart from the memory pool (air 32 MiB at `0x25000000`, goggle 110 MiB at `0x29200000`). A live sibling instance does not explain it (ruled out above, and the goggle's RF decoders were the candidate). Whether the goggle is genuinely immune or just unmeasured is open: `ml-cam2enc -e` runs there unmodified and would answer it.

## Consequences for callers

- **One long-lived encoder instance per firmware boot.** Create the encoder once and keep it for the session; this is also the vendor's usage pattern. Recording start/stop must gate the muxer/file branch, not the encoder.
- **Any test harness that opens the encoder repeatedly is measuring this defect, not its subject.** A series of per-run encoder instances produces ~50% corrupt output regardless of what is fed in.
- The decoder has shown no equivalent behaviour under heavy instance churn.
- Clearing the state requires re-booting the VCPU from firmware. Today that means a power cycle: a warm `rmmod`/`insmod` of wave5 fails (`vpu_init_with_bitcode: -16`) and costs the codec for the rest of the boot. An in-driver VPU reset plus firmware re-upload (without module teardown) is a plausible but unbuilt recovery path.

## Reproduction

No camera, no bring-up, ~30 s on the device:

```
glue/camera/au-enc-repeat.sh          # RUNS=, FRAMES=, EXTRA= (extra ml-cam2enc flags)
```

`ml-cam2enc -e` feeds the encoder a fixed synthetic pattern from dma-heap buffers and prints `STREAM: <coded> coded, <bytes> bytes, hash <hex>` (FNV-1a over every coded byte), so repeat runs of one build must hash identically. Healthy 60-frame HEVC 1080p: `16306 bytes, 06b372ee507e0199`. Watchdog: `1 coded, 0 bytes`. Garbage: megabytes. `glue/camera/au-enc-holder.sh` is the sibling-instance variant.
