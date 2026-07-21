# Wave5 codec capabilities (WAVE521C on the Proxima-9311), decoded

What the codec hardware/firmware actually reports and what we have hardware-validated. Sources: the `GET_VPU_INFO` result registers captured in a golden trace of the vendor stack (read-back after the vendor's INIT/QUERY), decoded per the mainline driver's field definitions (`wave5-hw.c` `setup_wave5_properties` + the `W521_FEATURE_*` bit macros), plus our own probe prints and decode measurements on the open driver. Status tracking stays in `../STATUS.md`; this is the stable reference.

## Raw GET_VPU_INFO result registers (golden trace)

| register | offset | value | decoded |
|---|---|---|---|
| `W5_RET_PRODUCT_NAME` | 0x011c | `0x57415645` | ASCII "WAVE" |
| `W5_RET_PRODUCT_VERSION` | 0x0120 | `0x0000521c` | WAVE521C |
| `W5_RET_FW_VERSION` | 0x0118 | `0x00050b8e` | fw revision 330638 (vendor runtime; see note below) |
| `W5_RET_CUSTOMER_ID` | 0x0140 | `0x00000000` | generic build |
| `W5_RET_STD_DEF0` | 0x0124 | `0x000d0980` | hardware config, see bits below |
| `W5_RET_STD_DEF1` | 0x0128 | `0x08818003` | codec support bits, see below |
| `W5_RET_CONF_FEATURE` | 0x012c | `0x00000507` | feature bits, see below |
| `W5_RET_CONF_DATE` | 0x0130 | `0x01343ca7` | 20200615 decimal = RTL config date 2020-06-15 |
| `W5_RET_CONF_REVISION` | 0x0134 | `0x00031345` | RTL config revision 201541 |

Firmware note: our open driver (loading the extracted `chagall.bin`) prints `Firmware Revision: 329715` (`0x507f3`), while the vendor trace read `330638` (`0x50b8e`). Same silicon, slightly different firmware builds between the extracted blob and whatever the vendor runtime had loaded when the trace was taken. Both decode bit-exact for us, so the delta is cosmetic until proven otherwise.

## Decoded capability bits

`W5_RET_STD_DEF1 = 0x08818003` (codec support, W521 field layout):

| bit | meaning | value |
|---|---|---|
| 0 | HEVC encoder | **1** |
| 1 | AVC (H.264) encoder | **1** |
| 2 | HEVC decoder | **0** (!) |
| 3 | AVC decoder | **0** (!) |
| 15,16,23,27 | set, not decoded by mainline | 1 |

The decoder bits reading 0 is the known **Artosyn integration quirk**: this WAVE521C's config registers do not advertise the decoder even though the firmware fully implements it (the vendor stack never checks these bits; mainline does). That is exactly why our overlay carries the decoder-capability fix that forces DECODE support on this product - without it the mainline driver would register no decoder at all. Empirically the decoder exists and is bit-exact (below).

`W5_RET_CONF_FEATURE = 0x00000507`:

| bit | meaning | value |
|---|---|---|
| 3 | HEVC 10-bit encode | 0 (8-bit encode only) |
| 11 | AVC 10-bit encode | 0 |
| 0,1,2,8,10 | set, not decoded by mainline | 1 |

No 10-bit decode flag exists in the W521 layout (mainline only parses one for WAVE515), and the driver exposes 8-bit output formats only - treat this codec as **8-bit in/out** unless someone proves Main10 streams decode.

`W5_RET_STD_DEF0 = 0x000d0980` (hardware config):

| bit | meaning | value |
|---|---|---|
| 16 | backbone present | **1** (single combined backbone; bus-idle mask is `0x3f` on this integration, the mainline `0x00ff1f3f` mask is the reset-hang bug our overlay fixes) |
| 22 | per-vcore backbone | 0 |
| 28 | vcpu backbone | 0 |
| 7,8,11,18,19 | set, not decoded by mainline | 1 |

## Hardware-validated decode matrix (open driver)

All results **bit-exact vs host ffmpeg** (mean-abs-diff 0.000 on Y, U and V):

| stream | result |
|---|---|
| HEVC 640x360 | bit-exact, 8/8 frames |
| HEVC 1280x720 | bit-exact, 8/8 frames |
| HEVC 1920x1080 | bit-exact (needs a lean CAPTURE queue, see memory note) |
| H.264 1280x720 | bit-exact |

Throughput (stateful V4L2 m2m, discard sink, low-latency no-B streams): **323 fps @ 1280x720**, **165 fps @ 1920x1080**. A stateful decoder has no framerate cap - it decodes as fast as it is fed - so any link rate (60/90/120 fps) has huge margin.

Driver-advertised input ranges (untested beyond 1080p, and 1080p is the panel/pipeline max anyway): HEVC 8x8 to 8192x4320, H.264 32x32 to 8192x4320. Output pixel formats offered: I420 (`YU12`, validated), NV12/NV21, YUV422P/NV16/NV61 plus multiplanar variants (untested).

## Memory budget at 1080p (the practical constraint)

The codec allocates from its dedicated MMZ pool (the DTS reserved-memory node), bound as the device's dma-coherent pool. The pool allocator is page-granular first-fit (`patches/0011-dma-coherent-page-granular.patch`; mainline rounds every allocation to a power-of-2 page order, which costs 25-30% overhead on codec-sized buffers - e.g. a 3,133,440 B encoder FBC would occupy 4 MiB, a 5,041,296 B vb_task 8 MiB). At 1920x1080 the FBC reference pool plus linear CAPTURE buffers are the big consumers; a deep B-pyramid stream (7+ reference slots) combined with a generous CAPTURE queue can still fail framebuffer registration with ENOMEM (`Framebuffer preparation, fail: -12`). FPV/DVR streams are low-latency no-B encodes, so this does not bite in practice.

Do NOT cap the decoder's linear CAPTURE count (`dec_cap_bufs` param): any value below the stream's `fbc_buf_count` (9 for 1080p60 low-latency streams) starves the firmware's display-buffer rotation and it spins at ~25-40 fps with both A53 cores pegged. `dec_cap_bufs=0` (stock depth) is the only correct setting.

## Concurrent operation (2x decode + encode, V4L2)

The full DVR load runs concurrently through the standard stateful V4L2 m2m interface, all three channels on the one device/pool. Hardware-validated at 60 fps sustained:

| channel | config | pool cost (page-granular) |
|---|---|---|
| decoder 0 (`/dev/video0` instance) | H.265 960x1080 @ 60, 9 linear CAPTURE + FBC refs | ~36.5 MiB |
| decoder 1 (second instance) | H.265 960x1080 @ 60, same | ~36.5 MiB |
| encoder (`/dev/video1`) | H.264 1920x1080 @ 60, 8 FBC + linear src queue | ~16 MiB |

Working set ~92 MiB of 108 (plus ~3 MiB common/fw). Measured result: both decoders at 60 fps, encoder consuming every composite frame (480/480 over an 8 s run, 0 drops), output MP4 decodes clean on the host. On the pre-0011 power-of-2 allocator the same load ENOMEMs at the encoder's first FBC allocation.

## Hardware-validated encode matrix (open driver)

All results verified by decoding the device-encoded stream with host ffmpeg and measuring PSNR against the raw source frames (I+P GOP, rate control enabled):

| stream | quality | notes |
|---|---|---|
| HEVC 1280x720 @ 5 Mbps | 42.6-45.3 dB | Main profile |
| HEVC 1920x1080 @ 8 Mbps | 43.7-48.1 dB, all planes 47+ dB | Main profile, proper 1920x1080 conf window |
| H.264 1280x720 @ 5 Mbps | 42.4-44.7 dB | Baseline profile |
| H.264 1920x1080 @ 8 Mbps | 43.3-46.0 dB | Baseline profile |

Throughput (stateful V4L2 m2m, discard sink): **171 fps @ 1280x720**, **108 fps @ 1920x1080** - and that includes the test tool's userspace memcpy of every raw frame into the vb2 buffer, so the hardware floor is higher. Keyframe flags, EOS drain (`V4L2_ENC_CMD_STOP` -> `V4L2_BUF_FLAG_LAST`) and per-frame `V4L2_EVENT_EOS` all behave.

Userspace contract (matters for hand-rolled clients; GStreamer/FFmpeg handle it):

- Source I420 plane offsets use the driver-aligned OUTPUT height, not the visible height: chroma starts at `bytesperline * ALIGN(height, 16)` (1088 rows at 1080p). This matches the vendor's own `align16(w) * align16(h) * 3 / 2` source-buffer size convention.
- Set `VIDIOC_S_SELECTION(OUTPUT, V4L2_SEL_TGT_CROP, WxH)` for the conformance window, or the stream signals the aligned size (1088-tall 1080p with visible padding rows).

Both WAVE521C encode fixes live in the overlay: sec-AXI forced off (same zero-size-accounting mainline TODO as decode, `wave5_vpu_enc_validate_sec_axi`), and `finish_encode`'s error path made to finish the m2m job (mainline silently leaks it and a later STREAMOFF deadlocks the whole VPU).

## Encoder parameters (V4L2 control surface)

The driver exposes the standard V4L2 stateful-encoder controls (set before STREAMON; from the ctrl handler in `wave5-vpu-enc.c`). Framerate is set via `VIDIOC_S_PARM` on the OUTPUT queue (`timeperframe`), not a control.

Rate control - the one gotcha:

- `FRAME_RC_ENABLE` (bool, **default 0 = OFF**): with RC off the encoder runs **fixed-QP** (the `*_I_FRAME_QP` default 30) and `BITRATE` is ignored. The validated matrix above was measured in this fixed-QP-30 mode (the streams came out ~1.5 Mbps at 720p, ~4-5 Mbps at 1080p). For actual bitrate targeting set `FRAME_RC_ENABLE=1` + `BITRATE`; `MB_RC_ENABLE` (default 0) adds macroblock/CU-level RC on top.
- `BITRATE` 0..700 Mbps (default 0), `BITRATE_MODE`: CBR only, `VBV_SIZE` 10..3000 ms (default 1000; the driver forces 3000 when RC is off).

Common: `GOP_SIZE` 0..2047 (default 0; the fw GOP preset is IPP single-ref, so streams are I+P only - no B frames), `PREPEND_SPSPPS_TO_IDR` (default 0), `AU_DELIMITER` (default 1 = AUD NALs on), `MULTI_SLICE_MODE` single/max-MB + `MULTI_SLICE_MAX_MB`, `HFLIP`/`VFLIP`/`ROTATE` (0/90/180/270), `MIN_BUFFERS_FOR_OUTPUT` (read after seq init).

HEVC: `HEVC_PROFILE` Main or Main10 menu (default Main - **Main10 will not produce 10-bit**, the silicon is 8-bit, see CONF_FEATURE above), `HEVC_LEVEL` up to 5.1 (default: driver picks), `HEVC_{MIN,MAX}_QP` 0..63 (defaults 8/51), `HEVC_I_FRAME_QP` (default 30, the fixed-QP knob), `HEVC_REFRESH_TYPE` IDR + `HEVC_REFRESH_PERIOD` 0..2047, loop-filter mode + beta/tc offsets, `HEVC_LOSSLESS_CU`, `HEVC_CONST_INTRA_PRED`, `HEVC_WAVEFRONT`, `HEVC_STRONG_SMOOTHING` (default on), `HEVC_TMV_PREDICTION` (default on), `HEVC_MAX_NUM_MERGE_MV_MINUS1` (default 2).

H.264: `H264_PROFILE` menu up to High 4:4:4 Predictive (default Baseline; the validated streams are Baseline), `H264_LEVEL` up to 5.1, `H264_{MIN,MAX}_QP` 0..63 (8/51), `H264_I_FRAME_QP` (30), `H264_I_PERIOD` 0..2047, `H264_ENTROPY_MODE` CAVLC/CABAC (default CAVLC), `H264_8X8_TRANSFORM` (default on), `H264_LOOP_FILTER_MODE` + alpha/beta, `H264_CONSTRAINED_INTRA_PREDICTION`, `H264_CHROMA_QP_INDEX_OFFSET` -12..12.

Not exposed by the mainline driver (fw supports more via ENC_SET_PARAM, unmapped): B-frame GOP presets, custom GOP structure, ROI/custom QP maps, HVS-QP tuning knobs beyond the fixed defaults in `wave5_set_enc_openparam` (hvs_qp_scale 2, nr weights 7/4, rdo_skip 1, lambda_scaling 1).
