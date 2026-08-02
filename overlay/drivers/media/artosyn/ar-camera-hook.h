/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-camera-hook.h - the per-frame callback the ISP hands to the output stage.
 *
 * The CVISP writes YUV to DRAM but its own completion path is not usable by
 * this driver: the vendor services it in cvisp_dispatch_irq, and neither its
 * interrupt number nor its acknowledge register has been recovered. The VIF's
 * buffer completion is not an alternative either, because it only runs for the
 * v4l2 capture path and the bypass view that would drive it has never completed
 * on this hardware.
 *
 * What does fire once per frame is the ISP's statistics event, measured at one
 * per frame against the ISP interrupt count. So the ISP owns the frame tick and
 * the output stage subscribes to it.
 *
 * Direction matters. The ISP exports the registration and the CVISP calls it,
 * so the module dependency runs cvisp -> isp, matching the order they are
 * loaded in. The reverse would require the output stage to be present before
 * the ISP could load at all.
 *
 * The callback runs in hard interrupt context: no sleeping, no logging, and
 * nothing that takes a mutex.
 */

#ifndef AR_CAMERA_HOOK_H
#define AR_CAMERA_HOOK_H

/*
 * Install or remove the per-frame callback. Passing NULL removes it. Safe
 * against a concurrent interrupt: the ISP holds a spinlock across both the
 * update and the call, so a callback is never invoked after this returns.
 */
void ar_isp_set_frame_hook(void (*fn)(void *ctx), void *ctx);

/*
 * Bring the input path up: the VIF's clock and block configuration, its
 * completion path, and the sensor. Returns once the pixel domain is confirmed
 * live, because the ISP configuration that follows reads registers and a read
 * with the pixel domain dead hangs the SoC.
 *
 * -ENODEV if the VIF has not probed or its sensor never bound, -EBUSY if the
 * input path is already running, -ETIMEDOUT if no frame event arrives.
 */
int ar_vif_input_start(void);
void ar_vif_input_stop(void);

/*
 * Configure the ISP and arm its output. Must be called with the input path
 * already live. -ENODEV if the ISP has not probed.
 */
int ar_isp_pipeline_start(void);

#endif /* AR_CAMERA_HOOK_H */
