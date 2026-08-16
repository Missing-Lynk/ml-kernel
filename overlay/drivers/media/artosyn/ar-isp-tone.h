/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-tone.h - the AEC trigger selector for the gamma and DRC pages.
 *
 * Recovered from is_aec_trigger_compute in libmpp_service.so at 0x179530, the
 * helper both stages call. The runnable proof is
 * kernel/scripts/isp/check-trigger-scalar.py, which reproduces two captured
 * vendor pages from the blob through this selector and the blend beside it in
 * ar-isp-codec.h.
 *
 * The vendor's AE hands every triggered ISP module one payload carrying two
 * abscissae, and each module reads whichever its own tuning header names. The
 * five noise and demosaic ladders take a linear gain; gamma, DRC, cm and cm2
 * take the scalar this file selects on, which runs 0 to 550 and is the open
 * item in plans/isp-tone-selector.md.
 *
 * Selection walks the stage's band table, a pair of floats per entry:
 *
 *	scalar inside band i           -> low = high = i, no blend
 *	scalar in the gap after band i -> low = i, high = i + 1, blended
 *	scalar below the first band    -> entry 0
 *	scalar above the last band     -> the last entry
 *
 * Pure data transforms: no register access and no kernel API beyond the integer
 * types, so the same source compiles host-side against the captures.
 */

#ifndef AR_ISP_TONE_H
#define AR_ISP_TONE_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-bytes.h"
#include "ar-isp-codec.h"
#include "ar-isp-ladder.h"

/* Band tables, and the header word holding each entry count. */


/* A band is two float32, the inclusive low and high edge. */
#define AR_ISP_TONE_BAND_STRIDE		8

struct ar_isp_tone_pick {
	unsigned int low;
	unsigned int high;
	u32 t_q12;
};

/*
 * The scalar arrives as a Q8 integer because that is what a module parameter
 * can carry; the vendor's is a float. Q8 is finer than any band edge in the
 * blob, all of which are whole counts.
 */
static inline void ar_isp_tone_select(struct ar_isp_tone_pick *out,
				      const u8 *blob, u32 bands_off,
				      unsigned int count, u32 scalar_q8)
{
	u32 prev_hi = 0;

	/* Q16 is what the shared float helper produces; every band edge in the
	 * blob is a whole count, so narrowing to the parameter's Q8 is exact.
	 */
	for (unsigned int i = 0; i < count; i++) {
		u32 lo = ar_isp_f32_q16(ar_isp_get_le32(blob + bands_off +
						        i * AR_ISP_TONE_BAND_STRIDE)) >> 8;
		u32 hi = ar_isp_f32_q16(ar_isp_get_le32(blob + bands_off +
						        i * AR_ISP_TONE_BAND_STRIDE + 4)) >> 8;

		if (scalar_q8 <= hi) {
			/*
			 * Inside band i, or in the gap that precedes it. The
			 * gap case cannot arise for i == 0: the first band's
			 * low edge is 0 on every stage, so nothing sits under
			 * it.
			 */
			if (scalar_q8 >= lo || !i) {
				out->low = i;
				out->high = i;
				out->t_q12 = 0;
			} else {
				out->low = i - 1;
				out->high = i;
				out->t_q12 = (u32)(((u64)(scalar_q8 - prev_hi) <<
						    AR_ISP_TONE_BLEND_BITS) /
						   (lo - prev_hi));
			}

			return;
		}

		prev_hi = hi;
	}

	/* Past the last band: the vendor clamps to the last entry. */
	out->low = count - 1;
	out->high = count - 1;
	out->t_q12 = 0;
}

static inline void ar_isp_tone_pick_gamma(struct ar_isp_tone_pick *out,
					  const u8 *blob, u32 scalar_q8)
{
	unsigned int count = ar_isp_get_le32(blob + AR_ISP_GAMMA_BLOB_COUNT);

	if (count < 1 || count > AR_ISP_GAMMA_BLOB_CURVES)
		count = AR_ISP_GAMMA_BLOB_CURVES;

	ar_isp_tone_select(out, blob, AR_ISP_GAMMA_BLOB_BANDS, count, scalar_q8);
}

static inline void ar_isp_tone_pick_drc(struct ar_isp_tone_pick *out,
					const u8 *blob, u32 scalar_q8)
{
	unsigned int count = ar_isp_get_le32(blob + AR_ISP_DRC_BLOB_COUNT);

	if (count < 1 || count > AR_ISP_DRC_BLOB_PROFILES)
		count = AR_ISP_DRC_BLOB_PROFILES;

	ar_isp_tone_select(out, blob, AR_ISP_DRC_BLOB_BANDS, count, scalar_q8);
}

#endif /* AR_ISP_TONE_H */
