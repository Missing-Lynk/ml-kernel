/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-dpc.h - the vendor's defect-pixel-correction payload, carried verbatim.
 *
 * The vendor isp_memcpy's a whole register image out of its init_config object
 * into the head of bank 0x0c00, so the payload is fixed library data and
 * carrying it verbatim is the vendor's own mechanism.
 *
 * Read from a streaming vendor unit on 2026-08-02 at the 1080p60 operating
 * point, over the exact extent the register replay covers (0x0c00 to 0x0d08),
 * so nothing outside it changes hands.
 *
 * Two independent defects in the replay this replaces, both measured against
 * this capture:
 *
 *   Thresholds far too low. 0x0c1c and 0x0c28 carried 60 against the vendor's
 *   1000, 0x0c20 200 against 1400, 0x0c34 1300 against 3000. A defect detector
 *   with a sixteenth of the intended threshold fires on ordinary scene detail,
 *   which is the measured over-correction: it lands hardest at frame edges,
 *   where the neighbourhood a detector needs is truncated.
 *
 *   A second parameter group enabled that the vendor never enables. Everything
 *   from 0x0c78 to 0x0cdc reads zero on the vendor and carried a full duplicate
 *   threshold set here, 0x0c78 among them holding the value 1 where the vendor
 *   holds 0. So the replay ran a second correction pass with the same too-low
 *   thresholds on top of the first.
 *
 * Applied after the setup tables, for the reason the ccm and rnr stages are: a
 * bring-up that runs a prefix of the replay would otherwise leave whichever
 * partial state the prefix reached.
 */

#ifndef AR_ISP_DPC_H
#define AR_ISP_DPC_H

static const struct ar_isp_reg ar_isp_dpc_payload[] = {
	{ 0x0c00, 0x00000000 },
	{ 0x0c04, 0x00000780 },
	{ 0x0c08, 0x00000438 },
	{ 0x0c0c, 0x00000000 },
	{ 0x0c10, 0x00000001 },
	{ 0x0c14, 0x00000040 },
	{ 0x0c18, 0x00000000 },
	{ 0x0c1c, 0x000003e8 },
	{ 0x0c20, 0x00000578 },
	{ 0x0c24, 0x00000640 },
	{ 0x0c28, 0x000003e8 },
	{ 0x0c2c, 0x000000c8 },
	{ 0x0c30, 0x00000320 },
	{ 0x0c34, 0x00000bb8 },
	{ 0x0c38, 0x000000c8 },
	{ 0x0c3c, 0x00000190 },
	{ 0x0c40, 0x000003e8 },
	{ 0x0c44, 0x000000c8 },
	{ 0x0c48, 0x00000320 },
	{ 0x0c4c, 0x00000033 },
	{ 0x0c50, 0x00000020 },
	{ 0x0c54, 0x00000040 },
	{ 0x0c58, 0x00000080 },
	{ 0x0c5c, 0x00000100 },
	{ 0x0c60, 0x00000200 },
	{ 0x0c64, 0x00000000 },
	{ 0x0c68, 0x00000020 },
	{ 0x0c6c, 0x00000060 },
	{ 0x0c70, 0x000000a0 },
	{ 0x0c74, 0x000000e0 },
	{ 0x0c78, 0x00000000 },
	{ 0x0c7c, 0x00000000 },
	{ 0x0c80, 0x00000000 },
	{ 0x0c84, 0x00000000 },
	{ 0x0c88, 0x00000000 },
	{ 0x0c8c, 0x00000000 },
	{ 0x0c90, 0x00000000 },
	{ 0x0c94, 0x00000000 },
	{ 0x0c98, 0x00000000 },
	{ 0x0c9c, 0x00000000 },
	{ 0x0ca0, 0x00000000 },
	{ 0x0ca4, 0x00000000 },
	{ 0x0ca8, 0x00000000 },
	{ 0x0cac, 0x00000000 },
	{ 0x0cb0, 0x00000000 },
	{ 0x0cb4, 0x00000000 },
	{ 0x0cb8, 0x00000000 },
	{ 0x0cbc, 0x00000000 },
	{ 0x0cc0, 0x00000000 },
	{ 0x0cc4, 0x00000000 },
	{ 0x0cc8, 0x00000000 },
	{ 0x0ccc, 0x00000000 },
	{ 0x0cd0, 0x00000000 },
	{ 0x0cd4, 0x00000000 },
	{ 0x0cd8, 0x00000000 },
	{ 0x0cdc, 0x00000000 },
	{ 0x0ce0, 0x00000000 },
	{ 0x0ce4, 0x00000000 },
	{ 0x0ce8, 0x00000000 },
	{ 0x0cec, 0x00000000 },
	{ 0x0cf0, 0x00000000 },
	{ 0x0cf4, 0x00000000 },
	{ 0x0cf8, 0x00000000 },
	{ 0x0cfc, 0x00000000 },
	{ 0x0d00, 0x00000000 },
	{ 0x0d04, 0x00000000 },
	{ 0x0d08, 0x000000ff },
};

#endif /* AR_ISP_DPC_H */
