/* SPDX-License-Identifier: GPL-2.0 */
/*
 * tone-dump.c - run the shipped tone selector and page builders host-side.
 *
 * The companion to ladder-dump.c, for the other abscissa. It includes
 * ar-isp-tone.h and ar-isp-codec.h unmodified out of the driver directory,
 * selects the gamma curve and DRC profile for one AEC trigger scalar, builds
 * both pages, and writes them out. check-trigger-scalar.py diffs them against
 * pages captured off the streaming vendor, so what is compared is the code the
 * kernel runs and not a Python restatement of it.
 *
 * The DRC page is the strong test. Its dynamic banks carry the tuning file's
 * own 20-bit samples with no transform in between, so a byte-exact match proves
 * the selector, the blend weight and the packer together. Gamma decimates and
 * carries a small residual of its own, which the checker reports rather than
 * demands.
 *
 *   cc -I<driver dir> -o tone-dump tone-dump.c
 *   ./tone-dump <tuning.bin> <scalar-q8> <gamma-out.bin> <drc-out.bin>
 *
 * The selection is printed to stderr so a caller can read it without parsing
 * the pages.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * The headers use the kernel's fixed-width names and nothing else, so the whole
 * compatibility layer is these typedefs.
 */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int32_t s32;
typedef int64_t s64;

#include "ar-isp-tone.h"
#include "vendor-tables/ar-isp-drc-tail.h"
#include "vendor-tables/ar-isp-gamma-page1.h"

/* Large enough for every sensor's tuning file; the shipped one is 859 KiB. */
#define BLOB_MAX (2 * 1024 * 1024)

static u8 *read_blob(const char *path, size_t *len_out)
{
	FILE *f = fopen(path, "rb");
	u8 *buf;
	size_t len;

	if (!f) {
		perror(path);
		exit(1);
	}

	buf = malloc(BLOB_MAX);
	if (!buf) {
		fprintf(stderr, "out of memory\n");
		exit(1);
	}

	len = fread(buf, 1, BLOB_MAX, f);
	fclose(f);

	/* Every offset this reads sits under 0xa2000; the shipped file is
	 * 0xd6c58. The bound only has to rule out a truncated or wrong file.
	 */
	if (len < 0xa2000) {
		fprintf(stderr, "%s: %zu bytes, too short for a tuning file\n",
			path, len);
		exit(1);
	}

	*len_out = len;

	return buf;
}

static void write_page(const char *path, const u8 *page, size_t len)
{
	FILE *f = fopen(path, "wb");

	if (!f) {
		perror(path);
		exit(1);
	}

	if (fwrite(page, 1, len, f) != len) {
		perror(path);
		exit(1);
	}

	fclose(f);
}

int main(int argc, char **argv)
{
	static u8 gamma_page[AR_ISP_GAMMA_SIZE];
	static u8 drc_page[AR_ISP_DRC_SIZE];
	struct ar_isp_tone_pick gamma, drc;
	size_t len;
	u8 *blob;
	u32 scalar;

	if (argc != 5) {
		fprintf(stderr,
			"usage: %s <tuning.bin> <scalar-q8> <gamma-out> <drc-out>\n",
			argv[0]);

		return 2;
	}

	blob = read_blob(argv[1], &len);
	scalar = (u32)strtoul(argv[2], NULL, 0);

	ar_isp_tone_pick_gamma(&gamma, blob, scalar);
	ar_isp_tone_pick_drc(&drc, blob, scalar);

	fprintf(stderr, "scalar %u.%03u: gamma %u to %u weight %u, "
			"drc %u to %u weight %u\n",
		scalar >> 8, (scalar & 0xff) * 1000 / 256,
		gamma.low, gamma.high, gamma.t_q12,
		drc.low, drc.high, drc.t_q12);

	/*
	 * The whole of both pages, exactly as ar_isp_tables_apply builds them:
	 * the selected and blended half from the tuning file, then the carried
	 * halves that no AE input moves. A capture covers both, so comparing
	 * only the dynamic part would leave the carried tail unchecked.
	 */
	ar_isp_gamma_from_blob(gamma_page, blob, gamma.low, gamma.high,
			       gamma.t_q12);
	ar_isp_gamma_pack_page(gamma_page + AR_ISP_GAMMA_PAGE,
			       ar_isp_gamma_page1, AR_ISP_GAMMA_PAGE1_TAIL);

	ar_isp_drc_from_blob(drc_page, blob, drc.low, drc.high, drc.t_q12);
	ar_isp_drc_pack_bank(drc_page + 2 * AR_ISP_DRC_BANK,
			     ar_isp_drc_tail_bank0);
	ar_isp_drc_pack_bank(drc_page + 3 * AR_ISP_DRC_BANK,
			     ar_isp_drc_tail_bank1);

	write_page(argv[3], gamma_page, 2 * AR_ISP_GAMMA_PAGE);
	write_page(argv[4], drc_page, AR_ISP_DRC_SIZE);

	printf("%u %u %u %u %u %u\n", gamma.low, gamma.high, gamma.t_q12,
	       drc.low, drc.high, drc.t_q12);

	return 0;
}
