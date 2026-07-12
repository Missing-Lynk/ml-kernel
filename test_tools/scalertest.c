// SPDX-License-Identifier: GPL-2.0
/*
 * scalertest - self-contained Tier-0 residual test for ar_scaler, used when the
 * closed libhal_scaler is not present. Allocates a src + dst buffer from MMZ
 * (/dev/mmz_userdev), issues ONE single CropResize on /dev/arscaler, and reports
 * the return code. The point is NOT pixel-correctness (we have no golden image
 * here) but whether the op COMPLETES: rc==0 means the clock bring-up + completion
 * IRQ path work; rc==-ETIMEDOUT (or a 500-jiffy stall) means the recovered clock
 * sequence / register packing is wrong and the IRQ never fired. After the call,
 * read /proc/arscaler/state on the device to inspect the programmed registers.
 *
 * Run on the device after `insmod ar_osal.ko ...` + `insmod ar_scaler.ko`.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "../modules/ar_uapi.h"		/* IOC_MMB_ALLOC / IOC_MMB_FREE / struct mmb_info */

/* mirror of struct ar_scaler_op in ar_scaler.c (64 bytes; all fields physical). */
struct scaler_op {
	unsigned int srcphy, srcw, srch, srcstride;
	unsigned int crop_x, crop_y, cropw, croph;
	unsigned int dstphy, dstw, dsth, dststride;
	unsigned int channels, control, interp, ctrl3c;
};
#define SCALER_IOC_SINGLE	_IOWR('Z', 1, struct scaler_op)	/* 0xc0405a01 */

/* alloc an MMZ block, return its phys (0 on failure). */
static unsigned long mmz_alloc(int mmz, unsigned long size)
{
	struct mmb_info mi;

	memset(&mi, 0, sizeof(mi));
	mi.size = size;
	mi.align = 0x1000;
	strncpy(mi.mmb_name, "scalertest", sizeof(mi.mmb_name) - 1);
	if (ioctl(mmz, IOC_MMB_ALLOC, &mi)) {
		perror("IOC_MMB_ALLOC");
		return 0;
	}

	return (unsigned long)mi.phys_addr;
}

int main(void)
{
	int mmz, sc, rc;
	struct scaler_op op;
	/* 1280x720 -> 640x360, RGB-ish stride aligned to 16; 4 bytes/px assumed */
	const unsigned int sw = 1280, sh = 720, dw = 640, dh = 360;
	unsigned int sstr = (sw * 4 + 15) & ~15u, dstr = (dw * 4 + 15) & ~15u;
	unsigned long sphys, dphys;

	mmz = open("/dev/mmz_userdev", O_RDWR);
	if (mmz < 0) {
		perror("open mmz_userdev");
		return 1;
	}

	sc = open("/dev/arscaler", O_RDWR);
	if (sc < 0) {
		perror("open arscaler");
		return 1;
	}

	sphys = mmz_alloc(mmz, (unsigned long)sstr * sh);
	dphys = mmz_alloc(mmz, (unsigned long)dstr * dh);
	if (!sphys || !dphys)
		return 1;

	printf("src phys=0x%lx (%ux%u str=%u)  dst phys=0x%lx (%ux%u str=%u)\n",
	       sphys, sw, sh, sstr, dphys, dw, dh, dstr);

	memset(&op, 0, sizeof(op));
	op.srcphy = sphys;
	op.srcw = sw;
	op.srch = sh;
	op.srcstride = sstr;
	op.crop_x = 0;
	op.crop_y = 0;
	op.cropw = sw;
	op.croph = sh;
	op.dstphy = dphys;
	op.dstw = dw;
	op.dsth = dh;
	op.dststride = dstr;
	op.channels = 1;

	errno = 0;
	rc = ioctl(sc, SCALER_IOC_SINGLE, &op);
	if (rc == 0) {
		printf("CropResize COMPLETED (rc=0) - clock + completion IRQ path OK\n");
	} else {
		printf("CropResize FAILED rc=%d errno=%d (%s)\n", rc, errno,
		       strerror(errno));
		printf("  (-ETIMEDOUT/-62 => IRQ never fired: clock seq or reg packing wrong;\n");
		printf("   check /proc/arscaler/state and dmesg)\n");
	}

	return rc ? 1 : 0;
}
