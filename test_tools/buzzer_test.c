// SPDX-License-Identifier: GPL-2.0
/*
 * buzzer_test - exercise the goggle buzzer through the artosyn_pwm driver's sysfs ABI.
 *
 * The buzzer is channel 0 of the 2nd artosyn PWM controller (reg 0x1002000, DT node
 * pwm@1002000). This tool drives it the same way userspace will: export the channel,
 * set the period, and step the duty cycle. It goes through /sys/class/pwm, so it
 * validates the whole driver path - DT probe -> pwmchip registration -> ar_pwm_apply /
 * ar_pwm_get_state - not a /dev/mem back door.
 *
 * Two things are independently adjustable on a PWM buzzer:
 *   loudness = duty cycle at a fixed period. From stock customerHmBuzzerEnable: period
 *              260000 ns (~3.8 kHz), duty = 13000*vol ns for volume 1..10 (0 = silent).
 *              We stop at the firmware
 *              max (vol 10 = 130000 ns = 50% duty); the HW itself goes to 100%.
 *   pitch    = tone frequency = 1/period. Sweeping it means stepping the period while
 *              holding the duty at a fixed 50% ratio (the loudest the firmware uses).
 *
 * The controller enumerates as a pwmchipN whose number depends on probe order. Rather
 * than hard-code it, we scan /sys/class/pwm/pwmchip*\/device and pick the one whose
 * platform device is "<addr>.pwm" (default 1002000). This also prints which pwmchipN
 * the driver gave the buzzer - the number the board profile needs.
 *
 * Usage:
 *   buzzer_test                 auto-detect the chip, sweep volume 1..10 (hear the steps)
 *   buzzer_test vol             same as above (explicit)
 *   buzzer_test pitch           auto-detect, sweep pitch 1..5 kHz at fixed 50% duty
 *   buzzer_test <volume>        auto-detect, play a single volume 0..10 for ~1s
 *   buzzer_test <chipdir> ...   use an explicit chip dir first, e.g.
 *                               buzzer_test /sys/class/pwm/pwmchip1 pitch
 *
 * Build: aarch64-linux-gnu-gcc -static -O2 buzzer_test.c -o buzzer_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>

#define PWM_CLASS   "/sys/class/pwm"
#define BUZZER_ADDR "1002000"    /* 2nd artosyn PWM controller -> "1002000.pwm" */
#define BUZZER_CHAN 0
#define PERIOD_NS   260000L       /* ~3.8 kHz tone, the stock buzzer period */
#define DUTY_STEP_NS 13000L       /* duty = 13000 * volume */
#define BEEP_MS     350           /* per-step chirp length in a sweep */
#define GAP_MS      150           /* silence between steps */

/* Pitch sweep: step the frequency across this range at a fixed 50% duty. */
#define PITCH_LO_HZ  1000L
#define PITCH_HI_HZ  5000L
#define PITCH_STEPS  20

/* Write a decimal value to a sysfs attribute. Returns 0, or -1 (errno set) on failure. */
static int write_long(const char *path, long value)
{
	FILE *f = fopen(path, "w");

	if (!f)
		return -1;

	int ok = (fprintf(f, "%ld\n", value) >= 0);

	if (fclose(f) != 0)
		ok = 0;

	return ok ? 0 : -1;
}

/* Find the pwmchip dir the buzzer controller bound to, by matching its platform device
 * name "<addr>.pwm". Writes the chosen "/sys/class/pwm/pwmchipN" into out. Returns 0 on
 * success, -1 if no matching chip is present.
 */
static int find_buzzer_chip(char *out, size_t outlen)
{
	DIR *d = opendir(PWM_CLASS);

	if (!d) {
		fprintf(stderr, "open %s: %s\n", PWM_CLASS, strerror(errno));
		return -1;
	}

	struct dirent *e;
	int found = -1;

	while ((e = readdir(d))) {
		if (strncmp(e->d_name, "pwmchip", 7) != 0)
			continue;

		char link[512];
		char target[512];

		snprintf(link, sizeof(link), "%s/%s/device", PWM_CLASS, e->d_name);
		ssize_t n = readlink(link, target, sizeof(target) - 1);

		if (n < 0)
			continue;
		target[n] = '\0';

		/* target basename looks like "1002000.pwm" */
		const char *base = strrchr(target, '/');

		base = base ? base + 1 : target;
		if (strstr(base, BUZZER_ADDR ".pwm")) {
			snprintf(out, outlen, "%s/%s", PWM_CLASS, e->d_name);
			printf("buzzer controller (%s.pwm) -> %s\n", BUZZER_ADDR, e->d_name);
			found = 0;
			break;
		}
	}
	closedir(d);

	if (found < 0) {
		fprintf(stderr, "no pwmchip for %s.pwm found under %s (is pwm@1002000 in the DT and artosyn_pwm loaded?)\n",
			BUZZER_ADDR, PWM_CLASS);
	}

	return found;
}

/* Export channel 0 (harmless if already exported). Fills chan_dir with its pwmN path.
 * The period is programmed per-tone by play_tone(), so nothing is set here.
 */
static int setup_channel(const char *chip, char *chan_dir, size_t len)
{
	char export_path[512];

	snprintf(export_path, sizeof(export_path), "%s/export", chip);
	if (write_long(export_path, BUZZER_CHAN) != 0 && errno != EBUSY) {
		fprintf(stderr, "export %s: %s\n", export_path, strerror(errno));
		return -1;
	}

	/* the pwmN sysfs may take a moment to appear after export */
	usleep(50 * 1000);

	snprintf(chan_dir, len, "%s/pwm%d", chip, BUZZER_CHAN);
	return 0;
}

/* Program period+duty and enable for `ms`, then disable. duty<=0 leaves it silent.
 * We write duty=0 before changing the period so the PWM core never sees duty>period
 * (which it rejects), regardless of whether the period is growing or shrinking.
 */
static void play_tone(const char *chan_dir, long period, long duty, int ms)
{
	char period_path[512];
	char duty_path[512];
	char enable_path[512];

	snprintf(period_path, sizeof(period_path), "%s/period", chan_dir);
	snprintf(duty_path, sizeof(duty_path), "%s/duty_cycle", chan_dir);
	snprintf(enable_path, sizeof(enable_path), "%s/enable", chan_dir);

	if (write_long(duty_path, 0) != 0 ||
		write_long(period_path, period) != 0 ||
		write_long(duty_path, duty) != 0) {
		fprintf(stderr, "program tone (%ld ns / %ld ns): %s\n",
				period, duty, strerror(errno));

		return;
	}

	if (duty <= 0)
		return;

	write_long(enable_path, 1);
	usleep(ms * 1000);
	write_long(enable_path, 0);
}

/* Loudness step: fixed stock period, duty = DUTY_STEP_NS * volume. volume 0 = silent. */
static void beep(const char *chan_dir, int volume, int ms)
{
	long duty = DUTY_STEP_NS * volume;

	printf("  volume %2d -> duty %6ld ns (%2ld%%)%s\n",
		       volume, duty, duty * 100 / PERIOD_NS, volume ? "" : "  [silent]");
	play_tone(chan_dir, PERIOD_NS, duty, ms);
}

/* Pitch step: given frequency, period = 1e9/freq and duty = period/2 (50%). */
static void chirp(const char *chan_dir, long freq_hz, int ms)
{
	long period = 1000000000L / freq_hz;
	long duty = period / 2;

	printf("  %5ld Hz -> period %6ld ns (duty %ld ns, 50%%)\n", freq_hz, period, duty);
	play_tone(chan_dir, period, duty, ms);
}

int main(int argc, char **argv)
{
	char chip[300];        /* PWM_CLASS + "/" + dirname (<= NAME_MAX) */
	char chan_dir[336];    /* chip + "/pwmN" */

	enum { MODE_VOL_SWEEP, MODE_PITCH_SWEEP, MODE_SINGLE } mode = MODE_VOL_SWEEP;
	int single_volume = 0;

	/* Arg parsing: [chipdir] [vol|pitch|<volume>]. A leading "/..." is a chip dir. */
	const char *chip_arg = NULL;
	const char *mode_arg = NULL;

	if (argc >= 2 && argv[1][0] == '/') {
		chip_arg = argv[1];
		mode_arg = (argc >= 3) ? argv[2] : NULL;
	} else if (argc >= 2) {
		mode_arg = argv[1];
	}

	if (mode_arg) {
		if (strcmp(mode_arg, "pitch") == 0) {
			mode = MODE_PITCH_SWEEP;
		} else if (strcmp(mode_arg, "vol") == 0) {
			mode = MODE_VOL_SWEEP;
		} else {
			mode = MODE_SINGLE;
			single_volume = atoi(mode_arg);
			if (single_volume < 0)
				single_volume = 0;

			if (single_volume > 10)
				single_volume = 10;
		}
	}

	if (chip_arg) {
		snprintf(chip, sizeof(chip), "%s", chip_arg);
		printf("buzzer controller -> %s (explicit)\n", chip);
	} else if (find_buzzer_chip(chip, sizeof(chip)) != 0) {
		return 1;
	}

	if (setup_channel(chip, chan_dir, sizeof(chan_dir)) != 0)
		return 1;

	switch (mode) {
		case MODE_SINGLE: {
			printf("single tone:\n");
			beep(chan_dir, single_volume, 1000);
		} break;

		case MODE_PITCH_SWEEP: {
			printf("sweeping pitch %ld..%ld Hz over %d steps (each %d ms):\n",
			       PITCH_LO_HZ, PITCH_HI_HZ, PITCH_STEPS, BEEP_MS);

			for (int i = 0; i < PITCH_STEPS; i++) {
				long freq = PITCH_LO_HZ +
							(PITCH_HI_HZ - PITCH_LO_HZ) * i / (PITCH_STEPS - 1);
				chirp(chan_dir, freq, BEEP_MS);
				usleep(GAP_MS * 1000);
			}
		} break;

		case MODE_VOL_SWEEP: {
			printf("sweeping volume 1..10 (each %d ms):\n", BEEP_MS);
			for (int v = 1; v <= 10; v++) {
				beep(chan_dir, v, BEEP_MS);
				usleep(GAP_MS * 1000);
			}
		} break;
	}

	printf("done (channel left exported, disabled)\n");
	return 0;
}
