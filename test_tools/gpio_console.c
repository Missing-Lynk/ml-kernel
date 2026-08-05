// SPDX-License-Identifier: GPL-2.0
// gpio_console <chip-label> [line ...] - hold a set of gpio lines open and drive them
// interactively, one command per line of stdin. Every line keeps a single chardev request
// for the whole session, so changing a state does not pass through high-Z the way a
// kill-and-respawn wrapper around gpio_hold does - which matters when an undriven line
// leaves an LED glowing on leakage and that glow is what is being measured.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#define MAX_LINES 8

static volatile sig_atomic_t interrupted;

static void on_signal(int sig)
{
	(void)sig;
	interrupted = 1;
}

/* What we last asked for, which GET_VALUES cannot tell us apart: a line reading 1 may be
 * driven high or an input sitting high.
 */
enum state { ST_INPUT, ST_LOW, ST_HIGH };

static const char *state_name(enum state s)
{
	const char *name = "?";

	switch (s) {
		case ST_INPUT: {
			name = "z (input)";
		} break;

		case ST_LOW: {
			name = "lo";
		} break;

		case ST_HIGH: {
			name = "hi";
		} break;
	}

	return name;
}

/* The whole token has to be a line number. atoi() returns 0 for "red" and for a typo, and
 * strtoul() alone accepts a "12abc" prefix, so either would select line 0 - a line the
 * operator never named, on a chip where a neighbouring line may be a reset or a rail.
 */
static int parse_line(const char *tok, unsigned int *out)
{
	unsigned long v;
	char *end;

	if (tok[0] == '-')
		return -1;

	errno = 0;
	v = strtoul(tok, &end, 0);
	if (errno != 0 || end == tok || *end != '\0' || v > UINT_MAX)
		return -1;

	*out = (unsigned int)v;
	return 0;
}

static int open_chip(const char *want)
{
	char path[40];

	for (int i = 0; i < 16; i++) {
		struct gpiochip_info ci;
		int c;

		snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
		c = open(path, O_RDWR);
		if (c < 0)
			continue;

		memset(&ci, 0, sizeof(ci));
		if (ioctl(c, GPIO_GET_CHIPINFO_IOCTL, &ci) == 0 && strcmp(ci.label, want) == 0)
			return c;

		close(c);
	}

	return -1;
}

/* Reconfigure an already-granted request. INPUT releases the drive (high-Z); LOW/HIGH drive
 * the pad, with the level carried as an OUTPUT_VALUES attribute so the direction change and
 * the value land in one ioctl rather than driving a stale level first.
 */
static int set_state(int lfd, enum state s)
{
	struct gpio_v2_line_config cfg;

	memset(&cfg, 0, sizeof(cfg));

	if (s == ST_INPUT) {
		cfg.flags = GPIO_V2_LINE_FLAG_INPUT;
	} else {
		cfg.flags = GPIO_V2_LINE_FLAG_OUTPUT;
		cfg.num_attrs = 1;
		cfg.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
		cfg.attrs[0].attr.values = (s == ST_HIGH) ? 1 : 0;
		cfg.attrs[0].mask = 1;
	}

	return ioctl(lfd, GPIO_V2_LINE_SET_CONFIG_IOCTL, &cfg);
}

int main(int argc, char **argv)
{
	const char *chip_label = argc > 1 ? argv[1] : "ar-gpio0";
	unsigned int line[MAX_LINES];
	enum state state[MAX_LINES];
	int lfd[MAX_LINES];
	int count = 0;
	struct sigaction sa;
	char buf[128];
	int i, fd;

	memset(&sa, 0, sizeof(sa));

	if (argc > 2) {
		if (argc - 2 > MAX_LINES) {
			fprintf(stderr, "at most %d lines per session\n", MAX_LINES);
			return 1;
		}

		for (i = 2; i < argc; i++) {
			if (parse_line(argv[i], &line[count]) < 0) {
				fprintf(stderr, "bad line number '%s'\n", argv[i]);
				return 1;
			}

			count++;
		}
	} else {
		/* The air unit's LED pair. */
		line[count++] = 0;
		line[count++] = 1;
	}

	fd = open_chip(chip_label);
	if (fd < 0) {
		fprintf(stderr, "chip '%s' not found\n", chip_label);
		return 1;
	}

	for (i = 0; i < count; i++) {
		struct gpio_v2_line_request req;

		memset(&req, 0, sizeof(req));
		req.offsets[0] = line[i];
		req.num_lines = 1;
		strncpy(req.consumer, "gpio_console", sizeof(req.consumer) - 1);
		req.config.flags = GPIO_V2_LINE_FLAG_INPUT;

		if (ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
			fprintf(stderr, "line %u: GET_LINE failed - already claimed?\n", line[i]);
			fprintf(stderr, "if leds-gpio holds it: echo leds > /sys/bus/platform/drivers/leds-gpio/unbind\n");
			return 1;
		}

		lfd[i] = req.fd;
		state[i] = ST_INPUT;
	}

	printf("chip %s, lines held:", chip_label);
	for (i = 0; i < count; i++)
		printf(" %u", line[i]);

	printf("\nall lines start as inputs (high-Z), nothing driven yet.\n\n");
	printf("commands:  <line> lo | hi | z     drive low / drive high / release to input\n");
	printf("           all lo | hi | z        every held line at once\n");
	printf("           s                      show state\n");
	printf("           q                      quit (releases every line back to input)\n\n");

	/* Deliberately without SA_RESTART, which signal() would apply: the blocking fgets()
	 * below has to fail with EINTR so ^C leaves through the same release path as `q`.
	 */
	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	while (!interrupted) {
		unsigned int target = 0;
		char *tok, *arg;
		enum state want;
		int all, hit;

		printf("gpio> ");
		fflush(stdout);

		/* NULL is EOF or the EINTR from a caught signal; both end the session. */
		if (!fgets(buf, sizeof(buf), stdin))
			break;

		/* A signal that arrived while the line was being typed, rather than while
		 * fgets was blocked, must not let one more command through.
		 */
		if (interrupted)
			break;

		tok = strtok(buf, " \t\r\n");
		if (!tok)
			continue;

		if (!strcmp(tok, "q") || !strcmp(tok, "quit"))
			break;

		if (!strcmp(tok, "s") || !strcmp(tok, "status")) {
			for (i = 0; i < count; i++) {
				struct gpio_v2_line_values v;

				memset(&v, 0, sizeof(v));
				v.mask = 1;
				if (ioctl(lfd[i], GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0)
					v.bits = 0;

				printf("  line %u: set %-9s reads %llu\n",
				       line[i], state_name(state[i]), (unsigned long long)v.bits);
			}

			continue;
		}

		arg = strtok(NULL, " \t\r\n");
		if (!arg) {
			printf("  need a value: <line>|all lo|hi|z\n");
			continue;
		}

		if (!strcmp(arg, "lo") || !strcmp(arg, "0")) {
			want = ST_LOW;
		} else if (!strcmp(arg, "hi") || !strcmp(arg, "1")) {
			want = ST_HIGH;
		} else if (!strcmp(arg, "z") || !strcmp(arg, "in")) {
			want = ST_INPUT;
		} else {
			printf("  unknown value '%s' (want lo, hi or z)\n", arg);
			continue;
		}

		all = !strcmp(tok, "all");
		if (!all && parse_line(tok, &target) < 0) {
			printf("  '%s' is not a line number or 'all'\n", tok);
			continue;
		}

		hit = 0;

		for (i = 0; i < count; i++) {
			if (!all && line[i] != target)
				continue;

			hit = 1;
			if (set_state(lfd[i], want) < 0) {
				printf("  line %u: SET_CONFIG failed\n", line[i]);
				continue;
			}

			state[i] = want;
			printf("  line %u -> %s\n", line[i], state_name(want));
		}

		if (!hit)
			printf("  line '%s' is not held by this session\n", tok);
	}

	/* ^C lands mid-prompt, so start a fresh line before the closing message. */
	if (interrupted)
		printf("\ninterrupted\n");

	/* Closing every request returns the lines to inputs, which is where they started. */
	for (i = 0; i < count; i++)
		close(lfd[i]);

	printf("released.\n");
	return 0;
}
