// SPDX-License-Identifier: GPL-2.0
// button_test - read a board's buttons from evdev and print each press/release.
//
// Two boards, two mechanisms, one evdev interface:
//   - goggle: a resistor ladder on ADC channel 0, decoded by the in-kernel adc-keys driver
//     (fed by the artosyn_adc IIO provider); see ../docs/artosyn-adc.md.
//   - air unit: the single bind button on GPIO 42, decoded by gpio-keys-polled; the input
//     device is named "ml-bind-button".
//
// This opens the evdev node and prints every EV_KEY event with its friendly label. On release
// it also prints the hold time, computed from the event timestamps (press -> release), which
// is the same measurement the bind gesture uses (hold <= 2 s = pair). Codes and labels are the
// DT keymaps in devices/betafpv-vr04-goggle/proxima-9311.dts and
// devices/betafpv-vr04-air/proxima-9311-air.dts.
//
//   button_test                    # auto-detect the keys device (else /dev/input/event0)
//   button_test /dev/input/eventN  # explicit evdev node
//
// Press buttons to see them; Ctrl-C to stop.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/input.h>

// linux,code -> friendly label, verbatim from the DT nodes: the goggle's adc-keys ladder plus
// the air unit's gpio-keys-polled bind button (KEY_CONNECT).
struct keymap {
	int code;
	const char *label;
};

static const struct keymap KEYS[] = {
	{ 0xda, "bind" },
	{ 0x49, "bind" },
	{ 0x42, "back" },
	{ 0x4d, "record" },
	{ 0x57, "up" },
	{ 0x53, "down" },
	{ 0x41, "left" },
	{ 0x44, "right" },
	{ 0x45, "enter" },
};

static const char *label_for(int code)
{
	for (size_t i = 0; i < sizeof(KEYS) / sizeof(KEYS[0]); i++) {
		if (KEYS[i].code == code)
			return KEYS[i].label;
	}

	return "?";
}

static const char *action_for(int value)
{
	switch (value) {
	case 0: {
		return "release";
	}

	case 1: {
		return "press";
	}

	case 2: {
		return "repeat";
	}

	default: {
		return "?";
	}
	}
}

// Press timestamps, one slot per key code seen, so a release can report its hold time. The
// ladder reports one key at a time and the air unit has a single button, so a handful of slots
// covers every real case; an overflowing code simply reports no hold time.
struct press {
	int code;
	struct timeval ts;
};

static struct press PRESSES[8];

static void press_record(int code, struct timeval ts)
{
	for (size_t i = 0; i < sizeof(PRESSES) / sizeof(PRESSES[0]); i++) {
		if (PRESSES[i].code == code || PRESSES[i].code == 0) {
			PRESSES[i].code = code;
			PRESSES[i].ts = ts;
			return;
		}
	}
}

// Hold time in ms from the recorded press to this release, or -1 if the press was not seen
// (started before the tool did, or the slots were full).
static long press_elapsed_ms(int code, struct timeval ts)
{
	for (size_t i = 0; i < sizeof(PRESSES) / sizeof(PRESSES[0]); i++) {
		if (PRESSES[i].code != code)
			continue;

		long ms = (ts.tv_sec - PRESSES[i].ts.tv_sec) * 1000
			+ (ts.tv_usec - PRESSES[i].ts.tv_usec) / 1000;

		PRESSES[i].code = 0;
		return ms;
	}

	return -1;
}

// Pick the evdev node: explicit arg wins, else the first /dev/input/event* whose device name
// looks like a keys device (the goggle's adc-keys node or the air unit's "ml-bind-button"),
// else /dev/input/event0.
static void find_device(const char *arg, char *path, size_t plen)
{
	if (arg) {
		snprintf(path, plen, "%s", arg);
		return;
	}

	DIR *d = opendir("/dev/input");

	if (d) {
		struct dirent *e;

		while ((e = readdir(d)) != NULL) {
			if (strncmp(e->d_name, "event", 5) != 0)
				continue;

			char p[272];

			snprintf(p, sizeof(p), "/dev/input/%s", e->d_name);
			int fd = open(p, O_RDONLY);

			if (fd < 0)
				continue;

			char name[128] = "";

			ioctl(fd, EVIOCGNAME(sizeof(name)), name);
			close(fd);
			if (strstr(name, "adc") || strstr(name, "key") || strstr(name, "button")) {
				snprintf(path, plen, "%s", p);
				closedir(d);
				return;
			}
		}
		closedir(d);
	}

	snprintf(path, plen, "/dev/input/event0");
}

int main(int argc, char **argv)
{
	char path[272];

	find_device(argc > 1 ? argv[1] : NULL, path, sizeof(path));

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		perror(path);
		return 1;
	}

	char name[128] = "";

	ioctl(fd, EVIOCGNAME(sizeof(name)), name);
	setbuf(stdout, NULL);
	fprintf(stderr, "button_test: reading %s (\"%s\"); press buttons, Ctrl-C to stop\n", path, name);

	struct input_event ev;

	for (;;) {
		ssize_t n = read(fd, &ev, sizeof(ev));

		if (n != (ssize_t)sizeof(ev)) {
			if (n < 0)
				perror("read");

			return 1;
		}

		if (ev.type != EV_KEY)
			continue;

		if (ev.value == 1)
			press_record(ev.code, ev.time);

		char held[32] = "";

		if (ev.value == 0) {
			long ms = press_elapsed_ms(ev.code, ev.time);

			if (ms >= 0)
				snprintf(held, sizeof(held), "  (held %ld ms)", ms);
		}

		printf("%-7s  code=%d (0x%02x)  %s%s\n",
		       label_for(ev.code), ev.code, ev.code, action_for(ev.value), held);
	}
}
