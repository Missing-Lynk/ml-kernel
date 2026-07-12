// SPDX-License-Identifier: GPL-2.0
// button_test - read the goggle's front-panel buttons and print each press/release.
//
// The buttons are a resistor ladder on ADC channel 0, decoded by the in-kernel adc-keys
// driver (fed by the artosyn_adc IIO provider) into evdev key events on /dev/input/event0.
// This opens that evdev node and prints every EV_KEY event with its friendly label. Codes and
// labels are the DT keymap in dts/proxima-9311.dts; see ../docs/artosyn-adc.md.
//
//   button_test                    # auto-detect the adc-keys device (else /dev/input/event0)
//   button_test /dev/input/eventN  # explicit evdev node
//
// Press buttons to see them; Ctrl-C to stop.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/input.h>

// adc-keys button ladder: linux,code -> friendly label (verbatim from the DT adc-keys node).
struct keymap {
	int code;
	const char *label;
};

static const struct keymap KEYS[] = {
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

// Pick the evdev node: explicit arg wins, else the first /dev/input/event* whose device name
// looks like the adc-keys device, else /dev/input/event0.
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

		printf("%-7s  code=%d (0x%02x)  %s\n",
		       label_for(ev.code), ev.code, ev.code, action_for(ev.value));
	}
}
