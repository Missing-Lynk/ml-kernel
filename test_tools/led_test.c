// SPDX-License-Identifier: GPL-2.0
// led_test - drive the status RGB LED over SPI (WS2812 on /dev/spidev32765.0).
//
// The status LED is a 3-pixel WS2812 chain on the DesignWare SPI master, exclusively - the
// display panel uses DSI + GPIO + PWM, not this bus (see ../docs/status-led.md). Each WS2812 data bit is sent as one SPI
// byte at 6.25 MHz: 0x80 = "0" (~0.16 us high pulse), 0xFC = "1" (~0.96 us high pulse). One
// pixel is 24 bits GRB, MSB first -> 24 bytes; the frame is 3 pixels = 72 bytes. The
// inter-transfer CS gap latches the frame (no explicit reset bytes, matching the vendor frames).
//
// NOTE on the "0" symbol: the vendor binary encodes it as 0xC0 (two high SPI bits), but on this
// board's 3-LED chain that high pulse is long enough to be misread as a "1", and the error
// compounds pixel-to-pixel (all-red comes out red/pink/orange down the chain). 0x80 (one high
// SPI bit) reads cleanly on all three pixels across R/G/B/white. See status-led.md.
//
//   led_test                 # rainbow cycle (default), 40 ms/step, until Ctrl-C
//   led_test rainbow [ms]    # rainbow cycle with a custom per-step delay
//   led_test off             # all pixels off
//   led_test RRGGBB          # solid colour (hex), e.g. led_test ff8000 = orange
//
// The spidev node is auto-detected (/dev/spidev*.0); override with SPIDEV=/dev/spidevN.0.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <glob.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define NLED   3
#define FRAME  (NLED * 24)   // 72 bytes
#define T0     0x80          // WS2812 "0" bit as one SPI byte (~0.16 us high; 0xC0 leaks down the chain)
#define T1     0xFC          // WS2812 "1" bit as one SPI byte (~0.96 us high)
#define SPEED  6250000       // 6.25 MHz, per status-led.md

// Encode one pixel (r,g,b) into 24 SPI bytes: WS2812 wire order is G,R,B, MSB first.
static void encode_pixel(uint8_t *out, uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t grb[3] = { g, r, b };

	for (int c = 0; c < 3; c++) {
		for (int bit = 7; bit >= 0; bit--)
			*out++ = ((grb[c] >> bit) & 1) ? T1 : T0;
	}
}

// Build a full 72-byte frame with all NLED pixels set to the same colour.
static void encode_frame(uint8_t *frame, uint8_t r, uint8_t g, uint8_t b)
{
	for (int i = 0; i < NLED; i++)
		encode_pixel(frame + i * 24, r, g, b);
}

// Hue (0..359) -> RGB at full saturation/value, for the rainbow cycle.
static void hue_rgb(int h, uint8_t *r, uint8_t *g, uint8_t *b)
{
	int seg = h / 60;
	int f = h % 60;
	uint8_t q = 255 - 255 * f / 60;
	uint8_t t = 255 * f / 60;

	switch (seg % 6) {
	case 0:
		*r = 255;
		*g = t;
		*b = 0;
		break;
	case 1:
		*r = q;
		*g = 255;
		*b = 0;
		break;
	case 2:
		*r = 0;
		*g = 255;
		*b = t;
		break;
	case 3:
		*r = 0;
		*g = q;
		*b = 255;
		break;
	case 4:
		*r = t;
		*g = 0;
		*b = 255;
		break;
	default:
		*r = 255;
		*g = 0;
		*b = q;
		break;
	}
}

static int spi_send(int fd, const uint8_t *frame)
{
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)frame,
		.len = FRAME,
		.speed_hz = SPEED,
		.bits_per_word = 8,
	};

	return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}

static const char *find_spidev(void)
{
	static char buf[64];
	const char *env = getenv("SPIDEV");

	if (env)
		return env;

	glob_t g;

	if (glob("/dev/spidev*.0", 0, NULL, &g) == 0 && g.gl_pathc > 0) {
		snprintf(buf, sizeof(buf), "%s", g.gl_pathv[0]);
		globfree(&g);

		return buf;
	}

	return "/dev/spidev32765.0";
}

int main(int argc, char **argv)
{
	int rainbow = 0;
	int step_ms = 40;
	uint8_t r = 0;
	uint8_t g = 0;
	uint8_t b = 0;

	if (argc < 2 || !strcmp(argv[1], "rainbow")) {
		rainbow = 1;
		if (argc > 2)
			step_ms = atoi(argv[2]);
	} else if (!strcmp(argv[1], "off")) {
		r = g = b = 0;
	} else {
		uint32_t rgb = strtoul(argv[1], 0, 16);

		r = rgb >> 16;
		g = rgb >> 8;
		b = rgb;
	}

	const char *dev = find_spidev();
	int fd = open(dev, O_RDWR | O_CLOEXEC);

	if (fd < 0) {
		perror(dev);
		return 1;
	}

	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = SPEED;

	ioctl(fd, SPI_IOC_WR_MODE, &mode);
	ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	fprintf(stderr, "led_test: %s @ %d Hz, %d pixels\n", dev, SPEED, NLED);

	uint8_t frame[FRAME];

	if (!rainbow) {
		encode_frame(frame, r, g, b);
		if (spi_send(fd, frame) < 0) {
			perror("SPI_IOC_MESSAGE");
			return 1;
		}

		return 0;
	}

	setbuf(stdout, NULL);
	printf("rainbow cycle (%d ms/step); Ctrl-C to stop\n", step_ms);
	for (int h = 0; ; h = (h + 6) % 360) {
		hue_rgb(h, &r, &g, &b);
		encode_frame(frame, r, g, b);
		if (spi_send(fd, frame) < 0) {
			perror("SPI_IOC_MESSAGE");
			return 1;
		}
		usleep(step_ms * 1000);
	}
}
