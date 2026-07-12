/* SPDX-License-Identifier: GPL-2.0 */
/* drm_glyphs.h - shared 8x8 glyph renderer for ARGB4444 overlay buffers.
 * Include once per translation unit that draws overlay text.
 * All symbols are static to avoid linker conflicts when multiple .c files
 * include this header (each tool is a separate binary, so static is fine).
 */
#pragma once
#include <stdint.h>

/* 8x8 bitmap glyphs, MSB = leftmost column, rows top to bottom. */
static const uint8_t FONT[][8] = {
	{0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0}, /* 0 */
	{0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0}, /* 1 */
	{0x3C, 0x66, 0x06, 0x0C, 0x30, 0x60, 0x7E, 0}, /* 2 */
	{0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0}, /* 3 */
	{0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x0C, 0}, /* 4 */
	{0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0}, /* 5 */
	{0x3C, 0x66, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0}, /* 6 */
	{0x7E, 0x66, 0x0C, 0x18, 0x18, 0x18, 0x18, 0}, /* 7 */
	{0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0}, /* 8 */
	{0x3C, 0x66, 0x66, 0x3E, 0x06, 0x66, 0x3C, 0}, /* 9 */
};
static const uint8_t G_SP[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t G_A[8]  = {0x18, 0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0};
static const uint8_t G_E[8]  = {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x7E, 0};
static const uint8_t G_F[8]  = {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0};
static const uint8_t G_M[8]  = {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63, 0};
static const uint8_t G_P[8]  = {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0};
static const uint8_t G_R[8]  = {0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0};
static const uint8_t G_S[8]  = {0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0};

static const uint8_t *glyph(char c)
{
	if (c >= '0' && c <= '9')
		return FONT[c - '0'];
	switch (c) {
	case 'A': return G_A;
	case 'E': return G_E;
	case 'F': return G_F;
	case 'M': return G_M;
	case 'P': return G_P;
	case 'R': return G_R;
	case 'S': return G_S;
	default:  return G_SP;
	}
}

/* Draw a scaled 8x8 glyph into an ARGB4444 buffer.
 * b=buffer base, sp=stride in uint16_t, x/y=top-left, s=pixel scale, col=colour.
 */
static void gl16(uint16_t *b, int sp, int x, int y, int s, uint16_t col, const uint8_t *g)
{
	for (int ry = 0; ry < 8; ry++) {
		for (int rx = 0; rx < 8; rx++) {
			if ((g[ry] >> (7 - rx)) & 1) {
				for (int dy = 0; dy < s; dy++)
					for (int dx = 0; dx < s; dx++)
						b[(y + ry * s + dy) * sp + x + rx * s + dx] = col;
			}
		}
	}
}
