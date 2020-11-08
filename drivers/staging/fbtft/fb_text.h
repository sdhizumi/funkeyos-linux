/*
 * Copyright (C) 2020 Vincent Buso
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_FBTFT_TEXT_H
#define __LINUX_FBTFT_TEXT_H

#include <linux/fbtft.h>

#define MONACO_WIDTH 5
#define MONACO_HEIGHT 8
#define MONACO_BYTES_PER_CHAR (MONACO_WIDTH * MONACO_HEIGHT / 8)

#define RGB565(r, g, b) (((r & 0x1f) << 11) | ((g & 0x3f) << 5) | (b & 0x1f))

void DrawText(u16 *framebuffer, int framebufferWidth,
              int framebufferStrideBytes, int framebufferHeight,
              const char *text, int x, int y, u16 color, u16 bgColor);
void draw_low_battery(u16 *framebuffer, int framebufferWidth,
		      int framebufferHeight);

/*extern unsigned char fontdata8x8[64*16];
extern unsigned char fontdata6x8[256-32][8];*/

void basic_text_out16_bg(void *fb, int w, int h, int x, int y,
                         unsigned short fg_color, unsigned short bg_color,
                         const char *text);
void basic_text_out16_nf(void *fb, int w, int h, int x, int y,
                         unsigned short fg_color, const char *text);
void basic_text_out16(void *fb, int w, int h, int x, int y,
                      unsigned short fg_color, const char *texto, ...);
void basic_text_out_uyvy_nf(void *fb, int w, int x, int y, const char *text);

#endif //__LINUX_FBTFT_TEXT_H
