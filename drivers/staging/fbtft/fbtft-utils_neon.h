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

#ifndef __LINUX_FBTFT_UTILS_NEON_H
#define __LINUX_FBTFT_UTILS_NEON_H

#include <linux/types.h>

/* 	
	NEON optimized matrix transpose 
 	(dimensions multiple of 4, 16bits pixels)
*/
void fbtft_transpose_neon(u16 *src, u16* dst, int w, int h);

/*  
    NEON optimized matrix transpose inverse
    (dimensions multiple of 4, 16bits pixels)
*/
void fbtft_transpose_inv_neon(u16* src, u16* dst, int w, int h);

/*  
    NEON optimized matrix rotate 90° CW 
    (dimensions multiple of 4, 16bits pixels)
*/
void fbtft_rotate_90cw_neon(u16* src, u16* dst, int w, int h);

/*  
    NEON optimized matrix rotate 270° CW
    (dimensions multiple of 4, 16bits pixels)
*/
void fbtft_rotate_270cw_neon(u16* src, u16* dst, int w, int h);

#endif //__LINUX_FBTFT_UTILS_NEON_H
