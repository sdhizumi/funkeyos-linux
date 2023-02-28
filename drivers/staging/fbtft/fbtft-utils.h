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

#ifndef __LINUX_FBTFT_UTILS_H
#define __LINUX_FBTFT_UTILS_H

#include <linux/types.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/* 
Soft Matrix Rotation with only 1 pixel of extra RAM needed
Works only on 2D square matrices
Great CPU performance
*/
void fbtft_rotate_soft(u16 *mat, int size, int rotation);

/*  
    Soft matrix transpose  
    (dimensions multiple of 4, 16bits pixels)
*/
u8* fbtft_transpose_soft(u8* vmem_src, u8* vmem_dst, int w, int h);

/*  
    Soft inverse transpose
    (dimensions multiple of 4, 16bits pixels)
*/
u8* fbtft_transpose_inv_soft(u8* vmem_src, u8* vmem_dst, int w, int h);

/*  
    Soft matrix rotate 90° CW 
    (dimensions multiple of 4, 16bits pixels)
*/
u8* fbtft_rotate_90cw_soft(u8* vmem_src, u8* vmem_dst, int w, int h);

/*  
    Soft matrix rotate 270° CW
    (dimensions multiple of 4, 16bits pixels)
*/
u8* fbtft_rotate_270cw_soft(u8* vmem_src, u8* vmem_dst, int w, int h);

#endif //__LINUX_FBTFT_UTILS_H
