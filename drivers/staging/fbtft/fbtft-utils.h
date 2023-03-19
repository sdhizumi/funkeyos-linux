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

// Support math

// Min, max, abs
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ABS(x) (((x) < 0) ? (-x) : (x))

// Division with error correct (16bpp)
#define Half(A) (((A) >> 1) & 0x7BEF)
#define Quarter(A) (((A) >> 2) & 0x39E7)
// Error correction expressions to piece back the lower bits together (16bpp)
#define RestHalf(A) ((A) & 0x0821)
#define RestQuarter(A) ((A) & 0x1863)

// Error correction expressions for quarters of pixels
#define Corr1_3(A, B)     Quarter(RestQuarter(A) + (RestHalf(B) << 1) + RestQuarter(B))
#define Corr3_1(A, B)     Quarter((RestHalf(A) << 1) + RestQuarter(A) + RestQuarter(B))

// Error correction expressions for halves (16bpp)
#define Corr1_1(A, B)     ((A) & (B) & 0x0821)

// Quarters (16bpp)
#define Weight1_3(A, B)   (Quarter(A) + Half(B) + Quarter(B) + Corr1_3(A, B))
#define Weight3_1(A, B)   (Half(A) + Quarter(A) + Quarter(B) + Corr3_1(A, B))

// Halves (16bpp)
#define Weight1_1(A, B)   (Half(A) + Half(B) + Corr1_1(A, B))

// Type of mix ponderation
typedef enum{
    FBTFT_MIX_NONE,
    FBTFT_MIX_3_1,
    FBTFT_MIX_2_2,
    FBTFT_MIX_1_3,
} FBTFT_MIX_PONDERATION_E;

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

/*  
    Mix 2 source frames and soft matrix rotate 270° CW
    (dimensions multiple of 4, 16bits pixels)
*/
u8* fbtft_rotate_270cw_soft_mix_src(u8* vmem_src1, u8* vmem_src2, u8* vmem_dst, int w, int h, FBTFT_MIX_PONDERATION_E ponderation);

#endif //__LINUX_FBTFT_UTILS_H
