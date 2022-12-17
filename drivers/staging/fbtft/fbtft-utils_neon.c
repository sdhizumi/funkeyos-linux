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



#ifndef __ARM_NEON__
#error You should compile this file with '-mfloat-abi=softfp -mfpu=neon'
#endif

/*
 * Pull in the reference implementations while instructing GCC (through
 * -ftree-vectorize) to attempt to exploit implicit parallelism and emit
 * NEON instructions.
 */
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC optimize "tree-vectorize"
#else
/*
 * While older versions of GCC do not generate incorrect code, they fail to
 * recognize the parallel nature of these functions, and emit plain ARM code,
 * which is known to be slower than the optimized ARM code in asm-arm/xor.h.
 */
#warning This code requires at least version 4.6 of GCC
#endif

#pragma GCC diagnostic ignored "-Wunused-variable"

#include "fbtft-utils_neon.h"
#include <arm_neon.h>


/* 	
	NEON optimized matrix transpose 
 	(dimensions multiple of 4, 16bits pixels)
*/
void fbtft_transpose_neon(u16* src, u16* dst, int w, int h){
	
	/* Vars */
    uint16x4x4_t v_tmp;
    int y, x;

    /* Main loop */
    for (y=0; y<h; y+=4){
        for (x=0; x<w; x+=4){

            /* Neon Load */
            v_tmp.val[0] = vld1_u16(src + (y+0)*w + x );
            v_tmp.val[1] = vld1_u16(src + (y+1)*w + x );
            v_tmp.val[2] = vld1_u16(src + (y+2)*w + x );
            v_tmp.val[3] = vld1_u16(src + (y+3)*w + x );

            /* Neon store (4 interleaved) */
            vst4_lane_u16(dst + (x+0)*h + y, v_tmp, 0);
            vst4_lane_u16(dst + (x+1)*h + y, v_tmp, 1);
            vst4_lane_u16(dst + (x+2)*h + y, v_tmp, 2);
            vst4_lane_u16(dst + (x+3)*h + y, v_tmp, 3);
        }
    }
}