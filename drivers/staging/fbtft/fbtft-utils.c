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

#include "fbtft-utils.h"

/* 
Soft Matrix Rotation with only 1 pixel of extra RAM needed
Works only on 2D square matrices (16bits/pixel)
Great performance on small matrices
*/
void fbtft_rotate_soft(u16 *mat, int size, int rotation)
{
    int i, j;
    u16 temp;
    int N = size;

#define AT(i, j)    ((i) * N + (j))

    if (rotation == 90) {
        /* Rotate screen 90° Clockwise */
        for (i = 0; i < N / 2; i++) {
            for (j = i; j < N - i - 1; j++) {
                temp = mat[AT(i, j)];
                mat[AT(i, j)] = mat[AT(N - 1 - j, i)];
                mat[AT(N - 1 - j, i)] = mat[AT(N - 1 - i, N - 1 - j)];
                mat[AT(N - 1 - i, N - 1 - j)] = mat[AT(j, N - 1 - i)];
                mat[AT(j, N - 1 - i)] = temp;
            }
        }
    } else if (rotation == 270) {
        /* Rotate screen 270° Clockwise */
        for (i = 0; i < N / 2; i++) {
            for (j = i; j < N - i - 1; j++) {
                temp = mat[AT(i, j)];
                mat[AT(i, j)] = mat[AT(j, N - 1 - i)];
                mat[AT(j, N-1-i)] = mat[AT(N - 1 - i, N - 1 - j)];
                mat[AT(N - 1 - i, N - 1 - j)] = mat[AT(N - 1 - j, i)];
                mat[AT(N - 1 - j, i)] = temp;
            }
        }
    }
}

/*  
    Soft matrix transpose (16bits/pixel)
*/
u8* fbtft_transpose_soft(u8* vmem_src, u8* vmem_dst, int w, int h){
    
    /* Vars */
    int y, x;
    u16 *src = (u16*) vmem_src;
    u16 *dst = (u16*) vmem_dst;

    /* Main loop */
    for (y=0; y<h; y++){
        for (x=0; x<w; x++){
            dst[ x*h + y ] = src[ y*w + x ];
        }
    }

    return vmem_dst;
}

/*  
    Soft inverse transpose (16bits/pixel)
*/
u8* fbtft_transpose_inv_soft(u8* vmem_src, u8* vmem_dst, int w, int h){
    
    /* Vars */
    int y, x;
    u16 *src = (u16*) vmem_src;
    u16 *dst = (u16*) vmem_dst;

    /* Main loop */
    for (y=0; y<h; y++){
        for (x=0; x<w; x++){
            dst[ ( (w-1) - x )*h + (h-1-y) ] = src[ y*w + x ];
        }
    }

    return vmem_dst;
}

/*  
    Soft matrix rotate 90° CW (16bits/pixel)
*/
u8* fbtft_rotate_90cw_soft(u8* vmem_src, u8* vmem_dst, int w, int h){
    
    /* Vars */
    int y, x;
    u16 *src = (u16*) vmem_src;
    u16 *dst = (u16*) vmem_dst;

    /* Main loop */
    for (y=0; y<h; y++){
        for (x=0; x<w; x++){
            dst[ x*h + (h-1-y) ] = src[ y*w + x ];
        }
    }

    return vmem_dst;
}

/*  
    Soft matrix rotate 270° CW (16bits/pixel)
*/
u8* fbtft_rotate_270cw_soft(u8* vmem_src, u8* vmem_dst, int w, int h){
    
    /* Vars */
    int y, x;
    u16 *src = (u16*) vmem_src;
    u16 *dst = (u16*) vmem_dst;

    /* Main loop */
    for (y=0; y<h; y++){
        for (x=0; x<w; x++){
            dst[ ( (w-1) - x )*h + y ] = src[ y*w + x ];
        }
    }

    return vmem_dst;
}