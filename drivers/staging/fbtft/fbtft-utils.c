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
#include <linux/fbtft.h>

#define NB_DEBUG_TIME_VALS  30
#undef X
#define X(a,b,c) b,
const char * fbtft_debug_time_triggers_long_str[] = {FBTFT_DEBUG_TIME_TRIGGERS};
#undef X
#define X(a,b,c) c,
const char * fbtft_debug_time_triggers_short_str[] = {FBTFT_DEBUG_TIME_TRIGGERS};

static ktime_t ts_now = 0;
static u32 idx_debug_time = 0;
typedef struct {
    u32                             delta_us;
    E_FBTFT_DEBUG_TIME_TRIGGERS     trigger;
    int                             index;
} S_FBTFT_TOC;

static S_FBTFT_TOC toc_events[NB_DEBUG_TIME_VALS] = {0};

/* 
Soft Matrix Rotation with only 1 pixel of extra RAM needed
Works only on 2D square matrices
Great CPU performance
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
    Soft matrix transpose  
    (dimensions multiple of 4, 16bits pixels)
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
    Soft inverse transpose
    (dimensions multiple of 4, 16bits pixels)
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
    Soft matrix rotate 90° CW 
    (dimensions multiple of 4, 16bits pixels)
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
    Soft matrix rotate 270° CW
    (dimensions multiple of 4, 16bits pixels)
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

/*  
    Mix 2 source frames and soft matrix rotate 270° CW
    (dimensions multiple of 4, 16bits pixels)
*/
u8* fbtft_rotate_270cw_soft_mix_src(u8* vmem_src1, u8* vmem_src2, u8* vmem_dst, int w, int h, FBTFT_MIX_PONDERATION_E ponderation){
    
    /* Vars */
    int y, x;
    u16 *src1 = (u16*) vmem_src1;
    u16 *src2 = (u16*) vmem_src2;
    u16 *dst = (u16*) vmem_dst;

    switch (ponderation){
    
    default:
    case FBTFT_MIX_2_2:

        /* Main loop */
        for (y=0; y<h; y++){
            for (x=0; x<w; x++){
                dst[ ( (w-1) - x )*h + y ] = Weight1_1(src1[ y*w + x ], src2[ y*w + x ]);
            }
        }
        break;

    case FBTFT_MIX_3_1:

        /* Main loop */
        for (y=0; y<h; y++){
            for (x=0; x<w; x++){
                dst[ ( (w-1) - x )*h + y ] = Weight3_1(src1[ y*w + x ], src2[ y*w + x ]);
            }
        }
        break;

    case FBTFT_MIX_1_3:

        /* Main loop */
        for (y=0; y<h; y++){
            for (x=0; x<w; x++){
                dst[ ( (w-1) - x )*h + y ] = Weight1_3(src1[ y*w + x ], src2[ y*w + x ]);
            }
        }
        break;
    }

    return vmem_dst;
}




/*
    Debug function to restart stopwatch
*/
void __fbtft_time_tic(void){
    
    /* Get delta in us */
    if(ts_now){
        u32 delta_us = ktime_us_delta(ktime_get(), ts_now);
        fbtft_time_toc(TIC, FBTFT_NO_TIME_INDEX, false);
    }

    ts_now = ktime_get();
}

/*
    Debug function to store stopwatch current count
    trigger = event for which time needs to be stored
    index = additional index info, use FBTFT_NO_TIME_INDEX if meant to be left empty
    print_now = display result now
*/
void __fbtft_time_toc(E_FBTFT_DEBUG_TIME_TRIGGERS trigger, int index, bool print_now){
    
    /* Get delta in us */
    u32 delta_us = ktime_us_delta(ktime_get(), ts_now);

    /* Sanity check */
    u32 real_trigger = trigger;
    if(trigger > FBTFT_NB_DEBUG_TIME_TRIGGERS){
        trigger = FBTFT_NB_DEBUG_TIME_TRIGGERS;
    }

    /* Print now */
    if(print_now){
        printk("toc(%d: %s) = %dus\n", real_trigger, fbtft_debug_time_triggers_short_str[trigger], delta_us);
    }

    /* Store trigger infos */
    toc_events[idx_debug_time] = (S_FBTFT_TOC){
        .delta_us = delta_us, 
        .trigger = trigger,
        .index = index
    };

    /* Next index */
    idx_debug_time = (idx_debug_time+1)%NB_DEBUG_TIME_VALS;
}

/*
    Debug function to dump stored times (short trigger names)
*/
void __fbtft_time_dump(bool short_trigger_name){

    /* Vars */
    int i, cnt = 0;
    int idx = idx_debug_time%NB_DEBUG_TIME_VALS;

    /* Get string size */
    for(i=0; i<NB_DEBUG_TIME_VALS; i++){
        int val;

        /* Delta us size */
        val = toc_events[idx].delta_us/10;
        while (val){
            cnt++;
            val /= 10;
        }
        cnt++;
        
        /* Trigger name size */
        if(short_trigger_name){
            cnt += strlen(fbtft_debug_time_triggers_short_str[toc_events[idx].trigger]);
        }
        else{
            cnt += strlen(fbtft_debug_time_triggers_long_str[toc_events[idx].trigger]);   
        }

        /* Index size */
        if(toc_events[idx].index != FBTFT_NO_TIME_INDEX){
            val = toc_events[idx].index/10;
            while (val){
                cnt++;
                val /= 10;
            }
            cnt += (toc_events[idx].index < 0)?1:0;
        }

        /* padding for ", "":"  and extra */
        cnt += 5;

        /* Next idx */ 
        idx = (idx+1)%NB_DEBUG_TIME_VALS;
    }
    //printk("cnt = %d", cnt);

    /* Declare string (in stack) */
    char dump_str[cnt+1]; // +1 for string termination
    dump_str[0] = 0;

    /* Print String */
    for(i=0; i<NB_DEBUG_TIME_VALS; i++){

        /* Concatenate event*/
        if(toc_events[idx].index == FBTFT_NO_TIME_INDEX){
            sprintf(dump_str+strlen(dump_str), "%s:%d, ", 
                    short_trigger_name ? fbtft_debug_time_triggers_short_str[toc_events[idx].trigger] : fbtft_debug_time_triggers_long_str[toc_events[idx].trigger],
                    toc_events[idx].delta_us
            );
        }
        else{
            sprintf(dump_str+strlen(dump_str), "%s_%d:%d, ", 
                    short_trigger_name ? fbtft_debug_time_triggers_short_str[toc_events[idx].trigger] : fbtft_debug_time_triggers_long_str[toc_events[idx].trigger],
                    toc_events[idx].index,
                    toc_events[idx].delta_us
            );
        }

        /* Next idx */ 
        idx = (idx+1)%NB_DEBUG_TIME_VALS;
    }

    /* Print string */ 
    printk(dump_str);
}