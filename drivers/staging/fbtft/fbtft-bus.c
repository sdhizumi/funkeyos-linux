// SPDX-License-Identifier: GPL-2.0
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h> /* usleep_range */
#include <video/mipi_display.h>
#include <linux/fbtft.h>
#include "fb_text.h"

/*****************************************************************************
 *
 *   void (*write_reg)(struct fbtft_par *par, int len, ...);
 *
 *****************************************************************************/

#define define_fbtft_write_reg(func, type, modifier)                          \
void func(struct fbtft_par *par, int len, ...)                                \
{                                                                             \
	va_list args;                                                         \
	int i, ret;                                                           \
	int offset = 0;                                                       \
	type *buf = (type *)par->buf;                                         \
									      \
	if (unlikely(par->debug & DEBUG_WRITE_REGISTER)) {                    \
		va_start(args, len);                                          \
		for (i = 0; i < len; i++) {                                   \
			buf[i] = (type)va_arg(args, unsigned int);            \
		}                                                             \
		va_end(args);                                                 \
		fbtft_par_dbg_hex(DEBUG_WRITE_REGISTER, par, par->info->device, type, buf, len, "%s: ", __func__);   \
	}                                                                     \
									      \
	va_start(args, len);                                                  \
									      \
	if (par->startbyte) {                                                 \
		*(u8 *)par->buf = par->startbyte;                             \
		buf = (type *)(par->buf + 1);                                 \
		offset = 1;                                                   \
	}                                                                     \
									      \
	*buf = modifier((type)va_arg(args, unsigned int));                    \
	ret = fbtft_write_buf_dc(par, par->buf, sizeof(type) + offset, 0);    \
	if (ret < 0)							      \
		goto out;						      \
	len--;                                                                \
									      \
	if (par->startbyte)                                                   \
		*(u8 *)par->buf = par->startbyte | 0x2;                       \
									      \
	if (len) {                                                            \
		i = len;                                                      \
		while (i--)						      \
			*buf++ = modifier((type)va_arg(args, unsigned int));  \
		fbtft_write_buf_dc(par, par->buf,			      \
				   len * (sizeof(type) + offset), 1);	      \
	}                                                                     \
out:									      \
	va_end(args);                                                         \
}                                                                             \
EXPORT_SYMBOL(func);

define_fbtft_write_reg(fbtft_write_reg8_bus8, u8, )
define_fbtft_write_reg(fbtft_write_reg16_bus8, u16, cpu_to_be16)
define_fbtft_write_reg(fbtft_write_reg16_bus16, u16, )

void fbtft_write_reg8_bus9(struct fbtft_par *par, int len, ...)
{
	va_list args;
	int i, ret;
	int pad = 0;
	u16 *buf = (u16 *)par->buf;

	if (unlikely(par->debug & DEBUG_WRITE_REGISTER)) {
		va_start(args, len);
		for (i = 0; i < len; i++)
			*(((u8 *)buf) + i) = (u8)va_arg(args, unsigned int);
		va_end(args);
		fbtft_par_dbg_hex(DEBUG_WRITE_REGISTER, par,
			par->info->device, u8, buf, len, "%s: ", __func__);
	}
	if (len <= 0)
		return;

	if (par->spi && (par->spi->bits_per_word == 8)) {
		/* we're emulating 9-bit, pad start of buffer with no-ops
		 * (assuming here that zero is a no-op)
		 */
		pad = (len % 4) ? 4 - (len % 4) : 0;
		for (i = 0; i < pad; i++)
			*buf++ = 0x000;
	}

	va_start(args, len);
	*buf++ = (u8)va_arg(args, unsigned int);
	i = len - 1;
	while (i--) {
		*buf = (u8)va_arg(args, unsigned int);
		*buf++ |= 0x100; /* dc=1 */
	}
	va_end(args);
	ret = par->fbtftops.write(par, par->buf, (len + pad) * sizeof(u16));
	if (ret < 0) {
		dev_err(par->info->device,
			"write() failed and returned %d\n", ret);
		return;
	}
}
EXPORT_SYMBOL(fbtft_write_reg8_bus9);

static int prev_write_line_start = -1;
static int prev_write_line_end = -1;
static int write_line_start = -1;
static int write_line_end = -1;
static bool lock = false;

int fbtft_start_new_screen_transfer_async(struct fbtft_par *par)
{
	// printk("%s\n", __func__);

	/* Exit, not ready yet */
	if (par->pdata->te_irq_enabled && !par->ready_for_spi_async)
		return -1;

	/* Received TE interrupt, but still handling previous transfer */
	if (lock){
/* Debug TE overflows */
//#define TE_OVERFLOW_DEBUG
#ifdef TE_OVERFLOW_DEBUG
		static ktime_t ts_lock_disp_last_time;
		static int lock_cnt = 0;
		
		lock_cnt++;
		ktime_t ts_now_lock = ktime_get();
		if (!ktime_to_ns(ts_lock_disp_last_time))
			ts_lock_disp_last_time = ts_now_lock;
		if(ts_now_lock<ts_lock_disp_last_time) ts_lock_disp_last_time = ts_now_lock; // overflow

		#define SECS_MAX_PRINT_TE_LOCK	10
		long delta_us_lock = ktime_us_delta(ts_now_lock, ts_lock_disp_last_time);
		if( delta_us_lock > (SECS_MAX_PRINT_TE_LOCK*1000000) ){
			pr_info("%s: Warning, TE too fast (%d interrupts not handled last %ld secs)\n", 
				__func__, lock_cnt, delta_us_lock/1000000);
			ts_lock_disp_last_time = ts_now_lock;
			lock_cnt = 0;
		}
#endif //TE_OVERFLOW_DEBUG
		return -1;
	}
	lock = true;

    /* Freq */
#define SECS_SPI_ASYNC_FREQ		FBTFT_FREQ_UPDATE_SECS
    static ktime_t prev_ts = {0};
    static ktime_t ts_last_dma_transfer = {0};
    static int count = 0;
	ktime_t ts_now = ktime_get();
	// Overflow:
	if(ts_now < ts_last_dma_transfer){
		count = 0;
		prev_ts = ts_now;
	}
	else{
    	count++;
	}
	par->us_between_dma_transfers = (int)ktime_us_delta(ts_now, ts_last_dma_transfer);
	ts_last_dma_transfer = ts_now;
	int delta_us = ktime_us_delta(ts_now, prev_ts);
	if( delta_us > SECS_SPI_ASYNC_FREQ*1000000){
		//par->freq_dma_transfers = count*1000000/delta_us; // floored value
		par->freq_dma_transfers = (2*count*1000000+delta_us) / (2*delta_us); // rounded value
		fbtft_par_dbg(DEBUG_TIME_EACH_UPDATE, par,
			 "Display update%s: fps=%ld\n, par->nb_backbuffers_full=%d", 
			 par->pdata->te_irq_enabled?" (TE)":"",
			 par->freq_dma_transfers, par->nb_backbuffers_full);
//#define DEBUG_SPI_ASYNC_FREQ
#ifdef DEBUG_SPI_ASYNC_FREQ
		printk("Display update%s: fps=%ld, par->nb_backbuffers_full=%d\n", 
			 par->pdata->te_irq_enabled?" (TE)":"",
			 par->freq_dma_transfers, par->nb_backbuffers_full);
#endif //DEBUG_SPI_ASYNC_FREQ
		count = 0;
		prev_ts = ts_now;
	}


	/* Set vmem buf to transfer over SPI */
	fbtft_set_vmem_buf(par);

	/* New line to write */
	write_line_start = par->write_line_start;
	write_line_end = par->write_line_end;
	par->write_line_start = -1;
	par->write_line_end = -1;

	/* Set window for interlacing */
	if (par->interlacing) {
		par->length_data_transfer = par->info->var.yres * 2;
		write_line_start = par->odd_line?1:0;
		write_line_end = write_line_start;
		fbtft_write_cmd_window_line(par);

	} 
	/* Start sending full screen */
	else {

//#define FORCE_RESEND_TRANSFER_CMD 
#ifdef FORCE_RESEND_TRANSFER_CMD
		#warning force send SPI transfer command
		par->must_send_data_transfer_cmd = true;
#endif
		/* Force write full screen */
		write_line_start = 0;
		write_line_end = par->info->var.yres - 1;

		/* Send first CMD to start data transfers */
		if(par->must_send_data_transfer_cmd){
			par->must_send_data_transfer_cmd = false;
			par->length_data_transfer = par->info->var.yres * par->info->fix.line_length;
			fbtft_write_init_cmd_data_transfers(par);
		}
		/* Send data */
		else{
			fbtft_write_vmem16_bus8_async(par, 
				write_line_start * par->info->fix.line_length, 
				par->length_data_transfer);
		}
	}
	return 0;
}
EXPORT_SYMBOL(fbtft_start_new_screen_transfer_async);

static u8 cmd_window_line = MIPI_DCS_SET_PAGE_ADDRESS;
static u8 buf_ylim[4];

static void spi_complete_cmd_window_line(void *arg)
{
	struct fbtft_par *par = (struct fbtft_par *)arg;
	// printk("%s\n", __func__);

	/* Start data write for line info */
	fbtft_write_data_window_line(par);
}

int fbtft_write_cmd_window_line(struct fbtft_par *par){

	int ret = 0;

	//printk("%s\n", __func__);

	/* Resetting to 0 for incoming cmd init data write */
	set_dc(par, 0);

	/* Start sending cmd init data */
	ret = par->fbtftops.write_async(par, &cmd_window_line, 1, spi_complete_cmd_window_line);
	if (ret < 0)
		dev_err(par->info->device,
			"write() failed and returned %d\n", ret);

	return ret;

}
EXPORT_SYMBOL(fbtft_write_cmd_window_line);

static void spi_complete_data_window_line(void *arg)
{
	struct fbtft_par *par = (struct fbtft_par *) arg;
	//printk("%s\n", __func__);

	/* Start sending cmd for real data transfer */
	fbtft_write_init_cmd_data_transfers(par);
}

int fbtft_write_data_window_line(struct fbtft_par *par)
{
	int ret = 0;

	/* Setting new line coordinates */
	buf_ylim[0] = (write_line_start >> 8) & 0xFF;
	buf_ylim[1] = write_line_start & 0xFF;
	buf_ylim[2] = (write_line_end >> 8) & 0xFF;
	buf_ylim[3] = write_line_end & 0xFF;

	//printk("%s, buf[0] = %d, buf[1] = %d, buf[2] = %d, buf[3] = %d\n", __func__, buf[0], buf[1], buf[2], buf[3]);

	/* Resetting to 1 for incoming data */
	set_dc(par, 1);

	/* Start sending window_line data */
	ret = par->fbtftops.write_async(par, buf_ylim, 4, spi_complete_data_window_line);
	if (ret < 0)
		dev_err(par->info->device,
			"write() failed and returned %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(fbtft_write_data_window_line);

static void spi_complete_cmd_init_data_write(void *arg)
{
	struct fbtft_par *par = (struct fbtft_par *) arg;
	//printk("%s\n", __func__);

	fbtft_write_vmem16_bus8_async(par, write_line_start * par->info->fix.line_length, par->length_data_transfer);
}

int fbtft_write_init_cmd_data_transfers(struct fbtft_par *par)
{
	static const u8 init_data_cmd_buf = MIPI_DCS_WRITE_MEMORY_START;
	int ret = 0;

	// printk("%s\n", __func__);

	/* Resetting to 0 for incoming cmd: "init data write" */
	set_dc(par, 0);

	/* Start sending cmd init data */
	ret = par->fbtftops.write_async(par, (u8 *) &init_data_cmd_buf, 1,
					spi_complete_cmd_init_data_write);
	if (ret < 0)
		dev_err(par->info->device, "write() failed and returned %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(fbtft_write_init_cmd_data_transfers);

static void spi_complete_data_write(void *arg)
{
	struct fbtft_par *par = (struct fbtft_par *) arg;
	//printk("%s, par->interlacing=%d, write_line_start=%d\n", __func__, par->interlacing?1:0, write_line_start);

	/* sleep */
	//msleep(1);

	if (par->interlacing) {
		/* Check if last line */
		bool last_line = (par->odd_line && write_line_start >= par->info->var.yres-1) ||
						(!par->odd_line && write_line_start >= par->info->var.yres-2);

		if (last_line) {
			/* Start sending cmd init data */
			par->odd_line = !par->odd_line;
			lock = false;
			if (!par->pdata->te_irq_enabled)
				fbtft_start_new_screen_transfer_async(par);
		} else {
			write_line_start += 2;
			write_line_end = write_line_start;

			/* Setting window for next line */
			fbtft_write_cmd_window_line(par);
		}
	} else {
		lock = false;
		if (!par->pdata->te_irq_enabled)
			fbtft_start_new_screen_transfer_async(par);
	}
}

/*****************************************************************************
 *
 *   int (*write_vmem)(struct fbtft_par *par);
 *
 *****************************************************************************/

/* 16 bit pixel over 8-bit databus */
int fbtft_write_vmem16_bus8_async(struct fbtft_par *par, size_t offset, size_t len)
{
	u16 *vmem16;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s(offset=%zu, len=%zu)\n",
		__func__, offset, len);

	/* DC pin = 1  for data transfers */
	set_dc(par, 1);

	/* FORCED non buffered write here */
	/* since there is only one SPI */
	/* message cached in async mode */
	vmem16 = (u16 *)(par->vmem_ptr + offset);
	return par->fbtftops.write_async(par, (u8 *) vmem16, len, spi_complete_data_write);
}
EXPORT_SYMBOL(fbtft_write_vmem16_bus8_async);

/*****************************************************************************
 *
 *   int (*write_vmem)(struct fbtft_par *par);
 *
 *****************************************************************************/

/* 16 bit pixel over 8-bit databus */
int fbtft_write_vmem16_bus8(struct fbtft_par *par, size_t offset, size_t len)
{
	u16 *vmem16;
	__be16 *txbuf16 = par->txbuf.buf;
	size_t remain;
	size_t to_copy;
	size_t tx_array_size;
	int i;
	int ret = 0;
	size_t startbyte_size = 0;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s(offset=%zu, len=%zu)\n",
		__func__, offset, len);

	remain = len / 2;
	vmem16 = (u16 *)(par->vmem_ptr + offset);

	set_dc(par, 1);

	/* non buffered write */
	if (!par->txbuf.buf)
		return par->fbtftops.write(par, vmem16, len);

	/* buffered write */
	tx_array_size = par->txbuf.len / 2;

	if (par->startbyte) {
		txbuf16 = par->txbuf.buf + 1;
		tx_array_size -= 2;
		*(u8 *)(par->txbuf.buf) = par->startbyte | 0x2;
		startbyte_size = 1;
	}

	while (remain) {
		to_copy = min(tx_array_size, remain);
		dev_dbg(par->info->device, "    to_copy=%zu, remain=%zu\n",
						to_copy, remain - to_copy);

		for (i = 0; i < to_copy; i++)
			txbuf16[i] = cpu_to_be16(vmem16[i]);

		vmem16 = vmem16 + to_copy;
		ret = par->fbtftops.write(par, par->txbuf.buf,
						startbyte_size + to_copy * 2);
		if (ret < 0)
			return ret;
		remain -= to_copy;
	}

	return ret;
}
EXPORT_SYMBOL(fbtft_write_vmem16_bus8);

/* 16 bit pixel over 9-bit SPI bus: dc + high byte, dc + low byte */
int fbtft_write_vmem16_bus9(struct fbtft_par *par, size_t offset, size_t len)
{
	u8 *vmem8;
	u16 *txbuf16 = par->txbuf.buf;
	size_t remain;
	size_t to_copy;
	size_t tx_array_size;
	int i;
	int ret = 0;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s(offset=%zu, len=%zu)\n",
		__func__, offset, len);

	if (!par->txbuf.buf) {
		dev_err(par->info->device, "%s: txbuf.buf is NULL\n", __func__);
		return -1;
	}

	remain = len;
	vmem8 = par->info->screen_buffer + offset;

	tx_array_size = par->txbuf.len / 2;

	while (remain) {
		to_copy = min(tx_array_size, remain);
		dev_dbg(par->info->device, "    to_copy=%zu, remain=%zu\n",
						to_copy, remain - to_copy);

#ifdef __LITTLE_ENDIAN
		for (i = 0; i < to_copy; i += 2) {
			txbuf16[i]     = 0x0100 | vmem8[i + 1];
			txbuf16[i + 1] = 0x0100 | vmem8[i];
		}
#else
		for (i = 0; i < to_copy; i++)
			txbuf16[i]   = 0x0100 | vmem8[i];
#endif
		vmem8 = vmem8 + to_copy;
		ret = par->fbtftops.write(par, par->txbuf.buf, to_copy * 2);
		if (ret < 0)
			return ret;
		remain -= to_copy;
	}

	return ret;
}
EXPORT_SYMBOL(fbtft_write_vmem16_bus9);

int fbtft_write_vmem8_bus8(struct fbtft_par *par, size_t offset, size_t len)
{
	dev_err(par->info->device, "%s: function not implemented\n", __func__);
	return -1;
}
EXPORT_SYMBOL(fbtft_write_vmem8_bus8);

/* 16 bit pixel over 16-bit databus */
int fbtft_write_vmem16_bus16(struct fbtft_par *par, size_t offset, size_t len)
{
	u16 *vmem16;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s(offset=%zu, len=%zu)\n",
		__func__, offset, len);

	vmem16 = (u16 *)(par->info->screen_buffer + offset);

	/* no need for buffered write with 16-bit bus */
	return fbtft_write_buf_dc(par, vmem16, len, 1);
}
EXPORT_SYMBOL(fbtft_write_vmem16_bus16);
