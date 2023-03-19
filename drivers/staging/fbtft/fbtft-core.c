/*
 * Copyright (C) 2013 Noralf Tronnes
 *
 * This driver is inspired by:
 *   st7735fb.c, Copyright (C) 2011, Matt Porter
 *   broadsheetfb.c, Copyright (C) 2008, Jaya Kumar
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>
#include <linux/hrtimer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

/* To support deferred IO */
#include <linux/rmap.h>
#include <linux/pagemap.h>

#include <linux/fbtft.h>
#include "fb_text.h"
#include "internal.h"

#ifdef __ARM_NEON__
#undef __ARM_NEON__
#warning fbtft-core should not be compiled with -D __ARM_NEON__
#endif
#include <asm/neon.h>
#include "fbtft-utils_neon.h"
#include "fbtft-utils.h"

static unsigned long debug;
module_param(debug, ulong, 0000);
MODULE_PARM_DESC(debug, "override device debug level");

int fbtft_write_buf_dc(struct fbtft_par *par, void *buf, size_t len, int dc)
{
	int ret;

	set_dc(par, dc);

	ret = par->fbtftops.write(par, buf, len);
	if (ret < 0)
		dev_err(par->info->device,
			"write() failed and returned %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(fbtft_write_buf_dc);

void fbtft_dbg_hex(const struct device *dev, int groupsize,
		   void *buf, size_t len, const char *fmt, ...)
{
	va_list args;
	static char textbuf[512];
	char *text = textbuf;
	size_t text_len;

	va_start(args, fmt);
	text_len = vscnprintf(text, sizeof(textbuf), fmt, args);
	va_end(args);

	hex_dump_to_buffer(buf, len, 32, groupsize, text + text_len,
			   512 - text_len, false);

	if (len > 32)
		dev_info(dev, "%s ...\n", text);
	else
		dev_info(dev, "%s\n", text);
}
EXPORT_SYMBOL(fbtft_dbg_hex);

static unsigned long fbtft_request_gpios_match(struct fbtft_par *par,
					       const struct fbtft_gpio *gpio)
{
	int ret;
	unsigned int val;

	fbtft_par_dbg(DEBUG_REQUEST_GPIOS_MATCH, par, "%s('%s')\n",
		      __func__, gpio->name);

	if (strcasecmp(gpio->name, "reset") == 0) {
		par->gpio.reset = gpio->gpio;
		return GPIOF_OUT_INIT_HIGH;
	} else if (strcasecmp(gpio->name, "dc") == 0) {
		par->gpio.dc = gpio->gpio;
		return GPIOF_OUT_INIT_LOW;
	} else if (strcasecmp(gpio->name, "cs") == 0) {
		par->gpio.cs = gpio->gpio;
		return GPIOF_OUT_INIT_HIGH;
	} else if (strcasecmp(gpio->name, "wr") == 0) {
		par->gpio.wr = gpio->gpio;
		return GPIOF_OUT_INIT_HIGH;
	} else if (strcasecmp(gpio->name, "rd") == 0) {
		par->gpio.rd = gpio->gpio;
		return GPIOF_OUT_INIT_HIGH;
	} else if (strcasecmp(gpio->name, "latch") == 0) {
		par->gpio.latch = gpio->gpio;
		return GPIOF_OUT_INIT_LOW;
	} else if (gpio->name[0] == 'd' && gpio->name[1] == 'b') {
		ret = kstrtouint(&gpio->name[2], 10, &val);
		if (ret == 0 && val < 16) {
			par->gpio.db[val] = gpio->gpio;
			return GPIOF_OUT_INIT_LOW;
		}
	} else if (strcasecmp(gpio->name, "led") == 0) {
		par->gpio.led[0] = gpio->gpio;
		return GPIOF_OUT_INIT_LOW;
	} else if (strcasecmp(gpio->name, "led_") == 0) {
		par->gpio.led[0] = gpio->gpio;
		return GPIOF_OUT_INIT_HIGH;
	}

	return FBTFT_GPIO_NO_MATCH;
}

static int fbtft_request_gpios(struct fbtft_par *par)
{
	struct fbtft_platform_data *pdata = par->pdata;
	const struct fbtft_gpio *gpio;
	unsigned long flags;
	int ret;

	if (!(pdata && pdata->gpios))
		return 0;

	gpio = pdata->gpios;
	while (gpio->name[0]) {
		flags = FBTFT_GPIO_NO_MATCH;
		/* if driver provides match function, try it first,
		 * if no match use our own
		 */
		if (par->fbtftops.request_gpios_match)
			flags = par->fbtftops.request_gpios_match(par, gpio);
		if (flags == FBTFT_GPIO_NO_MATCH)
			flags = fbtft_request_gpios_match(par, gpio);
		if (flags != FBTFT_GPIO_NO_MATCH) {
			ret = devm_gpio_request_one(par->info->device,
					gpio->gpio, flags,
					par->info->device->driver->name);
			if (ret < 0) {
				dev_err(par->info->device,
					"%s: gpio_request_one('%s'=%d) failed with %d\n",
					__func__, gpio->name,
					gpio->gpio, ret);
				return ret;
			}
			fbtft_par_dbg(DEBUG_REQUEST_GPIOS, par,
				      "%s: '%s' = GPIO%d\n",
				      __func__, gpio->name, gpio->gpio);
		}
		gpio++;
	}

	return 0;
}

#ifdef CONFIG_OF
static int fbtft_request_one_gpio(struct fbtft_par *par,
				  const char *name, int index, int *gpiop)
{
	struct device *dev = par->info->device;
	struct device_node *node = dev->of_node;
	int gpio, flags, ret = 0;
	enum of_gpio_flags of_flags;

	if (of_find_property(node, name, NULL)) {
		gpio = of_get_named_gpio_flags(node, name, index, &of_flags);
		if (gpio == -ENOENT)
			return 0;
		if (gpio == -EPROBE_DEFER)
			return gpio;
		if (gpio < 0) {
			dev_err(dev,
				"failed to get '%s' from DT\n", name);
			return gpio;
		}

		/* active low translates to initially low */
		flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW :
							GPIOF_OUT_INIT_HIGH;
		ret = devm_gpio_request_one(dev, gpio, flags,
					    dev->driver->name);
		if (ret) {
			dev_err(dev,
				"gpio_request_one('%s'=%d) failed with %d\n",
				name, gpio, ret);
			return ret;
		}
		if (gpiop)
			*gpiop = gpio;
		fbtft_par_dbg(DEBUG_REQUEST_GPIOS, par, "%s: '%s' = GPIO%d\n",
			      __func__, name, gpio);
	}

	return ret;
}

static int fbtft_request_gpios_dt(struct fbtft_par *par)
{
	int i;
	int ret;

	if (!par->info->device->of_node)
		return -EINVAL;

	ret = fbtft_request_one_gpio(par, "reset-gpios", 0, &par->gpio.reset);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "dc-gpios", 0, &par->gpio.dc);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "rd-gpios", 0, &par->gpio.rd);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "wr-gpios", 0, &par->gpio.wr);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "cs-gpios", 0, &par->gpio.cs);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "latch-gpios", 0, &par->gpio.latch);
	if (ret)
		return ret;
	for (i = 0; i < 16; i++) {
		ret = fbtft_request_one_gpio(par, "db-gpios", i,
					     &par->gpio.db[i]);
		if (ret)
			return ret;
		ret = fbtft_request_one_gpio(par, "led-gpios", i,
					     &par->gpio.led[i]);
		if (ret)
			return ret;
		ret = fbtft_request_one_gpio(par, "aux-gpios", i,
					     &par->gpio.aux[i]);
		if (ret)
			return ret;
	}

	return 0;
}
#endif

#ifdef CONFIG_FB_BACKLIGHT
static int fbtft_backlight_update_status(struct backlight_device *bd)
{
	struct fbtft_par *par = bl_get_data(bd);
	bool polarity = !!(bd->props.state & BL_CORE_DRIVER1);

	fbtft_par_dbg(DEBUG_BACKLIGHT, par,
		"%s: polarity=%d, power=%d, fb_blank=%d\n",
		__func__, polarity, bd->props.power, bd->props.fb_blank);

	if ((bd->props.power == FB_BLANK_UNBLANK) &&
	    (bd->props.fb_blank == FB_BLANK_UNBLANK))
		gpio_set_value(par->gpio.led[0], polarity);
	else
		gpio_set_value(par->gpio.led[0], !polarity);

	return 0;
}

static int fbtft_backlight_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

void fbtft_unregister_backlight(struct fbtft_par *par)
{
	if (par->info->bl_dev) {
		par->info->bl_dev->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(par->info->bl_dev);
		backlight_device_unregister(par->info->bl_dev);
		par->info->bl_dev = NULL;
	}
}

static const struct backlight_ops fbtft_bl_ops = {
	.get_brightness	= fbtft_backlight_get_brightness,
	.update_status	= fbtft_backlight_update_status,
};

void fbtft_register_backlight(struct fbtft_par *par)
{
	struct backlight_device *bd;
	struct backlight_properties bl_props = { 0, };

	if (par->gpio.led[0] == -1) {
		fbtft_par_dbg(DEBUG_BACKLIGHT, par,
			      "%s(): led pin not set, exiting.\n", __func__);
		return;
	}

	bl_props.type = BACKLIGHT_RAW;
	/* Assume backlight is off, get polarity from current state of pin */
	bl_props.power = FB_BLANK_POWERDOWN;
	if (!gpio_get_value(par->gpio.led[0]))
		bl_props.state |= BL_CORE_DRIVER1;

	bd = backlight_device_register(dev_driver_string(par->info->device),
				       par->info->device, par,
				       &fbtft_bl_ops, &bl_props);
	if (IS_ERR(bd)) {
		dev_err(par->info->device,
			"cannot register backlight device (%ld)\n",
			PTR_ERR(bd));
		return;
	}
	par->info->bl_dev = bd;

	if (!par->fbtftops.unregister_backlight)
		par->fbtftops.unregister_backlight = fbtft_unregister_backlight;
}
#else
void fbtft_register_backlight(struct fbtft_par *par) { };
void fbtft_unregister_backlight(struct fbtft_par *par) { };
#endif
EXPORT_SYMBOL(fbtft_register_backlight);
EXPORT_SYMBOL(fbtft_unregister_backlight);

static void fbtft_set_addr_win(struct fbtft_par *par, int xs, int ys, int xe,
			       int ye)
{
	static int prev_xs = -1;
	static int prev_ys = -1;
	static int prev_xe = -1;
	static int prev_ye = -1;

	/* Check if values need to be sent */
	if (prev_xs != xs || prev_xe != xe) {

	  /* Save prev bounding box values */
		prev_xs = xs;
		prev_xe = xe;

		/* Set new bounding Box */
		write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
			  (xs >> 8) & 0xFF, xs & 0xFF, (xe >> 8) & 0xFF, xe & 0xFF);
	}

	// Check if values need to be sent
	if (prev_ys != ys || prev_ye != ye) {

		// Save prev bounding box values
		prev_ys = ys;
		prev_ye = ye;

		/* Set new bounding Box */
		write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
			  (ys >> 8) & 0xFF, ys & 0xFF, (ye >> 8) & 0xFF, ye & 0xFF);
	}
}

static void fbtft_reset(struct fbtft_par *par)
{
	if (par->gpio.reset == -1)
		return;
	fbtft_par_dbg(DEBUG_RESET, par, "%s()\n", __func__);
	gpio_set_value_cansleep(par->gpio.reset, 0);
	usleep_range(20, 40);
	gpio_set_value_cansleep(par->gpio.reset, 1);
	msleep(120);
}

static void fbtft_update_display(struct fbtft_par *par, unsigned int start_line,
				 unsigned int end_line)
{
	size_t offset, len;
	ktime_t ts_start, ts_end;
	long fps, throughput, write_time;
	bool timeit = false;
	int ret = 0;

	if (unlikely(par->debug & (DEBUG_TIME_FIRST_UPDATE |
			DEBUG_TIME_EACH_UPDATE))) {
		if ((par->debug & DEBUG_TIME_EACH_UPDATE) ||
		    ((par->debug & DEBUG_TIME_FIRST_UPDATE) &&
		    !par->first_update_done)) {
			ts_start = ktime_get();
			timeit = true;
		}
	}

	/* Sanity checks */
	if (start_line > end_line) {

		/* Special case: no update needed */
		if (start_line == (par->info->var.yres - 1) || end_line == 0)
			return;
		dev_warn(par->info->device,
			 "%s: start_line=%u is larger than end_line=%u. Shouldn't happen, will do full display update\n",
			 __func__, start_line, end_line);
		start_line = 0;
		end_line = par->info->var.yres - 1;
	}
	if (start_line > par->info->var.yres - 1 ||
	    end_line > par->info->var.yres - 1) {
		dev_warn(par->info->device,
			"%s: start_line=%u or end_line=%u is larger than max=%d. Shouldn't happen, will do full display update\n",
			 __func__, start_line,
			 end_line, par->info->var.yres - 1);
		start_line = 0;
		end_line = par->info->var.yres - 1;
	}

	/* Reset write Window and init write cmd */
	if (par->fbtftops.set_addr_win) {

		fbtft_par_dbg(DEBUG_UPDATE_DISPLAY, par, "%s(start_line=%u, end_line=%u)\n",
			      __func__, start_line, end_line);
		if (par->pdata->rotate == 90){
			par->fbtftops.set_addr_win(par, par->driver_xres-par->info->var.xres, start_line,
				par->driver_xres - 1, end_line);
		}
		else if (par->pdata->rotate == 180){
			par->fbtftops.set_addr_win(par, 0, par->driver_xres-par->info->var.xres,
				par->info->var.xres - 1, par->driver_xres - 1);	
		}
		else if (par->pdata->rotate_soft == 90 || par->pdata->rotate_soft == 270){
			/* Software rotate only simulates hardware rotation, 
				so the var.xres and var.yres were reversed but 
				the real display xres and yres have not changed 
				Btw, fullscreen here.
			*/
			par->fbtftops.set_addr_win(par, 0, 0,
				par->info->var.yres - 1, par->info->var.xres - 1);
		}
		else{
			par->fbtftops.set_addr_win(par, 0, start_line,
				par->info->var.xres - 1, end_line);
		}
	}

	/* Send cmd to start transfer */
	write_reg(par, MIPI_DCS_WRITE_MEMORY_START);

	/* Send data over SPI */
	offset = start_line * par->info->fix.line_length;
	len = (end_line - start_line + 1) * par->info->fix.line_length;
	ret = par->fbtftops.write_vmem(par, offset, len);
	if (ret < 0)
		dev_err(par->info->device,
			"%s: write_vmem failed to update display buffer\n",
			__func__);

	if (unlikely(timeit)) {
		static int nb_fps_values = 0;
		ts_end = ktime_get();
		if (!ktime_to_ns(par->update_time))
			par->update_time = ts_start;

		fps = ktime_us_delta(ts_start, par->update_time);
		par->update_time = ts_start;
		fps = fps ? 1000000 / fps : 0;

		write_time = ktime_us_delta(ts_end, ts_start);
		throughput = write_time ? (len * 1000) / write_time : 0;
		throughput = throughput * 1000 / 1024;

		if (fps) {
			par->avg_fps += fps;
			nb_fps_values++;

			if (nb_fps_values == 120) {
				dev_info(par->info->device,
					 "Display update: %ld kB/s, write_time: %ldus, fps=%ld\n",
					 throughput, write_time, par->avg_fps/nb_fps_values);
				par->avg_fps = 0;
				nb_fps_values = 0;
			}
		}

		par->first_update_done = true;
	}
}

static void fbtft_mkdirty(struct fb_info *info, int y, int height)
{
	struct fbtft_par *par = info->par;
	struct fb_deferred_io *fbdefio = info->fbdefio;

	/* This disables fbtft's defered io, useful in spi_async mode or
	if any other driver handles screens updates instead of fbtft */
	if (par->spi_async_mode)
		return;

	/* special case, needed ? */
	if (y == -1) {
		y = 0;
		height = info->var.yres - 1;
	}

	/* Mark display lines/area as dirty */
	spin_lock(&par->dirty_lock);
	if (y < par->dirty_lines_start)
		par->dirty_lines_start = y;
	if (y + height - 1 > par->dirty_lines_end)
		par->dirty_lines_end = y + height - 1;
	spin_unlock(&par->dirty_lock);

	/* Schedule deferred_io to update display (no-op if already on queue)*/
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static bool fbtft_need_hid(struct fbtft_par *par)
{
	return (par->notification[0] || par->low_battery);
}


static u8 *fbtft_vmem_add_hid(struct fbtft_par *par, u8* vmem_src, u8* vmem_dst)
{
	/* Exit if nothing to display */
	if (!fbtft_need_hid(par)){
		return vmem_src;
	}

	/* Vars */
	int x_notif = 0;
	int y_notif = 0;

	/* Bypass */
//#define FORCE_POSTPROCESS_COPY
#ifdef FORCE_POSTPROCESS_COPY
	direct = false;
#endif	//FORCE_POSTPROCESS_COPY

	/* Vmem_src */
	if(!vmem_src){
		vmem_src = par->vmem_ptr;
	}
	/* Vmem_dst */
	if(!vmem_dst){
		vmem_dst = par->vmem_postprocess_hid;
	}

	/* Copy to post process buffer before post process */
	if (vmem_dst != vmem_src) {

		/* Copy buffer before post processing
		Since the HID for battery, notifs... 
		is added on top of the real framebufffer, 
		we must not write directly inside the real 
		framebuffer or it might never be cleared.
		*/
		par->write_line_start = 0;
		par->write_line_end = par->info->var.yres - 1;
		memcpy(vmem_dst, vmem_src,
			par->info->var.yres * par->info->fix.line_length);
	}

	/* Notifications */
	if (par->notification[0]) {
		x_notif = 0;
		y_notif = 0;
		basic_text_out16_bg((u16 *)vmem_dst, par->info->var.xres, par->info->var.yres,
			x_notif, y_notif, RGB565(255, 255, 255), RGB565(0, 0, 0), par->notification);

		if (y_notif < par->write_line_start)
			par->write_line_start = y_notif;
		if (y_notif + MONACO_HEIGHT > par->write_line_end)
			par->write_line_end = y_notif + MONACO_HEIGHT;
	}

	/* Low battery icon */
	if (par->low_battery) {
		draw_low_battery((u16 *)vmem_dst, par->info->var.xres, par->info->var.yres);
	}

	return vmem_dst;
}

#ifdef FBTFT_USE_DELAYED_WORKER_FOR_BH
/*
	Post process function (rotation/HID) to be called by timer 
	for Bottom-half processing of TE interrupt
*/
void fbtft_worker_postprocess_function(struct work_struct *work){
	/* Vars */
	struct delayed_work *delayed_worker = container_of(work, struct delayed_work, work);
	struct delayed_work_with_data *delayed_worker_with_data = container_of(delayed_worker, struct delayed_work_with_data, delayed_worker);
	struct fbtft_par *par = (struct fbtft_par *) delayed_worker_with_data->data_ptr;
	FBTFT_MIX_PONDERATION_E mix_ponderation = FBTFT_MIX_NONE;
	u8* vmem_src1; u8* vmem_src2;
	static int latest_mixed_framebuffer_idx;
	int vmem_oldest_full_framebuffer_idx;

	/* Debug */
	//fbtft_time_toc(FBTFT_DELAYED_WORK_STARTED, FBTFT_NO_TIME_INDEX, false);

	/* Find buffer to process */
	if(par->nb_framebuffers_full > 0){

		/* Get oldest filled buffer */
		vmem_oldest_full_framebuffer_idx = par->last_full_framebuffer_idx + 1 - par->nb_framebuffers_full;
		if(vmem_oldest_full_framebuffer_idx < 0) {
			vmem_oldest_full_framebuffer_idx = FBTFT_NB_FRAMEBUFFERS+vmem_oldest_full_framebuffer_idx;
		}

	    /* Debug */
	    fbtft_time_toc(SET_POSTPROCESS_BUF, vmem_oldest_full_framebuffer_idx, false);

		/* Debug */
		//printk("TE - nb_fb = %d, using idx %d", par->nb_framebuffers_full, vmem_oldest_full_framebuffer_idx);
		par->nb_framebuffers_full--;
	}
	else{

		/* Debug */
		//printk("!TE - nb_fb = %d, using idx %d", par->nb_framebuffers_full, par->last_full_framebuffer_idx);

	    /* Debug */
	    fbtft_time_toc(SET_POSTPROCESS_BUF, par->last_full_framebuffer_idx, false);

		/* Use last filled framebuffer */
		vmem_oldest_full_framebuffer_idx = par->last_full_framebuffer_idx;
	}
	par->vmem_postprocessed = par->info->screen_buffer + (vmem_oldest_full_framebuffer_idx*par->vmem_size);


#ifdef FBTFT_MIX_SURCHARGED_FRAMES

	/* No mix if HID needed */
	if(fbtft_need_hid(par)){
		par->framebuffer_surcharge_status = FB_SURCHARGE_S_NONE;
	}

	/* 	State machine if framebuffers were filled 
		too fast (ioctl calls > TE calls)
		This mixes the lost frame with the adjacent 
		ones in order not to loose information 
	*/
	if(par->framebuffer_surcharge_status != FB_SURCHARGE_S_NONE){

		/* 	Sanity check 
			Do not merge frame if ioctl call is about to happen. 
			This might be merging a buffer being written (concurrent access + software tearing) 
		*/
		unsigned int us_since_ioctl = (unsigned int)ktime_us_delta(ktime_get(), par->ts_last_ioctl_call);
		if( (par->framebuffer_surcharge_status == FB_SURCHARGE_S_INIT || (par->framebuffer_surcharge_status != FB_SURCHARGE_S_INIT  &&  par->cur_framebuffer_idx == latest_mixed_framebuffer_idx) ) &&
			par->freq_ioctl_calls && 
			us_since_ioctl > 1000000/par->freq_ioctl_calls - 2000
		){
			//printk("next ioctl too close! last: %dus, status: %d", us_since_ioctl, par->framebuffer_surcharge_status);
			par->framebuffer_surcharge_status = FB_SURCHARGE_S_NONE;
		}

		/* Process states */
		switch(par->framebuffer_surcharge_status){

		case FB_SURCHARGE_S_NONE:
			break;

		case FB_SURCHARGE_S_INIT:

			/* 	Set ponderation
				(aaab) in (aaab)(bccc) or (aaab)(bbcc)(cddd)
			*/
			mix_ponderation = FBTFT_MIX_3_1; 

			/* Set vmem sources */
			//printk("m1 :%d", par->cur_framebuffer_idx);
			vmem_src1 = par->info->screen_buffer + (par->cur_framebuffer_idx*par->vmem_size);
			latest_mixed_framebuffer_idx = (par->cur_framebuffer_idx + 1) % FBTFT_NB_FRAMEBUFFERS;
			//printk("m2 :%d", latest_mixed_framebuffer_idx);
			vmem_src2 = par->info->screen_buffer + (latest_mixed_framebuffer_idx*par->vmem_size) ;

			/* Next state */
			par->nb_mixed_frames++;
			if(FBTFT_NB_FRAMEBUFFERS > 3){
				par->framebuffer_surcharge_status = FB_SURCHARGE_S_MID;
			}
			else{
				par->framebuffer_surcharge_status = FB_SURCHARGE_S_LAST;	
			}
			break;

		case FB_SURCHARGE_S_MID:

			/* 	Set ponderation
				(bbcc) in (aaab)(bbcc)(cddd)
			*/
			mix_ponderation = FBTFT_MIX_2_2; 

			/* Set vmem sources */
			//printk("m1 :%d", latest_mixed_framebuffer_idx);
			vmem_src1 = par->info->screen_buffer + (latest_mixed_framebuffer_idx*par->vmem_size);
			latest_mixed_framebuffer_idx = (latest_mixed_framebuffer_idx + 1) % FBTFT_NB_FRAMEBUFFERS;
			//printk("m2 :%d", latest_mixed_framebuffer_idx);
			vmem_src2 = par->info->screen_buffer + (latest_mixed_framebuffer_idx*par->vmem_size) ;
			
			/* Next state */
			par->framebuffer_surcharge_status = FB_SURCHARGE_S_LAST;
			break;

		case FB_SURCHARGE_S_LAST:

			/* 	Set ponderation
				(bccc) in (aaab)(bccc) or (cddd) in (aaab)(bbcc)(cddd)
			*/
			mix_ponderation = FBTFT_MIX_1_3; 

			/* Set vmem sources */
			//printk("m1 :%d", latest_mixed_framebuffer_idx);
			vmem_src1 = par->info->screen_buffer + (latest_mixed_framebuffer_idx*par->vmem_size);
			latest_mixed_framebuffer_idx = (latest_mixed_framebuffer_idx + 1) % FBTFT_NB_FRAMEBUFFERS;
			//printk("m2 :%d", latest_mixed_framebuffer_idx);
			vmem_src2 = par->info->screen_buffer + (latest_mixed_framebuffer_idx*par->vmem_size) ;
			
			/* Next state */
			par->framebuffer_surcharge_status = FB_SURCHARGE_S_NONE;
			break;

		default:
			
			/* Default state */
			par->framebuffer_surcharge_status = FB_SURCHARGE_S_NONE;
			break;
		}
	}
#endif //FBTFT_MIX_SURCHARGED_FRAMES

	/* Postprocess: Print HID */
	par->vmem_postprocessed = fbtft_vmem_add_hid(par, par->vmem_postprocessed, par->vmem_postprocess_hid);

	/* Postprocess: Rotate */
	if (par->pdata->rotate_soft == 270){
#ifdef FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
	#ifdef FBTFT_NEON_OPTIMS
		preempt_disable();
		kernel_neon_begin();
		par->vmem_postprocessed = (u8*) fbtft_transpose_inv_neon((u16*) par->vmem_postprocessed, 
			(u16*)par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
		kernel_neon_end();
		preempt_enable();
	#else //not defined FBTFT_NEON_OPTIMS
		par->vmem_postprocessed = fbtft_transpose_inv_soft( par->vmem_postprocessed, 
			par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
	#endif //FBTFT_NEON_OPTIMS
#else //rotate (not FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE)
	    
	    /* Debug */
	    fbtft_time_toc(ROTATION_STARTED, par->cur_postprocess_buffer_idx, false);

	#ifdef FBTFT_NEON_OPTIMS
		preempt_disable();
		kernel_neon_begin();
		par->vmem_postprocessed = (u8*) fbtft_rotate_270cw_neon((u16*) par->vmem_postprocessed, 
			(u16*)par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
		kernel_neon_end();
		preempt_enable();
	#else //not defined FBTFT_NEON_OPTIMS

		if(mix_ponderation == FBTFT_MIX_NONE){
			//printk("n: %d", vmem_oldest_full_framebuffer_idx);
			par->vmem_postprocessed = fbtft_rotate_270cw_soft( par->vmem_postprocessed, 
				par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
				par->info->var.xres, par->info->var.yres);	
		}
		else{
			par->vmem_postprocessed = fbtft_rotate_270cw_soft_mix_src( 
				vmem_src1, vmem_src2, 
				par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
				par->info->var.xres, par->info->var.yres, mix_ponderation);	
		}
	#endif //FBTFT_NEON_OPTIMS

	    /* Debug */
	    fbtft_time_toc(ROTATION_ENDED, par->cur_postprocess_buffer_idx, false);
#endif //FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
	}
	else if (par->pdata->rotate_soft == 90)
	{
#ifdef FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
	#ifdef FBTFT_NEON_OPTIMS
		preempt_disable();
		kernel_neon_begin();
		par->vmem_postprocessed = (u8*) fbtft_transpose_neon((u16*)par->vmem_postprocessed, 
			(u16*)par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
		kernel_neon_end();
		preempt_enable();
	#else //not defined FBTFT_NEON_OPTIMS
		par->vmem_postprocessed = fbtft_transpose_soft( par->vmem_postprocessed, 
			par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
	#endif //FBTFT_NEON_OPTIMS
#else //rotate (not FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE)
	#ifdef FBTFT_NEON_OPTIMS
		preempt_disable();
		kernel_neon_begin();
		par->vmem_postprocessed = (u8*) fbtft_rotate_90cw_neon((u16*)par->vmem_postprocessed, 
			(u16*)par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
		kernel_neon_end();
		preempt_enable();
	#else //not defined FBTFT_NEON_OPTIMS
		par->vmem_postprocessed = fbtft_rotate_90cw_soft( par->vmem_postprocessed, 
			par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
	#endif //FBTFT_NEON_OPTIMS
#endif //FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
	}
	/* No rotation: direct copy to current backbuffer */
	/*else{
		memcpy(par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx],
			par->vmem_postprocessed, 
			par->info->var.yres * par->info->fix.line_length);
		par->vmem_postprocessed = par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx];
	}*/

	/* Increment postprocess_buffers idx */
	par->last_full_postprocess_buffer_idx = par->cur_postprocess_buffer_idx;
	par->cur_postprocess_buffer_idx = (par->cur_postprocess_buffer_idx+1)%FBTFT_NB_POSTPROCESS_BUFFERS;
	if(par->nb_postprocess_buffers_full < FBTFT_NB_POSTPROCESS_BUFFERS){
		par->nb_postprocess_buffers_full++;
	}
}

#else // FBTFT_USE_DELAYED_WORKER_FOR_BH

/*
	Post process function (rotation/HID) to be called by timer 
	for Bottom-half processing of TE interrupt
*/
void fbtft_timer_postprocess_function(struct timer_list *timer){
	
	/* Vars */
	struct timer_with_data *timer_postprocess = container_of(timer, struct timer_with_data, timer);
	struct fbtft_par *par = (struct fbtft_par *) timer_postprocess->data_ptr;

	/* Debug */
	//fbtft_time_toc(FBTFT_DELAYED_WORK_STARTED, FBTFT_NO_TIME_INDEX, false);

	/* Postprocess: Print HID */
	par->vmem_postprocessed = fbtft_vmem_add_hid(par, par->vmem_ptr_to_postprocess, par->vmem_postprocess_hid);

	/* Postprocess: Rotate */
	if (par->pdata->rotate_soft == 270){
#ifdef FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
		par->vmem_postprocessed = fbtft_transpose_inv_soft(par->vmem_postprocessed, 
			par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
#else //rotate (not FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE)
	    
	    /* Debug */
	    fbtft_time_toc(ROTATION_STARTED, par->cur_postprocess_buffer_idx, false);

		par->vmem_postprocessed = fbtft_rotate_270cw_soft(par->vmem_postprocessed, 
			par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
	    
	    /* Debug */
	    fbtft_time_toc(ROTATION_ENDED, par->cur_postprocess_buffer_idx, false);
#endif //FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
	}
	else if (par->pdata->rotate_soft == 90){
#ifdef FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
		par->vmem_postprocessed = fbtft_transpose_soft(par->vmem_postprocessed, 
			par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
#else //rotate (not FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE)
		par->vmem_postprocessed = fbtft_rotate_90cw_soft(par->vmem_postprocessed, 
			par->vmem_postprocess_buffers[par->cur_postprocess_buffer_idx], 
			par->info->var.xres, par->info->var.yres);
#endif //FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
	}

	/* Increment postprocess_buffers idx */
	par->last_full_postprocess_buffer_idx = par->cur_postprocess_buffer_idx;
	par->cur_postprocess_buffer_idx = (par->cur_postprocess_buffer_idx+1)%FBTFT_NB_POSTPROCESS_BUFFERS;
	if(par->nb_postprocess_buffers_full < FBTFT_NB_POSTPROCESS_BUFFERS){
		par->nb_postprocess_buffers_full++;
	}
}
#endif // FBTFT_USE_DELAYED_WORKER_FOR_BH

/*
	Function to initiate bottom-half post process (not during Top-half TE interrupt)
	if framebuffers are full
 */
void fbtft_trigger_delayed_framebuffer_post_processing(struct fbtft_par *par){

	/* Use current vmem buf and start delayed work to set next vmem ptr */
#ifdef FBTFT_USE_DELAYED_WORKER_FOR_BH
	schedule_delayed_work(&par->delayed_worker_postprocess.delayed_worker, 1); // if HZ=200, this means between 5ms and 10ms later
#else // FBTFT_USE_DELAYED_WORKER_FOR_BH
	mod_timer(&par->timer_postprocess.timer, jiffies+1); // if HZ=200, this means between 5ms and 10ms later
#endif // FBTFT_USE_DELAYED_WORKER_FOR_BH
}

/* 
* Function to get video memory buffer to transfer
*/
u8 *fbtft_get_vmem_buf_to_transfer(struct fbtft_par *par){

	/* Vars */
	u8 *vmem = par->vmem_ptr;

    /* Debug */
    fbtft_time_toc(SET_VIDEO_BUF, FBTFT_NO_TIME_INDEX, false);
	
	/* Backbuffers filled */	
	if(par->nb_postprocess_buffers_full > 0){
		
		/* Get last full screen buffer */
		vmem = par->vmem_postprocessed;

		/* Clear flags */
		par->nb_postprocess_buffers_full=0;
		
	}
	else{ // No backbuffers full (only for the first TE irq)

		/* Get latest filled buffer */
		vmem = par->info->screen_buffer + (par->last_full_framebuffer_idx*par->vmem_size);

		/* Print HID */
		vmem = fbtft_vmem_add_hid(par, vmem, par->vmem_postprocess_hid);

		/* Rotate and copy to unused backbuffer */
		if (par->pdata->rotate_soft == 270){
#ifdef FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
			vmem = fbtft_transpose_inv_soft(vmem, 
				par->vmem_postprocess_buffers[par->last_full_postprocess_buffer_idx], 
				par->info->var.xres, par->info->var.yres);
#else //rotate (not FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE)
			vmem = fbtft_rotate_270cw_soft(vmem, 
				par->vmem_postprocess_buffers[par->last_full_postprocess_buffer_idx], 
				par->info->var.xres, par->info->var.yres);
#endif //FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
		}
		else if (par->pdata->rotate_soft == 90){
#ifdef FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
			vmem = fbtft_transpose_soft(vmem, 
				par->vmem_postprocess_buffers[par->last_full_postprocess_buffer_idx], 
				par->info->var.xres, par->info->var.yres);
#else //rotate (not FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE)
			vmem = fbtft_rotate_90cw_soft(vmem, 
				par->vmem_postprocess_buffers[par->last_full_postprocess_buffer_idx], 
				par->info->var.xres, par->info->var.yres);
#endif //FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
		}
	}

	return vmem;
}


static void fbtft_first_io(struct fb_info *info){
	/*struct fbtft_par *par = info->par;
	printk("f\n");*/
}

static void fbtft_deferred_io(struct fb_info *info, struct list_head *pagelist)
{

	struct fbtft_par *par = info->par;

	/* This disables fbtft's defered io, useful in spi_async mode or
	if any other driver handles screens updates instead of fbtft */
	if (par->spi_async_mode)
		return VM_FAULT_LOCKED;

	unsigned int dirty_lines_start, dirty_lines_end;
	struct page *page;
	unsigned long index;
	unsigned int y_low = 0, y_high = 0;
	int count = 0;
	
	//#define FPS_DEBUG
	#if 0
		long fps;
		static ktime_t update_time;
		static int nb_fps_values = 0;
		static long avg_fps = 0;
		ktime_t ts_now = ktime_get();

		/* First measurement */
		if (!ktime_to_ns(update_time))
				update_time = ts_now;

		fps = ktime_us_delta(ts_now, update_time);
		update_time = ts_now;
		fps = fps ? 1000000 / fps : 0;

		if (fps) {
			avg_fps += fps;
			nb_fps_values++;

			if (nb_fps_values == 200) {
				fbtft_par_dbg(DEBUG_TIME_EACH_UPDATE, par,
					 "fbtft_deferred_io update: fps=%ld\n", avg_fps / nb_fps_values);
				avg_fps = 0;
				nb_fps_values = 0;
			}
		}
	#endif //FPS_DEBUG

	spin_lock(&par->dirty_lock);
	dirty_lines_start = par->dirty_lines_start;
	dirty_lines_end = par->dirty_lines_end;
	/* set display line markers as clean */
	par->dirty_lines_start = par->info->var.yres - 1;
	par->dirty_lines_end = 0;
	spin_unlock(&par->dirty_lock);

	/* Mark display lines as dirty */
	list_for_each_entry(page, pagelist, lru) {
		count++;
		index = page->index << PAGE_SHIFT;
		y_low = index / info->fix.line_length;
		y_high = (index + PAGE_SIZE - 1) / info->fix.line_length;
		dev_dbg(info->device,
			"count=%d, page->index=%lu y_low=%d y_high=%d\n",
			count, page->index, y_low, y_high);
		if (y_high > info->var.yres - 1)
			y_high = info->var.yres - 1;
		if (y_low < dirty_lines_start)
			dirty_lines_start = y_low;
		if (y_high > dirty_lines_end)
			dirty_lines_end = y_high;
	}

	/* Copy buffer */
	par->write_line_start = dirty_lines_start;
	par->write_line_end = dirty_lines_end;

	/* Set vmem buf to transfer over SPI */
	par->vmem_ptr = fbtft_get_vmem_buf_to_transfer(par);

	/* Screen upgrade */
	par->fbtftops.update_display(par, dirty_lines_start, dirty_lines_end);
}

static void fbtft_fb_fillrect(struct fb_info *info,
			      const struct fb_fillrect *rect)
{
	struct fbtft_par *par = info->par;

	dev_dbg(info->dev,
		"%s: dx=%d, dy=%d, width=%d, height=%d\n",
		__func__, rect->dx, rect->dy, rect->width, rect->height);
	sys_fillrect(info, rect);

	par->fbtftops.mkdirty(info, rect->dy, rect->height);
}

static void fbtft_fb_copyarea(struct fb_info *info,
			      const struct fb_copyarea *area)
{
	struct fbtft_par *par = info->par;

	dev_dbg(info->dev,
		"%s: dx=%d, dy=%d, width=%d, height=%d\n",
		__func__,  area->dx, area->dy, area->width, area->height);
	sys_copyarea(info, area);

	par->fbtftops.mkdirty(info, area->dy, area->height);
}

static void fbtft_fb_imageblit(struct fb_info *info,
			       const struct fb_image *image)
{
	struct fbtft_par *par = info->par;

	dev_dbg(info->dev,
		"%s: dx=%d, dy=%d, width=%d, height=%d\n",
		__func__,  image->dx, image->dy, image->width, image->height);
	sys_imageblit(info, image);

	par->fbtftops.mkdirty(info, image->dy, image->height);
}

static ssize_t fbtft_fb_write(struct fb_info *info, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct fbtft_par *par = info->par;
	ssize_t res;

	dev_dbg(info->dev,
		"%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);
	res = fb_sys_write(info, buf, count, ppos);

	/* TODO: only mark changed area update all for now */
	par->fbtftops.mkdirty(info, -1, 0);

	return res;
}

static struct page *fb_deferred_io_page(struct fb_info *info, unsigned long offs)
{
	void *screen_base = (void __force *) info->screen_base;
	struct page *page;

	if (is_vmalloc_addr(screen_base + offs))
		page = vmalloc_to_page(screen_base + offs);
	else
		page = pfn_to_page((info->fix.smem_start + offs) >> PAGE_SHIFT);

	return page;
}

/* This is to find and return the vmalloc-ed fb pages */
static int fb_deferred_io_fault(struct vm_fault *vmf)
{
	unsigned long offset;
	struct page *page;
	struct fb_info *info = vmf->vma->vm_private_data;

	offset = vmf->pgoff << PAGE_SHIFT;
	if (offset >= info->fix.smem_len){
		printk("fault\n");
		return VM_FAULT_SIGBUS;
	}

	page = fb_deferred_io_page(info, offset);
	if (!page){
		printk("no page\n");
		return VM_FAULT_SIGBUS;
	}

	get_page(page);

	if (vmf->vma->vm_file)
		page->mapping = vmf->vma->vm_file->f_mapping;
	else
		printk(KERN_ERR "no mapping available\n");

	BUG_ON(!page->mapping);
	page->index = vmf->pgoff;

	vmf->page = page;
	return 0;
}

/* vm_ops->page_mkwrite handler */
static int fb_deferred_io_mkwrite(struct vm_fault *vmf)
{
	struct page *page = vmf->page;
	struct fb_info *info = vmf->vma->vm_private_data;
	struct fb_deferred_io *fbdefio = info->fbdefio;
	struct page *cur;
	struct fbtft_par *par = info->par;

	/* This disables fbtft's defered io, useful in spi_async mode or
	if any other driver handles screens updates instead of fbtft */
	if (par->spi_async_mode)
		return VM_FAULT_LOCKED;

	/* this is a callback we get when userspace first tries to
	write to the page. we schedule a workqueue. that workqueue
	will eventually mkclean the touched pages and execute the
	deferred framebuffer IO. then if userspace touches a page
	again, we repeat the same scheme */

	file_update_time(vmf->vma->vm_file);

	/* protect against the workqueue changing the page list */
	mutex_lock(&fbdefio->lock);

	/* first write in this cycle, notify the driver */
	if (fbdefio->first_io && list_empty(&fbdefio->pagelist)){
		fbdefio->first_io(info);
	}

	/*
	 * We want the page to remain locked from ->page_mkwrite until
	 * the PTE is marked dirty to avoid page_mkclean() being called
	 * before the PTE is updated, which would leave the page ignored
	 * by defio.
	 * Do this by locking the page here and informing the caller
	 * about it with VM_FAULT_LOCKED.
	 */
	lock_page(page);

	/* we loop through the pagelist before adding in order
	to keep the pagelist sorted */
	list_for_each_entry(cur, &fbdefio->pagelist, lru) {
		/* this check is to catch the case where a new
		process could start writing to the same page
		through a new pte. this new access can cause the
		mkwrite even when the original ps's pte is marked
		writable */
		if (unlikely(cur == page))
			goto page_already_added;
		else if (cur->index > page->index)
			break;
	}

	list_add_tail(&page->lru, &cur->lru);

page_already_added:
	mutex_unlock(&fbdefio->lock);

	/* Update by timers */
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);

	return VM_FAULT_LOCKED;
}

static const struct vm_operations_struct fb_deferred_io_vm_ops = {
	.fault		= fb_deferred_io_fault,
	.page_mkwrite	= fb_deferred_io_mkwrite,
};

static int fbtft_fb_deferred_io_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_ops = &fb_deferred_io_vm_ops;
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	if (!(info->flags & FBINFO_VIRTFB))
		vma->vm_flags |= VM_IO;
	vma->vm_private_data = info;
	return 0;
}

/* from pxafb.c */
static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int fbtft_fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
			      unsigned int blue, unsigned int transp,
			      struct fb_info *info)
{
	unsigned int val;
	int ret = 1;

	dev_dbg(info->dev,
		"%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
		__func__, regno, red, green, blue, transp);

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red,   &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue,  &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;
	}
	return ret;
}

static int fbtft_fb_blank(int blank, struct fb_info *info)
{
	struct fbtft_par *par = info->par;
	int ret = -EINVAL;

	dev_dbg(info->dev, "%s(blank=%d)\n",
		__func__, blank);

	if (!par->fbtftops.blank)
		return ret;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		ret = par->fbtftops.blank(par, true);
		break;
	case FB_BLANK_UNBLANK:
		ret = par->fbtftops.blank(par, false);
		break;
	}
	return ret;
}

/* 
	Called by FBIOPAN_DISPLAY ioctl (by SDL_Flip() in FB_FlipHWSurface)
*/
static int fbtft_fb_pan_display (struct fb_var_screeninfo *var, struct fb_info *info)
{
	/* Vars */ 
	struct fbtft_par *par = info->par;
	ktime_t ts_now;
	int ret = -EINVAL;

	dev_dbg(info->dev, "%s\n", __func__);


	/* Compute frequency of ioctl(FBIOPAN_DISPLAY) calls */
	ts_now = ktime_get();
	if(ts_now < par->ts_last_ioctl_call) par->ts_last_ioctl_call = ts_now; //overflow
	par->us_between_ioctl_calls = (unsigned int)ktime_us_delta(ts_now, par->ts_last_ioctl_call);
	par->ts_last_ioctl_call = ts_now;

#define IOCTL_FREQ_SECS		FBTFT_FREQ_UPDATE_SECS
	static ktime_t ts_earlier_fps = 0;
	static unsigned int count = 0;
	static unsigned int us_between_frames_avg = 0;

	// Overflow:
	if(ts_now < ts_earlier_fps){
		us_between_frames_avg = 0;
		ts_earlier_fps = ts_now; 
		count = 0;
	}
	else{
		count++;
		us_between_frames_avg += (unsigned int) par->us_between_ioctl_calls;
	}
	int delta_us = (int)ktime_us_delta(ts_now, ts_earlier_fps);

	if( delta_us > (IOCTL_FREQ_SECS*1000000) && count ){
		//par->freq_ioctl_calls = count*1000000/delta_us; // floored value
		par->freq_ioctl_calls = (2*count*1000000+delta_us) / (2*delta_us); // rounded value
//#define DEBUG_IOCTL_FREQ
#ifdef DEBUG_IOCTL_FREQ
		printk("freq IOCTL calls %ld/s, avg ms between frames: %ld\n", 
			par->freq_ioctl_calls, us_between_frames_avg/1000/count);
#endif //DEBUG_IOCTL_FREQ
		us_between_frames_avg = 0;
		count = 0;
		ts_earlier_fps = ts_now;
	}


	/* Flip current framebuffer */
	struct fb_fix_screeninfo *fix = &info->fix;
	par->last_full_framebuffer_idx = var->yoffset/fix->ypanstep;
	par->cur_framebuffer_idx = (par->last_full_framebuffer_idx+1)%FBTFT_NB_FRAMEBUFFERS;
	/*printk("last_full_postprocess_buffer_idx=%d, cur_postprocess_buffer_idx=%d\n", 
		par->last_full_postprocess_buffer_idx, par->cur_postprocess_buffer_idx);*/
	if(par->nb_framebuffers_full < FBTFT_NB_FRAMEBUFFERS-1){  // -1 because one framebuffer is never "full" since it's active (next one to be rewritten)
		par->nb_framebuffers_full++;
		
		/* Debug */
		//printk("IOCTL - nb_fb=%d", par->nb_framebuffers_full);
	}
	else if(par->nb_framebuffers_full == FBTFT_NB_FRAMEBUFFERS-1){
		//printk("IOCTL - LF"); // Lost frame
		par->framebuffer_surcharge_status = FB_SURCHARGE_S_INIT;
	}


	return 0;
}

static void fbtft_merge_fbtftops(struct fbtft_ops *dst, struct fbtft_ops *src)
{
	if (src->write)
		dst->write = src->write;
	if (src->write_async)
		dst->write_async = src->write_async;
	if (src->read)
		dst->read = src->read;
	if (src->write_vmem)
		dst->write_vmem = src->write_vmem;
	if (src->write_register)
		dst->write_register = src->write_register;
	if (src->set_addr_win)
		dst->set_addr_win = src->set_addr_win;
	if (src->reset)
		dst->reset = src->reset;
	if (src->mkdirty)
		dst->mkdirty = src->mkdirty;
	if (src->update_display)
		dst->update_display = src->update_display;
	if (src->init_display)
		dst->init_display = src->init_display;
	if (src->blank)
		dst->blank = src->blank;
	if (src->fb_pan_display)
		dst->fb_pan_display = src->fb_pan_display;
	if (src->request_gpios_match)
		dst->request_gpios_match = src->request_gpios_match;
	if (src->request_gpios)
		dst->request_gpios = src->request_gpios;
	if (src->verify_gpios)
		dst->verify_gpios = src->verify_gpios;
	if (src->register_backlight)
		dst->register_backlight = src->register_backlight;
	if (src->unregister_backlight)
		dst->unregister_backlight = src->unregister_backlight;
	if (src->set_var)
		dst->set_var = src->set_var;
	if (src->set_gamma)
		dst->set_gamma = src->set_gamma;
}

/**
 * fbtft_framebuffer_alloc - creates a new frame buffer info structure
 *
 * @display: pointer to structure describing the display
 * @dev: pointer to the device for this fb, this can be NULL
 *
 * Creates a new frame buffer info structure.
 *
 * Also creates and populates the following structures:
 *   info->fbops
 *   info->fbdefio
 *   info->pseudo_palette
 *   par->fbtftops
 *   par->txbuf
 *
 * Returns the new structure, or NULL if an error occurred.
 *
 */
struct fb_info *fbtft_framebuffer_alloc(struct fbtft_display *display,
					struct device *dev,
					struct fbtft_platform_data *pdata)
{
	struct fb_info *info;
	struct fbtft_par *par;
	struct fb_ops *fbops = NULL;
	struct fb_deferred_io *fbdefio = NULL;
	u8 *vmem = NULL;
	u8 *vmem_postprocess_buffers[FBTFT_NB_POSTPROCESS_BUFFERS] = {NULL};
	u8 *vmem_postprocess_hid = NULL;
	void *txbuf = NULL;
	void *buf = NULL;
	unsigned int width, height;
	unsigned int disp_width, disp_height;
	int txbuflen = display->txbuflen;
	unsigned int bpp = display->bpp;
	unsigned int fps = display->fps;
	int vmem_size, i;
	const s16 *init_sequence = display->init_sequence;
	char *gamma = display->gamma;
	u32 *gamma_curves = NULL;

	/* sanity check */
	if (display->gamma_num * display->gamma_len >
			FBTFT_GAMMA_MAX_VALUES_TOTAL) {
		dev_err(dev, "FBTFT_GAMMA_MAX_VALUES_TOTAL=%d is exceeded\n",
			FBTFT_GAMMA_MAX_VALUES_TOTAL);
		return NULL;
	}

	/* defaults */
	if (!fps)
		fps = 20;
	if (!bpp)
		bpp = 16;

	if (!pdata) {
		dev_err(dev, "platform data is missing\n");
		return NULL;
	}

	/* Override values? */
	if (pdata->fps)
		fps = pdata->fps;
	if (pdata->txbuflen)
		txbuflen = pdata->txbuflen;
	if (pdata->display.init_sequence)
		init_sequence = pdata->display.init_sequence;
	if (pdata->gamma)
		gamma = pdata->gamma;
	if (pdata->display.debug)
		display->debug = pdata->display.debug;
	if (pdata->display.backlight)
		display->backlight = pdata->display.backlight;
	if (!pdata->display.width)
		pdata->display.width = display->width;
	if (!pdata->display.height)
		pdata->display.height = display->height;
	if (pdata->display.buswidth)
		display->buswidth = pdata->display.buswidth;
	if (pdata->display.regwidth)
		display->regwidth = pdata->display.regwidth;

	display->debug |= debug;
	fbtft_expand_debug_value(&display->debug);

	if (pdata->rotate == 90 || pdata->rotate == 270
		|| pdata->rotate_soft == 90 || pdata->rotate_soft == 270 
		) {
		width =  pdata->display.height;
		height = pdata->display.width;
		disp_width = display->height;
		disp_height = display->width;
	}
	else{
		width =  pdata->display.width;
		height = pdata->display.height;
		disp_width = display->width;
		disp_height = display->height;
	}

	vmem_size = width * height * bpp / 8;
	//vmem = devm_kzalloc(dev, vmem_size, GFP_KERNEL); // TRAP. Managed kzalloc. Memory allocated with this function is automatically freed on driver detach
	//vmem = kzalloc(vmem_size, GFP_DMA | GFP_KERNEL);
	vmem = vzalloc(vmem_size*FBTFT_NB_FRAMEBUFFERS); // Allocate directly 3 frame buffers
	if (!vmem)
		goto alloc_fail;

	for (i = 0; i < FBTFT_NB_POSTPROCESS_BUFFERS; ++i)
	{
		vmem_postprocess_buffers[i] = devm_kzalloc(dev, vmem_size, GFP_KERNEL);
		//vmem_postprocess_buffers[i] = kzalloc(vmem_size, GFP_DMA | GFP_KERNEL);
		//vmem_postprocess_buffers[i] = vzalloc(vmem_size);
		if (!vmem_postprocess_buffers[i])
			goto alloc_fail;
	}

	vmem_postprocess_hid = devm_kzalloc(dev, vmem_size, GFP_KERNEL);
	//vmem_postprocess_hid = kzalloc(vmem_size, GFP_DMA | GFP_KERNEL);
	//vmem_postprocess_hid = vzalloc(vmem_size);
	if (!vmem_postprocess_hid)
		goto alloc_fail;

	fbops = devm_kzalloc(dev, sizeof(struct fb_ops), GFP_KERNEL);
	if (!fbops)
		goto alloc_fail;

	fbdefio = devm_kzalloc(dev, sizeof(struct fb_deferred_io), GFP_KERNEL);
	if (!fbdefio)
		goto alloc_fail;

	buf = devm_kzalloc(dev, 128, GFP_KERNEL);
	if (!buf)
		goto alloc_fail;

	if (display->gamma_num && display->gamma_len) {
		gamma_curves = devm_kcalloc(dev,
					    display->gamma_num *
					    display->gamma_len,
					    sizeof(gamma_curves[0]),
					    GFP_KERNEL);
		if (!gamma_curves)
			goto alloc_fail;
	}

	info = framebuffer_alloc(sizeof(struct fbtft_par), dev);
	if (!info)
		goto alloc_fail;

	info->screen_buffer = vmem;
	info->fbops = fbops;
	info->fbdefio = fbdefio;

	fbops->owner        	=      dev->driver->owner;
	fbops->fb_read      	=      fb_sys_read;
	fbops->fb_write     	=      fbtft_fb_write;
	fbops->fb_fillrect  	=      fbtft_fb_fillrect;
	fbops->fb_copyarea  	=      fbtft_fb_copyarea;
	fbops->fb_imageblit 	=      fbtft_fb_imageblit;
	fbops->fb_setcolreg 	=      fbtft_fb_setcolreg;
	fbops->fb_blank     	=      fbtft_fb_blank;
	fbops->fb_pan_display   =      fbtft_fb_pan_display;

	fbdefio->delay =           HZ/fps;
	fbdefio->deferred_io =     fbtft_deferred_io;
	fbdefio->first_io =		   fbtft_first_io;
	fb_deferred_io_init(info);

	// Surcharge fb_mmap (after fb_deferred_io_init)
	fbops->fb_mmap = fbtft_fb_deferred_io_mmap;

	strncpy(info->fix.id, dev->driver->name, 16);
	info->fix.type =           FB_TYPE_PACKED_PIXELS;
	info->fix.visual =         FB_VISUAL_TRUECOLOR;
	info->fix.xpanstep =	   0;
	info->fix.ypanstep =	   height;
	info->fix.ywrapstep =	   0;
	info->fix.line_length =    width * bpp / 8;
	info->fix.accel =          FB_ACCEL_NONE;
	info->fix.smem_len =       vmem_size*FBTFT_NB_FRAMEBUFFERS;

	info->var.rotate =         pdata->rotate;
	info->var.xres =           width;
	info->var.yres =           height;
	info->var.xres_virtual =   info->var.xres;
	info->var.yres_virtual =   info->var.yres*FBTFT_NB_FRAMEBUFFERS; // So that SDL allows SDL_DOUBLEBUF and SDL_TRIPLEBUF
	info->var.bits_per_pixel = bpp;
	info->var.nonstd =         1;

	/* RGB565 */
	info->var.red.offset =     11;
	info->var.red.length =     5;
	info->var.green.offset =   5;
	info->var.green.length =   6;
	info->var.blue.offset =    0;
	info->var.blue.length =    5;
	info->var.transp.offset =  0;
	info->var.transp.length =  0;

	info->flags =              FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

	par = info->par;
	par->info = info;
	par->driver_xres = disp_width;
	par->driver_yres = disp_height;
	par->vmem_size = vmem_size;
	for (i = 0; i < FBTFT_NB_POSTPROCESS_BUFFERS; ++i)
	{
		par->vmem_postprocess_buffers[i] = vmem_postprocess_buffers[i];
	}
	par->vmem_postprocess_hid = vmem_postprocess_hid;
	par->cur_framebuffer_idx = 0;
	par->last_full_framebuffer_idx = par->cur_postprocess_buffer_idx;
	par->cur_postprocess_buffer_idx = 0;
	par->last_full_postprocess_buffer_idx = par->cur_postprocess_buffer_idx;
	par->vmem_ptr = par->info->screen_buffer + (par->cur_framebuffer_idx*par->vmem_size);
	par->nb_postprocess_buffers_full = 0;
	par->framebuffer_surcharge_status = FB_SURCHARGE_S_NONE;

	par->pdata = pdata;
	pdata->par = par;
	par->debug = display->debug;
	par->buf = buf;
	spin_lock_init(&par->dirty_lock);
	par->bgr = pdata->bgr;
	par->spi_async_mode = pdata->spi_async_mode;
	par->ready_for_spi_async = false;
	par->interlacing = pdata->interlacing;
	par->startbyte = pdata->startbyte;
	par->init_sequence = init_sequence;
	par->gamma.curves = gamma_curves;
	par->gamma.num_curves = display->gamma_num;
	par->gamma.num_values = display->gamma_len;
	mutex_init(&par->gamma.lock);
	info->pseudo_palette = par->pseudo_palette;

	if (par->gamma.curves && gamma) {
		if (fbtft_gamma_parse_str(par,
			par->gamma.curves, gamma, strlen(gamma)))
			goto alloc_fail;
	}

	/* Transmit buffer */
	if (txbuflen == -1)
		txbuflen = vmem_size + 2; /* add in case startbyte is used */
	if (txbuflen > vmem_size + 2)
		txbuflen = 0;
	if(par->spi_async_mode)
		txbuflen = 0;	/* Send whole buffer from video memory in async mode */

#ifdef __LITTLE_ENDIAN
	if ((!txbuflen) && (bpp > 8) && !par->spi_async_mode){
		txbuflen = PAGE_SIZE; /* need buffer for byteswapping */
	}
#endif

	if (txbuflen > 0) {
		txbuf = devm_kzalloc(par->info->device, txbuflen, GFP_KERNEL);
		if (!txbuf)
			goto alloc_fail;
		par->txbuf.buf = txbuf;
		par->txbuf.len = txbuflen;
	}

	/* Initialize gpios to disabled */
	par->gpio.reset = -1;
	par->gpio.dc = -1;
	par->gpio.rd = -1;
	par->gpio.wr = -1;
	par->gpio.cs = -1;
	par->gpio.latch = -1;
	for (i = 0; i < 16; i++) {
		par->gpio.db[i] = -1;
		par->gpio.led[i] = -1;
		par->gpio.aux[i] = -1;
	}
	par->dc_last_value = -1;

	/* default fbtft operations */
	par->fbtftops.write = fbtft_write_spi;
	par->fbtftops.write_async = fbtft_write_spi_async;
	par->fbtftops.read = fbtft_read_spi;
	par->fbtftops.write_vmem = fbtft_write_vmem16_bus8;
	par->fbtftops.write_register = fbtft_write_reg8_bus8;
	par->fbtftops.set_addr_win = fbtft_set_addr_win;
	par->fbtftops.reset = fbtft_reset;
	par->fbtftops.mkdirty = fbtft_mkdirty;
	par->fbtftops.update_display = fbtft_update_display;
	par->fbtftops.request_gpios = fbtft_request_gpios;
	if (display->backlight)
		par->fbtftops.register_backlight = fbtft_register_backlight;

	/* Init timer */
#ifdef FBTFT_USE_DELAYED_WORKER_FOR_BH
	INIT_DELAYED_WORK(&par->delayed_worker_postprocess.delayed_worker, fbtft_worker_postprocess_function);
	par->delayed_worker_postprocess.data_ptr = par;
#else // FBTFT_USE_DELAYED_WORKER_FOR_BH
	timer_setup(&par->timer_postprocess.timer, fbtft_timer_postprocess_function, 0);
	par->timer_postprocess.data_ptr = par;
#endif // FBTFT_USE_DELAYED_WORKER_FOR_BH

	/* Use driver provided functions */
	fbtft_merge_fbtftops(&par->fbtftops, &display->fbtftops);

	return info;

alloc_fail:
	vfree(vmem);

	return NULL;
}
EXPORT_SYMBOL(fbtft_framebuffer_alloc);

/**
 * fbtft_framebuffer_release - frees up all memory used by the framebuffer
 *
 * @info: frame buffer info structure
 *
 */
void fbtft_framebuffer_release(struct fb_info *info)
{
	fb_deferred_io_cleanup(info);
	vfree(info->screen_buffer);
	framebuffer_release(info);
}
EXPORT_SYMBOL(fbtft_framebuffer_release);

/**
 *	fbtft_register_framebuffer - registers a tft frame buffer device
 *	@fb_info: frame buffer info structure
 *
 *  Sets SPI driverdata if needed
 *  Requests needed gpios.
 *  Initializes display
 *  Updates display.
 *	Registers a frame buffer device @fb_info.
 *
 *	Returns negative errno on error, or zero for success.
 *
 */
int fbtft_register_framebuffer(struct fb_info *fb_info)
{
	int ret;
	char text1[50] = "";
	char text2[50] = "";
	char text3[50] = "";
	struct fbtft_par *par = fb_info->par;
	struct spi_device *spi = par->spi;

	/* sanity checks */
	if (!par->fbtftops.init_display) {
		dev_err(fb_info->device, "missing fbtftops.init_display()\n");
		return -EINVAL;
	}

	if (spi)
		spi_set_drvdata(spi, fb_info);
	if (par->pdev)
		platform_set_drvdata(par->pdev, fb_info);

	ret = par->fbtftops.request_gpios(par);
	if (ret < 0)
		goto reg_fail;

	if (par->fbtftops.verify_gpios) {
		ret = par->fbtftops.verify_gpios(par);
		if (ret < 0)
			goto reg_fail;
	}

	ret = par->fbtftops.init_display(par);
	if (ret < 0)
		goto reg_fail;
	
	if (par->fbtftops.set_var) {
		ret = par->fbtftops.set_var(par);
		if (ret < 0)
			goto reg_fail;
	}

	/* update the entire display */
	par->fbtftops.update_display(par, 0, par->info->var.yres - 1);

	if (par->fbtftops.set_gamma && par->gamma.curves) {
		ret = par->fbtftops.set_gamma(par, par->gamma.curves);
		if (ret)
			goto reg_fail;
	}

	if (par->fbtftops.register_backlight)
		par->fbtftops.register_backlight(par);

	ret = register_framebuffer(fb_info);
	if (ret < 0)
		goto reg_fail;

	fbtft_sysfs_init(par);

	if (par->txbuf.buf && par->txbuf.len >= 1024){
		sprintf(text1, ", %zu KiB SPI buf memory", par->txbuf.len >> 10);
	}
	if (spi){
		sprintf(text2, ", spi%d.%d at %d MHz", spi->master->bus_num,
			spi->chip_select, spi->max_speed_hz / 1000000);
	}
	if (fb_info->var.xres != par->driver_xres || fb_info->var.yres != par->driver_yres ){
		sprintf(text3, " used res out of %dx%d driver full res", 
			par->driver_xres, par->driver_yres);
	}

	dev_info(fb_info->dev,
		 "%s frame buffer, %dx%d%s, %d KiB video memory%s, %d buffers, fps=%lu%s%s%s\n",
		 fb_info->fix.id, fb_info->var.xres, fb_info->var.yres, text3,
		 fb_info->fix.smem_len >> 10, text1, 
		 FBTFT_NB_FRAMEBUFFERS,
		 HZ / fb_info->fbdefio->delay, text2,
		 par->spi_async_mode ? ", SPI mode async" : "",
		 par->interlacing ? ", Interlaced" : "");

#ifdef CONFIG_FB_BACKLIGHT
	/* Turn on backlight if available */
	if (fb_info->bl_dev) {
		fb_info->bl_dev->props.power = FB_BLANK_UNBLANK;
		fb_info->bl_dev->ops->update_status(fb_info->bl_dev);
	}
#endif

	return 0;

reg_fail:
	if (par->fbtftops.unregister_backlight)
		par->fbtftops.unregister_backlight(par);

	return ret;
}
EXPORT_SYMBOL(fbtft_register_framebuffer);

/**
 *	fbtft_unregister_framebuffer - releases a tft frame buffer device
 *	@fb_info: frame buffer info structure
 *
 *  Frees SPI driverdata if needed
 *  Frees gpios.
 *	Unregisters frame buffer device.
 *
 */
int fbtft_unregister_framebuffer(struct fb_info *fb_info)
{
	struct fbtft_par *par = fb_info->par;

	if (par->fbtftops.unregister_backlight)
		par->fbtftops.unregister_backlight(par);
	fbtft_sysfs_exit(par);
	return unregister_framebuffer(fb_info);
}
EXPORT_SYMBOL(fbtft_unregister_framebuffer);

#ifdef CONFIG_OF
/**
 * fbtft_init_display_dt() - Device Tree init_display() function
 * @par: Driver data
 *
 * Return: 0 if successful, negative if error
 */
static int fbtft_init_display_dt(struct fbtft_par *par)
{
	struct device_node *node = par->info->device->of_node;
	struct property *prop;
	const __be32 *p;
	u32 val;
	int buf[64], i, j;

	if (!node)
		return -EINVAL;

	prop = of_find_property(node, "init", NULL);
	p = of_prop_next_u32(prop, NULL, &val);
	if (!p)
		return -EINVAL;

	par->fbtftops.reset(par);
	if (par->gpio.cs != -1)
		gpio_set_value(par->gpio.cs, 0);  /* Activate chip */

	while (p) {
		if (val & FBTFT_OF_INIT_CMD) {
			val &= 0xFFFF;
			i = 0;
			while (p && !(val & 0xFFFF0000)) {
				if (i > 63) {
					dev_err(par->info->device,
					"%s: Maximum register values exceeded\n",
					__func__);
					return -EINVAL;
				}
				buf[i++] = val;
				p = of_prop_next_u32(prop, p, &val);
			}
			/* make debug message */
			fbtft_par_dbg(DEBUG_INIT_DISPLAY, par,
				      "init: write_register:\n");
			for (j = 0; j < i; j++)
				fbtft_par_dbg(DEBUG_INIT_DISPLAY, par,
					      "buf[%d] = %02X\n", j, buf[j]);

			par->fbtftops.write_register(par, i,
				buf[0], buf[1], buf[2], buf[3],
				buf[4], buf[5], buf[6], buf[7],
				buf[8], buf[9], buf[10], buf[11],
				buf[12], buf[13], buf[14], buf[15],
				buf[16], buf[17], buf[18], buf[19],
				buf[20], buf[21], buf[22], buf[23],
				buf[24], buf[25], buf[26], buf[27],
				buf[28], buf[29], buf[30], buf[31],
				buf[32], buf[33], buf[34], buf[35],
				buf[36], buf[37], buf[38], buf[39],
				buf[40], buf[41], buf[42], buf[43],
				buf[44], buf[45], buf[46], buf[47],
				buf[48], buf[49], buf[50], buf[51],
				buf[52], buf[53], buf[54], buf[55],
				buf[56], buf[57], buf[58], buf[59],
				buf[60], buf[61], buf[62], buf[63]);
		} else if (val & FBTFT_OF_INIT_DELAY) {
			fbtft_par_dbg(DEBUG_INIT_DISPLAY, par,
				      "init: msleep(%u)\n", val & 0xFFFF);
			msleep(val & 0xFFFF);
			p = of_prop_next_u32(prop, p, &val);
		} else {
			dev_err(par->info->device, "illegal init value 0x%X\n",
				val);
			return -EINVAL;
		}
	}

	return 0;
}
#endif

/**
 * fbtft_init_display() - Generic init_display() function
 * @par: Driver data
 *
 * Uses par->init_sequence to do the initialization
 *
 * Return: 0 if successful, negative if error
 */
int fbtft_init_display(struct fbtft_par *par)
{
	int buf[64];
	char msg[128];
	char str[16];
	int i = 0;
	int j;

	/* sanity check */
	if (!par->init_sequence) {
		dev_err(par->info->device,
			"error: init_sequence is not set\n");
		return -EINVAL;
	}

	/* make sure stop marker exists */
	for (i = 0; i < FBTFT_MAX_INIT_SEQUENCE; i++)
		if (par->init_sequence[i] == -3)
			break;
	if (i == FBTFT_MAX_INIT_SEQUENCE) {
		dev_err(par->info->device,
			"missing stop marker at end of init sequence\n");
		return -EINVAL;
	}

	par->fbtftops.reset(par);
	if (par->gpio.cs != -1)
		gpio_set_value(par->gpio.cs, 0);  /* Activate chip */

	i = 0;
	while (i < FBTFT_MAX_INIT_SEQUENCE) {
		if (par->init_sequence[i] == -3) {
			/* done */
			return 0;
		}
		if (par->init_sequence[i] >= 0) {
			dev_err(par->info->device,
				"missing delimiter at position %d\n", i);
			return -EINVAL;
		}
		if (par->init_sequence[i + 1] < 0) {
			dev_err(par->info->device,
				"missing value after delimiter %d at position %d\n",
				par->init_sequence[i], i);
			return -EINVAL;
		}
		switch (par->init_sequence[i]) {
		case -1:
			i++;
			/* make debug message */
			strcpy(msg, "");
			j = i + 1;
			while (par->init_sequence[j] >= 0) {
				sprintf(str, "0x%02X ", par->init_sequence[j]);
				strcat(msg, str);
				j++;
			}
			fbtft_par_dbg(DEBUG_INIT_DISPLAY, par,
				      "init: write(0x%02X) %s\n",
				      par->init_sequence[i], msg);

			/* Write */
			j = 0;
			while (par->init_sequence[i] >= 0) {
				if (j > 63) {
					dev_err(par->info->device,
					"%s: Maximum register values exceeded\n",
					__func__);
					return -EINVAL;
				}
				buf[j++] = par->init_sequence[i++];
			}
			par->fbtftops.write_register(par, j,
				buf[0], buf[1], buf[2], buf[3],
				buf[4], buf[5], buf[6], buf[7],
				buf[8], buf[9], buf[10], buf[11],
				buf[12], buf[13], buf[14], buf[15],
				buf[16], buf[17], buf[18], buf[19],
				buf[20], buf[21], buf[22], buf[23],
				buf[24], buf[25], buf[26], buf[27],
				buf[28], buf[29], buf[30], buf[31],
				buf[32], buf[33], buf[34], buf[35],
				buf[36], buf[37], buf[38], buf[39],
				buf[40], buf[41], buf[42], buf[43],
				buf[44], buf[45], buf[46], buf[47],
				buf[48], buf[49], buf[50], buf[51],
				buf[52], buf[53], buf[54], buf[55],
				buf[56], buf[57], buf[58], buf[59],
				buf[60], buf[61], buf[62], buf[63]);
			break;
		case -2:
			i++;
			fbtft_par_dbg(DEBUG_INIT_DISPLAY, par,
				"init: mdelay(%d)\n", par->init_sequence[i]);
			mdelay(par->init_sequence[i++]);
			break;
		default:
			dev_err(par->info->device,
				"unknown delimiter %d at position %d\n",
				par->init_sequence[i], i);
			return -EINVAL;
		}
	}

	dev_err(par->info->device,
		"%s: something is wrong. Shouldn't get here.\n", __func__);
	return -EINVAL;
}
EXPORT_SYMBOL(fbtft_init_display);

/**
 * fbtft_verify_gpios() - Generic verify_gpios() function
 * @par: Driver data
 *
 * Uses @spi, @pdev and @buswidth to determine which GPIOs is needed
 *
 * Return: 0 if successful, negative if error
 */
static int fbtft_verify_gpios(struct fbtft_par *par)
{
	struct fbtft_platform_data *pdata = par->pdata;
	int i;

	fbtft_par_dbg(DEBUG_VERIFY_GPIOS, par, "%s()\n", __func__);

	if (pdata->display.buswidth != 9 && par->startbyte == 0 &&
							par->gpio.dc < 0) {
		dev_err(par->info->device,
			"Missing info about 'dc' gpio. Aborting.\n");
		return -EINVAL;
	}

	if (!par->pdev)
		return 0;

	if (par->gpio.wr < 0) {
		dev_err(par->info->device, "Missing 'wr' gpio. Aborting.\n");
		return -EINVAL;
	}
	for (i = 0; i < pdata->display.buswidth; i++) {
		if (par->gpio.db[i] < 0) {
			dev_err(par->info->device,
				"Missing 'db%02d' gpio. Aborting.\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
/* returns 0 if the property is not present */
static u32 fbtft_of_value(struct device_node *node, const char *propname)
{
	int ret;
	u32 val = 0;

	ret = of_property_read_u32(node, propname, &val);
	if (ret == 0)
		pr_info("%s: %s = %u\n", __func__, propname, val);

	return val;
}


static irqreturn_t irq_TE_handler(int irq_no, void *dev_id)
{
    struct fbtft_platform_data *pdata = (struct fbtft_platform_data *) dev_id;
    struct fbtft_par *par = pdata->par;
    ktime_t ts_now;

    /* Freq */
#define SECS_TE_IRQ_FREQ	FBTFT_FREQ_UPDATE_SECS
    static ktime_t prev_ts = 0;
    static int te_count = 0;
	ts_now = ktime_get();
	// Overflow:
	if(ts_now < prev_ts){
		te_count = 0;
		prev_ts = ts_now;
	}
	else{
    	te_count++;
	}
	int delta_us = (int)ktime_us_delta(ts_now, prev_ts);
	if( delta_us > SECS_TE_IRQ_FREQ*1000000){
		//par->freq_te = te_count*1000000/delta_us; // floored value
		par->freq_te = (2*te_count*1000000+delta_us) / ( 2*delta_us); // rounded value
//#define DEBUG_TE_IRQ_FREQ
#ifdef DEBUG_TE_IRQ_FREQ
		printk("TE irq: %d times/sec\n", par->freq_te);
#endif //DEBUG_TE_IRQ_FREQ
		te_count = 0;
		prev_ts = ts_now;
	}

	/* Sanity check: SPI async data transfers not initialized yet */
	if(!par->ready_for_spi_async){
		return IRQ_HANDLED;
	}

	/* Sanity check: to avoid applicative tearing if user process fps is lower than TE freq
	   if (backbuffers not yet full) => wait a litte so that 
	   the user process has the time to fill a backbuffer 
	*/
	ts_now = ktime_get();
	if(ts_now < par->ts_last_ioctl_call) par->ts_last_ioctl_call = ts_now; //overflow
	int us_since_last_ioctl = (int)ktime_us_delta(ts_now, par->ts_last_ioctl_call);
#define FBTFT_TE_IRQ_WAIT_ONE_FULL_BUFFER	
#ifdef FBTFT_TE_IRQ_WAIT_ONE_FULL_BUFFER
	if(	!par->nb_framebuffers_full &&
		us_since_last_ioctl <= 1000000/FBTFT_MIN_FPS_WITHOUT_APPLICATIVE_TEARING &&
		us_since_last_ioctl < (par->us_between_ioctl_calls*2) ){

		// Debug
		/*printk("%s exit IRQ_HANDLED. par->nb_framebuffers_full=%d, us_since_last_ioctl=%d, par->us_between_ioctl_calls=%d\n", 
			__func__,
			par->nb_framebuffers_full, 
			us_since_last_ioctl, 
			par->us_between_ioctl_calls);*/
		//printk("TE nb_fb=%d", par->nb_framebuffers_full);

		// Exit
		return IRQ_HANDLED;		
	}
#endif //FBTFT_TE_IRQ_WAIT_ONE_FULL_BUFFER
	
	/* Reset ioctl frequency metrics here */	
	if(us_since_last_ioctl > IOCTL_FREQ_SECS*1000000){
		par->freq_ioctl_calls = 0;
	}

	/* Trigger new SPI transfer */
	fbtft_start_new_screen_transfer_async(pdata->par);

    return IRQ_HANDLED;
}



static struct fbtft_platform_data *fbtft_probe_dt(struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct fbtft_platform_data *pdata;
	int gpio, err;
	enum of_gpio_flags of_flags;
	char *te_irq_name;

	if (!node) {
		dev_err(dev, "Missing platform data or DT\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->display.width = fbtft_of_value(node, "width");
	pdata->display.height = fbtft_of_value(node, "height");
	pdata->display.regwidth = fbtft_of_value(node, "regwidth");
	pdata->display.buswidth = fbtft_of_value(node, "buswidth");
	pdata->display.backlight = fbtft_of_value(node, "backlight");
	pdata->display.bpp = fbtft_of_value(node, "bpp");
	pdata->display.debug = fbtft_of_value(node, "debug");
	pdata->rotate = fbtft_of_value(node, "rotate");
	pdata->rotate_soft = fbtft_of_value(node, "rotate_soft");
	pdata->bgr = of_property_read_bool(node, "bgr");
	pdata->spi_async_mode = of_property_read_bool(node, "spi_async_mode");
	pdata->interlacing = of_property_read_bool(node, "interlacing");
	pdata->fps = fbtft_of_value(node, "fps");
	pdata->txbuflen = fbtft_of_value(node, "txbuflen");
	pdata->startbyte = fbtft_of_value(node, "startbyte");
	of_property_read_string(node, "gamma", (const char **)&pdata->gamma);

	if (of_find_property(node, "led-gpios", NULL))
		pdata->display.backlight = 1;
	if (of_find_property(node, "init", NULL))
		pdata->display.fbtftops.init_display = fbtft_init_display_dt;
	pdata->display.fbtftops.request_gpios = fbtft_request_gpios_dt;

	/* TE signal for Vsync - Checking if GPIO is correct */
	pdata->te_irq_enabled = false;
	pdata->te_irq_id = 0;
	te_irq_name = "te-irq";
	if (of_find_property(node, te_irq_name, NULL)) {
		gpio = of_get_named_gpio_flags(node, te_irq_name, 0, &of_flags);
		if (gpio == -ENOENT || gpio == -EPROBE_DEFER || gpio < 0) {
			dev_err(dev,
				"failed to get '%s' from DT\n", te_irq_name);
		}
		else{
			pr_info("%s: %s = GPIO%d\n", __func__, te_irq_name, gpio);

			pdata->te_irq_id = gpio_to_irq(gpio);
			if(pdata->te_irq_id < 0) {
		        dev_err(dev,"%s - Unable to request IRQ: %d\n", __func__, pdata->te_irq_id);
		    }
		    else{
				pr_info("%s: TE GPIO%d => IRQ id = %d\n", __func__, gpio, pdata->te_irq_id);
				pdata->te_irq_enabled = true;
			}
		}
	}

	return pdata;
}
#else
static struct fbtft_platform_data *fbtft_probe_dt(struct device *dev)
{
	dev_err(dev, "Missing platform data\n");
	return ERR_PTR(-EINVAL);
}
#endif

/**
 * fbtft_probe_common() - Generic device probe() helper function
 * @display: Display properties
 * @sdev: SPI device
 * @pdev: Platform device
 *
 * Allocates, initializes and registers a framebuffer
 *
 * Either @sdev or @pdev should be NULL
 *
 * Return: 0 if successful, negative if error
 */
int fbtft_probe_common(struct fbtft_display *display,
			struct spi_device *sdev, struct platform_device *pdev)
{
	struct device *dev;
	struct fb_info *info;
	struct fbtft_par *par;
	struct fbtft_platform_data *pdata;
	int ret, err;

	if (sdev)
		dev = &sdev->dev;
	else
		dev = &pdev->dev;

	if (unlikely(display->debug & DEBUG_DRIVER_INIT_FUNCTIONS))
		dev_info(dev, "%s()\n", __func__);

	pdata = dev->platform_data;
	if (!pdata) {
		pdata = fbtft_probe_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	info = fbtft_framebuffer_alloc(display, dev, pdata);
	if (!info)
		return -ENOMEM;

	par = info->par;
	par->spi = sdev;
	par->pdev = pdev;

	if (display->buswidth == 0) {
		dev_err(dev, "buswidth is not set\n");
		return -EINVAL;
	}

	/* write register functions */
	if (display->regwidth == 8 && display->buswidth == 8) {
		par->fbtftops.write_register = fbtft_write_reg8_bus8;
	} else
	if (display->regwidth == 8 && display->buswidth == 9 && par->spi) {
		par->fbtftops.write_register = fbtft_write_reg8_bus9;
	} else if (display->regwidth == 16 && display->buswidth == 8) {
		par->fbtftops.write_register = fbtft_write_reg16_bus8;
	} else if (display->regwidth == 16 && display->buswidth == 16) {
		par->fbtftops.write_register = fbtft_write_reg16_bus16;
	} else {
		dev_warn(dev,
			"no default functions for regwidth=%d and buswidth=%d\n",
			display->regwidth, display->buswidth);
	}

	/* write_vmem() functions */
	if (display->buswidth == 8)
		par->fbtftops.write_vmem = fbtft_write_vmem16_bus8;
	else if (display->buswidth == 9)
		par->fbtftops.write_vmem = fbtft_write_vmem16_bus9;
	else if (display->buswidth == 16)
		par->fbtftops.write_vmem = fbtft_write_vmem16_bus16;

	/* GPIO write() functions */
	if (par->pdev) {
		if (display->buswidth == 8)
			par->fbtftops.write = fbtft_write_gpio8_wr;
		else if (display->buswidth == 16)
			par->fbtftops.write = fbtft_write_gpio16_wr;
	}

	/* 9-bit SPI setup */
	if (par->spi && display->buswidth == 9) {
		if (par->spi->master->bits_per_word_mask & SPI_BPW_MASK(9)) {
			par->spi->bits_per_word = 9;
		} else {
			dev_warn(&par->spi->dev,
				"9-bit SPI not available, emulating using 8-bit.\n");
			/* allocate buffer with room for dc bits */
			par->extra = devm_kzalloc(par->info->device,
				par->txbuf.len + (par->txbuf.len / 8) + 8,
				GFP_KERNEL);
			if (!par->extra) {
				ret = -ENOMEM;
				goto out_release;
			}
			par->fbtftops.write = fbtft_write_spi_emulate_9;
		}
	}

	if (!par->fbtftops.verify_gpios)
		par->fbtftops.verify_gpios = fbtft_verify_gpios;

	/* make sure we still use the driver provided functions */
	fbtft_merge_fbtftops(&par->fbtftops, &display->fbtftops);

	/* use init_sequence if provided */
	if (par->init_sequence)
		par->fbtftops.init_display = fbtft_init_display;

	/* use platform_data provided functions above all */
	fbtft_merge_fbtftops(&par->fbtftops, &pdata->display.fbtftops);
	
	ret = fbtft_register_framebuffer(info);
	if (ret < 0)
		goto out_release;

	/** Initialize TE interrupt */
	if(par->pdata->te_irq_enabled){
		/* Setting TE interrupt on Falling edge for now.
		Should be rising according to the datasheet 
		(time when display is not reading GRAM)
		but falling is better in practice since it gives 
		some controlled delay that hides the tearing line */
		/*#warning Rising edge for tests
		err = request_irq(par->pdata->te_irq_id, irq_TE_handler, 
				IRQF_SHARED | IRQF_TRIGGER_RISING,
	            "TE", par->pdata);*/
		err = request_irq(par->pdata->te_irq_id, irq_TE_handler, 
				IRQF_SHARED | IRQF_TRIGGER_FALLING,
	            "TE", par->pdata);
		if (err < 0) {
			dev_err(dev,"ERROR initializing TE signal irq\n");
			par->pdata->te_irq_enabled = false;
		}
	}

	/* Start constant Display update using spi async */
	if (par->spi_async_mode) {
		par->must_send_data_transfer_cmd = true;
		par->write_line_start = 0;
		par->write_line_end = par->info->var.yres - 1;
		if (par->pdata->te_irq_enabled){
			par->ready_for_spi_async = true;
		}
		else{
			fbtft_start_new_screen_transfer_async(par);
		}
	}
	return 0;

out_release:
	fbtft_framebuffer_release(info);

	return ret;
}
EXPORT_SYMBOL(fbtft_probe_common);

/**
 * fbtft_remove_common() - Generic device remove() helper function
 * @dev: Device
 * @info: Framebuffer
 *
 * Unregisters and releases the framebuffer
 *
 * Return: 0 if successful, negative if error
 */
int fbtft_remove_common(struct device *dev, struct fb_info *info)
{
	struct fbtft_par *par;

	if (!info)
		return -EINVAL;
	par = info->par;
	if (par)
		fbtft_par_dbg(DEBUG_DRIVER_INIT_FUNCTIONS, par,
			      "%s()\n", __func__);
	fbtft_unregister_framebuffer(info);
	fbtft_framebuffer_release(info);

	return 0;
}
EXPORT_SYMBOL(fbtft_remove_common);

MODULE_LICENSE("GPL");
