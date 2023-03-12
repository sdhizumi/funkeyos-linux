// SPDX-License-Identifier: GPL-2.0
#include <linux/fbtft.h>
#include "internal.h"

static int get_next_ulong(char **str_p, unsigned long *val, char *sep, int base)
{
	char *p_val;

	if (!str_p || !(*str_p))
		return -EINVAL;

	p_val = strsep(str_p, sep);

	if (!p_val)
		return -EINVAL;

	return kstrtoul(p_val, base, val);
}

int fbtft_gamma_parse_str(struct fbtft_par *par, u32 *curves,
			  const char *str, int size)
{
	char *str_p, *curve_p = NULL;
	char *tmp;
	unsigned long val = 0;
	int ret = 0;
	int curve_counter, value_counter;

	fbtft_par_dbg(DEBUG_SYSFS, par, "%s() str=\n", __func__);

	if (!str || !curves)
		return -EINVAL;

	fbtft_par_dbg(DEBUG_SYSFS, par, "%s\n", str);

	tmp = kmemdup(str, size + 1, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/* replace optional separators */
	str_p = tmp;
	while (*str_p) {
		if (*str_p == ',')
			*str_p = ' ';
		if (*str_p == ';')
			*str_p = '\n';
		str_p++;
	}

	str_p = strim(tmp);

	curve_counter = 0;
	while (str_p) {
		if (curve_counter == par->gamma.num_curves) {
			dev_err(par->info->device, "Gamma: Too many curves\n");
			ret = -EINVAL;
			goto out;
		}
		curve_p = strsep(&str_p, "\n");
		value_counter = 0;
		while (curve_p) {
			if (value_counter == par->gamma.num_values) {
				dev_err(par->info->device,
					"Gamma: Too many values\n");
				ret = -EINVAL;
				goto out;
			}
			ret = get_next_ulong(&curve_p, &val, " ", 16);
			if (ret)
				goto out;
			curves[curve_counter * par->gamma.num_values + value_counter] = val;
			value_counter++;
		}
		if (value_counter != par->gamma.num_values) {
			dev_err(par->info->device, "Gamma: Too few values\n");
			ret = -EINVAL;
			goto out;
		}
		curve_counter++;
	}
	if (curve_counter != par->gamma.num_curves) {
		dev_err(par->info->device, "Gamma: Too few curves\n");
		ret = -EINVAL;
		goto out;
	}

out:
	kfree(tmp);
	return ret;
}

static ssize_t
sprintf_gamma(struct fbtft_par *par, u32 *curves, char *buf)
{
	ssize_t len = 0;
	unsigned int i, j;

	mutex_lock(&par->gamma.lock);
	for (i = 0; i < par->gamma.num_curves; i++) {
		for (j = 0; j < par->gamma.num_values; j++)
			len += scnprintf(&buf[len], PAGE_SIZE,
			     "%04x ", curves[i * par->gamma.num_values + j]);
		buf[len - 1] = '\n';
	}
	mutex_unlock(&par->gamma.lock);

	return len;
}

static ssize_t store_gamma_curve(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;
	u32 tmp_curves[FBTFT_GAMMA_MAX_VALUES_TOTAL];
	int ret;

	ret = fbtft_gamma_parse_str(par, tmp_curves, buf, count);
	if (ret)
		return ret;

	ret = par->fbtftops.set_gamma(par, tmp_curves);
	if (ret)
		return ret;

	mutex_lock(&par->gamma.lock);
	memcpy(par->gamma.curves, tmp_curves,
	       par->gamma.num_curves * par->gamma.num_values * sizeof(tmp_curves[0]));
	mutex_unlock(&par->gamma.lock);

	return count;
}

static ssize_t show_gamma_curve(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return sprintf_gamma(par, par->gamma.curves, buf);
}

static struct device_attribute gamma_device_attrs[] = {
	__ATTR(gamma, 0660, show_gamma_curve, store_gamma_curve),
};

int fbtft_overlay_parse_str(struct fbtft_par *par, u32 *values,
			  const char *str, int size)
{
	char *str_p, *cur_line = NULL;
	char *tmp;
	unsigned long val = 0;
	int ret = 0;
	int line_counter, value_counter;

	fbtft_par_dbg(DEBUG_SYSFS, par, "%s() str=%s\n", __func__, str);

	if (!str || !values)
		return -EINVAL;

	fbtft_par_dbg(DEBUG_SYSFS, par, "%s\n", str);

	tmp = kmemdup(str, size + 1, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/* replace optional separators */
	str_p = tmp;
	while (*str_p) {
		if (*str_p == ',')
			*str_p = ' ';
		str_p++;
	}

	str_p = strim(tmp);

	line_counter = 0;
	while (str_p) {
		if (line_counter >= 1) {
			dev_err(par->info->device, "Overlay: Too many lines\n");
			ret = -EINVAL;
			goto out;
		}
		cur_line = strsep(&str_p, "\n");
		value_counter = 0;
		while (cur_line) {
			if (value_counter == FBTFT_OVERLAY_NB_VALUES) {
				dev_err(par->info->device,
					"Overlay: Too many values\n");
				ret = -EINVAL;
				goto out;
			}
			ret = get_next_ulong(&cur_line, &val, " ", 10);
			if (ret)
				goto out;

			// Check width
			if (value_counter == 2 && val > par->info->var.xres - 1) {
				dev_err(par->info->device, "Overlay: width (%lu) > display width (%d)\n",
					val, par->info->var.xres);
				ret = -EINVAL;
				goto out;
			}

			// Check height
			if (value_counter == 3 && val > par->info->var.yres - 1) {
				dev_err(par->info->device, "Overlay: height (%lu) > display height (%d)\n",
					val, par->info->var.yres);
				ret = -EINVAL;
				goto out;
			}

			values[value_counter] = val;
			value_counter++;
		}
		if (value_counter != FBTFT_OVERLAY_NB_VALUES) {
			dev_err(par->info->device, "Overlay: Too few values\n");
			ret = -EINVAL;
			goto out;
		}

		// Check x
		if (values[0] > values[2]) {
			dev_err(par->info->device, "Overlay: x (%u) > width (%u)\n",
				values[0], values[2]);
			ret = -EINVAL;
			goto out;
		}

		// Check y
		if (values[1] > values[3]) {
			dev_err(par->info->device, "Overlay: y (%u) > height (%u)\n",
				values[1], values[3]);
			ret = -EINVAL;
			goto out;
		}

		line_counter++;
	}

out:
	kfree(tmp);
	return ret;
}

static ssize_t store_overlay(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;
	u32 tmp_overlay[FBTFT_OVERLAY_NB_VALUES];
	int ret;

	ret = fbtft_overlay_parse_str(par, tmp_overlay, buf, count);
	if (ret)
		return ret;

	mutex_lock(&par->overlay.lock);
	par->overlay.x = tmp_overlay[0];
	par->overlay.y = tmp_overlay[1];
	par->overlay.w = tmp_overlay[2];
	par->overlay.h = tmp_overlay[3];
	mutex_unlock(&par->overlay.lock);

	return count;
}

static ssize_t show_overlay(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	ssize_t len = sprintf(buf, "%u,%u,%u,%u\n", par->overlay.x, par->overlay.y,
		par->overlay.w, par->overlay.h);

	return len;
}

static struct device_attribute overlay_device_attrs[] = {
	__ATTR(overlay, 0660, show_overlay, store_overlay),
};

void fbtft_expand_rotate_soft_value(unsigned long *rotate_soft)
{
	switch (*rotate_soft) {
	case 0:
	case 90:
	case 180:
	case 270:
		break;
	case 1:
		*rotate_soft = 90;
		break;
	case 2:
		*rotate_soft = 180;
		break;
	case 3:
		*rotate_soft = 270;
	default:
		printk("Wrong Rotate soft value: %lu\n", *rotate_soft);
		 *rotate_soft = 0;
		break;
	}
}

static ssize_t store_rotate_soft(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;
	int ret;

	ret = kstrtoul(buf, 10, &par->pdata->rotate_soft);
	if (ret)
		return ret;
	fbtft_expand_rotate_soft_value(&par->pdata->rotate_soft);

	/* Schedule deferred_io to update display (no-op if already on queue)*/
	if (!par->spi_async_mode){
		par->dirty_lines_start = 0;
		par->dirty_lines_end = par->info->var.yres - 1;
		schedule_delayed_work(&par->info->deferred_work, par->info->fbdefio->delay);
	}

	return count;
}

static ssize_t show_rotate_soft(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%lu\n", par->pdata->rotate_soft);
}

static struct device_attribute rotate_soft_device_attr =
	//__ATTR(rotate_soft, 0660, show_rotate_soft, store_rotate_soft);
	__ATTR(rotate_soft, 0440, show_rotate_soft, NULL);

static ssize_t store_notification(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;
	char *tmp;

	fbtft_par_dbg(DEBUG_SYSFS, par, "%s(), count=%d, str=%s\n", __func__, count, buf);
	//printk("%s(), count=%d, str=%s\n", __func__, count, buf);

	tmp = kmemdup(buf, count + 1, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/* Clear notif buffer if empty string */
	if (!strcmp(buf, "clear")){
		fbtft_par_dbg(DEBUG_SYSFS, par, "Clearing notif buffer\n");
		par->notification[0] = 0;
	}
	else{
		strncpy(par->notification, tmp, FBTFT_NOTIF_MAX_SIZE);
	}

	/* Schedule deferred_io to update display (no-op if already on queue)*/
	if (!par->spi_async_mode){
		par->dirty_lines_start = 0;
		par->dirty_lines_end = par->info->var.yres - 1;
		schedule_delayed_work(&par->info->deferred_work, par->info->fbdefio->delay);
	}

	return count;
}

static ssize_t show_notification(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%s\n", par->notification);
}

static struct device_attribute notification_device_attr =
	__ATTR(notification, 0660, show_notification, store_notification);

static ssize_t store_low_battery(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;
	int ret;

	ret = kstrtoul(buf, 10, &par->low_battery);
	if (ret)
		return ret;

	par->low_battery = par->low_battery?1:0;

	return count;
}

static ssize_t show_low_battery(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%lu\n", par->low_battery);
}

static struct device_attribute low_battery_device_attr =
	__ATTR(low_battery, 0660, show_low_battery, store_low_battery);

static ssize_t store_switch_backbuf(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;
	struct fb_fix_screeninfo *fix = &fb_info->fix;
	struct fb_var_screeninfo var = {
			.yoffset = par->cur_postprocess_buffer_idx*fix->ypanstep
		};
	fb_info->fbops->fb_pan_display(&var, fb_info);
	
	return count;
}

static ssize_t show_switch_backbuf(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	return 0;
}

static struct device_attribute switch_backbuf_device_attr =
	__ATTR(switch_backbuf, 0660, show_switch_backbuf, store_switch_backbuf);

static ssize_t show_freq_ioctl_calls(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%d\n", par->freq_ioctl_calls);
}

static struct device_attribute freq_ioctl_calls_device_attr =
	__ATTR(freq_ioctl_calls, 0440, show_freq_ioctl_calls, NULL);

static ssize_t show_freq_ioctl_processes(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%d\n", par->freq_ioctl_processes);
}

static struct device_attribute freq_ioctl_processes_device_attr =
	__ATTR(freq_ioctl_processes, 0440, show_freq_ioctl_processes, NULL);

static ssize_t show_freq_te(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%d\n", par->freq_te);
}

static struct device_attribute freq_te_device_attr =
	__ATTR(freq_te, 0440, show_freq_te, NULL);

static ssize_t show_freq_dma_transfer(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%d\n", par->freq_dma_transfers);
}

static struct device_attribute freq_dma_transfer_device_attr =
	__ATTR(freq_dma_transfer, 0440, show_freq_dma_transfer, NULL);

void fbtft_expand_debug_value(unsigned long *debug)
{
	switch (*debug & 0x7) {
	case 1:
		*debug |= DEBUG_LEVEL_1;
		break;
	case 2:
		*debug |= DEBUG_LEVEL_2;
		break;
	case 3:
		*debug |= DEBUG_LEVEL_3;
		break;
	case 4:
		*debug |= DEBUG_LEVEL_4;
		break;
	case 5:
		*debug |= DEBUG_LEVEL_5;
		break;
	case 6:
		*debug |= DEBUG_LEVEL_6;
		break;
	case 7:
		*debug = DEBUG_LEVEL_7;
		break;
	/*case 8:
		*debug = DEBUG_LEVEL_8; // FPS
		break;*/
	}
}

static ssize_t store_debug(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;
	int ret;

	ret = kstrtoul(buf, 10, &par->debug);
	if (ret)
		return ret;
	fbtft_expand_debug_value(&par->debug);

	return count;
}

static ssize_t show_debug(struct device *device,
			  struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct fbtft_par *par = fb_info->par;

	return snprintf(buf, PAGE_SIZE, "%lu\n", par->debug);
}

static struct device_attribute debug_device_attr =
	__ATTR(debug, 0660, show_debug, store_debug);

void fbtft_sysfs_init(struct fbtft_par *par)
{
	device_create_file(par->info->dev, &debug_device_attr);
	device_create_file(par->info->dev, &low_battery_device_attr);
	device_create_file(par->info->dev, &switch_backbuf_device_attr);
	device_create_file(par->info->dev, &rotate_soft_device_attr);
	device_create_file(par->info->dev, &notification_device_attr);
	device_create_file(par->info->dev, &freq_ioctl_calls_device_attr);
	device_create_file(par->info->dev, &freq_ioctl_processes_device_attr);
	device_create_file(par->info->dev, &freq_te_device_attr);
	device_create_file(par->info->dev, &freq_dma_transfer_device_attr);
	//device_create_file(par->info->dev, &overlay_device_attrs[0]);
	if (par->gamma.curves && par->fbtftops.set_gamma)
		device_create_file(par->info->dev, &gamma_device_attrs[0]);
}

void fbtft_sysfs_exit(struct fbtft_par *par)
{
	device_remove_file(par->info->dev, &debug_device_attr);
	device_remove_file(par->info->dev, &low_battery_device_attr);
	device_remove_file(par->info->dev, &switch_backbuf_device_attr);
	device_remove_file(par->info->dev, &rotate_soft_device_attr);
	device_remove_file(par->info->dev, &notification_device_attr);
	device_remove_file(par->info->dev, &freq_ioctl_calls_device_attr);
	device_remove_file(par->info->dev, &freq_ioctl_processes_device_attr);
	device_remove_file(par->info->dev, &freq_te_device_attr);
	device_remove_file(par->info->dev, &freq_dma_transfer_device_attr);
	//device_remove_file(par->info->dev, &overlay_device_attrs[0]);
	if (par->gamma.curves && par->fbtftops.set_gamma)
		device_remove_file(par->info->dev, &gamma_device_attrs[0]);
}
