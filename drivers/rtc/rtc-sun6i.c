/*
 * An RTC driver for Allwinner A31/A23
 *
 * Copyright (c) 2014, Chen-Yu Tsai <wens@csie.org>
 *
 * based on rtc-sunxi.c
 *
 * An RTC driver for Allwinner A10/A20
 *
 * Copyright (c) 2013, Carlo Caione <carlo.caione@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/types.h>

/*
 * Allows for effectively applying __must_check to a macro so we can have
 * both the type-agnostic benefits of the macros while also being able to
 * enforce that the return value is, in fact, checked.
 */
static inline bool __must_check __must_check_overflow(bool overflow)
{
	return unlikely(overflow);
}

/*
 * For simplicity and code hygiene, the fallback code below insists on
 * a, b and *d having the same type (similar to the min() and max()
 * macros), whereas gcc's type-generic overflow checkers accept
 * different types. Hence we don't just make check_add_overflow an
 * alias for __builtin_add_overflow, but add type checks similar to
 * below.
 */
#define check_add_overflow(a, b, d) __must_check_overflow(({	\
	typeof(a) __a = (a);			\
	typeof(b) __b = (b);			\
	typeof(d) __d = (d);			\
	(void) (&__a == &__b);			\
	(void) (&__a == __d);			\
	__builtin_add_overflow(__a, __b, __d);	\
}))

#define check_mul_overflow(a, b, d) __must_check_overflow(({	\
	typeof(a) __a = (a);			\
	typeof(b) __b = (b);			\
	typeof(d) __d = (d);			\
	(void) (&__a == &__b);			\
	(void) (&__a == __d);			\
	__builtin_mul_overflow(__a, __b, __d);	\
}))

/*
 * Compute a*b+c, returning SIZE_MAX on overflow. Internal helper for
 * struct_size() below.
 */
static inline __must_check size_t __ab_c_size(size_t a, size_t b, size_t c)
{
	size_t bytes;

	if (check_mul_overflow(a, b, &bytes))
		return SIZE_MAX;
	if (check_add_overflow(bytes, c, &bytes))
		return SIZE_MAX;

	return bytes;
}

/**
 * struct_size() - Calculate size of structure with trailing array.
 * @p: Pointer to the structure.
 * @member: Name of the array member.
 * @count: Number of elements in the array.
 *
 * Calculates size of memory needed for structure @p followed by an
 * array of @count number of @member elements.
 *
 * Return: number of bytes needed or SIZE_MAX on overflow.
 */
#define struct_size(p, member, count)					\
	__ab_c_size(count,						\
		    sizeof(*(p)->member) + __must_be_array((p)->member),\
		    sizeof(*(p)))

/* Control register */
#define SUN6I_LOSC_CTRL				0x0000
#define SUN6I_LOSC_CTRL_KEY			(0x16aa << 16)
#define SUN6I_LOSC_CTRL_AUTO_SWT_BYPASS		BIT(15)
#define SUN6I_LOSC_CTRL_AUTO_SWT_EN		BIT(14)
#define SUN6I_LOSC_CTRL_ALM_DHMS_ACC		BIT(9)
#define SUN6I_LOSC_CTRL_RTC_HMS_ACC		BIT(8)
#define SUN6I_LOSC_CTRL_RTC_YMD_ACC		BIT(7)
#define SUN6I_LOSC_CTRL_EXT_LOSC_EN		BIT(4)
#define SUN6I_LOSC_CTRL_GSM			(2 << 2)
#define SUN6I_LOSC_CTRL_EXT_OSC			BIT(0)
#define SUN6I_LOSC_CTRL_ACC_MASK		GENMASK(9, 7)

#define SUN6I_LOSC_CLK_PRESCAL			0x0008

/* RTC */
#define SUN6I_RTC_YMD				0x0010
#define SUN6I_RTC_HMS				0x0014

/* Alarm 0 (counter) */
#define SUN6I_ALRM_COUNTER			0x0020
#define SUN6I_ALRM_CUR_VAL			0x0024
#define SUN6I_ALRM_EN				0x0028
#define SUN6I_ALRM_EN_CNT_EN			BIT(0)
#define SUN6I_ALRM_IRQ_EN			0x002c
#define SUN6I_ALRM_IRQ_EN_CNT_IRQ_EN		BIT(0)
#define SUN6I_ALRM_IRQ_STA			0x0030
#define SUN6I_ALRM_IRQ_STA_CNT_IRQ_PEND		BIT(0)

/* Alarm 1 (wall clock) */
#define SUN6I_ALRM1_EN				0x0044
#define SUN6I_ALRM1_IRQ_EN			0x0048
#define SUN6I_ALRM1_IRQ_STA			0x004c
#define SUN6I_ALRM1_IRQ_STA_WEEK_IRQ_PEND	BIT(0)

/* Alarm config */
#define SUN6I_ALARM_CONFIG			0x0050
#define SUN6I_ALARM_CONFIG_WAKEUP		BIT(0)

#define SUN6I_LOSC_OUT_GATING			0x0060
#define SUN6I_LOSC_OUT_GATING_EN_OFFSET		0

#define SUNXI_GPDATA_REG(x)			(0x100 + ((x) << 2)) /* x: 0 ~ 7 */

/*
 * Get date values
 */
#define SUN6I_DATE_GET_DAY_VALUE(x)		((x)  & 0x0000001f)
#define SUN6I_DATE_GET_MON_VALUE(x)		(((x) & 0x00000f00) >> 8)
#define SUN6I_DATE_GET_YEAR_VALUE(x)		(((x) & 0x003f0000) >> 16)
#define SUN6I_LEAP_GET_VALUE(x)			(((x) & 0x00400000) >> 22)

/*
 * Get time values
 */
#define SUN6I_TIME_GET_SEC_VALUE(x)		((x)  & 0x0000003f)
#define SUN6I_TIME_GET_MIN_VALUE(x)		(((x) & 0x00003f00) >> 8)
#define SUN6I_TIME_GET_HOUR_VALUE(x)		(((x) & 0x001f0000) >> 16)

/*
 * Set date values
 */
#define SUN6I_DATE_SET_DAY_VALUE(x)		((x)       & 0x0000001f)
#define SUN6I_DATE_SET_MON_VALUE(x)		((x) <<  8 & 0x00000f00)
#define SUN6I_DATE_SET_YEAR_VALUE(x)		((x) << 16 & 0x003f0000)
#define SUN6I_LEAP_SET_VALUE(x)			((x) << 22 & 0x00400000)

/*
 * Set time values
 */
#define SUN6I_TIME_SET_SEC_VALUE(x)		((x)       & 0x0000003f)
#define SUN6I_TIME_SET_MIN_VALUE(x)		((x) <<  8 & 0x00003f00)
#define SUN6I_TIME_SET_HOUR_VALUE(x)		((x) << 16 & 0x001f0000)

/*
 * The year parameter passed to the driver is usually an offset relative to
 * the year 1900. This macro is used to convert this offset to another one
 * relative to the minimum year allowed by the hardware.
 *
 * The year range is 1970 - 2033. This range is selected to match Allwinner's
 * driver, even though it is somewhat limited.
 */
#define SUN6I_YEAR_MIN				1970
#define SUN6I_YEAR_MAX				2033
#define SUN6I_YEAR_OFF				(SUN6I_YEAR_MIN - 1900)

/*
 * There are other differences between models, including:
 *
 *   - number of GPIO pins that can be configured to hold a certain level
 *   - crypto-key related registers (H5, H6)
 *   - boot process related (super standby, secondary processor entry address)
 *     registers (R40, H6)
 *   - SYS power domain controls (R40)
 *   - DCXO controls (H6)
 *   - RC oscillator calibration (H6)
 *
 * These functions are not covered by this driver.
 */
struct sun6i_rtc_clk_data {
	unsigned long rc_osc_rate;
	unsigned int fixed_prescaler : 16;
	unsigned int has_prescaler : 1;
	unsigned int has_out_clk : 1;
	unsigned int export_iosc : 1;
	unsigned int has_losc_en : 1;
	unsigned int has_auto_swt : 1;
	unsigned int has_rtc_fixup : 1;
};

struct sun6i_rtc_dev {
	struct rtc_device *rtc;
	const struct sun6i_rtc_clk_data *data;
	void __iomem *base;
	int irq;
	unsigned long alarm;

	struct clk_hw hw;
	struct clk_hw *int_osc;
	struct clk *losc;
	struct clk *ext_losc;

	spinlock_t lock;
};

static struct sun6i_rtc_dev *sun6i_rtc;
static int losc_err_flag = 0;

#define OSC_32K	32000
#define OSC_ORG	32768
/*
 * This RTC count flow on sun8iw8 platform.
 * 32.768KHz---   --- [32768 count] ---
 *              \/                     \___[seconds reg]
 *              /\                     /
 * 32KHz    ---   --- [32000 count] ---
 *
 * We can calculate how much seconds missing, call sunxi_rtc_fixup.
 */
static void sun6i_rtc_fixup(struct device *dev, unsigned long *org_time, unsigned long *time)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);
	unsigned long delta;
	unsigned long t1, t2;
	bool out;
	unsigned long flags;

	if (!time || !chip->data->has_rtc_fixup)
		return;

	spin_lock_irqsave(&chip->lock, flags);
	if (!org_time) {
		t1 = (unsigned long long) readl(chip->base + SUNXI_GPDATA_REG(6)) << 32;
		t1 |= (unsigned long) readl(chip->base + SUNXI_GPDATA_REG(7));
	} else
		t1 = *org_time;

	t2 = *time;

	out = readl(chip->base + SUN6I_LOSC_CTRL) & SUN6I_LOSC_CTRL_EXT_OSC;
	spin_unlock_irqrestore(&chip->lock, flags);

	delta = (t2 > t1) ? (t2 - t1) : (t1 - t2);
	delta = (delta * (OSC_ORG - OSC_32K)) / OSC_ORG;

	if (out)
		*time -= delta;
	else
		*time += delta;
}


static void sun6i_rtc_save(struct device *dev, unsigned long time, bool force)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);
	unsigned long long tsec = (unsigned long long) time;
	unsigned int save = (0x80000000 & readl(chip->base + SUNXI_GPDATA_REG(6)));
	unsigned long flags;

	if (!chip->data->has_rtc_fixup)
		return;

	if (force || !save) {
		spin_lock_irqsave(&chip->lock, flags);
		writel((unsigned int) tsec, chip->base + SUNXI_GPDATA_REG(7));
		writel(((unsigned int) (tsec >> 32) | 0x80000000), chip->base + SUNXI_GPDATA_REG(6));
		spin_unlock_irqrestore(&chip->lock, flags);
	}
}

static unsigned long sun6i_rtc_osc_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct sun6i_rtc_dev *rtc = container_of(hw, struct sun6i_rtc_dev, hw);
	u32 val = 0;

	val = readl(rtc->base + SUN6I_LOSC_CTRL);
	if (val & SUN6I_LOSC_CTRL_EXT_OSC)
		return parent_rate;

	if (rtc->data->fixed_prescaler)
		parent_rate /= rtc->data->fixed_prescaler;

	if (rtc->data->has_prescaler) {
		val = readl(rtc->base + SUN6I_LOSC_CLK_PRESCAL);
		val &= GENMASK(4, 0);
	}

	return parent_rate / (val + 1);
}

static u8 sun6i_rtc_osc_get_parent(struct clk_hw *hw)
{
	struct sun6i_rtc_dev *rtc = container_of(hw, struct sun6i_rtc_dev, hw);

	return readl(rtc->base + SUN6I_LOSC_CTRL) & SUN6I_LOSC_CTRL_EXT_OSC;
}

static int sun6i_rtc_osc_set_parent(struct clk_hw *hw, u8 index)
{
	struct sun6i_rtc_dev *rtc = container_of(hw, struct sun6i_rtc_dev, hw);
	unsigned long flags;
	u32 val;

	if (index > 1)
		return -EINVAL;

	spin_lock_irqsave(&rtc->lock, flags);
	val = readl(rtc->base + SUN6I_LOSC_CTRL);
	val &= ~SUN6I_LOSC_CTRL_EXT_OSC;
	val |= SUN6I_LOSC_CTRL_KEY;
	val |= index ? SUN6I_LOSC_CTRL_EXT_OSC : 0;
	if (rtc->data->has_losc_en) {
		val &= ~SUN6I_LOSC_CTRL_EXT_LOSC_EN;
		val |= index ? SUN6I_LOSC_CTRL_EXT_LOSC_EN : 0;
	}
	writel(val, rtc->base + SUN6I_LOSC_CTRL);
	spin_unlock_irqrestore(&rtc->lock, flags);

	return 0;
}

static const struct clk_ops sun6i_rtc_osc_ops = {
	.recalc_rate	= sun6i_rtc_osc_recalc_rate,

	.get_parent	= sun6i_rtc_osc_get_parent,
	.set_parent	= sun6i_rtc_osc_set_parent,
};

static void __init sun6i_rtc_clk_init(struct device_node *node,
				      const struct sun6i_rtc_clk_data *data)
{
	struct clk_hw_onecell_data *clk_data;
	struct sun6i_rtc_dev *rtc;
	struct clk_init_data init = {
		.ops		= &sun6i_rtc_osc_ops,
		.name		= "losc",
	};
	const char *iosc_name = "rtc-int-osc";
	const char *clkout_name = "osc32k-out";
	const char *parents[2];
	u32 reg;

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return;

	rtc->data = data;
	clk_data = kzalloc(struct_size(clk_data, hws, 3), GFP_KERNEL);
	if (!clk_data) {
		kfree(rtc);
		return;
	}

	spin_lock_init(&rtc->lock);

	rtc->base = of_io_request_and_map(node, 0, of_node_full_name(node));
	if (IS_ERR(rtc->base)) {
		pr_crit("Can't map RTC registers");
		goto err;
	}

	reg = SUN6I_LOSC_CTRL_KEY;
	if (rtc->data->has_auto_swt) {
		/* Bypass auto-switch to int osc, on ext losc failure */
		reg |= SUN6I_LOSC_CTRL_AUTO_SWT_BYPASS;
		writel(reg, rtc->base + SUN6I_LOSC_CTRL);
	}

	/* Switch to the external, more precise, oscillator, if present */
	if (of_get_property(node, "clocks", NULL)) {
		reg |= SUN6I_LOSC_CTRL_EXT_OSC;
		reg |= SUN6I_LOSC_CTRL_GSM;
		if (rtc->data->has_losc_en)
			reg |= SUN6I_LOSC_CTRL_EXT_LOSC_EN;
		writel(reg, rtc->base + SUN6I_LOSC_CTRL);
		reg = readl(rtc->base + SUN6I_LOSC_CTRL);
		if (!(reg & SUN6I_LOSC_CTRL_EXT_OSC)) {
			printk(KERN_ERR "[RTC] WARNING: Rtc time will be wrong!!\n");
			losc_err_flag = 1;
		}
	} else
		writel(reg, rtc->base + SUN6I_LOSC_CTRL);

	/* Yes, I know, this is ugly. */
	sun6i_rtc = rtc;

	/* Only read IOSC name from device tree if it is exported */
	if (rtc->data->export_iosc)
		of_property_read_string_index(node, "clock-output-names", 2,
					      &iosc_name);

	rtc->int_osc = clk_hw_register_fixed_rate_with_accuracy(NULL,
								iosc_name,
								NULL, 0,
								rtc->data->rc_osc_rate,
								300000000);
	if (IS_ERR(rtc->int_osc)) {
		pr_crit("Couldn't register the internal oscillator\n");
		goto err;
	}

	parents[0] = clk_hw_get_name(rtc->int_osc);
	/* If there is no external oscillator, this will be NULL and ... */
	parents[1] = of_clk_get_parent_name(node, 0);

	rtc->hw.init = &init;

	init.parent_names = parents;
	/* ... number of clock parents will be 1. */
	init.num_parents = of_clk_get_parent_count(node) + 1;
	of_property_read_string_index(node, "clock-output-names", 0,
				      &init.name);

	rtc->losc = clk_register(NULL, &rtc->hw);
	if (IS_ERR(rtc->losc)) {
		pr_crit("Couldn't register the LOSC clock\n");
		goto err_register;
	}

	of_property_read_string_index(node, "clock-output-names", 1,
				      &clkout_name);
	rtc->ext_losc = clk_register_gate(NULL, clkout_name, init.name,
					  0, rtc->base + SUN6I_LOSC_OUT_GATING,
					  SUN6I_LOSC_OUT_GATING_EN_OFFSET, 0,
					  &rtc->lock);
	if (IS_ERR(rtc->ext_losc)) {
		pr_crit("Couldn't register the LOSC external gate\n");
		goto err_register;
	}

	clk_data->num = 2;
	clk_data->hws[0] = &rtc->hw;
	clk_data->hws[1] = __clk_get_hw(rtc->ext_losc);
	if (rtc->data->export_iosc) {
		clk_data->hws[2] = rtc->int_osc;
		clk_data->num = 3;
	}
	of_clk_add_hw_provider(node, of_clk_hw_onecell_get, clk_data);
	return;

err_register:
	clk_hw_unregister_fixed_rate(rtc->int_osc);
err:
	kfree(clk_data);
}

static const struct sun6i_rtc_clk_data sun6i_a31_rtc_data = {
	.rc_osc_rate = 667000, /* datasheet says 600 ~ 700 KHz */
	.has_prescaler = 1,
};

static void __init sun6i_a31_rtc_clk_init(struct device_node *node)
{
	sun6i_rtc_clk_init(node, &sun6i_a31_rtc_data);
}
CLK_OF_DECLARE_DRIVER(sun6i_a31_rtc_clk, "allwinner,sun6i-a31-rtc",
		      sun6i_a31_rtc_clk_init);

static const struct sun6i_rtc_clk_data sun8i_a23_rtc_data = {
	.rc_osc_rate = 667000, /* datasheet says 600 ~ 700 KHz */
	.has_prescaler = 1,
	.has_out_clk = 1,
};

static void __init sun8i_a23_rtc_clk_init(struct device_node *node)
{
	sun6i_rtc_clk_init(node, &sun8i_a23_rtc_data);
}
CLK_OF_DECLARE_DRIVER(sun8i_a23_rtc_clk, "allwinner,sun8i-a23-rtc",
		      sun8i_a23_rtc_clk_init);

static const struct sun6i_rtc_clk_data sun8i_h3_rtc_data = {
	.rc_osc_rate = 16000000,
	.fixed_prescaler = 32,
	.has_prescaler = 1,
	.has_out_clk = 1,
	.export_iosc = 1,
};

static void __init sun8i_h3_rtc_clk_init(struct device_node *node)
{
	sun6i_rtc_clk_init(node, &sun8i_h3_rtc_data);
}
CLK_OF_DECLARE_DRIVER(sun8i_h3_rtc_clk, "allwinner,sun8i-h3-rtc",
		      sun8i_h3_rtc_clk_init);
/* As far as we are concerned, clocks for H5 are the same as H3 */
CLK_OF_DECLARE_DRIVER(sun50i_h5_rtc_clk, "allwinner,sun50i-h5-rtc",
		      sun8i_h3_rtc_clk_init);

static const struct sun6i_rtc_clk_data sun50i_h6_rtc_data = {
	.rc_osc_rate = 16000000,
	.fixed_prescaler = 32,
	.has_prescaler = 1,
	.has_out_clk = 1,
	.export_iosc = 1,
	.has_losc_en = 1,
	.has_auto_swt = 1,
};

static void __init sun50i_h6_rtc_clk_init(struct device_node *node)
{
	sun6i_rtc_clk_init(node, &sun50i_h6_rtc_data);
}
CLK_OF_DECLARE_DRIVER(sun50i_h6_rtc_clk, "allwinner,sun50i-h6-rtc",
		      sun50i_h6_rtc_clk_init);

/*
 * The R40 user manual is self-conflicting on whether the prescaler is
 * fixed or configurable. The clock diagram shows it as fixed, but there
 * is also a configurable divider in the RTC block.
 */
static const struct sun6i_rtc_clk_data sun8i_r40_rtc_data = {
	.rc_osc_rate = 16000000,
	.fixed_prescaler = 512,
};
static void __init sun8i_r40_rtc_clk_init(struct device_node *node)
{
	sun6i_rtc_clk_init(node, &sun8i_r40_rtc_data);
}
CLK_OF_DECLARE_DRIVER(sun8i_r40_rtc_clk, "allwinner,sun8i-r40-rtc",
		      sun8i_r40_rtc_clk_init);

static const struct sun6i_rtc_clk_data sun8i_v3_rtc_data = {
	.rc_osc_rate = 32000,
	.has_out_clk = 1,
	.has_auto_swt = 1,
	.has_rtc_fixup = 1,
};

static void __init sun8i_v3_rtc_clk_init(struct device_node *node)
{
	sun6i_rtc_clk_init(node, &sun8i_v3_rtc_data);
}
CLK_OF_DECLARE_DRIVER(sun8i_v3_rtc_clk, "allwinner,sun8i-v3-rtc",
		      sun8i_v3_rtc_clk_init);

static irqreturn_t sun6i_rtc_alarmirq(int irq, void *id)
{
	struct sun6i_rtc_dev *chip = (struct sun6i_rtc_dev *) id;
	irqreturn_t ret = IRQ_NONE;
	u32 val;

	spin_lock(&chip->lock);
	val = readl(chip->base + SUN6I_ALRM_IRQ_STA);

	if (val & SUN6I_ALRM_IRQ_STA_CNT_IRQ_PEND) {
		val |= SUN6I_ALRM_IRQ_STA_CNT_IRQ_PEND;
		writel(val, chip->base + SUN6I_ALRM_IRQ_STA);

		rtc_update_irq(chip->rtc, 1, RTC_AF | RTC_IRQF);

		ret = IRQ_HANDLED;
	}
	spin_unlock(&chip->lock);

	return ret;
}

static void sun6i_rtc_setaie(int to, struct sun6i_rtc_dev *chip)
{
	u32 alrm_val = 0;
	u32 alrm_irq_val = 0;
	u32 alrm_wake_val = 0;
	unsigned long flags;

	if (to) {
		alrm_val = SUN6I_ALRM_EN_CNT_EN;
		alrm_irq_val = SUN6I_ALRM_IRQ_EN_CNT_IRQ_EN;
		alrm_wake_val = SUN6I_ALARM_CONFIG_WAKEUP;
	} else {
		writel(SUN6I_ALRM_IRQ_STA_CNT_IRQ_PEND,
		       chip->base + SUN6I_ALRM_IRQ_STA);
	}

	spin_lock_irqsave(&chip->lock, flags);
	writel(alrm_val, chip->base + SUN6I_ALRM_EN);
	writel(alrm_irq_val, chip->base + SUN6I_ALRM_IRQ_EN);
	writel(alrm_wake_val, chip->base + SUN6I_ALARM_CONFIG);
	spin_unlock_irqrestore(&chip->lock, flags);
}

static int sun6i_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);
	u32 date, time;
	unsigned long origin;

	/* only for alarm losc err occur */
	if (losc_err_flag) {
		rtc_tm->tm_sec	= 0;
		rtc_tm->tm_min	= 0;
		rtc_tm->tm_hour = 0;

		rtc_tm->tm_mday = 0;
		rtc_tm->tm_mon	= 0;
		rtc_tm->tm_year = 0;
		return -1;
	}

	/*
	 * read again in case it changes
	 */
	do {
		date = readl(chip->base + SUN6I_RTC_YMD);
		time = readl(chip->base + SUN6I_RTC_HMS);
	} while ((date != readl(chip->base + SUN6I_RTC_YMD)) ||
		 (time != readl(chip->base + SUN6I_RTC_HMS)));

	rtc_tm->tm_sec  = SUN6I_TIME_GET_SEC_VALUE(time);
	rtc_tm->tm_min  = SUN6I_TIME_GET_MIN_VALUE(time);
	rtc_tm->tm_hour = SUN6I_TIME_GET_HOUR_VALUE(time);

	rtc_tm->tm_mday = SUN6I_DATE_GET_DAY_VALUE(date);
	rtc_tm->tm_mon  = SUN6I_DATE_GET_MON_VALUE(date);
	rtc_tm->tm_year = SUN6I_DATE_GET_YEAR_VALUE(date);

	rtc_tm->tm_mon  -= 1;

	/*
	 * switch from (data_year->min)-relative offset to
	 * a (1900)-relative one
	 */
	rtc_tm->tm_year += SUN6I_YEAR_OFF;

	/*
	 * If it first time read and have not save it
	 * we should save it to backup register
	 */
	sun6i_rtc_save(dev, time, false);
	rtc_tm_to_time(rtc_tm, &origin);
	sun6i_rtc_fixup(dev, NULL, &origin);
	rtc_time_to_tm(origin, rtc_tm);

	dev_dbg(dev, "Read hardware RTC time %04d-%02d-%02d %02d:%02d:%02d\n",
			rtc_tm->tm_year + 1900, rtc_tm->tm_mon + 1, rtc_tm->tm_mday,
			rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);
	return 0;
}

static int sun6i_rtc_getalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);
	unsigned long flags;
	u32 alrm_st;
	u32 alrm_en;
	unsigned long alarm_cur = 0;
	unsigned long alarm_cnt = 0;
	unsigned long alarm_seconds = 0;

	spin_lock_irqsave(&chip->lock, flags);
	alarm_cnt = readl(chip->base + SUN6I_ALRM_COUNTER);
	alarm_cur = readl(chip->base + SUN6I_ALRM_CUR_VAL);
	alrm_en = readl(chip->base + SUN6I_ALRM_IRQ_EN);
	alrm_st = readl(chip->base + SUN6I_ALRM_IRQ_STA);
	spin_unlock_irqrestore(&chip->lock, flags);

	dev_dbg(dev, "alarm_cnt: %lu, alarm_cur: %lu\n", alarm_cnt, alarm_cur);
	if (alarm_cur > alarm_cnt) {
		/* alarm is disabled. */
		wkalrm->enabled = 0;
		wkalrm->time.tm_mon = -1;
		wkalrm->time.tm_mday = -1;
		wkalrm->time.tm_year = -1;
		wkalrm->time.tm_hour = -1;
		wkalrm->time.tm_min = -1;
		wkalrm->time.tm_sec = -1;
		return 0;
	}

	if (sun6i_rtc_gettime(dev, &wkalrm->time))
		return -EINVAL;

	rtc_tm_to_time(&wkalrm->time, &alarm_seconds);
	alarm_cnt = (alarm_cnt - alarm_cur);
	alarm_cur = 0;
	sun6i_rtc_fixup(dev, &alarm_cur, &alarm_cnt);
	alarm_seconds += alarm_cnt;

	rtc_time_to_tm(alarm_seconds, &wkalrm->time);
	dev_dbg(dev, "alarm_seconds: %lu\n", alarm_seconds);

	wkalrm->enabled = !!(alrm_en & SUN6I_ALRM_EN_CNT_EN);
	wkalrm->pending = !!(alrm_st & SUN6I_ALRM_EN_CNT_EN);
	rtc_time_to_tm(chip->alarm, &wkalrm->time);

	return 0;
}

static int sun6i_rtc_setalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);
	struct rtc_time *alrm_tm = &wkalrm->time;
	struct rtc_time tm_now;
	unsigned long time_now = 0;
	unsigned long time_set = 0;
	unsigned long time_gap = 0;
	int ret = 0;

	ret = sun6i_rtc_gettime(dev, &tm_now);
	if (ret < 0) {
		dev_err(dev, "Error in getting time\n");
		return -EINVAL;
	}

	rtc_tm_to_time(alrm_tm, &time_set);
	rtc_tm_to_time(&tm_now, &time_now);

	dev_dbg(dev, "Now RTC time %04d-%02d-%02d %02d:%02d:%02d\n",
			tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
			tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
	dev_dbg(dev, "Set alarm time %04d-%02d-%02d %02d:%02d:%02d\n",
			alrm_tm->tm_year + 1900, alrm_tm->tm_mon + 1, alrm_tm->tm_mday,
			alrm_tm->tm_hour, alrm_tm->tm_min, alrm_tm->tm_sec);

	if (time_set <= time_now) {
		dev_err(dev, "Date to set in the past\n");
		return -EINVAL;
	}

	/*
	 * Here, time_set & time_now are normal seconds,
	 * but we need to calculate fake time_now to get missing seconds.
	 */
	sun6i_rtc_fixup(dev, &time_set, &time_now);

	time_gap = time_set - time_now;

	if (time_gap > U32_MAX) {
		dev_err(dev, "Date too far in the future\n");
		return -EINVAL;
	}

	/* Clear the alm counter enable bit */
	sun6i_rtc_setaie(0, chip);

	/* Clear the alm count & irq value */
	writel(0, chip->base + SUN6I_ALRM_COUNTER);
	writel(0, chip->base + SUN6I_ALRM_IRQ_EN);
	usleep_range(100, 300);

	/* Rewrite the alm count & irq value */
	writel(time_gap, chip->base + SUN6I_ALRM_COUNTER);
	writel(SUN6I_ALRM_IRQ_EN_CNT_IRQ_EN, chip->base + SUN6I_ALRM_COUNTER);
	chip->alarm = time_set;

	/* Decide whether we should start the counter to down count */
	sun6i_rtc_setaie(wkalrm->enabled, chip);

	return 0;
}

static int sun6i_rtc_wait(struct sun6i_rtc_dev *chip, int offset,
			  unsigned int mask, unsigned int ms_timeout)
{
	const unsigned long timeout = jiffies + msecs_to_jiffies(ms_timeout);
	u32 reg;

	do {
		reg = readl(chip->base + offset);
		reg &= mask;

		if (!reg)
			return 0;

	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int sun6i_rtc_settime(struct device *dev, struct rtc_time *rtc_tm)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);
	u32 date = 0;
	u32 time = 0;
	int year;
	unsigned long origin;

	year = rtc_tm->tm_year + 1900;
	if (year < SUN6I_YEAR_MIN || year > SUN6I_YEAR_MAX) {
		dev_err(dev, "rtc only supports year in range %d - %d\n",
			SUN6I_YEAR_MIN, SUN6I_YEAR_MAX);
		return -EINVAL;
	}
	dev_dbg(dev, "Will set time: %04d-%02d-%02d %02d:%02d:%02d\n",
			rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday,
			rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);

	rtc_tm_to_time(rtc_tm, &origin);
	rtc_tm->tm_year -= SUN6I_YEAR_OFF;
	rtc_tm->tm_mon += 1;

	date = SUN6I_DATE_SET_DAY_VALUE(rtc_tm->tm_mday) |
		SUN6I_DATE_SET_MON_VALUE(rtc_tm->tm_mon)  |
		SUN6I_DATE_SET_YEAR_VALUE(rtc_tm->tm_year);

	if (is_leap_year(rtc_tm->tm_year + SUN6I_YEAR_MIN))
		date |= SUN6I_LEAP_SET_VALUE(1);

	time = SUN6I_TIME_SET_SEC_VALUE(rtc_tm->tm_sec)  |
		SUN6I_TIME_SET_MIN_VALUE(rtc_tm->tm_min)  |
		SUN6I_TIME_SET_HOUR_VALUE(rtc_tm->tm_hour);

	/* Check whether registers are writable */
	if (sun6i_rtc_wait(chip, SUN6I_LOSC_CTRL,
			   SUN6I_LOSC_CTRL_ACC_MASK, 50)) {
		dev_err(dev, "rtc is still busy.\n");
		return -EBUSY;
	}

	/* We need to save origin time as seconds */
	sun6i_rtc_save(dev, origin, true);

	writel(time, chip->base + SUN6I_RTC_HMS);

	/*
	 * After writing the RTC HH-MM-SS register, the
	 * SUN6I_LOSC_CTRL_RTC_HMS_ACC bit is set and it will not
	 * be cleared until the real writing operation is finished
	 */

	if (sun6i_rtc_wait(chip, SUN6I_LOSC_CTRL,
			   SUN6I_LOSC_CTRL_RTC_HMS_ACC, 50)) {
		dev_err(dev, "Failed to set rtc time.\n");
		return -ETIMEDOUT;
	}

	writel(date, chip->base + SUN6I_RTC_YMD);

	/*
	 * After writing the RTC YY-MM-DD register, the
	 * SUN6I_LOSC_CTRL_RTC_YMD_ACC bit is set and it will not
	 * be cleared until the real writing operation is finished
	 */

	if (sun6i_rtc_wait(chip, SUN6I_LOSC_CTRL,
			   SUN6I_LOSC_CTRL_RTC_YMD_ACC, 50)) {
		dev_err(dev, "Failed to set rtc time.\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int sun6i_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: will %s RTC IRQ\n", __func__,
			enabled ? "enable" : "disabled");
	if (!enabled)
		sun6i_rtc_setaie(enabled, chip);

	return 0;
}

static const struct rtc_class_ops sun6i_rtc_ops = {
	.read_time		= sun6i_rtc_gettime,
	.set_time		= sun6i_rtc_settime,
	.read_alarm		= sun6i_rtc_getalarm,
	.set_alarm		= sun6i_rtc_setalarm,
	.alarm_irq_enable	= sun6i_rtc_alarm_irq_enable
};

#ifdef CONFIG_PM_SLEEP
/* Enable IRQ wake on suspend, to wake up from RTC. */
static int sun6i_rtc_suspend(struct device *dev)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);

	return 0;
}

/* Disable IRQ wake on resume. */
static int sun6i_rtc_resume(struct device *dev)
{
	struct sun6i_rtc_dev *chip = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sun6i_rtc_pm_ops,
	sun6i_rtc_suspend, sun6i_rtc_resume);

static int sun6i_rtc_probe(struct platform_device *pdev)
{
	struct sun6i_rtc_dev *chip = sun6i_rtc;
	int ret;

	if (!chip)
		return -ENODEV;

	platform_set_drvdata(pdev, chip);

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0)
		return chip->irq;

	ret = devm_request_irq(&pdev->dev, chip->irq, sun6i_rtc_alarmirq,
			       0, dev_name(&pdev->dev), chip);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return ret;
	}

	/* clear the alarm counter value */
	writel(0, chip->base + SUN6I_ALRM_COUNTER);

	/* clear the alarm current value */
	writel(0, chip->base + SUN6I_ALRM_CUR_VAL);

	/* disable counter alarm */
	writel(0, chip->base + SUN6I_ALRM_EN);

	/* disable counter alarm interrupt */
	writel(0, chip->base + SUN6I_ALRM_IRQ_EN);

	/* disable week alarm */
	writel(0, chip->base + SUN6I_ALRM1_EN);

	/* disable week alarm interrupt */
	writel(0, chip->base + SUN6I_ALRM1_IRQ_EN);

	/* clear counter alarm pending interrupts */
	writel(SUN6I_ALRM_IRQ_STA_CNT_IRQ_PEND,
	       chip->base + SUN6I_ALRM_IRQ_STA);

	/* clear week alarm pending interrupts */
	writel(SUN6I_ALRM1_IRQ_STA_WEEK_IRQ_PEND,
	       chip->base + SUN6I_ALRM1_IRQ_STA);

	/* disable alarm wakeup */
	writel(0, chip->base + SUN6I_ALARM_CONFIG);

	clk_prepare_enable(chip->losc);

	device_init_wakeup(&pdev->dev, 1);

	chip->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(chip->rtc))
		return PTR_ERR(chip->rtc);

	chip->rtc->ops = &sun6i_rtc_ops;

	ret = devm_rtc_register_device(chip->rtc);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "RTC enabled\n");

	return 0;
}

/*
 * As far as RTC functionality goes, all models are the same. The
 * datasheets claim that different models have different number of
 * registers available for non-volatile storage, but experiments show
 * that all SoCs have 16 registers available for this purpose.
 */
static const struct of_device_id sun6i_rtc_dt_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-rtc" },
	{ .compatible = "allwinner,sun8i-a23-rtc" },
	{ .compatible = "allwinner,sun8i-h3-rtc" },
	{ .compatible = "allwinner,sun8i-r40-rtc" },
	{ .compatible = "allwinner,sun8i-v3-rtc" },
	{ .compatible = "allwinner,sun50i-h5-rtc" },
	{ .compatible = "allwinner,sun50i-h6-rtc" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sun6i_rtc_dt_ids);

static struct platform_driver sun6i_rtc_driver = {
	.probe		= sun6i_rtc_probe,
	.driver		= {
		.name		= "sun6i-rtc",
		.of_match_table = sun6i_rtc_dt_ids,
		.pm = &sun6i_rtc_pm_ops,
	},
};
builtin_platform_driver(sun6i_rtc_driver);
