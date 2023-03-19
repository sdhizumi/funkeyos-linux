/*
 * FB driver for the ST7789V LCD Controller
 *
 * Copyright (C) 2015 Dennis Menschel
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

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <video/mipi_display.h>

#include <linux/fbtft.h>

#define DRVNAME "fb_st7789v"
//#define SAEF_SETTINGS

#ifndef SAEF_SETTINGS
	#define DEFAULT_GAMMA \
		"f0 08 0e 09 08 04 2f 33 45 36 13 12 2a 2d\n" \
		"f0 0e 12 0c 0a 15 2e 32 44 39 17 18 2b 2f"
#else
	#define DEFAULT_GAMMA \
		"f0 19 1e 0A 09 15 3D 44 51 19 14 13 2c 31\n" \
		"f0 18 1E 0A 09 25 3F 43 52 19 14 13 2c 31"
#endif

/**
 * enum st7789v_command - ST7789V display controller commands
 *
 * @PORCTRL: porch setting
 * @GCTRL: gate control
 * @VCOMS: VCOM setting
 * @VDVVRHEN: VDV and VRH command enable
 * @VRHS: VRH set
 * @VDVS: VDV set
 * @VCMOFSET: VCOM offset set
 * @PWCTRL1: power control 1
 * @PVGAMCTRL: positive voltage gamma control
 * @NVGAMCTRL: negative voltage gamma control
 *
 * The command names are the same as those found in the datasheet to ease
 * looking up their semantics and usage.
 *
 * Note that the ST7789V display controller offers quite a few more commands
 * which have been omitted from this list as they are not used at the moment.
 * Furthermore, commands that are compliant with the MIPI DCS have been left
 * out as well to avoid duplicate entries.
 */
enum st7789v_command {
	TEOFF = 0x34,
	TEON = 0x35,
	STE = 0x44,
	PORCTRL = 0xB2,
	GCTRL = 0xB7,
	VCOMS = 0xBB,
	LCMCTRL = 0xC0,
	VDVVRHEN = 0xC2,
	VRHS = 0xC3,
	VDVS = 0xC4,
	VCMOFSET = 0xC5,
	FRCTRL2 = 0xC6,
	PWCTRL1 = 0xD0,
	PVGAMCTRL = 0xE0,
	NVGAMCTRL = 0xE1,
};

#define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define MADCTL_MV BIT(5) /* bitmask for page/column order */
#define MADCTL_MX BIT(6) /* bitmask for column address order */
#define MADCTL_MY BIT(7) /* bitmask for page address order */
#define MADCTL_ML BIT(4) /* bitmask for LCD vertical refresh order */
#define MADCTL_MH BIT(2) /* bitmask for LCD horizontal refresh order */

#define FRCTRL2_COL_INV	0xE0

/**
 * init_display() - initialize the display controller
 *
 * @par: FBTFT parameter object
 *
 * Most of the commands in this init function set their parameters to the
 * same default values which are already in place after the display has been
 * powered up. (The main exception to this rule is the pixel format which
 * would default to 18 instead of 16 bit per pixel.)
 * Nonetheless, this sequence can be used as a template for concrete
 * displays which usually need some adjustments.
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int init_display(struct fbtft_par *par)
{

#ifndef SAEF_SETTINGS
	/* Software reset */
	write_reg(par, 0x01);
	mdelay(5);

	/* turn off sleep mode */
	write_reg(par, 0x11);
	mdelay(120);

	write_reg(par, 0x36, 0x00);
	write_reg(par, 0x3A, 0x05);

	write_reg(par, 0xB0, 0x00, 0xF8); // RAMCTRL: little endian

	/* Back/front porch */
	/* A good alternative here to be sure the display read is done
	before the SPI write, is to reduce the back porch 
	and raise the front porch
	while leaving equal the sum of both */
#ifdef FBTFT_CONF2
	//write_reg(par, 0xB2,0x25,0x25,0x00,0x33,0x33); 
	write_reg(par, 0xB2,0x20,0x20,0x00,0x33,0x33); //ok 2
	//write_reg(par, 0xB2,0x15,0x15,0x00,0x33,0x33); 
	//write_reg(par, 0xB2,0x13,0x13,0x00,0x33,0x33); 
	//write_reg(par, 0xB2,0x11,0x11,0x00,0x33,0x33); 
	//write_reg(par, 0xB2,0x0F,0x0F,0x00,0x33,0x33); //ok 6
	//write_reg(par, 0xB2,0x0C,0x0C,0x00,0x33,0x33);
	//write_reg(par, 0xB2,0x0A,0x0A,0x00,0x33,0x33);
#else //FBTFT_CONF2
	write_reg(par, 0xB2,0x0C,0x0C,0x00,0x33,0x33);	//default (59.3Hz if FRCTRL2==15)
	//write_reg(par, 0xB2,0x09,0x09,0x00,0x33,0x33);	// Test for precise 60.4Hz (if FRCTRL2==15)
	//write_reg(par, 0xB2,0x07,0x07,0x00,0x33,0x33);	// Test for precise 61,1Hz (if FRCTRL2==15)
	//write_reg(par, 0xB2,0x7F,0x7F,0x00,0x33,0x33);	//max
#endif //FBTFT_CONF2
	
	write_reg(par, 0xB7,0x00);
	write_reg(par, 0xBB,0x36);
	
	//#warning tests LCMCTRL
	//write_reg(par, 0xC0,0x2C);	// default
	//write_reg(par, 0xC0,0x2D); 	// gate inversion
	//write_reg(par, 0xC0,0x2E);	// XOR MV setting

	write_reg(par, 0xC2,0x01);
	write_reg(par, 0xC3,0x13);
	write_reg(par, 0xC4,0x20);
	write_reg(par, 0xD6,0xA1);
	write_reg(par, 0xD0,0xA4,0xA1);
	/*write_reg(par, 0x21);
	write_reg(par, 0xE0,0x00,0x19,0x1E,0x0A,0x09,0x15,0x3D,0x44,0x51,0x12,0x03,0x00,0x3F,0x3F);
	write_reg(par, 0xE1,0x00,0x18,0x1E,0x0A,0x09,0x25,0x3F,0x43,0x52,0x33,0x03,0x00,0x3F,0x3F);
	write_reg(par, 0x29);*/

	//#warning tests GATECTRL
	//write_reg(par, 0xE4, 0x27, 0x00, 0x10);	// default
	//write_reg(par, 0xE4, 0x27, 0x00, 0x11);	// Gate scan direction 319->0
	//write_reg(par, 0xE4, 0x27, 0x00, 0x14);	// non-interlace mode
	//write_reg(par, 0xE4, 0x27, 0x20, 0x10);	// 320 gate lines, first can line is 159/319
	//write_reg(par, 0xE4, 0x1E, 0x00, 0x10);	// 240 gate lines, first line is 0
#else
	/* turn off sleep mode */
	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);

	/* set pixel format to RGB-565 */
	write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);

	write_reg(par, PORCTRL, 0x08, 0x08, 0x00, 0x22, 0x22);

	/*
	 * VGH = 13.26V
	 * VGL = -10.43V
	 */
	write_reg(par, GCTRL, 0x35);

	/*
	 * VDV and VRH register values come from command write
	 * (instead of NVM)
	 */
	write_reg(par, VDVVRHEN, 0x01, 0xFF);

	/*
	 * VAP =  4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 * VAN = -4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 */
	write_reg(par, VRHS, 0x0B);

	/* VDV = 0V */
	write_reg(par, VDVS, 0x20);

	/* VCOM = 0.9V */
	write_reg(par, VCOMS, 0x20);

	/* VCOM offset = 0V */
	write_reg(par, VCMOFSET, 0x20);

	/*
	 * AVDD = 6.8V
	 * AVCL = -4.8V
	 * VDS = 2.3V
	 */
	write_reg(par, PWCTRL1, 0xA4, 0xA1);
#endif

	/* Display Inversion of colors */
	write_reg(par, 0x21);

	/* Activate TE signal for Vsync only */
	write_reg(par, TEON, 0x00); // Mode 1 (only Vsync)
	//write_reg(par, TEON, 0x01); // Mode 2 (all Vsync and Hsync) - Not working: no TE interrupt received at all

	/* Set TE tearline */
	/*const uint16_t tearline=10;
	write_reg(par, STE, (tearline>>8), (tearline&0xff) );*/

	/* Refresh rate */
	u8 frctrl2_par = 0;
	//frctrl2_par |= FRCTRL2_COL_INV;

	/* This config bellow does not really change anything
		The TE frequency (that should happen at every Vsync)
		is always at the same frequency (58-59 times per sec). 
		Only changing the back and front porch size	
		really have an impact on this frequency.
		Careful to adjust SPI speed to avoid tearing
	*/
#ifdef FBTFT_CONF2
	frctrl2_par |= 0x12; //ok
	//frctrl2_par |= 0x13; 
	//frctrl2_par |= 0x14; // tearing (reduce SPI spead)
	//frctrl2_par |= 0x18; // tearing 
#else // not(FBTFT_CONF2)
	frctrl2_par |= 0x0F; // -> FKS v60fps
#endif //FBTFT_CONF2
	write_reg(par, FRCTRL2, frctrl2_par);

	/* Turn on display */
	write_reg(par, MIPI_DCS_SET_DISPLAY_ON);

	return 0;
}

/**
 * set_var() - apply LCD properties like rotation and BGR mode
 *
 * @par: FBTFT parameter object
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_var(struct fbtft_par *par)
{
	u8 madctl_par = 0;

	if (par->bgr)
		madctl_par |= MADCTL_BGR;

	switch (par->info->var.rotate) {
	case 0:
		break;
	case 90:
		madctl_par |= (MADCTL_MV | MADCTL_MY);
		break;
	case 180:
		madctl_par |= (MADCTL_MX | MADCTL_MY);
		break;
	case 270:
		madctl_par |= (MADCTL_MV | MADCTL_MX);
		break;
	default:
		return -EINVAL;
	}

	switch (par->pdata->rotate_soft) {
	case 0:
	case 180:
		break;
	case 90:
	case 270:
#ifdef FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
		madctl_par |= (MADCTL_MY | MADCTL_MX | MADCTL_ML | MADCTL_MH);		
		write_reg(par, 0xE4, 0x27, 0x00, 0x11);	// Gate scan direction 319->0
#endif //FBTFT_TRANSPOSE_INSTEAD_OF_ROTATE
		break;
	default:
		// not handled
		return -EINVAL;
	}

//#define HACK_TEST_LAST_TEARLINE
#ifdef HACK_TEST_LAST_TEARLINE
	#warning HACK_TEST_LAST_TEARLINE
	madctl_par |= (MADCTL_MY | MADCTL_MX | MADCTL_ML | MADCTL_MH);
#endif //HACK_TEST_LAST_TEARLINE

	write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);

	// All windows settings are surcharged after in fbtft_set_addr_win
	/* Xstart at 0 , Xend at 239 */
	write_reg(par, 0x2A, 0x00, 0x00, 0x00, 0xEF);

	/* Ystart at 0 , Yend at 319 */
	write_reg(par, 0x2B, 0x00, 0x00, 0x01, 0x3F);

	return 0;
}

/**
 * set_gamma() - set gamma curves
 *
 * @par: FBTFT parameter object
 * @curves: gamma curves
 *
 * Before the gamma curves are applied, they are preprocessed with a bitmask
 * to ensure syntactically correct input for the display controller.
 * This implies that the curves input parameter might be changed by this
 * function and that illegal gamma values are auto-corrected and not
 * reported as errors.
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_gamma(struct fbtft_par *par, u32 *curves)
{
	int i;
	int j;
	int c; /* curve index offset */

	/*
	 * Bitmasks for gamma curve command parameters.
	 * The masks are the same for both positive and negative voltage
	 * gamma curves.
	 */
	const u8 gamma_par_mask[] = {
		0xFF, /* V63[3:0], V0[3:0]*/
		0x3F, /* V1[5:0] */
		0x3F, /* V2[5:0] */
		0x1F, /* V4[4:0] */
		0x1F, /* V6[4:0] */
		0x3F, /* J0[1:0], V13[3:0] */
		0x7F, /* V20[6:0] */
		0x77, /* V36[2:0], V27[2:0] */
		0x7F, /* V43[6:0] */
		0x3F, /* J1[1:0], V50[3:0] */
		0x1F, /* V57[4:0] */
		0x1F, /* V59[4:0] */
		0x3F, /* V61[5:0] */
		0x3F, /* V62[5:0] */
	};

	for (i = 0; i < par->gamma.num_curves; i++) {
		c = i * par->gamma.num_values;
		for (j = 0; j < par->gamma.num_values; j++)
			curves[c + j] &= gamma_par_mask[j];
		write_reg(
			par, PVGAMCTRL + i,
			curves[c + 0], curves[c + 1], curves[c + 2],
			curves[c + 3], curves[c + 4], curves[c + 5],
			curves[c + 6], curves[c + 7], curves[c + 8],
			curves[c + 9], curves[c + 10], curves[c + 11],
			curves[c + 12], curves[c + 13]);
	}
	return 0;
}

/**
 * blank() - blank the display
 *
 * @par: FBTFT parameter object
 * @on: whether to enable or disable blanking the display
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int blank(struct fbtft_par *par, bool on)
{
	if (on)
		write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
	else
		write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = 240,
	.height = 320,
	.gamma_num = 2,
	.gamma_len = 14,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.init_display = init_display,
		.set_var = set_var,
		.set_gamma = set_gamma,
		.blank = blank,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st7789v", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st7789v");
MODULE_ALIAS("platform:st7789v");

MODULE_DESCRIPTION("FB driver for the ST7789V LCD Controller");
MODULE_AUTHOR("Dennis Menschel");
MODULE_LICENSE("GPL");
