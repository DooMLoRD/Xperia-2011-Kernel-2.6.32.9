/* drivers/video/msm/mddi_nt_panels/mddi_nt_sharp_ls040t8lx01.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "mddihost.h"
#include "msm_fb_panel.h"
#include "mddi_nt_panel.h"

/* ------------------------------------------------------------------- *
 *	Revision C
 * ------------------------------------------------------------------- */
static const struct novatek_reg_set novatek_init_regs_rev_c[] = {
	{ 0xFB80, 0x0001 },
	{ 0x0180, 0x0014 },
	{ 0x0280, 0x0000 },	/* AVDDP1, AVDDP2=? */
	{ 0x0480, 0x003D },	/* AVDDP1_SEL=?, AVDDP2_SEL=? */
	{ 0x0780, 0x0000 },	/* 1/display clock=0.1us */
	{ 0x0880, 0x0000 },	/* Set frequency (DCLK) of AVDDP1,
				 * AVDDP2 generating circuit */
	{ 0x0980, 0x0004 },	/* Set frequency (DCLK) of VGH/VGL
				 * generating circuit */
	{ 0x0A80, 0x0042 },	/* & Source output level for porch and
				 * blanking (Hi-Z) */
	{ 0x1280, 0x0000 },	/* pre-charge setting */
	{ 0x1380, 0x0010 },	/* VGH=2XVR, VGL=-1VR */
	{ 0x1480, 0x0000 },	/* VR=4.1V */
	{ 0x1580, 0x00A0 },	/* GVDDP=4.8V */
	{ 0x1A80, 0x0064 },	/* VCOMDC1=2.0V */
	{ 0x1F80, 0x0000 },	/* VCMM=0x00, DCVCOM1 */
	{ 0x2080, 0x0001 },
	{ 0x2180, 0x0073 },	/* VCOMH=4.0125V */
	{ 0x9480, 0x00B4 },	/* STV Start Setting */
	{ 0x9580, 0x0000 },	/* STV End setting */
	{ 0x9680, 0x0000 },	/* CKV Start Setting */
	{ 0x9780, 0x00AA },	/* CKV End Setting */
	{ 0x9880, 0x000B },	/* ASW 1-3 Start Setting */
	{ 0x9980, 0x002A },	/* ASW 1-3 High width setting */
	{ 0x9A80, 0x0009 },	/* ASW 1-3 Low width setting */
	{ 0x9B80, 0x0001 },	/* Pre-charge start setting */
	{ 0x9C80, 0x0001 },	/* Pre-charge high width setting */
	{ 0x9D80, 0x0000 },
	{ 0x9E80, 0x0000 },	/* OEV start and width setting */
	{ 0x9F80, 0x0000 },	/* Set start position of VCOM polarity
				 * inversion */
	{ 0xA080, 0x0003 },	/* Source hold time */
	{ 0xA280, 0x0006 },	/* CTB=1 vertical fresh order
				 * (GATE Reverse scan) */
	{ 0xA380, 0x002E },	/* Enable output => CGOUT6/4/3/2
				 * (STV/CKV2/CKV1/UD) */
	{ 0xA480, 0x000E },	/* Enable output => CGOUT12/11/10
				 * (OEV/FR/FDON) */
	{ 0xA580, 0x00C0 },	/* Enable output => CGOUT24/23(ASW2/ASW3) */
	{ 0xA680, 0x0001 },	/* Polarity CGOUT25 = High,
				 * enable output=>CGOUT25(ASW1) */
	{ 0xA780, 0x0000 },	/* Polarity CGOUT8-1 = High */
	{ 0xA980, 0x0000 },	/* Polarity CGOUT12(FR) = ? Others
				 * (CGOUT(15~12,10~08) = ? */
	{ 0xAA80, 0x0000 },	/* Polarity CGOUT24-17 = High */
	{ 0xE780, 0x0000 },	/* Inversion setting (Normal Idle,
				 * Partial Mode = Line inversion) */
	{ 0xEE80, 0x0080 },	/* Command lock disable */
	{ 0xED80, 0x0000 },	/* VACT Line = 0 */
	{ 0xFB80, 0x0000 },
	{ 0x8CC9, 0x0040 },	/* Disable internal resistors */
	{ 0x8CC1, 0x0040 },
	{ 0xFBC0, 0x0001 },
	{ 0x89CE, 0x0003 },
	{ 0x8FCE, 0x0013 },
	{ 0x8FCF, 0x0011 },
	{ 0x90C0, 0x0013 },
	{ 0x90C1, 0x0003 },
	{ 0x68C0, 0x0008 },
	{ 0xF380, 0x00CC },
	{ 0x3500, 0x0000 },	/* Set TE ON */
	{ 0x4400, 0x0001 },
	{ 0x4401, 0x0000 },	/* Set Tear line */
	{ 0x3600, 0x00C0 },
	{0, 0}
};

static const struct novatek_reg_set novatek_setup_regs_rev_c[] = {
	{ 0x1100, 0x0000 },
	{ 0x0000,    120 },	/* sleep_ms(120) */
	{ 0x2A00, 0x0000 },	/* SET_HORIZONTAL_ADDRESS_0 */
	{ 0x2A01, 0x0000 },	/* SET_HORIZONTAL_ADDRESS_1 */
	{ 0x2A02, 0x0001 },	/* SET_HORIZONTAL_ADDRESS_2 */
	{ 0x2A03, 0x00DF },	/* SET_HORIZONTAL_ADDRESS_3 */
	{ 0x2B00, 0x0000 },	/* SET_VERTICAL_ADDRESS_0 */
	{ 0x2B01, 0x0000 },	/* SET_VERTICAL_ADDRESS_1 */
	{ 0x2B02, 0x0003 },	/* SET_VERTICAL_ADDRESS_2 */
	{ 0x2B03, 0x0055 },	/* SET_VERTICAL_ADDRESS_3 */
	{ 0x2D00, 0x0001 },	/* SET_RAM_ADDRESS_0 */
	{ 0x2D01, 0x00DF },	/* SET_RAM_ADDRESS_1 */
	{ 0x2D02, 0x0003 },	/* SET_RAM_ADDRESS_2 */
	{ 0x2D03, 0x0055 },	/* SET_RAM_ADDRESS_3 */
	{0, 0}
};

static const struct novatek_reg_set novatek_display_on_regs_rev_c[] = {
	{ 0x2900, 0x0000 },	/* SET_DISPLAY_ON */
	{ 0x0280, 0x0000 },
	{ 0x0880, 0x0044 },
	{ 0x0980, 0x0054 },
	{ 0x1480, 0x000D },
	{ 0x5300, 0x002C },
	{ 0x5100, 0x00FF },	/* LED ON */
	{ 0x5500, 0x0003 },
	{ 0x3BC0, 0x00FF },
	{0, 0}
};

/* ------------------------------------------------------------------- *
 *	Revision D - Sequence corresponds to Rev 03 except the
 * 			MDDI link up/down
 * ------------------------------------------------------------------- */
static const struct novatek_reg_set novatek_init_regs_rev_d[] = {
	{ 0x1100, 0x0000 }, /* EXIT_SLEEP_MODE */
	{ 0x0000,    120 }, /* sleep 120 ms */
	{0, 0}
};

static const struct novatek_reg_set novatek_setup_regs_rev_d[] = {
	{ 0x3500, 0x0000 }, /* Set TE ON */
	{ 0x4400, 0x0001 }, /* Set TE scanline */
	{ 0x4401, 0x0000 }, /* Set TE scanline */
	{ 0x2A00, 0x0000 }, /* SET_HORIZONTAL_ADDRESS */
	{ 0x2A01, 0x0000 }, /* SET_HORIZONTAL_ADDRESS */
	{ 0x2A02, 0x0001 }, /* SET_HORIZONTAL_ADDRESS */
	{ 0x2A03, 0x00DF }, /* SET_HORIZONTAL_ADDRESS */
	{ 0x2B00, 0x0000 }, /* SET_VERTICAL_ADDRESS */
	{ 0x2B01, 0x0000 }, /* SET_VERTICAL_ADDRESS */
	{ 0x2B02, 0x0003 }, /* SET_VERTICAL_ADDRESS */
	{ 0x2B03, 0x0055 }, /* SET_VERTICAL_ADDRESS */
	{ 0x2D00, 0x0000 }, /* SET_RAM_ADDRESS */
	{ 0x2D01, 0x0000 }, /* SET_RAM_ADDRESS */
	{ 0x2D02, 0x0003 }, /* SET_RAM_ADDRESS */
	{ 0x2D03, 0x0055 }, /* SET_RAM_ADDRESS */
	{ 0x3600, 0x0044 }, /* SET_ADDRESS_MODE */
	{ 0x3A00, 0x0077 }, /* SET_24Bit Pixel */
	{0, 0}
};

static const struct novatek_reg_set novatek_display_on_regs_rev_d[] = {
	{ 0x2900, 0x0000 }, /* SET_DISPLAY_ON */
	{ 0x5300, 0x002C },
	{ 0x5100, 0x00FF },	/* LED ON */
	{ 0x5500, 0x0003 },
	{ 0x3BC0, 0x00FF },
	{0, 0}
};

/* ------------------------------------------------------------------- *
 *	Common Setting - Sequence corresponds to Rev 03 with deep
 *			standby as power off mode. MDDI link up/down
 * 			is not implemented
 * ------------------------------------------------------------------- */
static const struct novatek_reg_set novatek_display_off_regs[] = {
	{ 0x5300, 0x0000 },
	{ 0xEE80, 0x0080 },
	{ 0x2800, 0x0000 },	/* SET_DISPLAY_OFF */
	{ 0, 0 }
};

static const struct novatek_reg_set novatek_takedown_regs[] = {
	{ 0x1000, 0x0000 },	/* SLEEP IN */
	{ 0x0000,    120 },	/* sleep 120 ms */
	{ 0, 0 }
};

static const struct novatek_reg_set novatek_standby_regs[] = {
	{ 0x4F00, 0x0001 },
	{ 0, 0 }
};

static struct msm_fb_panel_data novatek_panel_data;

static struct msm_fb_panel_data *get_panel_info(void)
{
	novatek_panel_data.panel_info.xres = 480;
	novatek_panel_data.panel_info.yres = 854;
	novatek_panel_data.panel_info.bpp = 24;
	novatek_panel_data.panel_info.type = MDDI_PANEL;
	novatek_panel_data.panel_info.wait_cycle = 0;
	novatek_panel_data.panel_info.pdest = DISPLAY_1;
	novatek_panel_data.panel_info.fb_num = 2;
	novatek_panel_data.panel_info.clk_rate = 192000000;
	novatek_panel_data.panel_info.clk_min =  190000000;
	novatek_panel_data.panel_info.clk_max =  200000000;
	novatek_panel_data.panel_info.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	novatek_panel_data.panel_info.lcd.refx100 = 6164;
	novatek_panel_data.panel_info.lcd.v_back_porch = 14;
	novatek_panel_data.panel_info.lcd.v_front_porch = 2;
	novatek_panel_data.panel_info.lcd.v_pulse_width = 0;
	novatek_panel_data.panel_info.lcd.vsync_notifier_period = 0;

	return &novatek_panel_data;
}

static struct novatek_controller novatek_controller_panel_rev_c = {
	.init		= novatek_init_regs_rev_c,
	.setup		= novatek_setup_regs_rev_c,
	.turn_on	= novatek_display_on_regs_rev_c,
	.turn_off	= novatek_display_off_regs,
	.takedown	= novatek_takedown_regs,
	.standby	= novatek_standby_regs,
	.get_panel_info = get_panel_info,
};

static struct novatek_controller novatek_controller_panel_rev_d = {
	.init		= novatek_init_regs_rev_d,
	.setup		= novatek_setup_regs_rev_d,
	.turn_on	= novatek_display_on_regs_rev_d,
	.turn_off	= novatek_display_off_regs,
	.takedown	= novatek_takedown_regs,
	.standby	= novatek_standby_regs,
	.get_panel_info = get_panel_info,
};

const struct panel_id novatek_panel_id_sharp_ls040t8lx01_rev_c_x = {
	.name = "Sharp MDDI Type 2 (cut1)",
	.reg_count = 2,
	.regs = { {0xDA00, 0x70}, {0xDC00, 0x0A} },
	.pinfo = &novatek_controller_panel_rev_c,
	.mddi_type = 1,
	.width = 51,
	.height = 89,
	.suspend_support = 1,
	.esd_support = 1,
};

const struct panel_id novatek_panel_id_sharp_ls040t8lx01_rev_c = {
	.name = "Sharp MDDI Type 2 (cut1)",
	.reg_count = 2,
	.regs = { {0xDA00, 0x70}, {0xDC00, 0x00} },
	.pinfo = &novatek_controller_panel_rev_c,
	.mddi_type = 1,
	.width = 51,
	.height = 89,
	.suspend_support = 1,
	.esd_support = 1,
};

const struct panel_id novatek_panel_id_sharp_ls040t8lx01_rev_d = {
	.name = "Sharp MDDI Type 2 (revision D)",
	.reg_count = 2,
	.regs = { {0xDA00, 0x70}, {0xDC00, 0x0B} },
	.pinfo = &novatek_controller_panel_rev_d,
	.mddi_type = 2,
	.width = 51,
	.height = 89,
	.suspend_support = 1,
	.esd_support = 1,
};

