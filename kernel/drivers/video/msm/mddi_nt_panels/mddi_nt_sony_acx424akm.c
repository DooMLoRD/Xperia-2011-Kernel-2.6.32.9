/* drivers/video/msm/mddi_nt_panels/mddi_nt_sony_acx424akm.c
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

static const struct novatek_reg_set novatek_init_regs[] = {
	{ 0xAE00, 0x0003 },	/* Set MDDI1.2 2Lane */

	/* LED PWM */
	{ 0x22C0, 0x0002 },
	{ 0x5100, 0x00FF },
	{ 0x5E00, 0x0000 },
	{ 0x5300, 0x002C },
	{ 0x5500, 0x0003 },

	/* FTE */
	{ 0x4400, 0x0000 },	/* SET_TEAR_SCANLINE */
	{ 0x4401, 0x0000 },	/* SET_TEAR_SCANLINE */
	{ 0x3500, 0x0000 },	/* SET_TEAR_ON */

	/* Rotation Function */
	{ 0xC980, 0x0001 },
	{ 0xA280, 0x0003 },

	/* DDC */
	{ 0xC980, 0x0001 },
	{ 0x0980, 0x0034 },

	{ 0x3600, 0x0000 },	/* SET_ADDRESS_MODE */
	{ 0, 0}
};

static const struct novatek_reg_set novatek_setup_regs[] = {
	{ 0x1100, 0x0000 },
	{ 0x0000,     10 },	/* sleep_ms(10) */
	{ 0, 0}
};

static const struct novatek_reg_set novatek_display_on_regs[] = {
	{ 0x2900, 0x0000 },	/* SET_DISPLAY_ON */
	{ 0, 0}
};

static const struct novatek_reg_set novatek_display_off_regs[] = {
	{ 0x2800, 0x0000 },	/* SET_DISPLAY_OFF */
	{ 0x0000,     30 },	/* sleep_ms(30) */
	{ 0, 0 }
};

static const struct novatek_reg_set novatek_takedown_regs[] = {
	{ 0x1000, 0x0000 },
	{ 0x0000,    180 },	/* sleep_ms(180) */
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
	novatek_panel_data.panel_info.clk_rate = 192000000;
	novatek_panel_data.panel_info.clk_min =  190000000;
	novatek_panel_data.panel_info.clk_max =  200000000;
	novatek_panel_data.panel_info.bl_max = 4;
	novatek_panel_data.panel_info.bl_min = 1;
	novatek_panel_data.panel_info.fb_num = 2;
	novatek_panel_data.panel_info.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	novatek_panel_data.panel_info.lcd.refx100 = 6000;
	novatek_panel_data.panel_info.lcd.v_back_porch = 12;
	novatek_panel_data.panel_info.lcd.v_front_porch = 14;
	novatek_panel_data.panel_info.lcd.v_pulse_width = 0;
	novatek_panel_data.panel_info.lcd.vsync_notifier_period = 0;

	return &novatek_panel_data;
}

static struct novatek_controller novatek_controller_panel = {
	.init		= novatek_init_regs,
	.setup		= novatek_setup_regs,
	.turn_on	= novatek_display_on_regs,
	.turn_off	= novatek_display_off_regs,
	.takedown	= novatek_takedown_regs,
	.standby	= novatek_standby_regs,
	.get_panel_info = get_panel_info,
};

const struct panel_id novatek_panel_id_sony_acx424akm_type1 = {
	.name = "Sony MDDI Type 1 ACX424AKM",
	.reg_count = 2,
	.regs = { {0xDB00, 0x80}, {0xDC00, 0x02} },
	.pinfo = &novatek_controller_panel,
	.mddi_type = 1,
	.width = 46,
	.height = 82,
	.suspend_support = 1,
	.esd_support = 0,
};

const struct panel_id novatek_panel_id_sony_acx424akm = {
	.name = "Sony MDDI Type 2 ACX424AKM",
	.reg_count = 2,
	.regs = { {0xDA00, 0x01}, {0xDC00, 0x02} },
	.pinfo = &novatek_controller_panel,
	.width = 46,
	.height = 82,
	.mddi_type = 2,
	.suspend_support = 1,
	.esd_support = 1,
};

