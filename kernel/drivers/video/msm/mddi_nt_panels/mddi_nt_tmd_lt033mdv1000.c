/* drivers/video/msm/mddi_nt_panels/mddi_nt_tmd_lt033mdv1000.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Kenji Tokutake <Kenji.Tokutake@sonyericsson.com>
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
	{ 0x0000,    120 },	/* sleep_ms(120) */
	{ 0xC980, 0x0000 },
	{ 0xE780, 0x0000 },
	{ 0xF380, 0x00CC },
	{ 0xEE80, 0x0000 },
	{ 0x3500, 0x0000 },
	{ 0x0000,     10 },	/* sleep_ms(10) */
	{ 0, 0}
};

static const struct novatek_reg_set novatek_setup_regs[] = {
	{ 0x1100, 0x0000 },
	{ 0x0000,    120 },	/* sleep_ms(120) */
	{ 0xA280, 0x0006 },	/* Rotation mode */
	{ 0x2080, 0x0001 },	/* Gamma general setting */
	{ 0x2180, 0x0062 },
	{ 0x2480, 0x007C },	/* Gamma setting for Red */
	{ 0x2580, 0x0083 },
	{ 0x2680, 0x0083 },
	{ 0x2780, 0x0086 },
	{ 0x2880, 0x001A },
	{ 0x2980, 0x0033 },
	{ 0x2A80, 0x005D },
	{ 0x2B80, 0x0092 },
	{ 0x2D80, 0x0024 },
	{ 0x2F80, 0x002C },
	{ 0x3080, 0x00CB },
	{ 0x3180, 0x000F },
	{ 0x3280, 0x0021 },
	{ 0x3380, 0x002E },
	{ 0x3480, 0x00E7 },
	{ 0x3580, 0x00F7 },
	{ 0x3680, 0x00F5 },
	{ 0x3780, 0x0073 },
	{ 0x3880, 0x0020 },
	{ 0x3980, 0x0020 },
	{ 0x3A80, 0x0022 },
	{ 0x3B80, 0x002E },
	{ 0x3D80, 0x0050 },
	{ 0x3F80, 0x005D },
	{ 0x4080, 0x0070 },
	{ 0x4180, 0x0049 },
	{ 0x4280, 0x0012 },
	{ 0x4380, 0x001A },
	{ 0x4480, 0x0083 },
	{ 0x4580, 0x0021 },
	{ 0x4680, 0x004B },
	{ 0x4780, 0x0065 },
	{ 0x4880, 0x0090 },
	{ 0x4980, 0x0093 },
	{ 0x4A80, 0x0093 },
	{ 0x4B80, 0x001A },
	{ 0x4C80, 0x001A },	/* Gamma setting for Green */
	{ 0x4D80, 0x0036 },
	{ 0x4E80, 0x005D },
	{ 0x4F80, 0x007B },
	{ 0x5080, 0x0023 },
	{ 0x5180, 0x0037 },
	{ 0x5280, 0x0063 },
	{ 0x5380, 0x0099 },
	{ 0x5480, 0x0020 },
	{ 0x5580, 0x0027 },
	{ 0x5680, 0x00D0 },
	{ 0x5780, 0x0017 },
	{ 0x5880, 0x002F },
	{ 0x5980, 0x0048 },
	{ 0x5A80, 0x00BB },
	{ 0x5B80, 0x00DF },
	{ 0x5C80, 0x00F4 },
	{ 0x5D80, 0x0073 },
	{ 0x5E80, 0x0020 },
	{ 0x5F80, 0x001F },
	{ 0x6080, 0x0034 },
	{ 0x6180, 0x0059 },
	{ 0x6280, 0x0036 },
	{ 0x6380, 0x004D },
	{ 0x6480, 0x0067 },
	{ 0x6580, 0x0044 },
	{ 0x6680, 0x0018 },
	{ 0x6780, 0x001E },
	{ 0x6880, 0x007C },
	{ 0x6980, 0x001A },
	{ 0x6A80, 0x0047 },
	{ 0x6B80, 0x005C },
	{ 0x6C80, 0x009B },
	{ 0x6D80, 0x00B8 },
	{ 0x6E80, 0x00DF },
	{ 0x6F80, 0x0079 },
	{ 0x7080, 0x005D },	/* Gamma setting for Blue */
	{ 0x7180, 0x00AE },
	{ 0x7280, 0x00BE },
	{ 0x7380, 0x00C5 },
	{ 0x7480, 0x0022 },
	{ 0x7580, 0x0035 },
	{ 0x7680, 0x0062 },
	{ 0x7780, 0x00B0 },
	{ 0x7880, 0x0021 },
	{ 0x7980, 0x0027 },
	{ 0x7A80, 0x00DA },
	{ 0x7B80, 0x0012 },
	{ 0x7C80, 0x0030 },
	{ 0x7D80, 0x0043 },
	{ 0x7E80, 0x00BE },
	{ 0x7F80, 0x00DB },
	{ 0x8080, 0x00F5 },
	{ 0x8180, 0x0073 },
	{ 0x8280, 0x0020 },
	{ 0x8380, 0x0020 },
	{ 0x8480, 0x0038 },
	{ 0x8580, 0x0055 },
	{ 0x8680, 0x003B },
	{ 0x8780, 0x004D },
	{ 0x8880, 0x006B },
	{ 0x8980, 0x003A },
	{ 0x8A80, 0x0019 },
	{ 0x8B80, 0x001E },
	{ 0x8C80, 0x0063 },
	{ 0x8D80, 0x001F },
	{ 0x8E80, 0x004C },
	{ 0x8F80, 0x005E },
	{ 0x9080, 0x0050 },
	{ 0x9180, 0x0057 },
	{ 0x9280, 0x0067 },
	{ 0x9380, 0x0038 },
	{ 0, 0}
};

static const struct novatek_reg_set novatek_display_on_regs[] = {
	{ 0x2900, 0x0000 },	/* SET_DISPLAY_ON */
	{ 0, 0}
};

static const struct novatek_reg_set novatek_display_off_regs[] = {
	{ 0x2800, 0x0000 },	/* SET_DISPLAY_OFF */
	{ 0x0000,     17 },	/* sleep_ms(17) */
	{ 0, 0 }
};

static const struct novatek_reg_set novatek_takedown_regs[] = {
	{ 0x1000, 0x0000 },
	{ 0x0000,     70 },	/* sleep_ms(70) */
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
	novatek_panel_data.panel_info.lcd.refx100 = 6350;
	novatek_panel_data.panel_info.lcd.v_back_porch = 10;
	novatek_panel_data.panel_info.lcd.v_front_porch = 4;
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

const struct panel_id novatek_panel_id_tmd_lt033mdv1000 = {
	.name = "TMD MDDI Type 2 LT033MDV1000",
	.reg_count = 2,
	.regs = { {0xDC00, 0x08}, {0xDC00, 0x08} },
	.pinfo = &novatek_controller_panel,
	.mddi_type = 2,
	.width = 41,
	.height = 73,
	.suspend_support = 1,
	.esd_support = 1,
};

