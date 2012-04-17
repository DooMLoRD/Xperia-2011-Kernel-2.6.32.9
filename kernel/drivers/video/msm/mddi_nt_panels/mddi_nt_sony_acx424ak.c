/* drivers/video/msm/mddi_nt_panels/mddi_nt_sony_acx424ak.c
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

#ifdef CONFIG_MACH_SEMC_IYOKAN
	#define REFRESH_RATE 6400
#else
	#define REFRESH_RATE 6500
#endif

static const struct novatek_reg_set novatek_init_regs[] = {
	{ 0xAE00, 0x0003 },	/* Set MDDI1.2 2Lane */

	/* LED PWM */
	{ 0x5500, 0x0000 },
	{ 0x5300, 0x0000 },

	/* FTE */
	{ 0x4400, 0x0000 },	/* SET_TEAR_SCANLINE */
	{ 0x4401, 0x0000 },	/* SET_TEAR_SCANLINE */
	{ 0x3500, 0x0000 },	/* SET_TEAR_ON */

	/* Gamma 8.1.6.3 */
	{ 0xC980, 0x0001 },

	{ 0x2480, 0x0005 },
	{ 0x2580, 0x0009 },
	{ 0x2680, 0x0015 },
	{ 0x2780, 0x001E },
	{ 0x2880, 0x0014 },
	{ 0x2980, 0x0026 },
	{ 0x2A80, 0x005B },
	{ 0x2B80, 0x0039 },
	{ 0x2D80, 0x0021 },
	{ 0x2F80, 0x0027 },
	{ 0x3080, 0x0070 },
	{ 0x3180, 0x0017 },
	{ 0x3280, 0x0039 },
	{ 0x3380, 0x0052 },
	{ 0x3480, 0x0058 },
	{ 0x3580, 0x00DB },
	{ 0x3680, 0x00F6 },
	{ 0x3780, 0x007F },
	{ 0x3880, 0x0005 },
	{ 0x3980, 0x0009 },
	{ 0x3A80, 0x0015 },
	{ 0x3B80, 0x001E },
	{ 0x3D80, 0x0014 },
	{ 0x3F80, 0x0026 },
	{ 0x4080, 0x005B },
	{ 0x4180, 0x0039 },
	{ 0x4280, 0x0021 },
	{ 0x4380, 0x0027 },
	{ 0x4480, 0x0070 },
	{ 0x4580, 0x0017 },
	{ 0x4680, 0x0039 },
	{ 0x4780, 0x0052 },
	{ 0x4880, 0x0058 },
	{ 0x4980, 0x00DB },
	{ 0x4A80, 0x00F6 },
	{ 0x4B80, 0x007F },

	{ 0x4C80, 0x0025 },
	{ 0x4D80, 0x002B },
	{ 0x4E80, 0x003D },
	{ 0x4F80, 0x004C },
	{ 0x5080, 0x001D },
	{ 0x5180, 0x0032 },
	{ 0x5280, 0x0061 },
	{ 0x5380, 0x004A },
	{ 0x5480, 0x0020 },
	{ 0x5580, 0x0027 },
	{ 0x5680, 0x0078 },
	{ 0x5780, 0x0019 },
	{ 0x5880, 0x003E },
	{ 0x5980, 0x0054 },
	{ 0x5A80, 0x0056 },
	{ 0x5B80, 0x008D },
	{ 0x5C80, 0x00E3 },
	{ 0x5D80, 0x007F },
	{ 0x5E80, 0x0025 },
	{ 0x5F80, 0x002B },
	{ 0x6080, 0x003D },
	{ 0x6180, 0x004C },
	{ 0x6280, 0x001D },
	{ 0x6380, 0x0032 },
	{ 0x6480, 0x0061 },
	{ 0x6580, 0x004A },
	{ 0x6680, 0x0020 },
	{ 0x6780, 0x0027 },
	{ 0x6880, 0x0078 },
	{ 0x6980, 0x0019 },
	{ 0x6A80, 0x003E },
	{ 0x6B80, 0x0054 },
	{ 0x6C80, 0x0056 },
	{ 0x6D80, 0x008D },
	{ 0x6E80, 0x00E3 },
	{ 0x6F80, 0x007F },

	{ 0x7080, 0x0036 },
	{ 0x7180, 0x0049 },
	{ 0x7280, 0x0071 },
	{ 0x7380, 0x0074 },
	{ 0x7480, 0x0018 },
	{ 0x7580, 0x0030 },
	{ 0x7680, 0x0064 },
	{ 0x7780, 0x005E },
	{ 0x7880, 0x001A },
	{ 0x7980, 0x0022 },
	{ 0x7A80, 0x0088 },
	{ 0x7B80, 0x0016 },
	{ 0x7C80, 0x003E },
	{ 0x7D80, 0x0054 },
	{ 0x7E80, 0x0064 },
	{ 0x7F80, 0x00B5 },
	{ 0x8080, 0x00EC },
	{ 0x8180, 0x007F },
	{ 0x8280, 0x0036 },
	{ 0x8380, 0x0049 },
	{ 0x8480, 0x0071 },
	{ 0x8580, 0x0074 },
	{ 0x8680, 0x0018 },
	{ 0x8780, 0x0030 },
	{ 0x8880, 0x0064 },
	{ 0x8980, 0x005E },
	{ 0x8A80, 0x001A },
	{ 0x8B80, 0x0022 },
	{ 0x8C80, 0x0088 },
	{ 0x8D80, 0x0016 },
	{ 0x8E80, 0x003E },
	{ 0x8F80, 0x0054 },
	{ 0x9080, 0x0064 },
	{ 0x9180, 0x00B5 },
	{ 0x9280, 0x00EC },
	{ 0x9380, 0x007F },
	/* Gamma 8.1.6.3 End */

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
	{ 0, 0}
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
	novatek_panel_data.panel_info.lcd.refx100 = REFRESH_RATE;
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

const struct panel_id novatek_panel_id_sony_acx424ak = {
	.name = "Sony MDDI Type 2 ACX424AK",
	.reg_count = 2,
	.regs = { {0xDA00, 0x01}, {0xDC00, 0x05} },
	.pinfo = &novatek_controller_panel,
	.mddi_type = 2,
	.width = 46,
	.height = 82,
	.suspend_support = 1,
	.esd_support = 1,
};

