/* drivers/video/msm/mddi_nt_panels/mddi_nt_panel.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


/*
 * TODO: Add describing comments.
 */

#ifndef MDDI_NT_PANEL_H
#define MDDI_NT_PANEL_H

#include <linux/types.h>

struct novatek_controller {
	const struct novatek_reg_set *init;
	const struct novatek_reg_set *standby;
	const struct novatek_reg_set *resume;

	const struct novatek_reg_set *setup;
	const struct novatek_reg_set *takedown;

	const struct novatek_reg_set *turn_on;
	const struct novatek_reg_set *turn_off;

	struct msm_fb_panel_data *(*get_panel_info) (void);
};

struct panel_reg {
	u16	addr;
	u16	value;
};

struct panel_id {
	const char			*name;
	const int			reg_count;
	const struct panel_reg		regs[2];
	struct novatek_controller	*pinfo;
	const int			mddi_type;
	const u32			width;	/* in mm */
	const u32			height;	/* in mm */
	const int			suspend_support;
	const int			esd_support;
};

struct novatek_reg_set {
	u16 reg;
	u16 val;
};

#endif
