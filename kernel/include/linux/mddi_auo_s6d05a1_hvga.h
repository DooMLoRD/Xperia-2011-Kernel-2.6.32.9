#ifndef __ARCH_ARM_MACH_MSM_MDDI_AUO_S6D05A1_HVGA_H
#define __ARCH_ARM_MACH_MSM_MDDI_AUO_S6D05A1_HVGA_H

#define MDDI_AUO_S6D05A1_HVGA_NAME "mddi_auo_s6d05a1_hvga"

#include <linux/msm_fb_panel.h>

enum {
	DBC_MODE_UI,
	DBC_MODE_IMAGE,
	DBC_MODE_VIDEO,
};

struct auo_hvga_platform_data {
	void (*power_on)(void);
	void (*power_off)(void);
	void (*window_adjust)(u16 x1, u16 x2, u16 y1, u16 y2);
	void (*exit_deep_standby) (void);
	int dbc_on;
	int dbc_mode;
	struct msm_fb_panel_data *panel_data;
};

#endif

