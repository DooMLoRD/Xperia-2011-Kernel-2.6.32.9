#ifndef __ARCH_ARM_MACH_MSM_MDDI_HITACHI_R61529_HVGA_H
#define __ARCH_ARM_MACH_MSM_MDDI_HITACHI_R61529_HVGA_H

#define MDDI_HITACH_R61529_HVGA_NAME "mddi_hitachi_r61529_hvga"

#include "../../drivers/video/msm/msm_fb_panel.h"

enum {
	DBC_MODE_UI,
	DBC_MODE_IMAGE,
	DBC_MODE_VIDEO,
};

struct hitachi_hvga_platform_data {
	void (*power_on)(void);
	void (*power_off)(void);
	void (*window_adjust)(u16 x1, u16 x2, u16 y1, u16 y2);
	void (*exit_deep_standby) (void);
	int dbc_on;
	int dbc_mode;
	struct msm_fb_panel_data *panel_data;
};

#endif
