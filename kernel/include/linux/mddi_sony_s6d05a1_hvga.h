#ifndef __ARCH_ARM_MACH_MSM_MDDI_SONY_S6D05A1_HVGA_H
#define __ARCH_ARM_MACH_MSM_MDDI_SONY_S6D05A1_HVGA_H

#define MDDI_SONY_S6D05A1_HVGA_NAME "mddi_sony_s6d05a1_hvga"

struct sony_hvga_platform_data {
	void (*power_on)(void);
	void (*power_off)(void);
	void (*window_adjust)(u16 x1, u16 x2, u16 y1, u16 y2);
	void (*exit_deep_standby) (void);
};

#endif

