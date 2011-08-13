#ifndef __ARCH_ARM_MACH_MSM_FB_PANEL_H
#define __ARCH_ARM_MACH_MSM_FB_PANEL_H

struct platform_device;

struct msm_fb_data_type;

typedef void (*msm_fb_vsync_handler_type) (void *arg);

/* panel device locaiton */
typedef enum {
	DISPLAY_1 = 0,		/* attached as first device */
	DISPLAY_2,		/* attached on second device */
	MAX_PHYS_TARGET_NUM,
} DISP_TARGET_PHYS;

/* panel type list */
#define NO_PANEL		0xffff	/* No Panel */
#define MDDI_PANEL		1	/* MDDI */
#define EBI2_PANEL		2	/* EBI2 */
#define LCDC_PANEL		3	/* internal LCDC type */
#define EXT_MDDI_PANEL		4	/* Ext.MDDI */
#define TV_PANEL		5	/* TV */
#define HDMI_PANEL		6	/* HDMI TV */
#define DTV_PANEL		7	/* DTV */
#define MIPI_VIDEO_PANEL	8	/* MIPI */
#define MIPI_CMD_PANEL		9	/* MIPI */

/* panel info type */
struct lcd_panel_info {
	__u32 vsync_enable;
	__u32 refx100;
	__u32 v_back_porch;
	__u32 v_front_porch;
	__u32 v_pulse_width;
	__u32 hw_vsync_mode;
	__u32 vsync_notifier_period;
	__u32 rev;
};

struct lcdc_panel_info {
	__u32 h_back_porch;
	__u32 h_front_porch;
	__u32 h_pulse_width;
	__u32 v_back_porch;
	__u32 v_front_porch;
	__u32 v_pulse_width;
	__u32 border_clr;
	__u32 underflow_clr;
	__u32 hsync_skew;
};

struct mddi_panel_info {
	__u32 vdopkt;
};

struct msm_panel_info {
	__u32 xres;
	__u32 yres;
	__u32 bpp;
	__u32 type;
	__u32 wait_cycle;
	DISP_TARGET_PHYS pdest;
	__u32 bl_max;
	__u32 bl_min;
	__u32 fb_num;
	__u32 clk_rate;
	__u32 clk_min;
	__u32 clk_max;
	__u32 frame_count;

	/* physical size in mm */
	__u32 width;
	__u32 height;

	union {
		struct mddi_panel_info mddi;
	};

	union {
		struct lcd_panel_info lcd;
		struct lcdc_panel_info lcdc;
	};
};

struct msm_fb_panel_data {
	struct msm_panel_info panel_info;
	void (*set_rect) (int x, int y, int xres, int yres);
	void (*set_vsync_notifier) (msm_fb_vsync_handler_type, void *arg);
	void (*set_backlight) (struct msm_fb_data_type *);

	/* function entry chain */
	int (*on) (struct platform_device *pdev);
	int (*controller_on_panel_on) (struct platform_device *pdev);
	int (*off) (struct platform_device *pdev);
	void (*window_adjust)(u16 x1, u16 x2, u16 y1, u16 y2);
	int power_on_panel_at_pan;
	struct platform_device *next;
	int (*clk_func) (int enable);
};

#endif
