#ifndef _SII9024_H_
#define _SII9024_H_

#define HDMI_SII_GPIO_HPD	93
#define HDMI_SII_GPIO_POWER	102
#define HDMI_SII_GPIO_RESET	105

struct sii9024_platform_data {
	int (*setchippower)(int);
	int xres;
	int yres;
	int type;
	int pdest;
	int wait_cycle;
	int bpp;
	int fb_num;
	int clk_rate;
	int lcdc_h_back_porch;
	int lcdc_h_front_porch;
	int lcdc_h_pulse_width;
	int lcdc_v_back_porch;
	int lcdc_v_front_porch;
	int lcdc_v_pulse_width;
	int lcdc_border_clr;
	int lcdc_underflow_clr;
	int lcdc_hsync_skew;
};

struct hdmi_sii_state {
	struct kobject kobj;
	int hpd;
	int power;
	int reset;
};

#endif
