/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#ifndef __HDMI_COMMON_H__
#define __HDMI_COMMON_H__

#ifdef DEBUG
#define DEV_DBG(args...)	pr_info("HDMI: " args)
#else
#define DEV_DBG(args...)	(void)0
#endif /* DEBUG */
#define DEV_INFO(args...)	dev_info(hdmi_common_state->dev, args)
#define DEV_WARN(args...)	dev_warn(hdmi_common_state->dev, args)
#define DEV_ERR(args...)	dev_err(hdmi_common_state->dev, args)

/* all video formats defined by EIA CEA 861D */
#define HDMI_VFRMT_640x480p60_4_3	0
#define HDMI_VFRMT_720x480p60_4_3	1
#define HDMI_VFRMT_720x480p60_16_9	2
#define HDMI_VFRMT_1280x720p60_16_9	3
#define HDMI_VFRMT_1920x1080i60_16_9	4
#define HDMI_VFRMT_720x480i60_4_3	5
#define HDMI_VFRMT_1440x480i60_4_3	HDMI_VFRMT_720x480i60_4_3
#define HDMI_VFRMT_720x480i60_16_9	6
#define HDMI_VFRMT_1440x480i60_16_9	HDMI_VFRMT_720x480i60_16_9
#define HDMI_VFRMT_720x240p60_4_3	7
#define HDMI_VFRMT_1440x240p60_4_3	HDMI_VFRMT_720x240p60_4_3
#define HDMI_VFRMT_720x240p60_16_9	8
#define HDMI_VFRMT_1440x240p60_16_9	HDMI_VFRMT_720x240p60_16_9
#define HDMI_VFRMT_2880x480i60_4_3	9
#define HDMI_VFRMT_2880x480i60_16_9	10
#define HDMI_VFRMT_2880x240p60_4_3	11
#define HDMI_VFRMT_2880x240p60_16_9	12
#define HDMI_VFRMT_1440x480p60_4_3	13
#define HDMI_VFRMT_1440x480p60_16_9	14
#define HDMI_VFRMT_1920x1080p60_16_9	15
#define HDMI_VFRMT_720x576p50_4_3	16
#define HDMI_VFRMT_720x576p50_16_9	17
#define HDMI_VFRMT_1280x720p50_16_9	18
#define HDMI_VFRMT_1920x1080i50_16_9	19
#define HDMI_VFRMT_720x576i50_4_3	20
#define HDMI_VFRMT_1440x576i50_4_3	HDMI_VFRMT_720x576i50_4_3
#define HDMI_VFRMT_720x576i50_16_9	21
#define HDMI_VFRMT_1440x576i50_16_9	HDMI_VFRMT_720x576i50_16_9
#define HDMI_VFRMT_720x288p50_4_3	22
#define HDMI_VFRMT_1440x288p50_4_3	HDMI_VFRMT_720x288p50_4_3
#define HDMI_VFRMT_720x288p50_16_9	23
#define HDMI_VFRMT_1440x288p50_16_9	HDMI_VFRMT_720x288p50_16_9
#define HDMI_VFRMT_2880x576i50_4_3	24
#define HDMI_VFRMT_2880x576i50_16_9	25
#define HDMI_VFRMT_2880x288p50_4_3	26
#define HDMI_VFRMT_2880x288p50_16_9	27
#define HDMI_VFRMT_1440x576p50_4_3	28
#define HDMI_VFRMT_1440x576p50_16_9	29
#define HDMI_VFRMT_1920x1080p50_16_9	30
#define HDMI_VFRMT_1920x1080p24_16_9	31
#define HDMI_VFRMT_1920x1080p25_16_9	32
#define HDMI_VFRMT_1920x1080p30_16_9	33
#define HDMI_VFRMT_2880x480p60_4_3	34
#define HDMI_VFRMT_2880x480p60_16_9	35
#define HDMI_VFRMT_2880x576p50_4_3	36
#define HDMI_VFRMT_2880x576p50_16_9	37
#define HDMI_VFRMT_1920x1250i50_16_9	38
#define HDMI_VFRMT_1920x1080i100_16_9	39
#define HDMI_VFRMT_1280x720p100_16_9	40
#define HDMI_VFRMT_720x576p100_4_3	41
#define HDMI_VFRMT_720x576p100_16_9	42
#define HDMI_VFRMT_720x576i100_4_3	43
#define HDMI_VFRMT_1440x576i100_4_3	HDMI_VFRMT_720x576i100_4_3
#define HDMI_VFRMT_720x576i100_16_9	44
#define HDMI_VFRMT_1440x576i100_16_9	HDMI_VFRMT_720x576i100_16_9
#define HDMI_VFRMT_1920x1080i120_16_9	45
#define HDMI_VFRMT_1280x720p120_16_9	46
#define HDMI_VFRMT_720x480p120_4_3	47
#define HDMI_VFRMT_720x480p120_16_9	48
#define HDMI_VFRMT_720x480i120_4_3	49
#define HDMI_VFRMT_1440x480i120_4_3	HDMI_VFRMT_720x480i120_4_3
#define HDMI_VFRMT_720x480i120_16_9	50
#define HDMI_VFRMT_1440x480i120_16_9	HDMI_VFRMT_720x480i120_16_9
#define HDMI_VFRMT_720x576p200_4_3	51
#define HDMI_VFRMT_720x576p200_16_9	52
#define HDMI_VFRMT_720x576i200_4_3	53
#define HDMI_VFRMT_1440x576i200_4_3	HDMI_VFRMT_720x576i200_4_3
#define HDMI_VFRMT_720x576i200_16_9	54
#define HDMI_VFRMT_1440x576i200_16_9	HDMI_VFRMT_720x576i200_16_9
#define HDMI_VFRMT_720x480p240_4_3	55
#define HDMI_VFRMT_720x480p240_16_9	56
#define HDMI_VFRMT_720x480i240_4_3	57
#define HDMI_VFRMT_1440x480i240_4_3	HDMI_VFRMT_720x480i240_4_3
#define HDMI_VFRMT_720x480i240_16_9	58
#define HDMI_VFRMT_1440x480i240_16_9	HDMI_VFRMT_720x480i240_16_9
#define HDMI_VFRMT_MAX			59
#define HDMI_VFRMT_FORCE_32BIT		0x7FFFFFFF

struct hdmi_disp_mode_timing_type {
	uint32	video_format;
	uint32	active_h;
	uint32	front_porch_h;
	uint32	pulse_width_h;
	uint32	back_porch_h;
	boolean	active_low_h;
	uint32	active_v;
	uint32	front_porch_v;
	uint32	pulse_width_v;
	uint32	back_porch_v;
	boolean	active_low_v;
	/* Must divide by 1000 to get the actual frequency in MHZ */
	uint32	pixel_freq;
	/* Must divide by 1000 to get the actual frequency in HZ */
	uint32	refresh_rate;
	boolean	interlaced;
	boolean	supported;
};

#define HDMI_SETTINGS_640x480p60_4_3					\
	{HDMI_VFRMT_640x480p60_4_3,      640,  16,  96,  48,  TRUE,	\
	 480, 10, 2, 33, TRUE, 25200, 60000, FALSE, TRUE}
#define HDMI_SETTINGS_720x480p60_4_3					\
	{HDMI_VFRMT_720x480p60_4_3,      720,  16,  62,  60,  TRUE,	\
	 480, 9, 6, 30,  TRUE, 27030, 60000, FALSE, TRUE}
#define HDMI_SETTINGS_720x480p60_16_9					\
	{HDMI_VFRMT_720x480p60_16_9,     720,  16,  62,  60,  TRUE,	\
	 480, 9, 6, 30,  TRUE, 27030, 60000, FALSE, TRUE}
#define HDMI_SETTINGS_1280x720p60_16_9					\
	{HDMI_VFRMT_1280x720p60_16_9,    1280, 110, 40,  220, FALSE,	\
	 720, 5, 5, 20, FALSE, 74250, 60000, FALSE, TRUE}
#define HDMI_SETTINGS_1920x1080i60_16_9					\
	{HDMI_VFRMT_1920x1080i60_16_9,   1920, 88,  44,  148, FALSE,	\
	 540, 2, 5, 5, FALSE, 74250, 60000, FALSE, TRUE}
#define HDMI_SETTINGS_1440x480i60_4_3					\
	{HDMI_VFRMT_1440x480i60_4_3,     1440, 38,  124, 114, TRUE,	\
	 240, 4, 3, 15, TRUE, 27000, 60000, TRUE, TRUE}
#define HDMI_SETTINGS_1440x480i60_16_9					\
	{HDMI_VFRMT_1440x480i60_16_9,    1440, 38,  124, 114, TRUE,	\
	 240, 4, 3, 15, TRUE, 27000, 60000, TRUE, TRUE}
#define HDMI_SETTINGS_1920x1080p60_16_9					\
	{HDMI_VFRMT_1920x1080p60_16_9,   1920, 88,  44,  148,  FALSE,	\
	 1080, 4, 5, 36, FALSE, 148500, 60000, FALSE, TRUE}
#define HDMI_SETTINGS_720x576p50_4_3					\
	{HDMI_VFRMT_720x576p50_4_3,      720,  12,  64,  68,   TRUE,	\
	 576,  5, 5, 39, TRUE, 27000, 50000, FALSE, TRUE}
#define HDMI_SETTINGS_720x576p50_16_9					\
	{HDMI_VFRMT_720x576p50_16_9,     720,  12,  64,  68,   TRUE,	\
	 576,  5, 5, 39, TRUE, 27000, 50000, FALSE, TRUE}
#define HDMI_SETTINGS_1280x720p50_16_9					\
	{HDMI_VFRMT_1280x720p50_16_9,    1280, 440, 40,  220,  FALSE,	\
	 720,  5, 5, 20, FALSE, 74250, 50000, FALSE, TRUE}
#define HDMI_SETTINGS_1440x576i50_4_3					\
	{HDMI_VFRMT_1440x576i50_4_3,     1440, 24,  126, 138,  TRUE,	\
	 288,  2, 3, 19, TRUE, 27000, 50000, TRUE, TRUE}
#define HDMI_SETTINGS_1440x576i50_16_9					\
	{HDMI_VFRMT_1440x576i50_16_9,    1440, 24,  126, 138,  TRUE,	\
	 288,  2, 3, 19, TRUE, 27000, 50000, TRUE, TRUE}
#define HDMI_SETTINGS_1920x1080p50_16_9					\
	{HDMI_VFRMT_1920x1080p50_16_9,   1920,  528,  44,  148,  FALSE,	\
	 1080, 4, 5, 36, FALSE, 148500, 50000, FALSE, TRUE}
#define HDMI_SETTINGS_1920x1080p24_16_9					\
	{HDMI_VFRMT_1920x1080p24_16_9,   1920,  638,  44,  148,  FALSE,	\
	 1080, 4, 5, 36, FALSE, 74250, 24000, FALSE, TRUE}
#define HDMI_SETTINGS_1920x1080p25_16_9					\
	{HDMI_VFRMT_1920x1080p25_16_9,   1920,  528,  44,  148,  FALSE,	\
	 1080, 4, 5, 36, FALSE, 74250, 25000, FALSE, TRUE}
#define HDMI_SETTINGS_1920x1080p30_16_9					\
	{HDMI_VFRMT_1920x1080p30_16_9,   1920,  88,   44,  148,  FALSE,	\
	 1080, 4, 5, 36, FALSE, 74250, 30000, FALSE, TRUE}

/* A lookup table for all the supported display modes by the HDMI
 * hardware and driver.  Use HDMI_SETUP_LUT in the module init to
 * setup the LUT with the supported modes. */
extern struct hdmi_disp_mode_timing_type
	hdmi_common_supported_video_mode_lut[HDMI_VFRMT_MAX];

/* Structure that encapsulates all the supported display modes by the HDMI sink
 * device */
struct hdmi_disp_mode_list_type {
	uint32	disp_mode_list[HDMI_VFRMT_MAX];
	uint32	num_of_elements;
};

struct hdmi_common_state_type {
	boolean hpd_state;
	boolean hdcp_active;
	struct kobject *uevent_kobj;

	uint32 video_resolution;
	struct device *dev;

	struct hdmi_disp_mode_list_type disp_mode_list;
	int (*read_edid_block)(int block, uint8 *edid_buf);
};

/* The HDMI driver needs to initialize the common state. */
extern struct hdmi_common_state_type *hdmi_common_state;
extern struct mutex hdmi_common_state_hpd_mutex;

#define VFRMT_NOT_SUPPORTED(VFRMT) \
	{VFRMT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, FALSE}
#define HDMI_SETUP_LUT(MODE) do {					\
		struct hdmi_disp_mode_timing_type mode			\
			= HDMI_SETTINGS_ ## MODE;			\
		hdmi_common_supported_video_mode_lut[mode.video_format]	\
			= mode;						\
	} while (0)

int hdmi_common_read_edid(void);
const char *video_format_2string(uint32 format);
int hdmi_common_state_create(struct platform_device *pdev);
void hdmi_common_state_remove(void);
void hdmi_common_get_video_format_from_drv_data(struct msm_fb_data_type *mfd);
const struct hdmi_disp_mode_timing_type *hdmi_common_get_mode(uint32 mode);
const struct hdmi_disp_mode_timing_type *hdmi_common_get_supported_mode(
	uint32 mode);
void hdmi_common_init_panel_info(struct msm_panel_info *pinfo);

#endif /* __HDMI_COMMON_H__ */
