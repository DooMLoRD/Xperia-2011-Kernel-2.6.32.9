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

#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/mutex.h>

#define DEBUG

#include "msm_fb.h"
#include "hdmi_common.h"

struct hdmi_common_state_type *hdmi_common_state;
EXPORT_SYMBOL(hdmi_common_state);
DEFINE_MUTEX(hdmi_common_state_hpd_mutex);
EXPORT_SYMBOL(hdmi_common_state_hpd_mutex);

struct hdmi_disp_mode_timing_type
	hdmi_common_supported_video_mode_lut[HDMI_VFRMT_MAX] = {
	HDMI_SETTINGS_640x480p60_4_3,
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x480p60_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x480p60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1280x720p60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080i60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480i60_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480i60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x240p60_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x240p60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x480i60_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x480i60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x240p60_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x240p60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480p60_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480p60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080p60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x576p50_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x576p50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1280x720p50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080i50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576i50_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576i50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x288p50_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x288p50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x576i50_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x576i50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x288p50_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x288p50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576p50_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576p50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080p50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080p24_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080p25_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080p30_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x480p60_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x480p60_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x576p50_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_2880x576p50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1250i50_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080i100_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1280x720p100_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x576p100_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x576p100_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576i100_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576i100_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1920x1080i120_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1280x720p120_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x480p120_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x480p120_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480i120_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480i120_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x576p200_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x576p200_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576i200_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x576i200_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x480p240_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_720x480p240_16_9),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480i240_4_3),
	VFRMT_NOT_SUPPORTED(HDMI_VFRMT_1440x480i240_16_9),
};
EXPORT_SYMBOL(hdmi_common_supported_video_mode_lut);

const char *video_format_2string(uint32 format)
{
	switch (format) {
	default:
	case HDMI_VFRMT_640x480p60_4_3:    return " 640x 480 p60  4/3";
	case HDMI_VFRMT_720x480p60_4_3:    return " 720x 480 p60  4/3";
	case HDMI_VFRMT_720x480p60_16_9:   return " 720x 480 p60 16/9";
	case HDMI_VFRMT_1280x720p60_16_9:  return "1280x 720 p60 16/9";
	case HDMI_VFRMT_1920x1080i60_16_9: return "1920x1080 i60 16/9";
	case HDMI_VFRMT_1440x480i60_4_3:   return "1440x 480 i60  4/3";
	case HDMI_VFRMT_1440x480i60_16_9:  return "1440x 480 i60 16/9";
	case HDMI_VFRMT_1440x240p60_4_3:   return "1440x 240 p60  4/3";
	case HDMI_VFRMT_1440x240p60_16_9:  return "1440x 240 p60 16/9";
	case HDMI_VFRMT_2880x480i60_4_3:   return "2880x 480 i60  4/3";
	case HDMI_VFRMT_2880x480i60_16_9:  return "2880x 480 i60 16/9";
	case HDMI_VFRMT_2880x240p60_4_3:   return "2880x 240 p60  4/3";
	case HDMI_VFRMT_2880x240p60_16_9:  return "2880x 240 p60 16/9";
	case HDMI_VFRMT_1440x480p60_4_3:   return "1440x 480 p60  4/3";
	case HDMI_VFRMT_1440x480p60_16_9:  return "1440x 480 p60 16/9";
	case HDMI_VFRMT_1920x1080p60_16_9: return "1920x1080 p60 16/9";
	case HDMI_VFRMT_720x576p50_4_3:    return " 720x 576 p50  4/3";
	case HDMI_VFRMT_720x576p50_16_9:   return " 720x 576 p50 16/9";
	case HDMI_VFRMT_1280x720p50_16_9:  return "1280x 720 p50 16/9";
	case HDMI_VFRMT_1920x1080i50_16_9: return "1920x1080 i50 16/9";
	case HDMI_VFRMT_1440x576i50_4_3:   return "1440x 576 i50  4/3";
	case HDMI_VFRMT_1440x576i50_16_9:  return "1440x 576 i50 16/9";
	case HDMI_VFRMT_1440x288p50_4_3:   return "1440x 288 p50  4/3";
	case HDMI_VFRMT_1440x288p50_16_9:  return "1440x 288 p50 16/9";
	case HDMI_VFRMT_2880x576i50_4_3:   return "2880x 576 i50  4/3";
	case HDMI_VFRMT_2880x576i50_16_9:  return "2880x 576 i50 16/9";
	case HDMI_VFRMT_2880x288p50_4_3:   return "2880x 288 p50  4/3";
	case HDMI_VFRMT_2880x288p50_16_9:  return "2880x 288 p50 16/9";
	case HDMI_VFRMT_1440x576p50_4_3:   return "1440x 576 p50  4/3";
	case HDMI_VFRMT_1440x576p50_16_9:  return "1440x 576 p50 16/9";
	case HDMI_VFRMT_1920x1080p50_16_9: return "1920x1080 p50 16/9";
	case HDMI_VFRMT_1920x1080p24_16_9: return "1920x1080 p24 16/9";
	case HDMI_VFRMT_1920x1080p25_16_9: return "1920x1080 p25 16/9";
	case HDMI_VFRMT_1920x1080p30_16_9: return "1920x1080 p30 16/9";
	case HDMI_VFRMT_2880x480p60_4_3:   return "2880x 480 p60  4/3";
	case HDMI_VFRMT_2880x480p60_16_9:  return "2880x 480 p60 16/9";
	case HDMI_VFRMT_2880x576p50_4_3:   return "2880x 576 p50  4/3";
	case HDMI_VFRMT_2880x576p50_16_9:  return "2880x 576 p50 16/9";
	case HDMI_VFRMT_1920x1250i50_16_9: return "1920x1250 i50 16/9";
	case HDMI_VFRMT_1920x1080i100_16_9:return "1920x1080 i100 16/9";
	case HDMI_VFRMT_1280x720p100_16_9: return "1280x 720 p100 16/9";
	case HDMI_VFRMT_720x576p100_4_3:   return " 720x 576 p100  4/3";
	case HDMI_VFRMT_720x576p100_16_9:  return " 720x 576 p100 16/9";
	case HDMI_VFRMT_1440x576i100_4_3:  return "1440x 576 i100  4/3";
	case HDMI_VFRMT_1440x576i100_16_9: return "1440x 576 i100 16/9";
	case HDMI_VFRMT_1920x1080i120_16_9:return "1920x1080 i120 16/9";
	case HDMI_VFRMT_1280x720p120_16_9: return "1280x 720 p120 16/9";
	case HDMI_VFRMT_720x480p120_4_3:   return " 720x 480 p120  4/3";
	case HDMI_VFRMT_720x480p120_16_9:  return " 720x 480 p120 16/9";
	case HDMI_VFRMT_1440x480i120_4_3:  return "1440x 480 i120  4/3";
	case HDMI_VFRMT_1440x480i120_16_9: return "1440x 480 i120 16/9";
	case HDMI_VFRMT_720x576p200_4_3:   return " 720x 576 p200  4/3";
	case HDMI_VFRMT_720x576p200_16_9:  return " 720x 576 p200 16/9";
	case HDMI_VFRMT_1440x576i200_4_3:  return "1440x 576 i200  4/3";
	case HDMI_VFRMT_1440x576i200_16_9: return "1440x 576 i200 16/9";
	case HDMI_VFRMT_720x480p240_4_3:   return " 720x 480 p240  4/3";
	case HDMI_VFRMT_720x480p240_16_9:  return " 720x 480 p240 16/9";
	case HDMI_VFRMT_1440x480i240_4_3:  return "1440x 480 i240  4/3";
	case HDMI_VFRMT_1440x480i240_16_9: return "1440x 480 i240 16/9";
	}
}
EXPORT_SYMBOL(video_format_2string);

static ssize_t hdmi_common_rda_video_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = snprintf(buf, PAGE_SIZE, "%d\n",
		hdmi_common_state->video_resolution+1);
	DEV_DBG("%s: '%d'\n", __func__, hdmi_common_state->video_resolution+1);
	return ret;
}

static int atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}

static ssize_t hdmi_common_wta_video_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	uint32 video_mode;
	const struct hdmi_disp_mode_timing_type *disp_mode;

	mutex_lock(&hdmi_common_state_hpd_mutex);
	if (!hdmi_common_state->hpd_state) {
		mutex_unlock(&hdmi_common_state_hpd_mutex);
		DEV_INFO("%s: FAILED: display off or cable disconnected\n",
			__func__);
		return ret;
	}
	mutex_unlock(&hdmi_common_state_hpd_mutex);

	video_mode = atoi(buf)-1;
	disp_mode = hdmi_common_get_supported_mode(video_mode);
	if (!disp_mode) {
		DEV_INFO("%s: FAILED: mode not supported (%d)\n",
			__func__, video_mode);
		return ret;
	}

	kobject_uevent(hdmi_common_state->uevent_kobj, KOBJ_OFFLINE);

	hdmi_common_state->disp_mode_list.num_of_elements = 1;
	hdmi_common_state->disp_mode_list.disp_mode_list[0] = video_mode;
	DEV_DBG("%s: 'mode=%d %s' successful (sending OFF/ONLINE)\n", __func__,
		video_mode, video_format_2string(video_mode));

	kobject_uevent(hdmi_common_state->uevent_kobj, KOBJ_ONLINE);
	return ret;
}

static ssize_t hdmi_common_rda_video_mode_str(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = snprintf(buf, PAGE_SIZE, "%s\n",
		video_format_2string(hdmi_common_state->video_resolution));
	DEV_DBG("%s: '%s'\n", __func__,
		video_format_2string(hdmi_common_state->video_resolution));
	return ret;
}

static ssize_t hdmi_common_rda_edid_modes(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;

	buf[0] = 0;
	if (hdmi_common_state->disp_mode_list.num_of_elements) {
		uint32 *video_mode = hdmi_common_state->disp_mode_list
			.disp_mode_list;
		for (i = 0; i < hdmi_common_state->disp_mode_list
			.num_of_elements; ++i) {
			if (ret > 0)
				ret += snprintf(buf+ret, PAGE_SIZE-ret, ",%d",
					*video_mode++ + 1);
			else
				ret += snprintf(buf+ret, PAGE_SIZE-ret, "%d",
					*video_mode++ + 1);
		}
	} else
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%d",
			hdmi_common_state->video_resolution+1);

	DEV_DBG("%s: '%s'\n", __func__, buf);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	return ret;
}

static ssize_t hdmi_common_rda_connected(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	mutex_lock(&hdmi_common_state_hpd_mutex);
	ret = snprintf(buf, PAGE_SIZE, "%d\n",
		hdmi_common_state->hpd_state);
	DEV_DBG("%s: '%d'\n", __func__,
		hdmi_common_state->hpd_state);
	mutex_unlock(&hdmi_common_state_hpd_mutex);
	return ret;
}

static ssize_t hdmi_common_rda_hdcp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = snprintf(buf, PAGE_SIZE, "%d\n",
		hdmi_common_state->hdcp_active);
	DEV_DBG("%s: '%d'\n", __func__,
		hdmi_common_state->hdcp_active);
	return ret;
}

static DEVICE_ATTR(video_mode, S_IRUGO | S_IWUGO, hdmi_common_rda_video_mode,
	hdmi_common_wta_video_mode);
static DEVICE_ATTR(video_mode_str, S_IRUGO, hdmi_common_rda_video_mode_str,
	NULL);
static DEVICE_ATTR(edid_modes, S_IRUGO, hdmi_common_rda_edid_modes, NULL);
static DEVICE_ATTR(connected, S_IRUGO, hdmi_common_rda_connected, NULL);
static DEVICE_ATTR(hdcp, S_IRUGO, hdmi_common_rda_hdcp, NULL);
static struct attribute *hdmi_common_fs_attrs[] = {
	&dev_attr_video_mode.attr,
	&dev_attr_video_mode_str.attr,
	&dev_attr_edid_modes.attr,
	&dev_attr_connected.attr,
	&dev_attr_hdcp.attr,
	NULL,
};
static struct attribute_group hdmi_common_fs_attr_group = {
	.attrs = hdmi_common_fs_attrs,
};

/* create HDMI kobject and initialize */
int hdmi_common_state_create(struct platform_device *pdev)
{
	int rc;
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);

	rc = sysfs_create_group(&mfd->fbi->dev->kobj,
		&hdmi_common_fs_attr_group);
	if (rc) {
		DEV_ERR("%s: sysfs group creation failed, rc=%d\n", __func__,
			rc);
		return rc;
	}

	hdmi_common_state->uevent_kobj = &mfd->fbi->dev->kobj;
	DEV_ERR("%s: sysfs group %p\n", __func__,
		hdmi_common_state->uevent_kobj);

	kobject_uevent(hdmi_common_state->uevent_kobj, KOBJ_ADD);
	DEV_DBG("%s: kobject_uevent(KOBJ_ADD)\n", __func__);
	return 0;
}
EXPORT_SYMBOL(hdmi_common_state_create);

void hdmi_common_state_remove(void)
{
	if (hdmi_common_state->uevent_kobj)
		sysfs_remove_group(hdmi_common_state->uevent_kobj,
			&hdmi_common_fs_attr_group);
	hdmi_common_state->uevent_kobj = NULL;
}
EXPORT_SYMBOL(hdmi_common_state_remove);

/* The Logic ID for HDMI TX Core. Currently only support 1 HDMI TX Core. */
struct hdmi_edid_video_mode_property_type {
	uint32	video_code;
	uint32	active_h;
	uint32	active_v;
	boolean	interlaced;
	uint32	total_h;
	uint32	total_blank_h;
	uint32	total_v;
	uint32	total_blank_v;
	/* Must divide by 1000 to get the frequency */
	uint32	freq_h;
	/* Must divide by 1000 to get the frequency */
	uint32	freq_v;
	/* Must divide by 1000 to get the frequency */
	uint32	pixel_freq;
	/* Must divide by 1000 to get the frequency */
	uint32	refresh_rate;
	boolean	aspect_ratio_4_3;
};

/* LUT is sorted from lowest Active H to highest Active H - ease searching */
static struct hdmi_edid_video_mode_property_type
	hdmi_edid_disp_mode_lut[] = {

	/* All 640 H Active */
	{HDMI_VFRMT_640x480p60_4_3, 640, 480, FALSE, 800, 160, 525, 45,
	 31465, 59940, 25175, 59940, TRUE},
	{HDMI_VFRMT_640x480p60_4_3, 640, 480, FALSE, 800, 160, 525, 45,
	 31500, 60000, 25200, 60000, TRUE},

	/* All 720 H Active */
	{HDMI_VFRMT_720x576p50_4_3,  720, 576, FALSE, 864, 144, 625, 49,
	 31250, 50000, 27000, 50000, TRUE},
	{HDMI_VFRMT_720x480p60_4_3,  720, 480, FALSE, 858, 138, 525, 45,
	 31465, 59940, 27000, 59940, TRUE},
	{HDMI_VFRMT_720x480p60_4_3,  720, 480, FALSE, 858, 138, 525, 45,
	 31500, 60000, 27030, 60000, TRUE},
	{HDMI_VFRMT_720x576p100_4_3, 720, 576, FALSE, 864, 144, 625, 49,
	 62500, 100000, 54000, 100000, TRUE},
	{HDMI_VFRMT_720x480p120_4_3, 720, 480, FALSE, 858, 138, 525, 45,
	 62937, 119880, 54000, 119880, TRUE},
	{HDMI_VFRMT_720x480p120_4_3, 720, 480, FALSE, 858, 138, 525, 45,
	 63000, 120000, 54054, 120000, TRUE},
	{HDMI_VFRMT_720x576p200_4_3, 720, 576, FALSE, 864, 144, 625, 49,
	 125000, 200000, 108000, 200000, TRUE},
	{HDMI_VFRMT_720x480p240_4_3, 720, 480, FALSE, 858, 138, 525, 45,
	 125874, 239760, 108000, 239000, TRUE},
	{HDMI_VFRMT_720x480p240_4_3, 720, 480, FALSE, 858, 138, 525, 45,
	 126000, 240000, 108108, 240000, TRUE},

	/* All 1280 H Active */
	{HDMI_VFRMT_1280x720p50_16_9,  1280, 720, FALSE, 1980, 700, 750, 30,
	 37500, 50000, 74250, 50000, FALSE},
	{HDMI_VFRMT_1280x720p60_16_9,  1280, 720, FALSE, 1650, 370, 750, 30,
	 44955, 59940, 74176, 59940, FALSE},
	{HDMI_VFRMT_1280x720p60_16_9,  1280, 720, FALSE, 1650, 370, 750, 30,
	 45000, 60000, 74250, 60000, FALSE},
	{HDMI_VFRMT_1280x720p100_16_9, 1280, 720, FALSE, 1980, 700, 750, 30,
	 75000, 100000, 148500, 100000, FALSE},
	{HDMI_VFRMT_1280x720p120_16_9, 1280, 720, FALSE, 1650, 370, 750, 30,
	 89909, 119880, 148352, 119880, FALSE},
	{HDMI_VFRMT_1280x720p120_16_9, 1280, 720, FALSE, 1650, 370, 750, 30,
	 90000, 120000, 148500, 120000, FALSE},

	/* All 1440 H Active */
	{HDMI_VFRMT_1440x576i50_4_3, 1440, 576, TRUE,  1728, 288, 625, 24,
	 15625, 50000, 27000, 50000, TRUE},
	{HDMI_VFRMT_720x288p50_4_3,  1440, 288, FALSE, 1728, 288, 312, 24,
	 15625, 50080, 27000, 50000, TRUE},
	{HDMI_VFRMT_720x288p50_4_3,  1440, 288, FALSE, 1728, 288, 313, 25,
	 15625, 49920, 27000, 50000, TRUE},
	{HDMI_VFRMT_720x288p50_4_3,  1440, 288, FALSE, 1728, 288, 314, 26,
	 15625, 49761, 27000, 50000, TRUE},
	{HDMI_VFRMT_1440x576p50_4_3, 1440, 576, FALSE, 1728, 288, 625, 49,
	 31250, 50000, 54000, 50000, TRUE},
	{HDMI_VFRMT_1440x480i60_4_3, 1440, 480, TRUE,  1716, 276, 525, 22,
	 15734, 59940, 27000, 59940, TRUE},
	{HDMI_VFRMT_1440x240p60_4_3, 1440, 240, FALSE, 1716, 276, 262, 22,
	 15734, 60054, 27000, 59940, TRUE},
	{HDMI_VFRMT_1440x240p60_4_3, 1440, 240, FALSE, 1716, 276, 263, 23,
	 15734, 59826, 27000, 59940, TRUE},
	{HDMI_VFRMT_1440x480p60_4_3, 1440, 480, FALSE, 1716, 276, 525, 45,
	 31469, 59940, 54000, 59940, TRUE},
	{HDMI_VFRMT_1440x480i60_4_3, 1440, 480, TRUE,  1716, 276, 525, 22,
	 15750, 60000, 27027, 60000, TRUE},
	{HDMI_VFRMT_1440x240p60_4_3, 1440, 240, FALSE, 1716, 276, 262, 22,
	 15750, 60115, 27027, 60000, TRUE},
	{HDMI_VFRMT_1440x240p60_4_3, 1440, 240, FALSE, 1716, 276, 263, 23,
	 15750, 59886, 27027, 60000, TRUE},
	{HDMI_VFRMT_1440x480p60_4_3, 1440, 480, FALSE, 1716, 276, 525, 45,
	 31500, 60000, 54054, 60000, TRUE},
	{HDMI_VFRMT_1440x576i100_4_3, 1440, 576, TRUE,  1728, 288, 625, 24,
	 31250, 100000, 54000, 100000, TRUE},
	{HDMI_VFRMT_1440x480i120_4_3, 1440, 480, TRUE,  1716, 276, 525, 22,
	 31469, 119880, 54000, 119880, TRUE},
	{HDMI_VFRMT_1440x480i120_4_3, 1440, 480, TRUE,  1716, 276, 525, 22,
	 31500, 120000, 54054, 120000, TRUE},
	{HDMI_VFRMT_1440x576i200_4_3, 1440, 576, TRUE,  1728, 288, 625, 24,
	 62500, 200000, 108000, 200000, TRUE},
	{HDMI_VFRMT_1440x480i240_4_3, 1440, 480, TRUE,  1716, 276, 525, 22,
	 62937, 239760, 108000, 239000, TRUE},
	{HDMI_VFRMT_1440x480i240_4_3, 1440, 480, TRUE,  1716, 276, 525, 22,
	 63000, 240000, 108108, 240000, TRUE},

	/* All 1920 H Active */
	{HDMI_VFRMT_1920x1080p24_16_9, 1920, 1080, FALSE, 2750, 830, 1125,
	 45, 26973, 23976, 74176, 24000, FALSE},
	{HDMI_VFRMT_1920x1080p24_16_9, 1920, 1080, FALSE, 2750, 830, 1125,
	 45, 27000, 24000, 74250, 24000, FALSE},
	{HDMI_VFRMT_1920x1080p25_16_9, 1920, 1080, FALSE, 2640, 720, 1125,
	 45, 28125, 25000, 74250, 25000, FALSE},
	{HDMI_VFRMT_1920x1080p30_16_9, 1920, 1080, FALSE, 2200, 280, 1125,
	 45, 33716, 29970, 74176, 30000, FALSE},
	{HDMI_VFRMT_1920x1080p30_16_9, 1920, 1080, FALSE, 2200, 280, 1125,
	 45, 33750, 30000, 74250, 30000, FALSE},
	{HDMI_VFRMT_1920x1080p50_16_9, 1920, 1080, FALSE, 2640, 720, 1125,
	 45, 56250, 50000, 148500, 50000, FALSE},
	{HDMI_VFRMT_1920x1080i50_16_9, 1920, 1080, TRUE,  2304, 384, 1250,
	 85, 31250, 50000, 72000, 50000, FALSE},
	{HDMI_VFRMT_1920x1080i60_16_9, 1920, 1080, TRUE,  2200, 280, 1125,
	 22, 33716, 59940, 74176, 59940, FALSE},
	{HDMI_VFRMT_1920x1080p60_16_9, 1920, 1080, FALSE, 2200, 280, 1125,
	 45, 67433, 59940, 148352, 59940, FALSE},
	{HDMI_VFRMT_1920x1080i60_16_9, 1920, 1080, TRUE,  2200, 280, 1125,
	 22, 33750, 60000, 74250, 60000, FALSE},
	{HDMI_VFRMT_1920x1080p60_16_9, 1920, 1080, TRUE,  2200, 280, 1125,
	 45, 67500, 60000, 148500, 60000, FALSE},
	{HDMI_VFRMT_1920x1080i100_16_9, 1920, 1080, TRUE,  2640, 720, 1125,
	 22, 56250, 100000, 148500, 100000, FALSE},
	{HDMI_VFRMT_1920x1080i120_16_9, 1920, 1080, TRUE,  2200, 280, 1125,
	 22, 67432, 119880, 148352, 119980, FALSE},
	{HDMI_VFRMT_1920x1080i120_16_9, 1920, 1080, TRUE,  2200, 280, 1125,
	 22, 67500, 120000, 148500, 120000, FALSE},

	/* All 2880 H Active */
	{HDMI_VFRMT_2880x576i50_4_3, 2880, 576, TRUE,  3456, 576, 625, 24,
	 15625, 50000, 54000, 50000, TRUE},
	{HDMI_VFRMT_2880x288p50_4_3, 2880, 576, FALSE, 3456, 576, 312, 24,
	 15625, 50080, 54000, 50000, TRUE},
	{HDMI_VFRMT_2880x288p50_4_3, 2880, 576, FALSE, 3456, 576, 313, 25,
	 15625, 49920, 54000, 50000, TRUE},
	{HDMI_VFRMT_2880x288p50_4_3, 2880, 576, FALSE, 3456, 576, 314, 26,
	 15625, 49761, 54000, 50000, TRUE},
	{HDMI_VFRMT_2880x576p50_4_3, 2880, 576, FALSE, 3456, 576, 625, 49,
	 31250, 50000, 108000, 50000, TRUE},
	{HDMI_VFRMT_2880x480i60_4_3, 2880, 480, TRUE,  3432, 552, 525, 22,
	 15734, 59940, 54000, 59940, TRUE},
	{HDMI_VFRMT_2880x240p60_4_3, 2880, 480, FALSE, 3432, 552, 262, 22,
	 15734, 60054, 54000, 59940, TRUE},
	{HDMI_VFRMT_2880x240p60_4_3, 2880, 480, FALSE, 3432, 552, 263, 23,
	 15734, 59940, 54000, 59940, TRUE},
	{HDMI_VFRMT_2880x480p60_4_3, 2880, 480, FALSE, 3432, 552, 525, 45,
	 31469, 59940, 108000, 59940, TRUE},
	{HDMI_VFRMT_2880x480i60_4_3, 2880, 480, TRUE,  3432, 552, 525, 22,
	 15750, 60000, 54054, 60000, TRUE},
	{HDMI_VFRMT_2880x240p60_4_3, 2880, 240, FALSE, 3432, 552, 262, 22,
	 15750, 60115, 54054, 60000, TRUE},
	{HDMI_VFRMT_2880x240p60_4_3, 2880, 240, FALSE, 3432, 552, 262, 23,
	 15750, 59886, 54054, 60000, TRUE},
	{HDMI_VFRMT_2880x480p60_4_3, 2880, 480, FALSE, 3432, 552, 525, 45,
	 31500, 60000, 108108, 60000, TRUE},
};

static void hdmi_edid_extract_vendor_id(const uint8 *in_buf,
	char *vendor_id)
{
	uint32 id_codes = ((uint32)in_buf[8] << 8) + in_buf[9];

	vendor_id[0] = 'A' - 1 + ((id_codes >> 10) & 0x1F);
	vendor_id[1] = 'A' - 1 + ((id_codes >> 5) & 0x1F);
	vendor_id[2] = 'A' - 1 + (id_codes & 0x1F);
	vendor_id[3] = 0;
}

static uint32 hdmi_edid_extract_ieee_reg_id(const uint8 *in_buf)
{
	/* the start of data block collection, start of Video Data Block */
	uint32 offset = 4;
	/* L1 + 1 is start of Audio Data Block */
	offset += 1 + (in_buf[offset] & 0x1F);
	/* L2 + 1 is start of Speaker Allocation Data Block */
	offset += 1 + (in_buf[offset] & 0x1F);
	/* L3 + 1 is start of Vendor Specific Data Block */
	offset += 1 + (in_buf[offset] & 0x1F);

	return ((uint32)in_buf[offset+3] << 16)
		+ ((uint32)in_buf[offset+2] << 8)
		+ (uint32)in_buf[offset+1];
}

static void hdmi_edid_detail_desc(const uint8 *data_buf, uint32 *disp_mode)
{
	boolean	aspect_ratio_4_3    = FALSE;
	boolean	interlaced          = FALSE;
	uint32	active_h            = 0;
	uint32	active_v            = 0;
	uint32	blank_h             = 0;
	uint32	blank_v             = 0;
	uint32	ndx                 = 0;
	uint32	max_num_of_elements = 0;
	uint32	img_size_h          = 0;
	uint32	img_size_v          = 0;

	/* See VESA Spec */
	/* EDID_TIMING_DESC_UPPER_H_NIBBLE[0x4]: Relative Offset to the EDID
	 *   detailed timing descriptors - Upper 4 bit for each H active/blank
	 *   field */
	/* EDID_TIMING_DESC_H_ACTIVE[0x2]: Relative Offset to the EDID detailed
	 *   timing descriptors - H active */
	active_h = ((((uint32)data_buf[0x4] >> 0x4) & 0xF) << 8)
		| data_buf[0x2];

	/* EDID_TIMING_DESC_H_BLANK[0x3]: Relative Offset to the EDID detailed
	 *   timing descriptors - H blank */
	blank_h = (((uint32)data_buf[0x4] & 0xF) << 8)
		| data_buf[0x3];

	/* EDID_TIMING_DESC_UPPER_V_NIBBLE[0x7]: Relative Offset to the EDID
	 *   detailed timing descriptors - Upper 4 bit for each V active/blank
	 *   field */
	/* EDID_TIMING_DESC_V_ACTIVE[0x5]: Relative Offset to the EDID detailed
	 *   timing descriptors - V active */
	active_v = ((((uint32)data_buf[0x7] >> 0x4) & 0xF) << 8)
		| data_buf[0x5];

	/* EDID_TIMING_DESC_V_BLANK[0x6]: Relative Offset to the EDID detailed
	 *   timing descriptors - V blank */
	blank_v = (((uint32)data_buf[0x7] & 0xF) << 8)
		| data_buf[0x6];

	/* EDID_TIMING_DESC_IMAGE_SIZE_UPPER_NIBBLE[0xE]: Relative Offset to the
	 *   EDID detailed timing descriptors - Image Size upper nibble
	 *   V and H */
	/* EDID_TIMING_DESC_H_IMAGE_SIZE[0xC]: Relative Offset to the EDID
	 *   detailed timing descriptors - H image size */
	/* EDID_TIMING_DESC_V_IMAGE_SIZE[0xD]: Relative Offset to the EDID
	 *   detailed timing descriptors - V image size */
	img_size_h = ((((uint32)data_buf[0xE] >> 0x4) & 0xF) << 8)
		| data_buf[0xC];
	img_size_v = (((uint32)data_buf[0xE] & 0xF) << 8)
		| data_buf[0xD];

	aspect_ratio_4_3 = (img_size_h * 3 == img_size_v * 4);

	max_num_of_elements  = sizeof(hdmi_edid_disp_mode_lut)
		/ sizeof(*hdmi_edid_disp_mode_lut);

	/* Break table in half and search using H Active */
	ndx = active_h < hdmi_edid_disp_mode_lut[max_num_of_elements / 2]
		.active_h ? 0 : max_num_of_elements / 2;

	/* EDID_TIMING_DESC_INTERLACE[0xD:8]: Relative Offset to the EDID
	 *   detailed timing descriptors - Interlace flag */
	interlaced = (data_buf[0xD] & 0x80) >> 7;

	DEV_DBG("%s: A[%ux%u] B[%ux%u] V[%ux%u] %s\n", __func__,
		active_h, active_v, blank_h, blank_v, img_size_h, img_size_v,
		interlaced ? "i" : "p");

	*disp_mode = HDMI_VFRMT_FORCE_32BIT;
	while (ndx < max_num_of_elements) {
		const struct hdmi_edid_video_mode_property_type *edid =
			hdmi_edid_disp_mode_lut+ndx;

		if ((interlaced    == edid->interlaced)    &&
			(active_h  == edid->active_h)      &&
			(blank_h   == edid->total_blank_h) &&
			(blank_v   == edid->total_blank_v) &&
			((active_v == edid->active_v) ||
			 (active_v == (edid->active_v + 1)))
		) {
			if (edid->aspect_ratio_4_3 && !aspect_ratio_4_3)
				/* Aspect ratio 16:9 */
				*disp_mode = edid->video_code + 1;
			else
				/* Aspect ratio 4:3 */
				*disp_mode = edid->video_code;

			DEV_DBG("%s: mode found:%d\n", __func__, *disp_mode);
			break;
		}
		++ndx;
	}
	if (ndx == max_num_of_elements)
		DEV_INFO("%s: *no mode* found\n", __func__);
}

static void add_supported_video_format(
	struct hdmi_disp_mode_list_type *disp_mode_list,
	uint32 video_format)
{
	const struct hdmi_disp_mode_timing_type *timing =
		hdmi_common_get_supported_mode(video_format);
	boolean supported = timing != NULL;

	if (video_format >= HDMI_VFRMT_MAX)
		return;

	DEV_DBG("EDID: format: %d [%s], %s\n",
		video_format, video_format_2string(video_format),
		supported ? "Supported" : "Not-Supported");
	if (supported)
		disp_mode_list->disp_mode_list[
			disp_mode_list->num_of_elements++] = video_format;
}

static void hdmi_edid_get_display_mode(const uint8 *data_buf,
	struct hdmi_disp_mode_list_type *disp_mode_list,
	uint32 num_og_cea_blocks, boolean short_desc)
{
	uint32 num_of_disp_mode	= 0;
	uint32 i		= 0;
	uint32 video_format	= HDMI_VFRMT_640x480p60_4_3;
	boolean has480p		= FALSE;

	disp_mode_list->num_of_elements = 0;
	if (short_desc) {
		/* EDID_SHORT_VIDEO_DESC_LENGTH[0x84] - Video data block,
		 *   indicate the number of short descriptors */
		num_of_disp_mode = data_buf[0x84] & 0x1F;

		for (i = 0; i < num_of_disp_mode; i++) {
			/* Subtract 1 because it is zero based in the driver,
			 * while the Video identification code is 1 based in the
			 * CEA_861D spec */
			/* EDID_SHORT_VIDEO_DESC[0x85] - Video data block, start
			 *   of first short descriptor */
			video_format = (data_buf[0x85 + i] & 0x7F) - 1;
			add_supported_video_format(disp_mode_list,
				video_format);
			if (video_format == HDMI_VFRMT_640x480p60_4_3)
				has480p = TRUE;
		}
	} else if (!num_og_cea_blocks) {
		/* Detailed timing descriptors */
		uint32 desc_offset = 0;
		/* Maximum 4 timing descriptor in block 0 - No CEA
		 * extension in this case */
		/* EDID_FIRST_TIMING_DESC[0x36] - 1st detailed timing
		 *   descriptor */
		/* EDID_DETAIL_TIMING_DESC_BLCK_SZ[0x12] - Each detailed timing
		 *   descriptor has block size of 18 */
		while (4 > i && 0 != data_buf[0x36+desc_offset]) {
			hdmi_edid_detail_desc(data_buf+0x36+desc_offset,
				&video_format);
			add_supported_video_format(disp_mode_list,
				video_format);
			if (video_format == HDMI_VFRMT_640x480p60_4_3)
				has480p = TRUE;
			desc_offset += 0x12;
			++i;
		}
	} else if (1 == num_og_cea_blocks) {
		uint32 desc_offset = 0;
		/* Parse block 1 - CEA extension byte offset of first
		 * detailed timing generation - offset is relevant to
		 * the offset of block 1 */

		/* EDID_CEA_EXTENSION_FIRST_DESC[0x82]: Offset to CEA
		 * extension first timing desc - indicate the offset of
		 * the first detailed timing descriptor */
		 /* EDID_BLOCK_SIZE = 0x80  Each page size in the EDID ROM */
		desc_offset = data_buf[0x82];
		while (0 != data_buf[0x80 + desc_offset]) {
			hdmi_edid_detail_desc(data_buf+0x36+desc_offset,
				&video_format);
			add_supported_video_format(disp_mode_list,
				video_format);
			if (video_format == HDMI_VFRMT_640x480p60_4_3)
				has480p = TRUE;
			desc_offset += 0x12;
			++i;
		}
	}

	if (!has480p)
		/* Need to add default 640 by 480 timings, in case not described
		 * in the EDID structure.
		 * All DTV sink devices should support this mode */
		add_supported_video_format(disp_mode_list,
			HDMI_VFRMT_640x480p60_4_3);
}

static int hdmi_common_read_edid_block(int block, uint8 *edid_buf)
{
	uint32 ndx, check_sum;
	int status = hdmi_common_state->read_edid_block(block, edid_buf);
	if (status)
		goto error;

	/* Calculate checksum */
	check_sum = 0;
	for (ndx = 0; ndx < 0x80; ++ndx)
		check_sum += edid_buf[ndx];

	if (check_sum & 0xFF) {
		const u8 *b = edid_buf;
		DEV_ERR("%s: failed CHECKSUM (read:%x, expected:%x)\n",
			__func__, (uint8)edid_buf[0x7F], (uint8)check_sum);

		for (ndx = 0; ndx < 0x100; ndx += 16)
			DEV_DBG("EDID[%02x-%02x] %02x %02x %02x %02x  "
				"%02x %02x %02x %02x    %02x %02x %02x %02x  "
				"%02x %02x %02x %02x\n", ndx, ndx+15,
				b[ndx+0], b[ndx+1], b[ndx+2], b[ndx+3],
				b[ndx+4], b[ndx+5], b[ndx+6], b[ndx+7],
				b[ndx+8], b[ndx+9], b[ndx+10], b[ndx+11],
				b[ndx+12], b[ndx+13], b[ndx+14], b[ndx+15]);

		status = -EPROTO;
		goto error;
	}

error:
	return status;
}

static boolean check_edid_header(const uint8 *edid_buf)
{
	return (edid_buf[0] == 0x00) && (edid_buf[1] == 0xff)
		&& (edid_buf[2] == 0xff) && (edid_buf[3] == 0xff)
		&& (edid_buf[4] == 0xff) && (edid_buf[5] == 0xff)
		&& (edid_buf[6] == 0xff) && (edid_buf[7] == 0x00);
}

int hdmi_common_read_edid(void)
{
	int status = 0;
	uint32 cea_extension_ver = 0;
	uint32 num_og_cea_blocks  = 0;
	uint32 ieee_reg_id = 0;
	boolean short_desc = FALSE;
	char vendor_id[5];
	/* EDID_BLOCK_SIZE[0x80] Each page size in the EDID ROM */
	uint8 edid_buf[0x80 * 2];

	memset(&hdmi_common_state->disp_mode_list, 0,
		sizeof(hdmi_common_state->disp_mode_list));
	memset(edid_buf, 0, sizeof(edid_buf));

	status = hdmi_common_read_edid_block(0, edid_buf);
	if (status || !check_edid_header(edid_buf)) {
		if (!status)
			status = -EPROTO;
		DEV_ERR("%s: edid read block(0) failed: %d "
			"[%02x%02x%02x%02x%02x%02x%02x%02x]\n", __func__,
			status,
			edid_buf[0], edid_buf[1], edid_buf[2], edid_buf[3],
			edid_buf[4], edid_buf[5], edid_buf[6], edid_buf[7]);
		goto error;
	}
	hdmi_edid_extract_vendor_id(edid_buf, vendor_id);

	/* EDID_CEA_EXTENSION_FLAG[0x7E] - CEC extension byte */
	num_og_cea_blocks = edid_buf[0x7E];

	/* Find out any CEA extension blocks following block 0 */
	switch (num_og_cea_blocks) {
	case 0: /* No CEA extension */
		break;
	case 1: /* Read block 1 */
		status = hdmi_common_read_edid_block(1, edid_buf+0x80);
		if (status || edid_buf[0x80] != 2) {
			if (!status)
				status = -EPROTO;
			DEV_ERR("%s: ddc read block(1) failed: %d\n", __func__,
				status);
			goto error;
		}
		ieee_reg_id =
			hdmi_edid_extract_ieee_reg_id(edid_buf+0x80);
		break;
	default:
		DEV_ERR("%s: ddc read failed, not supported multi-blocks: %d\n",
			__func__, num_og_cea_blocks);
		status = -EPROTO;
		goto error;
	}

	if (num_og_cea_blocks) {
		/* EDID_CEA_EXTENSION_VERSION[0x81]: Offset to CEA extension
		 * version number - v1,v2,v3 (v1 is seldom, v2 is obsolete,
		 * v3 most common) */
		cea_extension_ver = edid_buf[0x81];
		switch (cea_extension_ver) {
		case 1:
			break;
		case 2: /* Obsolete from CEA_861D spec */
			break;
		case 3:
			short_desc = TRUE;
			break;
		default:
			break;
		}
	}

	/* EDID_VERSION[0x12] - EDID Version */
	/* EDID_REVISION[0x13] - EDID Revision */
	DEV_DBG("EDID (V=%d.%d, #CEABlocks=%d[V%d], ID=%s, IEEE=%x)\n",
		edid_buf[0x12], edid_buf[0x13], num_og_cea_blocks,
		cea_extension_ver, vendor_id, ieee_reg_id);

	hdmi_edid_get_display_mode(edid_buf,
		&hdmi_common_state->disp_mode_list,
		num_og_cea_blocks, short_desc);

	return 0;

error:
	hdmi_common_state->disp_mode_list.num_of_elements = 1;
	hdmi_common_state->disp_mode_list.disp_mode_list[0] =
		hdmi_common_state->video_resolution;
	return status;
}
EXPORT_SYMBOL(hdmi_common_read_edid);

void hdmi_common_get_video_format_from_drv_data(struct msm_fb_data_type *mfd)
{
	uint32 format;
	struct fb_var_screeninfo *var = &mfd->fbi->var;

	if (var->reserved[3]) {
		format = var->reserved[3]-1;
	} else {
		DEV_DBG("detecting resolution from %dx%d use var->reserved[3]"
			" to specify mode", mfd->var_xres, mfd->var_yres);
		switch (mfd->var_xres) {
		default:
		case  640:
			format = HDMI_VFRMT_640x480p60_4_3;
			break;
		case  720:
			format = (mfd->var_yres == 480)
				? HDMI_VFRMT_720x480p60_16_9
				: HDMI_VFRMT_720x576p50_16_9;
			break;
		case 1280:
			format = HDMI_VFRMT_1280x720p60_16_9;
			break;
		case 1440:
			format = (mfd->var_yres == 480)
				? HDMI_VFRMT_1440x480i60_16_9
				: HDMI_VFRMT_1440x576i50_16_9;
			break;
		case 1920:
			format = HDMI_VFRMT_1920x1080p60_16_9;
			break;
		}
	}

	if (hdmi_common_state->video_resolution != format)
		DEV_DBG("switching %s => %s", video_format_2string(
			hdmi_common_state->video_resolution),
			video_format_2string(format));
	else
		DEV_DBG("resolution %s", video_format_2string(
			hdmi_common_state->video_resolution));
	hdmi_common_state->video_resolution = format;
}
EXPORT_SYMBOL(hdmi_common_get_video_format_from_drv_data);

const struct hdmi_disp_mode_timing_type *hdmi_common_get_mode(uint32 mode)
{
	if (mode >= HDMI_VFRMT_MAX)
		return NULL;

	return &hdmi_common_supported_video_mode_lut[mode];
}
EXPORT_SYMBOL(hdmi_common_get_mode);

const struct hdmi_disp_mode_timing_type *hdmi_common_get_supported_mode(
	uint32 mode)
{
	const struct hdmi_disp_mode_timing_type *ret
		= hdmi_common_get_mode(mode);

	if (ret == NULL || !ret->supported)
		return NULL;
	return ret;
}
EXPORT_SYMBOL(hdmi_common_get_supported_mode);

void hdmi_common_init_panel_info(struct msm_panel_info *pinfo)
{
	const struct hdmi_disp_mode_timing_type *timing =
		hdmi_common_get_supported_mode(
		hdmi_common_state->video_resolution);

	if (timing == NULL)
		return;

	pinfo->xres = timing->active_h;
	pinfo->yres = timing->active_v;
	pinfo->clk_rate = timing->pixel_freq*1000;

	pinfo->lcdc.h_back_porch = timing->back_porch_h;
	pinfo->lcdc.h_front_porch = timing->front_porch_h;
	pinfo->lcdc.h_pulse_width = timing->pulse_width_h;
	pinfo->lcdc.v_back_porch = timing->back_porch_v;
	pinfo->lcdc.v_front_porch = timing->front_porch_v;
	pinfo->lcdc.v_pulse_width = timing->pulse_width_v;

	pinfo->type = DTV_PANEL;
	pinfo->pdest = DISPLAY_2;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 1;

	/* blk */
	pinfo->lcdc.border_clr = 0;
	/* blue */
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;
}
EXPORT_SYMBOL(hdmi_common_init_panel_info);
