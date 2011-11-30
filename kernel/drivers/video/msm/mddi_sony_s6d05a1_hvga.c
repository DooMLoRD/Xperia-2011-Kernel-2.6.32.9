/* drivers/video/msm/mddi_sony_s6d05a1_hvga.c
 *
 * MDDI client driver for the sony HVGA display
*  with driver IC Samsung S6D05A1X01.
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <mach/gpio.h>
#include <mach/vreg.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/swab.h>
#include <mach/gpio.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/autoconf.h>
#include <linux/mddi_sony_s6d05a1_hvga.h>

/* Internal version number */
#define MDDI_DRIVER_VERSION 0x0003

/* Todo: Remove. Change panel ID function for SW sample */
/*#define ENABLE_FOR_SW_SAMPLE*/

/* DISPLAY ID value */
#define MDDI_SONY_CELL_ID1 0x02
#define MDDI_SONY_CELL_ID2 0xA1

/* TODO: temp ID value for SW sample */
#define MDDI_SONY_DISPLAY_DRIVER_IC_ID 0xF0

/* Frame time, used for delays */
#define MDDI_FRAME_TIME 13

enum lcd_registers_t {
	LCD_REG_COLUMN_ADDRESS = 0x2a,
	LCD_REG_PAGE_ADDRESS = 0x2b,
	LCD_REG_DRIVER_IC_ID = 0xA1, /* TODO: check: temp SW sample */
	LCD_REG_CELL_ID = 0xDA,
	LCD_REG_MODULE_ID = 0xDB,
	LCD_REG_REVISION_ID = 0xDC
};

/* Byte order,  word0=P4P3P2P1, little endian */
#define write_client_reg_nbr(__X, __Y0, __Y1, __Y2, __Y3, __NBR) \
  mddi_host_register_write16(__X, __Y0, __Y1, __Y2, __Y3, __NBR, \
			TRUE, NULL, MDDI_HOST_PRIM);

#define write_client_reg_xl(__X, __Y, __NBR) \
  mddi_host_register_write_xl(__X, __Y, __NBR, \
			TRUE, NULL, MDDI_HOST_PRIM);

/* Function protos */
static ssize_t show_driver_version(
	struct device *pdev,
	struct device_attribute *attr,
	char *buf);

static ssize_t show_debug_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	char *buf);

static ssize_t store_debug_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	const char *buf,
	size_t count);

static ssize_t show_dbc_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	char *buf);

static ssize_t store_dbc_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	const char *buf,
	size_t count);

static ssize_t show_dbc_mode_ctrl(
	struct device *dev_p,
	struct device_attribute *attr,
	char *buf);

static ssize_t store_dbc_mode_ctrl(
	struct device *dev_p,
	struct device_attribute *attr,
	const char *buf,
	size_t count);

static ssize_t show_power_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	char *buf);

static ssize_t store_power_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	const char *buf,
	size_t count);


/* Function Configuration */
#define DBC_OFF 0
#define DBC_ON	1

static int dbc_ctrl = DBC_ON;
module_param(dbc_ctrl, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dbc_ctrl, "Dynamic Backlight Control DBC_OFF = 0, DBC_ON = 1");


#define DBC_MODE_UI	1
#define DBC_MODE_IMAGE	2
#define DBC_MODE_VIDEO	3

static int dbc_mode = DBC_MODE_VIDEO;

#define POWER_OFF 0
#define POWER_ON  1

static int power_ctrl = POWER_OFF;

/* Debug levels */
#define LEVEL_QUIET 0
#define LEVEL_DEBUG 1
#define LEVEL_TRACE 2
#define LEVEL_PARAM 3

static int debug = LEVEL_QUIET;
module_param(debug, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(debug,
		"Debug level QUIET = 0, DEBUG = 1, TRACE = 2, PARAM = 3");

#define DBG_STR "MDDI: SONY HVGA:"

#undef MDDI_DEBUG
#define MDDI_DEBUG(level, string, args...) \
printk(KERN_WARNING string, ##args);

#undef MDDI_ERROR
#define MDDI_ERROR(string, args...) \
printk(KERN_WARNING string, ##args);

#undef MDDI_ALERT
#define MDDI_ALERT(string, args...) \
printk(KERN_ALERT string, ##args);

/* driver attributes */
static DEVICE_ATTR(display_driver_version, 0444, show_driver_version, NULL);
static DEVICE_ATTR(dbc_ctrl, 0644, show_dbc_ctrl, store_dbc_ctrl);
static DEVICE_ATTR(dbc_mode, 0644, show_dbc_mode_ctrl, store_dbc_mode_ctrl);
static DEVICE_ATTR(power_ctrl, 0644, show_power_ctrl, store_power_ctrl);
static DEVICE_ATTR(debug, 0644, show_debug_ctrl, store_debug_ctrl);

enum mddi_sony_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_POWER_ON,
	LCD_STATE_ON,
	LCD_STATE_SLEEP
};

static enum mddi_sony_lcd_state lcd_state = LCD_STATE_OFF;

static DEFINE_MUTEX(mddi_mutex);
static DEFINE_MUTEX(sony_panel_ids_lock);

static struct lcd_data_t {
	struct {
		u16 x1;
		u16 x2;
		u16 y1;
		u16 y2;
	} last_window;
} lcd_data;

static struct panel_ids {
	u32 driver_ic_id;
	u32 cell_id;
	u32 module_id;
	u32 revision_id;
} panel_ids;

/* Display resolutin */
#define SONY_HVGA_PANEL_XRES 320
#define SONY_HVGA_PANEL_YRES 480

static struct msm_fb_panel_data sony_hvga_panel_data;
static struct sony_hvga_platform_data *panel_ext;

static void sony_lcd_dbc_on(void)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	if (dbc_ctrl) {
		MDDI_DEBUG(LEVEL_PARAM, "dbc_ctrl = %d\n", dbc_ctrl);

		/* Manual brightness */
		write_client_reg_nbr(0x51, 0x000000FF, 0, 0, 0, 1);

		/* Minimum Brightness */
		write_client_reg_nbr(0x5E, 0x00000000, 0, 0, 0, 1);

		/* Mobile Image Enhancement Mode */
		write_client_reg_nbr(0x55, 0x00000003, 0, 0, 0, 1);

		/* BL Control */
		write_client_reg_nbr(0x53, 0x00000024, 0, 0, 0, 1);
	}

}

static void sony_lcd_dbc_off(void)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	if (dbc_ctrl) {
		MDDI_DEBUG(LEVEL_PARAM, "dbc_ctrl = %d\n", dbc_ctrl);

		/* Mobile Image Enhancement Mode */
		write_client_reg_nbr(0x55, 0x00000000, 0, 0, 0, 1);

		/* BL Control */
		write_client_reg_nbr(0x53, 0x00000000, 0, 0, 0, 1);
	}
}

static void sony_lcd_window_address_set(enum lcd_registers_t reg, u16 start, u16 stop)
{
	uint32 para;

	para = start;
	para = (para << 16) | (start + stop);
	para = swab32(para);
	write_client_reg_nbr(reg, para, 0, 0, 0, 1);
	if (reg == LCD_REG_COLUMN_ADDRESS) {
		lcd_data.last_window.x1 = start;
		lcd_data.last_window.x2 = stop;
	} else {
		lcd_data.last_window.y1 = start;
		lcd_data.last_window.y2 = stop;
	}
}

static void sony_lcd_driver_init(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;

	if (pdev) {
		panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

		MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

		/* Page Address Set */
		sony_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS,
				0, panel->panel_info.xres - 1);
		sony_lcd_window_address_set(LCD_REG_PAGE_ADDRESS,
				0, panel->panel_info.yres - 1);

		/* MADCTL */
		write_client_reg_nbr(0x36, 0x00000000, 0, 0, 0, 1);

		/* Interface Pixel Format, 18 bpp*/
		write_client_reg_nbr(0x3A, 0x00000077, 0, 0, 0, 1);

		/* PASSWD1 */
		write_client_reg_nbr(0xF0, 0x00005A5A, 0, 0, 0, 1);

		{
			/* DISCTL */
			static uint32 reg_disctl[5] = {
				0x080F4E3B, 0x00080808, 0x00000808, 0x084E0000, 0x00000546};
			write_client_reg_xl(0xF2, reg_disctl, 5);
		}

		/* PWRCTL */
		write_client_reg_nbr(0xF4, 0x00000007, 0x00000000,
				0x00054600, 0x0000546, 4);

		/* VCMCTL */
		write_client_reg_nbr(0xF5, 0x001A1800, 0x00000200,
				0x1A180000, 0, 3);

		/* SRGCTL */
		write_client_reg_nbr(0xF6, 0x03080001, 0x00010001,
				0x00000000, 0, 3);

		/* IFCTL */
		write_client_reg_nbr(0xF7, 0x03108048, 0x00000000,
				0, 0, 2);

		/* PANELCTL */
		write_client_reg_nbr(0xF8, 0x00000055, 0, 0, 0, 1);

		/* GAMMSEL */
		write_client_reg_nbr(0xF9, 0x00000027, 0, 0, 0, 1);

		/* PGAMMACTL */
		write_client_reg_nbr(0xFA, 0x2106030B, 0x1E272321,
				0x2A2B2B2A, 0x00000014, 4);

		/* NGAMMACTL */
		write_client_reg_nbr(0xFB, 0x2A14030B, 0x1E2A2B2B,
				0x21212327, 0x00000006, 4);

		/* MIECTRL */
		write_client_reg_nbr(0xC0, 0x00108080, 0, 0, 0, 1);

		/* BCMODE */
		write_client_reg_nbr(0xC1, 0x00000012, 0, 0, 0, 1);

		/* WRMIECTL */
		write_client_reg_nbr(0xC2, 0x01000008, 0x010000DF,
				0x0000003F, 0, 3);

		/* WRBLCTL */
		write_client_reg_nbr(0xC3, 0x00203501, 0, 0, 0, 1);

		/* PASSWD1 */
		write_client_reg_nbr(0xF0, 0x0000A5A5, 0, 0, 0, 1);

		/* FTE ON */
		write_client_reg_nbr(0x44, 0x00000000, 0, 0, 0, 1);
		write_client_reg_nbr(0x35, 0x00000000, 0, 0, 0, 1);
	}
}

static void sony_lcd_window_adjust(uint16 x1, uint16 x2, uint16 y1, uint16 y2)
{
	MDDI_DEBUG(LEVEL_PARAM, "%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	MDDI_DEBUG(LEVEL_TRACE, "%s (column) [%d, %d]\n", __func__, x1, x2);
	sony_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);

	MDDI_DEBUG(LEVEL_TRACE, "%s (page) [%d, %d]\n", __func__, y1, y2);
	sony_lcd_window_address_set(LCD_REG_PAGE_ADDRESS, y1, y2);

#ifdef TEMP_REMOVE_FOR_HW_BUG_FIX
	if (lcd_data.last_window.x1 != x1 || lcd_data.last_window.x2 != x2) {
		MDDI_DEBUG(LEVEL_TRACE, "%s (column) [%d, %d]\n",
				__func__, x1, x2);
		sony_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);
	}

	if (lcd_data.last_window.y1 != y1 || lcd_data.last_window.y2 != y2) {
		MDDI_DEBUG(LEVEL_TRACE, "%s (page) [%d, %d]\n",
				__func__, y1, y2);
		sony_lcd_window_address_set(LCD_REG_PAGE_ADDRESS, y1, y2);
	}

	/* Temp solution for 1st and 2nd cut HW sample - To be removed! */
	if (((panel_ids.driver_ic_id & 0xFF) ==
				MDDI_sony_DISPLAY_DRIVER_IC_ID) &&
				((panel_ids.cell_id & 0xFF) == 0x00))
		write_client_reg_nbr(0x2C, 0, 0, 0, 0, 1);
#endif

	mutex_unlock(&mddi_mutex);
}

static void sony_lcd_enter_sleep(void)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	/* Display off */
	write_client_reg_nbr(0x28, 0, 0, 0, 0, 1);
	mddi_wait(50); /* >50 ms */
	/* Sleep in */
	write_client_reg_nbr(0x10, 0, 0, 0, 0, 1);
	mddi_wait(120); /* >120 ms */
}


static void sony_lcd_exit_sleep(void)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	/* Sleep out */
	write_client_reg_nbr(0x11, 0x00000000, 0, 0, 0, 1);
	mddi_wait(120); /* >120 ms */

	/* Display on */
	write_client_reg_nbr(0x29, 0x00000000, 0, 0, 0, 1);
}


static void sony_lcd_enter_deepstandby(void)
{
	MDDI_DEBUG(LEVEL_TRACE,
		"%s [%d]\n", __func__, lcd_state);
	/* PASSWD2 */
	write_client_reg_nbr(0xF1, 0x00005A5A, 0, 0, 0, 1);

	/* DSTB */
	write_client_reg_nbr(0xDF, 0x00000001, 0, 0, 0, 1);
}

static void sony_toggle_reset(struct platform_device *pdev)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		if (panel_ext->exit_deep_standby)
			panel_ext->exit_deep_standby();
	}
}

static void sony_lcd_exit_deepstandby(struct platform_device *pdev)
{
	MDDI_DEBUG(LEVEL_TRACE,
		"%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		/* Reset toggle */
		sony_toggle_reset(pdev);
	}
}

static void sony_power_on(struct platform_device *pdev)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		if (panel_ext->power_on)
			panel_ext->power_on();
	}
}

static void sony_power_off(struct platform_device *pdev)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		if (panel_ext->power_off)
			panel_ext->power_off();
	}
}

static int mddi_sony_lcd_on(struct platform_device *pdev)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);
	if (power_ctrl) {
		switch (lcd_state) {
		case LCD_STATE_OFF:
			sony_power_on(pdev);
			lcd_state = LCD_STATE_POWER_ON;
			break;

		case LCD_STATE_POWER_ON:
			sony_lcd_driver_init(pdev);
			sony_lcd_exit_sleep();
			sony_lcd_dbc_on();
			lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_SLEEP:
			sony_lcd_exit_deepstandby(pdev);
			sony_lcd_driver_init(pdev);
			sony_lcd_exit_sleep();
			sony_lcd_dbc_on();
			lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_ON:
			break;

		default:
			break;
		}
	}
	mutex_unlock(&mddi_mutex);
	return 0;
}


static int mddi_sony_lcd_off(struct platform_device *pdev)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);
	if (power_ctrl) {
		switch (lcd_state) {
		case LCD_STATE_POWER_ON:
			sony_power_off(pdev);
			lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_ON:
			sony_lcd_dbc_off();
			sony_lcd_enter_sleep();
			sony_lcd_enter_deepstandby();
			lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_SLEEP:
			sony_power_off(pdev);
			lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_OFF:
			break;

		default:
			break;
		}
	}
	mutex_unlock(&mddi_mutex);
	return 0;
}

static ssize_t show_driver_version(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", MDDI_DRIVER_VERSION);
}

static ssize_t show_debug_ctrl(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	return snprintf(buf, PAGE_SIZE, "%i\n", debug);
}

static ssize_t store_debug_ctrl(struct device *dev_p,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;

	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		MDDI_ALERT("Invalid flag for debug\n");
		ret = -EINVAL;
		goto unlock;
	}

	debug = ret;

	MDDI_DEBUG(LEVEL_PARAM, "%s debug set to %d\n", __func__, debug);
	ret = strnlen(buf, count);

unlock:
	mutex_unlock(&mddi_mutex);
	return ret;
}

static ssize_t show_dbc_ctrl(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	return snprintf(buf, PAGE_SIZE, "%i\n", dbc_ctrl);
}

static ssize_t store_dbc_ctrl(struct device *dev_p,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;

	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		MDDI_ALERT("Invalid flag for dbc ctrl\n");
		ret = -EINVAL;
		goto unlock;
	}

	if (ret)
		dbc_ctrl = 1;
	else
		dbc_ctrl = 0;

	MDDI_DEBUG(LEVEL_PARAM, "%s dbc_ctrl set to %d\n", __func__, dbc_ctrl);

	ret = strnlen(buf, count);

unlock:
	mutex_unlock(&mddi_mutex);
	return ret;
}

static ssize_t show_dbc_mode_ctrl(struct device *dev_p,
				struct device_attribute *attr,
				char *buf)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	return snprintf(buf, PAGE_SIZE, "%i\n", dbc_mode);
}

static ssize_t store_dbc_mode_ctrl(struct device *dev_p,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	ssize_t ret;

	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		MDDI_ALERT("Invalid flag for dbc mode\n");
		ret = -EINVAL;
		goto unlock;
	}

	switch (ret) {
	case DBC_MODE_UI:
	case DBC_MODE_IMAGE:
	case DBC_MODE_VIDEO:
		dbc_mode = ret;
		break;
	default:
		MDDI_ALERT("Invalid value for dbc mode\n");
		ret = -EINVAL;
		goto unlock;
	}

	if (lcd_state != LCD_STATE_ON) {
		MDDI_ALERT("%s: LCD in sleep. "
			"Do not perform any register commands!\n", __func__);
		ret = -EINVAL;
		goto unlock;
	}

	/* Mobile Image Enhancement Mode */
	write_client_reg_nbr(0x55, dbc_mode, 0, 0, 0, 1);

	MDDI_DEBUG(LEVEL_PARAM, "%s dbc_mode set to %d\n", __func__, dbc_mode);

	ret = strnlen(buf, count);

unlock:
	mutex_unlock(&mddi_mutex);
	return ret;
}

static ssize_t show_power_ctrl(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	return snprintf(buf, PAGE_SIZE, "%i\n", power_ctrl);
}

static ssize_t store_power_ctrl(struct device *dev_p,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	ssize_t ret;
	struct platform_device *pdev = NULL;

	pdev = container_of(dev_p, struct platform_device, dev);

	if (!pdev) {
		ret = -ENOMEM;
		goto error;
	}

	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		MDDI_ALERT("Invalid flag for power_ctrl\n");
		ret = -EINVAL;
		goto unlock;
	}

	if (ret) {
		power_ctrl = POWER_ON;
		sony_power_on(pdev);
		sony_lcd_driver_init(pdev);
		sony_lcd_exit_sleep();
		sony_lcd_dbc_on();
		lcd_state = LCD_STATE_ON;
	} else {
		power_ctrl = POWER_OFF;
		sony_lcd_dbc_off();
		sony_lcd_enter_sleep();
		sony_power_off(pdev);
		lcd_state = LCD_STATE_SLEEP;
	}

	MDDI_DEBUG(LEVEL_PARAM, "%s power_ctrl set to %d\n",
			__func__, power_ctrl);

	ret = strnlen(buf, count);

unlock:
	mutex_unlock(&mddi_mutex);
error:
	return ret;
}

static void lcd_attribute_register(struct platform_device *pdev)
{
	int ret;

	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);

	ret = device_create_file(&pdev->dev, &dev_attr_display_driver_version);
	if (ret != 0)
		MDDI_ERROR("Failed to register display_driver_version"
			   "attributes (%d)\n", ret);

	ret = device_create_file(&pdev->dev, &dev_attr_debug);
	if (ret != 0)
		MDDI_ERROR("Failed to register debug attributes (%d)\n", ret);

	ret = device_create_file(&pdev->dev, &dev_attr_dbc_ctrl);
	if (ret != 0)
		MDDI_ERROR("Failed to register dbc attributes (%d)\n", ret);

	ret = device_create_file(&pdev->dev, &dev_attr_dbc_mode);
	if (ret != 0)
		MDDI_ERROR("Failed to register dbc mode attributes (%d)\n",
				ret);

	ret = device_create_file(&pdev->dev, &dev_attr_power_ctrl);
	if (ret != 0)
		MDDI_ERROR("Failed to register power attributes (%d)\n", ret);
}

static int check_panel_ids(void)
{
	int ret;

	mutex_lock(&sony_panel_ids_lock);

#ifdef ENABLE_FOR_SW_SAMPLE
	ret = mddi_host_register_read(LCD_REG_DRIVER_IC_ID,
			&panel_ids.driver_ic_id, 1, MDDI_HOST_PRIM);
	if (ret < 0) {
		MDDI_DEBUG(LEVEL_TRACE, "%s Failed to read LCD_REG_DRIVER_IC_ID\n",
							DBG_STR);
		mutex_unlock(&sony_panel_ids_lock);
		return -ENODEV;
	}

	ret = mddi_host_register_read(LCD_REG_CELL_ID, &panel_ids.cell_id,
				      1, MDDI_HOST_PRIM);
	if (ret < 0) {
		MDDI_DEBUG(LEVEL_TRACE, "%s Failed to read LCD_REG_CELL_ID\n",
							DBG_STR);
		panel_ids.cell_id = 0xFF;
	}
#else
	ret = mddi_host_register_read(LCD_REG_CELL_ID, &panel_ids.cell_id,
				      1, MDDI_HOST_PRIM);
	if ((ret < 0)) {
		MDDI_DEBUG(LEVEL_TRACE, "%s Failed to read LCD_REG_CELL_ID\n",
							DBG_STR);
		mutex_unlock(&sony_panel_ids_lock);
		return -ENODEV;
	}

	if (((panel_ids.cell_id & 0xFF) != MDDI_SONY_CELL_ID1) &&
		((panel_ids.cell_id & 0xFF) != MDDI_SONY_CELL_ID2)) {
		MDDI_DEBUG(LEVEL_TRACE, "%s This is not sony LCD\n",
							DBG_STR);
		mutex_unlock(&sony_panel_ids_lock);
		return -ENODEV;
	}

	ret = mddi_host_register_read(LCD_REG_DRIVER_IC_ID,
			&panel_ids.driver_ic_id, 1, MDDI_HOST_PRIM);
	if (ret < 0) {
		MDDI_DEBUG(LEVEL_TRACE, "%s Failed to read LCD_REG_DRIVER_IC_ID\n",
							DBG_STR);
		panel_ids.driver_ic_id = 0xFF;
	}
#endif

	ret = mddi_host_register_read(LCD_REG_MODULE_ID, &panel_ids.module_id,
							1, MDDI_HOST_PRIM);
	if (ret < 0) {
		MDDI_DEBUG(LEVEL_TRACE, "%s Failed to read LCD_REG_MODULE_ID\n",
							DBG_STR);
		panel_ids.module_id = 0xFF;
	}
	ret = mddi_host_register_read(LCD_REG_REVISION_ID,
						&panel_ids.revision_id,
						1, MDDI_HOST_PRIM);
	if (ret < 0) {
		MDDI_DEBUG(LEVEL_TRACE, "%s Failed to read LCD_REG_REVISION_ID\n",
							DBG_STR);
		panel_ids.revision_id = 0xFF;
	}

	mutex_unlock(&sony_panel_ids_lock);
	return 0;
}

static int mddi_sony_lcd_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct msm_fb_panel_data *panel_data;

	dev_dbg(&pdev->dev, "probing\n");

	panel_ext = pdev->dev.platform_data;
	if (panel_ext == NULL)
		return -EINVAL;

	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);
	if (!pdev) {
		MDDI_ERROR("Display failed in probe\n");
		ret = -ENODEV;
		goto exit_point;
	}
	if (!pdev->dev.platform_data) {
		MDDI_ERROR("Display failed in probe, no platform data\n");
		ret = -ENODEV;
		goto exit_point;
	}

	panel_data = &sony_hvga_panel_data;
	/*platform_set_drvdata(pdev, panel_data);*/


	if (!check_panel_ids()) {
		MDDI_DEBUG(LEVEL_TRACE, "%s Found display with cell ID = 0x%x, "
				"module ID = 0x%x, revision ID = 0x%x, "
				"driver IC ID = 0x%x, driver ID = 0x%x\n",
				DBG_STR, panel_ids.cell_id & 0xFF,
				panel_ids.module_id & 0xFF,
				panel_ids.revision_id & 0xFF,
				panel_ids.driver_ic_id & 0xFF,
				MDDI_DRIVER_VERSION);

		lcd_state = LCD_STATE_POWER_ON;
		power_ctrl = POWER_ON;

		panel_data->on  = mddi_sony_lcd_on;
		panel_data->off = mddi_sony_lcd_off;
		panel_ext->window_adjust =
						sony_lcd_window_adjust;

		pdev->dev.platform_data = &sony_hvga_panel_data;
		/* adds mfd on driver_data */
		msm_fb_add_device(pdev);

		/* Add SYSFS to module */
		lcd_attribute_register(pdev);

		ret = 0;
	}
exit_point:
	return ret;
}

static int __devexit mddi_sony_lcd_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_display_driver_version);
	device_remove_file(&pdev->dev, &dev_attr_debug);
	device_remove_file(&pdev->dev, &dev_attr_dbc_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_dbc_mode);
	device_remove_file(&pdev->dev, &dev_attr_power_ctrl);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_sony_lcd_probe,
	.remove = __devexit_p(mddi_sony_lcd_remove),
	.driver = {
		.name   = "mddi_sony_s6d05a1_hvga",
	},
};

static void __init msm_mddi_sony_hvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &sony_hvga_panel_data;

	panel_data->panel_info.xres = SONY_HVGA_PANEL_XRES;
	panel_data->panel_info.yres = SONY_HVGA_PANEL_YRES;
	panel_data->panel_info.pdest = DISPLAY_1;
	panel_data->panel_info.type = MDDI_PANEL;
	/*panel_date->panel_info.mddi.vdopkt = 0x0023;*/
	panel_data->panel_info.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	panel_data->panel_info.wait_cycle = 0;
	panel_data->panel_info.bpp = 24;
	/*panel_data->panel_info.bpp = 16;*/
	panel_data->panel_info.clk_rate = 192000000;
	panel_data->panel_info.clk_min =  190000000;
	panel_data->panel_info.clk_max = 200000000;
	panel_data->panel_info.fb_num = 2;
	panel_data->panel_info.bl_max = 4;
	panel_data->panel_info.bl_min = 1;

	panel_data->panel_info.lcd.vsync_enable = TRUE;
	/*panel_data->panel_info.lcd.vsync_enable = FALSE;*/
	panel_data->panel_info.lcd.refx100 = 8500;
	panel_data->panel_info.lcd.v_back_porch = 1;
	panel_data->panel_info.lcd.v_front_porch = 16;
	panel_data->panel_info.lcd.v_pulse_width = 0;
	panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
	panel_data->panel_info.lcd.vsync_notifier_period = 0;
	/*panel_data->panel_info.lcd.vsync_notifier_period = (1 * HZ);*/
}

static void __init msm_mddi_semc_mogami_display_init(void)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s\n", __func__);
	/*semc_mogami_lcd_power_on(11, 2, 21);*/
	msm_mddi_sony_hvga_display_device_init();
}

static int __init mddi_sony_lcd_init(void)
{
	int ret;

	MDDI_DEBUG(LEVEL_TRACE,  "%s (ver:0x%x) [%d]\n",
			__func__, MDDI_DRIVER_VERSION, lcd_state);

	msm_mddi_semc_mogami_display_init();
	ret = platform_driver_register(&this_driver);

	return ret;
}

static void __exit mddi_sony_lcd_exit(void)
{
	MDDI_DEBUG(LEVEL_TRACE, "%s [%d]\n", __func__, lcd_state);
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jei1.zhang@sonyericsson.com");
MODULE_DESCRIPTION("MDDI implementation of the sony HVGA display");

module_init(mddi_sony_lcd_init);
module_exit(mddi_sony_lcd_exit);
