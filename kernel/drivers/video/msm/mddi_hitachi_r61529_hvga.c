/* drivers/video/msm/mddi_hitachi_r61529_hvga.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/autoconf.h>
#include <linux/mddi_hitachi_r61529_hvga.h>

/* Internal version number */
#define MDDI_DRIVER_VERSION 0x0004

/* DISPLAY ID value */
#define MDDI_HITACHI_CELL_ID 0xFA

/* Frame time, used for delays */
#define MDDI_FRAME_TIME 13

#define POWER_OFF 0
#define POWER_ON  1

enum lcd_registers {
	LCD_REG_COLUMN_ADDRESS = 0x2a,
	LCD_REG_PAGE_ADDRESS = 0x2b,
	LCD_REG_DRIVER_IC_ID = 0xA1, /* TODO: check: temp SW sample */
	LCD_REG_CELL_ID = 0xDA,
	LCD_REG_MODULE_ID = 0xDB,
	LCD_REG_REVISION_ID = 0xDC
};

enum mddi_hitachi_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_POWER_ON,
	LCD_STATE_ON,
	LCD_STATE_SLEEP
};

struct coords {
	u16 x1;
	u16 x2;
	u16 y1;
	u16 y2;
};

struct hitachi_record {
	struct hitachi_hvga_platform_data *pdata;
	struct mutex mddi_mutex;
	int power_ctrl;
	enum mddi_hitachi_lcd_state lcd_state;
	struct coords last_window;
	struct platform_device *pdev;
};

#define DBC_CONTROL_SIZE  5
static u32 dbc_control_on_data[DBC_CONTROL_SIZE] = {
	0xF0020201, 0x04C0C0F0, 0x1990901F, 0x00A36235, 0x00000000};
static u32 dbc_control_off_data[DBC_CONTROL_SIZE] = {
	0xFF020200, 0x04EBEBFF, 0x1F90901F, 0x00AA6B3D, 0x00000000};

#ifdef MDDI_HITACHI_DISPLAY_INITIAL
#define GAMMA_SETTING_SIZE 6
static u32 gamma_setting_A[GAMMA_SETTING_SIZE] = {
	0x1F150C00, 0x273B482E, 0x0104101A, 0x1F150C00, 0x273B482E, 0x0104101A};
static u32 gamma_setting_B[GAMMA_SETTING_SIZE] = {
	0x1F150C00, 0x273B482E, 0x0104101A, 0x1F150C00, 0x273B482E, 0x0104101A};
static u32 gamma_setting_C[GAMMA_SETTING_SIZE] = {
	0x1F150C00, 0x273B482E, 0x0104101A, 0x1F150C00, 0x273B482E, 0x0104101A};
#endif

static void hitachi_lcd_dbc_on(struct hitachi_record *rd)
{
	if (rd->pdata->dbc_on) {
		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x04, TRUE, 0);

		/* Backlight control1 set */
		mddi_host_register_write_xl(0xB8, dbc_control_on_data,
				DBC_CONTROL_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control2 set */
		mddi_host_register_write16(0xB9, 0x0802FF00, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x03, TRUE, 0);
	}
}

static void hitachi_lcd_dbc_off(struct hitachi_record *rd)
{
	if (rd->pdata->dbc_on) {
		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x04, TRUE, 0);

		/* Backlight control1 set */
		mddi_host_register_write_xl(0xB8, dbc_control_off_data,
				DBC_CONTROL_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control2 set */
		mddi_host_register_write16(0xB9, 0x0802FF00, 0, 0, 0, 1, TRUE,
							NULL, MDDI_HOST_PRIM);

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x03, TRUE, 0);
	}
}

#ifdef MDDI_HITACHI_DISPLAY_INITIAL
static void hitachi_lcd_driver_init(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;

	if (pdev) {
		panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x04, TRUE, 0);

		/* Frame Memory Acess and Interface Setting */
		mddi_host_register_write16(0xB3, 0x00000002, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* Display mode and Frame memory write mode */
		mddi_queue_register_write(0xB4, 0x00, TRUE, 0);

		/* DSI Control */
		mddi_host_register_write16(0xB6, 0x00008351, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* MDDI Control */
		mddi_host_register_write16(0xB7, 0x21110000, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control1 set */
		mddi_host_register_write_xl(0xB8, dbc_control_on_data,
			DBC_CONTROL_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control2 set */
		mddi_host_register_write16(0xB9, 0x0802FF00, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* Display Timing Setting for Normal Mode*/
		mddi_host_register_write16(0xC1, 0x08082707, 0x00000030,
			0, 0, 2, TRUE, NULL, MDDI_HOST_PRIM);

		/* Source/Gate Diving Timing Setting */
		mddi_host_register_write16(0xC4, 0x03030040, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* DPI Polarity Control */
		mddi_queue_register_write(0xC6, 0x00, TRUE, 0);

		/* Gamma Setting A set */
		mddi_host_register_write_xl(0xC8, gamma_setting_A,
			GAMMA_SETTING_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Gamma Setting B set */
		mddi_host_register_write_xl(0xC9, gamma_setting_B,
			GAMMA_SETTING_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Gamma Setting C set */
		mddi_host_register_write_xl(0xCA, gamma_setting_C,
			GAMMA_SETTING_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Power Setting */
		mddi_host_register_write16(0xD0, 0x200806A9, 0x0001043C,
			0x06000108, 0x20000001, 4,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* VCOM Setting */
		mddi_host_register_write16(0xD1, 0x00202000, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* NV Memory Access Control */
		mddi_host_register_write16(0xE0, 0x00000000, 0, 0, 0, 1,
			TRUE, NULL, MDDI_HOST_PRIM);

		/* Set_DDB Write Control */
		mddi_host_register_write16(0xE1, 0x01010101, 0x00000000,
			0, 0, 2, TRUE, NULL, MDDI_HOST_PRIM);

		/* NV Memory Load Control */
		mddi_queue_register_write(0xE2, 0xFF, TRUE, 0);

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x03, TRUE, 0);
	}
}
#endif

static void hitachi_lcd_exit_sleep(struct hitachi_record *rd)
{
	/*Address Mode Set */
	mddi_queue_register_write(0x36, 0x00, TRUE, 0);

	/*Pixle Format Set */
	mddi_queue_register_write(0x3A, 0x77, TRUE, 0);

	/* Exit sleep mode */
	mddi_queue_register_write(0x11, 0x00, TRUE, 0);

	mddi_wait(110);/* >108ms */

	/* Set tear on */
	mddi_queue_register_write(0x35, 0x00, TRUE, 0);
}

static void hitachi_lcd_enter_sleep(void)
{
	/* Set tear off */
	mddi_queue_register_write(0x34, 0x00, TRUE, 0);

	/* Enter sleep mode */
	mddi_queue_register_write(0x10, 0x00, TRUE, 0);
	mddi_wait(100);/* >90ms */
}

static void hitachi_lcd_display_on(void)
{
	mddi_queue_register_write(0x29, 0x00, TRUE, 0);
}

static void hitachi_lcd_display_off(void)
{
	mddi_queue_register_write(0x28, 0x00, TRUE, 0);
	mddi_wait(21); /* >20 ms */
}

static void hitachi_lcd_enter_deepstandby(void)
{
	mddi_queue_register_write(0xB0, 0x00, TRUE, 0);
	mddi_queue_register_write(0xB1, 0x01, TRUE, 0);
	mddi_wait(2); /* >1 ms */
}

static void hitachi_toggle_reset(struct hitachi_record *rd)
{
	if (rd->pdata->exit_deep_standby)
		rd->pdata->exit_deep_standby();
}

static void hitachi_lcd_exit_deepstandby(struct hitachi_record *rd)
{
	hitachi_toggle_reset(rd);
}

static void hitachi_power_on(struct hitachi_record *rd)
{
	if (rd->pdata->power_on)
		rd->pdata->power_on();
}

static void hitachi_power_off(struct hitachi_record *rd)
{
	if (rd->pdata->power_off)
		rd->pdata->power_off();
}

static struct hitachi_record *get_hitachi_record_from_mfd(
						struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	if (!pdev)
		return NULL;
	mfd = platform_get_drvdata(pdev);
	if (mfd == NULL)
		return NULL;
	if (mfd->key != MFD_KEY || mfd->panel_pdev == NULL)
		return NULL;

	return platform_get_drvdata(mfd->panel_pdev);
}

static int mddi_hitachi_lcd_on(struct platform_device *pdev)
{
	int ret = 0;
	struct hitachi_record *rd;

	rd = get_hitachi_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_OFF:
			hitachi_power_on(rd);
			rd->lcd_state = LCD_STATE_POWER_ON;
			break;

		case LCD_STATE_POWER_ON:
			hitachi_lcd_exit_sleep(rd);
#ifdef MDDI_HITACHI_DISPLAY_INITIAL
			hitachi_lcd_driver_init(pdev);
#endif
			hitachi_lcd_dbc_on(rd);
			hitachi_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;
		case LCD_STATE_SLEEP:
			hitachi_lcd_exit_deepstandby(rd);
			hitachi_lcd_exit_sleep(rd);
#ifdef MDDI_HITACHI_DISPLAY_INITIAL
			hitachi_lcd_driver_init(pdev);
#endif
			hitachi_lcd_dbc_on(rd);
			hitachi_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_ON:
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return ret;
}


static int mddi_hitachi_lcd_off(struct platform_device *pdev)
{
	int ret = 0;
	struct hitachi_record *rd;

	rd = get_hitachi_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			hitachi_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_ON:
			hitachi_lcd_display_off();
			hitachi_lcd_dbc_off(rd);
			hitachi_lcd_enter_sleep();
			hitachi_lcd_enter_deepstandby();
			rd->lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_SLEEP:
			hitachi_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_OFF:
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return 0;
}

static ssize_t show_driver_info(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "R61529 Hitachi HVGA, drv ver:0x%x\n",
							MDDI_DRIVER_VERSION);
}

/* driver attributes */
static DEVICE_ATTR(display_driver_info, 0444, show_driver_info, NULL);

static void lcd_attribute_register(struct platform_device *pdev)
{
	int ret;

	ret = device_create_file(&pdev->dev, &dev_attr_display_driver_info);
	if (ret != 0)
		pr_err("Failed to register display_driver_version"
						"attributes (%d)\n", ret);
}

static int check_panel_ids(struct hitachi_record *rd)
{
	int ret = 0;
	u32 readID = 0;

	mutex_lock(&rd->mddi_mutex);

	/* Temporaryly for EP build because currently interface
	   can't support HITACHI 0xA1 read. EP build will read 0 in 0xDA,
	   which for now will be considered to be a Hitachi display */
	ret = mddi_host_register_read(0xDA, &readID, 1, MDDI_HOST_PRIM);
	if (ret < 0) {
		pr_err("mddi_hitachi_hvga: Failed to read Display ID\n");
		ret = -ENODEV;
		goto error;
	}
	if ((readID & 0xFF) != 0) {
		pr_err("mddi_hitachi_hvga: Detected a non-Hitachi display\n");
		ret = -ENODEV;
		goto error;
	}
	printk(KERN_INFO "mddi_hitachi_hvga: Found Hitachi HVGA display,"
						" 0xDA = 0x%x\n", readID);
error:
	mutex_unlock(&rd->mddi_mutex);
	return ret;
}

static int mddi_hitachi_lcd_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct hitachi_record *rd;

	if (!pdev) {
		pr_err("%s: no platform_device\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}
	if (!pdev->dev.platform_data) {
		pr_err("%s: no platform data\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}

	rd = kzalloc(sizeof(struct hitachi_record), GFP_KERNEL);
	if (rd == NULL) {
		ret = -ENOMEM;
		goto exit_point;
	}

	rd->pdev = pdev;
	rd->pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, rd);
	mutex_init(&rd->mddi_mutex);

	if (!check_panel_ids(rd)) {
		rd->lcd_state = LCD_STATE_POWER_ON;
		rd->power_ctrl = POWER_ON;

		rd->pdata->panel_data->panel_info.mddi.vdopkt =
						MDDI_DEFAULT_PRIM_PIX_ATTR;
		rd->pdata->panel_data->panel_info.lcd.vsync_enable = TRUE;
		rd->pdata->panel_data->panel_info.lcd.refx100 = 8500;
		rd->pdata->panel_data->panel_info.lcd.v_back_porch = 8;
		rd->pdata->panel_data->panel_info.lcd.v_front_porch = 8;
		rd->pdata->panel_data->panel_info.lcd.v_pulse_width = 0;
		rd->pdata->panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
		rd->pdata->panel_data->panel_info.lcd.vsync_notifier_period = 0;
		rd->pdata->panel_data->on  = mddi_hitachi_lcd_on;
		rd->pdata->panel_data->off = mddi_hitachi_lcd_off;
		pdev->dev.platform_data = rd->pdata->panel_data;

		/* adds mfd on driver_data */
		msm_fb_add_device(pdev);

		/* Add SYSFS to module */
		lcd_attribute_register(pdev);

		printk(KERN_INFO "%s: Probe success!", __func__);
		ret = 0;
	} else {
		kfree(rd);
	}
exit_point:
	return ret;
}

static int __devexit mddi_hitachi_lcd_remove(struct platform_device *pdev)
{
	struct hitachi_record *rd;

	device_remove_file(&pdev->dev, &dev_attr_display_driver_info);
	rd = platform_get_drvdata(pdev);
	if (rd)
		kfree(rd);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_hitachi_lcd_probe,
	.remove = __devexit_p(mddi_hitachi_lcd_remove),
	.driver = {
		.name = MDDI_HITACH_R61529_HVGA_NAME,
	},
};

static int __init mddi_hitachi_lcd_init(void)
{
	return platform_driver_register(&this_driver);
}

static void __exit mddi_hitachi_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("johan.olson@sonyericsson.com");
MODULE_DESCRIPTION("Driver for Renesas R61529 with Hitachi HVGA panel");

module_init(mddi_hitachi_lcd_init);
module_exit(mddi_hitachi_lcd_exit);
