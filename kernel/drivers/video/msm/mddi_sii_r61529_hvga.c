/* drivers/video/msm/mddi_sii_r61529_hvga.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Macro Luo <macro.luo@sonyericsson.com>
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
#include <linux/mddi_sii_r61529_hvga.h>

/* Internal version number */
#define MDDI_DRIVER_VERSION 0x0004

/* DISPLAY ID value */
#define MDDI_SII_CELL_ID 0xFA

/* DISPLAY DRIVERIC ID value */
#define MDDI_SII_DISPLAY_DRIVER_IC_ID 0x06

/* Frame time, used for delays */
#define MDDI_FRAME_TIME 13

#define POWER_OFF 0
#define POWER_ON  1

enum lcd_registers {
	LCD_REG_COLUMN_ADDRESS = 0x2a,
	LCD_REG_PAGE_ADDRESS = 0x2b,
	LCD_REG_DRIVER_IC_ID = 0xA1,
	LCD_REG_MODULE_ID = 0xA8,
	LCD_REG_REVISION_ID = 0xA8
};

enum mddi_sii_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_POWER_ON,
	LCD_STATE_DISPLAY_OFF,
	LCD_STATE_ON,
	LCD_STATE_SLEEP
};

struct coords {
	u16 x1;
	u16 x2;
	u16 y1;
	u16 y2;
};

struct panel_ids {
	u32 driver_ic_id;
	u32 module_id;
	u32 revision_id;
};


struct sii_record {
	struct sii_hvga_platform_data *pdata;
	struct mutex mddi_mutex;
	int power_ctrl;
	enum mddi_sii_lcd_state lcd_state;
	struct coords last_window;
	struct platform_device *pdev;
	struct panel_ids pid;
};

#define DBC_CONTROL_SIZE  20
/* maximum 30% setting*/
static u32 dbc_control_on_data[DBC_CONTROL_SIZE] = {
		0x00000001, 0x00000009, 0x00000009, 0x000000FF, 0X000000FF,
		0x000000E1, 0x000000E1, 0x0000000C, 0x00000018, 0X00000010,
		0x00000010, 0x00000037, 0x0000005A, 0x00000087, 0X000000BE,
		0x000000FF, 0x00000000, 0x00000000, 0x00000000, 0X00000000};

static u32 dbc_control_off_data[DBC_CONTROL_SIZE] = {
		0x00000000, 0x00000006, 0x00000006, 0x000000FF, 0X000000FF,
		0x000000ED, 0x000000ED, 0x0000000C, 0x00000018, 0X00000010,
		0x00000010, 0x00000037, 0x0000005A, 0x00000087, 0X000000BE,
		0x000000FF, 0x00000000, 0x00000000, 0x00000000, 0X00000000};

#ifdef MDDI_SII_DISPLAY_INITIAL
#define GAMMA_SETTING_SIZE 6
static u32 gamma_setting_A[GAMMA_SETTING_SIZE] = {
	0x1F150C00, 0x273B482E, 0x0104101A, 0x1F150C00, 0x273B482E, 0x0104101A};
static u32 gamma_setting_B[GAMMA_SETTING_SIZE] = {
	0x1F150C00, 0x273B482E, 0x0104101A, 0x1F150C00, 0x273B482E, 0x0104101A};
static u32 gamma_setting_C[GAMMA_SETTING_SIZE] = {
	0x1F150C00, 0x273B482E, 0x0104101A, 0x1F150C00, 0x273B482E, 0x0104101A};
#endif

static void sii_lcd_dbc_on(struct sii_record *rd)
{
	if (rd->pdata->dbc_on) {
		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x04, TRUE, 0);

		/* Backlight control1 set */
		mddi_host_register_write_xl(0xB8, dbc_control_on_data,
				DBC_CONTROL_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control2 set */
		mddi_host_register_write16(0xB9, 0x00000000, 0x000000F2,
				0x00000001, 0x00000008, 4,
				TRUE, NULL, MDDI_HOST_PRIM);

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x03, TRUE, 0);
	}
}

static void sii_lcd_dbc_off(struct sii_record *rd)
{
	if (rd->pdata->dbc_on) {
		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x04, TRUE, 0);

		/* Backlight control1 set */
		mddi_host_register_write_xl(0xB8, dbc_control_off_data,
				DBC_CONTROL_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control2 set */
		mddi_host_register_write16(0xB9, 0x00000000, 0x000000FF,
				0x00000002, 0x00000008, 4,
				TRUE, NULL, MDDI_HOST_PRIM);

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x03, TRUE, 0);
	}
}

static void sii_lcd_window_address_set(enum lcd_registers reg,
						u16 start, u16 stop)
{
	uint32 para;

	para = start;
	para = (para << 16) | (start + stop);
	para = swab32(para);
	mddi_queue_register_write(reg, para, TRUE, 0);
}

#ifdef MDDI_SII_DISPLAY_INITIAL
static void sii_lcd_driver_init(struct platform_device *pdev)
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
		mddi_host_register_write16(0xB9, 0x00000000, 0x000000F2,
			0x00000001, 0x00000008, 4,
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

static void sii_lcd_window_adjust(uint16 x1, uint16 x2,
					uint16 y1, uint16 y2)
{
	sii_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);
	sii_lcd_window_address_set(LCD_REG_PAGE_ADDRESS, y1, y2);
	mddi_queue_register_write(0x3C, 0x00, TRUE, 0);
}

static void sii_lcd_exit_sleep(struct sii_record *rd)
{
	/* Page Address Set */
	mddi_queue_register_write(0x2A, 0x0000013F, TRUE, 0);
	mddi_queue_register_write(0x2B, 0x000001DF, TRUE, 0);

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

static void sii_lcd_enter_sleep(void)
{
	/* Set tear off */
	mddi_queue_register_write(0x34, 0x00, TRUE, 0);

	/* Enter sleep mode */
	mddi_queue_register_write(0x10, 0x00, TRUE, 0);
	mddi_wait(100);/* >90ms */
}

static void sii_lcd_display_on(void)
{
	mddi_queue_register_write(0x29, 0x00, TRUE, 0);
}

static void sii_lcd_display_off(void)
{
	mddi_queue_register_write(0x28, 0x00, TRUE, 0);
	mddi_wait(21); /* >20 ms */
}

static void sii_lcd_enter_deepstandby(void)
{
	mddi_queue_register_write(0xB0, 0x00, TRUE, 0);
	mddi_queue_register_write(0xB1, 0x01, TRUE, 0);
	mddi_wait(2); /* >1 ms */
}

static void sii_toggle_reset(struct sii_record *rd)
{
	if (rd->pdata->exit_deep_standby)
		rd->pdata->exit_deep_standby();
}

static void sii_lcd_exit_deepstandby(struct sii_record *rd)
{
	sii_toggle_reset(rd);
}

static void sii_power_on(struct sii_record *rd)
{
	if (rd->pdata->power_on)
		rd->pdata->power_on();
}

static void sii_power_off(struct sii_record *rd)
{
	if (rd->pdata->power_off)
		rd->pdata->power_off();
}

static struct sii_record *get_sii_record_from_mfd(
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

static int mddi_sii_ic_on_panel_off(struct platform_device *pdev)
{
	int ret = 0;
	struct sii_record *rd;

	rd = get_sii_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_OFF:
			sii_power_on(rd);
			rd->lcd_state = LCD_STATE_POWER_ON;
			break;

		case LCD_STATE_POWER_ON:
			sii_lcd_exit_sleep(rd);
#ifdef MDDI_SII_DISPLAY_INITIAL
			sii_lcd_driver_init(pdev);
#endif
			sii_lcd_dbc_on(rd);
			rd->lcd_state = LCD_STATE_DISPLAY_OFF;
			break;

		case LCD_STATE_SLEEP:
			sii_lcd_exit_deepstandby(rd);
			sii_lcd_exit_sleep(rd);
#ifdef MDDI_SII_DISPLAY_INITIAL
			sii_lcd_driver_init(pdev);
#endif
			sii_lcd_dbc_on(rd);
			rd->lcd_state = LCD_STATE_DISPLAY_OFF;
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return ret;
}

static int mddi_sii_ic_on_panel_on(struct platform_device *pdev)
{
	int ret = 0;
	struct sii_record *rd;

	rd = get_sii_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			sii_lcd_exit_sleep(rd);
#ifdef MDDI_SII_DISPLAY_INITIAL
			sii_lcd_driver_init(pdev);
#endif
			sii_lcd_dbc_on(rd);
			sii_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_SLEEP:
			sii_lcd_exit_deepstandby(rd);
			sii_lcd_exit_sleep(rd);
#ifdef MDDI_SII_DISPLAY_INITIAL
			sii_lcd_driver_init(pdev);
#endif
			sii_lcd_dbc_on(rd);
			sii_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_DISPLAY_OFF:
			sii_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return ret;
}

static int mddi_sii_ic_off_panel_off(struct platform_device *pdev)
{
	int ret = 0;
	struct sii_record *rd;

	rd = get_sii_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			sii_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_ON:
			sii_lcd_display_off();
			sii_lcd_dbc_off(rd);
			sii_lcd_enter_sleep();
			sii_lcd_enter_deepstandby();
			rd->lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_SLEEP:
			sii_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_DISPLAY_OFF:
			sii_lcd_dbc_off(rd);
			sii_lcd_enter_sleep();
			sii_lcd_enter_deepstandby();
			rd->lcd_state = LCD_STATE_SLEEP;
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
	return snprintf(buf, PAGE_SIZE, "R61529 Sii HVGA, drv ver:0x%x\n",
							MDDI_DRIVER_VERSION);
}

/* driver attributes */
static DEVICE_ATTR(display_driver_info, 0444, show_driver_info, NULL);

static void lcd_attribute_register(struct platform_device *pdev)
{
	int ret;

	ret = device_create_file(&pdev->dev, &dev_attr_display_driver_info);
	if (ret != 0)
		dev_err(&pdev->dev, "Failed to register display_driver_version"
						"attributes (%d)\n", ret);
}

static int check_panel_ids(struct sii_record *rd)
{
	int ret = 0;

	mutex_lock(&rd->mddi_mutex);

	ret = mddi_host_register_read(LCD_REG_DRIVER_IC_ID,
					&rd->pid.driver_ic_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0) {
		pr_err("mddi_sii_hvga: Failed to read Display ID\n");
		ret = -ENODEV;
		goto error;
	}
	if ((rd->pid.driver_ic_id & 0xFF) !=
			MDDI_SII_DISPLAY_DRIVER_IC_ID) {
		pr_err("mddi_sii_hvga: Detected a non-Sii display\n");
		ret = -ENODEV;
		goto error;
	}

	ret = mddi_host_register_read(LCD_REG_MODULE_ID,
					&rd->pid.module_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		pr_err("mddi_sii_hvga: Failed to read LCD_REG_MODULE_ID\n");

	ret = mddi_host_register_read(LCD_REG_REVISION_ID,
					&rd->pid.revision_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		pr_err("mddi_sii_hvga: "
				"Failed to read LCD_REG_REVISION_ID\n");

	pr_info("Found display with module ID = 0x%x, "
			"revision ID = 0x%x, driver IC ID = 0x%x, "
			"driver ID = 0x%x\n",
			rd->pid.module_id & 0xFF,
			rd->pid.revision_id & 0xFF,
			rd->pid.driver_ic_id & 0xFF,
			MDDI_DRIVER_VERSION);

error:
	mutex_unlock(&rd->mddi_mutex);
	return ret;
}

static int mddi_sii_lcd_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct sii_record *rd;

	if (!pdev) {
		dev_err(&pdev->dev, "%s: no platform_device\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}
	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}

	rd = kzalloc(sizeof(struct sii_record), GFP_KERNEL);
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
		rd->pdata->panel_data->panel_info.lcd.refx100 = 6500;
		rd->pdata->panel_data->panel_info.lcd.v_back_porch = 8;
		rd->pdata->panel_data->panel_info.lcd.v_front_porch = 8;
		rd->pdata->panel_data->panel_info.lcd.v_pulse_width = 0;
		rd->pdata->panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
		rd->pdata->panel_data->panel_info.lcd.vsync_notifier_period = 0;
		rd->pdata->panel_data->on  = mddi_sii_ic_on_panel_off;
		rd->pdata->panel_data->controller_on_panel_on =
						mddi_sii_ic_on_panel_on;
		rd->pdata->panel_data->off = mddi_sii_ic_off_panel_off;
		rd->pdata->panel_data->window_adjust =
						sii_lcd_window_adjust;
		rd->pdata->panel_data->power_on_panel_at_pan = 0;
		pdev->dev.platform_data = rd->pdata->panel_data;

		/* adds mfd on driver_data */
		msm_fb_add_device(pdev);

		/* Add SYSFS to module */
		lcd_attribute_register(pdev);

		dev_info(&pdev->dev, "%s: Probe success!", __func__);
		ret = 0;
	} else {
		kfree(rd);
	}
exit_point:
	return ret;
}

static int __devexit mddi_sii_lcd_remove(struct platform_device *pdev)
{
	struct sii_record *rd;

	device_remove_file(&pdev->dev, &dev_attr_display_driver_info);
	rd = platform_get_drvdata(pdev);
	kfree(rd);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_sii_lcd_probe,
	.remove = __devexit_p(mddi_sii_lcd_remove),
	.driver = {
		.name = MDDI_SII_R61529_HVGA_NAME,
	},
};

static int __init mddi_sii_lcd_init(void)
{
	return platform_driver_register(&this_driver);
}

static void __exit mddi_sii_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("macro.luo@sonyericsson.com");
MODULE_DESCRIPTION("Driver for Renesas R61529 with SII HVGA panel");

module_init(mddi_sii_lcd_init);
module_exit(mddi_sii_lcd_exit);
