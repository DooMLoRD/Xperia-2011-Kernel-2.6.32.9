/* drivers/video/msm/mddi_auo_s6d05a1_hvga.c
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
#include <linux/mddi_auo_s6d05a1_hvga.h>
#include <asm/byteorder.h>

/* Internal version number */
#define MDDI_DRIVER_VERSION 0x0003

/* Driver_IC_ID value for display */
#define MDDI_AUO_DISPLAY_DRIVER_IC_ID 0xF0

/* Byte order,  word0=P4P3P2P1, little endian */
#define write_client_reg_nbr(__X, __Y0, __Y1, __Y2, __Y3, __NBR) \
  mddi_host_register_write16(__X, __Y0, __Y1, __Y2, __Y3, __NBR, \
			TRUE, NULL, MDDI_HOST_PRIM);

#define write_client_reg_xl(__X, __Y, __NBR) \
  mddi_host_register_write_xl(__X, __Y, __NBR, \
			TRUE, NULL, MDDI_HOST_PRIM);

enum lcd_registers {
	LCD_REG_COLUMN_ADDRESS = 0x2a,
	LCD_REG_PAGE_ADDRESS = 0x2b,
	LCD_REG_DRIVER_IC_ID = 0xA1,
	LCD_REG_MODULE_ID = 0xDB,
	LCD_REG_REVISION_ID = 0xDC
};

enum mddi_auo_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_POWER_ON,
	LCD_STATE_DISPLAY_OFF,
	LCD_STATE_ON,
	LCD_STATE_SLEEP
};

struct panel_ids {
	u32 driver_ic_id;
	u32 module_id;
	u32 revision_id;
};

struct auo_record {
	struct auo_hvga_platform_data *pdata;
	struct mutex mddi_mutex;
	int power_ctrl;
	enum mddi_auo_lcd_state lcd_state;
	struct platform_device *pdev;
	struct panel_ids pid;
};

static void auo_lcd_dbc_on(struct auo_record *rd)
{
	if (rd->pdata->dbc_on) {
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

static void auo_lcd_dbc_off(struct auo_record *rd)
{
	if (rd->pdata->dbc_on) {
		/* Mobile Image Enhancement Mode */
		write_client_reg_nbr(0x55, 0x00000000, 0, 0, 0, 1);

		/* BL Control */
		write_client_reg_nbr(0x53, 0x00000000, 0, 0, 0, 1);
	}
}

static void auo_lcd_window_address_set(enum lcd_registers reg,
						u16 start, u16 stop)
{
	uint32 para;

	para = start;
	para = (para << 16) | (start + stop);
#ifdef __LITTLE_ENDIAN
	para = swab32(para);
#endif
	write_client_reg_nbr(reg, para, 0, 0, 0, 1);
}

static uint32 reg_disctl[5] = {0x08034E3B, 0x00080808, 0x00000808,
			0x08540000, 0x00080808};

static void auo_lcd_driver_init(void)
{
	/* Page Address Set */
	write_client_reg_nbr(0x2A, 0x0000013F, 0, 0, 0, 1);
	write_client_reg_nbr(0x2B, 0x000001DF, 0, 0, 0, 1);

	/* PASSWD1 */
	write_client_reg_nbr(0xF0, 0x00005A5A, 0, 0, 0, 1);

	/* PASSWD2 */
	write_client_reg_nbr(0xF1, 0x00005A5A, 0, 0, 0, 1);

	/* DISCTL */
	write_client_reg_xl(0xF2, reg_disctl, 5);

	/* PWRCTL */
	write_client_reg_nbr(0xF4, 0x00000008, 0x00000000,
			0x00037900, 0x0000379, 4);

	/* VCMCTL */
	write_client_reg_nbr(0xF5, 0x00755D00, 0x00000300,
			0x755D0004, 0, 3);

	/* SRGCTL */
	write_client_reg_nbr(0xF6, 0x03080004, 0x00010001,
			0x00000000, 0, 2);

	/* IFCTL */
	write_client_reg_nbr(0xF7, 0x02108048, 0x00000000,
			0, 0, 2);

	/* PANELCTL */
	write_client_reg_nbr(0xF8, 0x00000011, 0, 0, 0, 1);

	/* WRDISBV */
	write_client_reg_nbr(0x51, 0x000000FF, 0, 0, 0, 1);

	/* WRCTRLD */
	write_client_reg_nbr(0x53, 0x0000002C, 0, 0, 0, 1);

	/* WRCABC */
	write_client_reg_nbr(0x55, 0x00000003, 0, 0, 0, 1);

	/* WRCABCMB */
	write_client_reg_nbr(0x5E, 0x00000000, 0, 0, 0, 1);

	/* MIECTRL */
	write_client_reg_nbr(0xC0, 0x003F8080, 0, 0, 0, 1);

	/* BCMODE */
	write_client_reg_nbr(0xC1, 0x00000013, 0, 0, 0, 1);

	/* GAMMSEL */
	write_client_reg_nbr(0xF9, 0x00000024, 0, 0, 0, 1);

	/* PGAMMACTL */
	write_client_reg_nbr(0xFA, 0x1B040B0B, 0x291C1A19,
			0x3F3F3C36, 0x00000011, 4);

	/* GAMMSEL */
	write_client_reg_nbr(0xF9, 0x00000022, 0, 0, 0, 1);

	/* PGAMMACTL */
	write_client_reg_nbr(0xFA, 0x1B050B0B, 0x25211F1D,
			0x3F3F3B34, 0x00000000, 4);

	/* GAMMSEL */
	write_client_reg_nbr(0xF9, 0x00000021, 0, 0, 0, 1);

	/* PGAMMACTL */
	write_client_reg_nbr(0xFA, 0x3B000B0B, 0x1F2B3038,
			0x373A382F, 0x0000004, 4);

	/* COLMOD*/
	write_client_reg_nbr(0x3A, 0x00000077, 0, 0, 0, 1);

	/* MADCTL */
	write_client_reg_nbr(0x36, 0x00000000, 0, 0, 0, 1);

	/* TEON */
	write_client_reg_nbr(0x35, 0x00000000, 0, 0, 0, 1);

	/* PASET */
	write_client_reg_nbr(0x2B, 0xDF010000, 0, 0, 0, 1);

	/* CASET */
	write_client_reg_nbr(0x2A, 0x3F010000, 0, 0, 0, 1);
}

static void auo_lcd_window_adjust(uint16 x1, uint16 x2, uint16 y1, uint16 y2)
{
	auo_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);
	auo_lcd_window_address_set(LCD_REG_PAGE_ADDRESS, y1, y2);
	/* Workaround: 0x3Ch at start of column bug */
	write_client_reg_nbr(0x3C, 0, 0, 0, 0, 1);
}

static void auo_lcd_enter_sleep(void)
{
	/* Sleep in */
	write_client_reg_nbr(0x10, 0, 0, 0, 0, 1);
	mddi_wait(120); /* >120 ms */
}

static void auo_lcd_display_off(void)
{
	/* Display off */
	write_client_reg_nbr(0x28, 0, 0, 0, 0, 1);
	mddi_wait(50); /* >50 ms */
}


static void auo_lcd_exit_sleep(void)
{
	/* Sleep out */
	write_client_reg_nbr(0x11, 0x00000000, 0, 0, 0, 1);
	mddi_wait(120); /* >120 ms */
}

static void auo_lcd_display_on(void)
{
	/* Display on */
	write_client_reg_nbr(0x29, 0x00000000, 0, 0, 0, 1);
}


static void auo_lcd_enter_deepstandby(void)
{
	/* PASSWD2 */
	write_client_reg_nbr(0xF1, 0x00005A5A, 0, 0, 0, 1);

	/* DSTB */
	write_client_reg_nbr(0xDF, 0x00000001, 0, 0, 0, 1);
}

static void auo_toggle_reset(struct auo_record *rd)
{
	if (rd->pdata->exit_deep_standby)
		rd->pdata->exit_deep_standby();
}

static void auo_lcd_exit_deepstandby(struct auo_record *rd)
{
	auo_toggle_reset(rd);
}

static void auo_power_on(struct auo_record *rd)
{
	if (rd->pdata->power_on)
		rd->pdata->power_on();
}

static void auo_power_off(struct auo_record *rd)
{
	if (rd->pdata->power_off)
		rd->pdata->power_off();
}

static struct auo_record *get_auo_record_from_mfd(
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

static int mddi_auo_ic_on_panel_off(struct platform_device *pdev)
{

	int ret = 0;
	struct auo_record *rd;

	rd = get_auo_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_OFF:
			auo_power_on(rd);
			rd->lcd_state = LCD_STATE_POWER_ON;
			break;

		case LCD_STATE_POWER_ON:
			auo_lcd_driver_init();
			auo_lcd_exit_sleep();
			auo_lcd_dbc_on(rd);
			rd->lcd_state = LCD_STATE_DISPLAY_OFF;
			break;

		case LCD_STATE_SLEEP:
			auo_lcd_exit_deepstandby(rd);
			auo_lcd_driver_init();
			auo_lcd_exit_sleep();
			auo_lcd_dbc_on(rd);
			rd->lcd_state = LCD_STATE_DISPLAY_OFF;
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return 0;
}

static int mddi_auo_ic_on_panel_on(struct platform_device *pdev)
{
	int ret = 0;
	struct auo_record *rd;

	rd = get_auo_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			auo_lcd_driver_init();
			auo_lcd_exit_sleep();
			auo_lcd_dbc_on(rd);
			auo_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_SLEEP:
			auo_lcd_exit_deepstandby(rd);
			auo_lcd_driver_init();
			auo_lcd_exit_sleep();
			auo_lcd_dbc_on(rd);
			auo_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_DISPLAY_OFF:
			auo_lcd_display_on();
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
	return 0;
}


static int mddi_auo_ic_off_panel_off(struct platform_device *pdev)
{
	int ret = 0;
	struct auo_record *rd;

	rd = get_auo_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			auo_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_ON:
			auo_lcd_display_off();
			auo_lcd_dbc_off(rd);
			auo_lcd_enter_sleep();
			auo_lcd_enter_deepstandby();
			rd->lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_SLEEP:
			auo_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_DISPLAY_OFF:
			auo_lcd_dbc_off(rd);
			auo_lcd_enter_sleep();
			auo_lcd_enter_deepstandby();
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
	return snprintf(buf, PAGE_SIZE, "s6d5a1 AUO HVGA, drv ver:0x%x\n",
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

static int check_panel_ids(struct auo_record *rd)
{
	int ret = 0;

	mutex_lock(&rd->mddi_mutex);

	ret = mddi_host_register_read(LCD_REG_DRIVER_IC_ID,
					&rd->pid.driver_ic_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0) {
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
			"Failed to read Display ID\n", __func__);
		ret = -ENODEV;
		goto error;
	}
	if ((rd->pid.driver_ic_id & 0xFF) !=
			MDDI_AUO_DISPLAY_DRIVER_IC_ID) {
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
			"Detected a non-AUO display\n", __func__);
		ret = -ENODEV;
		goto error;
	}

	ret = mddi_host_register_read(LCD_REG_MODULE_ID,
					&rd->pid.module_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
			"Failed to read LCD_REG_MODULE_ID\n", __func__);

	ret = mddi_host_register_read(LCD_REG_REVISION_ID,
					&rd->pid.revision_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
			"Failed to read LCD_REG_REVISION_ID\n", __func__);

	dev_info(&rd->pdev->dev, "Found display with module ID = 0x%x, "
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

static int mddi_auo_lcd_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct auo_record *rd;

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}

	rd = kzalloc(sizeof(struct auo_record), GFP_KERNEL);
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
		rd->power_ctrl = 1;

		rd->pdata->panel_data->panel_info.mddi.vdopkt =
						MDDI_DEFAULT_PRIM_PIX_ATTR;
		rd->pdata->panel_data->panel_info.lcd.vsync_enable = TRUE;
		rd->pdata->panel_data->panel_info.lcd.refx100 = 6500;
		rd->pdata->panel_data->panel_info.lcd.v_back_porch = 8;
		rd->pdata->panel_data->panel_info.lcd.v_front_porch = 8;
		rd->pdata->panel_data->panel_info.lcd.v_pulse_width = 0;
		rd->pdata->panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
		rd->pdata->panel_data->panel_info.lcd.vsync_notifier_period = 0;
		rd->pdata->panel_data->on  = mddi_auo_ic_on_panel_off;
		rd->pdata->panel_data->controller_on_panel_on =
						mddi_auo_ic_on_panel_on;
		rd->pdata->panel_data->off = mddi_auo_ic_off_panel_off;
		rd->pdata->panel_data->window_adjust =
						auo_lcd_window_adjust;
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

static int __devexit mddi_auo_lcd_remove(struct platform_device *pdev)
{
	struct auo_record *rd;

	device_remove_file(&pdev->dev, &dev_attr_display_driver_info);
	rd = platform_get_drvdata(pdev);
	kfree(rd);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_auo_lcd_probe,
	.remove = __devexit_p(mddi_auo_lcd_remove),
	.driver = {
		.name   = MDDI_AUO_S6D05A1_HVGA_NAME,
	},
};

static int __init mddi_auo_lcd_init(void)
{
	return platform_driver_register(&this_driver);
}

static void __exit mddi_auo_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("macro.luo@sonyericsson.com");
MODULE_DESCRIPTION("MDDI implementation of the auo HVGA display");

module_init(mddi_auo_lcd_init);
module_exit(mddi_auo_lcd_exit);
