/* drivers/video/msm/mddi_novatek_fwvga.c
 *
 * Copyright (c) 2009-2010 Sony Ericsson Mobile Communications
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <mach/mddi_novatek_fwvga.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include "mddi_nt_panels/mddi_nt_panel.h"
#include "mdp4.h"
#define write_client_reg(__X, __Y) \
  mddi_queue_register_write(__X, __Y, TRUE, 0)

#ifdef CONFIG_FB_MSM_PANEL_ESD
/* ESD check interval */
#define ESD_POLL_TIME_MS 1000
static void esd_recovery_resume(struct platform_device *pdev);
static void esd_recovery_init(struct platform_device *pdev);
#endif
#define MIN_REF_RATE 14285 /* ref100=70Hz */
#define MAX_REF_RATE 18182 /* ref100=55Hz */
/*
 * {STANDBY}-------------resume()
 *     ^                    |
 *     |                    v
 * standby()<------o------init() <---- OFF
 *                 |
 *              {SLEEP}
 *                 |
 * setup()<--------o-----takedown()
 *    |                     ^
 *    |                     |
 *  {       DISPLAY_OFF       }
 *    |                     |
 *    v                     |
 * turn_on()---{NORMAL}->turn_off()
 */

enum novatek_mode {
	NVT_MODE_OFF,
	NVT_MODE_SLEEP,
	NVT_MODE_STANDBY,
	NVT_MODE_DISPLAY_OFF,
	NVT_MODE_NORMAL,
};

static const struct panel_id *novatek_panel_id;
static struct i2c_client *novatek_i2c_client;
static DEFINE_MUTEX(novatek_panel_lock);

struct novatek_record {
	enum novatek_mode mode;
	struct novatek_fwvga_platform_data *pdata;
	const struct panel_id *panel;
	int power;
	struct i2c_client *client;
#ifdef CONFIG_FB_MSM_PANEL_ESD
	struct delayed_work work;
#endif
};

static ssize_t novatek_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mddi_client_type);
}

static struct device_attribute novatek_dev_attr_type = {
	.attr = {
		.name = "type",
		.mode = 0444,
	},
	.show = novatek_type_show,
};

static ssize_t novatek_vsync_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	unsigned long data;
	struct platform_device *pdev = NULL;
	struct novatek_record *rd;
	struct msm_fb_panel_data *panel_data;
	struct msm_panel_info *pinfo;

	pdev = container_of(dev, struct platform_device, dev);
	if (!pdev)
		return -ENOMEM;
	rd = platform_get_drvdata(pdev);
	panel_data = rd->panel->pinfo->get_panel_info();
	pinfo = &panel_data->panel_info;

	ret = strict_strtoul(buf, 10, &data);
	data = (data >> 16) & 0xffff;

	if ((MIN_REF_RATE < data) && (data < MAX_REF_RATE)) {
		pinfo->lcd.refx100 = 100000000 / data;
		mdp_vsync_config_update(pinfo);
	}
	ret = strnlen(buf, size);
	return ret;
}

static struct device_attribute novatek_dev_attr_vsync = {
	.attr = {
		.name = "vsync",
		.mode = 0200,
	},
	.store = novatek_vsync_store,
};

static void hr_msleep(int ms)
{
	struct timespec req_time;
	long ret;

	req_time.tv_sec = ms / 1000;
	req_time.tv_nsec = (ms % 1000) * 1000000;

	ret = hrtimer_nanosleep(&req_time, NULL, HRTIMER_MODE_REL,
							CLOCK_MONOTONIC);
	if (ret != 0)
		printk(KERN_ERR "%s: nanosleep failed, ret = %ld\n", __func__,
									ret);
}

static void novatek_controller_execute(const struct novatek_reg_set *rs)
{
	int n;
#ifndef CONFIG_FB_MSM_MDDI_NOVATEK_DISABLE_MDDI
	int rc;
#endif
#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_DISABLE_MDDI
	u8 i2c_data[4];

	if (!novatek_i2c_client) {
		pr_err("MDDI_NOVATEK: no i2c client!\n");
		return;
	}
#endif
	if (rs == NULL) {
		pr_err(MDDI_NOVATEK_FWVGA_NAME
				": no register set for state!\n");
		return;
	}

	for (n = 0; ; ++n) {
		if (rs[n].reg == 0) {
			if (rs[n].val == 0)
				break;
			hr_msleep(rs[n].val);
		} else {
#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_DISABLE_MDDI
			i2c_data[0] = rs[n].reg >> 8;
			i2c_data[1] = rs[n].reg & 0xFF;
			i2c_data[2] = rs[n].val >> 8;
			i2c_data[3] = rs[n].val & 0xFF;
			i2c_master_send(novatek_i2c_client, i2c_data, 4);
#else
			rc = write_client_reg(rs[n].reg, rs[n].val);
			if (rc != 0)
				return;
#endif
		}
	}
}

static int novatek_i2c_read(struct i2c_client *client, u16 reg,
		u16 *data, int size)
{
	int ret;
	u8	i2c_data[4];

	i2c_data[0] = reg >> 8;
	i2c_data[1] = reg & 0xFF;
	i2c_data[2] = reg >> 8;
	i2c_data[3] = reg & 0xFF;
	i2c_master_send(client, i2c_data, size);
	ret = i2c_master_recv(client, i2c_data, size);

	*data = (i2c_data[0] << 8) | i2c_data[1];

	return ret;
}

static int novatek_power(struct platform_device *pdev, int on)
{
	struct novatek_record *rd;
	int rc = 0;

	rd = platform_get_drvdata(pdev);

	/* msm_fb occasionally gets confused as to whether it wants the panel
	 * on or off. */
	if (on == rd->power)
		return 0;

	if (on) {
		if (rd->pdata->power)
			rc = rd->pdata->power(1);

		if (rc == 0 && rd->pdata->reset)
			rd->pdata->reset();
	} else {
		if (rd->pdata->power)
			rc = rd->pdata->power(0);
	}

	if (rc)
		dev_err(&pdev->dev, "unable to power %s\n", on ? "on" : "off");
	else
		rd->power = on;

	return rc;
}

static int novatek_ic_off_panel_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct novatek_record *rd;

	mfd = platform_get_drvdata(pdev);
	if (mfd == NULL)
		return -ENODEV;
	if (mfd->key != MFD_KEY || mfd->panel_pdev == NULL)
		return -EINVAL;

	rd = platform_get_drvdata(mfd->panel_pdev);

	/* this not needed in lcd_on, since 'mode' never changes */
	if (!rd->panel->suspend_support)
		return 0;

	switch (rd->mode) {
	case NVT_MODE_NORMAL:
		novatek_controller_execute(rd->panel->pinfo->turn_off);
	case NVT_MODE_DISPLAY_OFF:
		novatek_controller_execute(rd->panel->pinfo->takedown);
	case NVT_MODE_SLEEP:
		novatek_controller_execute(rd->panel->pinfo->standby);
		novatek_power(mfd->panel_pdev, 0);
	default:
		break;
	}

#ifdef CONFIG_FB_MSM_PANEL_ESD
	if (rd->panel->esd_support)
		cancel_delayed_work_sync(&rd->work);
#endif

	rd->mode = NVT_MODE_OFF;
	return 0;
}

static int novatek_ic_on_panel_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct novatek_record *rd;

	mfd = platform_get_drvdata(pdev);
	if (mfd == NULL)
		return -ENODEV;
	if (mfd->key != MFD_KEY || mfd->panel_pdev == NULL)
		return -EINVAL;

	rd = platform_get_drvdata(mfd->panel_pdev);

	switch (rd->mode) {
	case NVT_MODE_OFF:
		novatek_power(mfd->panel_pdev, 1);
		novatek_controller_execute(rd->panel->pinfo->init);
	case NVT_MODE_SLEEP:
		novatek_controller_execute(rd->panel->pinfo->setup);
	default:
		break;
	}

	rd->mode = NVT_MODE_DISPLAY_OFF;

	return 0;
}

static int novatek_ic_on_panel_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct novatek_record *rd;

	mfd = platform_get_drvdata(pdev);
	if (mfd == NULL)
		return -ENODEV;
	if (mfd->key != MFD_KEY || mfd->panel_pdev == NULL)
		return -EINVAL;

	rd = platform_get_drvdata(mfd->panel_pdev);

	switch (rd->mode) {
	case NVT_MODE_OFF:
		novatek_power(mfd->panel_pdev, 1);
		novatek_controller_execute(rd->panel->pinfo->init);
	case NVT_MODE_SLEEP:
		novatek_controller_execute(rd->panel->pinfo->setup);
	case NVT_MODE_DISPLAY_OFF:
		novatek_controller_execute(rd->panel->pinfo->turn_on);
	default:
		break;
	}

	rd->mode = NVT_MODE_NORMAL;

#ifdef CONFIG_FB_MSM_PANEL_ESD
	if (rd->panel->esd_support)
		esd_recovery_resume(mfd->panel_pdev);
#endif

	return 0;
}

#ifdef CONFIG_FB_MSM_PANEL_ESD
static int esd_failure_check(struct novatek_record *rd)
{
	u16 id = 0;
	int ret;

	ret = novatek_i2c_read(rd->client, 0x0F00, &id, 4);
	if (ret < 0) {
		printk(KERN_ERR"%s: i2c_read error\n", __func__);
		return -ENODEV;
	}
	id &= 0x01;

	if (id) {
		printk(KERN_INFO "MDDI: display checksum error 0x%02x.\n", id);
		return 1;
	}
	return 0;
}

static void esd_recovery_func(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct novatek_record *rd;
	int ret = 0;
	int timeout = msecs_to_jiffies(ESD_POLL_TIME_MS);

	dwork = container_of(work, struct delayed_work, work);
	rd = container_of(dwork, struct novatek_record, work);

	if (rd->mode == NVT_MODE_NORMAL) {
		ret = esd_failure_check(rd);

		if (ret == 1) {
			printk(KERN_WARNING "%s ESD recovery started.\n",
								__func__);
			novatek_controller_execute(rd->panel->pinfo->init);
			novatek_controller_execute(rd->panel->pinfo->setup);
			novatek_controller_execute(rd->panel->pinfo->turn_on);
			printk(KERN_WARNING "%s ESD recovery finished.\n",
								__func__);
		}

		schedule_delayed_work(&rd->work, timeout);
	}
}

static void esd_recovery_init(struct platform_device *pdev)
{

	struct novatek_record *rd;

	rd = platform_get_drvdata(pdev);
	INIT_DELAYED_WORK(&rd->work, esd_recovery_func);

}

static void esd_recovery_resume(struct platform_device *pdev)
{
	struct novatek_record *rd;

	rd = platform_get_drvdata(pdev);
	schedule_delayed_work(&rd->work, ESD_POLL_TIME_MS);
}
#endif

static int panel_id_reg_check(struct i2c_client *client,
		const struct panel_id *id)
{
	int	ret = 0;
	int	reg = 0;
	u16	data;

	for (reg = 0; reg < id->reg_count; ++reg) {
		ret = novatek_i2c_read(client,
				id->regs[reg].addr, &data, 4);
		if (ret < 0)
			return -ENODEV;

		if (data != id->regs[reg].value)
			return -ENODEV;
	}

	return 0;
}

static const struct i2c_device_id novatek_i2c_idtable[] = {
	{MDDI_NOVATEK_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, novatek_i2c_idtable);

static int __devinit novatek_i2c_probe(struct i2c_client *,
		const struct i2c_device_id *);
static int __devexit novatek_i2c_remove(struct i2c_client *);

static struct i2c_driver novatek_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = MDDI_NOVATEK_I2C_NAME,
	},
	.probe   = novatek_i2c_probe,
	.remove  = __devexit_p(novatek_i2c_remove),
	.id_table = novatek_i2c_idtable,
};

static int __devexit novatek_i2c_remove(struct i2c_client *client)
{
	device_init_wakeup(&client->dev, 0);

	return 0;
}

static int __devinit novatek_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int	n = 0;
	struct novatek_i2c_pdata *pdata;
	int	ret = -ENODEV;

	dev_dbg(&client->dev, "probing\n");

	pdata = client->dev.platform_data;

	client->driver = &novatek_i2c_driver;

	while (pdata->panels[n] != NULL) {
		ret = panel_id_reg_check(client, pdata->panels[n]);
		if (ret == 0)
			break;
		n++;
	}

	novatek_panel_id = pdata->panels[n];

	novatek_i2c_client = client;

	return ret;
}

static int __devexit novatek_controller_remove(struct platform_device *pdev)
{
	struct novatek_record *rd;

	device_remove_file(&pdev->dev, &novatek_dev_attr_vsync);
	rd = platform_get_drvdata(pdev);

	/* there's no msm_fb_remove_device :/ */

	platform_set_drvdata(pdev, NULL);
	kfree(rd);
	return 0;
}

static int __devinit novatek_controller_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_panel_info *pinfo;
	struct msm_fb_panel_data *panel_data;
	struct novatek_fwvga_platform_data *pdata;
	struct novatek_record *rd;

	dev_dbg(&pdev->dev, "probing\n");

	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		return -EINVAL;

	rd = kzalloc(sizeof(struct novatek_record), GFP_KERNEL);
	if (rd == NULL)
		return -ENOMEM;

	rd->pdata = pdata;
	platform_set_drvdata(pdev, rd);

	rc = novatek_power(pdev, 1);
	if (rc)
		goto err_panel_on;

	mutex_lock(&novatek_panel_lock);
	/* This will call the I2C probe, which will detect the display.
	 * If that probe fails we'll receive the error here */
	rc = i2c_add_driver(&novatek_i2c_driver);
	if (rc) {
		dev_err(&pdev->dev, "no panel found\n");
		mutex_unlock(&novatek_panel_lock);
		goto err_i2c_reg_failed;
	}

	if (novatek_panel_id == NULL) {
		/* something went wrong, we shouldn't get here */
		dev_err(&pdev->dev, "no panel\n");
		rc = -ENODEV;
		mutex_unlock(&novatek_panel_lock);
		goto err_no_panel;
	}
	rd->panel = novatek_panel_id;
	mutex_unlock(&novatek_panel_lock);

	dev_info(&pdev->dev, "Found %s\n", rd->panel->name);
	panel_data = rd->panel->pinfo->get_panel_info();
	panel_data->on = novatek_ic_on_panel_off;
	panel_data->controller_on_panel_on = novatek_ic_on_panel_on;
	panel_data->off = novatek_ic_off_panel_off;
	panel_data->power_on_panel_at_pan = 0;

	pinfo = &panel_data->panel_info;
	pinfo->width = rd->panel->width;
	pinfo->height = rd->panel->height;
	pinfo->lcd.rev = rd->panel->mddi_type;

	dev_info(&pdev->dev, "Using MDDI type %d\n", pinfo->lcd.rev);
	if (pinfo->lcd.rev == 2) {
		pinfo->lcd.vsync_enable = TRUE;
		pinfo->lcd.hw_vsync_mode = TRUE;
	} else {
		pinfo->lcd.vsync_enable = FALSE;
		pinfo->lcd.hw_vsync_mode = FALSE;
	}

	/* In order for quickest startup time on boot, we don't power off the
	 * panel because it will be turned on immediately by msm_fb's
	 * bootlogo code */
#ifndef CONFIG_FB_MSM_LOGO
	/* It is important here that this happens before msm_fb_add_device,
	 * since it will indirectly power on the device if it is needed. */
	novatek_power(pdev, 0);
#endif

	pdev->dev.platform_data = panel_data;

	rd->client = novatek_i2c_client;
#ifdef CONFIG_FB_MSM_PANEL_ESD
	if (rd->panel->esd_support)
		esd_recovery_init(pdev);
#endif
	msm_fb_add_device(pdev);

	rc = device_create_file(&pdev->dev, &novatek_dev_attr_type);
	if (rc)
		dev_err(&pdev->dev, "create dev_attr_type failed.\n");
		/* non-fatal */
	rc = device_create_file(&pdev->dev, &novatek_dev_attr_vsync);
	if (rc)
		dev_err(&pdev->dev, "create dev_attr_type failed.\n");
	return 0;

err_no_panel:
err_i2c_reg_failed:
	novatek_power(pdev, 0);
err_panel_on:
	platform_set_drvdata(pdev, NULL);
	kfree(rd);
	return rc;
}

static struct platform_driver novatek_fwvga_driver = {
	.probe	= novatek_controller_probe,
	.remove = novatek_controller_remove,
	.driver = {
		.name	= MDDI_NOVATEK_FWVGA_NAME,
	},
};

static int __init novatek_init(void)
{
	return platform_driver_register(&novatek_fwvga_driver);
}

static void __exit novatek_exit(void)
{
	platform_driver_unregister(&novatek_fwvga_driver);
}

module_init(novatek_init);
module_exit(novatek_exit);

