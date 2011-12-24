/* kernel/drivers/video/msm/hdmi_sii9024a.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Suzuki Yusaku <Yusaku.Suzuki@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * Based on drivers/video/msm/hdmi_sii9022.c
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/switch.h>

#include <linux/i2c/sii9024.h>


#include "msm_fb.h"

#define DEVICE_NAME "sii9024a"
#define SII9024A_DEVICE_ID   0xB0

struct msm_panel_info *pinfo;

/* HDMI Power control using kobject */
static struct hdmi_sii_state *hdmi_sii_state_obj;

static int (*setchippower)(int);


static ssize_t hdmi_sii_attr_show(struct kobject *kobj, struct attribute *attr,
								char *buf)
{
	return sprintf(buf, "HPD=%d, POWER=%d, RESET=%d\n",
			hdmi_sii_state_obj->hpd,
			hdmi_sii_state_obj->power,
			hdmi_sii_state_obj->reset);
}

static ssize_t hdmi_sii_attr_store(struct kobject *kobj, struct attribute *attr,
						const char *buf, size_t len)
{
	char hpd_on[]     = "HPD_ON";
	char hpd_off[]    = "HPD_OFF";
	char power_on[]   = "POWER_ON";
	char power_off[]  = "POWER_OFF";
	char reset_high[] = "RESET_HIGH";
	char reset_low[]  = "RESET_LOW";

	char pwr_on[]     = "CHIP_ON";
	char pwr_off[]    = "CHIP_OFF";

	static const char hdmi_active[] = "HDMI_ACTIVE";
	static const char hdmi_inactive[] = "HDMI_INACTIVE";


	if (!(strncmp(buf, hpd_on, strlen(hpd_on)))) {
		gpio_set_value(HDMI_SII_GPIO_HPD, 1);
		hdmi_sii_state_obj->hpd = 1;
	} else if (!(strncmp(buf, hpd_off, strlen(hpd_off)))) {
		gpio_set_value(HDMI_SII_GPIO_HPD, 0);
		hdmi_sii_state_obj->hpd = 0;
	} else if (!(strncmp(buf, power_on, strlen(power_on)))) {
		gpio_set_value(HDMI_SII_GPIO_POWER, 1);
		hdmi_sii_state_obj->power = 1;
	} else if (!(strncmp(buf, power_off, strlen(power_off)))) {
		gpio_set_value(HDMI_SII_GPIO_POWER, 0);
		hdmi_sii_state_obj->power = 0;
	} else if (!(strncmp(buf, reset_high, strlen(reset_high)))) {
		gpio_set_value(HDMI_SII_GPIO_RESET, 1);
		hdmi_sii_state_obj->reset = 1;
	} else if (!(strncmp(buf, reset_low, strlen(reset_low)))) {
		gpio_set_value(HDMI_SII_GPIO_RESET, 0);
		hdmi_sii_state_obj->reset = 0;
	} else if (!(strncmp(buf, pwr_on, strlen(pwr_on)))) {
		if (setchippower != NULL)
			setchippower(1);
	} else if (!(strncmp(buf, pwr_off, strlen(pwr_off)))) {
		if (setchippower != NULL)
			setchippower(0);
	} else if (!(strncmp(buf, hdmi_active, sizeof(hdmi_active)))) {
		switch_set_state(&hdmi_sii_state_obj->pdata->hdmi_switch, 1);
	} else if (!(strncmp(buf, hdmi_inactive, sizeof(hdmi_inactive)))) {
		switch_set_state(&hdmi_sii_state_obj->pdata->hdmi_switch, 0);
	} else
		return -EINVAL;

	return len;
}

static struct sysfs_ops hdmi_sii_sysfs_ops = {
	.show = hdmi_sii_attr_show,
	.store = hdmi_sii_attr_store,
};

static void hdmi_sii_release(struct kobject *kobj)
{
	kset_unregister(hdmi_sii_state_obj->kobj.kset);

	kobject_put(&hdmi_sii_state_obj->kobj);

	kfree(hdmi_sii_state_obj);
}

static struct kobj_attribute hdmi_sii_attr =
	__ATTR(hdmi_sii_state_obj, 0666, NULL, NULL);

static struct attribute *hdmi_sii_attrs[] = {
	&hdmi_sii_attr.attr,
	NULL,
};

static struct kobj_type hdmi_sii_ktype = {
	.sysfs_ops = &hdmi_sii_sysfs_ops,
	.release = hdmi_sii_release,
	.default_attrs = hdmi_sii_attrs,
};

/* create HDMI kobject and initialize */
static int create_hdmi_sii_state_kobj(struct sii9024_platform_data *pdata)
{
	int ret;

	hdmi_sii_state_obj = kzalloc(sizeof(struct hdmi_sii_state), GFP_KERNEL);
	if (!hdmi_sii_state_obj)
		return -ENOMEM;

	hdmi_sii_state_obj->kobj.kset = kset_create_and_add("hdmi_sii_kset",
							NULL, kernel_kobj);

	ret = kobject_init_and_add(&hdmi_sii_state_obj->kobj, &hdmi_sii_ktype,
					NULL, "hdmi_sii_kobj");
	if (ret) {
		kobject_put(&hdmi_sii_state_obj->kobj);

		kset_unregister(hdmi_sii_state_obj->kobj.kset);

		kfree(hdmi_sii_state_obj);

		return -EPERM;
	}

	hdmi_sii_state_obj->hpd   = 0;
	hdmi_sii_state_obj->power = 0;
	hdmi_sii_state_obj->reset = 0;
	hdmi_sii_state_obj->pdata = pdata;

	return 0;
}

static int hdmi_sii_power_on(struct platform_device *pdev)
{
	return 0;
}

static int hdmi_sii_power_off(struct platform_device *pdev)
{
	return 0;
}

static int hdmi_sii_enable(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id hmdi_sii_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

static struct msm_fb_panel_data hdmi_panel_data = {
	.on  = hdmi_sii_power_on,
	.off = hdmi_sii_power_off,
};

static struct platform_device hdmi_device = {
	.name = DEVICE_NAME ,
	.id   = 2,
	.dev  = {
		.platform_data = &hdmi_panel_data,
	}
};

static int hdmi_sii_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;
	struct sii9024_platform_data *pdata;
	pdata = client->dev.platform_data;
	pinfo = &hdmi_panel_data.panel_info;

	pinfo->xres               = pdata->xres;
	pinfo->yres               = pdata->yres;
	pinfo->type               = pdata->type;
	pinfo->pdest              = pdata->pdest;
	pinfo->wait_cycle         = pdata->wait_cycle;
	pinfo->bpp                = pdata->bpp;
	pinfo->fb_num             = pdata->fb_num;
	pinfo->clk_rate           = pdata->clk_rate;
	pinfo->lcdc.h_back_porch  = pdata->lcdc_h_back_porch;
	pinfo->lcdc.h_front_porch = pdata->lcdc_h_front_porch;
	pinfo->lcdc.h_pulse_width = pdata->lcdc_h_pulse_width;
	pinfo->lcdc.v_back_porch  = pdata->lcdc_v_back_porch;
	pinfo->lcdc.v_front_porch = pdata->lcdc_v_front_porch;
	pinfo->lcdc.v_pulse_width = pdata->lcdc_v_pulse_width;
	pinfo->lcdc.border_clr    = pdata->lcdc_border_clr;
	pinfo->lcdc.underflow_clr = pdata->lcdc_underflow_clr;
	pinfo->lcdc.hsync_skew    = pdata->lcdc_hsync_skew;

	setchippower = pdata->setchippower;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	rc = hdmi_sii_enable(client);

	if (create_hdmi_sii_state_kobj(pdata))
		return -ENOMEM;

	pdata->hdmi_switch.name = "hdmi";
	switch_dev_register(&pdata->hdmi_switch);

	msm_fb_add_device(&hdmi_device);

	return rc;
}

static int __devexit hdmi_sii_remove(struct i2c_client *client)
{
	int err = 0 ;
	if (!client->adapter) {
		printk(KERN_ERR "<%s> No HDMI Device\n",
			__func__);
		return -ENODEV;
	}
	i2c_unregister_device(client);
	return err ;
}

static struct i2c_driver hdmi_sii_i2c_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = hdmi_sii_probe,
	.remove =  __exit_p(hdmi_sii_remove),
	.id_table = hmdi_sii_id,
};

static int __init hdmi_sii_init(void)
{
	int ret;

	if (msm_fb_detect_client("sii9024a"))
		return 0;

	ret = i2c_add_driver(&hdmi_sii_i2c_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add i2c driver\n", __func__);

	return ret;
}

static void __exit hdmi_sii_exit(void)
{
	i2c_del_driver(&hdmi_sii_i2c_driver);
}

module_init(hdmi_sii_init);
module_exit(hdmi_sii_exit);
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
MODULE_AUTHOR("Sony Ericsson Mobile Communications, Inc.");
MODULE_DESCRIPTION("SiI9024A HDMI driver");
MODULE_ALIAS("platform:hdmi-sii9024a");
