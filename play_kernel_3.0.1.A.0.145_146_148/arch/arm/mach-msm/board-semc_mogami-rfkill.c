/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

static struct rfkill *bt_rfk;
static const char bt_name[] = "wl127x";

struct rfkill_gpio_data {
	int gpio;
};

static int bluetooth_set_power(void *data, bool blocked)
{
	int ret = -EINVAL;
	int (*power_control)(int enable);

	if (data) {
		power_control = data;
		ret = (power_control)(!blocked);
	}
	return ret;
}

static struct rfkill_ops mogami_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int mogami_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */

	rc = bluetooth_set_power(pdev->dev.platform_data, default_state);
	if (rc)
		goto err_rfkill;

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&mogami_rfkill_ops, pdev->dev.platform_data);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	platform_set_drvdata(pdev, bt_rfk);
	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill:
	return rc;
}

static int mogami_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill_gpio_data *gdata = platform_get_drvdata(pdev);

	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	gpio_free(gdata->gpio);
	kfree(gdata);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver mogami_rfkill_driver = {
	.probe = mogami_rfkill_probe,
	.remove = mogami_rfkill_remove,
	.driver = {
		.name = "mogami-rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init mogami_rfkill_init(void)
{
	return platform_driver_register(&mogami_rfkill_driver);
}

static void __exit mogami_rfkill_exit(void)
{
	platform_driver_unregister(&mogami_rfkill_driver);
}

module_init(mogami_rfkill_init);
module_exit(mogami_rfkill_exit);
MODULE_DESCRIPTION("mogami rfkill");
MODULE_AUTHOR("Vitaly Wool <vitaly.wool@sonyericsson.com>");
MODULE_LICENSE("GPL");
