/*
 * leds-low-current-pmic.c - MSM LOW CURRENT PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <mach/msm_rpcrouter.h>

#ifdef  DBG_MSG
#define SEDBG(fmt, args...) printk(KERN_INFO "leds low current LED: " fmt, ##args)
#else
#define SEDBG(fmt, args...)
#endif //SEDBG

#define PM_LIBPROG 0x30000061
#define PM_LIBVERS 0x00030001
#define PM_LOW_CURRENT_LED_SET_CURRENT_PROC 72
#define MAX_LOW_CURRENT_SINK_VOLTAGE  30;

enum rpc_pm_low_current_led_type {
  RPC_PM_LOW_CURRENT_LED_DRV0,
  RPC_PM_LOW_CURRENT_LED_DRV1,
  RPC_PM_LOW_CURRENT_LED_DRV2,
  RPC_PM_LOW_CURRENT_LED_DRV_INVALID
};

static void low_current_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret = 0;
	int curr = 0;

	static struct msm_rpc_endpoint *led_endpoint;
	struct pm_low_current_led_set_args
	{
		struct rpc_request_hdr hdr;
		uint led_type;
		uint curr;
	} req;

	if (!led_endpoint)
	{
		led_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(led_endpoint))
		{
			printk(KERN_ERR "init led rpc failed!\n");
			led_endpoint = 0;
			return;
		}
	}

	SEDBG("set low current LED %d\n", value);

	req.led_type = RPC_PM_LOW_CURRENT_LED_DRV0;

	if (value)
	{
		curr = value * MAX_LOW_CURRENT_SINK_VOLTAGE;
		curr /= LED_FULL;
		req.curr = cpu_to_be32(curr);
	}
	else
	{
		req.curr = cpu_to_be32(0);
	}

	msm_rpc_call(led_endpoint, PM_LOW_CURRENT_LED_SET_CURRENT_PROC, &req, sizeof(req), 5* HZ);

	if (ret)
		dev_err(led_cdev->dev, "can't set low current LED\n");
}

static struct led_classdev low_current_led = {
	.name			= "low-current-led",
	.brightness_set	= low_current_led_set,
	.brightness		= LED_OFF,
};

static int low_current_led_probe(struct platform_device *pdev)
{
	int rc;
	SEDBG("low_current_led_probe\n");
	rc = led_classdev_register(&pdev->dev, &low_current_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register low current led class driver\n");
		return rc;
	}
	low_current_led_set(&low_current_led, LED_OFF);
	return rc;
}

static int __devexit low_current_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&low_current_led);

	return 0;
}

static struct platform_driver low_current_led_driver = {
	.probe		= low_current_led_probe,
	.remove		= __devexit_p(low_current_led_remove),
	.driver		= {
		.name	= "pmic-low-current-leds",
		.owner	= THIS_MODULE,
	},
};


static int __init low_current_led_init(void)
{
	SEDBG("low_current_led_init\n");
	return platform_driver_register(&low_current_led_driver);
}

static void __exit low_current_led_exit(void)
{
	SEDBG("low_current_led_exit\n");
	platform_driver_unregister(&low_current_led_driver);
}


module_init(low_current_led_init);
module_exit(low_current_led_exit);

MODULE_DESCRIPTION("PMIC Low Current LED driver");
MODULE_LICENSE("GPL v2");
