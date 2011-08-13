/* kernel/drivers/misc/tuner_pm/oneseg_tunerpm_driver.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Tetsuya Matsumoto <tetsuya.x.matsumoto@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/pm.h>
#include <linux/gpio.h>

#include <linux/oneseg_tunerpm.h>

/* delay time after tuner power on (msec) */
#define D_ONESEG_DEVICE_RST_WAITTIME		(3)
/* delay time (1st) after tuner HW reset (usec) */
#define D_ONESEG_DEVICE_RST_DELAY1			(4)
/* delay time (2nd) after tuner HW reset (usec) */
#define D_ONESEG_DEVICE_RST_DELAY2			(28)

struct oneseg_tunerpm_drvdata {
	struct platform_device *pdev;
	spinlock_t delaylock;
};

/**
 * oneseg_tunerpm_dev_init() - Device initialization.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 *   -EIO - Fail of initialization
 */
static int oneseg_tunerpm_dev_init(
	struct oneseg_tunerpm_drvdata *drvdata
)
{
	struct oneseg_tunerpm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;
	int ret;

	spin_lock_init(&drvdata->delaylock);

	ret = gpio_request(pfdata->gpio_rst, "Oneseg tuner HW reset");
	if (ret) {
		dev_err(&drvdata->pdev->dev, "RST request %d\n", ret);
		goto err_request_gpio_rst_req;
	}

	ret = gpio_direction_output(pfdata->gpio_rst, 0);
	if (ret) {
		dev_err(&drvdata->pdev->dev, "RST status %d\n", ret);
		goto err_request_gpio_rst;
	}

	ret = gpio_request(pfdata->gpio_pwr, "Oneseg tuner power");
	if (ret) {
		dev_err(&drvdata->pdev->dev, "PWR request %d\n", ret);
		goto err_request_gpio_pwr_req;
	}

	ret = gpio_direction_output(pfdata->gpio_pwr, 0);
	if (ret) {
		dev_err(&drvdata->pdev->dev, "PWR status %d\n", ret);
		goto err_request_gpio_pwr;
	}

	return ret;

err_request_gpio_pwr:
	gpio_free(pfdata->gpio_pwr);
err_request_gpio_pwr_req:
err_request_gpio_rst:
	gpio_free(pfdata->gpio_rst);
err_request_gpio_rst_req:

	return -EIO;
}

/**
 * oneseg_tunerpm_dev_finalize() - Device finalization.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int oneseg_tunerpm_dev_finalize(
	struct oneseg_tunerpm_drvdata *drvdata
)
{
	struct oneseg_tunerpm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;

	gpio_free(pfdata->gpio_pwr);
	gpio_free(pfdata->gpio_rst);

	return 0;
}

/**
 * oneseg_tunerpm_dev_gpio_put() - put value to GPIO.
 * @gpio:	[IN]    number of gpio
 * @data:	[IN]    put value
 */
static void oneseg_tunerpm_dev_gpio_put(
	unsigned gpio,
	int      data
)
{
	gpio_set_value(gpio, data);
}

/**
 * oneseg_tunerpm_dev_tuner_power_on() - power on oneseg tuner device.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int oneseg_tunerpm_dev_tuner_power_on(
	struct oneseg_tunerpm_drvdata *drvdata
)
{
	struct oneseg_tunerpm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;
	unsigned long flag;

	oneseg_tunerpm_dev_gpio_put(pfdata->gpio_pwr, 1);
	msleep(D_ONESEG_DEVICE_RST_WAITTIME);
	spin_lock_irqsave(&drvdata->delaylock, flag);
	oneseg_tunerpm_dev_gpio_put(pfdata->gpio_rst, 1);
	udelay(D_ONESEG_DEVICE_RST_DELAY1);
	spin_unlock_irqrestore(&drvdata->delaylock, flag);
	udelay(D_ONESEG_DEVICE_RST_DELAY2);

	dev_info(&drvdata->pdev->dev, "PowerOn\n");

	return 0;
}

/**
 * oneseg_tunerpm_dev_tuner_power_off() - power off oneseg tuner device.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int oneseg_tunerpm_dev_tuner_power_off(
	struct oneseg_tunerpm_drvdata *drvdata
)
{
	struct oneseg_tunerpm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;

	oneseg_tunerpm_dev_gpio_put(pfdata->gpio_rst, 0);
	oneseg_tunerpm_dev_gpio_put(pfdata->gpio_pwr, 0);

	dev_info(&drvdata->pdev->dev, "PowerOff\n");

	return 0;
}

/**
 * oneseg_tunerpm_driver_powerctrl_store() - The ioctl handler of power off
 * @dev:	[IN]	device data
 * @attr:	[IN]	device attribute data
 * @buf:	[IN]	data from user side.
 * @count:	[IN]	amount of data from user side.
 *
 * Return codes
 *   0 - Success
 *   -EINVAL - Failure
 */
static ssize_t oneseg_tunerpm_driver_powerctrl_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count
)
{
	struct oneseg_tunerpm_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long value;

	if (strict_strtoul(buf, 0, &value)) {
		dev_err(&drvdata->pdev->dev, "Invalid value for power_ctrl\n");
		goto err_strict_strtoul;
	}

	if (value == D_ONESEG_TUNER_POWER_ON)
		oneseg_tunerpm_dev_tuner_power_on(drvdata);
	else if (value == D_ONESEG_TUNER_POWER_OFF)
		oneseg_tunerpm_dev_tuner_power_off(drvdata);

	return count;

err_strict_strtoul:
	return 0;
}

static DEVICE_ATTR(power_ctrl, S_IWUSR | S_IRUSR,
			NULL, oneseg_tunerpm_driver_powerctrl_store);

/**
 * oneseg_tunerpm_probe() - Prove oneseg tunerpm driver.
 * @pdev:	[IN]	platform device data
 *
 * Return codes
 *   0 - Success
 *   -ENODEV - fail to probe
 *   -EINVAL - no platform data
 *   -ENOMEM - no memory for driver data
 */
static int oneseg_tunerpm_probe(struct platform_device *pdev)
{
	int	ret = -ENODEV;
	struct oneseg_tunerpm_platform_data *pfdata;
	struct oneseg_tunerpm_drvdata *drvdata;

	pfdata = pdev->dev.platform_data;

	if (!pfdata) {
		dev_err(&pdev->dev, "No platform data.\n");
		ret = -EINVAL;
		goto err_get_platform_data;
	}

	drvdata = kzalloc(sizeof(struct oneseg_tunerpm_drvdata), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "No enough memory for oneseg_tunerpm\n");
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	ret = oneseg_tunerpm_dev_init(drvdata);
	if (ret) {
		dev_err(&pdev->dev, "Fail to initialize\n");
		goto err_gpio_init;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_power_ctrl);
	if (ret) {
		dev_err(&pdev->dev, "Fail to initialize\n");
		goto err_device_create_file;
	}

	return 0;

err_device_create_file:
	oneseg_tunerpm_dev_finalize(drvdata);
err_gpio_init:
	kfree(drvdata);
err_alloc_data:
err_get_platform_data:

	return ret;
}

/**
 * oneseg_tunerpm_remove() - Remove oneseg tunerpm driver.
 * @pdev:	[IN]	platform device data
 */
static int __devexit oneseg_tunerpm_remove(struct platform_device *pdev)
{
	struct oneseg_tunerpm_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	device_remove_file(&pdev->dev, &dev_attr_power_ctrl);
	oneseg_tunerpm_dev_finalize(drvdata);
	kfree(drvdata);

	return 0;
}

#ifdef CONFIG_SUSPEND
/**
 * oneseg_tunerpm_suspend() - Suspend oneseg tunerpm driver.
 * @pdev:	[IN]	platform device data
 */
static int oneseg_tunerpm_suspend(struct device *dev)
{
	return 0;
}

/**
 * oneseg_tunerpm_resume() - Resume oneseg tunerpm driver.
 * @pdev:	[IN]	platform device data
 */
static int oneseg_tunerpm_resume(struct device *dev)
{
	return 0;
}
#else
#define oneseg_tunerpm_suspend	NULL
#define oneseg_tunerpm_resume	NULL
#endif

static const struct dev_pm_ops oneseg_tunerpm_pm_ops = {
	.suspend	= oneseg_tunerpm_suspend,
	.resume		= oneseg_tunerpm_resume,
};

/**
 * brief Platform driver data structure of FeliCa driver
 */
static struct platform_driver oneseg_tuner_pm_driver = {
	.probe		= oneseg_tunerpm_probe,
	.remove		= __exit_p(oneseg_tunerpm_remove),
	.driver		= {
		.name		= D_ONESEG_TUNERPM_DRIVER_NAME,
		.owner		= THIS_MODULE,
		.pm = &oneseg_tunerpm_pm_ops,
	},
};

/**
 * oneseg_tunerpm_driver_init() - The module init handler.
 *
 * Return codes
 *   0 - Success
 *   -errno - Failure
 */
static int __init oneseg_tunerpm_driver_init(void)
{
	return platform_driver_register(&oneseg_tuner_pm_driver);
}

/**
 * oneseg_tunerpm_driver_exit() - The module exit handler.
 *
 * Return codes
 *   0 - Success
 *   -errno - Failure
 */
static void __exit oneseg_tunerpm_driver_exit(void)
{
	platform_driver_unregister(&oneseg_tuner_pm_driver);
}

module_init(oneseg_tunerpm_driver_init);
module_exit(oneseg_tunerpm_driver_exit);

MODULE_LICENSE("GPL");
