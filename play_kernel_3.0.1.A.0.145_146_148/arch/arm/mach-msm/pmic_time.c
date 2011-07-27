/*
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/platform_device.h>
#include <mach/pmic.h>

static ssize_t pmic_time_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t rc;
	uint tm = 0;

	rc = pmic_rtc_get_time((struct rtc_time *)&tm);
	if (!rc)
		rc = sprintf(buf, "%u\n", tm);

	return rc;
}

static DEVICE_ATTR(pmic_time, S_IRUGO, pmic_time_show, NULL);

static int __init pmic_time_probe(struct platform_device *pdev)
{
	int rc;

	rc = device_create_file(&pdev->dev, &dev_attr_pmic_time);
	if (rc)
		printk(KERN_ERR "%s: failed to register pmic_time\n",
			__func__);

	return rc;
}

static int __devexit pmic_time_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_pmic_time);
	return 0;
}

static struct platform_driver pmic_time_driver = {
	.probe  = pmic_time_probe,
	.remove = __devexit_p(pmic_time_remove),
	.driver = {
		.name = "pmic_time",
	},
};

static int __init pmic_time_init(void)
{
	int rc;

	rc = platform_driver_register(&pmic_time_driver);
	if (rc)
		pr_err("%s: platform_driver_register failed: %d\n",
			__func__, rc);

	return rc;
}

static void __exit pmic_time_exit(void)
{
	platform_driver_unregister(&pmic_time_driver);
}

module_init(pmic_time_init);
module_exit(pmic_time_exit);

MODULE_DESCRIPTION("pmic time device");
MODULE_LICENSE("GPL v2");


