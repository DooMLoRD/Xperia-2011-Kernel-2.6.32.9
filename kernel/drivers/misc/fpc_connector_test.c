/* /kernel/drivers/misc/fpc_connector_test.c
 *
 * Copyright (c) 2010 Sony Ericsson Mobile Comm
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fpc_connector_test.h>

struct fpc_test_data {
	struct fpc_test_platform_data *pd;
	struct device_attribute *da;
	struct device *dev;
};

static ssize_t fpc_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fpc_test_data *d = dev_get_drvdata(dev);
	struct fpc_connections_set *fpc = d->pd->fpc;
	unsigned index = attr - d->da;
	int pin = fpc->connections[index].pin;
	int value = d->pd->fpc_pin_read(pin);

	if (value < 0)
		return value;
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static int fpc_test_create_attributes(struct fpc_test_data *d)
{
	unsigned i;
	struct fpc_connections_set *fpc = d->pd->fpc;
	struct device_attribute *da =
		kzalloc(sizeof(*da) * fpc->num, GFP_KERNEL);

	if (!da) {
		dev_err(d->dev, "Failed to allocate attributes\n");
		return -ENOMEM;
	}
	for (i = 0; i < fpc->num; i++) {
		da[i].show = fpc_test_show;
		da[i].attr.name = fpc->connections[i].name;
		da[i].attr.mode = 0444;
		if (device_create_file(d->dev, &da[i]))
			goto err_exit;
	}
	d->da = da;
	return 0;

err_exit:
	dev_err(d->dev, "Failed to create attribute '%s'\n",
		fpc->connections[i].name);
	while (i > 0)
		device_remove_file(d->dev, &da[--i]);
	kfree(da);
	return -ENODEV;
}

static int __devinit fpc_test_probe(struct platform_device *pdev)
{
	struct fpc_test_platform_data *pdata = pdev->dev.platform_data;
	struct fpc_test_data *d;

	if (!pdata || !pdata->fpc_pin_read || !pdata->fpc) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENODEV;
	}
	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}
	d->dev = &pdev->dev;
	d->pd = pdev->dev.platform_data;
	if (fpc_test_create_attributes(d)) {
		kfree(d);
		return -ENODEV;
	}
	dev_info(d->dev, "probe ok\n");
	platform_set_drvdata(pdev, d);
	return 0;
}

static int __devexit fpc_test_remove(struct platform_device *pdev)
{
	unsigned i;
	struct fpc_test_data *d = platform_get_drvdata(pdev);
	struct fpc_connections_set *fpc = d->pd->fpc;

	platform_set_drvdata(pdev, NULL);
	for (i = 0; i < fpc->num; i++)
		device_remove_file(d->dev, &d->da[i]);
	kfree(d->da);
	kfree(d);
	return 0;
}

static struct platform_driver fpc_test_driver = {
	.driver	= {
		.name	= FPC_TEST_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= fpc_test_probe,
	.remove		= __devexit_p(fpc_test_remove),
};

static int __init fpc_test_init(void)
{
	return platform_driver_register(&fpc_test_driver);
}
module_init(fpc_test_init);

static void __exit fpc_test_exit(void)
{
	platform_driver_unregister(&fpc_test_driver);
}
module_exit(fpc_test_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
MODULE_DESCRIPTION("Flat printed circuit cable test driver");
MODULE_LICENSE("GPL v2");
