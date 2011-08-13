/* linux/drivers/input/touchscreen/clearpad_i2c.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Yusuke Yoshimura <Yusuke.Yoshimura@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/clearpad.h>

static int clearpad_i2c_read(struct device *dev, u8 reg, u8 *buf, u8 len)
{
	s32 rc;
	int rsize = I2C_SMBUS_BLOCK_MAX;
	int off;

	for (off = 0; off < len; off += rsize) {
		if (len < off + I2C_SMBUS_BLOCK_MAX)
			rsize = len - off;
		rc = i2c_smbus_read_i2c_block_data(to_i2c_client(dev),
				reg + off, rsize, &buf[off]);
		if (rc < 0) {
			dev_err(dev, "%s: rc = %d\n", __func__, rc);
			return rc;
		}
	}
	return 0;
}

static int clearpad_i2c_write(struct device *dev, u8 reg, const u8 *buf, u8 len)
{
	int rc;
	u8 i;
	for (i = 0; i < len; i++) {
		rc = i2c_smbus_write_byte_data(to_i2c_client(dev),
				reg + i, buf[i]);
		if (rc)
			break;
	}
	return rc;
}

static struct clearpad_bus_data clearpad_i2c_bus_data = {
	.bustype	= BUS_I2C,
	.read		= clearpad_i2c_read,
	.write		= clearpad_i2c_write,
};

static int __devinit clearpad_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	struct clearpad_data clearpad_data = {
		.pdata = client->dev.platform_data,
		.bdata = &clearpad_i2c_bus_data,
	};
	struct platform_device *pdev;
	int rc;

	pdev = platform_device_alloc(CLEARPAD_NAME, -1);
	if (!pdev)
		return -ENOMEM;

	pdev->dev.parent = &client->dev;
	rc = platform_device_add_data(pdev,
			&clearpad_data, sizeof(clearpad_data));
	if (rc)
		goto err_device_put;

	rc = platform_device_add(pdev);
	if (rc)
		goto err_device_put;

	if (!pdev->dev.driver) {
		rc = -ENODEV;
		goto err_device_put;
	}
	dev_info(&client->dev, "%s: sucess\n", __func__);
	return 0;

err_device_put:
	platform_device_put(pdev);
	return rc;
}

static const struct i2c_device_id clearpad_id[] = {
	{ CLEARPADI2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, clearpad_id);

static struct i2c_driver clearpad_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= CLEARPADI2C_NAME,
	},
	.id_table	= clearpad_id,
	.probe		= clearpad_i2c_probe,
};

static int __init clearpad_i2c_init(void)
{
	return i2c_add_driver(&clearpad_i2c_driver);
}

static void __exit clearpad_i2c_exit(void)
{
	i2c_del_driver(&clearpad_i2c_driver);
}

MODULE_DESCRIPTION(CLEARPADI2C_NAME "ClearPad I2C Driver");
MODULE_LICENSE("GPL v2");

module_init(clearpad_i2c_init);
module_exit(clearpad_i2c_exit);
