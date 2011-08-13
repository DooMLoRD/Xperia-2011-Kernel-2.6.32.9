/*
 * Copyright (C) 2009,2010 Sony Ericsson Mobile Communications Inc.
 *
 * Author: Courtney Cavin <courtney.cavin@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/gp2ap002a00f.h>

struct gp2a_data {
	struct input_dev *device;
	struct gp2a_platform_data *pdata;
	struct i2c_client *i2c_client;
};

enum gp2a_addr {
	GP2A_ADDR_PROX,
	GP2A_ADDR_GAIN,
	GP2A_ADDR_HYS,
	GP2A_ADDR_CYCLE,
	GP2A_ADDR_OPMOD,
	GP2A_ADDR_CON = 0x6
};

enum gp2a_controls {
	GP2A_CTRL_SSD = 0x01,
};

#define NUM_TRIES 5
static int gp2a_write_byte(struct gp2a_data *dt, u8 reg, u8 val)
{
	s32 rc;
	int n;
	struct device *dev = &dt->i2c_client->dev;

	for (n = 0; n < NUM_TRIES; n++) {
		rc = i2c_smbus_write_byte_data(dt->i2c_client, reg, val);
		if (rc < 0)
			dev_err(dev, "i2c_smbus write failed, %d\n", rc);
		else
			return 0;
		msleep(10);
	}
	return -EIO;
}

/* TODO: retrieve configuration from pdata */
static int gp2a_initialize(struct gp2a_data *dt)
{
	int rc;
	/* 8ms refract time */
	rc = gp2a_write_byte(dt, GP2A_ADDR_GAIN, 0x08);
	if (rc)
		return rc;

	rc = gp2a_write_byte(dt, GP2A_ADDR_HYS, 0xce);
	if (rc)
		return rc;

	/* 256 ms */
	rc = gp2a_write_byte(dt, GP2A_ADDR_CYCLE, 0x04);
	if (rc)
		return rc;

	/* Online */
	rc = gp2a_write_byte(dt, GP2A_ADDR_OPMOD, 0x00);

	return rc;
}

static int gp2a_report(struct gp2a_data *dt)
{
	int vo;

	vo = gpio_get_value(dt->pdata->gpio);

	input_report_abs(dt->device, ABS_DISTANCE, vo ? 255 : 0);
	input_sync(dt->device);

	return 0;
}

static irqreturn_t gp2a_irq(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct gp2a_data *dt;

	dt = dev_get_drvdata(dev);

	gp2a_report(dt);

	return IRQ_HANDLED;
}

static int gp2a_device_open(struct input_dev *dev)
{
	struct gp2a_data *dt = input_get_drvdata(dev);
	int rc;

	rc = gp2a_write_byte(dt, GP2A_ADDR_OPMOD, GP2A_CTRL_SSD);
	if (rc < 0) {
		dev_err(&dev->dev, "Unable to activate, err %d\n", rc);
		return rc;
	}

	enable_irq(dt->i2c_client->irq);

	if (dt->pdata->wake)
		enable_irq_wake(dt->i2c_client->irq);

	gp2a_report(dt);

	return 0;
}

static void gp2a_device_close(struct input_dev *dev)
{
	struct gp2a_data *dt = input_get_drvdata(dev);
	int rc;

	if (dt->pdata->wake)
		disable_irq_wake(dt->i2c_client->irq);

	disable_irq(dt->i2c_client->irq);
	rc = gp2a_write_byte(dt, GP2A_ADDR_OPMOD, 0);
	if (rc < 0)
		dev_err(&dev->dev, "Unable to deactivate, err %d\n", rc);
}

static int gp2a_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int gp2a_remove(struct i2c_client *client);

static const struct i2c_device_id gp2a_i2c_id[] = {
	{GP2A_I2C_NAME, 0},
	{}
};

static struct i2c_driver gp2a_i2c_driver = {
	.driver = {
		.name  = GP2A_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= gp2a_probe,
	.remove		= __devexit_p(gp2a_remove),
	.id_table	= gp2a_i2c_id,
};

static int gp2a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gp2a_platform_data *pdata;
	struct gp2a_data *dt;
	int rc;

	pdata = client->dev.platform_data;
	if (!pdata || !pdata->gpio_setup || !pdata->gpio_shutdown)
		return -EINVAL;

	rc = pdata->gpio_setup();
	if (rc)
		return rc;

	rc = gpio_direction_input(pdata->gpio);
	if (rc)
		goto err_gpio_dir_failed;

	dt = kzalloc(sizeof(struct gp2a_data), GFP_KERNEL);
	if (!dt) {
		rc = -ENOMEM;
		goto err_alloc_failed;
	}

	client->driver = &gp2a_i2c_driver;
	dt->pdata = pdata;
	dt->i2c_client = client;
	i2c_set_clientdata(client, dt);

	rc = gp2a_initialize(dt);
	if (rc) {
		rc = -EFAULT;
		goto err_init_device;
	}

	dt->device = input_allocate_device();
	if (!dt->device) {
		rc = -ENOMEM;
		goto err_allocate_device;
	}

	input_set_drvdata(dt->device, dt);

	dt->device->open = gp2a_device_open;
	dt->device->close = gp2a_device_close;
	dt->device->name = GP2A_I2C_NAME;
	dt->device->id.bustype = BUS_I2C;
	set_bit(EV_ABS, dt->device->evbit);
	set_bit(ABS_DISTANCE, dt->device->absbit);

	input_set_abs_params(dt->device, ABS_DISTANCE, 0, 255, 0, 0);

	rc = input_register_device(dt->device);
	if (rc) {
		dev_err(&dt->device->dev, "device registration failed\n");
		input_free_device(dt->device);
		goto err_register_device;
	}

	rc = request_threaded_irq(dt->i2c_client->irq, NULL,
			gp2a_irq, IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			GP2A_I2C_NAME, &dt->i2c_client->dev);
	if (rc) {
		dev_err(&dt->device->dev, "irq request failed\n");
		goto err_req_irq;
	}
	disable_irq(dt->i2c_client->irq);

	return 0;

err_req_irq:
	input_unregister_device(dt->device);
err_register_device:
err_allocate_device:
err_init_device:
	kfree(dt);
err_alloc_failed:
err_gpio_dir_failed:
	pdata->gpio_shutdown();
	return rc;
}

static int gp2a_remove(struct i2c_client *client)
{
	struct gp2a_data *dt = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);

	dt->pdata->gpio_shutdown();

	free_irq(dt->i2c_client->irq, &dt->i2c_client->dev);

	input_unregister_device(dt->device);

	kfree(dt);

	return 0;
}

static int __init gp2a_init(void)
{
	return i2c_add_driver(&gp2a_i2c_driver);
}

static void __exit gp2a_exit(void)
{
	i2c_del_driver(&gp2a_i2c_driver);
}

module_init(gp2a_init);
module_exit(gp2a_exit);

MODULE_AUTHOR("Courtney Cavin <courtney.cavin@sonyericsson.com>");
MODULE_DESCRIPTION("Sharp GP2AP002A00F I2C Proximity/Opto sensor driver");
MODULE_LICENSE("GPL v2");
