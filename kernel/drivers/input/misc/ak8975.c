/*
 * Copyright 2009 Sony Ericsson Mobile Corporation
 *
 * Author: Courtney Cavin <courtney.cavin@sonyericsson.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <mach/ak8975.h>
#include <mach/gpio.h>

#define AK8975_NAME        "ak8975"
#define AK8975_DEVICE_NAME "/dev/ak8975"
#define AK8975_VENDOR      0x0001

struct ak8975_data {
	struct input_dev *device;
	struct ak8975_platform_data *platform;
	struct i2c_client *i2c_client;
	struct work_struct work;
};

enum ak8975_reg {
	AK8975_REG_WIA    = 0x00,
	AK8975_REG_INFO   = 0x01,
	AK8975_REG_ST1    = 0x02,
	AK8975_REG_HXL    = 0x03,
	AK8975_REG_HXH    = 0x04,
	AK8975_REG_HYL    = 0x05,
	AK8975_REG_HYH    = 0x06,
	AK8975_REG_HZL    = 0x07,
	AK8975_REG_HZH    = 0x08,
	AK8975_REG_ST2    = 0x09,
	AK8975_REG_CTRL1  = 0x0A,
	AK8975_REG_CTRL2  = 0x0B,
	AK8975_REG_ATSC   = 0x0C,
	AK8975_REG_ASAX   = 0x10,
	AK8975_REG_ASAY   = 0x11,
	AK8975_REG_ASAZ   = 0x12,
};

enum ak8975_mode {
	AK8975_MODE_POWER_DOWN  = 0x00,
	AK8975_MODE_SINGLE      = 0x01,
	AK8975_MODE_CONT        = 0x02,
	AK8975_MODE_EXT_TRIGGER = 0x04,
	AK8975_MODE_TEST        = 0x08,
	AK8975_MODE_FUSE        = 0x0f,
};

enum ak8975_reset {
	AK8975_CTRL2_RESET = 0x01,
};

static struct mutex ak8975_lock;

static int ak8975_initialize(struct ak8975_data *dt)
{
	int rc;
	rc = i2c_smbus_write_byte_data(dt->i2c_client, AK8975_REG_CTRL1,
			AK8975_MODE_CONT);
	if (rc < 0)
		dev_err(&dt->i2c_client->dev, "failed to initialize device.\n");
	return rc;
}

static s16 acc_conv(u8 h, u8 l)
{
	return (s16)(~((u16)(h << 8) | l)) + 1;
}

static void ak8975_worker(struct work_struct *work)
{
	struct ak8975_data *dt;
	int intr, h, l;

	dt = container_of(work, struct ak8975_data, work);

	mutex_lock(&ak8975_lock);

	intr = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_ST1);
	if (intr < 0) {
		dev_err(&dt->i2c_client->dev, "Failed to receive irq data.");
		mutex_unlock(&ak8975_lock);
		return;
	}

	/* if no interrupt was triggered, why are we here? */
	if (!(intr & 1)) {
		mutex_unlock(&ak8975_lock);
		return;
	}

	/* X */
	l = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_HXL);
	h = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_HXH);
	if (h < 0 || l < 0)
		goto err;
	input_report_abs(dt->device, ABS_X, acc_conv(h, l));

	/* Y */
	l = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_HYL);
	h = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_HYH);
	if (h < 0 || l < 0)
		goto err;
	input_report_abs(dt->device, ABS_Y, acc_conv(h, l));

	/* Z */
	l = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_HZL);
	h = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_HZH);
	if (h < 0 || l < 0)
		goto err;
	input_report_abs(dt->device, ABS_Z, acc_conv(h, l));

	intr = i2c_smbus_read_byte_data(dt->i2c_client, AK8975_REG_ST2);
	if (intr < 0) {
		dev_err(&dt->i2c_client->dev, "Failed to receive status data.");
		mutex_unlock(&ak8975_lock);
		return;
	}

	input_sync(dt->device);
	mutex_unlock(&ak8975_lock);

	return;
 err:
	dev_err(&dt->i2c_client->dev, "i2c communication failed.\n");
	mutex_unlock(&ak8975_lock);
}

static irqreturn_t ak8975_irq(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct ak8975_data *dt;

	dt = dev_get_drvdata(dev);

	schedule_work(&dt->work);

	return IRQ_HANDLED;
}

static int ak8975_device_open(struct input_dev *dev)
{
	int rc = 0;
	struct ak8975_data *dt = input_get_drvdata(dev);

	rc = ak8975_initialize(dt);
	if (rc < 0)
		return rc;

	rc = gpio_direction_input(dt->platform->gpio);
	if (rc) {
		dev_err(&dt->device->dev, "Unable to setup gpio\n");
		return rc;
	}

	rc = request_irq(dt->i2c_client->irq, &ak8975_irq, IRQF_TRIGGER_RISING,
			AK8975_NAME, &dt->i2c_client->dev);
	if (rc) {
		dev_err(&dt->device->dev, "Unable to request irq\n");
		return rc;
	}

	return rc;
}

static void ak8975_device_close(struct input_dev *dev)
{
	struct ak8975_data *dt = input_get_drvdata(dev);

	i2c_smbus_write_byte_data(dt->i2c_client, AK8975_REG_CTRL1,
			AK8975_MODE_POWER_DOWN);

	free_irq(dt->i2c_client->irq, &dt->i2c_client->dev);
}

int ak8975_suspend(struct i2c_client *client, pm_message_t pm)
{
	struct ak8975_data *dt = i2c_get_clientdata(client);
	int rc;

	rc = i2c_smbus_write_byte_data(dt->i2c_client, AK8975_REG_CTRL1,
			AK8975_MODE_POWER_DOWN);

	return rc;
}

int ak8975_resume(struct i2c_client *client)
{
	struct ak8975_data *dt = i2c_get_clientdata(client);
	int rc;

	rc = ak8975_initialize(dt);

	return rc;
}

static int ak8975_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int ak8975_remove(struct i2c_client *client);

static const struct i2c_device_id ak8975_i2c_id[] = {
	{AK8975_NAME, 0},
	{}
};

static struct i2c_driver ak8975_i2c_driver = {
	.driver = {
		.name  = AK8975_NAME,
		.owner = THIS_MODULE,
	},
	.probe         = ak8975_probe,
	.remove        = __devexit_p(ak8975_remove),
	.id_table      = ak8975_i2c_id,
	.suspend       = ak8975_suspend,
	.resume       = ak8975_resume,
};

static int ak8975_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ak8975_data *dt;
	int rc;

	dt = kzalloc(sizeof(struct ak8975_data), GFP_KERNEL);
	if (!dt)
		return -ENOMEM;

	INIT_WORK(&dt->work, ak8975_worker);

	client->driver = &ak8975_i2c_driver;
	dt->i2c_client = client;

	dt->platform = dt->i2c_client->dev.platform_data;
	if (!dt->platform) {
		kfree(dt);
		return -EFAULT;
	}
	
	if (!dt->platform->gpio_setup || !dt->platform->gpio_shutdown) {
		kfree(dt);
		return -EFAULT;
	}

	rc = dt->platform->gpio_setup();
	if (rc) {
		kfree(dt);
		return -EFAULT;
	}

	i2c_set_clientdata(client, dt);

	dt->device = input_allocate_device();
	if (!dt->device) {
		kfree(dt);
		return -ENOMEM;
	}

	input_set_drvdata(dt->device, dt);

	dt->device->open = ak8975_device_open;
	dt->device->close = ak8975_device_close;
	dt->device->name = AK8975_NAME;
	dt->device->phys = AK8975_DEVICE_NAME;
	dt->device->id.vendor = AK8975_VENDOR;
	dt->device->id.product = 1;
	dt->device->id.version = 1;
	dt->device->id.bustype = BUS_I2C;
	set_bit(EV_ABS, dt->device->evbit);
	set_bit(ABS_X, dt->device->absbit);
	set_bit(ABS_Y, dt->device->absbit);
	set_bit(ABS_Z, dt->device->absbit);
	dt->device->absmax[ABS_X] = 4095;
	dt->device->absmin[ABS_X] = -4096;
	dt->device->absmax[ABS_Y] = 4095;
	dt->device->absmin[ABS_Y] = -4096;
	dt->device->absmax[ABS_Z] = 4095;
	dt->device->absmin[ABS_Z] = -4096;

	rc = input_register_device(dt->device);
	if (rc) {
		dev_err(&dt->device->dev, "failed to register device");
		input_set_drvdata(dt->device, NULL);
		input_free_device(dt->device);
		kfree(dt);
	}

	return rc;
}

static int ak8975_remove(struct i2c_client *client)
{
	struct ak8975_data *dt = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);

	dt->platform->gpio_shutdown();

	input_free_device(dt->device);
	input_set_drvdata(dt->device, NULL);

	kfree(dt);

	return 0;
}

static int __init ak8975_init(void)
{
	mutex_init(&ak8975_lock);

	return i2c_add_driver(&ak8975_i2c_driver);
}

static void __exit ak8975_exit(void)
{
	i2c_del_driver(&ak8975_i2c_driver);
}

module_init(ak8975_init);
module_exit(ak8975_exit);

MODULE_AUTHOR("Courtney Cavin <courtney.cavin@sonyericsson.com>");
MODULE_DESCRIPTION("AK8975 I2C 3-axis electronic compass");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ak8975");
