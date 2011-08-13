/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * Protocol driver for Bosch BMA150 accelerometer
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/bma150_ng.h>

#define BMA150_REG_CHIPID		0x00
#define BMA150_REG_ACCX_LS		0x02
#define BMA150_REG_CONTROL_0A		0x0A
#define BMA150_REG_CONTROL_0B		0x0B
#define BMA150_REG_ANY_MOTION_THRESH	0x10
#define BMA150_REG_WIDTH_BANDW		0x14
#define BMA150_REG_CONTROL_15		0x15
#define BMA150_LAST_REG			0x15

#define BMA150_REG_C0A_RESET_INT	0x40
#define BMA150_REG_C0A_SLEEP		0x01

#define BMA150_REG_C0B_ANY_MOTION	0x40
#define BMA150_REG_C0B_ENABLE_HG	0x02
#define BMA150_REG_C0B_ENABLE_LG	0x01

#define BMA150_REG_WID_BANDW_MASK	0x07

#define BMA150_REG_C15_SPI4		0x80
#define BMA150_REG_C15_EN_ADV_INT	0x40
#define BMA150_REG_C15_NEW_DATA_INT	0x20
#define BMA150_REG_C15_LATCH_INT	0x10

#define BMA150_BANDW_INIT		0x01

/* temperature offset of -30 degrees in units of 0.5 degrees */
#define BMA150_TEMP_OFFSET		60

#define BMA150_NAME			"bma150"
#define BMA150_DEVICE_NAME		"/dev/bma150"
#define BMA150_VENDORID			0x0001

/* Expected chip id information */
#define BMA150_CHIPID_MAJOR		(0x02)
#define BMA150_CHIPID_MINOR		(0x12)

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("bma150_ng");

/*
 * Data returned from accelerometer.
 * Temp is in units of 0.5 degrees C
 */
struct bma150_accel_data {
	int	accel_x;
	int	accel_y;
	int	accel_z;
	int	temp;
};

#ifndef CONFIG_INPUT_BMA150_RATE_MSEC
#define CONFIG_INPUT_BMA150_RATE_MSEC		 33
#endif
#define RESOLUTION_MSEC (1000 / HZ)

struct driver_data {
	struct input_dev	*ip_dev;
	struct i2c_client	*ic_dev;

	struct dentry		*dbfs_root;
	struct dentry		*dbfs_regs;

	struct task_struct	*bma150d;
	atomic_t		rate_msec;
	int			(*power)(bool);

	const struct bma150_platform_data *pdata;
	struct mutex lock;
};

static int bma150_noop_power(bool b)
{
	return 0;
}

static int bma150_read(struct i2c_client *ic_dev, u8 reg, u8 *buf, int len)
{
	int rc;
	rc = i2c_smbus_read_i2c_block_data(ic_dev, reg, len, buf);
	return rc > 0 ? 0 : rc;
}

static int bma150_write(struct i2c_client *ic_dev, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(ic_dev, reg, val);
}



#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_INPUT_BMA150_NG_REG_ACCESS)
static int bma150_dbfs_open(struct inode *inode, struct file *fp)
{
	fp->private_data = inode->i_private;
	return 0;
}

static ssize_t bma150_dbfs_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *f_pos)
{
	u8 *p;
	u8 *np;
	u8 *mbuf;
	int rc;
	unsigned int val;
	u8 reg;
	u8 data;
	struct driver_data *dd;

	/* format of write data is "A[A] D[D]" eg. "AA DD", "A D" etc
	   where A is address in hex, D is data in hex.
	   Multiple address/data pairs may be separated by spaces.
	*/
	if (count < 3)
		return 0;

	dd = fp->private_data;

	mbuf = kzalloc(count, GFP_KERNEL);
	if (!mbuf) {
		rc = -ENOMEM;
		goto dbfs_write_exit;
	}

	if (copy_from_user(mbuf, buf, count)) {
		rc = -EFAULT;
		goto dbfs_write_exit_copy;
	}

	p = mbuf;

	while (isspace(*p))
		p++;
	do {
		val = simple_strtoul(p, (char **)&np, 16);
		if ((val > BMA150_LAST_REG) || (p == np)) {
			rc = -EINVAL;
			goto dbfs_write_exit_copy;
		}
		while (isspace(*np) && ((np - mbuf) < count))
			np++;
		p = np;
		reg = (u8)val;

		val = simple_strtoul(p, (char **)&np, 16);
		if ((val > 0xFF)  || (p == np)) {
			rc = -EINVAL;
			goto dbfs_write_exit_copy;
		}
		while (isspace(*np) && ((np - mbuf) < count))
			np++;
		p = np;
		data = (u8)val;

		rc = bma150_write(dd->ic_dev, reg, data);

	} while (!rc && (np - mbuf) < count);

	if (rc)
		goto dbfs_write_exit;
	kfree(mbuf);

	return count;

dbfs_write_exit_copy:
	kfree(mbuf);
dbfs_write_exit:
	return rc;
}

static ssize_t bma150_dbfs_read(struct file *fp, char __user *buf,
				size_t count, loff_t *f_pos)
{
	u8 rx;
	u8 mbuf[8];
	int rc;
	int copy_size;
	struct driver_data *dd;

	dd = fp->private_data;
	if ((int)*f_pos > BMA150_LAST_REG) {
		rc = 0;
		goto dbfs_read_exit;
	}

	rc = bma150_read(dd->ic_dev, (u8)*f_pos, &rx, 1);
	if (rc)
		goto dbfs_read_exit;

	snprintf(mbuf, ARRAY_SIZE(mbuf), "%02x ", rx);
	copy_size = min(count, strlen(mbuf) + 1);
	if (copy_to_user(buf, mbuf, copy_size))
		return -EFAULT;
	(*f_pos)++;

	return copy_size;
dbfs_read_exit:
	return rc;
}

static const struct file_operations dbfs_fops = {
	.owner	= THIS_MODULE,
	.open	= bma150_dbfs_open,
	.read	= bma150_dbfs_read,
	.write	= bma150_dbfs_write,
};

static void __devinit bma150_create_dbfs_entry(struct driver_data *dd)
{
	char buf[16];

	snprintf(buf, sizeof(buf), BMA150_NAME);
	dd->dbfs_root = debugfs_create_dir(BMA150_NAME, NULL);
	if (IS_ERR_OR_NULL(dd->dbfs_root)) {
		dd->dbfs_root = NULL;
		goto dbfs_err_root;
	}

	dd->dbfs_regs = debugfs_create_file("registers",
					   S_IRUGO | S_IWUGO,
					   dd->dbfs_root, dd,
					   &dbfs_fops);
	if (IS_ERR_OR_NULL(dd->dbfs_regs)) {
		dd->dbfs_regs = NULL;
		goto dbfs_err_regs;
	}
	return;

dbfs_err_regs:
	debugfs_remove(dd->dbfs_root);
dbfs_err_root:
	return;
}

static void __devexit bma150_remove_dbfs_entry(struct driver_data *dd)
{
	if (dd->dbfs_regs)
		debugfs_remove(dd->dbfs_regs);
	if (dd->dbfs_root)
		debugfs_remove(dd->dbfs_root);
}
#else
static void __devinit bma150_create_dbfs_entry(struct driver_data *dd) { }

static void __devexit bma150_remove_dbfs_entry(struct driver_data *dd) { }
#endif

static int bma150_power_down(struct driver_data *dd)
{
	int rc;

	rc = bma150_write(dd->ic_dev, BMA150_REG_CONTROL_0A,
			BMA150_REG_C0A_SLEEP);
	if (rc)
		dev_err(&dd->ic_dev->dev, "%s: %s: error %d\n",
			__func__, "bma150_write()", rc);

	rc = dd->power(false);
	if (rc)
		dev_err(&dd->ic_dev->dev, "%s: %s: error %d\n",
			__func__, "dd->power()", rc);

	dev_dbg(&dd->ic_dev->dev, "%s: returns %d\n", __func__, rc);

	return rc;
}

static int bma150_power_up(struct driver_data *dd)
{
	int rc;

	dev_dbg(&dd->ic_dev->dev, "power up\n");

	rc = dd->power(true);
	if (rc)
		dev_err(&dd->ic_dev->dev, "%s: %s: error %d\n",
			__func__, "dd->power()", rc);

	if (!rc)
		rc = bma150_write(dd->ic_dev, BMA150_REG_CONTROL_0A, 0x00);

	dev_dbg(&dd->ic_dev->dev, "%s: returns %d\n", __func__, rc);
	return rc;
}

static int bma150_config(struct driver_data *dd)
{
	char rx_buf[2];
	int rc;
	u8 reg_bandw;
	u8 reg_15;

	rc = bma150_read(dd->ic_dev, BMA150_REG_CHIPID, rx_buf, sizeof rx_buf);
	if (rc)
		goto config_exit;
	if ((rx_buf[0] != BMA150_CHIPID_MAJOR) ||
			(rx_buf[1] != BMA150_CHIPID_MINOR)) {
		dev_err(&dd->ic_dev->dev,
				"bma150 accelerometer not detected "
				"(0x%02x 0x%02x)\n",
				rx_buf[0],
				rx_buf[1]);
		rc = -ENODEV;
		goto config_exit;
	}
	dev_dbg(&dd->ic_dev->dev,
			"detected chip id %d (0x%02x 0x%02x)\n",
			rx_buf[0] & 0x07,
			rx_buf[0],
			rx_buf[1]);

	rc = bma150_read(dd->ic_dev, BMA150_REG_WIDTH_BANDW, rx_buf, 2);
	if (rc)
		goto config_exit;

	reg_bandw	= rx_buf[0];
	reg_15		= rx_buf[1];

	rc = bma150_write(dd->ic_dev, BMA150_REG_CONTROL_15,
			reg_15 | BMA150_REG_C15_EN_ADV_INT |
			BMA150_REG_C15_LATCH_INT);
	if (rc)
		goto config_exit;

	rc = bma150_write(dd->ic_dev, BMA150_REG_CONTROL_0B, 0);
	if (rc)
		goto config_exit;

	rc = bma150_write(dd->ic_dev, BMA150_REG_WIDTH_BANDW,
			(reg_bandw & ~BMA150_REG_WID_BANDW_MASK) |
			BMA150_BANDW_INIT);

config_exit:
	return rc;
}

static void convert_regdata_to_accel_data(u8 *buf, struct bma150_accel_data *a)
{
	/* The BMA150 returns 10-bit values split over 2 bytes */
	a->accel_x = ((buf[0] & 0xC0) >> 6) | (buf[1] << 2);
	a->accel_y = ((buf[2] & 0xC0) >> 6) | (buf[3] << 2);
	a->accel_z = ((buf[4] & 0xC0) >> 6) | (buf[5] << 2);
	/* convert 10-bit signed value to 32-bit */
	if (a->accel_x & 0x200)
		a->accel_x = a->accel_x - 0x400;
	if (a->accel_y & 0x200)
		a->accel_y = a->accel_y - 0x400;
	if (a->accel_z & 0x200)
		a->accel_z = a->accel_z - 0x400;
	/* 0-based, units are 0.5 degree C */
	a->temp = buf[6] - BMA150_TEMP_OFFSET;
}

static int bma150d(void *data)
{
	struct driver_data *dd = data;
	struct bma150_accel_data acc_data;
	int rc;
	u8 buf[8];
	struct sched_param sp = { .sched_priority = MAX_RT_PRIO - 1 };

	sched_setscheduler(current, SCHED_RR, &sp);

	do {
		rc = bma150_read(dd->ic_dev, BMA150_REG_ACCX_LS,
				 buf, sizeof buf);
		if (rc) {
			dev_err(&dd->ic_dev->dev,
				"%s: fail with error %d, exiting\n",
				__func__, rc);
			mutex_lock(&dd->lock);
			/* avoid to call kthread_stop() */
			dd->bma150d = NULL;
			mutex_unlock(&dd->lock);
			return rc;
		}

		convert_regdata_to_accel_data(buf, &acc_data);
		input_report_abs(dd->ip_dev, ABS_X, acc_data.accel_x);
		input_report_abs(dd->ip_dev, ABS_Y, acc_data.accel_y);
		input_report_abs(dd->ip_dev, ABS_Z, acc_data.accel_z);
		input_report_abs(dd->ip_dev, ABS_MISC, acc_data.temp);
		input_sync(dd->ip_dev);
	} while (!kthread_should_stop() &&
		 !msleep_interruptible(atomic_read(&dd->rate_msec)));

	return rc;
}

static int bma150_kthread_start(struct driver_data *dd)
{
	dd->bma150d = kthread_run(bma150d, dd, "bma150d");
	if (IS_ERR(dd->bma150d))
		return PTR_ERR(dd->bma150d);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int bma150_suspend(struct device *dev)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	struct task_struct *ptr;

	mutex_lock(&dd->lock);
	if (dd->ip_dev->users && dd->bma150d && !IS_ERR(dd->bma150d)) {
		ptr = dd->bma150d;
		dd->bma150d = NULL;
	} else {
		mutex_unlock(&dd->lock);
		goto power_down;
	}
	mutex_unlock(&dd->lock);
	kthread_stop(ptr);

power_down:
	bma150_power_down(dd);

	return 0;
}

static int bma150_resume(struct device *dev)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int rc = 0;

	mutex_lock(&dd->lock);
	if (dd->ip_dev->users && !bma150_power_up(dd) && !bma150_config(dd))
		rc = bma150_kthread_start(dd);
	mutex_unlock(&dd->lock);
	return rc;
}
#endif

static int bma150_open(struct input_dev *dev)
{
	int rc = 0;
	struct driver_data *dd = input_get_drvdata(dev);

	rc = bma150_power_up(dd);
	if (rc)
		goto open_err_power_up;

	rc = bma150_config(dd);
	if (rc)
		goto open_err_config;

	mutex_lock(&dd->lock);
	rc = bma150_kthread_start(dd);
	mutex_unlock(&dd->lock);
	if (rc)
		goto open_err_kthread;

	return 0;

open_err_kthread:
open_err_config:
	bma150_power_down(dd);
open_err_power_up:

	return rc;
}

static void bma150_release(struct input_dev *dev)
{
	struct driver_data *dd = input_get_drvdata(dev);

	mutex_lock(&dd->lock);
	if (dd->bma150d && !IS_ERR(dd->bma150d))
		kthread_stop(dd->bma150d);
	dd->bma150d = NULL;
	mutex_unlock(&dd->lock);

	bma150_power_down(dd);
}

static void rate_set(struct driver_data *dd, unsigned long new_rate)
{
	/* Since the accuracy is determined by 1/HZ (10 ms) we round
	   upwards to that. By the current driver design, we will also
	   get a constant offset of 1/HZ. */
	if (new_rate > 0) {
		new_rate = RESOLUTION_MSEC * ((new_rate - 1) / RESOLUTION_MSEC);
		dev_dbg(&dd->ic_dev->dev, "%s: poll rate rounded to %lu ms\n",
			__func__, new_rate);
	}
	atomic_set(&dd->rate_msec, new_rate);
}

static ssize_t attr_rate_set(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t size)
{
	unsigned long new_rate;

	if (strict_strtoul(buf, 10, &new_rate) == 0) {
		struct driver_data *dd = dev_get_drvdata(dev);
		rate_set(dd, new_rate);
		return size;
	}
	return -EINVAL;
}

static struct device_attribute attributes[] = {
	__ATTR(rate, 0200, NULL, attr_rate_set)
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static const char *get_phys_path(struct device *dev)
{
	/* Get path from input device kobject and prepend "/sys". */

	char *ret = NULL;
	char *kobj_path = kobject_get_path(&dev->kobj, GFP_KERNEL);

	if (kobj_path) {
		const char *sys = "/sys";
		int sys_len = strlen(sys);
		int path_len = strlen(kobj_path);
		int tot_len = sys_len + path_len + 1;
		ret = kmalloc(tot_len, GFP_KERNEL);
		if (ret) {
			memcpy(ret, sys, sys_len);
			memcpy(ret + sys_len, kobj_path, path_len);
			*(ret + tot_len - 1) = 0;
		}
		kfree(kobj_path);
	}

	if (!ret)
		dev_err(dev, "%s: failed to get physical path\n", __func__);
	return ret;
}

static int __devinit bma150_probe(struct i2c_client *ic_dev,
		const struct i2c_device_id *id)
{
	struct driver_data *dd;
	const struct bma150_platform_data *pdata;
	const char *phys_path;
	int rc;

	if (!ic_dev->irq)
		return -EINVAL;

	pdata = dev_get_platdata(&ic_dev->dev);

	dd = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	dd->pdata = pdata;
	dd->ic_dev = ic_dev;

	if (pdata && pdata->gpio_setup) {
		rc = pdata->gpio_setup(true);
		if (rc)
			goto probe_exit_alloc;
	}

	rate_set(dd, pdata ? pdata->rate_msec : CONFIG_INPUT_BMA150_RATE_MSEC);

	if (pdata && pdata->power)
		dd->power = pdata->power;
	else
		dd->power = bma150_noop_power;

	rc = bma150_power_up(dd);
	if (rc)
		goto probe_exit_free_gpio;

	rc = bma150_config(dd);
	if (rc)
		goto probe_exit_power;

	bma150_create_dbfs_entry(dd);
	i2c_set_clientdata(ic_dev, dd);

	mutex_init(&dd->lock);
	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	dd->ip_dev->open	= bma150_open;
	dd->ip_dev->close	= bma150_release;
	dd->ip_dev->name	= BMA150_NAME;
	phys_path		= get_phys_path(&ic_dev->dev);
	dd->ip_dev->phys        = phys_path;
	dd->ip_dev->id.vendor	= BMA150_VENDORID;
	dd->ip_dev->id.product	= 1;
	dd->ip_dev->id.version	= 1;

	set_bit(EV_ABS, dd->ip_dev->evbit);

	input_set_abs_params(dd->ip_dev, ABS_X, -511, 511, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_Y, -511, 511, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_Z, -511, 511, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_MISC, -127, 127, 0, 0);

	rc = input_register_device(dd->ip_dev);
	if (rc) {
		input_free_device(dd->ip_dev);
		dev_err(&dd->ip_dev->dev,
				"bma150_probe: input_register_device rc=%d\n",
				rc);
		goto probe_err_free_phys;
	}

	rc = add_sysfs_interfaces(&ic_dev->dev);
	if (rc)
		goto probe_err_sysfs_interfaces;

	/* Power down the hardware until needed */
	rc = bma150_power_down(dd);
	if (rc)
		goto probe_err_reg_dev;

	return rc;

probe_err_reg_dev:
	remove_sysfs_interfaces(&ic_dev->dev);
probe_err_sysfs_interfaces:
	input_unregister_device(dd->ip_dev);
probe_err_free_phys:
	kfree(phys_path);
probe_err_reg:
	dd->ip_dev = NULL;
	bma150_remove_dbfs_entry(dd);
	i2c_set_clientdata(ic_dev, NULL);
probe_exit_power:
	bma150_power_down(dd);
probe_exit_free_gpio:
	if (pdata && pdata->gpio_setup)
		rc = pdata->gpio_setup(false);
probe_exit_alloc:
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit bma150_remove(struct i2c_client *ic_dev)
{
	struct driver_data *dd = i2c_get_clientdata(ic_dev);
	const char *phys_path;

	if (dd->bma150d && !IS_ERR(dd->bma150d))
		kthread_stop(dd->bma150d);

	remove_sysfs_interfaces(&ic_dev->dev);
	phys_path = dd->ip_dev->phys;
	input_unregister_device(dd->ip_dev);
	kfree(phys_path);
	bma150_remove_dbfs_entry(dd);
	bma150_power_down(dd);
	i2c_set_clientdata(ic_dev, NULL);
	kfree(dd);

	return 0;
}

static const struct i2c_device_id bma150_i2c_id[] = {
	{BMA150_NAME, 0},
	{}
};

#ifdef CONFIG_SUSPEND
static struct dev_pm_ops bma150_pm_ops = {
	.suspend	= bma150_suspend,
	.resume		= bma150_resume,
};
#endif

static struct i2c_driver bma150_driver = {
	.driver = {
		.name	= BMA150_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_SUSPEND
		.pm	= &bma150_pm_ops,
#endif
	},
	.probe		= bma150_probe,
	.remove		= __devexit_p(bma150_remove),
	.id_table	= bma150_i2c_id,
};

static int __init bma150_init(void)
{
	return i2c_add_driver(&bma150_driver);
}
module_init(bma150_init);

static void __exit bma150_exit(void)
{
	i2c_del_driver(&bma150_driver);
}
module_exit(bma150_exit);
