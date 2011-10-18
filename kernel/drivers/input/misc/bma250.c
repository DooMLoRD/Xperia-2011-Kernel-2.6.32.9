/*
 * Bosh BMA 250. Digital, triaxial acceleration sensor.
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Marcus Bauer <marcus.bauer@sonyericsson.com>
 *
 * NOTE: This file has been created by Sony Ericsson Mobile Communications AB.
 *       This file contains code from: bma150.c
 *       The orginal bma150.c header is included below:
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Protocol driver for Bosch BMA250 accelerometer
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/bma250.h>
#include <linux/delay.h>

#define BMA250_NAME                      "bma250"
#define BMA250_VENDORID                  0x0001

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("bma250");

/* bma250 register restrictions */
#define BMA250_LAST_REG 0x3F
#define BMA250_NA (1 << 8)
#define BMA250_RD (2 << 8)
#define BMA250_WR (4 << 8)
#define BMA250_RW (8 << 8)
static uint16_t bma250_mask[] = {
	0xFF | BMA250_RD, /* 00 */
	0xFF | BMA250_RD, /* 01 */
	0xC1 | BMA250_RD, /* 02 */
	0xFF | BMA250_RD, /* 03 */
	0xC1 | BMA250_RD, /* 04 */
	0xFF | BMA250_RD, /* 05 */
	0xC1 | BMA250_RD, /* 06 */
	0xFF | BMA250_RD, /* 07 */
	0xFF | BMA250_RD, /* 08 */
	0xF7 | BMA250_RD, /* 09 */
	0x80 | BMA250_RD, /* 0A */
	0xFF | BMA250_RD, /* 0B */
	0xFF | BMA250_RD, /* 0C */
	0x00 | BMA250_NA, /* 0D */
	0x00 | BMA250_NA, /* 0E */
	0x0F | BMA250_RW, /* 0F */
	0x1F | BMA250_RW, /* 10 */
	0xDE | BMA250_RW, /* 11 */
	0x00 | BMA250_NA, /* 12 */
	0xC0 | BMA250_RW, /* 13 */
	0xFF | BMA250_RW, /* 14 */
	0x00 | BMA250_NA, /* 15 */
	0xF7 | BMA250_RW, /* 16 */
	0x1F | BMA250_RW, /* 17 */
	0x00 | BMA250_NA, /* 18 */
	0xF7 | BMA250_RW, /* 19 */
	0x81 | BMA250_RW, /* 1A */
	0xF7 | BMA250_RW, /* 1B */
	0x00 | BMA250_NA, /* 1C */
	0x00 | BMA250_NA, /* 1D */
	0x37 | BMA250_RW, /* 1E */
	0x00 | BMA250_NA, /* 1F */
	0x0F | BMA250_RW, /* 20 */
	0x8F | BMA250_RW, /* 21 */
	0xFF | BMA250_RW, /* 22 */
	0xFF | BMA250_RW, /* 23 */
	0xc7 | BMA250_RW, /* 24 */
	0xFF | BMA250_RW, /* 25 */
	0xFF | BMA250_RW, /* 26 */
	0x03 | BMA250_RW, /* 27 */
	0xFF | BMA250_RW, /* 28 */
	0x00 | BMA250_NA, /* 29 */
	0xC7 | BMA250_RW, /* 2A */
	0xDF | BMA250_RW, /* 2B */
	0x7F | BMA250_RW, /* 2C */
	0x3F | BMA250_RW, /* 2D */
	0x3F | BMA250_RW, /* 2E */
	0x30 | BMA250_RW, /* 2F */
	0x00 | BMA250_NA, /* 30 */
	0x01 | BMA250_RD, /* 31 */
	0x00 | BMA250_NA, /* 32 */
	0x0F | BMA250_RW, /* 33 */
	0x03 | BMA250_RW, /* 34 */
	0x00 | BMA250_NA, /* 35 */
	0xF3 | BMA250_RW, /* 36 */
	0x7F | BMA250_RW, /* 37 */
	0xFF | BMA250_RW, /* 38 */
	0xFF | BMA250_RW, /* 39 */
	0xFF | BMA250_RW, /* 3A */
	0xFF | BMA250_RW, /* 3B */
	0xFF | BMA250_RW, /* 3C */
	0xFF | BMA250_RW, /* 3D */
	0x00 | BMA250_NA, /* 3E */
	0x00 | BMA250_NA, /* 3F */
};

const struct registers use_chip_default = {
	.range            = -1,
	.bw_sel           = -1,
	.int_mode_ctrl    = -1,
	.int_enable1      = -1,
	.int_enable2      = -1,
	.int_pin1         = -1,
	.int_new_data     = -1,
	.int_pin2         = -1,
};

#define INTERRUPT_RESOLUTION   (0)
#define TIMER_RESOLUTION       (1)
#define INVALID   (0xFFFFFFFF)

struct bma250_cnf {
	unsigned int rate;
	unsigned int range;
	unsigned int resolution;
};

/*
 * Data returned from accelerometer.
 * Temp is in units of 0.5 degrees C
 */
struct bma250_accel_data {
	short            accel_x;
	short            accel_y;
	short            accel_z;
	int              temp;
};

struct driver_data {
	struct input_dev            *ip_dev;
	struct i2c_client           *ic_dev;
	unsigned char                shift;
	struct delayed_work          work_data;
	unsigned long                delay_jiffies;
	struct list_head             next_dd;
	struct dentry               *dbfs_root;
	struct dentry               *dbfs_regs;
	struct bma250_platform_data *pdata;
	struct bma250_cnf            new_cnf;
	struct bma250_cnf            cur_cnf;
	bool                         power;
	bool                         irq_pending;
};

static struct mutex               bma250_power_lock;
static struct mutex               bma250_dd_lock;
static struct list_head           dd_list;

struct driver_data *bma250_ic_get_data(struct i2c_client *ic_dev)
{
	return i2c_get_clientdata(ic_dev);
}

void bma250_ic_set_data(struct i2c_client *ic_dev, struct driver_data *data)
{
	i2c_set_clientdata(ic_dev, data);
}

int bma250_ic_read(struct i2c_client *ic_dev, u8 reg, u8 *buf, int len)
{
	int rc;
	rc = i2c_smbus_read_i2c_block_data(ic_dev, reg, len, buf);

	if (rc > 0)
		return 0;
	return rc;
}

int bma250_ic_write(struct i2c_client *ic_dev, u8 reg, u8 val)
{
	int rc;

	if (reg > BMA250_LAST_REG) {
		printk(KERN_ERR "%s: reg 0x%.02X, out of range.\n",
			__func__, reg);
		return -EINVAL;
	}
	if (bma250_mask[reg] & BMA250_NA) {
		printk(KERN_ERR "%s: reg 0x%.02X, reserved.\n",
			__func__, reg);
		return -EINVAL;
	}
	if (bma250_mask[reg] & BMA250_RD) {
		printk(KERN_ERR "%s: reg 0x%.02X, read only.\n",
			__func__, reg);
		return -EINVAL;

	}
	if (val & ~bma250_mask[reg]) {
		printk(KERN_ERR "%s: reg 0x%.02X val 0x%.02X, out of bounds.\n",
			__func__, reg, val);
		return -EINVAL;
	}
	rc = i2c_smbus_write_byte_data(ic_dev, reg, val);

	return rc;
}

/* 10bit to 16 bit will give a 6 bit offset, shift
   it down to represent 256 lsb/g for alla ranges */
static inline u8 bma250_range2shift(u8 range)
{
	switch (range) {
	case BMA250_RANGE_16G:
		return 3;
		break;
	case BMA250_RANGE_8G:
		return 4;
		break;
	case BMA250_RANGE_4G:
		return 5;
		break;
	default:
	case BMA250_RANGE_2G:
		return 6;
		break;
	}
}

static inline int bma250_reset_interrupt(struct driver_data *dd)
{
	int rc = 0;
	u8 val;

	if (dd->cur_cnf.resolution == INTERRUPT_RESOLUTION) {
		rc = bma250_ic_read(dd->ic_dev, BMA250_INT_CTRL_REG, &val, 1);
		if (rc)
			goto interrupt_error;

		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_CTRL_REG,
						val | BMA250_INT_RESET);
		if (rc)
			goto interrupt_error;
	}
	return rc;

interrupt_error:
	dev_err(&dd->ip_dev->dev,
		"%s: device failed, error %d\n", __func__, rc);
	return rc;
}

static inline void bma250_reset_timer(struct driver_data *dd)
{
	if (dd->cur_cnf.resolution == TIMER_RESOLUTION)
		schedule_delayed_work(&dd->work_data, dd->delay_jiffies);
}

static inline int bma250_range_handler(struct driver_data *dd)
{
	int rc = 0;
	u8 range, threshold, duration = 0;

	if (dd->new_cnf.range == dd->cur_cnf.range)
		return rc;

	if (dd->new_cnf.range == 16) {
		range = BMA250_RANGE_16G;
		threshold = 2;
	} else if (dd->new_cnf.range == 8) {
		range = BMA250_RANGE_8G;
		threshold = 3;
	} else if (dd->new_cnf.range == 4) {
		range = BMA250_RANGE_4G;
		threshold = 4;
	} else {
		range = BMA250_RANGE_2G;
		threshold = 5;
	}
	rc = bma250_ic_write(dd->ic_dev, BMA250_RANGE_REG, range);
	if (rc)
		goto range_error;

	/* threshold definition for the slope int, g-range dependant */
	rc = bma250_ic_write(dd->ic_dev, BMA250_SLOPE_THR, threshold);
	if (rc)
		goto range_error;

	/* number of samples (n + 1) to be evaluted for slope int */
	rc = bma250_ic_write(dd->ic_dev, BMA250_SLOPE_DUR, duration);
	if (rc)
		goto range_error;

	dd->shift = bma250_range2shift(range);
	dd->cur_cnf.range = dd->new_cnf.range;
	return rc;

range_error:
	dev_err(&dd->ip_dev->dev,
		"%s: device failed, error %d\n", __func__, rc);
	return rc;
}

static inline int bma250_rate_handler(struct driver_data *dd)
{
	int rc = 0;
	u8 sleep, filter;

	if (dd->new_cnf.rate == dd->cur_cnf.rate)
		return rc;

	if (dd->new_cnf.rate > 1000) {
		sleep = BMA250_MODE_SLEEP_1000MS;
	} else if (dd->new_cnf.rate > 500) {
		sleep = BMA250_MODE_SLEEP_500MS;
	} else if (dd->new_cnf.rate > 100) {
		sleep = BMA250_MODE_SLEEP_100MS;
	} else if (dd->new_cnf.rate > 50) {
		sleep = BMA250_MODE_SLEEP_50MS;
	} else if (dd->new_cnf.rate > 25) {
		sleep = BMA250_MODE_SLEEP_25MS;
	} else if (dd->new_cnf.rate > 10) {
		sleep = BMA250_MODE_SLEEP_10MS;
	} else if (dd->new_cnf.rate > 6) {
		sleep = BMA250_MODE_SLEEP_6MS;
	}  else {
		sleep = BMA250_MODE_NOSLEEP;
	}

	if (dd->new_cnf.rate > 100) {
		filter = BMA250_BW_7_81HZ;
	} else if (dd->new_cnf.rate > 50) {
		filter = BMA250_BW_15_63HZ;
	} else {
		filter = BMA250_BW_31_25HZ;
	}

	/* only possible to use sleep together with interrupt */
	if (dd->new_cnf.resolution == TIMER_RESOLUTION)
		sleep = BMA250_MODE_NOSLEEP;

	rc = bma250_ic_write(dd->ic_dev, BMA250_MODE_CTRL_REG, sleep);
	if (rc)
		goto rate_error;

	rc = bma250_ic_write(dd->ic_dev, BMA250_BW_SEL_REG, filter);
	if (rc)
		goto rate_error;

	dd->delay_jiffies = msecs_to_jiffies(dd->new_cnf.rate);
	dd->cur_cnf.rate = dd->new_cnf.rate;
	return rc;

rate_error:
	dev_err(&dd->ip_dev->dev,
		"%s: device failed, error %d\n", __func__, rc);
	return rc;
}

static inline int bma250_resolution_handler(struct driver_data *dd)
{
	int rc = 0;
	u8 interrupt;

	if (dd->new_cnf.resolution == dd->cur_cnf.resolution)
		return rc;

	if (dd->new_cnf.resolution == INTERRUPT_RESOLUTION) {
		/* slope and orientation interrupt based motion detect */
		interrupt = BMA250_INT_SLOPE_Z |
				BMA250_INT_SLOPE_Y |
				BMA250_INT_SLOPE_X |
				BMA250_INT_ORIENT;
	} else {
		/* disable all interrupts, use timer based solution */
		interrupt = 0;

		/* only possible to use sleep together with interrupt */
		rc = bma250_ic_write(dd->ic_dev,BMA250_MODE_CTRL_REG,
			BMA250_MODE_NOSLEEP);
		if (rc)
			goto resolution_error;
	}
	rc = bma250_ic_write(dd->ic_dev, BMA250_INT_ENABLE1_REG, interrupt);
	if (rc)
		dev_err(&dd->ip_dev->dev,
			"%s: device failed, error %d\n", __func__, rc);
	else
		dd->cur_cnf.resolution = dd->new_cnf.resolution;
	return rc;

resolution_error:
	dev_err(&dd->ip_dev->dev,
		"%s: device failed, error %d\n", __func__, rc);
	return rc;
}

static inline int bma250_update_settings(struct driver_data *dd)
{
	int rc;

	rc = bma250_range_handler(dd);
	if (rc)
		return rc;

	rc = bma250_rate_handler(dd);
	if (rc)
		return rc;

	rc = bma250_resolution_handler(dd);
	if (rc)
		return rc;

	bma250_reset_timer(dd);

	rc = bma250_reset_interrupt(dd);
	return rc;
}

static ssize_t bma250_rate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dd->cur_cnf.rate);
}

static ssize_t bma250_rate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int rc;
	unsigned long val;

	rc = strict_strtoul(buf, 10, &val);
	if (rc)
		return rc;

	if ((val >= 1) && (val <= 10000)) {
		dd->new_cnf.rate = (unsigned int)val;
		return strnlen(buf, count);
	}
	return -EINVAL;
}

static ssize_t bma250_range_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dd->cur_cnf.range);
}

static ssize_t bma250_range_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int rc;
	unsigned long val;

	rc = strict_strtoul(buf, 10, &val);
	if (rc)
		return rc;

	if ((val == 2) || (val == 4) || (val == 8) || (val == 16)) {
		dd->new_cnf.range = (unsigned int)val;
		return strnlen(buf, count);
	}
	return -EINVAL;
}

static ssize_t bma250_resolution_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dd->cur_cnf.resolution);
}

static ssize_t bma250_resolution_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int rc;
	unsigned long val;

	rc = strict_strtoul(buf, 10, &val);
	if (rc)
		return rc;

	if ((val == INTERRUPT_RESOLUTION) || (val == TIMER_RESOLUTION)) {
		dd->new_cnf.resolution = (unsigned int)val;
		return strnlen(buf, count);
	}
	return -EINVAL;
}

static struct device_attribute attributes[] = {
	__ATTR(bma250_rate, 0644, bma250_rate_show, bma250_rate_store),
	__ATTR(bma250_range, 0644, bma250_range_show, bma250_range_store),
	__ATTR(bma250_resolution, 0644, bma250_resolution_show,
				bma250_resolution_store)
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

#if defined(CONFIG_DEBUG_FS)
static int bma250_dbfs_open(struct inode *inode, struct file *fp)
{
	fp->private_data = inode->i_private;
	return 0;
}

static ssize_t bma250_dbfs_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *f_pos)
{
	u8                          *p;
	u8                          *np;
	u8                          *mbuf;
	int                          rc;
	unsigned int                 val;
	u8                           reg;
	u8                           data;
	struct driver_data          *dd;

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
		if ((val > BMA250_LAST_REG) || (p == np)) {
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

		rc = bma250_ic_write(dd->ic_dev, reg, data);

		/* update here to avoid checking g-range at each interrupt */
		if ((!rc) && (reg == BMA250_RANGE_REG))
			dd->shift = bma250_range2shift(data);

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

static ssize_t bma250_dbfs_read(struct file *fp, char __user *buf,
				size_t count, loff_t *f_pos)
{
	u8                           rx;
	u8                           mbuf[8];
	int                          rc;
	int                          copy_size;
	struct driver_data          *dd;

	dd = fp->private_data;
	if ((int)*f_pos > BMA250_LAST_REG) {
		rc = 0;
		goto dbfs_read_exit;
	}

	rc = bma250_ic_read(dd->ic_dev, (u8)*f_pos, &rx, 1);
	if (rc)
		goto dbfs_read_exit;

	snprintf(mbuf, ARRAY_SIZE(mbuf), "%02x %02x\n", (u8)*f_pos, rx);
	copy_size = min(count, strlen(mbuf) + 1);
	if (copy_to_user(buf, mbuf, copy_size))
		return -EFAULT;
	(*f_pos)++;

	return copy_size;
dbfs_read_exit:
	return rc;
}

static const struct file_operations dbfs_fops = {
	.owner    = THIS_MODULE,
	.open     = bma250_dbfs_open,
	.read     = bma250_dbfs_read,
	.write    = bma250_dbfs_write,
};

static void __devinit bma250_create_dbfs_entry(struct driver_data *dd)
{
	char buf[16];

	snprintf(buf, sizeof(buf), BMA250_NAME);
	dd->dbfs_root = debugfs_create_dir(buf, NULL);
	if (dd->dbfs_root <= (struct dentry *)NULL) {
		dd->dbfs_root = NULL;
		goto dbfs_err_root;
	}

	dd->dbfs_regs = debugfs_create_file("registers",
					   S_IRUGO | S_IWUGO,
					   dd->dbfs_root, dd,
					   &dbfs_fops);
	if (dd->dbfs_regs <= (struct dentry *)NULL) {
		dd->dbfs_regs = NULL;
		goto dbfs_err_regs;
	}
	return;

dbfs_err_regs:
	debugfs_remove(dd->dbfs_root);
dbfs_err_root:
	return;
}

static void __devexit bma250_remove_dbfs_entry(struct driver_data *dd)
{
	if (dd->dbfs_regs)
		debugfs_remove(dd->dbfs_regs);
	if (dd->dbfs_root)
		debugfs_remove(dd->dbfs_root);
}
#else
static void __devinit bma250_create_dbfs_entry(struct driver_data *dd) { }

static void __devexit bma250_remove_dbfs_entry(struct driver_data *dd) { }
#endif
static int  bma250_power_down(struct driver_data *dd)
{
	int                 rc;

	mutex_lock(&bma250_power_lock);

	if (!dd->ip_dev->users)
		rc = bma250_ic_write(dd->ic_dev, BMA250_MODE_CTRL_REG,
				BMA250_MODE_SUSPEND);
	else
		rc = bma250_ic_write(dd->ic_dev, BMA250_MODE_CTRL_REG,
				BMA250_MODE_LOWPOWER);
	dd->power = false;
	cancel_delayed_work(&dd->work_data);

	mutex_unlock(&bma250_power_lock);
	return rc;
}

static int  bma250_power_up(struct driver_data *dd)
{
	int                 rc;

	mutex_lock(&bma250_power_lock);

	dd->cur_cnf.rate = INVALID;
	dd->cur_cnf.range = INVALID;
	dd->cur_cnf.resolution = INVALID;
	dd->power = true;

	rc = bma250_ic_write(dd->ic_dev, BMA250_RESET_REG, BMA250_RESET);
	msleep(4);

	mutex_unlock(&bma250_power_lock);
	return rc;
}

static int bma250_config(struct driver_data *dd)
{
	int                 rc;
	u8                  rx_buf[2];
	const struct registers   *preg = &use_chip_default;
	struct bma250_platform_data *pdata = dd->ic_dev->dev.platform_data;

	/* use platform data register values if they exist */
	if (pdata && pdata->reg)
		preg = pdata->reg;

	rc = bma250_power_up(dd);
	if (rc)
		goto config_exit;

	rc = bma250_ic_read(dd->ic_dev, BMA250_CHIP_ID_REG, rx_buf, 2);
	if (rc)
		goto config_exit;

	if ((rx_buf[0] == 0x00) || (rx_buf[1] == 0x00)) {
		printk(KERN_ERR "bma250: device not found.\n");
		rc = -ENODEV;
		goto config_exit;
	}
	printk(KERN_INFO "bma250: detected chip id %d, rev 0x%X\n",
				rx_buf[0] & 0x07, rx_buf[1]);

	if (preg->int_pin1 >= 0) {
		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_PIN1_REG,
							preg->int_pin1);
		if (rc)
			goto config_exit;
	}

	if (preg->int_pin2 >= 0) {
		rc = bma250_ic_write(dd->ic_dev, BMA250_INT_PIN2_REG,
							preg->int_pin2);
		if (rc)
			goto config_exit;
	}

	rc = bma250_ic_write(dd->ic_dev, BMA250_INT_CTRL_REG,
						BMA250_INT_MODE_LATCHED);
	if (rc)
		goto config_exit;

	rc = bma250_update_settings(dd);

config_exit:

	return rc;
}

static inline int bma250_report_data(struct driver_data *dd)
{
	int                         rc = 0;
	u8                          rx_buf[7];
	struct bma250_accel_data    data;

	rc = bma250_ic_read(dd->ic_dev, BMA250_X_AXIS_LSB_REG, rx_buf, 7);
	if (rc)
		goto report_error;

	/* 10bit signed to 16bit signed */
	data.accel_x = ((rx_buf[1] << 8) | (rx_buf[0] & 0xC0));
	data.accel_y = ((rx_buf[3] << 8) | (rx_buf[2] & 0xC0));
	data.accel_z = ((rx_buf[5] << 8) | (rx_buf[4] & 0xC0));

	/* sensitivty 256lsb/g for all g-ranges */
	data.accel_x = data.accel_x >> dd->shift;
	data.accel_y = data.accel_y >> dd->shift;
	data.accel_z = data.accel_z >> dd->shift;

	/* sensitivty 0.5C, center temprature 24C */
	data.temp = (signed char)rx_buf[6] + 24*2;

	input_report_abs(dd->ip_dev, ABS_X, data.accel_x);
	input_report_abs(dd->ip_dev, ABS_Y, data.accel_y);
	input_report_abs(dd->ip_dev, ABS_Z, data.accel_z);
	input_report_abs(dd->ip_dev, ABS_MISC, data.temp);
	input_sync(dd->ip_dev);
	return rc;

report_error:
	dev_err(&dd->ip_dev->dev,
		"%s: device failed, error %d\n", __func__, rc);
	return rc;
}

#ifdef CONFIG_SUSPEND
static int bma250_suspend(struct device *dev)
{
	struct driver_data *dd = dev_get_drvdata(dev);

	bma250_power_down(dd);
	if (dd->pdata && dd->pdata->teardown)
		dd->pdata->teardown(&dd->ic_dev->dev);

	return 0;
}

static int bma250_resume(struct device *dev)
{
	struct driver_data *dd = dev_get_drvdata(dev);
	int rc;

	if (dd->pdata && dd->pdata->setup) {
		rc = dd->pdata->setup(&dd->ic_dev->dev);
		if (rc)
			return rc;
	}

	rc = bma250_config(dd);
	if (rc)
		return rc;

	if (dd->irq_pending) {
		dd->irq_pending= false;
		rc = bma250_report_data(dd);
		if (rc)
			return rc;
	}

	return rc;
}
#endif

static void bma250_work_f(struct work_struct *work)
{
	int                         rc;
	struct driver_data         *dd = container_of(work, struct driver_data,
							work_data.work);
	mutex_lock(&bma250_power_lock);

	if (dd->power) {
		rc = bma250_report_data(dd);
		if (rc)
			goto work_error;

		rc = bma250_update_settings(dd);
	}
work_error:
	mutex_unlock(&bma250_power_lock);
	return ;
}

static irqreturn_t bma250_irq(int irq, void *dev_id)
{
	int                         rc = 0;
	struct device              *dev = dev_id;
	struct driver_data         *dd = dev_get_drvdata(dev);

	mutex_lock(&bma250_power_lock);

	if (dd->power) {
		rc = bma250_report_data(dd);
		if (rc)
			goto irq_error;

		rc = bma250_update_settings(dd);
	} else {
		dd->irq_pending = true;
	}

irq_error:
	mutex_unlock(&bma250_power_lock);
	return IRQ_HANDLED;
}

static int bma250_open(struct input_dev *dev)
{
	int                 rc = 0;
	struct driver_data *dd = input_get_drvdata(dev);

	if (!dd->ic_dev->irq)
		return -EINVAL;

	rc = request_threaded_irq(dd->ic_dev->irq,
				  NULL,
				  &bma250_irq,
				  IRQF_TRIGGER_RISING,
				  BMA250_NAME,
				  &dd->ic_dev->dev);

	rc = set_irq_wake(dd->ic_dev->irq, 1);
	if (rc) {
		dev_err(&dd->ic_dev->dev,
			"%s: set_irq_wake failed with error %d\n",
			__func__,rc);
		goto probe_err_wake_irq;
	}

	rc = bma250_update_settings(dd);
	if (rc)
		goto probe_err_wake_irq;

	return rc;

probe_err_wake_irq:
	free_irq(dd->ic_dev->irq, &dd->ic_dev->dev);
	return rc;

}

static void bma250_release(struct input_dev *dev)
{
	int                 rc;
	struct driver_data *dd = input_get_drvdata(dev);
	rc = set_irq_wake(dd->ic_dev->irq, 0);
	if (rc)
		dev_err(&dd->ic_dev->dev,
			"%s: set_irq_wake failed with error %d\n",
			__func__, rc);
	free_irq(dd->ic_dev->irq, &dd->ic_dev->dev);
}

static int __devinit bma250_probe(struct i2c_client *ic_dev,
		const struct i2c_device_id *id)
{
	struct driver_data *dd;
	int                 rc;
	struct bma250_platform_data *pdata = ic_dev->dev.platform_data;

	dd = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	mutex_lock(&bma250_dd_lock);
	list_add_tail(&dd->next_dd, &dd_list);
	mutex_unlock(&bma250_dd_lock);
	dd->ic_dev = ic_dev;

	INIT_DELAYED_WORK(&dd->work_data, bma250_work_f);

	if (pdata && pdata->setup) {
		rc = pdata->setup(&ic_dev->dev);
		if (rc)
			goto probe_err_setup;
	}

	dd->pdata = pdata;

	/* initial configuration */
	dd->new_cnf.rate = 200;
	dd->new_cnf.range = 2;
	dd->new_cnf.resolution = INTERRUPT_RESOLUTION;

	rc = bma250_config(dd);
	if (rc)
		goto probe_err_cfg;

	bma250_create_dbfs_entry(dd);
	bma250_ic_set_data(ic_dev, dd);

	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	dd->ip_dev->open       = bma250_open;
	dd->ip_dev->close      = bma250_release;
	dd->ip_dev->name       = BMA250_NAME;
	dd->ip_dev->id.vendor  = BMA250_VENDORID;
	dd->ip_dev->id.product = 1;
	dd->ip_dev->id.version = 1;
	__set_bit(EV_ABS,       dd->ip_dev->evbit);
	__set_bit(ABS_X,        dd->ip_dev->absbit);
	__set_bit(ABS_Y,        dd->ip_dev->absbit);
	__set_bit(ABS_Z,        dd->ip_dev->absbit);
	__set_bit(ABS_MISC,     dd->ip_dev->absbit);
	input_set_abs_params(dd->ip_dev, ABS_X, -4096, 4095, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_Y, -4096, 4095, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_Z, -4096, 4095, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_MISC, -80, 175, 0, 0);

	rc = input_register_device(dd->ip_dev);
	if (rc) {
		input_free_device(dd->ip_dev);
		goto probe_err_reg;
	}

	rc = add_sysfs_interfaces(&dd->ip_dev->dev);
	if (rc)
		goto probe_err_sysfs;

	return rc;

probe_err_sysfs:
	input_unregister_device(dd->ip_dev);
probe_err_reg:
	bma250_remove_dbfs_entry(dd);
	bma250_ic_set_data(ic_dev, NULL);
probe_err_cfg:
	if (pdata && pdata->teardown)
		pdata->teardown(&ic_dev->dev);
probe_err_setup:
	mutex_lock(&bma250_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma250_dd_lock);
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit bma250_remove(struct i2c_client *ic_dev)
{
	struct driver_data *dd;
	int                 rc;
	struct bma250_platform_data *pdata = ic_dev->dev.platform_data;

	dd = bma250_ic_get_data(ic_dev);
	rc = bma250_power_down(dd);
	if (rc)
		dev_err(&dd->ip_dev->dev,
			"%s: power down failed with error %d\n",
			__func__, rc);
	remove_sysfs_interfaces(&dd->ip_dev->dev);
	input_unregister_device(dd->ip_dev);
	bma250_remove_dbfs_entry(dd);
	bma250_ic_set_data(ic_dev, NULL);
	if (pdata && pdata->teardown)
		pdata->teardown(&ic_dev->dev);
	mutex_lock(&bma250_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma250_dd_lock);
	kfree(dd);
	return 0;
}

static const struct i2c_device_id bma250_i2c_id[] = {
	{BMA250_NAME, 0},
	{}
};

#ifdef CONFIG_SUSPEND
static struct dev_pm_ops bma250_pm_ops = {
	.suspend	= bma250_suspend,
	.resume		= bma250_resume,
};
#endif

static struct i2c_driver bma250_driver = {
	.driver = {
		.name  = BMA250_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_SUSPEND
		.pm	= &bma250_pm_ops,
#endif
	},
	.probe         = bma250_probe,
	.remove        = __devexit_p(bma250_remove),
	.id_table      = bma250_i2c_id,
};


static int __init bma250_init(void)
{
	INIT_LIST_HEAD(&dd_list);
	mutex_init(&bma250_dd_lock);
	mutex_init(&bma250_power_lock);
	return i2c_add_driver(&bma250_driver);
}
module_init(bma250_init);

static void __exit bma250_exit(void)
{
	i2c_del_driver(&bma250_driver);
}
module_exit(bma250_exit);
