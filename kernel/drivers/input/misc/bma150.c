/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
 * Protocol driver for Bosch BMA150 accelerometer
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
#include <linux/bma150.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define BMA150_REG_CHIPID                0x00
#define BMA150_REG_ACCX_LS               0x02
#define BMA150_REG_CONTROL_0A            0x0A
#define BMA150_REG_CONTROL_0B            0x0B
#define BMA150_REG_ANY_MOTION_THRESH     0x10
#define BMA150_REG_WIDTH_BANDW           0x14
#define BMA150_REG_CONTROL_15            0x15
#define BMA150_LAST_REG                  0x15

#define BMA150_REG_C0A_RESET_INT         0x40
#define BMA150_REG_C0A_SLEEP             0x01

#define BMA150_REG_C0B_ANY_MOTION        0x40
#define BMA150_REG_C0B_ENABLE_HG         0x02
#define BMA150_REG_C0B_ENABLE_LG         0x01

#define BMA150_REG_WID_BANDW_MASK        0x07

#define BMA150_REG_C15_SPI4              0x80
#define BMA150_REG_C15_EN_ADV_INT        0x40
#define BMA150_REG_C15_NEW_DATA_INT      0x20
#define BMA150_REG_C15_LATCH_INT         0x10

#define BMA150_BANDW_INIT                0x00
#define BMA150_ANY_MOTION_INIT           0x01

/* temperature offset of -30 degrees in units of 0.5 degrees */
#define BMA150_TEMP_OFFSET               60

#define BMA150_NAME                      "bma150"
#define BMA150_DEVICE_NAME               "/dev/bma150"
#define BMA150_VENDORID                  0x0001

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("bma150");

/*
 * Data returned from accelerometer.
 * Temp is in units of 0.5 degrees C
 */
struct bma150_accel_data {
	int              accel_x;
	int              accel_y;
	int              accel_z;
	int              temp;
};

#ifdef CONFIG_INPUT_BMA150_I2C
typedef struct i2c_client bma150_ic_device_t;
#else
typedef struct spi_device bma150_ic_device_t;
#endif


struct driver_data {
	struct input_dev         *ip_dev;
	bma150_ic_device_t       *ic_dev;
	char                      bits_per_transfer;
	struct work_struct        work_data;
	bool                      config;
	struct list_head          next_dd;
	struct dentry            *dbfs_root;
#ifndef CONFIG_INPUT_BMA150_I2C
	struct dentry            *dbfs_bpw;
#endif
	struct dentry            *dbfs_regs;
	struct bma150_platform_data *pdata;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend     bma_early_suspend;
#endif
};

static struct mutex               bma150_dd_lock;
static struct list_head           dd_list;

struct driver_data *bma150_ic_get_data(bma150_ic_device_t *ic_dev)
{
#ifdef CONFIG_INPUT_BMA150_I2C
	return i2c_get_clientdata(ic_dev);
#else
	return spi_get_drvdata(ic_dev);
#endif
}

void bma150_ic_set_data(bma150_ic_device_t *ic_dev, struct driver_data *data)
{
#ifdef CONFIG_INPUT_BMA150_I2C
	i2c_set_clientdata(ic_dev, data);
#else
	spi_set_drvdata(ic_dev, data);
#endif
}

int bma150_ic_read(bma150_ic_device_t *ic_dev, u8 reg, u8 *buf, int len)
{
	int rc;
#ifdef CONFIG_INPUT_BMA150_I2C
	rc = i2c_smbus_read_i2c_block_data(ic_dev, reg, len, buf);
	if (rc > 0)
		return 0;
	return rc;
#else
	u8 tx_buf[2];
	u8 *rx_buf = kzalloc(len + 1, GFP_KERNEL);

	struct spi_message m;
	struct spi_transfer t;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	tx_buf[0] = 0x80 | reg;
	t.rx_buf = rx_buf;
	t.len = len + 1;

	spi_setup(ic_dev);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(ic_dev, &m);
	if (rc)
		return rc;

	memcpy(buf, rx_buf + 1, len);
	kfree(rx_buf);

	return rc;
#endif
}

int bma150_ic_write(bma150_ic_device_t *ic_dev, u8 reg, u8 val)
{
#ifdef CONFIG_INPUT_BMA150_I2C
	return i2c_smbus_write_byte_data(ic_dev, reg, val);
#else
	u8 tx_buf[2];

	struct spi_message m;
	struct spi_transfer t;

	memset(&t, 0, sizeof(t));

	t.tx_buf = tx_buf;
	tx_buf[0] = reg;
	tx_buf[1] = val;
	t.rx_buf = NULL;
	t.len = 2;

	spi_setup(ic_dev);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(ic_dev, &m);
#endif
}



#if defined(CONFIG_DEBUG_FS)
static int bma150_dbfs_open(struct inode *inode, struct file *fp)
{
	fp->private_data = inode->i_private;
	return 0;
}

static ssize_t bma150_dbfs_write(struct file *fp, const char __user *buf,
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

		rc = bma150_ic_write(dd->ic_dev, reg, data);

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
	u8                           rx;
	u8                           mbuf[8];
	int                          rc;
	int                          copy_size;
	struct driver_data          *dd;

	dd = fp->private_data;
	if ((int)*f_pos > BMA150_LAST_REG) {
		rc = 0;
		goto dbfs_read_exit;
	}

	rc = bma150_ic_read(dd->ic_dev, (u8)*f_pos, &rx, 1);
	if (rc)
		goto dbfs_read_exit;

	snprintf(mbuf, ARRAY_SIZE(mbuf), "%02x ", rx);
	copy_size = min(count, strlen(mbuf) + 1);
	if (copy_to_user(buf, mbuf, count))
		return -EFAULT;
	(*f_pos)++;

	return copy_size;
dbfs_read_exit:
	return rc;
}

static const struct file_operations dbfs_fops = {
	.owner    = THIS_MODULE,
	.open     = bma150_dbfs_open,
	.read     = bma150_dbfs_read,
	.write    = bma150_dbfs_write,
};

#ifndef CONFIG_INPUT_BMA150_I2C
static ssize_t bma150_dbfs_bpw_write(struct file *fp, const char __user *buf,
				     size_t count, loff_t *f_pos)
{
	unsigned long                new_bpw;
	int                          rc;
	int                          copy_len;
	char                         mbuf[16];
	struct driver_data          *dd;

	dd = fp->private_data;
	copy_len = min(count, ARRAY_SIZE(mbuf) - 1);
	if (copy_from_user(mbuf, buf, copy_len)) {
		rc = -EFAULT;
		goto dbfs_bpw_write_exit;
	}
	mbuf[copy_len] = '\0';
	rc = strict_strtoul(mbuf, 10, &new_bpw);
	if (rc)
		goto dbfs_bpw_write_exit;

	if ((new_bpw < 1) || (new_bpw > 4)) {
		rc = -EINVAL;
		goto dbfs_bpw_write_exit;
	}

	dd->ic_dev->bits_per_word = (new_bpw * BITS_PER_BYTE) & 0xff;
	return copy_len;

dbfs_bpw_write_exit:
	return rc;
}

static ssize_t bma150_dbfs_bpw_read(struct file *fp, char __user *buf,
				size_t count, loff_t *f_pos)
{
	char                         mbuf[16];
	struct driver_data          *dd;

	dd = fp->private_data;
	snprintf(mbuf, ARRAY_SIZE(mbuf), "%d",
		 dd->ic_dev->bits_per_word / BITS_PER_BYTE);
	return simple_read_from_buffer(buf, count, f_pos,
				       mbuf, strlen(mbuf) + 1);
}
#endif

#ifndef CONFIG_INPUT_BMA150_I2C
static const struct file_operations dbfs_bpw_fops = {
	.owner    = THIS_MODULE,
	.open     = bma150_dbfs_open,
	.read     = bma150_dbfs_bpw_read,
	.write    = bma150_dbfs_bpw_write,
};
#endif

static void __devinit bma150_create_dbfs_entry(struct driver_data *dd)
{
	char buf[16];

	snprintf(buf, sizeof(buf), BMA150_NAME);
	dd->dbfs_root = debugfs_create_dir(buf, NULL);
	if (dd->dbfs_root <= (struct dentry *)NULL) {
		dd->dbfs_root = NULL;
		goto dbfs_err_root;
	}

#ifndef CONFIG_INPUT_BMA150_I2C
	dd->dbfs_bpw = debugfs_create_file("bytes_per_word",
					   S_IRUGO | S_IWUGO,
					   dd->dbfs_root, dd,
					   &dbfs_bpw_fops);
	if (dd->dbfs_bpw <= (struct dentry *)NULL) {
		dd->dbfs_bpw = NULL;
		goto dbfs_err_bpw;
	}
#endif

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
#ifndef CONFIG_INPUT_BMA150_I2C
	debugfs_remove(dd->dbfs_bpw);
dbfs_err_bpw:
#endif
	debugfs_remove(dd->dbfs_root);
dbfs_err_root:
	return;
}

static void __devexit bma150_remove_dbfs_entry(struct driver_data *dd)
{
	if (dd->dbfs_regs)
		debugfs_remove(dd->dbfs_regs);
#ifndef CONFIG_INPUT_BMA150_I2C
	if (dd->dbfs_bpw)
		debugfs_remove(dd->dbfs_bpw);
#endif
	if (dd->dbfs_root)
		debugfs_remove(dd->dbfs_root);
}
#else
static void __devinit bma150_create_dbfs_entry(struct driver_data *dd) { }

static void __devexit bma150_remove_dbfs_entry(struct driver_data *dd) { }
#endif
static int __devexit bma150_power_down(struct driver_data *dd)
{
	int                 rc;

	rc = bma150_ic_write(dd->ic_dev, BMA150_REG_CONTROL_0A,
			BMA150_REG_C0A_SLEEP);
	return rc;
}

static int __devinit bma150_power_up(struct driver_data *dd)
{
	int                 rc;

	rc = bma150_ic_write(dd->ic_dev, BMA150_REG_CONTROL_0A, 0x00);
	return rc;
}

static int __devinit bma150_config(struct driver_data *dd)
{
	char                rx_buf[2];
	int                 rc;
	u8                  reg_bandw;
	u8                  reg_15;

	bma150_power_up(dd);

#ifndef CONFIG_INPUT_BMA150_I2C
	dd->ic_dev->bits_per_word = 32;
#endif
	rc = bma150_ic_read(dd->ic_dev, BMA150_REG_CHIPID, rx_buf, 2);

	if (rc)
		goto config_exit;
	if ((rx_buf[0] == 0x00) || (rx_buf[1] == 0x00)) {
		printk(KERN_ERR "bma150 accelerometer not detected\n");
		rc = -ENODEV;
		goto config_exit;
	}
	printk(KERN_INFO "bma150: detected chip id %d\n", rx_buf[0] & 0x07);

	rc = bma150_ic_read(dd->ic_dev, BMA150_REG_WIDTH_BANDW, rx_buf, 2);

	if (rc)
		goto config_exit;

	reg_bandw = rx_buf[0];
	reg_15    = rx_buf[1];

	rc = bma150_ic_write(dd->ic_dev, BMA150_REG_CONTROL_15,
			reg_15 | BMA150_REG_C15_EN_ADV_INT |
			BMA150_REG_C15_LATCH_INT);
	if (rc)
		goto config_exit;

	rc = bma150_ic_write(dd->ic_dev, BMA150_REG_CONTROL_0B,
			BMA150_REG_C0B_ANY_MOTION |
			BMA150_REG_C0B_ENABLE_HG  |
			BMA150_REG_C0B_ENABLE_LG);
	if (rc)
		goto config_exit;

	rc = bma150_ic_write(dd->ic_dev, BMA150_REG_ANY_MOTION_THRESH,
			BMA150_ANY_MOTION_INIT);
	if (rc)
		goto config_exit;

	rc = bma150_ic_write(dd->ic_dev, BMA150_REG_WIDTH_BANDW,
			(reg_bandw & ~BMA150_REG_WID_BANDW_MASK) |
			BMA150_BANDW_INIT);

config_exit:
	return rc;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int bma150_suspend(bma150_ic_device_t *ic_dev, pm_message_t mesg)
{
	struct driver_data *dd;
	dd = bma150_ic_get_data(ic_dev);

	bma150_power_down(dd);
	disable_irq(ic_dev->irq);

	if (dd->pdata && dd->pdata->teardown)
		dd->pdata->teardown(&ic_dev->dev);

	return 0;
}

static int bma150_resume(bma150_ic_device_t *ic_dev)
{
	struct driver_data *dd;
	int rc;

	dd = bma150_ic_get_data(ic_dev);

	if (dd->pdata && dd->pdata->setup) {
		rc = dd->pdata->setup(&ic_dev->dev);
		if (rc)
			return rc;
	}

	rc = bma150_config(dd);
	if (rc)
		return rc;

	enable_irq(ic_dev->irq);

	return rc;
}
#else /* !CONFIG_PM || (CONFIG_PM && CONFIG_HAS_EARLYSUSPEND) */
#define bma150_suspend NULL
#define bma150_resume NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma150_early_suspend(struct early_suspend *handler)
{
	struct driver_data *dd =
		container_of(handler, struct driver_data, bma_early_suspend);

	bma150_power_down(dd);
	disable_irq(dd->ic_dev->irq);

	if (dd->pdata && dd->pdata->teardown)
		dd->pdata->teardown(&dd->ic_dev->dev);

	printk(KERN_INFO "%s: done\n", __func__);
}

static void bma150_early_resume(struct early_suspend *handler)
{
	struct driver_data *dd;
	int rc;

	dd = container_of(handler, struct driver_data, bma_early_suspend);

	if (dd->pdata && dd->pdata->setup) {
		rc = dd->pdata->setup(&dd->ic_dev->dev);
		if (rc) {
			dev_err(&dd->ic_dev->dev, "%s: platform setup failed with error %d\n", __func__, rc);
			return;
		}
	}

	rc = bma150_config(dd);
	if (rc) {
		dev_err(&dd->ic_dev->dev, "%s: bma150_config() failed with error %d\n", __func__, rc);
		return;
	}

	enable_irq(dd->ic_dev->irq);

	printk(KERN_INFO "%s: done\n", __func__);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static irqreturn_t bma150_irq(int irq, void *dev_id)
{
	struct device      *dev = dev_id;
	struct driver_data *dd;

	dd = dev_get_drvdata(dev);
	schedule_work(&dd->work_data);
	return IRQ_HANDLED;
}

static int bma150_open(struct input_dev *dev)
{
	int                 rc = 0;
	struct driver_data *dd = input_get_drvdata(dev);

	if (!dd->ic_dev->irq)
		return -1;

	rc = request_irq(dd->ic_dev->irq,
			 &bma150_irq,
			 IRQF_TRIGGER_RISING,
			 BMA150_NAME,
			 &dd->ic_dev->dev);
	return rc;
}

static void bma150_release(struct input_dev *dev)
{
	struct driver_data *dd = input_get_drvdata(dev);

	free_irq(dd->ic_dev->irq, &dd->ic_dev->dev);
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
	a->temp = buf[7] - BMA150_TEMP_OFFSET;
}

static void bma150_work_f(struct work_struct *work)
{
	int                         rc;
	u8                          rx_buf[8];

	struct driver_data         *dd =
		container_of(work, struct driver_data, work_data);
	struct bma150_accel_data    acc_data;

	rc = bma150_ic_read(dd->ic_dev, BMA150_REG_ACCX_LS, rx_buf, 8);
	if (rc)
		goto workf_exit;

	convert_regdata_to_accel_data(rx_buf, &acc_data);
	input_report_abs(dd->ip_dev, ABS_X, acc_data.accel_x);
	input_report_abs(dd->ip_dev, ABS_Y, acc_data.accel_y);
	input_report_abs(dd->ip_dev, ABS_Z, acc_data.accel_z);
	input_report_abs(dd->ip_dev, ABS_MISC, acc_data.temp);
	input_sync(dd->ip_dev);

	rc = bma150_ic_write(dd->ic_dev, BMA150_REG_CONTROL_0A,
			BMA150_REG_C0A_RESET_INT);
	if (rc)
		goto workf_exit;

	return;

workf_exit:
	dev_err(&dd->ip_dev->dev, "%s: exit with error %d\n", __func__, rc);
}

#ifdef CONFIG_INPUT_BMA150_I2C
static int __devinit bma150_probe(bma150_ic_device_t *ic_dev,
		const struct i2c_device_id *id)
#else
static int __devinit bma150_probe(bma150_ic_device_t *ic_dev)
#endif
{
	struct driver_data *dd;
	int                 rc;
	char               *devname;
	struct bma150_platform_data *pdata = ic_dev->dev.platform_data;

	dd = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	devname = kzalloc(sizeof(BMA150_DEVICE_NAME) + 1, GFP_KERNEL);
	if (!devname) {
		rc = -ENOMEM;
		goto probe_exit_alloc;
	}

	mutex_lock(&bma150_dd_lock);
	list_add_tail(&dd->next_dd, &dd_list);
	mutex_unlock(&bma150_dd_lock);
	INIT_WORK(&dd->work_data, bma150_work_f);
	dd->ic_dev = ic_dev;

	if (pdata && pdata->setup) {
		rc = pdata->setup(&ic_dev->dev);
		if (rc)
			goto probe_err_setup;
	}

	dd->pdata = pdata;
	rc = bma150_config(dd);
	if (rc)
		goto probe_err_cfg;

	bma150_create_dbfs_entry(dd);
	bma150_ic_set_data(ic_dev, dd);

	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	snprintf(devname, sizeof(BMA150_DEVICE_NAME) + 1, BMA150_DEVICE_NAME);
	dd->ip_dev->open       = bma150_open;
	dd->ip_dev->close      = bma150_release;
	dd->ip_dev->name       = BMA150_NAME;
	dd->ip_dev->phys       = devname;
	dd->ip_dev->id.vendor  = BMA150_VENDORID;
	dd->ip_dev->id.product = 1;
	dd->ip_dev->id.version = 1;
	__set_bit(EV_ABS,    dd->ip_dev->evbit);
	__set_bit(ABS_X,     dd->ip_dev->absbit);
	__set_bit(ABS_Y,     dd->ip_dev->absbit);
	__set_bit(ABS_Z,     dd->ip_dev->absbit);
	__set_bit(ABS_MISC,  dd->ip_dev->absbit);

	input_set_abs_params(dd->ip_dev, ABS_X, -511, 511, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_Y, -511, 511, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_Z, -511, 511, 0, 0);
	input_set_abs_params(dd->ip_dev, ABS_MISC, -127, 127, 0, 0);
	
	rc = input_register_device(dd->ip_dev);
	if (rc) {
		dev_err(&dd->ip_dev->dev,
			"bma150_probe: input_register_device rc=%d\n",
		       rc);
		goto probe_err_reg_dev;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	dd->bma_early_suspend.suspend = bma150_early_suspend;
	dd->bma_early_suspend.resume = bma150_early_resume;
	register_early_suspend(&dd->bma_early_suspend);
#endif

	return rc;

probe_err_reg_dev:
	dd->ip_dev = NULL;
	input_free_device(dd->ip_dev);
probe_err_reg:
	bma150_remove_dbfs_entry(dd);
	bma150_ic_set_data(ic_dev, NULL);
probe_err_cfg:
	if (pdata && pdata->teardown)
		pdata->teardown(&ic_dev->dev);
probe_err_setup:
	mutex_lock(&bma150_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma150_dd_lock);
	kfree(devname);
probe_exit_alloc:
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit bma150_remove(bma150_ic_device_t *ic_dev)
{
	struct driver_data *dd;
	int                 rc;
	const char	   *devname;
	struct bma150_platform_data *pdata = ic_dev->dev.platform_data;

	dd = bma150_ic_get_data(ic_dev);
	devname = dd->ip_dev->phys;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&dd->bma_early_suspend);
#endif
	rc = bma150_power_down(dd);
	if (rc)
		dev_err(&dd->ip_dev->dev,
			"%s: power down failed with error %d\n",
			__func__, rc);
	input_unregister_device(dd->ip_dev);
	bma150_remove_dbfs_entry(dd);
	bma150_ic_set_data(ic_dev, NULL);
	if (pdata && pdata->teardown)
		pdata->teardown(&ic_dev->dev);
	mutex_lock(&bma150_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma150_dd_lock);
	kfree(devname);
	kfree(dd);

	return 0;
}

#ifdef CONFIG_INPUT_BMA150_I2C
static const struct i2c_device_id bma150_i2c_id[] = {
	{BMA150_NAME, 0},
	{}
};

static struct i2c_driver bma150_driver = {
#else
static struct spi_driver bma150_driver = {
#endif
	.driver = {
		.name  = BMA150_NAME,
		.owner = THIS_MODULE,
	},
	.probe         = bma150_probe,
	.remove        = __devexit_p(bma150_remove),
	.suspend       = bma150_suspend,
	.resume        = bma150_resume,
#ifdef CONFIG_INPUT_BMA150_I2C
	.id_table      = bma150_i2c_id,
#endif
};


static int __init bma150_init(void)
{
	INIT_LIST_HEAD(&dd_list);
	mutex_init(&bma150_dd_lock);

#ifdef CONFIG_INPUT_BMA150_I2C
	return i2c_add_driver(&bma150_driver);
#else
	return spi_register_driver(&bma150_driver);
#endif
}
module_init(bma150_init);

static void __exit bma150_exit(void)
{
#ifdef CONFIG_INPUT_BMA150_I2C
	i2c_add_driver(&bma150_driver);
#else
	spi_unregister_driver(&bma150_driver);
#endif
}
module_exit(bma150_exit);
