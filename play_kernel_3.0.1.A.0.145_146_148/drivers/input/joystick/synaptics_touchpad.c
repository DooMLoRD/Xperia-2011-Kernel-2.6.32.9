/* linux/drivers/input/joystick/synaptics_touchpad.c
 *
 * Copyright (C) 2009 Sony Ericsson Mobile Communications, INC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * <Driver name>
 *   Synaptics touchpad driver
 */

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/i2c/synaptics_touchpad.h>
#include <mach/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef	CONFIG_ARM
#include <asm/mach-types.h>
#endif

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
#include "synaptics_touchpad_firmware.h"
#endif

#define SYNAPTICS_TOUCHPAD_NAME "synaptics_touchpad"
#define SYNAPTICS_TOUCHPAD_DEVICE "dev/synaptics_touchpad"
#define SYNAPTICS_TOUCHPAD_VENDOR 0x1

#undef DEBUG

#define SYNAPTICS_MAX_N_FINGERS 10
#define SYNAPTICS_FINGER_OFF(n,x) ((((n) / 4) + !!(n % 4)) + 5 * (x))
#define SYNAPTICS_REG_MAX SYNAPTICS_FINGER_OFF(SYNAPTICS_MAX_N_FINGERS, SYNAPTICS_MAX_N_FINGERS)

#define SYNAPTICS_N_FUNCTIONS 3
#define SYNAPTICS_PDT_START 0xEF


struct synaptics_function_descriptor {
	u8 number;
	u8 int_count;
	u8 base[4];
};

struct synaptics_flash_block {
	int blocks;
	int length;
	int pos;
	u8 *data;
};

struct synaptics_flash_image {
	struct synaptics_flash_block data;
	struct synaptics_flash_block config;
};

struct synaptics_extents {
	int x_min, y_min;
	int x_max, y_max;
	int n_fingers;
};

enum synaptics_state {
	SYN_STATE_INIT,
	SYN_STATE_RUNNING,
	SYN_STATE_FLASH_ENABLE,
	SYN_STATE_FLASH_PROGRAM,
	SYN_STATE_FLASH_DATA,
	SYN_STATE_FLASH_CONFIG,
	SYN_STATE_FLASH_DISABLE,
	SYN_STATE_DISABLED,
};

enum synaptics_task {
	SYN_TASK_NONE,
	SYN_TASK_RESET,
	SYN_TASK_FLASH,
};

enum synaptics_active {
	SYN_ACTIVE_POWER = (1 << 0),
	SYN_ACTIVE_REQ   = (1 << 1),
	SYN_STANDBY      = (1 << 2),
};

enum synaptics_irq_user {
	IRQ_FLASH_ENABLE = (1 << 0),
	IRQ_SET_POWER    = (1 << 1),
};

struct synaptics_touchpad {
	enum   synaptics_state			state;
	enum   synaptics_task			task;
	struct input_dev			*input;
	struct i2c_client			*i2c;
	struct synaptics_touchpad_platform_data	*pdata;
	struct work_struct 			work;
	struct synaptics_function_descriptor	pdt[SYNAPTICS_N_FUNCTIONS];
	struct synaptics_flash_image		flash;
	struct synaptics_extents		extents;
	struct file_operations			ctrl_fops;
	int					active;
	int					irq_count;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend			early_suspend;
#endif
};

enum synaptics_touchpad_function {
	SYN_F01,
	SYN_F11,
	SYN_F34,

	SYN_F01_RMI    = SYN_F01,
	SYN_F11_2D     = SYN_F11,
	SYN_F34_FLASH  = SYN_F34,
};

enum synaptics_touchpad_reg_type {
	SYN_TYPE_DATA,
	SYN_TYPE_CTRL,
	SYN_TYPE_COMMAND,
	SYN_TYPE_QUERY,
};

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
static const char *synaptics_touchpad_flash_status[] = {
	[0] = "Success",
	[2] = "Flash Programming Not Enabled/Bad Command",
	[3] = "Invalid Block Number",
	[4] = "Block Not Erased",
	[5] = "Erase Key Incorrect",
	[6] = "Unknown Erase/Program Failure",
	[7] = "Device has been reset",
};

static const char *synaptics_touchpad_flash_reason[] = {
	[4] = "Configuration CRC Failure",
	[5] = "Firmware CRC Failure",
	[6] = "CRC In Progress",
};
#endif

#define SYN_ADDRESS(th, func, type, addr) \
  ((th)->pdt[func].base[type] + (addr))
#define SYNF(x, y, a) SYN_##x, SYN_TYPE_##y, a

static struct mutex synaptics_touchpad_lock;

static int reg_read(struct synaptics_touchpad *this, u16 reg, u8 *buf, int len)
{
	s32 rc;

#ifdef DEBUG
	int i;

	memset(buf, 0, len);
#endif
	rc = i2c_smbus_read_i2c_block_data(this->i2c, reg, len, buf);
	if (rc < 0)
		dev_err(&this->i2c->dev, "%s: %d\n", __func__, rc);

#ifdef DEBUG
	printk(SYNAPTICS_TOUCHPAD_NAME ": %s(%04x) => ", __func__, reg);
	i = len;
	while (i--)
		printk("%02x,", *(buf++));
	printk("\n");
#endif
	return (rc == len) ? 0: -EIO;
}

static int reg_write(struct synaptics_touchpad *this, u16 reg, u8 val)
{
	s32 rc;
#ifdef DEBUG
	printk(SYNAPTICS_TOUCHPAD_NAME ": %s(%04x) <= %02x\n",
			__func__, reg, val);
#endif
	rc = i2c_smbus_write_byte_data(this->i2c, reg, val);
	if (rc)
		dev_err(&this->i2c->dev, "%s: %d\n", __func__, rc);
	return rc;
}

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
static int reg_write_data(struct synaptics_touchpad *this, u16 reg,
		const u8 *buf, int len)
{
	s32 rc;

	rc = i2c_smbus_write_i2c_block_data(this->i2c, reg, len, buf);
	if (rc)
		dev_err(&this->i2c->dev, "%s: %d\n", __func__, rc);
#ifdef DEBUG
	printk(SYNAPTICS_TOUCHPAD_NAME ": %s(%04x) <= ", __func__, reg);
	while (len--)
		printk("%02x,", *(buf++));
	printk("\n");
#endif
	return rc;
}
#endif

static int synaptics_write(struct synaptics_touchpad *this,
		enum synaptics_touchpad_function func,
		enum synaptics_touchpad_reg_type type,
		u8 addr, u8 val)
{
	return reg_write(this, SYN_ADDRESS(this, func, type, addr), val);
}

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
static int synaptics_write_data(struct synaptics_touchpad *this,
		enum synaptics_touchpad_function func,
		enum synaptics_touchpad_reg_type type,
		u8 addr, const u8 *buf, int len)
{
	return reg_write_data(this, SYN_ADDRESS(this, func, type, addr), buf, len);
}
#endif

static int synaptics_read(struct synaptics_touchpad *this,
		enum synaptics_touchpad_function func,
		enum synaptics_touchpad_reg_type type,
		u8 addr, u8 *buf, int len)
{
	return reg_read(this, SYN_ADDRESS(this, func, type, addr), buf, len);
}

static void synaptics_touchpad_set_irq(struct synaptics_touchpad *dd,
					enum synaptics_irq_user user,
					bool enable)
{
	int mask;

	dev_dbg(&dd->i2c->dev, "%s: user_mask 0x%02x user 0x%02x enable %d\n",
			__func__, dd->irq_count, user, enable);

	if (enable)
		mask = dd->irq_count | user;
	else
		mask = dd->irq_count & ~user;

	if (mask && !dd->irq_count) {
		dev_dbg(&dd->i2c->dev, "%s: enabling IRQ\n", __func__);
		enable_irq(dd->i2c->irq);
	} else if (!mask && dd->irq_count) {
		dev_dbg(&dd->i2c->dev, "%s: disabling IRQ\n", __func__);
		disable_irq(dd->i2c->irq);
	}
	dd->irq_count = mask;
}

static int synaptics_touchpad_read_pdt(struct synaptics_touchpad *this)
{
	u8 addr = SYNAPTICS_PDT_START - 1;
	int i;
	int rc;

	for (i = 0; i < SYNAPTICS_N_FUNCTIONS; ++i) {
		struct synaptics_function_descriptor fdes;
		rc = reg_read(this, addr--, &fdes.number, 1);
		if (rc)
			return rc;
		rc = reg_read(this, addr--, &fdes.int_count, 1);
		if (rc)
			return rc;
		rc = reg_read(this, addr--, &fdes.base[SYN_TYPE_DATA], 1);
		if (rc)
			return rc;
		rc = reg_read(this, addr--, &fdes.base[SYN_TYPE_CTRL], 1);
		if (rc)
			return rc;
		rc = reg_read(this, addr--, &fdes.base[SYN_TYPE_COMMAND], 1);
		if (rc)
			return rc;
		rc = reg_read(this, addr--, &fdes.base[SYN_TYPE_QUERY], 1);
		if (rc)
			return rc;
#ifdef DEBUG
		printk(SYNAPTICS_TOUCHPAD_NAME ": F%02x_DATA = %02x\n",
				fdes.number, fdes.base[SYN_TYPE_DATA]);
		printk(SYNAPTICS_TOUCHPAD_NAME ": F%02x_CTRL = %02x\n",
				fdes.number, fdes.base[SYN_TYPE_CTRL]);
		printk(SYNAPTICS_TOUCHPAD_NAME ": F%02x_COMMAND = %02x\n",
				fdes.number, fdes.base[SYN_TYPE_COMMAND]);
		printk(SYNAPTICS_TOUCHPAD_NAME ": F%02x_QUERY = %02x\n",
				fdes.number, fdes.base[SYN_TYPE_QUERY]);
#endif
		switch (fdes.number) {
		case 0x01:
			memcpy(&this->pdt[SYN_F01], &fdes,
				sizeof(struct synaptics_function_descriptor));
			break;
		case 0x11:
			memcpy(&this->pdt[SYN_F11], &fdes,
				sizeof(struct synaptics_function_descriptor));
			break;
		case 0x34:
			memcpy(&this->pdt[SYN_F34], &fdes,
				sizeof(struct synaptics_function_descriptor));
			break;
		case 0x00: /* Early end of page descriptor table */
			return rc;
		default:
			break;
		}
	}
	return rc;
}

static int synaptics_touchpad_read_extents(struct synaptics_touchpad *this)
{
	int rc;
	u8 buf[4];
	enum registers {
		REG_X_LSB,
		REG_X_MSB,
		REG_Y_LSB,
		REG_Y_MSB,
	};
	int syn_finger_count[] = {1, 2, 3, 4, 5, 10, 0, 0};

	rc = synaptics_read(this, SYNF(F11_2D, CTRL, 0x06), buf, 4);
	if (rc)
		return rc;

	this->extents.x_min = 0;
	this->extents.y_min = 0;
	this->extents.x_max = (buf[REG_X_LSB] | (buf[REG_X_MSB] << 8));
	this->extents.y_max = (buf[REG_Y_LSB] | (buf[REG_Y_MSB] << 8));

	rc = synaptics_read(this, SYNF(F11_2D, QUERY, 0x01), buf, 1);
	if (rc)
		return rc;

	this->extents.n_fingers = syn_finger_count[buf[0] & 0x7];

	return rc;
}

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
static int synaptics_touchpad_flash(struct synaptics_touchpad *this);
#endif
static int synaptics_touchpad_input_init(struct synaptics_touchpad *this);

static int synaptics_touchpad_initialize(struct synaptics_touchpad *this)
{
	int rc;
	u8 buf[19];

	rc = synaptics_touchpad_read_pdt(this);
	if (rc)
		return rc;

	/* set device configured bit */
	rc = synaptics_write(this, SYNF(F01_RMI, CTRL, 0x00), 0x80);
	if (rc)
		return rc;

	/* read device configuration */
	rc = synaptics_read(this, SYNF(F01_RMI, QUERY, 0x00), buf, 17);
	if (rc)
		return rc;

	/* is this the proper device? */
	if (memcmp(buf+11, "TM145", 5)) {
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME ": bad device\n");
		return -ENODEV;
	}

	if (this->state != SYN_STATE_RUNNING)
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME ": product %d, "
				"firmware revision %d, bootloader date "
				"%04d-%02d-%02d\n", buf[2], buf[3],
				2000 + buf[4], buf[5], buf[6]);

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
	if (buf[2] == SYNAPTICS_TOUCHPAD_PRODUCT_FAMILY) {
		if (buf[3] < SYNAPTICS_TOUCHPAD_FIRMWARE_REVISION) {
			printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
					": updating firmware\n");
			this->task = SYN_TASK_FLASH;
			rc = synaptics_touchpad_flash(this);
			return rc;
		}
	}
#endif

	if (this->pdt[SYN_F11_2D].number != 0x11) {
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
				":no F11 (2D) function, device"
				" requires new firmware!\n");

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
		this->task = SYN_TASK_FLASH;
		rc = synaptics_touchpad_flash(this);
		return rc;
#else
		return -ENODEV;
#endif
	}

	/* set reporting mode to continuous while finger present */
	rc = synaptics_write(this, SYNF(F11_2D, CTRL, 0x00), 0x00);
	if (rc)
		return rc;

	rc = synaptics_touchpad_read_extents(this);
	if (rc)
		return rc;

	/* enable ABS event interrupts */
	rc = synaptics_write(this, SYNF(F01_RMI, CTRL, 0x01), 0x04);
	if (rc)
		return rc;

	this->state = SYN_STATE_RUNNING;

	rc = synaptics_touchpad_input_init(this);
	return rc;
}

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
static irqreturn_t synaptics_touchpad_irq(int irq, void *dev_id);

static int synaptics_flash_enable(struct synaptics_touchpad *this)
{
	int rc;
	u8 buf[2];

	///* enable flash event interrupts */
	//rc = synaptics_write(this, SYNF(F01_RMI, CTRL, 0x01), 0x01);
	//if (rc)
	//	return rc;

	/* read bootloader id */
	rc = synaptics_read(this, SYNF(F34_FLASH, QUERY, 0x00), buf, 2);
	if (rc)
		return rc;

	/* write bootloader id to block data */
	rc = synaptics_write_data(this, SYNF(F34_FLASH, DATA, 0x02), buf, 2);
	if (rc)
		return rc;

	mdelay(10);

	/* issue a flash program enable */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x12), 0xf);
	if (rc)
		return rc;

	this->state = SYN_STATE_FLASH_ENABLE;
	mdelay(100);

	mutex_lock(&synaptics_touchpad_lock);
	synaptics_touchpad_set_irq(this, IRQ_FLASH_ENABLE, true);
	mutex_unlock(&synaptics_touchpad_lock);

	return rc;
}

static int synaptics_flash_program(struct synaptics_touchpad *this)
{
	int rc;
	u8 buf[2];

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), buf, 1);
	if (rc)
		return rc;
	if (buf[0] != 0x80) {
		printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
				": failed enabling flash (%s)\n",
				synaptics_touchpad_flash_status[(buf[0]>>4)&7]);
		return -EIO;
	}


	printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME ": flashing enabled\n");

	/* PDT may have changed, re-read */
	rc = synaptics_touchpad_read_pdt(this);
	if (rc)
		return rc;

	/* read bootloader id */
	rc = synaptics_read(this, SYNF(F34_FLASH, QUERY, 0x00), buf, 2);
	if (rc)
		return rc;

	/* write bootloader id to block data */
	rc = synaptics_write_data(this, SYNF(F34_FLASH, DATA, 0x02), buf, 2);
	if (rc)
		return rc;

	mdelay(10);

	/* issue a firmware and configuration erase */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x12), 0x3);
	if (rc)
		return rc;

	printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME ": firmware erasing\n");
	printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME ": flashing data\n");

	this->state = SYN_STATE_FLASH_PROGRAM;
	return rc;
}


static int synaptics_flash_data(struct synaptics_touchpad *this)
{
	int rc, len;
	u8 buf;
	u8 *data;
	struct synaptics_flash_image *f = &this->flash;

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), &buf, 1);
	if (rc)
		return rc;
	if (buf != 0x80) {
		printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
				": failed flashing data (%s)\n",
				synaptics_touchpad_flash_status[(buf>>4)&7]);
		return -EIO;
	}

	/* block # low byte */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x00),
			f->data.pos & 0xff);
	if (rc)
		return rc;

	/* block # high byte */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x01),
			(f->data.pos >> 8) & 0xff);
	if (rc)
		return rc;

	data = f->data.data + f->data.pos * 16;
	len = f->data.length - f->data.pos * 16;
	if (len > 16)
		len = 16;

	/* write block data */
	rc = synaptics_write_data(this, SYNF(F34_FLASH, DATA, 0x02), data, len);
	if (rc)
		return rc;

	mdelay(10);

	/* issue a write data block command */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x12), 0x2);
	if (rc)
		return rc;

	if (f->data.pos % 100 == 0)
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME ": wrote %d blocks \n",
				f->data.pos);

	/* if we've reached the end of the data flashing */
	if (++f->data.pos == f->data.blocks) {
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
				": data flash finished\n");
		this->state = SYN_STATE_FLASH_DATA;
	}

	return rc;
}

static int synaptics_flash_config(struct synaptics_touchpad *this)
{
	int rc, len;
	u8 buf;
	u8 *data;
	struct synaptics_flash_image *f = &this->flash;

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), &buf, 1);
	if (rc)
		return rc;
	if (buf != 0x80) {
		printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
				": failed flashing config (%s)\n",
				synaptics_touchpad_flash_status[(buf>>4)&7]);
		return -EIO;
	}

	/* block # low byte */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x00),
			f->config.pos & 0xff);
	if (rc)
		return rc;

	/* block # high byte */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x01),
			(f->config.pos >> 8) & 0xff);
	if (rc)
		return rc;

	data = f->config.data + f->config.pos * 16;
	len = f->config.length - f->config.pos * 16;
	if (len > 16)
		len = 16;

	/* write block data */
	rc = synaptics_write_data(this, SYNF(F34_FLASH, DATA, 0x02), data, len);
	if (rc)
		return rc;

	mdelay(10);

	/* issue a write configuration block command */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x12), 0x6);
	if (rc)
		return rc;

	/* if we've reached the end of the configuration flashing */
	if (++f->config.pos == f->config.blocks) {
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
				": configuration flash finished\n");
		this->state = SYN_STATE_FLASH_CONFIG;
	}

	return rc;
}


static int synaptics_flash_disable(struct synaptics_touchpad *this)
{
	int rc;
	u8 buf;

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), &buf, 1);
	if (rc)
		return rc;
	if (buf != 0x80) {
		printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
				": failed flashing config (%s)\n",
				synaptics_touchpad_flash_status[(buf>>4)&7]);
		return -EIO;
	}

	mdelay(10);

	/* send a reset to the device to complete the flash procedure */
	rc = synaptics_write(this, SYNF(F01_RMI, COMMAND, 0x0), 0x1);
	if (rc)
		return rc;

	printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
			": flashing finished, resetting\n");
	this->task = SYN_TASK_RESET;
	this->state = SYN_STATE_FLASH_DISABLE;
	return rc;
}

static int synaptics_flash_verify(struct synaptics_touchpad *this)
{
	int rc;
	u8 buf;

	/* make sure that we are no longer in programming mode */
	rc = synaptics_read(this, SYNF(F01_RMI, DATA, 0x0), &buf, 1);
	if (rc)
		return rc;
	if (buf & (1 << 6)) {
		printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
				": failed disabling flash (%s)\n",
				synaptics_touchpad_flash_reason[buf&0xf]);
		return -EIO;
	}

	this->state = SYN_STATE_INIT;
	this->task = SYN_TASK_NONE;

	printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
			": device successfully flashed\n");

	synaptics_touchpad_set_irq(this, IRQ_FLASH_ENABLE, false);

	rc = synaptics_touchpad_initialize(this);
	return rc;
}

static void synaptics_firmware_load(struct synaptics_touchpad *this)
{
	u8 *data = synaptics_touchpad_firmware;
	struct synaptics_flash_image *f = &this->flash;

	/* Set up data block info */
	f->data.length = le32_to_cpu(*(u32 *)(data + 8));
	f->data.blocks = (f->data.length / 16) + !!(f->data.length % 16);
	f->data.data = data + 0x100;
	f->data.pos = 0;

	/* Set up configuration block info */
	f->config.length = le32_to_cpu(*(u32 *)(data + 12));
	f->config.blocks = (f->config.length / 16) + !!(f->config.length % 16);
	f->config.data = data + 0x100 + f->data.length;
	f->config.pos = 0;
}

static int synaptics_touchpad_flash(struct synaptics_touchpad *this)
{
	int rc = 0;

	if (this->task != SYN_TASK_FLASH) {
		printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
				": flash requested without reason\n");
		return -EBADE;
	}

	switch (this->state) {
	case SYN_STATE_INIT:
	case SYN_STATE_RUNNING:
		synaptics_firmware_load(this);
		rc = synaptics_flash_enable(this);
		break;
	case SYN_STATE_FLASH_ENABLE:
		rc = synaptics_flash_program(this);
		break;
	case SYN_STATE_FLASH_PROGRAM:
		rc = synaptics_flash_data(this);
		break;
	case SYN_STATE_FLASH_DATA:
		rc = synaptics_flash_config(this);
		break;
	case SYN_STATE_FLASH_CONFIG:
		rc = synaptics_flash_disable(this);
		break;
	case SYN_STATE_FLASH_DISABLE:
		rc = synaptics_flash_verify(this);
		break;
	case SYN_STATE_DISABLED:
		break;
	}

	if (rc) {
		printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
				": failed during flash\n");
		this->state = SYN_STATE_DISABLED;
		synaptics_touchpad_set_irq(this, IRQ_FLASH_ENABLE, false);
	}
	return rc;
}
#endif

static int synaptics_touchpad_set_power(struct synaptics_touchpad *this)
{
	int rc;
	int active;
	bool should_wake;
	u8 irq;
	int users;

	mutex_lock(&synaptics_touchpad_lock);
	active = this->active;
	users = this->input ? this->input->users : 0;

	dev_info(&this->i2c->dev, "%s: powered %d, activated %d,"
			" users %d, standby %d\n", __func__,
			!!(active & SYN_ACTIVE_POWER),
			!!(active & SYN_ACTIVE_REQ),
			users,
			!!(active & SYN_STANDBY));

	if (this->state == SYN_STATE_DISABLED) {
		mutex_unlock(&synaptics_touchpad_lock);
		return -ENODEV;
	}
	should_wake = (active & SYN_ACTIVE_REQ) && !(active & SYN_STANDBY) &&
			users;

	if (should_wake && !(active & SYN_ACTIVE_POWER)) {

		dev_dbg(&this->i2c->dev, "%s: touch panel wakeup\n",
				__func__);

		synaptics_touchpad_set_irq(this, IRQ_SET_POWER, true);
		synaptics_read(this, SYNF(F01, DATA, 0x01), &irq, 1);

		/* send a reset to the device */
		this->task = SYN_TASK_RESET;
		rc = synaptics_write(this, SYNF(F01_RMI, COMMAND, 0x0), 0x1);
		if (rc)
			dev_err(&this->i2c->dev,
				"%s: failed to reset\n", __func__);

		msleep(10);
		this->active |= SYN_ACTIVE_POWER;

	} else if (!should_wake && (active & SYN_ACTIVE_POWER)) {

		dev_dbg(&this->i2c->dev, "%s: touch panel standby\n",
				__func__);

		synaptics_touchpad_set_irq(this, IRQ_SET_POWER, false);

		rc = synaptics_write(this, SYNF(F01_RMI, CTRL, 0x0), 0x1);
		if (rc)
			dev_err(&this->i2c->dev,
				"%s: failed to enter sleep mode\n", __func__);

		/* lie to the listening applications, tell them that there
		 * are no fingers touching. */
		if (users) {
			input_mt_sync(this->input);
			input_sync(this->input);
		}
		this->active &= ~SYN_ACTIVE_POWER;
	}
	mutex_unlock(&synaptics_touchpad_lock);
	return 0;
}

static int synaptics_report_finger_n(struct synaptics_touchpad *this,
		int finger, u8 *buf)
{
	enum registers {
		REG_X_MSB,
		REG_Y_MSB,
		REG_XY_LSB,
		REG_XY_W,
		REG_Z,
	};

	/* check finger state */
	if (buf[finger >> 2] & (0x3 << ((finger % 4) << 1))) {
		int x, y, wy, wx;

		buf = buf + SYNAPTICS_FINGER_OFF(this->extents.n_fingers, finger);
		x = (buf[REG_X_MSB] << 4) | ((buf[REG_XY_LSB] & 0x0f));
		y = (buf[REG_Y_MSB] << 4) | ((buf[REG_XY_LSB] & 0xf0) >> 4);
		wx = (buf[REG_XY_W] & 0xf);
		wy = ((buf[REG_XY_W] >> 4) & 0xf);

		input_report_abs(this->input, ABS_MT_TRACKING_ID, finger + 1);
		input_report_abs(this->input, ABS_MT_POSITION_X, x);
		input_report_abs(this->input, ABS_MT_POSITION_Y, y);
		input_report_abs(this->input, ABS_MT_TOUCH_MAJOR, max(wx, wy));
		input_report_abs(this->input, ABS_MT_TOUCH_MINOR, min(wx, wy));
		input_report_abs(this->input, ABS_MT_ORIENTATION, (wx > wy));
		input_mt_sync(this->input);
		return 1;
	}
	return 0;
}

static void synaptics_touchpad_worker(struct work_struct *work)
{
	struct synaptics_touchpad *this;
	int count, rc, i;
	u8 buf[SYNAPTICS_REG_MAX];
	u8 status;
	u8 interrupt;

	this = container_of(work, struct synaptics_touchpad, work);

	if (mutex_trylock(&synaptics_touchpad_lock)) {
		/* first ATTN after reset on reprogrammable clearpad devices
		 * indicates bootloader start. we should ignore this */
		if (this->task == SYN_TASK_RESET) {
#ifdef DEBUG
			printk(SYNAPTICS_TOUCHPAD_NAME
					": irq dropped because of reset\n");
#endif
			this->task = SYN_TASK_NONE;
			mutex_unlock(&synaptics_touchpad_lock);
			return;
		}
	} else
		mutex_lock(&synaptics_touchpad_lock);

	count = 0;
	do {
		rc = synaptics_read(this, SYNF(F01, DATA, 0x01), &interrupt, 1);
		if (rc)
			goto err_i2c;
	} while (!interrupt && ++count < 10);

#ifdef DEBUG
	printk(SYNAPTICS_TOUCHPAD_NAME ": irq = %02x\n", interrupt);
#endif

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
	if ((interrupt & (1 << 0))) {
		if (this->task != SYN_TASK_FLASH)
			printk(KERN_ERR SYNAPTICS_TOUCHPAD_NAME
					": flash bit unexpectedly set\n");
		else
			synaptics_touchpad_flash(this);
		mutex_unlock(&synaptics_touchpad_lock);
		return;
	}
#endif

	if ((interrupt & (1 << 1))) {
		rc = synaptics_read(this, SYNF(F01, DATA, 0x00), &status, 1);
		if (rc)
			goto err_i2c;
#ifdef DEBUG
		printk(SYNAPTICS_TOUCHPAD_NAME ": status = %02x\n", status);
#endif
		if (status & 1) {
			printk(SYNAPTICS_TOUCHPAD_NAME ": device reset\n");
#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
			if (this->state == SYN_STATE_FLASH_DISABLE)
				synaptics_flash_verify(this);
			else
				synaptics_touchpad_initialize(this);
#else
			synaptics_touchpad_initialize(this);
#endif
		}

		rc = synaptics_read(this, SYNF(F01, DATA, 0x01), &interrupt, 1);
		if (rc)
			goto err_i2c;

	}

#ifdef CONFIG_JOYSTICK_SYNAPTICS_FLASH
	if (this->task == SYN_TASK_FLASH) {
		synaptics_touchpad_flash(this);
		mutex_unlock(&synaptics_touchpad_lock);
		return;
	}
#endif

	if (!(interrupt & (1 << 2))) {
#ifdef DEBUG
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME ": no work\n");
#endif
		rc = synaptics_read(this, SYNF(F01, DATA, 0x01), &interrupt, 1);
		mutex_unlock(&synaptics_touchpad_lock);
		return;
	}

	/* worker trails ioctl, ignore last event */
	if (!(this->active & SYN_ACTIVE_POWER)) {
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
				": late irq, dropping events\n");
		mutex_unlock(&synaptics_touchpad_lock);
		return;
	}

	rc = synaptics_read(this, SYNF(F11_2D, DATA, 0x00), buf,
			SYNAPTICS_FINGER_OFF(this->extents.n_fingers,
					this->extents.n_fingers));
	if (rc)
		goto err_i2c;

	for (count = i = 0; i < this->extents.n_fingers; ++i)
		count += synaptics_report_finger_n(this, i, buf);

	/* if no fingers were pressed, we need to output a MT sync so that the
	 * userspace can identify when the last finger has been removed from
	 * the device */
	if (!count)
		input_mt_sync(this->input);

	input_sync(this->input);

	mutex_unlock(&synaptics_touchpad_lock);
	return;

 err_i2c:
	mutex_unlock(&synaptics_touchpad_lock);
	dev_err(&this->i2c->dev, "I2C read error\n");
	return;
}

static irqreturn_t synaptics_touchpad_irq(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct synaptics_touchpad *this = dev_get_drvdata(dev);

	if (this->task == SYN_TASK_RESET) {
#ifdef DEBUG
		printk(KERN_INFO SYNAPTICS_TOUCHPAD_NAME
				": irq dropped because of reset\n");
#endif
		this->task = SYN_TASK_NONE;
	}
	else
		schedule_work(&this->work);

	return IRQ_HANDLED;
}

static int synaptics_touchpad_device_open(struct input_dev *dev)
{
	struct synaptics_touchpad *this = input_get_drvdata(dev);

	if (this->state == SYN_STATE_DISABLED)
		return -ENODEV;
	if (this->state != SYN_STATE_RUNNING)
		return -EBUSY;
	return synaptics_touchpad_set_power(this);
}

static void synaptics_touchpad_device_close(struct input_dev *dev)
{
	struct synaptics_touchpad *this = input_get_drvdata(dev);

	synaptics_touchpad_set_power(this);
}

static long synaptics_touchpad_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct synaptics_touchpad *this = container_of(file->f_op,
				struct synaptics_touchpad, ctrl_fops);

	switch (cmd) {
	case SYN_IOCTL_SET_ACTIVE:
#ifdef DEBUG
		printk(SYNAPTICS_TOUCHPAD_NAME ": active set to %lu\n", arg);
#endif
		mutex_lock(&synaptics_touchpad_lock);
		if (arg)
			this->active |= SYN_ACTIVE_REQ;
		else
			this->active &= ~SYN_ACTIVE_REQ;
		mutex_unlock(&synaptics_touchpad_lock);
		rc = synaptics_touchpad_set_power(this);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}


static int synaptics_touchpad_input_init(struct synaptics_touchpad *this)
{
	int rc;

	if (this->input)
		return 0;
	this->input = input_allocate_device();
	if (!this->input)
		return -ENOMEM;

	input_set_drvdata(this->input, this);

	this->input->open = synaptics_touchpad_device_open;
	this->input->close = synaptics_touchpad_device_close;
	this->input->name = SYNAPTICS_TOUCHPAD_NAME;
	this->input->phys = SYNAPTICS_TOUCHPAD_DEVICE;
	this->input->id.vendor = SYNAPTICS_TOUCHPAD_VENDOR;
	this->input->id.product = 1;
	this->input->id.version = 1;
	this->input->id.bustype = BUS_I2C;
	set_bit(EV_ABS, this->input->evbit);

	input_set_abs_params(this->input, ABS_MT_POSITION_X,
			this->extents.x_min, this->extents.x_max, 0, 0);
	input_set_abs_params(this->input, ABS_MT_POSITION_Y,
			this->extents.y_min, this->extents.y_max, 0, 0);
	set_bit(ABS_MT_TRACKING_ID, this->input->absbit);
	set_bit(ABS_MT_ORIENTATION, this->input->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, this->input->absbit);
	set_bit(ABS_MT_TOUCH_MINOR, this->input->absbit);

	rc = input_register_device(this->input);
	if (rc) {
		dev_err(&this->input->dev, "failed to register device\n");
		input_set_drvdata(this->input, NULL);
		input_free_device(this->input);
	}

	return rc;
}

static struct miscdevice synaptics_touchpad_ctrl = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "synaptics_ctrl",
};

static int synaptics_touchpad_ctrl_init(struct synaptics_touchpad *this)
{
	int rc;

	this->ctrl_fops.owner = THIS_MODULE;
	this->ctrl_fops.unlocked_ioctl = synaptics_touchpad_ioctl;
	synaptics_touchpad_ctrl.fops = &this->ctrl_fops;

	rc = misc_register(&synaptics_touchpad_ctrl);
	if (rc) {
		dev_err(&this->i2c->dev, "unable to register misc device\n");
		return -ENODEV;
	}
	return 0;
}

#ifdef CONFIG_SUSPEND
static int synaptics_touchpad_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_touchpad *this = i2c_get_clientdata(client);

	if (this->pdata->gpio_teardown)
		(* this->pdata->gpio_teardown)();

	return 0;
}

static int synaptics_touchpad_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_touchpad *this = i2c_get_clientdata(client);
	int rc = 0;


	if (this->pdata->gpio_setup) {
		rc = (* this->pdata->gpio_setup)();
		if (rc) {
			dev_err(&client->dev, "failed gpio init\n");
			return rc;
		}
	}
	rc = synaptics_touchpad_set_power(this);
	return rc;
}
#else
#define synaptics_touchpad_pm_suspend	NULL
#define synaptics_touchpad_pm_resume	NULL
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_touchpad_early_suspend(struct early_suspend *handler)
{
	struct synaptics_touchpad *dd =
		container_of(handler, struct synaptics_touchpad, early_suspend);

	dev_dbg(&dd->i2c->dev, "%s\n", __func__);
	mutex_lock(&synaptics_touchpad_lock);
	dd->active |= SYN_STANDBY;
	mutex_unlock(&synaptics_touchpad_lock);
	synaptics_touchpad_set_power(dd);
}

static void synaptics_touchpad_late_resume(struct early_suspend *handler)
{
	struct synaptics_touchpad *dd =
		container_of(handler, struct synaptics_touchpad, early_suspend);

	dev_dbg(&dd->i2c->dev, "%s\n", __func__);
	mutex_lock(&synaptics_touchpad_lock);
	dd->active &= ~SYN_STANDBY;
	mutex_unlock(&synaptics_touchpad_lock);
	synaptics_touchpad_set_power(dd);
}
#endif

static int __devinit synaptics_touchpad_probe_i2c(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct synaptics_touchpad *this;
	int rc;

	this = kzalloc(sizeof(struct synaptics_touchpad), GFP_KERNEL);
	if (!this)
		return -ENOMEM;

	INIT_WORK(&this->work, synaptics_touchpad_worker);

	/* Check the parameters */
	this->i2c = i2c;
	if (!this->i2c->irq) {
		dev_err(&i2c->dev, "no IRQ\n");
		rc = -ENODEV;
		goto err_free;
	}

	this->pdata = this->i2c->dev.platform_data;
	if (!this->pdata) {
		dev_err(&i2c->dev, "no platform data\n");
		rc = -EINVAL;
		goto err_free;
	}

	if (this->pdata->gpio_setup) {
		rc = (* this->pdata->gpio_setup)();
		if (rc) {
			dev_err(&i2c->dev, "failed gpio init\n");
			goto err_free;
		}
	}

	/* FIXME: This shouldn't hold up the kernel. See if we can:
	 * A) spawn a workqueue for this.
	 * B) reduce this time.
	 */
	mdelay(100);

	i2c_set_clientdata(i2c, this);
	dev_set_drvdata(&i2c->dev, this);

	this->active = SYN_ACTIVE_POWER;

	rc = synaptics_touchpad_ctrl_init(this);
	if (rc)
		goto err_teardown;

	mutex_lock(&synaptics_touchpad_lock);
	rc = request_irq(this->i2c->irq, &synaptics_touchpad_irq,
			IRQF_TRIGGER_FALLING,
			this->i2c->dev.driver->name,
			&this->i2c->dev);
	if (rc) {
		dev_err(&this->input->dev, "irq %d busy?\n", this->i2c->irq);
		mutex_unlock(&synaptics_touchpad_lock);
		goto err_input_unregister;
	}
	this->irq_count = IRQ_SET_POWER;
	mutex_unlock(&synaptics_touchpad_lock);

	rc = synaptics_touchpad_initialize(this);
	if (rc) {
		dev_err(&i2c->dev, "failed initialization (bad device?)\n");
		goto err_input_unregister;
	}

	synaptics_touchpad_set_power(this);

#ifdef CONFIG_HAS_EARLYSUSPEND
	this->early_suspend.suspend = synaptics_touchpad_early_suspend;
	this->early_suspend.resume = synaptics_touchpad_late_resume;
	register_early_suspend(&this->early_suspend);
#endif
	return 0;

err_input_unregister:
	if (this->input)
		input_unregister_device(this->input);
	misc_deregister(&synaptics_touchpad_ctrl);
err_teardown:
	if (this->pdata->gpio_teardown)
		(*this->pdata->gpio_teardown)();
err_free:
	kfree(this);
	return rc;
}

static int __devexit synaptics_touchpad_remove_i2c(struct i2c_client *i2c)
{
	struct synaptics_touchpad *this = dev_get_drvdata(&i2c->dev);

	free_irq(this->i2c->irq, &this->i2c->dev);
	unregister_early_suspend(&this->early_suspend);
	misc_deregister(&synaptics_touchpad_ctrl);
	if (this->input)
		input_unregister_device(this->input);

	if (this->pdata->gpio_teardown)
		(* this->pdata->gpio_teardown)();

	kfree(this);

	return 0;
}

static const struct i2c_device_id synaptics_touchpad_i2c_id[] = {
	{ SYNAPTICS_TOUCHPAD_NAME, 0 },
	{}
};

static const struct dev_pm_ops synaptics_touchpad_pm = {
	.suspend = synaptics_touchpad_pm_suspend,
	.resume = synaptics_touchpad_pm_resume,
};

static struct i2c_driver synaptics_touchpad_i2c_driver = {
	.driver = {
		.name	= SYNAPTICS_TOUCHPAD_NAME,
		.owner	= THIS_MODULE,
		.pm	= &synaptics_touchpad_pm,
	},
	.probe		= synaptics_touchpad_probe_i2c,
	.remove		= __devexit_p(synaptics_touchpad_remove_i2c),
	.id_table	= synaptics_touchpad_i2c_id,
};


static int __init synaptics_touchpad_init(void)
{
	int rc;

	mutex_init(&synaptics_touchpad_lock);
	rc = i2c_add_driver(&synaptics_touchpad_i2c_driver);

	return rc;
}

static void __exit synaptics_touchpad_exit(void)
{
	i2c_del_driver(&synaptics_touchpad_i2c_driver);
}

module_init(synaptics_touchpad_init);
module_exit(synaptics_touchpad_exit);

MODULE_DESCRIPTION(SYNAPTICS_TOUCHPAD_NAME " Touchpad Driver");
MODULE_LICENSE("GPL");
