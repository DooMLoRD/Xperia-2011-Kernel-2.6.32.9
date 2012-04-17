/* linux/drivers/input/touchscreen/clearpad_core.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Courtney Cavin <courtney.cavin@sonyericsson.com>
 *         Yusuke Yoshimura <Yusuke.Yoshimura@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/clearpad.h>
#include <mach/gpio.h>
#include <linux/ctype.h>
#include <linux/firmware.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_ARM
#include <asm/mach-types.h>
#endif

#define SYNAPTICS_CLEARPAD_VENDOR		0x1
#define SYNAPTICS_MAX_N_FINGERS			10
#define SYNAPTICS_DEVICE_STATUS			0x13
#define SYNAPTICS_MAX_Z_VALUE			255
#define SYNAPTICS_MAX_W_VALUE			15
#define SYNAPTICS_PDT_START			0xEF
#define SYNAPTICS_STRING_LENGTH			128
#define SYNAPTICS_RETRY_NUM_OF_INITIAL_CHECK	2
#define SYNAPTICS_FINGER_OFF(n, x) ((((n) / 4) + !!(n % 4)) + 5 * (x))
#define SYNAPTICS_REG_MAX \
	SYNAPTICS_FINGER_OFF(SYNAPTICS_MAX_N_FINGERS, SYNAPTICS_MAX_N_FINGERS)
#define HWTEST_SIZE_OF_COMMAND_PREFIX		2

#define SYN_ADDRESS(th, func, type, addr) ((th)->pdt[func].base[type] + (addr))
#define SYNSET(...)  __VA_ARGS__
#define SYNX(x) SYN_##x
#define SYNY(y) SYN_TYPE_##y
#define SYNF(x, y, a) SYNSET(SYNX(x), SYNY(y), a)

#define DEVICE_STATUS_UNCONFIGURED_RESET_OCCURRED	0x81
#define DEVICE_COMMAND_RESET				0x01
#define DEVICE_CONTROL_SLEEP_MODE_NORMAL_OPERATION	0x00
#define DEVICE_CONTROL_SLEEP_MODE_SENSOR_SLEEP		0x01
#define DEVICE_CONTROL_CONFIGURED			0x80
#define XY_REPORTING_MODE_REDUCED_REPORTING_MODE	0x01
#define FLASH_CONTROL_WRITE_FIRMWARE_BLOCK		0x02
#define FLASH_CONTROL_ERASE_ALL				0x03
#define FLASH_CONTROL_WRITE_CONFIGURATION_BLOCK		0x06
#define FLASH_CONTROL_ENABLE_FLASH_PROGRAMMING		0x0f
#define FLASH_CONTROL_PROGRAM_ENABLED			0x80

#define LOGx(this, LEVEL, X, ...)				\
do {								\
	dev_dbg(&this->pdev->dev, LEVEL "%s: %d: " X,		\
		       __func__, __LINE__, ## __VA_ARGS__);	\
} while (0)
#define LOG_STAT(this, X, ...) LOGx(this, "stat: ", X, ## __VA_ARGS__)
#define LOG_EVENT(this, X, ...) LOGx(this, "event: ", X, ## __VA_ARGS__)
#define LOG_CHECK(this, X, ...) LOGx(this, "check: ", X, ## __VA_ARGS__)
#define LOG_VERBOSE(this, X, ...) LOGx(this, "verbose: ", X, ## __VA_ARGS__)

#define LOCK(this)			\
do {					\
	LOG_VERBOSE(this, "LOCK\n");	\
	mutex_lock(&this->lock);	\
} while (0)
#define UNLOCK(this)			\
do {					\
	mutex_unlock(&this->lock);	\
	LOG_VERBOSE(this, "UNLOCK\n");	\
} while (0)

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

static const char *state_name[] = {
	[SYN_STATE_INIT]		= "init",
	[SYN_STATE_RUNNING]		= "running",
	[SYN_STATE_FLASH_ENABLE]	= "flash enable",
	[SYN_STATE_FLASH_PROGRAM]	= "flash program",
	[SYN_STATE_FLASH_DATA]		= "flash data",
	[SYN_STATE_FLASH_CONFIG]	= "flash config",
	[SYN_STATE_FLASH_DISABLE]	= "flash disable",
	[SYN_STATE_DISABLED]		= "disabled",
};

enum synaptics_task {
	SYN_TASK_NONE,
	SYN_TASK_RESET,
	SYN_TASK_FLASH,
};

static const char *task_name[] = {
	[SYN_TASK_NONE]		= "none",
	[SYN_TASK_RESET]	= "reset",
	[SYN_TASK_FLASH]	= "flash",
};

enum synaptics_active {
	SYN_ACTIVE_POWER	= (1 << 0),
	SYN_STANDBY		= (1 << 1),
	SYN_STANDBY_AFTER_FLASH	= (1 << 2),
};

enum synaptics_irq_user {
	IRQ_FLASH_ENABLE	= (1 << 0),
	IRQ_SET_POWER		= (1 << 1),
	IRQ_ABS			= (1 << 2),
};

enum synaptics_clearpad_function {
	SYN_F01_RMI,
	SYN_F11_2D,
	SYN_F34_FLASH,
	SYN_N_FUNCTIONS,
};

enum synaptics_clearpad_function_value {
	SYN_F01_RMI_VALUE	= 0x01,
	SYN_F11_2D_VALUE	= 0x11,
	SYN_F34_FLASH_VALUE	= 0x34,
	SYN_END_OF_PDT		= 0x00,
};

enum synaptics_clearpad_reg_type {
	SYN_TYPE_DATA,
	SYN_TYPE_CTRL,
	SYN_TYPE_COMMAND,
	SYN_TYPE_QUERY,
	SYN_TYPE_END,
};

static const char *synaptics_clearpad_flash_status[] = {
	[0] = "Success",
	[1] = "(Reserved)",
	[2] = "Flash Programming Not Enabled/Bad Command",
	[3] = "Invalid Block Number",
	[4] = "Block Not Erased",
	[5] = "Erase Key Incorrect",
	[6] = "Unknown Erase/Program Failure",
	[7] = "Device has been reset",
};

static const char *synaptics_clearpad_flash_reason[] = {
	[4] = "Configuration CRC Failure",
	[5] = "Firmware CRC Failure",
	[6] = "CRC In Progress",
};

enum synaptics_clearpad_firmware {
	HEADER_SIZE			= 0x100,
	HEADER_VERSION_OFFSET		= 0x07,
	HEADER_FIRMWARE_SIZE_OFFSET	= 0x08,
	HEADER_FIRMWARE_SIZE_SIZE	= 4,
	HEADER_CONFIG_SIZE_OFFSET	= 0x0c,
	HEADER_CONFIG_SIZE_SIZE		= 4,
	HEADER_PRODUCT_ID_OFFSET	= 0x10,
	HEADER_PRODUCT_ID_SIZE		= 10,
	HEADER_CUSTOMER_FAMILY_OFFSET	= 0x1e,
	HEADER_FIRMWARE_REVISION_OFFSET	= 0x1f,
};

static const int synaptics_number_of_fingers[] = {
	[0] = 1,
	[1] = 2,
	[2] = 3,
	[3] = 4,
	[4] = 5,
	[5] = 10,
	[6] = 0,
	[7] = 0,
};

enum synaptics_device_serialization_queries {
	SIZE_OF_DATE_CODE	= 3,
	SIZE_OF_TESTER_ID	= 2,
	SIZE_OF_SERIAL_NUMBER	= 2,
};

enum synaptics_flush_commands {
	SYN_LOAD_START,
	SYN_LOAD_END,
	SYN_FORCE_FLUSH,
};

static const char *flush_commands_str[] = {
	[SYN_LOAD_START]	= "load_start",
	[SYN_LOAD_END]		= "load_end",
	[SYN_FORCE_FLUSH]	= "force_flush",
};

struct synaptics_device_info {
	u8 manufacturer_id;
	u8 product_properties;
	u8 customer_family;
	u8 firmware_revision;
	u8 date[SIZE_OF_DATE_CODE];
	u8 tester_id[SIZE_OF_TESTER_ID];
	u8 serial_number[SIZE_OF_SERIAL_NUMBER];
	u8 product_id[HEADER_PRODUCT_ID_SIZE];
};

struct synaptics_point {
	int x;
	int y;
};

struct synaptics_pointer {
	struct synaptics_funcarea *funcarea;
	struct synaptics_point cur;
};

struct synaptics_function_descriptor {
	u8 number;
	u8 int_count;
	u8 base[SYN_TYPE_END];
};

struct synaptics_flash_block {
	int blocks;
	int length;
	int pos;
	const u8 *data;
};

struct synaptics_flash_image {
	u8 *image;
	size_t size;
	size_t buffer_size; /* allocated buffer size */
	u8 format_version;
	int firmware_size;
	int config_size;
	u8 customer_family;
	u8 firmware_revision;
	u8 product_id[HEADER_PRODUCT_ID_SIZE];
	struct synaptics_flash_block data;
	struct synaptics_flash_block config;
};

struct synaptics_extents {
	int x_min, y_min;
	int x_max, y_max;
	int n_fingers;
};

struct synaptics_clearpad {
	enum   synaptics_state state;
	enum   synaptics_task task;
	struct input_dev *input;
	struct platform_device *pdev;
	struct clearpad_platform_data *pdata;
	struct clearpad_bus_data *bdata;
	struct mutex lock;
	struct work_struct work;
	struct synaptics_device_info device_info;
	struct synaptics_funcarea *funcarea;
	struct synaptics_pointer pointer[SYNAPTICS_MAX_N_FINGERS];
	struct synaptics_function_descriptor pdt[SYN_N_FUNCTIONS];
	struct synaptics_flash_image flash;
	bool fwdata_available;
	struct synaptics_extents extents;
	int active;
	int irq_mask;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	char fwname[SYNAPTICS_STRING_LENGTH + 1];
	char result_info[SYNAPTICS_STRING_LENGTH + 1];
	wait_queue_head_t task_none_wq;
	bool flash_requested;
};

static char *make_string(u8 *array, size_t size)
{
	static char string[SYNAPTICS_STRING_LENGTH + 1];

	memset(string, 0, SYNAPTICS_STRING_LENGTH + 1);
	size = (SYNAPTICS_STRING_LENGTH < size) ?
				SYNAPTICS_STRING_LENGTH : size;
	memcpy(string, array, size);

	return string;
}

static int regs_read(struct synaptics_clearpad *this, u8 reg, u8 *buf, int len)
{
	return this->bdata->read(this->pdev->dev.parent, reg, buf, len);
}

static int regs_write(struct synaptics_clearpad *this, u8 reg, const u8 *buf,
		u8 len)
{
	return this->bdata->write(this->pdev->dev.parent, reg, buf, len);
}

static int synaptics_put(struct synaptics_clearpad *this,
		enum synaptics_clearpad_function func,
		enum synaptics_clearpad_reg_type type,
		u8 addr, u8 val)
{
	return regs_write(this, SYN_ADDRESS(this, func, type, addr), &val, 1);
}

static int synaptics_write(struct synaptics_clearpad *this,
		enum synaptics_clearpad_function func,
		enum synaptics_clearpad_reg_type type,
		u8 addr, const u8 *buf, u8 len)
{
	return regs_write(this, SYN_ADDRESS(this, func, type, addr), buf, len);
}

static int synaptics_read(struct synaptics_clearpad *this,
		enum synaptics_clearpad_function func,
		enum synaptics_clearpad_reg_type type,
		u8 addr, u8 *buf, u8 len)
{
	return regs_read(this, SYN_ADDRESS(this, func, type, addr), buf, len);
}

static void synaptics_clearpad_set_irq(struct synaptics_clearpad *this,
					enum synaptics_irq_user user,
					bool enable)
{
	int mask;

	LOG_STAT(this, "user_mask 0x%02x user 0x%02x enable %d\n",
		 this->irq_mask, user, enable);

	if (enable)
		mask = this->irq_mask | user;
	else
		mask = this->irq_mask & ~user;

	if (mask && !this->irq_mask) {
		LOG_STAT(this, "enable IRQ (user_mask 0x%02x)\n", mask);
		enable_irq(this->pdata->irq);
	} else if (!mask && this->irq_mask) {
		LOG_STAT(this, "disable IRQ (user_mask 0x%02x)\n", mask);
		disable_irq(this->pdata->irq);
	} else
		LOG_STAT(this, "no change IRQ (%s)\n",
				mask ? "enable" : "disable");
	this->irq_mask = mask;
}

static int synaptics_clearpad_read_pdt(struct synaptics_clearpad *this)
{
	u8 addr = SYNAPTICS_PDT_START - 1;
	int i;
	int rc;

	for (i = 0; i < SYN_N_FUNCTIONS; ++i) {
		struct synaptics_function_descriptor fdes;
		rc = regs_read(this, addr--, &fdes.number, 1);
		if (rc)
			return rc;
		rc = regs_read(this, addr--, &fdes.int_count, 1);
		if (rc)
			return rc;
		rc = regs_read(this, addr--, &fdes.base[SYN_TYPE_DATA], 1);
		if (rc)
			return rc;
		rc = regs_read(this, addr--, &fdes.base[SYN_TYPE_CTRL], 1);
		if (rc)
			return rc;
		rc = regs_read(this, addr--, &fdes.base[SYN_TYPE_COMMAND], 1);
		if (rc)
			return rc;
		rc = regs_read(this, addr--, &fdes.base[SYN_TYPE_QUERY], 1);
		if (rc)
			return rc;
		LOG_VERBOSE(this, "F%02x_DATA = %02x\n",
			    fdes.number, fdes.base[SYN_TYPE_DATA]);
		LOG_VERBOSE(this, "F%02x_CTRL = %02x\n",
			    fdes.number, fdes.base[SYN_TYPE_CTRL]);
		LOG_VERBOSE(this, "F%02x_COMMAND = %02x\n",
			    fdes.number, fdes.base[SYN_TYPE_COMMAND]);
		LOG_VERBOSE(this, "F%02x_QUERY = %02x\n",
			    fdes.number, fdes.base[SYN_TYPE_QUERY]);
		switch (fdes.number) {
		case SYN_F01_RMI_VALUE:
			memcpy(&this->pdt[SYN_F01_RMI], &fdes,
				sizeof(struct synaptics_function_descriptor));
			break;
		case SYN_F11_2D_VALUE:
			memcpy(&this->pdt[SYN_F11_2D], &fdes,
				sizeof(struct synaptics_function_descriptor));
			break;
		case SYN_F34_FLASH_VALUE:
			memcpy(&this->pdt[SYN_F34_FLASH], &fdes,
				sizeof(struct synaptics_function_descriptor));
			break;
		/* Early end of page descriptor table */
		case SYN_END_OF_PDT:
			return rc;
		default:
			break;
		}
	}
	return rc;
}

static int synaptics_clearpad_read_extents(struct synaptics_clearpad *this)
{
	int rc;
	u8 buf[4];
	enum registers {
		REG_X_LSB,
		REG_X_MSB,
		REG_Y_LSB,
		REG_Y_MSB,
	};

	rc = synaptics_read(this, SYNF(F11_2D, CTRL, 0x06), buf, sizeof(buf));
	if (rc)
		return rc;

	this->extents.x_min = 0;
	this->extents.y_min = 0;
	this->extents.x_max = (buf[REG_X_LSB] | (buf[REG_X_MSB] << 8));
	this->extents.y_max = (buf[REG_Y_LSB] | (buf[REG_Y_MSB] << 8));

	rc = synaptics_read(this, SYNF(F11_2D, QUERY, 0x01), buf, 1);
	if (rc)
		return rc;

	this->extents.n_fingers = synaptics_number_of_fingers[buf[0] & 0x07];
	dev_info(&this->pdev->dev, "number of fingers=%d\n",
			this->extents.n_fingers);

	return rc;
}

static void synaptics_firmware_reset(struct synaptics_clearpad *this)
{
	kfree(this->flash.image);
	memset(&this->flash, 0, sizeof(this->flash));
	this->fwdata_available = false;
	dev_info(&this->pdev->dev, "firmware image has been reset\n");
}

static int synaptics_clearpad_flash(struct synaptics_clearpad *this);

static int synaptics_clearpad_initialize(struct synaptics_clearpad *this)
{
	int rc;
	struct synaptics_device_info *info = &this->device_info;
	u8 buf;

	rc = synaptics_clearpad_read_pdt(this);
	if (rc)
		return rc;

	/* set device configured bit */
	rc = synaptics_put(this, SYNF(F01_RMI, CTRL, 0x00),
					DEVICE_CONTROL_CONFIGURED);
	if (rc)
		return rc;

	/* read device configuration */
	rc = synaptics_read(this, SYNF(F01_RMI, QUERY, 0x00), (u8 *)info, 21);
	if (rc)
		return rc;

	if (this->state != SYN_STATE_RUNNING) {
		dev_info(&this->pdev->dev,
			"device mid %d, prop %d, family 0x%02x, rev 0x%02x\n",
			 info->manufacturer_id, info->product_properties,
			 info->customer_family, info->firmware_revision);
		dev_info(&this->pdev->dev,
			"bl %04d-%02d-%02d, tester %d, s/n %d, id '%s'\n",
			 2000 + info->date[0], info->date[1], info->date[2],
			 ((int)info->tester_id[0] << 8) + info->tester_id[1],
			 ((int)info->serial_number[0] << 8)
			 + info->serial_number[10],
			 make_string(info->product_id, HEADER_PRODUCT_ID_SIZE));
	}

	if (this->flash.image) {
		dev_info(&this->pdev->dev, "force firmware update\n");
		this->task = SYN_TASK_FLASH;
		rc = synaptics_clearpad_flash(this);
		return rc;
	}

	if (this->pdt[SYN_F11_2D].number != SYN_F11_2D_VALUE) {
		dev_info(&this->pdev->dev,
		       "no F11 (2D) function, device"
				" requires new firmware!\n");

		rc = 0; /* ignore fw problem */
		goto running;
	}

	rc = synaptics_clearpad_read_extents(this);
	if (rc)
		return rc;

	/* enable ABS event interrupts */
	rc = synaptics_put(this, SYNF(F01_RMI, CTRL, 0x01), IRQ_ABS);
	if (rc)
		return rc;

	/* set reduced reporting mode */
	rc = synaptics_read(this, SYNF(F11_2D, CTRL, 0x00), &buf, 1);
	if (rc)
		return rc;
	buf |= XY_REPORTING_MODE_REDUCED_REPORTING_MODE;
	rc = synaptics_put(this, SYNF(F11_2D, CTRL, 0x00), buf);
	if (rc)
		return rc;
running:
	this->state = SYN_STATE_RUNNING;

	/* no more firmware update */
	snprintf(this->result_info, sizeof(this->result_info),
		"%s, family 0x%02x, fw rev 0x%02x, (%s)\n",
		make_string(this->device_info.product_id,
			HEADER_PRODUCT_ID_SIZE),
		this->device_info.customer_family,
		this->device_info.firmware_revision,
		this->flash_requested ? "fw updated" : "no fw update");
	this->flash_requested = false;

	/* notify end of task */
	dev_info(&this->pdev->dev, "result: %s", this->result_info);
	wake_up_interruptible(&this->task_none_wq);

	return rc;
}

static irqreturn_t synaptics_clearpad_irq(int irq, void *dev_id);

static int synaptics_flash_enable(struct synaptics_clearpad *this)
{
	int rc;
	u8 buf[2];

	/* read bootloader id */
	rc = synaptics_read(this, SYNF(F34_FLASH, QUERY, 0x00),
			buf, sizeof(buf));
	LOG_CHECK(this, "rc=%d\n", rc);
	if (rc)
		return rc;

	/* write bootloader id to block data */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x02),
			buf, sizeof(buf));
	LOG_CHECK(this, "rc=%d\n", rc);
	if (rc)
		return rc;

	msleep(10);

	/* issue a flash program enable */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x12),
			FLASH_CONTROL_ENABLE_FLASH_PROGRAMMING);
	LOG_CHECK(this, "rc=%d\n", rc);
	if (rc)
		return rc;

	this->state = SYN_STATE_FLASH_ENABLE;
	msleep(100);

	LOCK(this);
	synaptics_clearpad_set_irq(this, IRQ_FLASH_ENABLE, true);
	UNLOCK(this);

	return rc;
}

static int synaptics_flash_program(struct synaptics_clearpad *this)
{
	int rc;
	u8 buf[2];

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), buf, 1);
	if (rc)
		return rc;
	if (buf[0] != FLASH_CONTROL_PROGRAM_ENABLED) {
		dev_err(&this->pdev->dev,
		       "failed enabling flash (%s)\n",
		       synaptics_clearpad_flash_status[(buf[0]>>4)&7]);
		return -EIO;
	}

	dev_info(&this->pdev->dev, "flashing enabled\n");

	/* PDT may have changed, re-read */
	rc = synaptics_clearpad_read_pdt(this);
	if (rc)
		return rc;

	/* read bootloader id */
	rc = synaptics_read(this, SYNF(F34_FLASH, QUERY, 0x00), buf, 2);
	if (rc)
		return rc;

	/* write bootloader id to block data */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x02), buf, 2);
	if (rc)
		return rc;

	msleep(10);

	/* issue a firmware and configuration erase */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x12),
					FLASH_CONTROL_ERASE_ALL);
	if (rc)
		return rc;

	dev_info(&this->pdev->dev, "firmware erasing\n");
	dev_info(&this->pdev->dev, "flashing data\n");

	this->state = SYN_STATE_FLASH_PROGRAM;
	return rc;
}


static int synaptics_flash_data(struct synaptics_clearpad *this)
{
	int rc, len;
	u8 buf;
	const u8 *data;
	struct synaptics_flash_image *f = &this->flash;

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), &buf, 1);
	if (rc)
		return rc;
	if (buf != FLASH_CONTROL_PROGRAM_ENABLED) {
		dev_err(&this->pdev->dev,
				"failed flashing data (%s)\n",
				synaptics_clearpad_flash_status[(buf>>4)&7]);
		return -EIO;
	}

	/* block # low byte */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x00),
			f->data.pos & 0xff);
	if (rc)
		return rc;

	/* block # high byte */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x01),
			(f->data.pos >> 8) & 0xff);
	if (rc)
		return rc;

	data = f->data.data + f->data.pos * 16;
	len = f->data.length - f->data.pos * 16;
	if (len > 16)
		len = 16;

	/* write block data */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x02), data, len);
	if (rc)
		return rc;

	msleep(10);

	/* issue a write data block command */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x12),
			FLASH_CONTROL_WRITE_FIRMWARE_BLOCK);
	if (rc)
		return rc;

	if (f->data.pos % 100 == 0)
		dev_info(&this->pdev->dev,
		       "wrote %d blocks\n", f->data.pos);

	/* if we've reached the end of the data flashing */
	if (++f->data.pos == f->data.blocks) {
		dev_info(&this->pdev->dev,
				"data flash finished\n");
		this->state = SYN_STATE_FLASH_DATA;
	}

	return rc;
}

static int synaptics_flash_config(struct synaptics_clearpad *this)
{
	int rc, len;
	u8 buf;
	const u8 *data;
	struct synaptics_flash_image *f = &this->flash;

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), &buf, 1);
	if (rc)
		return rc;
	if (buf != FLASH_CONTROL_PROGRAM_ENABLED) {
		dev_err(&this->pdev->dev,
				"failed flashing config (%s)\n",
				synaptics_clearpad_flash_status[(buf>>4)&7]);
		return -EIO;
	}

	/* block # low byte */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x00),
			f->config.pos & 0xff);
	if (rc)
		return rc;

	/* block # high byte */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x01),
			(f->config.pos >> 8) & 0xff);
	if (rc)
		return rc;

	data = f->config.data + f->config.pos * 16;
	len = f->config.length - f->config.pos * 16;
	if (len > 16)
		len = 16;

	/* write block data */
	rc = synaptics_write(this, SYNF(F34_FLASH, DATA, 0x02), data, len);
	if (rc)
		return rc;

	msleep(10);

	/* issue a write configuration block command */
	rc = synaptics_put(this, SYNF(F34_FLASH, DATA, 0x12),
		FLASH_CONTROL_WRITE_CONFIGURATION_BLOCK);
	if (rc)
		return rc;

	/* if we've reached the end of the configuration flashing */
	if (++f->config.pos == f->config.blocks) {
		dev_info(&this->pdev->dev,
				"configuration flash finished\n");
		this->state = SYN_STATE_FLASH_CONFIG;
	}

	return rc;
}


static int synaptics_flash_disable(struct synaptics_clearpad *this)
{
	int rc;
	u8 buf;

	/* make sure that we are in programming mode and there are no issues */
	rc = synaptics_read(this, SYNF(F34_FLASH, DATA, 0x12), &buf, 1);
	if (rc)
		return rc;
	if (buf != FLASH_CONTROL_PROGRAM_ENABLED) {
		dev_err(&this->pdev->dev,
				"failed flashing config (%s)\n",
				synaptics_clearpad_flash_status[(buf>>4)&7]);
		return -EIO;
	}

	msleep(10);

	/* send a reset to the device to complete the flash procedure */
	rc = synaptics_put(this, SYNF(F01_RMI, COMMAND, 0x00),
						DEVICE_COMMAND_RESET);
	if (rc)
		return rc;

	dev_info(&this->pdev->dev,
			"flashing finished, resetting\n");
	this->task = SYN_TASK_RESET;
	this->state = SYN_STATE_FLASH_DISABLE;
	msleep(100);
	return rc;
}

static int synaptics_flash_verify(struct synaptics_clearpad *this)
{
	int rc;
	u8 buf;

	/* make sure that we are no longer in programming mode */
	rc = synaptics_read(this, SYNF(F01_RMI, DATA, 0x00), &buf, 1);
	LOG_CHECK(this, "rc=%d F01_RMI_DATA0=0x%x\n", rc, buf);
	if (rc)
		return rc;
	if (buf & (1 << 6)) {
		dev_err(&this->pdev->dev,
				"failed disabling flash (%s)\n",
				synaptics_clearpad_flash_reason[buf & 0x0f]);
		return -EIO;
	}

	this->state = SYN_STATE_INIT;
	this->task = SYN_TASK_NONE;

	dev_info(&this->pdev->dev,
			"device successfully flashed\n");

	synaptics_clearpad_set_irq(this, IRQ_FLASH_ENABLE, false);

	rc = synaptics_clearpad_initialize(this);
	LOG_CHECK(this, "rc=%d\n", rc);
	return rc;
}

static void synaptics_firmware_check(struct synaptics_clearpad *this)
{
	const u8 *data;
	struct synaptics_flash_image *f = &this->flash;

	data = this->flash.image;

	/* Set up data block info */
	f->data.length = le32_to_cpu(*(u32 *)(data + 8));
	f->data.blocks = (f->data.length / 16) + !!(f->data.length % 16);
	f->data.data = data + HEADER_SIZE;
	f->data.pos = 0;
	dev_info(&this->pdev->dev, "DATA: length=%d blocks=%d data=%p\n",
		 f->data.length, f->data.blocks, f->data.data);

	/* Set up configuration block info */
	f->config.length = le32_to_cpu(*(u32 *)(data + 12));
	f->config.blocks = (f->config.length / 16) + !!(f->config.length % 16);
	f->config.data = data + HEADER_SIZE + f->data.length;
	f->config.pos = 0;
	dev_info(&this->pdev->dev, "CONFIG: length=%d blocks=%d data=%p\n",
		 f->config.length, f->config.blocks, f->config.data);
}

static int synaptics_clearpad_flash(struct synaptics_clearpad *this)
{
	int rc = 0;

	if (this->task != SYN_TASK_FLASH) {
		dev_err(&this->pdev->dev,
				"flash requested without reason\n");
		return -EBADE;
	}

	switch (this->state) {
	case SYN_STATE_INIT:
	case SYN_STATE_RUNNING:
		rc = synaptics_flash_enable(this);
		LOG_CHECK(this, "rc=%d\n", rc);
		break;
	case SYN_STATE_FLASH_ENABLE:
		rc = synaptics_flash_program(this);
		LOG_CHECK(this, "rc=%d\n", rc);
		break;
	case SYN_STATE_FLASH_PROGRAM:
		rc = synaptics_flash_data(this);
		break;
	case SYN_STATE_FLASH_DATA:
		rc = synaptics_flash_config(this);
		break;
	case SYN_STATE_FLASH_CONFIG:
		rc = synaptics_flash_disable(this);
		synaptics_firmware_reset(this);
		LOG_CHECK(this, "rc=%d\n", rc);
		break;
	case SYN_STATE_FLASH_DISABLE:
		rc = synaptics_flash_verify(this);
		LOG_CHECK(this, "rc=%d\n", rc);
		break;
	case SYN_STATE_DISABLED:
		LOG_CHECK(this, "rc=%d\n", rc);
		break;
	}

	if (rc) {
		dev_err(&this->pdev->dev,
				"failed during flash\n");
		this->state = SYN_STATE_DISABLED;
		synaptics_clearpad_set_irq(this, IRQ_FLASH_ENABLE, false);

		snprintf(this->result_info, SYNAPTICS_STRING_LENGTH,
			"%s, family 0x%02x, fw rev 0x%02x, failed fw update\n",
			make_string(this->device_info.product_id,
			HEADER_PRODUCT_ID_SIZE),
			 this->device_info.customer_family,
			 this->device_info.firmware_revision);
		this->flash_requested = false;
		synaptics_firmware_reset(this);

		/* check if standby was reserved */
		if (this->active & SYN_STANDBY_AFTER_FLASH) {
			this->active &= ~SYN_STANDBY_AFTER_FLASH;
			this->active |= SYN_STANDBY;

			LOG_STAT(this, "active: %x (task: %s)\n",
				 this->active, task_name[this->task]);
		}

		/* notify end of task */
		dev_info(&this->pdev->dev, "result: %s", this->result_info);
		wake_up_interruptible(&this->task_none_wq);
	}
	return rc;
}

static int synaptics_clearpad_set_power(struct synaptics_clearpad *this)
{
	int rc = 0;
	int active;
	bool should_wake;
	u8 irq;
	int users;

	LOCK(this);
	active = this->active;
	users = this->input ? this->input->users : 0;

	LOG_STAT(this, "powered %d, users %d, standby %d\n",
		 !!(active & SYN_ACTIVE_POWER),
		 users,
		 !!(active & SYN_STANDBY));

	if (this->state == SYN_STATE_DISABLED) {
		dev_err(&this->pdev->dev, "state == SYN_STATE_DISABLED\n");
		rc = -ENODEV;
		goto err_unlock;
	}
	should_wake = !(active & SYN_STANDBY) && users;

	if (should_wake && !(active & SYN_ACTIVE_POWER)) {

		dev_info(&this->pdev->dev, "power ON\n");

		synaptics_clearpad_set_irq(this, IRQ_SET_POWER, true);
		synaptics_read(this, SYNF(F01_RMI, DATA, 0x01), &irq, 1);

		rc = synaptics_put(this, SYNF(F01_RMI, CTRL, 0x00),
			DEVICE_CONTROL_SLEEP_MODE_NORMAL_OPERATION);
		if (rc) {
			dev_err(&this->pdev->dev,
			       "failed to exit sleep mode\n");
			goto err_unlock;
		}

		msleep(10);
		this->active |= SYN_ACTIVE_POWER;

	} else if (!should_wake && (active & SYN_ACTIVE_POWER)) {

		dev_info(&this->pdev->dev, "power OFF\n");

		rc = synaptics_put(this, SYNF(F01_RMI, CTRL, 0x00),
			DEVICE_CONTROL_SLEEP_MODE_SENSOR_SLEEP);
		if (rc) {
			dev_err(&this->pdev->dev,
			       "failed to enter sleep mode\n");
			goto err_unlock;
		}
		msleep(10); /* wait for last irq */
		LOG_CHECK(this, "enter sleep mode\n");
		synaptics_clearpad_set_irq(this, IRQ_SET_POWER, false);

		/* Lie to the listening applications, tell them that there
		 * are no fingers touching. */
		if (users) {
			input_mt_sync(this->input);
			input_sync(this->input);
		}

		this->active &= ~SYN_ACTIVE_POWER;
	} else {
		dev_info(&this->pdev->dev, "no change (%d)\n", should_wake);
	}
err_unlock:
	UNLOCK(this);
	return rc;
}

static void synaptics_funcarea_initialize(struct synaptics_clearpad *this)
{
	struct synaptics_funcarea *funcarea;
	struct synaptics_button *button;
	const char *func_name[] = {
		[SYN_FUNCAREA_INSENSIBLE] = "insensible",
		[SYN_FUNCAREA_POINTER] = "pointer",
		[SYN_FUNCAREA_BUTTON] = "button",
	};

	this->funcarea = this->pdata->funcarea;
	funcarea = this->funcarea;

	if (funcarea == NULL) {
		dev_info(&this->pdev->dev, "no funcarea\n");
		return;
	}

	for (; funcarea->func != SYN_FUNCAREA_END; funcarea++) {
		switch (funcarea->func) {
		case SYN_FUNCAREA_POINTER:
			input_set_abs_params(this->input, ABS_MT_POSITION_X,
					     funcarea->x1, funcarea->x2, 0, 0);
			input_set_abs_params(this->input, ABS_MT_POSITION_Y,
					     funcarea->y1, funcarea->y2, 0, 0);
			input_set_abs_params(this->input, ABS_MT_PRESSURE,
					     0, SYNAPTICS_MAX_Z_VALUE, 0, 0);
			input_set_abs_params(this->input, ABS_MT_TOUCH_MAJOR,
					0, SYNAPTICS_MAX_W_VALUE + 1, 0, 0);
			input_set_abs_params(this->input, ABS_MT_TOUCH_MINOR,
					0, SYNAPTICS_MAX_W_VALUE + 1, 0, 0);
			input_set_abs_params(this->input, ABS_MT_ORIENTATION,
					-1, 1, 0, 0);
			break;
		case SYN_FUNCAREA_BUTTON:
			button = (struct synaptics_button *)funcarea->data;
			input_set_capability(this->input,
					     button->type, button->code);
			break;
		default:
			continue;
		}

		dev_info(&this->pdev->dev, "funcarea '%s' [%d, %d, %d, %d]\n",
			 func_name[funcarea->func], funcarea->x1,
			 funcarea->y1, funcarea->x2, funcarea->y2);
	}
}

static inline bool synaptics_funcarea_test(struct synaptics_funcarea *funcarea,
					   struct synaptics_point *point)
{
	return (funcarea->x1 <= point->x && point->x <= funcarea->x2
		&& funcarea->y1 <= point->y && point->y <= funcarea->y2);
}

static struct synaptics_funcarea *
synaptics_funcarea_search(struct synaptics_clearpad *this,
			  struct synaptics_point *point)
{
	struct synaptics_funcarea *funcarea = this->funcarea;

	if (funcarea == NULL)
		return NULL;

	for ( ; funcarea->func != SYN_FUNCAREA_END; funcarea++) {
		if (synaptics_funcarea_test(funcarea, point))
			return funcarea;
	}

	return NULL;
}

static bool synaptics_funcarea_crop(struct synaptics_funcarea *funcarea,
				    struct synaptics_point *point)
{
	int n = 0;

	if (point->x < funcarea->x1)
		point->x = funcarea->x1;
	else if (funcarea->x2 < point->x)
		point->x = funcarea->x2;
	else
		n++;

	if (point->y < funcarea->y1)
		point->y = funcarea->y1;
	else if (funcarea->y2 < point->y)
		point->y = funcarea->y2;
	else
		n++;

	return n != 2;
}

static int synaptics_funcarea_up(struct synaptics_clearpad *,
				 struct synaptics_pointer *);

static int synaptics_funcarea_down(struct synaptics_clearpad *this,
				   struct synaptics_pointer *pointer,
				   int id, int x, int y, int wx, int wy, int z)
{
	int touch_major, touch_minor;
	struct synaptics_funcarea *funcarea = this->funcarea;
	struct synaptics_button *button;
	struct synaptics_pointer previous_pointer;
	previous_pointer.funcarea = pointer->funcarea;

	pointer->cur.x = x;
	pointer->cur.y = y;
	pointer->funcarea
		= synaptics_funcarea_search(this, &pointer->cur);

	if (previous_pointer.funcarea)
		if (SYN_FUNCAREA_BTN_INBOUND == previous_pointer.funcarea->func)
			if (!synaptics_funcarea_test(previous_pointer.funcarea,
								&pointer->cur))
				synaptics_funcarea_up(this, &previous_pointer);

	if (pointer->funcarea == NULL)
		return 0;

	switch (pointer->funcarea->func) {
	case SYN_FUNCAREA_INSENSIBLE:
		LOG_EVENT(this, "insensible");
		return 0;
	case SYN_FUNCAREA_BOTTOM_EDGE:
		LOG_EVENT(this, "bottom edge\n");
		for (; funcarea->func != SYN_FUNCAREA_END; funcarea++) {
			if (funcarea->func == SYN_FUNCAREA_POINTER) {
				pointer->funcarea = funcarea;
				break;
			}
		}
	case SYN_FUNCAREA_POINTER:
		synaptics_funcarea_crop(pointer->funcarea, &pointer->cur);
		LOG_EVENT(this, "pointer %d (x,y)=(%d,%d) w=(%d,%d) z=%d\n",
			  id, pointer->cur.x, pointer->cur.y, wx, wy, z);
		touch_major = max(wx, wy) + 1;
		touch_minor = min(wx, wy) + 1;
		input_report_abs(this->input, ABS_MT_TRACKING_ID, id);
		input_report_abs(this->input, ABS_MT_POSITION_X,
				 pointer->cur.x);
		input_report_abs(this->input, ABS_MT_POSITION_Y,
				 pointer->cur.y);
		input_report_abs(this->input, ABS_MT_PRESSURE, z);
		input_report_abs(this->input, ABS_MT_TOUCH_MAJOR, touch_major);
		input_report_abs(this->input, ABS_MT_TOUCH_MINOR, touch_minor);
		input_report_abs(this->input, ABS_MT_ORIENTATION, (wx > wy));
		input_mt_sync(this->input);
		return 1;
	case SYN_FUNCAREA_BUTTON:
		LOG_EVENT(this, "button");
		button = (struct synaptics_button *)pointer->funcarea->data;
		if (button)
			button->down = true;
		return 0;
	default:
		break;
	}

	return 0;
}

static int synaptics_funcarea_up(struct synaptics_clearpad *this,
				 struct synaptics_pointer *pointer)
{
	struct synaptics_button *button;

	if (pointer->funcarea == NULL)
		return 0;

	switch (pointer->funcarea->func) {
	case SYN_FUNCAREA_INSENSIBLE:
	case SYN_FUNCAREA_POINTER:
		break;
	case SYN_FUNCAREA_BUTTON:
	case SYN_FUNCAREA_BTN_INBOUND:
		LOG_EVENT(this, "button up\n");
		button = (struct synaptics_button *)pointer->funcarea->data;
		if (button)
			button->down = false;
		break;
	default:
		break;
	}

	pointer->funcarea = NULL;

	return 0;
}

static void synaptics_report_button(struct synaptics_clearpad *this,
		struct synaptics_button *button)
{
	if (button->down) {
		if (!button->down_report) {
			button->down_report = true;
			input_report_key(this->input, button->code, 1);
			LOG_EVENT(this, "key(%d): down\n", button->code);
		}
	} else {
		if (button->down_report) {
			button->down_report = false;
			input_report_key(this->input, button->code, 0);
			LOG_EVENT(this, "key(%d): up\n", button->code);
		}
	}
}

static void
synaptics_funcarea_report_extra_events(struct synaptics_clearpad *this)
{
	struct synaptics_funcarea *funcarea = this->funcarea;
	struct synaptics_button *button;

	if (funcarea == NULL)
		return;

	for (; funcarea->func != SYN_FUNCAREA_END; funcarea++) {
		if (funcarea->func == SYN_FUNCAREA_BUTTON ||
			funcarea->func == SYN_FUNCAREA_BTN_INBOUND) {
			button = (struct synaptics_button *)funcarea->data;
			if (button)
				synaptics_report_button(this, button);
		}
	}
}

static int synaptics_report_finger_n(struct synaptics_clearpad *this,
		int finger, u8 *buf)
{
	enum registers {
		REG_X_MSB,
		REG_Y_MSB,
		REG_XY_LSB,
		REG_XY_W,
		REG_Z,
	};
	struct synaptics_pointer *pointer = &this->pointer[finger];

	/* check finger state */
	if (buf[finger >> 2] & (0x03 << ((finger % 4) << 1))) {
		int id, x, y, wy, wx, z;

		buf += SYNAPTICS_FINGER_OFF(this->extents.n_fingers, finger);
		id = finger;
		x = ((buf[REG_X_MSB] << 4) | ((buf[REG_XY_LSB] & 0x0f)));
		y = ((buf[REG_Y_MSB] << 4) | ((buf[REG_XY_LSB] & 0xf0) >> 4));
		wx = (buf[REG_XY_W] & 0x0f);
		wy = ((buf[REG_XY_W] >> 4) & 0x0f);
		z = buf[REG_Z];

		return synaptics_funcarea_down(this, pointer,
					       id, x, y, wx, wy, z);
	} else
		return synaptics_funcarea_up(this, pointer);
}

static void synaptics_clearpad_worker(struct work_struct *work)
{
	struct synaptics_clearpad *this;
	int count, rc, i;
	u8 buf[SYNAPTICS_REG_MAX];
	u8 status;
	u8 interrupt;

	this = container_of(work, struct synaptics_clearpad, work);

	LOCK(this);

	if (this->task == SYN_TASK_RESET) {
		/* first ATTN after reset on reprogrammable clearpad devices
		 * indicates bootloader start. we should ignore this */
			LOG_CHECK(this, "RESET\n");

		for (i = 0; i < SYNAPTICS_RETRY_NUM_OF_INITIAL_CHECK; i++) {
			rc = synaptics_read(this,
					SYNF(F01_RMI, DATA, 0x00), buf, 1);
			LOG_CHECK(this, "rc=%d F01_RMI_DATA0=0x%x\n",
								rc, buf[0]);
			if (rc)
				goto err_bus;

			if (DEVICE_STATUS_UNCONFIGURED_RESET_OCCURRED
								== buf[0]) {
				break;
			} else {
				dev_info(&this->pdev->dev,
			       "initial check failed: retry_num = %d\n", i);
				msleep(100);
			}
		}
	}

	count = 0;
	do {
		rc = synaptics_read(this, SYNF(F01_RMI, DATA, 0x01),
				    &interrupt, 1);
		LOG_CHECK(this, "rc=%d F01_RMI_DATA1=0x%x\n", rc, interrupt);
		if (rc)
			goto err_bus;
	} while (!interrupt && ++count < 10);

	if ((interrupt & IRQ_FLASH_ENABLE)) {
		if (this->task != SYN_TASK_FLASH)
			dev_err(&this->pdev->dev,
					"flash bit unexpectedly set\n");
		else
			synaptics_clearpad_flash(this);
		UNLOCK(this);
		return;
	}

	if ((interrupt & IRQ_SET_POWER)) {
		rc = synaptics_read(this, SYNF(F01_RMI, DATA, 0x00),
				    &status, 1);
		if (rc)
			goto err_bus;
		LOG_VERBOSE(this, "status = %02x\n", status);
		if (status & 1) {
			dev_info(&this->pdev->dev, "device reset\n");
			if (this->state == SYN_STATE_FLASH_DISABLE)
				synaptics_flash_verify(this);
			else
				synaptics_clearpad_initialize(this);
		}

		rc = synaptics_read(this, SYNF(F01_RMI, DATA, 0x01),
				    &interrupt, 1);
		if (rc)
			goto err_bus;
	}

	if (!(interrupt & IRQ_ABS)) {
		LOG_VERBOSE(this, "no work\n");
		rc = synaptics_read(this, SYNF(F01_RMI, DATA, 0x01),
				    &interrupt, 1);
		UNLOCK(this);
		return;
	}

	/* worker trails ioctl, ignore last event */
	if (!(this->active & SYN_ACTIVE_POWER)) {
		dev_info(&this->pdev->dev,
				"late irq, dropping events\n");
		UNLOCK(this);
		return;
	}

	rc = synaptics_read(this, SYNF(F11_2D, DATA, 0x35), &status, 1);
	LOG_CHECK(this, "rc=%d F11_2D_DATA09=0x%x\n", rc, status);
	if (rc)
		goto err_bus;

	rc = synaptics_read(this, SYNF(F11_2D, DATA, 0x00), buf,
			    SYNAPTICS_FINGER_OFF(this->extents.n_fingers,
						 this->extents.n_fingers));
	if (rc)
		goto err_bus;

	for (count = i = 0; i < this->extents.n_fingers; ++i)
		count += synaptics_report_finger_n(this, i, buf);

	/* if no fingers were pressed, we need to output a MT sync so that the
	 * userspace can identify when the last finger has been removed from
	 * the device */
	if (!count)
		input_mt_sync(this->input);

	synaptics_funcarea_report_extra_events(this);

	input_sync(this->input);

	UNLOCK(this);
	return;

 err_bus:
	UNLOCK(this);
	dev_err(&this->pdev->dev, "read error\n");
	return;
}

static irqreturn_t synaptics_clearpad_irq(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct synaptics_clearpad *this = dev_get_drvdata(dev);

	if (this)
		schedule_work(&this->work);

	return IRQ_HANDLED;
}

static int synaptics_clearpad_device_open(struct input_dev *dev)
{
	struct synaptics_clearpad *this = input_get_drvdata(dev);

	LOG_STAT(this, "state=%s\n", state_name[this->state]);

	if (this->state == SYN_STATE_DISABLED)
		return -ENODEV;
	if (this->state != SYN_STATE_RUNNING)
		return -EBUSY;

	return synaptics_clearpad_set_power(this);
}

static void synaptics_clearpad_device_close(struct input_dev *dev)
{
	struct synaptics_clearpad *this = input_get_drvdata(dev);

	LOG_STAT(this, "state=%s\n", state_name[this->state]);

	(void)synaptics_clearpad_set_power(this);
}

static int synaptics_clearpad_command_open(struct synaptics_clearpad *this,
		size_t image_size)
{
	int rc = 0;
	LOCK(this);
	/* allocate image buffer */
	this->flash.image = kmalloc(image_size, GFP_KERNEL);
	if (this->flash.image == NULL) {
		dev_err(&this->pdev->dev,
		       "buffer allocation error (%d bytes)\n", image_size);
		rc = -ENOMEM;
	} else {
		this->flash.buffer_size = image_size;
		dev_info(&this->pdev->dev,
			"prepared buffer size=%d\n", this->flash.buffer_size);
	}
	UNLOCK(this);
	return rc;
}

static ssize_t synaptics_clearpad_fwdata_write(struct kobject *kobj,
		struct bin_attribute *bin_attr,
		char *buf, loff_t pos, size_t size)
{
	int rc;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct synaptics_clearpad *this = dev_get_drvdata(dev);

	if (!this->flash.image) {
		size_t image_size;

		if (size < HEADER_SIZE) {
			dev_err(&this->pdev->dev, "invalid firmware size");
			return -EINVAL;
		}
		this->flash.format_version = buf[HEADER_VERSION_OFFSET];
		memcpy(&(this->flash.firmware_size),
				&buf[HEADER_FIRMWARE_SIZE_OFFSET],
				HEADER_FIRMWARE_SIZE_SIZE);
		memcpy(&(this->flash.config_size),
				&buf[HEADER_CONFIG_SIZE_OFFSET],
				HEADER_CONFIG_SIZE_SIZE);
		memcpy(this->flash.product_id,
				&buf[HEADER_PRODUCT_ID_OFFSET],
				HEADER_PRODUCT_ID_SIZE);
		this->flash.customer_family =
				buf[HEADER_CUSTOMER_FAMILY_OFFSET];
		this->flash.firmware_revision =
				buf[HEADER_FIRMWARE_REVISION_OFFSET];
		image_size = this->flash.firmware_size
				+ this->flash.config_size
				+ HEADER_SIZE;
		dev_info(&this->pdev->dev,
				"firmware_size=%d\n",
				this->flash.firmware_size);
		dev_info(&this->pdev->dev,
				"config_size=%d\n",
				this->flash.config_size);
		dev_info(&this->pdev->dev,
				"image_size=%d\n", image_size);
		rc = synaptics_clearpad_command_open(this, image_size);
		if (rc)
			return -EINVAL;
	}

	if (this->flash.size + size > this->flash.buffer_size) {
		dev_err(&this->pdev->dev,
		       "firmware buffer is too small\n");
		return -ENOMEM;
	}

	LOCK(this);
	memcpy(this->flash.image + this->flash.size, buf, size);
	this->flash.size += size;
	dev_info(&this->pdev->dev,
		"got %d bytes, total %d bytes\n", size, this->flash.size);
	UNLOCK(this);
	return size;
}

static struct bin_attribute synaptics_clearpad_fwdata = {
	.attr = {
		.name = "fwdata",
		.mode = 0600,
	},
	.size = 4096,
	.write = synaptics_clearpad_fwdata_write
};

static int synaptics_clearpad_check_task(struct synaptics_clearpad *this,
			   enum synaptics_state *state,
			   bool keeplock)
{
	LOCK(this);
	*state = this->state;
	if (*state == SYN_STATE_RUNNING || *state == SYN_STATE_DISABLED) {
		if (!keeplock)
			UNLOCK(this);
		return 1;
	}
	UNLOCK(this);

	return 0;
}

static int synaptics_clearpad_command_fw_load_start(
					struct synaptics_clearpad *this)
{
	int rc;
	if (this->flash.image == NULL) {
		synaptics_firmware_reset(this);
		rc = sysfs_create_bin_file(&this->input->dev.kobj,
				&synaptics_clearpad_fwdata);
		if (rc) {
			dev_err(&this->pdev->dev,
					"failed to create fwdata\n");
		}
	} else {
		dev_err(&this->pdev->dev,
				"flash.image allready exists\n");
		synaptics_firmware_reset(this);
		rc = -EINVAL;
	}
	return rc;
}

static int synaptics_clearpad_command_fw_flash(struct synaptics_clearpad *this)
{
	enum   synaptics_state state;
	int rc;

	if (!this->fwdata_available) {
		dev_err(&this->pdev->dev,
				"fwdata_available is not ready yet\n");
		rc = -EINVAL;
		goto error;
	}
	if (wait_event_interruptible(this->task_none_wq,
			synaptics_clearpad_check_task(this, &state, true))) {
		rc = -ERESTARTSYS;
		goto error;
	}

	memset(this->result_info, 0, SYNAPTICS_STRING_LENGTH);
	this->flash_requested = true;

	synaptics_firmware_check(this);

	if (this->active & SYN_STANDBY) {
		/* wake up during flashing */
		this->active &= ~SYN_STANDBY;
		this->active |= SYN_STANDBY_AFTER_FLASH;
		LOG_STAT(this, "active: %x (task: %s)\n",
			 this->active, task_name[this->task]);
	}

	UNLOCK(this);

	/* wake up */
	rc = synaptics_clearpad_set_power(this);
	if (rc)
		goto error;

	rc = synaptics_clearpad_initialize(this);
	if (rc)
		goto error;

	/* wait for end of flash */
	if (wait_event_interruptible(this->task_none_wq,
			synaptics_clearpad_check_task(this, &state, true))) {
		rc = -ERESTARTSYS;
		goto error;
	}

	/* check if standby was reserved */
	if (this->active & SYN_STANDBY_AFTER_FLASH) {
		this->active &= ~SYN_STANDBY_AFTER_FLASH;
		this->active |= SYN_STANDBY;
		LOG_STAT(this, "active: %x (task: %s)\n",
			 this->active, task_name[this->task]);
	}
	UNLOCK(this);

	/* restore previous state */
	rc = synaptics_clearpad_set_power(this);
	if (rc)
		goto error;
	return rc;

error:
	snprintf(this->result_info, SYNAPTICS_STRING_LENGTH,
		"%s, family %d, fw rev %d, failed fw update\n",
		make_string(this->device_info.product_id,
					HEADER_PRODUCT_ID_SIZE),
		 this->device_info.customer_family,
		 this->device_info.firmware_revision);
	LOCK(this);
	this->flash_requested = false;
	synaptics_firmware_reset(this);
	dev_info(&this->pdev->dev, "not started: %s", this->result_info);
	UNLOCK(this);
	return rc;
}

static int synaptics_clearpad_command_fw_load_end(
					struct synaptics_clearpad *this)
{
	int rc;

	if (!this->flash.image) {
		dev_err(&this->pdev->dev,
				"loading firmware is not started yet\n");
		rc = -EINVAL;
	} else if (this->flash.size == this->flash.buffer_size) {
		this->fwdata_available = true;
		rc = 0;
	} else {
		dev_err(&this->pdev->dev,
				"loading firmware is not finished yet\n");
		synaptics_firmware_reset(this);
		rc = -EINVAL;
	}
	sysfs_remove_bin_file(&this->input->dev.kobj,
			&synaptics_clearpad_fwdata);
	return rc;
}

static ssize_t synaptics_clearpad_state_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct synaptics_clearpad *this = dev_get_drvdata(dev);

	if (!strcmp(attr->attr.name, __stringify(fwinfo)))
		snprintf(buf, PAGE_SIZE,
			"%s, family 0x%02x, fw rev 0x%02x, task=%s, state=%s\n",
			make_string(this->device_info.product_id,
				HEADER_PRODUCT_ID_SIZE),
			this->device_info.customer_family,
			this->device_info.firmware_revision,
			task_name[this->task], state_name[this->state]);
	else if (!strcmp(attr->attr.name, __stringify(fwfamily)))
		snprintf(buf, PAGE_SIZE,
			"%x", this->device_info.customer_family);
	else if (!strcmp(attr->attr.name, __stringify(fwrevision)))
		snprintf(buf, PAGE_SIZE,
			"%x", this->device_info.firmware_revision);
	else if (!strcmp(attr->attr.name, __stringify(fwtask)))
		snprintf(buf, PAGE_SIZE,
			"%s", task_name[this->task]);
	else if (!strcmp(attr->attr.name, __stringify(fwstate)))
		snprintf(buf, PAGE_SIZE,
			"%s", state_name[this->state]);
	else
		snprintf(buf, PAGE_SIZE, "illegal sysfs file");
	return strlen(buf);
}

static ssize_t synaptics_clearpad_fwflush_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t size)
{
	struct synaptics_clearpad *this = dev_get_drvdata(dev);
	int rc;

	dev_info(&this->pdev->dev, "flush command: %s\n", buf);

	if (strncmp(buf, flush_commands_str[SYN_LOAD_START],
		strlen(flush_commands_str[SYN_LOAD_START])) == 0) {
		rc = synaptics_clearpad_command_fw_load_start(this);
	} else if (strncmp(buf, flush_commands_str[SYN_LOAD_END],
		strlen(flush_commands_str[SYN_LOAD_END])) == 0) {
		rc = synaptics_clearpad_command_fw_load_end(this);
	} else if (strncmp(buf, flush_commands_str[SYN_FORCE_FLUSH],
		strlen(flush_commands_str[SYN_FORCE_FLUSH])) == 0) {
		rc = synaptics_clearpad_command_fw_flash(this);
	} else {
		dev_err(&this->pdev->dev, "illegal command\n");
		rc = -EINVAL;
	}
	if (rc)
		dev_err(&this->pdev->dev, "%s failed\n", __func__);
	return strlen(buf);
}

static ssize_t synaptics_clearpad_hwtest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct synaptics_clearpad *this = dev_get_drvdata(dev);
	int rc;
	const char *str_num;
	unsigned long arg;
	u8 reg, value;

	if (strlen(buf) <= HWTEST_SIZE_OF_COMMAND_PREFIX)
		goto err_invalid_arg;

	str_num = buf + HWTEST_SIZE_OF_COMMAND_PREFIX;
	rc = strict_strtoul(str_num, 16, &arg);
	if (rc)
		goto err_invalid_arg;

	switch (buf[0]) {
	case 'R':
		reg = arg;
		rc = regs_read(this, reg, &value, 1);
		if (!rc)
			dev_info(&this->pdev->dev,
					"read addr=0x%02x, value=0x%02x\n",
					reg, value);
		break;
	case 'W':
		value = arg;
		reg = arg >> 8;
		rc = regs_write(this, reg, &value, 1);
		if (!rc)
			dev_info(&this->pdev->dev,
					"write addr=0x%02x, value=0x%02x\n",
					reg, value);
		break;
	default:
		break;
	}
	return strlen(buf);
err_invalid_arg:
	dev_err(&this->pdev->dev, "illegal command\n");
	return -EINVAL;
}

static DEVICE_ATTR(fwinfo, 0600, synaptics_clearpad_state_show, 0);
static DEVICE_ATTR(fwfamily, 0600, synaptics_clearpad_state_show, 0);
static DEVICE_ATTR(fwrevision, 0604, synaptics_clearpad_state_show, 0);
static DEVICE_ATTR(fwtask, 0600, synaptics_clearpad_state_show, 0);
static DEVICE_ATTR(fwstate, 0600, synaptics_clearpad_state_show, 0);
static DEVICE_ATTR(fwflush, 0600, 0, synaptics_clearpad_fwflush_store);
static DEVICE_ATTR(hwtest, 0600, 0, synaptics_clearpad_hwtest_store);

static struct attribute *synaptics_clearpad_attributes[] = {
	&dev_attr_fwinfo.attr,
	&dev_attr_fwfamily.attr,
	&dev_attr_fwrevision.attr,
	&dev_attr_fwtask.attr,
	&dev_attr_fwstate.attr,
	&dev_attr_fwflush.attr,
	&dev_attr_hwtest.attr,
	NULL
};

static const struct attribute_group synaptics_clearpad_attrs = {
	.attrs = synaptics_clearpad_attributes
};

static int synaptics_clearpad_input_init(struct synaptics_clearpad *this)
{
	int rc;

	this->input = input_allocate_device();
	if (!this->input)
		return -ENOMEM;

	input_set_drvdata(this->input, this);

	this->input->open = synaptics_clearpad_device_open;
	this->input->close = synaptics_clearpad_device_close;
	this->input->name = CLEARPAD_NAME;
	this->input->id.vendor = SYNAPTICS_CLEARPAD_VENDOR;
	this->input->id.product = 1;
	this->input->id.version = 1;
	this->input->id.bustype = this->bdata->bustype;
	set_bit(EV_ABS, this->input->evbit);

	set_bit(ABS_MT_TRACKING_ID, this->input->absbit);
	set_bit(ABS_MT_ORIENTATION, this->input->absbit);
	set_bit(ABS_MT_PRESSURE, this->input->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, this->input->absbit);
	set_bit(ABS_MT_TOUCH_MINOR, this->input->absbit);

	dev_info(&this->pdev->dev, "Touch area [%d, %d, %d, %d]\n",
		 this->extents.x_min, this->extents.y_min,
		 this->extents.x_max, this->extents.y_max);

	synaptics_funcarea_initialize(this);

	rc = input_register_device(this->input);
	if (rc) {
		dev_err(&this->pdev->dev,
		       "failed to register device\n");
		input_set_drvdata(this->input, NULL);
		input_free_device(this->input);
	}

	return rc;
}

static int synaptics_clearpad_pm_suspend(struct device *dev)
{
	struct synaptics_clearpad *this = dev_get_drvdata(dev);
	int rc = 0;
	bool go_suspend;

	LOCK(this);
	go_suspend = (this->task != SYN_TASK_FLASH);
	if (go_suspend)
		this->active |= SYN_STANDBY;
	else
		this->active |= SYN_STANDBY_AFTER_FLASH;

	LOG_STAT(this, "active: %x (task: %s)\n",
		 this->active, task_name[this->task]);
	UNLOCK(this);

	rc = synaptics_clearpad_set_power(this);
	return rc;
}

static int synaptics_clearpad_pm_resume(struct device *dev)
{
	struct synaptics_clearpad *this = dev_get_drvdata(dev);
	int rc = 0;
	bool go_resume;

	LOCK(this);
	go_resume = !!(this->active & (SYN_STANDBY | SYN_STANDBY_AFTER_FLASH));
	if (go_resume)
		this->active &= ~(SYN_STANDBY | SYN_STANDBY_AFTER_FLASH);

	LOG_STAT(this, "active: %x (task: %s)\n",
		 this->active, task_name[this->task]);
	UNLOCK(this);

	rc = synaptics_clearpad_set_power(this);
	return rc;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_clearpad_early_suspend(struct early_suspend *handler)
{
	struct synaptics_clearpad *this =
	container_of(handler, struct synaptics_clearpad, early_suspend);

	dev_info(&this->pdev->dev, "early suspend\n");
	synaptics_clearpad_pm_suspend(&this->pdev->dev);
}

static void synaptics_clearpad_late_resume(struct early_suspend *handler)
{
	struct synaptics_clearpad *this =
	container_of(handler, struct synaptics_clearpad, early_suspend);

	dev_info(&this->pdev->dev, "late resume\n");
	synaptics_clearpad_pm_resume(&this->pdev->dev);
}
#endif

static int __devinit clearpad_probe(struct platform_device *pdev)
{
	struct clearpad_data *cdata = pdev->dev.platform_data;
	struct synaptics_clearpad *this;
	int rc;
	int i;

	this = kzalloc(sizeof(struct synaptics_clearpad), GFP_KERNEL);
	if (!this)
		return -ENOMEM;

	mutex_init(&this->lock);
	INIT_WORK(&this->work, synaptics_clearpad_worker);
	init_waitqueue_head(&this->task_none_wq);

	dev_set_drvdata(&pdev->dev, this);
	this->pdev = pdev;
	this->pdata = cdata->pdata;
	if (!this->pdata) {
		dev_err(&this->pdev->dev, "no platform data\n");
		rc = -EINVAL;
		goto err_free;
	}
	this->bdata = cdata->bdata;
	if (!this->bdata) {
		dev_err(&this->pdev->dev, "no bus data\n");
		rc = -EINVAL;
		goto err_free;
	}

	if (this->pdata->gpio_configure) {
		rc = this->pdata->gpio_configure(1);
		if (rc) {
			dev_err(&this->pdev->dev,
			       "failed gpio init\n");
			goto err_free;
		}
	}

	msleep(400);

	this->active = SYN_ACTIVE_POWER;

	rc = synaptics_clearpad_input_init(this);
	if (rc)
		goto err_gpio_teardown;

	for (i = 0; i < SYNAPTICS_RETRY_NUM_OF_INITIAL_CHECK; i++) {
		u8 buf = 0;
		rc = regs_read(this, SYNAPTICS_DEVICE_STATUS, &buf, 1);
		if (rc)
			goto err_input_unregister;
		if (DEVICE_STATUS_UNCONFIGURED_RESET_OCCURRED == buf) {
			rc = synaptics_clearpad_initialize(this);
			if (rc) {
				dev_err(&this->pdev->dev,
						"failed initialization\n");
				goto err_input_unregister;
			}
			break;
		} else {
			dev_info(&this->pdev->dev,
					"initial check fail: retry = %d\n", i);
			msleep(100);
		}
	}
	this->state = SYN_STATE_RUNNING;

	rc = synaptics_clearpad_set_power(this);
	if (rc)
		goto err_input_unregister;

#ifdef CONFIG_HAS_EARLYSUSPEND
	this->early_suspend.suspend = synaptics_clearpad_early_suspend;
	this->early_suspend.resume = synaptics_clearpad_late_resume;
	register_early_suspend(&this->early_suspend);
#endif

	/* sysfs */
	rc = sysfs_create_group(&this->input->dev.kobj,
				&synaptics_clearpad_attrs);
	if (rc)
		goto err_input_unregister;

	LOCK(this);
	rc = request_irq(this->pdata->irq, &synaptics_clearpad_irq,
			IRQF_TRIGGER_FALLING,
			this->pdev->dev.driver->name,
			&this->pdev->dev);
	if (rc) {
		dev_err(&this->pdev->dev,
		       "irq %d busy?\n", this->pdata->irq);
		UNLOCK(this);
		goto err_input_unregister;
	}
	disable_irq(this->pdata->irq);
	UNLOCK(this);

	return 0;

err_input_unregister:
	input_set_drvdata(this->input, NULL);
	input_unregister_device(this->input);
err_gpio_teardown:
	if (this->pdata->gpio_configure)
		this->pdata->gpio_configure(0);
err_free:
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(this);
	return rc;
}

static int __devexit clearpad_remove(struct platform_device *pdev)
{
	struct synaptics_clearpad *this = dev_get_drvdata(&pdev->dev);

	free_irq(this->pdata->irq, &this->pdev->dev);
	unregister_early_suspend(&this->early_suspend);
	input_unregister_device(this->input);
	sysfs_remove_group(&this->input->dev.kobj, &synaptics_clearpad_attrs);

	if (this->pdata->gpio_configure)
		this->pdata->gpio_configure(0);

	kfree(this);

	return 0;
}

static void clearpad_shutdown(struct platform_device *pdev)
{
	struct synaptics_clearpad *this = dev_get_drvdata(&pdev->dev);

	if (this->pdata->vreg_off)
		this->pdata->vreg_off();
}

static const struct dev_pm_ops synaptics_clearpad_pm = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = synaptics_clearpad_pm_suspend,
	.resume = synaptics_clearpad_pm_resume,
#endif
};

static struct platform_driver clearpad_driver = {
	.driver = {
		.name	= CLEARPAD_NAME,
		.owner	= THIS_MODULE,
		.pm	= &synaptics_clearpad_pm,
	},
	.probe		= clearpad_probe,
	.remove		= __devexit_p(clearpad_remove),
	.shutdown	= clearpad_shutdown
};

static int __init clearpad_init(void)
{
	return platform_driver_register(&clearpad_driver);
}

static void __exit clearpad_exit(void)
{
	platform_driver_unregister(&clearpad_driver);
}

module_init(clearpad_init);
module_exit(clearpad_exit);

MODULE_DESCRIPTION(CLEARPAD_NAME "ClearPad Driver");
MODULE_LICENSE("GPL v2");
