/* linux/drivers/input/touchscreen/cy8ctma300_spi.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * Author: Kenji Tokutake <Kenji.Tokutake@SonyEricsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


/*#define DEBUG*/
#define MODULE_VER	"1.00"

#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/cy8ctma300_touch.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/timer.h>

#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_ARM
#include <asm/mach-types.h>
#endif

/*--------------------------------------------------------------------------*/
/* Operating Mode Register Address */
/* touch data (R) */
#define TP_REG_NINT_TMO_FLAG		0x00
#define TP_REG_TOUCH_CTR		0x00
#define TP_REG_FNGR_CNT			0x00
#define TP_REG_FNGR_STAT		0x01
#define TP_REG_FNGR_ID			0x03
#define TP_REG_FNGR_XH			0x04
#define TP_REG_FNGR_XL			0x05
#define TP_REG_FNGR_YH			0x06
#define TP_REG_FNGR_YL			0x07
#define TP_REG_FNGR_Z			0x08

/* FW info (R) */
#define TP_REG_FW_VER			0x2F
#define TP_REG_PROJ_ID			0x1F

/* auto power control */
#define TP_REG_L3_SLEEP			0x1B
#define TP_REG_L2_SLEEP			0x1C
#define TP_REG_L1_SLEEP			0x1D

/* control register (RW) */
#define TP_REG_SYS_CTRL			0x20
#define TP_REG_TMA_CTRL0		0x21	/* prescaler */
#define TP_REG_TMA_CTRL1		0x22	/* subconversion */
#define TP_REG_TMA_CTRL2		0x23	/* shift */
#define TP_REG_TMA_CTRL3		0x24	/* noisethreshold */
#define TP_REG_TMA_CTRL4		0x25	/* fingerthreshold */
#define TP_REG_TMA_CTRL5		0x26
#define TP_REG_NINT_TMO			0x27
#define TP_REG_INITMOVE_STEP		0x28
#define TP_REG_MOVE_STEP		0x29
#define TP_REG_AP_SLEEP_TIME		0x30

#define TP_REG_BIST			0x2E

/* Operating Mode Register Values */
/* errors */
#define TP_VAL_FNGR_CNT_ERR	0x0F

/* finger status */
#define TP_VAL_FNGR_MOVE	0x08
#define TP_VAL_FNGR_DOWN	0x04

/* finger status mask */
#define TP_MASK_FNGR_MOVE	0x08
#define TP_MASK_FNGR_DOWN	0x04

/* Operating Mode Macros */
#define TP_GET_TOUCH_CTR(byte)	((byte &  0xF0) >> 4)
#define TP_GET_FNGR_CNT(byte)	(byte & 0x0F)
#define TP_CHK_NINT_TMO(byte)	((byte & 0x80) >> 7)
#define TP_CHK_MOVE(byte, offset) (((byte >> offset) & TP_MASK_FNGR_MOVE) >> 3)
#define TP_CHK_DOWN(byte, offset) (((byte >> offset) & TP_MASK_FNGR_DOWN) >> 2)
#define TP_SHIFT_FNGR_STAT_MS	4
#define TP_SHIFT_FNGR_STAT_LS	0

#define TP_TOUCH_CNT_MAX	4

#define TP_TRACK_ACTIVE		1
#define TP_TRACK_DELETE		2
#define TP_TRACK_INACTIVE	3

#define TP_FNGR_NOTRACK		0
#define TP_FNGR_TRACK		1

#define SPI_DUMMY_DATA		3		/* AA,BB,CC */
#define TOUCH_DATA_BYTES	27	/* 27 + no dummy data */
#define TOUCH_DATA_FCNT		0 /* offset to finger count byte */
#define TOUCH_DATA_FSTAT1	1 /* offset to finger status 1 byte */
#define TOUCH_DATA_FSTAT2	2 /* offset to finger status 1 byte */

#define TP_MAX_SPI_MSG		64	/* at least 50 registers + 3 dummy */

#define TP_MODE_NORMAL		0x00
#define TP_MODE_SUSPEND		0x01
#define TP_MODE_BL		0x02
#define TP_MODE_ESD_TMR		0x04

#define GOING_TO_SUSPEND	0x01
#define READY_TO_SUSPEND	0x02
#define GOING_TO_BL_RESET	0x04
#define READY_BL_RESET		0x08
#define GOING_TO_RESUME		0x03
#define READY_TO_RESUME		0x05

#define TP_READY_INT		0x01
#define TP_READY_APP		0x02
#define TP_READY_BL		0x04

/*--------------------------------------------------------------------------*/
struct cy8ctma300_spi_touch {
	u8 id;
	u16 __attribute__ ((packed)) x;
	u16 __attribute__ ((packed)) y;
	u8 z;
};

struct cy8ctma300_spi_data {
	u8 touch_status;
	u8 finger_status[2];
	struct cy8ctma300_spi_touch finger_data[TP_TOUCH_CNT_MAX];
};

struct cy8ctma300_bl_data {
	u8 bl_status;
	u8 app_ver;
	u8 proj_id;
	u8 info_ready;
	u8 fw_upd;
};


struct cy8ctma300_touch {
	struct input_dev *input;
	struct spi_device *spi;
	struct work_struct irq_worker;
	struct work_struct resume_worker;
	struct cdev device_cdev;
	int device_major;
	struct class *device_class;
	struct cy8ctma300_spi_touch mt_pos[TP_TOUCH_CNT_MAX];
	u8 active_track_cnt;
	u8 track_state[TP_TOUCH_CNT_MAX];
	u8 track_detect[TP_TOUCH_CNT_MAX];
#ifdef CONFIG_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int init_complete;
	struct mutex touch_lock;
	unsigned long sflag;
	wait_queue_head_t wq;
	u8 mode;
	struct cy8ctma300_bl_data bl_data;
	struct mutex s_lock;
	struct timer_list esd_timer;
	struct work_struct esd_worker;
	u8 bist_failure_ct;
	bool charger_mode_status;
	bool wall_charger_status;
};

/*--------------------------------------------------------------------------*/
#ifdef	DEBUG
static void dump_buf(struct spi_device *spi, u8 *buf, unsigned len)
{
	int i;
	char print_buf[1024];
	char *print_p;

	if (!len || !buf)
		return;

	dev_dbg(&spi->dev, "%s: (%03d): ", __func__, len);

	print_p = print_buf;
	for (i = 1; i <= len; i++) {
		if (i % 16)
			print_p += sprintf(print_p, "0x%02X ", buf[i-1]);
		else
			print_p += sprintf(print_p, "0x%02X\n", buf[i-1]);

	}
	dev_dbg(&spi->dev, "%s", print_buf);
}

static void dump_buf_parse(struct spi_device *spi, u8 *buf, unsigned len)
{
	int i = 0;
	int ct = 0;
	u8 fdetect = 0;
	u16 x, y, z;
	u8 fstat = 0;
	u8 offset = 0;

	dev_dbg(&spi->dev, "%s: (%03d):\n", __func__, len);

	if (!len || !buf)
		return;

	fdetect = TP_GET_FNGR_CNT(buf[TP_REG_FNGR_CNT]);

	dev_dbg(&spi->dev, "%s: nINT Timeout(0x00/b7)=%d\n", __func__,
		TP_CHK_NINT_TMO(buf[TP_REG_NINT_TMO_FLAG]));

	dev_dbg(&spi->dev, "%s: FingerNum(0x00/b3-b0)=%d\n", __func__,
		fdetect);

	for (i = 0; i < TP_TOUCH_CNT_MAX; i++) {
		ct = 6 * i;
		x = buf[TP_REG_FNGR_XL + ct] | (buf[TP_REG_FNGR_XH + ct] << 8);
		y = buf[TP_REG_FNGR_YL + ct] | (buf[TP_REG_FNGR_YH + ct] << 8);
		z = buf[TP_REG_FNGR_Z + ct];

		fstat = buf[((i + 1) / 2) + ((i + 1) % 2)];

		offset = (i + 1) % 2 ? TP_SHIFT_FNGR_STAT_MS :
						TP_SHIFT_FNGR_STAT_LS;

		dev_dbg(&spi->dev, "%s: Finger(%d) STAT(mov=%d dwn=%d) ID=%d"
			", X=%d, Y=%d, Z=%d\n", __func__, i + 1,
			TP_CHK_MOVE(fstat, offset), TP_CHK_DOWN(fstat, offset),
			buf[TP_REG_FNGR_ID + ct], x, y, z);
	}
}
#endif

static int reg_write(struct spi_device *spi, u8 addr, u8 * buf, size_t len)
{
	int err = 0;
	u8 write_buf[TP_MAX_SPI_MSG];

	dev_dbg(&spi->dev, "%s: reg_write(0x%02X, %d)\n", __func__, addr, len);

	if (!buf || !len || (len > (ARRAY_SIZE(write_buf) - SPI_DUMMY_DATA)))
		return -EINVAL;

#ifdef DEBUG
	dump_buf(spi, buf, len);
#endif

	write_buf[0] = addr;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;

	/* copy the buffer data to write_buf */
	memcpy(&write_buf[3], buf, len);

	err = spi_write(spi, write_buf, len + SPI_DUMMY_DATA);

	if (err)
		dev_err(&spi->dev, "%s: reg_write error %d\n", __func__, err);

	return err;
}

static inline int reg_write_byte(struct spi_device *spi, u8 addr, u8 data)
{
	return reg_write(spi, addr, &data, 1);
}

static int reg_read(struct spi_device *spi, u8 addr, u8 * buf, size_t len)
{
	int err = 0;
	u8 read_buf[TP_MAX_SPI_MSG];

	struct spi_message sync_spi_message;
	struct spi_transfer sync_spi_transfer;

	dev_dbg(&spi->dev, "%s: reg_read (0x%02X, %d)\n", __func__, addr, len);

	if (!buf || !len || (len > (ARRAY_SIZE(read_buf) - SPI_DUMMY_DATA)))
		return -EINVAL;

	memset(&sync_spi_transfer, 0, sizeof(sync_spi_transfer));

	read_buf[0] = 0x80 | addr;

	sync_spi_transfer.tx_buf = read_buf;
	sync_spi_transfer.rx_buf = read_buf;
	sync_spi_transfer.len = len + SPI_DUMMY_DATA;

	spi_message_init(&sync_spi_message);
	spi_message_add_tail(&sync_spi_transfer, &sync_spi_message);

	dev_dbg(&spi->dev, "%s: reg_read spi_sync start\n", __func__);

	err = spi_sync(spi, &sync_spi_message);

	if (err) {
		dev_err(&spi->dev, "%s: reg_read error %d\n", __func__, err);
	} else {
#ifdef DEBUG
		dump_buf(spi, read_buf, len + SPI_DUMMY_DATA);
#endif
		memcpy(buf, read_buf + SPI_DUMMY_DATA, len);
	}

	return err;
}

static int reg_read_bl(struct spi_device *spi, u8 * buf, size_t len)
{
	int err = 0;
	u8 read_buf[TP_MAX_SPI_MSG];
	u8 write_buf[2];

	struct spi_message sync_spi_message;
	struct spi_transfer sync_spi_transfer;

	if (!buf || !len)
		return -EINVAL;

	memset(&sync_spi_transfer, 0, sizeof(sync_spi_transfer));

	write_buf[0] = 0x80;
	write_buf[1] = 0x00;

	len = len < 2 ? 2 : len;

	sync_spi_transfer.tx_buf = write_buf;
	sync_spi_transfer.rx_buf = read_buf;
	sync_spi_transfer.len = len;

	spi_message_init(&sync_spi_message);
	spi_message_add_tail(&sync_spi_transfer, &sync_spi_message);

	err = spi_sync(spi, &sync_spi_message);

	if (err) {
		dev_err(&spi->dev, "%s: reg_read error %d\n", __func__, err);
	} else {
#ifdef DEBUG
		dump_buf(spi, read_buf, len);
#endif
		memcpy(buf, read_buf, len);
	}

	return err;
}


static int reg_write_then_read_bl(struct spi_device *spi, u8 *wbuf,
				size_t wlen, u8 *rbuf, size_t rlen)
{
	int err = 0;

	if (!wbuf || !wlen)
		return -EINVAL;

	err = spi_write(spi, wbuf, wlen);

	if (err) {
		dev_err(&spi->dev, "%s: write error %d\n", __func__, err);
		return err;
	}
#ifdef DEBUG
	dump_buf(spi, wbuf, wlen);
#endif

	msleep(100);

	if (rbuf && rlen) {
		err = reg_read_bl(spi, rbuf, rlen);
#ifdef DEBUG
		if (!err)
			dump_buf(spi, rbuf, rlen);
#endif
	}

	return err;
}

#if defined(TOUCH_CALIB_INIT) && defined(TOUCH_CALIB_DUMP)
static void dump_calib_register(struct spi_device *spi)
{
	u8 read_buf = 0xFF;	/* 0xff is set for update*/

	/* Prescaler */
	if (!reg_read(spi, TP_REG_TMA_CTRL0, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: Prescaler=%x\n", __func__, read_buf);
	/* Subconversion */
	if (!reg_read(spi, TP_REG_TMA_CTRL1, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: subconversion=%x\n",
					__func__, read_buf);
	/* Shift Data */
	if (!reg_read(spi, TP_REG_TMA_CTRL2, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: shift=%x\n", __func__, read_buf);
	/* Noise Threshold */
	if (!reg_read(spi, TP_REG_TMA_CTRL3, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: noisethreshold=%x\n",
					__func__, read_buf);
	if (!reg_read(spi, TP_REG_TMA_CTRL4, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: fingerthreshold=%x\n",
					__func__, read_buf);
	/* nINT assertion Timeout */
	if (!reg_read(spi, 0x27, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: nINT assertion Timeout=%x\n",
					__func__, read_buf);

	if (!reg_read(spi, 0x28, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: Initial Step move=%x\n",
					__func__, read_buf);

	if (!reg_read(spi, 0x29, &read_buf, 1))
		dev_dbg(&spi->dev, "%s: Move step=%x\n", __func__, read_buf);
}
#endif

static void cy8ctma300_charger_mode_reset(struct cy8ctma300_touch *tp)
{
	tp->charger_mode_status = false;
	dev_dbg(&tp->spi->dev, "%s: Current charger mode: 0\n", __func__);
}

static void cy8ctma300_charger_status_update(struct cy8ctma300_touch *tp,
					    bool new_status)
{
	dev_dbg(&tp->spi->dev, "%s: Wall charger status: %d => %d\n",
		__func__, tp->wall_charger_status, new_status);
	tp->wall_charger_status = new_status;
}

static int cy8ctma300_charger_mode_update(struct cy8ctma300_touch *tp)
{
	int err = 0;
	if (tp->bl_data.app_ver >= 0x24) {
		if (tp->wall_charger_status)
			err = reg_write_byte(tp->spi, TP_REG_BIST, 0x10);
		else
			err = reg_write_byte(tp->spi, TP_REG_BIST, 0x00);
	}
	if (!err) {
		tp->charger_mode_status = tp->wall_charger_status;
		dev_dbg(&tp->spi->dev, "%s: Set charger mode: %d\n",
			__func__, tp->charger_mode_status);
	} else {
		dev_dbg(&tp->spi->dev, "%s: error %d\n", __func__, err);
	}
	return err;
}

static int reset_device(struct cy8ctma300_touch *tp)
{
	struct spi_device *spi = tp->spi;
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	int reset_time = 250;
	cy8ctma300_charger_mode_reset(tp);

	/* send reset signal to device (Active Low) */
	gpio_set_value(pdata->gpio_reset_pin, 0);
	udelay(reset_time);
	gpio_set_value(pdata->gpio_reset_pin, 1);

	return 0;
}


static void cy8ctma300_esd_worker(struct work_struct *work)
{
	struct cy8ctma300_touch *tp =
		container_of(work, struct cy8ctma300_touch, esd_worker);

	dev_dbg(&tp->spi->dev, "%s: start\n", __func__);

	mutex_lock(&tp->s_lock);
	if ((tp->mode & TP_MODE_BL) || (tp->mode & TP_MODE_SUSPEND))
		goto done;
	/* reset chip */
	if (tp->mode & TP_MODE_ESD_TMR) {
		if (work_pending(&tp->irq_worker))
			flush_work(&tp->irq_worker);
		reset_device(tp);
	}
done:
	mutex_unlock(&tp->s_lock);
}


static void cy8ctma300_esd_timeout(unsigned long this_arg)
{
	struct cy8ctma300_touch *tp = (struct cy8ctma300_touch *) this_arg;

	dev_dbg(&tp->spi->dev, "%s: start\n", __func__);

	del_timer(&tp->esd_timer);
	schedule_work(&tp->esd_worker);
}


static void cy8ctma300_esd_tmr_start(struct cy8ctma300_touch *tp)
{
	struct spi_device *spi = tp->spi;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	if (tp->mode & TP_MODE_ESD_TMR) {
		dev_dbg(&spi->dev, "%s: ESD already started\n", __func__);
		return;
	}
	tp->mode |= TP_MODE_ESD_TMR;
	mod_timer(&tp->esd_timer, (unsigned long)(jiffies + (2*HZ)));
}


static void cy8ctma300_esd_tmr_stop(struct cy8ctma300_touch *tp)
{
	struct spi_device *spi = tp->spi;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	if (!(tp->mode & TP_MODE_ESD_TMR)) {
		dev_dbg(&spi->dev, "%s: ESD not started\n", __func__);
		return;
	}
	tp->mode &= ~TP_MODE_ESD_TMR;
	if (timer_pending(&tp->esd_timer))
		del_timer(&tp->esd_timer);
}


static void cy8ctma300_esd_tmr_update(struct cy8ctma300_touch *tp)
{
	struct spi_device *spi = tp->spi;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	if (!(tp->mode & TP_MODE_ESD_TMR)) {
		dev_dbg(&spi->dev, "%s: ESD not started\n", __func__);
		return;
	}
	mod_timer(&tp->esd_timer, (unsigned long)(jiffies + (2*HZ)));
}


static void cy8ctma300_bist_check(struct cy8ctma300_touch *tp,
				  bool force_calibration)
{
	struct spi_device *spi = tp->spi;
	u8 read_buf = 0xFF;	/* 0xff is set for update*/
	int retry = 5;

	if (force_calibration)
		goto calibrate_and_bist;

	if (reg_read(spi, TP_REG_BIST, &read_buf, 1)) {
		dev_dbg(&spi->dev, "%s: error reading TP_REG_BIST\n",
					__func__);
		return;
	}

	/* BIST Pass */
	if (read_buf == 0x80) {
		tp->bist_failure_ct = 0;
		return;
	} else if (read_buf == 0x40) {
		tp->bist_failure_ct++;
		if (tp->bist_failure_ct < 10)
			return;
	}

	/* Execute calibration and BIST */
calibrate_and_bist:
	dev_dbg(&spi->dev, "%s: execute calibration\n", __func__);
	reg_write_byte(spi, TP_REG_BIST, 0x02);
	while (retry > 0) {
		msleep(300);
		if (reg_read(spi, TP_REG_BIST, &read_buf, 1)) {
			dev_dbg(&spi->dev, "%s: error reading TP_REG_BIST\n",
						__func__);
			return;
		}
		if (read_buf & 0x01) {
			tp->bist_failure_ct = 0;
			return;
		}
		dev_dbg(&spi->dev, "%s: Retry BIST=%x\n", __func__, read_buf);
		retry--;
	}
	dev_dbg(&spi->dev, "%s: BIST not succeeded\n", __func__);
	return;
}


static void cy8ctma300_setup(struct cy8ctma300_touch *tp)
{
	struct spi_device *spi = tp->spi;
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	u8 read_buffer = 0xFF;
	bool force_calibration = false;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	/* check fw version */
	if (tp->bl_data.app_ver == 0) {
		if (reg_read(spi, TP_REG_FW_VER, &read_buffer, 1)) {
			dev_dbg(&spi->dev, "%s: error reading TP_REG_FW_VER\n",
						__func__);
			return;
		}
		tp->bl_data.app_ver = read_buffer;
		dev_info(&spi->dev, "%s: FW version 0x%02X\n", __func__,
					tp->bl_data.app_ver);
		if (reg_read(spi, TP_REG_PROJ_ID, &tp->bl_data.proj_id, 1)) {
			dev_dbg(&spi->dev, "%s: error read TP_REG_PROJ_ID\n",
						__func__);
		}
		tp->bl_data.info_ready |= TP_READY_INT;
	} else if (tp->bl_data.fw_upd > 0) {
		if (reg_read(spi, TP_REG_FW_VER, &read_buffer, 1)) {
			dev_dbg(&spi->dev, "%s: error reading TP_REG_FW_VER\n",
						__func__);
			return;
		}
		force_calibration = true;
		tp->bl_data.fw_upd = 0;
		if (read_buffer != tp->bl_data.app_ver) {
			tp->bl_data.app_ver = read_buffer;
			dev_info(&spi->dev, "%s: FW updated 0x%02X\n",
						__func__, tp->bl_data.app_ver);
		} else {
			dev_info(&spi->dev, "%s: FW not updated\n", __func__);
		}
		if (reg_read(spi, TP_REG_PROJ_ID, &tp->bl_data.proj_id, 1)) {
			dev_dbg(&spi->dev, "%s: error read TP_REG_PROJ_ID\n",
						__func__);
		}
		tp->bl_data.info_ready |= TP_READY_INT;
	}

#ifdef TOUCH_CALIB_INIT
#ifdef TOUCH_CALIB_DUMP
	/* check calibration */
	dump_calib_register(spi);
#endif

	/* update the calibration registers */
	/* Prescaler */
	reg_write_byte(spi, TP_REG_TMA_CTRL0, 0x46);
	/* Subconversion */
	reg_write_byte(spi, TP_REG_TMA_CTRL1, 0x08);
	/* Shift Data */
	reg_write_byte(spi, TP_REG_TMA_CTRL2, 0x04);
	/* Noise Threshold */
	reg_write_byte(spi, TP_REG_TMA_CTRL3, 0x07);
	/* Finger Threshold */
	reg_write_byte(spi, TP_REG_TMA_CTRL4, 0x01);
	/* nINT assertion Timeout */
	reg_write_byte(spi, TP_REG_NINT_TMO, 0x64);
	/* Initial Step Value for Move */
	reg_write_byte(spi, TP_REG_INITMOVE_STEP, 0x08);
	/* Move step size after first move */
	reg_write_byte(spi, TP_REG_MOVE_STEP, 0x08);

	/* Calibration start command */
	reg_write_byte(spi, TP_REG_SYS_CTRL, 0x40);

#ifdef TOUCH_CALIB_DUMP
	dump_calib_register(spi);
#endif
	msleep(100);	/* to be compatible between v1.07 and v1.10 for now */
#endif

	cy8ctma300_bist_check(tp, force_calibration);
	/* auto power control */
	reg_write_byte(spi, TP_REG_L3_SLEEP, 0x00);
	reg_write_byte(spi, TP_REG_L2_SLEEP, 0x08);
	reg_write_byte(spi, TP_REG_L1_SLEEP, 0x08);
	if (pdata->esd_mode && tp->bl_data.app_ver >= 0x14) {
		reg_write_byte(spi, TP_REG_SYS_CTRL, 0xB0);
		cy8ctma300_esd_tmr_start(tp);
	} else {
		/* start command */
		reg_write_byte(spi, TP_REG_SYS_CTRL, 0x30);
	}
}


static void cy8ctma300_update_track(struct cy8ctma300_touch *tp, int track,
			struct cy8ctma300_spi_data *cur_touch, u8 fdetect)
{
	int i;
	int found = 0;
	u8 fstat;
	u8 offset;

	/* find finger and update */
	for (i = 0; i < fdetect; i++) {
		fstat = cur_touch->finger_status[(i / 2)];
		offset = (i + 1) %  2 ? TP_SHIFT_FNGR_STAT_MS :
						TP_SHIFT_FNGR_STAT_LS;

		if ((TP_CHK_MOVE(fstat, offset)) && (tp->mt_pos[track].id ==
					cur_touch->finger_data[i].id)) {
			/* correct x,y MSB and LSB */
			cur_touch->finger_data[i].x =
				(cur_touch->finger_data[i].x << 8) |
				(cur_touch->finger_data[i].x >> 8);
			cur_touch->finger_data[i].y =
				(cur_touch->finger_data[i].y << 8) |
				(cur_touch->finger_data[i].y >> 8);
			/* swap x and y */
			tp->mt_pos[track].x = cur_touch->finger_data[i].x;
			tp->mt_pos[track].y = cur_touch->finger_data[i].y;
			tp->mt_pos[track].z = cur_touch->finger_data[i].z;

			dev_dbg(&tp->spi->dev, "%s: MT track updated\n",
						__func__);
			found = 1;
			tp->track_detect[i] = TP_FNGR_TRACK;
			break;
		}

		/* not move finger found */
		if (tp->mt_pos[track].id == cur_touch->finger_data[i].id) {
			found = 1;
			tp->track_detect[i] = TP_FNGR_TRACK;
			break;
		}
	}

	/* delete track if not detected */
	if (!found) {
		tp->track_state[track] = TP_TRACK_DELETE;

		if (fdetect > 0)
			tp->active_track_cnt--;

		dev_dbg(&tp->spi->dev, "%s: MT track deleted\n", __func__);
	}

}


static void cy8ctma300_new_track(struct cy8ctma300_touch *tp, int track,
			struct cy8ctma300_spi_data *cur_touch, u8 fdetect)
{
	int i;
	u8 fstat;
	u8 offset;

	/* find down detect and add to track */
	for (i = 0; i < fdetect; i++) {

		if (tp->track_detect[i] == TP_FNGR_TRACK)
			continue;

		fstat = cur_touch->finger_status[(i / 2)];
		offset = (i + 1) %  2 ? TP_SHIFT_FNGR_STAT_MS :
						TP_SHIFT_FNGR_STAT_LS;

		if (TP_CHK_DOWN(fstat, offset)) {
			/* correct x,y MSB and LSB */
			cur_touch->finger_data[i].x =
				(cur_touch->finger_data[i].x << 8) |
				(cur_touch->finger_data[i].x >> 8);
			cur_touch->finger_data[i].y =
				(cur_touch->finger_data[i].y << 8) |
				(cur_touch->finger_data[i].y >> 8);

			tp->mt_pos[track].id = cur_touch->finger_data[i].id;
			/* swap x and y */
			tp->mt_pos[track].x = cur_touch->finger_data[i].x;
			tp->mt_pos[track].y = cur_touch->finger_data[i].y;
			tp->mt_pos[track].z = cur_touch->finger_data[i].z;
			tp->track_state[track] = TP_TRACK_ACTIVE;
			tp->active_track_cnt++;

			tp->track_detect[i] = TP_FNGR_TRACK;
			dev_dbg(&tp->spi->dev, "%s: MT track added\n",
						__func__);
			break;
		}
	}
}


static void cy8ctma300_mt_handler(struct cy8ctma300_touch *tp, u8 *read_buf)
{
	struct spi_device *spi = tp->spi;
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	struct cy8ctma300_spi_data *cur_touch =
		(struct cy8ctma300_spi_data *) read_buf;
	int i = 0;
	u8 fdetect = 0;
	u8 report = 0;
	int touch_major;
	int width_major;

	dev_dbg(&tp->spi->dev, "%s: start\n", __func__);

	fdetect = TP_GET_FNGR_CNT(cur_touch->touch_status);

	if (tp->active_track_cnt == 0)
		memset(&tp->track_state[0], TP_TRACK_INACTIVE,
						ARRAY_SIZE(tp->track_state));

	memset(&tp->track_detect[0], TP_FNGR_NOTRACK,
						ARRAY_SIZE(tp->track_detect));

	for (i = 0; i < TP_TOUCH_CNT_MAX; i++) {
		if (tp->track_state[i] == TP_TRACK_ACTIVE)
			cy8ctma300_update_track(tp, i, cur_touch, fdetect);
		else if (tp->track_state[i] == TP_TRACK_INACTIVE)
			cy8ctma300_new_track(tp, i, cur_touch, fdetect);

		if (tp->track_state[i] == TP_TRACK_DELETE) {
			width_major = 0;
			touch_major = 0;
			tp->track_state[i] = TP_TRACK_INACTIVE;
			report = 1;
			dev_dbg(&tp->spi->dev, "%s: MT report removed "
						"finger\n", __func__);
		} else if (tp->track_state[i] == TP_TRACK_ACTIVE) {
			width_major = pdata->width_major;
			touch_major = min((width_major * tp->mt_pos[i].z
					   / pdata->z_max) + 1, width_major);
			report = 1;
			dev_dbg(&tp->spi->dev, "%s: MT report active finger\n",
						__func__);
		} else {
			report = 0;
		}

		if (report) {
			input_report_abs(tp->input, ABS_MT_TRACKING_ID,
						tp->mt_pos[i].id);
			input_report_abs(tp->input, ABS_MT_POSITION_X,
						tp->mt_pos[i].x);
			input_report_abs(tp->input, ABS_MT_POSITION_Y,
						tp->mt_pos[i].y);
			input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR,
						touch_major);
			input_report_abs(tp->input, ABS_MT_WIDTH_MAJOR,
						width_major);
			input_mt_sync(tp->input);
			dev_dbg(&tp->spi->dev,
				"%s: [%d] (x, y)=(%d, %d) major=(%d / %d)\n",
				__func__, tp->mt_pos[i].id,
				tp->mt_pos[i].x, tp->mt_pos[i].y,
				touch_major, width_major);
		}
	}
	input_sync(tp->input);

	if (fdetect == 0)
		tp->active_track_cnt = 0;
}

static void cy8ctma300_touch_worker(struct work_struct *work)
{
	struct cy8ctma300_touch *tp =
		container_of(work, struct cy8ctma300_touch, irq_worker);
	struct spi_device *spi = tp->spi;
	u8 fdetect = 0;
	int err = 0;
	u8 read_buf[TOUCH_DATA_BYTES];

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	mutex_lock(&tp->touch_lock);

	/* SPI sync transaction */
	err = reg_read(spi, TP_REG_FNGR_CNT, read_buf, TOUCH_DATA_BYTES);
	if (err) {
		dev_err(&spi->dev, "%s: Error spi transaction\n", __func__);
		goto out;
	}

#ifdef DEBUG
	dump_buf_parse(spi, read_buf, TOUCH_DATA_BYTES);
#endif

	fdetect = TP_GET_FNGR_CNT(read_buf[TOUCH_DATA_FCNT]);

	if (tp->mode & TP_MODE_ESD_TMR) {
		cy8ctma300_esd_tmr_update(tp);
		if (fdetect == 0x0F && read_buf[TOUCH_DATA_FSTAT1] == 0 &&
					read_buf[TOUCH_DATA_FSTAT2] == 0)
			goto out;
	}

	if (fdetect > TP_TOUCH_CNT_MAX) {
		dev_err(&spi->dev, "%s: Error Invalid detected=%d\n",
					__func__, fdetect);
		goto out;
	}

	if (read_buf[TOUCH_DATA_FSTAT1] == 0 &&
			read_buf[TOUCH_DATA_FSTAT2] == 0) {

		cy8ctma300_setup(tp);
		goto out;
	}

	/* set charger mode if necessary */
	if (tp->charger_mode_status != tp->wall_charger_status) {
		err = cy8ctma300_charger_mode_update(tp);
		if (err)
			goto out;
	}

	/* finger info processing */
	cy8ctma300_mt_handler(tp, read_buf);

out:
	mutex_unlock(&tp->touch_lock);
	return;
}

static irqreturn_t cy8ctma300_touch_irq(int irq, void *handle)
{
	struct cy8ctma300_touch *tp = handle;

	dev_dbg(&tp->spi->dev, "%s: start\n", __func__);

	if (tp->bl_data.app_ver >= 0x14) {
		if (test_bit(GOING_TO_SUSPEND, &tp->sflag)) {
			dev_dbg(&tp->spi->dev, "%s: continue suspend\n",
						__func__);
			set_bit(READY_TO_SUSPEND, &tp->sflag);
			wake_up_interruptible(&tp->wq);
			return IRQ_HANDLED;
		}
	}
	if (test_bit(GOING_TO_BL_RESET, &tp->sflag)) {
		set_bit(READY_BL_RESET, &tp->sflag);
		wake_up_interruptible(&tp->wq);
		return IRQ_HANDLED;
	}
	if (test_bit(GOING_TO_RESUME, &tp->sflag)) {
		dev_dbg(&tp->spi->dev, "%s: GOING_TO_RESUME\n", __func__);
		set_bit(READY_TO_RESUME, &tp->sflag);
		wake_up_interruptible(&tp->wq);
		return IRQ_HANDLED;
	}
	schedule_work(&tp->irq_worker);

	return IRQ_HANDLED;
}

/*--------------------------------------------------------------------------*/

static int cy8ctma300_touch_suspend(struct spi_device *spi,
				pm_message_t message)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	int rc;
	u8 buf[TOUCH_DATA_FSTAT2 + 1];
	long t = msecs_to_jiffies(900);

	/*
	 * Ensure that the resume worker has run to completion,
	 * this is to prevent wrong sequence between resume and suspend.
	 */
	if (work_pending(&tp->resume_worker))
		flush_work(&tp->resume_worker);
	mutex_lock(&tp->s_lock);
	tp->mode |= TP_MODE_SUSPEND;
	if (tp->mode & TP_MODE_BL)
		goto done;
	if (tp->bl_data.app_ver < 0x14)
		goto out;
	set_bit(GOING_TO_SUSPEND, &tp->sflag);
	clear_bit(READY_TO_SUSPEND, &tp->sflag);
	reset_device(tp);
	dev_dbg(&tp->spi->dev, "%s: reset device\n", __func__);

	while (t > 0) {
		t = wait_event_interruptible_timeout(tp->wq,
				test_bit(READY_TO_SUSPEND, &tp->sflag), t);
		if (t <= 0)
			break;
		clear_bit(READY_TO_SUSPEND, &tp->sflag);
		rc = reg_read(tp->spi, TP_REG_FNGR_CNT, buf, ARRAY_SIZE(buf));
		if (rc)
			break;
		if (buf[TOUCH_DATA_FSTAT1] || buf[TOUCH_DATA_FSTAT2])
			continue;
		reg_write_byte(tp->spi, TP_REG_SYS_CTRL, 0x40);
		dev_dbg(&tp->spi->dev, "%s: device sleep\n", __func__);
		break;
	}
	clear_bit(GOING_TO_SUSPEND, &tp->sflag);
out:
	disable_irq(tp->spi->irq);
	if (work_pending(&tp->irq_worker))
		flush_work(&tp->irq_worker);
	if (tp->mode & TP_MODE_ESD_TMR)
		cy8ctma300_esd_tmr_stop(tp);
done:
	mutex_unlock(&tp->s_lock);
	return 0;
}

static void cy8ctma300_resume_worker(struct work_struct *work)
{
	struct cy8ctma300_touch *tp =
		container_of(work, struct cy8ctma300_touch, resume_worker);
	struct spi_device *spi = tp->spi;
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	long t = msecs_to_jiffies(100);
	int err = 0;
	u8 fdetect = 0;
	u8 read_buf[TOUCH_DATA_BYTES];

	dev_dbg(&spi->dev, "%s: start\n", __func__);
	mutex_lock(&tp->s_lock);
	dev_dbg(&spi->dev, "%s: SPI_CS wake-up sequence start\n", __func__);
	if (!pdata->spi_cs_set)
		goto reset;
	set_bit(GOING_TO_RESUME, &tp->sflag);
	clear_bit(READY_TO_RESUME, &tp->sflag);
	if (pdata->spi_cs_set(0)) {
		clear_bit(GOING_TO_RESUME, &tp->sflag);
		goto reset;
	}
	msleep(30);
	if (pdata->spi_cs_set(1)) {
		clear_bit(GOING_TO_RESUME, &tp->sflag);
		goto reset;
	}
	t = wait_event_interruptible_timeout(tp->wq,
			test_bit(READY_TO_RESUME, &tp->sflag), t);
	clear_bit(READY_TO_RESUME, &tp->sflag);
	clear_bit(GOING_TO_RESUME, &tp->sflag);
	if (t <= 0) {
		dev_err(&spi->dev, "%s: timeout\n", __func__);
		goto reset;
	}
	dev_dbg(&spi->dev, "%s: SPI_CS wake-up sequence end, OK\n", __func__);
	msleep(10);
	cy8ctma300_setup(tp);
	/* start command */
	reg_write_byte(spi, TP_REG_SYS_CTRL, 0x30);
	/* SPI sync transaction */
	err = reg_read(spi, TP_REG_FNGR_CNT, read_buf, TOUCH_DATA_BYTES);
	if (err) {
		dev_err(&spi->dev, "%s: Error spi transaction\n", __func__);
		goto done;
	}
	fdetect = TP_GET_FNGR_CNT(read_buf[TOUCH_DATA_FCNT]);
	if (fdetect > TP_TOUCH_CNT_MAX) {
		dev_err(&spi->dev, "%s: Error Invalid detected=%d\n",
					__func__, fdetect);
		goto done;
	}
	/* finger info processing */
	cy8ctma300_mt_handler(tp, read_buf);
	goto done;
reset:
	dev_err(&spi->dev, "%s: error in resuming the device\n", __func__);
	reset_device(tp);
done:
	mutex_unlock(&tp->s_lock);
	return;
}

static int cy8ctma300_touch_resume(struct spi_device *spi)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);

	mutex_lock(&tp->s_lock);
	tp->mode &= ~TP_MODE_SUSPEND;
	if (tp->mode & TP_MODE_BL)
		goto done;
	enable_irq(tp->spi->irq);
	if (tp->bl_data.app_ver >= 0x14 && tp->bl_data.app_ver < 0x23) {
		goto reset;
	} else if (tp->bl_data.app_ver >= 0x23) {
		mutex_unlock(&tp->s_lock);
		/* Let the resume worker process the resume sequence */
		schedule_work(&tp->resume_worker);
		return 0;
	}
reset:
	reset_device(tp);
done:
	mutex_unlock(&tp->s_lock);
	return 0;
}

#ifdef CONFIG_EARLYSUSPEND
static void cy8ctma300_touch_early_suspend(struct early_suspend *es)
{
	struct cy8ctma300_touch *tp;
	tp = container_of(es, struct cy8ctma300_touch, early_suspend);

	dev_dbg(&tp->spi->dev, "%s: early suspend\n", __func__);

	cy8ctma300_touch_suspend(tp->spi, PMSG_SUSPEND);
}

static void cy8ctma300_touch_late_resume(struct early_suspend *es)
{
	struct cy8ctma300_touch *tp;
	tp = container_of(es, struct cy8ctma300_touch, early_suspend);

	dev_dbg(&tp->spi->dev, "%s: late resume\n", __func__);

	cy8ctma300_touch_resume(tp->spi);
}
#endif

static int cy8ctma300_touch_open(struct inode *inode, struct file *file)
{
	struct cy8ctma300_touch *tp =
	container_of(inode->i_cdev, struct cy8ctma300_touch, device_cdev);

	dev_dbg(&tp->spi->dev, "%s: open start %d\n", __func__,
				tp->init_complete);

	file->private_data = tp;
	return 0;
}

static int cy8ctma300_touch_release(struct inode *inode, struct file *file)
{
	return 0;
}


static int do_calibration_valset(struct spi_device *spi,
			struct cy8ctma300_touch_ioctl_clbr *data)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	int err = -1;
	u8 read_buffer;

	mutex_lock(&tp->touch_lock);
	disable_irq(spi->irq);

	dev_dbg(&spi->dev, "%s: prescaler=%d\n", __func__, data->prescaler);
	dev_dbg(&spi->dev, "%s: subconversion=%d\n", __func__,
				data->subconversion);
	dev_dbg(&spi->dev, "%s: shift=%d\n", __func__, data->shift);
	dev_dbg(&spi->dev, "%s: noisethreshold=%d\n", __func__,
				data->noisethreshold);
	dev_dbg(&spi->dev, "%s: fingerthreshold=%d\n", __func__,
				data->fingerthreshold);
	switch (data->clbr_num) {
	case 0:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL0, data->prescaler);
		break;
	case 1:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL1,
						data->subconversion);
		break;
	case 2:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL2, data->shift);
		break;
	case 3:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL3,
						data->noisethreshold);
		break;
	case 4:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL4,
						data->fingerthreshold);
		break;
	/* put additional control register */
	case 5:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL0, data->prescaler);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL1,
						data->subconversion);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL2, data->shift);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL3,
						data->noisethreshold);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL4,
						data->fingerthreshold);
		break;
	case 6:
		err = reg_write_byte(spi, TP_REG_INITMOVE_STEP,
					data->initmovestep);
		break;
	case 7:
		err = reg_write_byte(spi, TP_REG_MOVE_STEP,
						data->movestep);
		break;
	/* put additional control register */
	case 8:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL0,
						data->prescaler);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL1,
						data->subconversion);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL2,
						data->shift);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL3,
						data->noisethreshold);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL4,
						data->fingerthreshold);
		err = reg_write_byte(spi, TP_REG_NINT_TMO,
						data->ninttimeout);
		err = reg_write_byte(spi, TP_REG_INITMOVE_STEP,
						data->initmovestep);
		err = reg_write_byte(spi, TP_REG_MOVE_STEP,
						data->movestep);
		break;
	default:
		err = -EINVAL;
		dev_err(&spi->dev, "%s: clbr_num error %d\n", __func__,
					data->clbr_num);
		goto done;
		break;
	}

	if (err) {
		dev_err(&spi->dev, "%s: error reading register\n", __func__);
		err = -EIO;
		goto done;
	}

	/* update system control to start calibration */
	err = reg_write_byte(spi, TP_REG_SYS_CTRL, 0x40);
	if (err) {
		dev_err(&spi->dev, "%s: error writing TP_REG_SYS_CTRL\n",
					__func__);
		err = -EIO;
		goto done;
	}

	/* adjust or remove this delay */
	msleep(200);

	/* Check if register was set then reset */
	err = reg_read(spi, TP_REG_SYS_CTRL, &read_buffer, 1);
	if (err) {
		dev_err(&spi->dev, "%s: error reading TP_REG_SYS_CTRL\n",
					__func__);
		err = -EIO;
		goto done;
	}

	if (read_buffer == 0x00) {
		/* start all-point mode */
		err = reg_write_byte(spi, TP_REG_SYS_CTRL, 0x20);
		if (err) {
			dev_err(&spi->dev, "%s: err ap mode\n", __func__);
			err = -EIO;
			goto done;
		}
	} else {
		dev_err(&spi->dev, "%s: err TP_REG_SYS_CTRL\n", __func__);
		err = -EIO;
		goto done;
	}

done:
	enable_irq(spi->irq);
	mutex_unlock(&tp->touch_lock);
	return err;
}


static int do_calibration_valget(struct spi_device *spi,
			struct cy8ctma300_touch_ioctl_clbr *data)
{
	int err = -1;

	err = reg_read(spi, TP_REG_TMA_CTRL0, &data->prescaler, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL1, &data->subconversion, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL2, &data->shift, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL3, &data->noisethreshold, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL4, &data->fingerthreshold, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_NINT_TMO, &data->ninttimeout, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_INITMOVE_STEP, &data->initmovestep, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_MOVE_STEP, &data->movestep, 1);
	if (err)
		goto read_err;

	dev_dbg(&spi->dev, "%s: prescaler=%d\n", __func__, data->prescaler);
	dev_dbg(&spi->dev, "%s: subconversion=%d\n", __func__,
				data->subconversion);
	dev_dbg(&spi->dev, "%s: shift=%d\n", __func__, data->shift);
	dev_dbg(&spi->dev, "%s: noisethreshold=%d\n", __func__,
				data->noisethreshold);
	dev_dbg(&spi->dev, "%s: fingerthreshold=%d\n", __func__,
				data->fingerthreshold);
	dev_dbg(&spi->dev, "%s: nINT timeout=%d\n", __func__,
				data->ninttimeout);
	dev_dbg(&spi->dev, "%s: initmovestep=%d\n", __func__,
				data->initmovestep);
	dev_dbg(&spi->dev, "%s: movestep=%d\n", __func__, data->movestep);

	goto done;

read_err:
	err = -EIO;
done:
	return err;
}

static ssize_t cy8ctma300_touch_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct cy8ctma300_touch *tp =
	(struct cy8ctma300_touch *)file->private_data;
	struct spi_device *spi = tp->spi;
	struct cy8ctma300_touch_ioctl_clbr data;

	int err = 0;

	struct cy8ctma300_touch_reg_read_req reg_read_req;
	struct cy8ctma300_touch_reg_write_req reg_write_req;

	dev_dbg(&spi->dev, "%s: start init_complete=%d\n", __func__,
						tp->init_complete);

	switch (cmd) {
	case IOCTL_VALSET:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: copy_from_user error\n",
						__func__);
			goto done;
		}

		err = do_calibration_valset(spi, &data);

		break;

	case IOCTL_VALGET:
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: invalid access\n", __func__);
			goto done;
		}

		err = do_calibration_valget(spi, &data);

		if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: copy_to_user error\n",
						__func__);
			goto done;
		}
		break;

	case IOCTL_TOUCH_REG_READ:
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&reg_read_req, (void __user *)arg,
			sizeof(struct cy8ctma300_touch_reg_read_req))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: copy_from_user error\n",
						__func__);
			goto done;
		}

		if (reg_read(spi, reg_read_req.reg, reg_read_req.buf,
						reg_read_req.len) != 0) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: reg_read error\n", __func__);
			goto done;
		}

		if (copy_to_user((void __user *)arg, (void *)&reg_read_req,
			sizeof(struct cy8ctma300_touch_reg_read_req))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: copy_to_user error\n",
						__func__);
			goto done;
		}
		break;

	case IOCTL_TOUCH_REG_WRITE:
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&reg_write_req, (void __user *)arg,
			sizeof(struct cy8ctma300_touch_reg_write_req))) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: copy_from_user error\n",
						__func__);
			goto done;
		}

		/* use the register write function here */
		if (reg_write(spi, reg_write_req.reg, reg_write_req.buf,
						reg_write_req.len) != 0) {
			err = -EFAULT;
			dev_err(&spi->dev, "%s: reg_write error\n", __func__);
			goto done;
		}

		break;

	default:
		dev_err(&spi->dev, "%s: cmd error\n", __func__);
		return -EINVAL;
		break;
	}

done:
	return err;

}


static int cy8ctma300_bl_reset(struct spi_device *spi, int sleep)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	int rc;
	u8 buf[TOUCH_DATA_FSTAT2 + 1];
	long t = msecs_to_jiffies(900);

	set_bit(GOING_TO_BL_RESET, &tp->sflag);
	clear_bit(READY_BL_RESET, &tp->sflag);
	reset_device(tp);
	dev_dbg(&tp->spi->dev, "%s: reset device\n", __func__);

	while (t > 0) {
		t = wait_event_interruptible_timeout(tp->wq,
				test_bit(READY_BL_RESET, &tp->sflag), t);
		if (t <= 0)
			break;
		clear_bit(READY_BL_RESET, &tp->sflag);
		rc = reg_read(tp->spi, TP_REG_FNGR_CNT, buf, ARRAY_SIZE(buf));
		if (rc)
			break;
		if (buf[TOUCH_DATA_FSTAT1] || buf[TOUCH_DATA_FSTAT2])
			continue;
		if (sleep) {
			reg_write_byte(tp->spi, TP_REG_SYS_CTRL, 0x40);
			dev_dbg(&tp->spi->dev, "%s: device sleep\n", __func__);
		}
		break;
	}
	clear_bit(GOING_TO_BL_RESET, &tp->sflag);
	return (t <= 0 ? 1 : 0);
}


static void cy8ctma300_bl_start(struct spi_device *spi)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	int err = 0;
	u8 bl_start[4] = {'S', 'E', 'M', 'C'};
	u8 bl_enter_cmd[12] = {0x00, 0x00, 0xFF, 0x00, 0x01, 0x02, 0x03,
			       0x04, 0x05, 0x06, 0x07, 0x38};
	u8 read_buf[5] = {0, 0, 0, 0, 0};
	u8 reset = 0;
	u8 sleep = 0;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	if (tp->mode & TP_MODE_BL) {
		dev_err(&spi->dev, "%s: bl already started\n", __func__);
		return;
	}

	mutex_lock(&tp->s_lock);
	tp->mode |= TP_MODE_BL;
	if (tp->mode & TP_MODE_SUSPEND)
		enable_irq(spi->irq);
	else if (tp->mode & TP_MODE_ESD_TMR)
		cy8ctma300_esd_tmr_stop(tp);
	mutex_unlock(&tp->s_lock);

	err = cy8ctma300_bl_reset(spi, 0);
	if (err)
		dev_err(&spi->dev, "%s: no ready IRQ\n", __func__);

	if (!(tp->bl_data.info_ready & TP_READY_BL)) {
		err = reg_write(spi, 0x2A, bl_start, 4);
		if (err)
			goto err;
		msleep(300);

		/* ENTER boot loader mode start */
		err = reg_write_then_read_bl(spi, bl_enter_cmd,
			ARRAY_SIZE(bl_enter_cmd), read_buf, 2);
		if (err)
			goto err;

		if ((read_buf[0] != 0x01) || (read_buf[1] != 0x00)) {
			dev_err(&spi->dev, "%s: BootloaderEnter error "
					"read_buf=%x,%x\n", __func__,
					read_buf[0], read_buf[1]);
			reset++;
			goto err;
		}
	}
	dev_dbg(&spi->dev, "%s: BootloaderEnter OK read_buf=%x,%x\n", __func__,
		read_buf[0], read_buf[1]);
	return;
err:
	mutex_lock(&tp->s_lock);
	if (reset) {
		/* return to normal or sleep */
		if (tp->mode & TP_MODE_SUSPEND &&
					tp->bl_data.app_ver >= 0x14)
			sleep = 1;
		err = cy8ctma300_bl_reset(spi, sleep);
		if (err)
			dev_err(&spi->dev, "%s: no ready IRQ\n", __func__);
		if (!(tp->mode & TP_MODE_SUSPEND))
			cy8ctma300_setup(tp);
	}
	if (tp->mode & TP_MODE_SUSPEND) {
		disable_irq(spi->irq);
		if (tp->mode & TP_MODE_ESD_TMR)
			cy8ctma300_esd_tmr_stop(tp);
	}
	tp->mode &= ~TP_MODE_BL;
	mutex_unlock(&tp->s_lock);
	return;
}


static void cy8ctma300_bl_end(struct spi_device *spi)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	u8 bl_exit_cmd[12] = {0x00, 0x00, 0xFF, 0x00, 0x01, 0x02, 0x03,
				 0x04, 0x05, 0x06, 0x07, 0x3B};
	int err = 0;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	if (!(tp->mode & TP_MODE_BL) &&
		!(tp->bl_data.info_ready & TP_READY_BL)) {
		dev_err(&spi->dev, "%s: bl not started\n", __func__);
		return;
	}

	/* EXIT bootloader mode start */
	err = reg_write_then_read_bl(spi, bl_exit_cmd,
			ARRAY_SIZE(bl_exit_cmd), NULL, 0);
	if (err) {
		dev_err(&spi->dev, "%s: write/read error\n", __func__);
		goto end;
	}
	/* need to wait after issuing EXIT bootloader mode command */
	msleep(900);
	dev_dbg(&spi->dev, "%s: BootloaderExit OK\n", __func__);
end:
	mutex_lock(&tp->s_lock);
	tp->bl_data.fw_upd = 1;
	tp->bl_data.info_ready = 0;

	/* return to normal or sleep */
	err = cy8ctma300_bl_reset(spi, 0);
	if (err)
		dev_err(&spi->dev, "%s: no ready IRQ\n", __func__);
	cy8ctma300_setup(tp);
	if (tp->mode & TP_MODE_SUSPEND) {
		if (tp->bl_data.app_ver >= 0x14)
			err = cy8ctma300_bl_reset(spi, 1);
		if (err)
			dev_err(&spi->dev, "%s: no ready IRQ\n", __func__);
		disable_irq(spi->irq);
		if (tp->mode & TP_MODE_ESD_TMR)
			cy8ctma300_esd_tmr_stop(tp);
	}
	tp->mode &= ~TP_MODE_BL;
	mutex_unlock(&tp->s_lock);
}


static void cy8ctma300_bl_check(struct spi_device *spi)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	u8 read_buf[4] = {0, 0, 0, 0};
	int err = 0;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	err = reg_read_bl(spi, read_buf, 4);
	if (err)
		return;
	if (read_buf[0] == 0x01 && read_buf[1] == 0x00)
		tp->bl_data.info_ready |= TP_READY_BL;
	else
		tp->bl_data.info_ready |= TP_READY_APP;
}


static ssize_t cy8ctma300_bl_fw_write(struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cy8ctma300_touch *tp = dev_get_drvdata(dev);
	struct spi_device *spi = tp->spi;
	int err = 0;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	mutex_lock(&tp->touch_lock);
	if (!(tp->mode & TP_MODE_BL)) {
		dev_err(&spi->dev, "%s: bl not started\n", __func__);
		goto end;
	}

	err = reg_write_then_read_bl(spi, buf, size, NULL, 0);

	if (err)
		dev_err(&spi->dev, "%s: write/read error\n", __func__);

end:
	mutex_unlock(&tp->touch_lock);
	return size;
}


static ssize_t cy8ctma300_bl_fw_read(struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t size)
{
	int count = 0;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cy8ctma300_touch *tp = dev_get_drvdata(dev);
	struct spi_device *spi = tp->spi;
	u8 read_buffer[2];
	int err = 0;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	mutex_lock(&tp->touch_lock);
	if (!(tp->mode & TP_MODE_BL)) {
		dev_err(&spi->dev, "%s: bl not started\n", __func__);
		goto end;
	}

	err = reg_read_bl(spi, read_buffer, 2);
	if (err) {
		dev_err(&spi->dev, "%s: read error %d\n", __func__, err);
		goto end;
	}

	tp->bl_data.bl_status = read_buffer[0];
	*(u8 *)buf = tp->bl_data.bl_status;
	count = sizeof(u8);

end:
	mutex_unlock(&tp->touch_lock);
	return count;
}


static struct bin_attribute cy8ctma300_firmware = {
	.attr = {
		.name = "firmware",
		.mode = 0644,
	},
	.size = 256,
	.read = cy8ctma300_bl_fw_read,
	.write = cy8ctma300_bl_fw_write,
};


static ssize_t cy8ctma300_cmd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(dev);
	struct spi_device *spi = tp->spi;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	return sprintf(buf, "0x%02X 0x%02X 0x%02X\n", tp->bl_data.app_ver,
					tp->bl_data.proj_id,
					tp->bl_data.info_ready);
}


static ssize_t cy8ctma300_cmd_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(dev);
	struct spi_device *spi = tp->spi;
	char cmdstr[25];
	int err = 0;
	int ret_val;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	mutex_lock(&tp->touch_lock);

	ret_val = sscanf(buf, "%s", cmdstr);
	if (ret_val != 1) {
		dev_err(&spi->dev, "%s: cmd read error\n", __func__);
			ret_val = -EINVAL;
			goto end;
	}

	if (strcmp(cmdstr, "blstart") == 0) {
		err = sysfs_create_bin_file(&dev->kobj, &cy8ctma300_firmware);
		if (err) {
			dev_err(&spi->dev, "%s: cannot create file\n",
				__func__);
			ret_val = -EINVAL;
			goto end;
		}
		cy8ctma300_bl_start(spi);
		if (!(tp->mode & TP_MODE_BL)) {
			ret_val = -EINVAL;
			sysfs_remove_bin_file(&dev->kobj, &cy8ctma300_firmware);
			goto end;
		}
	} else if (strcmp(cmdstr, "blend") == 0) {
		dev_dbg(&spi->dev, "%s: cmd blend\n", __func__);
		cy8ctma300_bl_end(spi);
		sysfs_remove_bin_file(&dev->kobj, &cy8ctma300_firmware);
	} else if (strcmp(cmdstr, "blcheck") == 0) {
		cy8ctma300_bl_check(spi);
	} else if (strcmp(cmdstr, "cmstart") == 0) {
		cy8ctma300_charger_status_update(tp, true);
	} else if (strcmp(cmdstr, "cmend") == 0) {
		cy8ctma300_charger_status_update(tp, false);
	} else {
		/* not supported command */
		dev_err(&spi->dev, "%s: cmd not supported\n", __func__);
	}

	ret_val = strlen(buf);

end:
	mutex_unlock(&tp->touch_lock);
	return  ret_val;
}


static DEVICE_ATTR(touch_cmd, S_IRUSR | S_IWUSR | S_IROTH, cy8ctma300_cmd_show,
			cy8ctma300_cmd_store);

static const struct file_operations cy8ctma300_touch_fops = {
	.owner = THIS_MODULE,
	.open = cy8ctma300_touch_open,
	.ioctl = cy8ctma300_touch_ioctl,
	.release = cy8ctma300_touch_release,
};

static int cy8ctma300_touch_probe(struct spi_device *spi)
{
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	struct cy8ctma300_touch *tp = NULL;
	int err = 0;
	struct input_dev *dev;
	dev_t device_t = MKDEV(0, 0);
	struct device *class_dev_t = NULL;

	dev_dbg(&spi->dev, "%s: start\n", __func__);

	if (!pdata) {
		dev_err(&spi->dev, "%s: no platform data?\n", __func__);
		return -ENODEV;
	}

	if (!spi->irq) {
		dev_err(&spi->dev, "%s: no IRQ?\n", __func__);
		return -ENODEV;
	}

	if (pdata && pdata->gpio_init) {
		err = pdata->gpio_init();
		if (err > 0)
			dev_err(&spi->dev, "CY8CTMA300_TOUCH: gpio "
						"configuration failed\n");
	}

	/* Set up SPI */
	spi->bits_per_word = 8;

	dev_dbg(&spi->dev, "%s: SPI setup (%uHz) OK\n", __func__,
				spi->max_speed_hz);

	err = spi_setup(spi);
	if (err < 0)
		return err;

	/* GPIO: set up */
	dev_dbg(&spi->dev, "%s: Requesting GPIO Reset ownership\n", __func__);

	err = gpio_request(pdata->gpio_reset_pin, "cy8ctma300_touch_reset");
	if (err)
		goto err_gpio_setup;

	dev_dbg(&spi->dev, "%s: Requesting GPIO IRQ ownership\n", __func__);

	err = gpio_request(pdata->gpio_irq_pin, "cy8ctma300_touch_irq");
	if (err)
		goto err_gpio_setup;

	dev_dbg(&spi->dev, "%s: Configuring GPIO Reset direction\n", __func__);

	err = gpio_direction_output(pdata->gpio_reset_pin, 1);
	if (err)
		goto err_gpio_setup;

	dev_dbg(&spi->dev, "%s: Configuring GPIO IRQ direction\n", __func__);

	err = gpio_direction_input(pdata->gpio_irq_pin);
	if (err)
		goto err_gpio_setup;

	tp = kzalloc(sizeof(struct cy8ctma300_touch), GFP_KERNEL);
	if (!tp) {
		err = -ENOMEM;
		goto err_gpio_setup;
	}

	dev_dbg(&spi->dev, "%s: Allocated private data\n", __func__);

	mutex_init(&tp->touch_lock);
	mutex_init(&tp->s_lock);

	tp->spi = spi;
	dev_set_drvdata(&spi->dev, tp);

	dev_dbg(&spi->dev, "%s: Driver data set\n", __func__);

	tp->esd_timer.function = cy8ctma300_esd_timeout;
	tp->esd_timer.data = (unsigned long)tp;
	tp->esd_timer.expires = (unsigned long) (jiffies + (2*HZ));
	init_timer(&tp->esd_timer);

	/* initialize workers */
	INIT_WORK(&tp->esd_worker, cy8ctma300_esd_worker);
	/*
	 * The resume sequence of this device requires huge delay,
	 * resume worker is added to prevent the kernel's thread
	 * (resume thread) from blocking.
	 */
	INIT_WORK(&tp->resume_worker, cy8ctma300_resume_worker);
	INIT_WORK(&tp->irq_worker, cy8ctma300_touch_worker);
	init_waitqueue_head(&tp->wq);

	dev = input_allocate_device();
	if (!dev) {
		err = -ENOMEM;
		goto err_cleanup_mem;
	}
	dev_dbg(&spi->dev, "%s: Allocated input device\n", __func__);
		tp->input = dev;

	dev->name = "cy8ctma300_touch";
	dev->phys = "cy8ctma300_touch/input0";
	dev->dev.parent = &spi->dev;

	dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/* cypressTMA300E multitouch support */
	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, pdata->width_major, 0, 0);
	input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, pdata->z_max, 0, 0);

	err = input_register_device(dev);
	if (err)
		goto err_cleanup_device_mem;

	dev_dbg(&spi->dev, "%s: Registered input device\n", __func__);

	err = alloc_chrdev_region(&device_t, 0, 1, "cypress_touchscreen");
	if (err)
		goto err_cleanup_input;

	dev_dbg(&spi->dev, "%s: Allocated character device\n", __func__);

	tp->device_major = MAJOR(device_t);

	cdev_init(&(tp->device_cdev), &cy8ctma300_touch_fops);
	tp->device_cdev.owner = THIS_MODULE;
	tp->device_cdev.ops = &cy8ctma300_touch_fops;

	err = cdev_add(&(tp->device_cdev), MKDEV(tp->device_major, 0), 1);
	if (err)
		goto err_cleanup_chrdev;

	dev_dbg(&spi->dev, "%s: Character device added\n", __func__);

	tp->device_class = class_create(THIS_MODULE, "cypress_touchscreen");
	if (IS_ERR(tp->device_class)) {
		err = -1;
		goto err_cleanup_cdev;
	}

	class_dev_t = device_create(tp->device_class, NULL,
		MKDEV(tp->device_major, 0), NULL, "cypress_touchscreen");

	if (IS_ERR(class_dev_t)) {
		err = -1;
		goto err_cleanup_class;
	}

	/* sysfs */
	err = device_create_file(&spi->dev, &dev_attr_touch_cmd);
	if (err)
		goto err_cleanup_device;

#ifdef CONFIG_EARLYSUSPEND
	/* register early suspend */
	tp->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tp->early_suspend.suspend = cy8ctma300_touch_early_suspend;
	tp->early_suspend.resume = cy8ctma300_touch_late_resume;
	register_early_suspend(&tp->early_suspend);
#endif

	/* workaround for irq-on effect to v1.10 and v1.07 */
	err = request_irq(tp->spi->irq, cy8ctma300_touch_irq,
			IRQF_TRIGGER_FALLING, tp->spi->dev.driver->name, tp);

	if (err) {
		dev_err(&spi->dev, "irq %d busy?\n", tp->spi->irq);
		goto err_cleanup_file;
	}

	dev_dbg(&spi->dev, "%s: Registered IRQ\n", __func__);

	tp->init_complete++;

	dev_dbg(&spi->dev, "%s: Device registered OK\n", __func__);

	return 0;

err_cleanup_file:
	device_remove_file(&spi->dev, &dev_attr_touch_cmd);
#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif
err_cleanup_device:
	device_destroy(tp->device_class, MKDEV(tp->device_major, 0));
err_cleanup_class:
	class_destroy(tp->device_class);
err_cleanup_cdev:
	cdev_del(&(tp->device_cdev));
err_cleanup_chrdev:
	unregister_chrdev_region(device_t, 1);
err_cleanup_input:
	input_unregister_device(dev);
	goto err_cleanup_mem;
err_cleanup_device_mem:
	input_free_device(dev);
err_cleanup_mem:
	mutex_destroy(&tp->touch_lock);
	mutex_destroy(&tp->s_lock);
	kfree(tp);
err_gpio_setup:
	gpio_free(pdata->gpio_reset_pin);
	gpio_free(pdata->gpio_irq_pin);
	dev_err(&spi->dev, "%s: probe() fail: %d\n", __func__, err);
	return err;
}

static int __devexit cy8ctma300_touch_remove(struct spi_device *spi)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	dev_t device_t;

	dev_dbg(&spi->dev, "%s: unregistering touchscreen\n", __func__);

	if (!tp)
		return -ENODEV;
	device_t = MKDEV(tp->device_major, 0);
	if (!tp->init_complete) {
		dev_err(&spi->dev, "%s: can't unregister driver, FW\n",
					__func__);
		return -EBUSY;
	}

	cy8ctma300_touch_suspend(spi, PMSG_SUSPEND);
	device_remove_file(&spi->dev, &dev_attr_touch_cmd);
	if (tp->device_class && !IS_ERR(tp->device_class)){
		device_destroy(tp->device_class, device_t);
		class_destroy(tp->device_class);
	}

#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif

	if (tp->input)
		input_unregister_device(tp->input);
	if (&tp->device_cdev) {
		cdev_del(&(tp->device_cdev));
		unregister_chrdev_region(device_t, 1);
	}
	free_irq(tp->spi->irq, tp);
	mutex_destroy(&tp->touch_lock);
	mutex_destroy(&tp->s_lock);
	kfree(tp);
	gpio_free(pdata->gpio_reset_pin);
	gpio_free(pdata->gpio_irq_pin);
	dev_dbg(&spi->dev, "%s: unregistered touchscreen\n", __func__);

	return 0;
}

static struct spi_driver cy8ctma300_touch_driver = {
	.driver = {
		.name = "cypress_touchscreen",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		},
	.probe = cy8ctma300_touch_probe,
	.remove = __devexit_p(cy8ctma300_touch_remove),
#ifndef CONFIG_EARLYSUSPEND
	.suspend = cy8ctma300_touch_suspend,
	.resume = cy8ctma300_touch_resume,
#endif
};

static int __init cy8ctma300_touch_init(void)
{
	int err;

	printk(KERN_DEBUG "%s: V%s built %s %s\n", __func__, MODULE_VER,
						__DATE__, __TIME__);
	err = spi_register_driver(&cy8ctma300_touch_driver);
	printk(KERN_DEBUG "%s: module init, result=%d\n", __func__, err);

	return err;
}

static void __exit cy8ctma300_touch_exit(void)
{
	spi_unregister_driver(&cy8ctma300_touch_driver);
	printk(KERN_DEBUG "%s: module exit\n", __func__);
}

module_init(cy8ctma300_touch_init);
module_exit(cy8ctma300_touch_exit);

MODULE_DESCRIPTION("Touchscreen driver for Cypress CY8CTMA300 hardware");
MODULE_LICENSE("GPL");
