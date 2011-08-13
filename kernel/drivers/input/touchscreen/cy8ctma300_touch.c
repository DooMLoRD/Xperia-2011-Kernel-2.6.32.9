/* linux/drivers/input/touchscreen/cy8ctma300_touch.c
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
 *   cy8ctma300 touchscreen driver
 *
 * <Supported firmware version>
 *   TBD
 *
 * <Supported FPGA version>
 *   TBD
 */


#undef	DEBUG
#define	DUMP_BUF_SHORT

#undef MEASURE_PERFORMANCE

#define MODULE_VER	"0.9b-mt"

#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/cypress_touch.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

#ifdef MEASURE_PERFORMANCE
#include <linux/jiffies.h>
#endif

#ifdef	CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef	CONFIG_ARM
#include <asm/mach-types.h>
#endif

/* FIXME- Use firmware-loader facility */
#include "cypress-firmware-0x0072.h"

/*--------------------------------------------------------------------------*/

/* Operating Mode Register */
#define	TP_REG_HST_MODE			0x00

#define	TP_REG_TT_MODE			0x01
#define	TP_REG_TT_STAT			0x02

#define	TP_REG_PNT1XH			0x03
#define	TP_REG_PNT1XL			0x04
#define	TP_REG_PNT1YH			0x05
#define	TP_REG_PNT1YL			0x06
#define	TP_REG_PNT1Z			0x07

#define	TP_REG_PNT12_ID			0x08

#define	TP_REG_PNT2XH			0x09
#define	TP_REG_PNT2XL			0x0A
#define	TP_REG_PNT2YH			0x0B
#define	TP_REG_PNT2YL			0x0C
#define	TP_REG_PNT2Z			0x0D

#define	TP_REG_PNT3XH			0x10
#define	TP_REG_PNT3XL			0x11
#define	TP_REG_PNT3YH			0x12
#define	TP_REG_PNT3YL			0x13
#define	TP_REG_PNT3Z			0x14

#define	TP_REG_PNT34_ID			0x15

#define	TP_REG_PNT4XH			0x16
#define	TP_REG_PNT4XL			0x17
#define	TP_REG_PNT4YH			0x18
#define	TP_REG_PNT4YL			0x19
#define	TP_REG_PNT4Z			0x1A

#define	TP_REG_GEST_CNT			0x0E
#define	TP_REG_GEST_ID			0x0F
#define	TP_REG_GEST_SET			0x1E

/* Gesture active distance. BEWARE: this is more than just for gestures */
#define	TP_GEST_ACTD			0x02

/* BIST Mode Register Set */
#define	TP_BISTR_BIST_STAT		0x01
#define	TP_BISTR_BIST_NUM		0x02
/* ... */
#define TP_BISTR_UID_0			0x07
#define	TP_BISTR_UID_1			0x08
#define	TP_BISTR_UID_2			0x09
/* you get the idea ... */
#define	TP_BISTR_BL_VERH		0x0F
#define	TP_BISTR_BL_VERL		0x10

#define	TP_BISTR_TTS_VERH		0x11
#define	TP_BISTR_TTS_VERL		0x12

#define	TP_BISTR_AP_IDH			0x13
#define	TP_BISTR_AP_IDL			0x14

#define	TP_BISTR_AP_VERH		0x15
#define	TP_BISTR_AP_VERL		0x16

#define	TP_REG_FW			0xFF

/* more of these later if/as I need 'em */

#define	TP_TM_BIT_BIST			(1 << 4)
#define	TP_TM_BIT_SRESET		(1 << 0)
#define	TP_TM_BIT_DEEPSLEEP		(1 << 1)

#define	TP_TT_BIT_DINVAL		(1 << 5)
#define	TP_TT_BIT_FACE			(1 << 4)
#define	TP_TT_BIT_TOUCH_MASK		(0xF)

#define	TP_BL_BIT_BLRDY			(1 << 0)
#define	TP_BL_BIT_BM			(1 << 1)
#define	TP_BL_BIT_ERR_MASK		(~(TP_BL_BIT_BLRDY | TP_BL_BIT_BM))

/* if set alone in TP_REG_FW we're in application mode OK
   in later (> 0.65) firmwares. Sanity check */
#define	TP_AP_BIT_APPMODE		(1 << 2)

#define	FIRMWARE_BLOCK_SIZE		(128)
#define	FIRMWARE_USER_BLOCKS		(237)
#define	FIRMWARE_TOTAL_BLOCKS		(256)

#define	TOUCH_DATA_BYTES		(TP_REG_PNT4Z + 1)

#define	BL_CMD_BL_ENTER			(0x38)
#define	BL_CMD_WR_BLOCK			(0x39)
#define	BL_CMD_BL_EXIT			(0x3B)
/* Big-Endian representation */
#define BL_CMD_EXECUTE			(0x53CA)

/* buffer valid bits, firmware 87+ 107+ */
#define TP_SPI_BUFFER_VALID_MASK	0xC0	/* mask bits 6, 7 in reg 2 */
#define TP_SPI_BUFFER_VALID		0x40	/* bit 6 set = accept data */
#define TP_SPI_BUFFER_VALID_VER		87	/* FW version 87 and above */
#define TP_FW_NSB_VALID_VER		114	/* FW version 0x72 and above */

/* cypress firmware SSD versus DSD type field - 0x13 */
#define TP_PANEL_TYPE_SSD		0x07	/* SSD display */
#define TP_PANEL_TYPE_DSD		0x08	/* DSD display */

#ifdef	DEBUG
#define	DEBUG_PRINTK(format, args...) \
	{ if (dump_noisy) \
		printk(format , ## args); }
#else
#define	DEBUG_PRINTK(format, args...)
#endif

#define	ENABLE_IRQ(this)	{ if (!this->irq_suspend_enabled++) \
					enable_irq(this->pdata->irq); }

#define	DISABLE_IRQ(this)	{ if (this->irq_suspend_enabled) {\
					disable_irq_nosync(this->pdata->irq); \
					this->irq_suspend_enabled = 0; } }

static int force_update;
static int shit_firmware;
module_param(force_update, int, 0);
MODULE_PARM_DESC(force_update, "Force flashing of driver firmware");

/*--------------------------------------------------------------------------*/

struct cy8ctma300_touch {
	struct input_dev			*input;
	spinlock_t				lock;
	struct mutex				mutex;
	struct spi_device			*spi;
	struct work_struct			fwupd_work;
	struct work_struct			isr_work;
	struct spi_message			async_spi_message;
	struct spi_transfer			async_spi_transfer;
	u8			async_read_buf[TOUCH_DATA_BYTES];
	struct cdev				device_cdev;
	int					device_major;
	struct class				*device_class;
	int					first_irq;
#ifdef CONFIG_EARLYSUSPEND
	struct early_suspend			early_suspend;
#endif
	int					irq_suspend_enabled;
	int					init_complete;
	int					has_been_initialized;
	struct cypress_touch_platform_data	*pdata;
#ifdef MEASURE_PERFORMANCE
	unsigned long				measure_start;
	int					measure_count;
	int					measure_dupes;
	u16					dupe_x;
	u16					dupe_y;
	u8					dupe_touch;
#endif
	u8 					suspend;
	int					use_spi_buffer_valid;
	int					tp_fw_type;	/* DSD/SSD */
	int					fw_version;
	struct cypress_callback			cb_struct;
	struct work_struct			charger_work;
	int	                                nsb_work_todo;
	int					nsb_state;
	int					nsb_new_state;
};

struct fw_packet {
	u8					bl_keys[8];
	u8					bl_cmd;
	u16					bl_blockno;
	u8					bl_data[FIRMWARE_BLOCK_SIZE];
	u8					bl_data_csum;
	u8					bl_cmd_csum;
	u16					bl_execute;
} __attribute__((packed));

/* send packet to get touch data from device */
static u8 const cy8ctma300_send_msg[TOUCH_DATA_BYTES + 2] = { 0, 0, };

static u8 const bootloader_keys[] = { 0, 1, 2, 3, 4, 5, 6, 7, };

static int dump_noisy = 1;

static int cy8ctma300_deferred_init(struct cy8ctma300_touch *this);
static int reset_device(struct cy8ctma300_touch *this);
static void charger_noise_suppression(struct cy8ctma300_touch *this,
					struct spi_device *spi);

/*--------------------------------------------------------------------------*/

static void dump_buf(u8 *buf, unsigned len)
{
#ifdef	DEBUG
	int i;

	if (!len || !buf || !dump_noisy)
		return;

#ifdef	DUMP_BUF_SHORT
	if (len > 12)
		len = 12;
#endif
	printk(KERN_INFO "CY8CTMA300_TOUCH: dump_buf (%03d): ", len);
	for (i = 0; i < len; i++)
		printk("0x%02X ", buf[i]);
	printk("\n");
#endif
};

static int reg_write(struct spi_device *spi, u8 addr, u8* buf, size_t len)
{
	int	err = 0;
	u8	*write_buf;

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: reg_write(0x%02X, %d)\n", addr, len);

	if (!buf || !len)
		return -EINVAL;

	write_buf = kzalloc(sizeof(u8) * (len + 2), GFP_KERNEL);
	if (!write_buf)
		return -ENOMEM;

	dump_buf(buf, len);

	write_buf[0] = 0x01;
	write_buf[1] = addr;

	memcpy(&write_buf[2], buf, len);

	err = spi_write(spi, write_buf, len + 2);
	msleep(10);

	kfree(write_buf);

	if (err)
		printk(KERN_ERR "CY8CTMA300_TOUCH: reg_write error %d\n", err);

	return err;
};

static inline int reg_write_byte(struct spi_device *spi, u8 addr, u8 data)
{
	return reg_write(spi, addr, &data, 1);
};

static int reg_read(struct spi_device *spi, u8 addr, u8* buf, size_t len)
{
	int	err = 0;
	u8	read_req[2];
	u8	*read_buf;
	size_t	newlen;

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: reg_read (0x%02X, %d)\n", addr, len);

	if (!buf || !len)
		return -EINVAL;

	/* re-buffer reads to ensure even-byte transfers */
	read_buf = kzalloc(sizeof(u8) * (newlen = len & 1 ? len + 1 : len),
		GFP_KERNEL);
	if (!read_buf)
		return -ENOMEM;

	read_req[0] = 0x00;
	read_req[1] = addr;

	err = spi_write(spi, read_req, 2);
	if (err) {
		printk(KERN_ERR
			"CY8CTMA300_TOUCH: reg_read() write err %d\n", err);
		goto out;
	};
	msleep(10);

	err = spi_read(spi, read_buf, newlen);
	if (err) {
		printk(KERN_ERR
			"CY8CTMA300_TOUCH: reg_read() read err %d\n", err);
	} else {
		memcpy(buf, read_buf, len);
		dump_buf(buf, len);
	};

out:
	kfree(read_buf);
	return err;
};

static int reg_read_intcause_async(struct spi_device *spi, void* complete)
{
	struct cy8ctma300_touch *this = dev_get_drvdata(&spi->dev);

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: reg_read_intcause_async()\n");

	this->async_spi_transfer.tx_buf = cy8ctma300_send_msg;
	this->async_spi_transfer.rx_buf = this->async_read_buf;
	this->async_spi_transfer.len = TOUCH_DATA_BYTES;

	spi_message_init(&this->async_spi_message);
	spi_message_add_tail(&this->async_spi_transfer,
		&this->async_spi_message);
	this->async_spi_message.complete = complete;
	this->async_spi_message.context = (void *)this;

	return spi_async(spi, &this->async_spi_message);
};

static int bl_flash_mode(struct spi_device *spi)
{
	int err;
	u8 status;

	mdelay(100);

	err = reg_read(spi, TP_REG_FW, &status, 1);
	if (err)
		return err;

	if (status & TP_BL_BIT_ERR_MASK)
		return status;

	return ((status == (TP_BL_BIT_BM | TP_BL_BIT_BLRDY)) ? 0 : 1);
};

static void do_checksum(struct fw_packet *pk)
{
	u8 *fw_hdr;

	pk->bl_data_csum = 0;
	pk->bl_cmd_csum = TP_REG_FW;	/* calc includes the FW addr byte */

	for (fw_hdr = &pk->bl_keys[0]; fw_hdr < &pk->bl_data[0]; fw_hdr++)
		pk->bl_cmd_csum += *fw_hdr;
	for (; fw_hdr < &pk->bl_data_csum; fw_hdr++)
		pk->bl_data_csum += *fw_hdr;

	/* checksum data val, then the csum byte itself */
	pk->bl_cmd_csum += (pk->bl_data_csum * 2);
};

static int write_firmware_block(struct spi_device *spi, u8 *block, u16 blockno)
{
	struct fw_packet pk;
	int err = 0;

	/* once bitten, twice shy */
	if (!blockno)
		return -EROFS;

	if (blockno >= FIRMWARE_TOTAL_BLOCKS)
		return -ERANGE;

	memcpy(&pk.bl_keys, bootloader_keys, sizeof(pk.bl_keys));
	pk.bl_cmd = BL_CMD_WR_BLOCK;
	pk.bl_blockno = (blockno << 8) | (blockno >> 8);
	memcpy(&pk.bl_data, block, FIRMWARE_BLOCK_SIZE);
	do_checksum(&pk);
	pk.bl_execute = BL_CMD_EXECUTE;

	err = reg_write(spi, TP_REG_FW, (u8 *)&pk, sizeof(struct fw_packet));
	if (err)
		return err;

	err = bl_flash_mode(spi);
	if (err < 0)
		return err;

	if (err)
		return -EPROTO;

	return err;
};

/* sanity check on stored firmware file. Not necessary now but will be for
   when the file is loaded the right way */
static int verify_fw_checksum(void)
{
	int i;
	u16 mastercsum = 0;
	u8 *p;

	/* calculate CSUM from 2nd block thru # of user blocks */
	p = &cypress_firmware[FIRMWARE_BLOCK_SIZE];
	i = FIRMWARE_USER_BLOCKS * FIRMWARE_BLOCK_SIZE;

	while (i--)
		mastercsum += *p++;

	p = &cypress_firmware[sizeof(cypress_firmware) - 2];
	if (mastercsum != (u16)((*p << 8) | *(p + 1))) {
		printk(KERN_ERR "CY8CTMA300_TOUCH: firmware file checksum error\n");
		return 1;
	};

	return 0;
};

static int send_bootloader_cmd(struct spi_device *spi, u8 cmd)
{
	struct fw_packet packet;
	int err;

	memcpy(&packet.bl_keys, bootloader_keys, sizeof(bootloader_keys));
	packet.bl_cmd = cmd;
	err = reg_write(spi, TP_REG_FW, (u8 *) &packet,
		sizeof(bootloader_keys) + 1);
	if (err)
		printk(KERN_ERR
			"CY8CTMA300_TOUCH: send_bootloader_cmd write: %d\n",
			err);

	return err;
};

static int perform_reset(struct spi_device *spi)
{
	int i = 5000000;
	int loops = 0, cur_val, new_val = 0xFF;
	struct cypress_touch_platform_data	*pdata = spi->dev.platform_data;

	gpio_set_value(pdata->gpio_reset_pin, 0);
	udelay(100);
	gpio_set_value(pdata->gpio_reset_pin, 1);

	cur_val = gpio_get_value(pdata->gpio_irq_pin);
	do {
		while (i-- && (cur_val ==
			(new_val = gpio_get_value(pdata->gpio_irq_pin))))
			;

		DEBUG_PRINTK(KERN_INFO
			"CY8CTMA300_TOUCH: loop %d val %d remain %d\n",
			loops, cur_val, i);
		cur_val = new_val;

		/* successful reset is 2 successful pulses before timeout */
		if (++loops == 4) {
			mdelay(100);
			/* some old firmware ACKs with IRQ before
				really ready */
			if (shit_firmware)
				mdelay(2000);
			return 0;
		};
	} while (i > 0);

	/* loops == 2 means bootloader failed (usually checksum) and we're in
	BL mode (ret 1), application won't start.
	Anything else is a bad TP (ret -1) */
	return (loops == 2) ? 1 : -1;

};

static int update_firmware(struct spi_device *spi)
{
	u16 i;
	int err = 0;
	int tries = 0;

	if (verify_fw_checksum())
		return -EBADF;

retry:
	/* try doing application mode first */
	reg_write_byte(spi, TP_REG_HST_MODE, 0);
	mdelay(100);

	/* try entering bootloader mode */
	err = send_bootloader_cmd(spi, BL_CMD_BL_ENTER);
	if (err)
		return err;

	/* check status */
	err = bl_flash_mode(spi);
	if (err) {
		if (err < 0) {
			printk(KERN_ERR
				"CY8CTMA300_TOUCH: "
				"bootloader enter read: %d\n",
				err);
			return err;
		} else {
			printk(KERN_ERR
				"CY8CTMA300_TOUCH: "
				"bootloader enter status: %d\n",
				err);
			if (tries++ < 2) {
				printk(KERN_ERR "CY8CTMA300_TOUCH: Retrying\n");
				perform_reset(spi);
				goto retry;
			}
			return -EPROTO;
		};
	};

	printk(KERN_INFO "CY8CTMA300_TOUCH: loading firmware: please wait\n");
	dump_noisy = 0;
	/* try programming the device */
	for (i = 1; i < FIRMWARE_TOTAL_BLOCKS; i++) {

		/* if we've programmed all the Application blocks,
			program the last block */
		if (i > FIRMWARE_USER_BLOCKS)
			i = FIRMWARE_TOTAL_BLOCKS - 1;

		err = write_firmware_block(spi,
			&cypress_firmware[i * FIRMWARE_BLOCK_SIZE], i);
		if (err) {
			printk(KERN_ERR
				"CY8CTMA300_TOUCH: "
				"firmware write error %d\n", err);
			return err;
		};

	};
	dump_noisy = 1;

	/* Exiting BL mode no longer needed per Cypress;
		it performs the same as the reset below
	err = send_bootloader_cmd(spi, BL_CMD_BL_EXIT);
	*/

	if (!err) {
		printk(KERN_INFO "CY8CTMA300_TOUCH: loading firmware: done\n");
		err = perform_reset(spi);
	}

	return err;
};

static int query_chip(struct spi_device *spi)
{
	u8 j[16];
	u16 blver, appver;
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	struct cy8ctma300_touch *this = dev_get_drvdata(&spi->dev);

	reg_write_byte(spi, TP_REG_HST_MODE, TP_TM_BIT_BIST);
	mdelay(100);
	reg_read(spi, TP_REG_HST_MODE, j, 1);
	if (j[0] != TP_TM_BIT_BIST)
		return -EIO;

	reg_read(spi, TP_BISTR_BL_VERH, j, 2);
	printk(KERN_INFO
		"CY8CTMA300_TOUCH: Bootloader  FW Ver %d.%d\n", j[0], j[1]);
	blver = j[0] << 8 | j[1];
	reg_read(spi, TP_BISTR_AP_VERH, j, 2);
	printk(KERN_INFO
		"CY8CTMA300_TOUCH: Application FW Ver %d.%d\n", j[0], j[1]);
	appver = j[0] << 8 | j[1];
	this->fw_version = appver;

	reg_read(spi, TP_BISTR_UID_0, j, 2);
	printk(KERN_INFO
		"CY8CTMA300_TOUCH: Silicon Revision %c\n", ((j[0] == 0x05)
		&& (j[1] == 0x80)) ? 'D' : 'E');

	reg_read(spi, TP_BISTR_AP_IDH, j, 1);
	printk(KERN_INFO "CY8CTMA300_TOUCH: Customer ID %d\n", j[0]);

	reg_read(spi, TP_BISTR_AP_IDL, j, 1);

	this->tp_fw_type = j[0];

	if (TP_PANEL_TYPE_SSD == this->tp_fw_type)
		printk(KERN_INFO "CY8CTMA300_TOUCH: SSD firmware"
			" [Project ID %d]\n", this->tp_fw_type);

	if (TP_PANEL_TYPE_DSD == this->tp_fw_type)
		printk(KERN_INFO "CY8CTMA300_TOUCH: DSD firmware"
			" [Project ID %d]\n", this->tp_fw_type);

	/* when firmware is 87 or above, use SPI buffer valid check */
	this->use_spi_buffer_valid = (TP_SPI_BUFFER_VALID_VER <= appver);

	if (pdata->no_fw_update == 0) {
		if (appver < 58) {
			printk(KERN_INFO "CY8CTMA300_TOUCH: old FW version, applying reset workarounds\n");
			shit_firmware++;
		};

		DEBUG_PRINTK(KERN_INFO
		     "CY8CTMA300_TOUCH: about to compare FW versions\n");
		if ((blver == 1) && (appver != FW_VERSION_MINOR)) {
			force_update = 1;
			return 1;
		} else if (appver == FW_VERSION_MINOR) {
			printk(KERN_ERR "CY8CTMA300_TOUCH: firmware versions match.\n");
		};
	} else {
		printk(KERN_INFO "CY8CTMA300_TOUCH: no FW update allowed.\n");
	};

	DEBUG_PRINTK(KERN_INFO
		"CY8CTMA300_TOUCH: about to re-enter normal mode\n");
	/* now put chip back into Normal mode */
	reg_write_byte(spi, TP_REG_HST_MODE, 0);
	mdelay(100);
	reg_read(spi, TP_REG_HST_MODE, j, 1);
	if (j[0] != TP_REG_HST_MODE)
		return -EIO;

	return 0;
}
static void cy8ctma300_fwupd_work(struct work_struct *work)
{
	struct cy8ctma300_touch	*this =
		container_of(work, struct cy8ctma300_touch, fwupd_work);
	struct spi_device	*spi = this->spi;
	int			err;
	u8			j[16];

	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: cy8ctma300_fwupd_work()\n");

	mdelay(100);
	err = update_firmware(spi);
	if (err) {
		printk(KERN_ERR
			"CY8CTMA300_TOUCH: "
			"firmware update failed %d, not registering device\n",
			err);
		return;
	};

	/* try to put device into BIST mode */
	/* have to reset FW status byte first,
	  as current firmware locks otherwise */
	reg_read(spi, TP_REG_FW, j, 1);
	printk(KERN_INFO "CY8CTMA300_TOUCH: TP_REG_FW 0x%02X\n", j[0]);

	err = query_chip(spi);
	if (!err) {

		if (j[0] == TP_AP_BIT_APPMODE) {
			DEBUG_PRINTK(KERN_INFO
				"CY8CTMA300_TOUCH: have sane firmware rev now");
			if (shit_firmware) {
				DEBUG_PRINTK(", clearing bad-FW flag");
				shit_firmware = 0;
			};

			DEBUG_PRINTK("\n");
		};

		cy8ctma300_deferred_init(this);
	} else {
		printk(KERN_ERR
			"CY8CTMA300_TOUCH: cy8ctma300_fwupd_work():"
			" chip query failed %d\n", err);
		this->init_complete++;
	};
};

static int reset_device(struct cy8ctma300_touch *this)
{
	struct spi_device			*spi = this->spi;
	int					reset_retries = 2;
	int					err = 0;

	u8 j[16];

reset_chip:
	if ((perform_reset(spi) < 0) || !reset_retries--)
		return -ENOMEDIUM;

	/* check FW status and attempt to flash chip if necessary */
	reg_read(spi, TP_REG_FW, j, 1);
	printk(KERN_INFO "CY8CTMA300_TOUCH: TP_REG_FW 0x%02X\n", j[0]);

	/* check for the case of older firmware that's not quite ready yet */
	if (j[0] == 0xFF) {
		shit_firmware++;
		goto reset_chip;
	};

	if ((force_update-- > 0) || (j[0] & (TP_BL_BIT_BLRDY | TP_BL_BIT_BM)))
		return 1;

	err = query_chip(spi);
	if (err < 0) {
		shit_firmware++;
		goto reset_chip;
	};

	return err;
};

static void cy8ctma300_send_input(struct cy8ctma300_touch *this, u8 down,
				  u16 x, u16 y, u16 z, u16 id)
{
	DEBUG_PRINTK(KERN_INFO
		"CY8CTMA300_TOUCH: down %u x %u y %u id %u\n", down, x, y, id);

	if (down != 1)
		input_mt_sync(this->input);

	input_report_abs(this->input, ABS_MT_POSITION_X, x);
	input_report_abs(this->input, ABS_MT_POSITION_Y, y);
	input_report_abs(this->input, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(this->input, ABS_MT_TRACKING_ID, id - 1);

};

static void cy8ctma300_touch_isr(void *context)
{
	struct cy8ctma300_touch *this = (struct cy8ctma300_touch *)context;
	schedule_work(&this->isr_work);
}

static void cy8ctma300_isr_work(struct work_struct *work)
{
	struct cy8ctma300_touch	*this =
		container_of(work, struct cy8ctma300_touch, isr_work);
	struct cypress_touch_platform_data
		*pdata = this->spi->dev.platform_data;
	u8 read_buf[TOUCH_DATA_BYTES];
	u16 xp = 0, yp = 0, z = 0, id = 0;
	u8 down;

	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: cy8ctma300_touch_isr()\n");
	mutex_lock(&this->mutex);
	if (this->suspend) {
		mutex_unlock(&this->mutex);
		return;
	}

	memcpy(read_buf, this->async_read_buf, TOUCH_DATA_BYTES);
	dump_buf(read_buf, TOUCH_DATA_BYTES);

	if (read_buf[TP_REG_TT_MODE] & TP_TT_BIT_DINVAL) {
		printk(KERN_INFO "CY8CTMA300_TOUCH: invalid data?\n");
		goto out;
	};

	/*
	 * For SSD FW-87+ and DSD-FW 107+, check bitamsk in 3rd byte
	 * for 01 in bits [7:6] to accept the data
	 * i.e. if (BYTE[2] & 0xC0 == 0x40) accept data, else reject
	 */

	if (this->use_spi_buffer_valid) {
		if ((read_buf[TP_REG_TT_STAT] & TP_SPI_BUFFER_VALID_MASK) !=
		   TP_SPI_BUFFER_VALID) {
			dev_info(&this->spi->dev,
				 "CY8CTMA300_TOUCH: rejected data.\n");
			goto out;
		}
	}

	down = read_buf[TP_REG_TT_STAT] & TP_TT_BIT_TOUCH_MASK;
	xp = read_buf[TP_REG_PNT1XL] | (read_buf[TP_REG_PNT1XH] << 8);
	yp = read_buf[TP_REG_PNT1YL] | (read_buf[TP_REG_PNT1YH] << 8);
	yp = (pdata->y_max) - yp;
	z = read_buf[TP_REG_PNT1Z];
	id = read_buf[TP_REG_PNT12_ID] >> 4 & 0xF;

	input_report_key(this->input, BTN_TOUCH, down ? 1 : 0);

	if (down == 1) {
		input_report_abs(this->input, ABS_X, xp);
		input_report_abs(this->input, ABS_Y, yp);
		input_report_abs(this->input, ABS_PRESSURE, z);
	};

	if (down) {
		cy8ctma300_send_input(this, down, xp, yp, z, id);

		if (down > 1) {
			xp = read_buf[TP_REG_PNT2XL] |
				(read_buf[TP_REG_PNT2XH] << 8);
			yp = read_buf[TP_REG_PNT2YL] |
				(read_buf[TP_REG_PNT2YH] << 8);
			yp = (pdata->y_max) - yp;
			z = read_buf[TP_REG_PNT2Z];
			id = read_buf[TP_REG_PNT12_ID] & 0xF;
			cy8ctma300_send_input(this, down, xp, yp, z, id);
		};

		if (down > 2) {
			xp = read_buf[TP_REG_PNT3XL] |
				(read_buf[TP_REG_PNT3XH] << 8);
			yp = read_buf[TP_REG_PNT3YL] |
				(read_buf[TP_REG_PNT3YH] << 8);
			yp = (pdata->y_max) - yp;
			z = read_buf[TP_REG_PNT3Z];
			id = read_buf[TP_REG_PNT34_ID] >> 4 & 0xF;
			cy8ctma300_send_input(this, down, xp, yp, z, id);
		};

		if (down > 3) {
			xp = read_buf[TP_REG_PNT4XL] |
				(read_buf[TP_REG_PNT4XH] << 8);
			yp = read_buf[TP_REG_PNT4YL] |
				(read_buf[TP_REG_PNT4YH] << 8);
			yp = (pdata->y_max) - yp;
			z = read_buf[TP_REG_PNT4Z];
			id = read_buf[TP_REG_PNT34_ID] & 0xF;
			cy8ctma300_send_input(this, down, xp, yp, z, id);
		};
	};

	input_mt_sync(this->input);
	input_sync(this->input);

#ifdef MEASURE_PERFORMANCE
	if (this->dupe_x == xp && this->dupe_y == yp &&
		this->dupe_touch == (down ? 1 : 0))
		++this->measure_dupes;

	this->dupe_x = xp;
	this->dupe_y = yp;
	this->dupe_touch = (down ? 1 : 0);
#endif

	DEBUG_PRINTK(KERN_INFO
		"CY8CTMA300_TOUCH: touch %s at (%3d, %3d)\n",
		down ? "down" : "up  ", xp, yp);
#ifdef MEASURE_PERFORMANCE
	if (this->measure_start < jiffies) {
		printk(KERN_INFO
			"CY8CTMA300_TOUCH: "
			"%d interrupts/events per second and %d dupes\n",
			this->measure_count, this->measure_dupes);
		this->measure_start = jiffies + msecs_to_jiffies(1000);
		this->measure_count = 0;
		this->measure_dupes = 0;
	}
	++this->measure_count;
#endif

out:
	mutex_unlock(&this->mutex);
	ENABLE_IRQ(this);
};

static irqreturn_t cy8ctma300_touch_irq(int irq, void *handle)
{
	struct cy8ctma300_touch *this = handle;
	unsigned long flags;

	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: cy8ctma300_touch_irq()\n");

	/* first IRQ is side-effect of reset operation */
	if (!this->first_irq++)
		return IRQ_HANDLED;

	DISABLE_IRQ(this);
	spin_lock_irqsave(&this->lock, flags);
	reg_read_intcause_async(this->spi, cy8ctma300_touch_isr);
	spin_unlock_irqrestore(&this->lock, flags);

	return IRQ_HANDLED;
};

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_PM
static int cy8ctma300_touch_suspend(struct spi_device *spi,
	pm_message_t message)
{
	struct cy8ctma300_touch *this = dev_get_drvdata(&spi->dev);

	DEBUG_PRINTK(KERN_INFO "CY8CTMA300_TOUCH: %s()\n", __func__);
	DISABLE_IRQ(this);

	mutex_lock(&this->mutex);
	this->suspend = 1;
	mutex_unlock(&this->mutex);

	return reg_write_byte(spi, TP_REG_HST_MODE, TP_TM_BIT_DEEPSLEEP);
}

static int cy8ctma300_touch_resume(struct spi_device *spi)
{
	int rc;
	u8 data;
	struct cy8ctma300_touch	*this = dev_get_drvdata(&spi->dev);
	DEBUG_PRINTK(KERN_INFO "CY8CTMA300_TOUCH: %s()\n", __func__);

	this->suspend = 0;
	ENABLE_IRQ(this);

	rc = reg_write_byte(spi, TP_REG_HST_MODE, 0);
	mdelay(100);
	reg_read(spi, TP_REG_HST_MODE, &data, 1);
	if (data != TP_REG_HST_MODE)
		return -ENODEV;

	mutex_lock(&this->mutex);
	if (this->nsb_work_todo)
		schedule_work(&this->charger_work);
	mutex_unlock(&this->mutex);

	return rc;
};
#endif

#if defined(CONFIG_EARLYSUSPEND) && defined(CONFIG_PM)
static void cy8ctma300_touch_early_suspend(struct early_suspend *es)
{
	struct cy8ctma300_touch *this;
	this = container_of(es, struct cy8ctma300_touch, early_suspend);

	DEBUG_PRINTK(KERN_INFO "CY8CTMA300_TOUCH: %s()\n", __func__);
	dev_dbg(&this->spi->dev, "CY8CTMA300_TOUCH: early suspend\n");

	cy8ctma300_touch_suspend(this->spi, PMSG_SUSPEND);
};

static void cy8ctma300_touch_late_resume(struct early_suspend *es)
{
	struct cy8ctma300_touch *this;
	this = container_of(es, struct cy8ctma300_touch, early_suspend);

	DEBUG_PRINTK(KERN_INFO "CY8CTMA300_TOUCH: %s()\n", __func__);
	dev_dbg(&this->spi->dev, "CY8CTMA300_TOUCH: late resume\n");

	cy8ctma300_touch_resume(this->spi);
};
#endif /* #ifdef CONFIG_EARLYSUSPEND */

static int cy8ctma300_touch_open(struct inode *inode, struct file *file)
{
	struct cy8ctma300_touch *this =
		container_of(inode->i_cdev, struct cy8ctma300_touch,
		device_cdev);
	file->private_data = this;
	return 0;
};

static int cy8ctma300_touch_release(struct inode *inode, struct file *file)
{
	return 0;
};

static ssize_t cy8ctma300_touch_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int err = 0;

	switch (cmd) {

	default:
		printk(KERN_ERR "CY8CTMA300_TOUCH: ioctl, cmd error\n");
		return -EINVAL;
		break;
	}

	return err;
};

static void charger_noise_suppression(struct cy8ctma300_touch *this,
					struct spi_device *spi)
{
	u8 j[1];
	int count = 0;
	int reg_is_valid = 0;
	int cmd;

	mutex_lock(&this->mutex);

	cmd = this->nsb_new_state ? 3 : 2;

	if ((this->suspend) || (!this->has_been_initialized)) {
		this->nsb_work_todo = 1;
		this->nsb_new_state = (cmd == 3) ? 1 : 0;
		mutex_unlock(&this->mutex);
		return;
	}

	DISABLE_IRQ(this);

	if (cmd == this->nsb_state) {
		this->nsb_work_todo = 0;
		ENABLE_IRQ(this);
		mutex_unlock(&this->mutex);
		return; /* nothing to do */
	}

	if (this->fw_version < TP_FW_NSB_VALID_VER) {
		this->nsb_work_todo = 0;
		ENABLE_IRQ(this);
		mutex_unlock(&this->mutex);
		return;
	}

	/*
	 * Undocumented:
	 * In SYSTEM INFO mode register with an offset [0x06]
	 * defines charger state.  By default its value is 0
	 * which means it is not initialized and just does normal scan.
	 * To enable Charger Noise Suppression do the following:
	 * Switch to SYS INFO MODE: write 0x10 to register 0x00
	 * Write 0x03 value in reg [0x06], write value 0x00 to reg
	 * [0x02] and switch back to operational mode by setting
	 * correspondent bits in reg [0x00].
	 *
	 * The same operation should be performed to clear bit but
	 * the value 0x02 should be written to reg [0x06] in SYS INFO MODE.
	 * The action is taken when mode is switched back to operational
	 * from SYS INFO.
	 *
	 * Cypress requested this be done in a while loop and added a verify bit
	 */
	do {
		reg_write_byte(spi, TP_REG_HST_MODE, 0x10);
		msleep(100);
		/* set or clear charger noise suppression bit */
		reg_write_byte(spi, 0x06, cmd);
		reg_write_byte(spi, 0x02, 0x00);
		reg_write_byte(spi, TP_REG_HST_MODE, 0x00);
		msleep(100);
		/* readback register 0x1B indicates bit set */
		reg_read(spi, 0x1B, j, 1);
		if (cmd == j[0]) {
			reg_is_valid = 1;
			this->nsb_state = cmd;
			this->nsb_work_todo = 0;
			break;
		}
		count++;
	} while ((count < 16) && (!reg_is_valid)); /* 16 per Cypress */
	if (!reg_is_valid)
		dev_err(&spi->dev, "%s: noise filter bit write failed.\n",
				__func__);
	else
		dev_info(&spi->dev, "%s: charger noise filter %s\n",
			__func__, cmd == 3 ? "enabled" : "disabled");

	ENABLE_IRQ(this);
	mutex_unlock(&this->mutex);
	return;
	}

static const struct file_operations cy8ctma300_touch_fops = {
	.owner   = THIS_MODULE,
	.open    = cy8ctma300_touch_open,
	.ioctl   = cy8ctma300_touch_ioctl,
	.release = cy8ctma300_touch_release,
};

static int cy8ctma300_deferred_init(struct cy8ctma300_touch *this)
{

	struct input_dev			*input_dev;
	struct spi_device			*spi = this->spi;
	struct cypress_touch_platform_data	*pdata = spi->dev.platform_data;
	dev_t					device_t = MKDEV(0, 0);
	struct device				*class_dev_t = NULL;
	int					err = 0;

	if (this->has_been_initialized)
		return 0;

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_cleanup_mem;
	}
	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Allocated input device\n");

	this->input = input_dev;

	input_dev->name = "cy8ctma300_touch";
	input_dev->phys = "cy8ctma300_touch/input0";
	input_dev->dev.parent = &spi->dev;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);

	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_2, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(this->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	set_bit(ABS_MT_TRACKING_ID, this->input->absbit);

	/* Single-Touch stuff */
	input_set_abs_params(input_dev, ABS_X, pdata->x_min,
		pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, pdata->y_min,
		pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	err = input_register_device(input_dev);
	if (err)
		goto err_cleanup_mem;

	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Registered input device\n");

	err = alloc_chrdev_region(&device_t, 0, 1, "cy8ctma300_touch");
	if (err)
		goto err_cleanup_input;

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: Allocated character device\n");

	this->device_major = MAJOR(device_t);

	cdev_init(&(this->device_cdev), &cy8ctma300_touch_fops);
	this->device_cdev.owner = THIS_MODULE;
	this->device_cdev.ops = &cy8ctma300_touch_fops;
	err = cdev_add(&(this->device_cdev), MKDEV(this->device_major, 0), 1);
	if (err)
		goto err_cleanup_chrdev;

	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Character device added\n");

	this->device_class = class_create(THIS_MODULE, "cy8ctma300_touch");
	if (IS_ERR(this->device_class)) {
		err = -1;
		goto err_cleanup_cdev;
	};
	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Class created\n");

	class_dev_t = device_create(this->device_class, NULL,
		MKDEV(this->device_major, 0), NULL, "cy8ctma300_touch");
	if (IS_ERR(class_dev_t)) {
		err = -1;
		goto err_cleanup_class;
	}
	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Device created\n");

#ifdef CONFIG_EARLYSUSPEND
	/* register early suspend */
	this->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	this->early_suspend.suspend = cy8ctma300_touch_early_suspend;
	this->early_suspend.resume = cy8ctma300_touch_late_resume;
	register_early_suspend(&this->early_suspend);
#endif /* #ifdef CONFIG_EARLYSUSPEND */

	err = request_irq(pdata->irq, cy8ctma300_touch_irq, pdata->irq_polarity,
		this->spi->dev.driver->name, this);
	if (err) {
		dev_err(&this->spi->dev, "irq %d busy?\n", pdata->irq);
		goto err_cleanup_device;
	};
	/* IRQs are enabled as a side-effect of requesting */
	this->irq_suspend_enabled++;
	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Registered IRQ\n");

	/* Set the gesture active distance */
	reg_write_byte(this->spi, TP_REG_GEST_SET, TP_GEST_ACTD);

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: cy8ctma300_deferred_init() ok\n");
	this->init_complete++;
	this->has_been_initialized = 1;

	mutex_lock(&this->mutex);
	if (this->nsb_work_todo) {
		if (!this->fw_version)
			query_chip(spi);
		schedule_work(&this->charger_work);
	}
	mutex_unlock(&this->mutex);

	return 0;

err_cleanup_device:
#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&this->early_suspend);
#endif
	device_destroy(this->device_class, MKDEV(this->device_major, 0));
err_cleanup_class:
	class_destroy(this->device_class);
err_cleanup_cdev:
	cdev_del(&(this->device_cdev));
err_cleanup_chrdev:
	unregister_chrdev_region(device_t, 1);
err_cleanup_input:
	input_unregister_device(input_dev);
err_cleanup_mem:
	if (input_dev)
		input_free_device(input_dev);

	return err;
};

static void cy_callback(void *d, int on)
{
	struct cy8ctma300_touch *this;
	this = container_of(d, struct cy8ctma300_touch, cb_struct);
	this->nsb_new_state = on;
	schedule_work(&this->charger_work);
};

static void cy8ctma300_charger_work(struct work_struct *work)
{
	struct cy8ctma300_touch *this =
	container_of(work, struct cy8ctma300_touch,
			charger_work);

	charger_noise_suppression(this, this->spi);
};


static int cy8ctma300_touch_probe(struct spi_device *spi)
{
	struct cypress_touch_platform_data	*pdata = spi->dev.platform_data;
	struct cy8ctma300_touch			*this;
	int					err;

	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: cy8ctma300_touch_probe()\n");

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	};

	if (!pdata->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	/* Set up SPI */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	printk(KERN_DEBUG
		"CY8CTMA300_TOUCH: SPI setup (requesting %uHz) OK\n",
		spi->max_speed_hz);

	/* GPIO: set up */
	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: Requesting GPIO Reset ownership\n");
	err = gpio_request(pdata->gpio_reset_pin,
		"cy8ctma300_touch_gpio_reset");
	if (err)
		goto err_gpio_setup;

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: Requesting GPIO IRQ ownership\n");
	err = gpio_request(pdata->gpio_irq_pin, "cy8ctma300_touch_gpio_irq");
	if (err)
		goto err_gpio_setup;

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: Configuring GPIO Reset direction\n");
	err = gpio_direction_output(pdata->gpio_reset_pin, 0);
	if (err)
		goto err_gpio_setup;

	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: Configuring GPIO IRQ direction\n");
	err = gpio_direction_input(pdata->gpio_irq_pin);
	if (err)
		goto err_gpio_setup;

	this = kzalloc(sizeof(struct cy8ctma300_touch), GFP_KERNEL);
	if (!this) {
		err = -ENOMEM;
		goto err_gpio_setup;
	};
	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Allocated private data\n");

	this->pdata = pdata;
	this->spi = spi;
	spin_lock_init(&this->lock);
	mutex_init(&this->mutex);
	dev_set_drvdata(&spi->dev, this);
	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Driver data set\n");

	if (pdata->register_cb) {
		this->cb_struct.cb = cy_callback;
		pdata->register_cb(&this->cb_struct);
	}

	/* register firmware validation and initialization to workqueue */
	INIT_WORK(&this->fwupd_work, cy8ctma300_fwupd_work);
	INIT_WORK(&this->charger_work, cy8ctma300_charger_work);
	INIT_WORK(&this->isr_work, cy8ctma300_isr_work);
	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: Firmware-update work queue init OK\n");

	DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: About to reset device\n");
	err = reset_device(this);
	if (err < 0)
		goto err_mem_free;

	if (!err) {
		DEBUG_PRINTK(KERN_DEBUG "CY8CTMA300_TOUCH: Chip reset OK\n");
		err = cy8ctma300_deferred_init(this);
		if (err) {
			goto err_mem_free;
		} else {
			printk(KERN_INFO "CY8CTMA300_TOUCH: Device registered OK\n");
			return 0;
		}
	} else {
		printk(KERN_INFO "CY8CTMA300_TOUCH: Deferring device init post-FW Update\n");
		schedule_work(&this->fwupd_work);
		return 0;
	};

err_mem_free:
	kfree(this);
err_gpio_setup:
	gpio_free(pdata->gpio_reset_pin);
	gpio_free(pdata->gpio_irq_pin);
	printk(KERN_ERR "CY8CTMA300_TOUCH: probe() fail: %d\n", err);
	return err;
};

static int __devexit cy8ctma300_touch_remove(struct spi_device *spi)
{
	struct cy8ctma300_touch	*this = dev_get_drvdata(&spi->dev);
	struct cypress_touch_platform_data	*pdata = spi->dev.platform_data;
	dev_t device_t = MKDEV(this->device_major, 0);

	DEBUG_PRINTK(KERN_INFO "CY8CTMA300_TOUCH: unregistering touchscreen\n");

	if (!this)
		return -ENODEV;

	gpio_free(pdata->gpio_reset_pin);
	gpio_free(pdata->gpio_irq_pin);

#ifdef CONFIG_PM
	cy8ctma300_touch_suspend(spi, PMSG_SUSPEND);
#endif

	if (!this->init_complete) {
		printk(KERN_ERR "CY8CTMA300_TOUCH: unregistering partially-loaded driver\n");
		return -EBUSY;
	};

#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&this->early_suspend);
#endif

	if (this->input)
		input_unregister_device(this->input);
	if (this->device_class) {
		device_destroy(this->device_class,
			MKDEV(this->device_major, 0));
		class_destroy(this->device_class);
	};
	if (&this->device_cdev) {
		cdev_del(&(this->device_cdev));
		unregister_chrdev_region(device_t, 1);
	};
	free_irq(pdata->irq, this);
	kfree(this);

	dev_dbg(&spi->dev, "CY8CTMA300_TOUCH: unregistered touchscreen\n");

	return 0;
};

static struct spi_driver cy8ctma300_touch_driver = {
	.driver = {
		.name	= "cypress_touchscreen",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= cy8ctma300_touch_probe,
	.remove		= __devexit_p(cy8ctma300_touch_remove),
#ifndef CONFIG_PM
	.suspend	= cy8ctma300_touch_suspend,
	.resume		= cy8ctma300_touch_resume,
#endif
};

static int __init cy8ctma300_touch_init(void)
{
	int err;

	printk(KERN_DEBUG
		"CY8CTMA300_TOUCH: V%s built %s %s\n",
		MODULE_VER, __DATE__, __TIME__);
	err = spi_register_driver(&cy8ctma300_touch_driver);
	DEBUG_PRINTK(KERN_DEBUG
		"CY8CTMA300_TOUCH: module init, result=%d\n", err);

	return err;
};

static void __exit cy8ctma300_touch_exit(void)
{
	spi_unregister_driver(&cy8ctma300_touch_driver);
	printk(KERN_DEBUG "CY8CTMA300_TOUCH: module exit\n");
};

module_init(cy8ctma300_touch_init);
module_exit(cy8ctma300_touch_exit);

MODULE_DESCRIPTION("Touchscreen driver for Cypress CY8CTMA300 hardware");
MODULE_LICENSE("GPL");
