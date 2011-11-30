/*
 * linux/drivers/power/bq27520_battery.c
 *
 * TI BQ27520 Fuel Gauge interface
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Authors: James Jacobsson <james.jacobsson@sonyericsson.com>
 *          Imre Sunyi <imre.sunyi@sonyericsson.com>
 *          Hiroyuki Namba <Hiroyuki.Namba@sonyericsson.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <asm/atomic.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/err.h>

#include <linux/i2c/bq27520_battery.h>

#include <asm/mach-types.h>

#define REG_CMD_CONTROL 0x00
#define REG_CMD_TEMPERATURE 0x06
#define REG_CMD_VOLTAGE 0x08
#define REG_CMD_FLAGS 0x0A
#define REG_CMD_SOC 0x2C
#define REG_CMD_AVG_CURRENT 0x14
#define REG_CMD_INS_CURRENT 0x30
#define REG_CMD_APPSTATUS 0x6A

#define REG_EXT_CMD_DATA_FLASH_CLASS 0x3E
#define REG_EXT_CMD_DATA_FLASH_BLOCK 0x3F
#define REG_EXT_CMD_BLOCK_DATA 0x40
#define REG_EXT_CMD_BLOCK_DATA_CONTROL 0x61
#define REG_EXT_CMD_DATA_FLASH_CLASS_MF 0x39 /* Subclass ID 57 */
#define REG_EXT_CMD_DATA_FLASH_CLASS_IT 0x52 /* Subclass ID 82 */

#define SUB_CMD_NULL 0x0000
#define SUB_CMD_FW_VERSION 0x0002
#define SUB_CMD_OCV_CMD 0x000C
#define SUB_CMD_BAT_INSERT 0x000D
#define SUB_CMD_BAT_REMOVE 0x000E
#define SUB_CMD_IT_ENABLE 0x0021
#define SUB_CMD_CHOOSE_A 0x0024
#define SUB_CMD_CHOOSE_B 0x0025
#define SUB_CMD_RESET 0x0041
#define SUB_CMD_ENTER_CLEAR_SEAL 0x0414
#define SUB_CMD_CLEAR_SEALED 0x3672
#define SUB_CMD_CLEAR_FULL_ACCESS_SEALED 0xFFFF

#define WAIT_ON_READ_SUB_CMD_US 100
/* According to datasheet this is the maximum time to program a
 * word in data flash memory.
 */
#define MAX_WORD_PROGRAMMING_TIME_MS 2

#define FC_MASK 0x0200
#define BAT_DET_MASK 0x0008
#define SYSDOWN_MASK 0x2
#define LU_PROF_MASK 0x1
#define INIT_COMP_MASK 0x80
#define SEALED_MASK 0x6000
#define QEN_MASK 0x01

#define RETRY_MAX 5
#define FAKE_CAPACITY_BATT_ALIEN 50

#define READ_FC_TIMER 10
#define OCV_CMD_TIMER 2

#define TEMP_WRITE_TIMEOUT_MS 2000
#define A_TEMP_COEF_DEFINE 2731

#define BITMASK_16 0xffff

#define USB_CHG  0x01
#define WALL_CHG 0x02

/* OCV measurement takes 2sec in device spec. */
#define DELAY_TIME_BEFORE_OCV_ISSUE 2000

/* OCV command execution is defined as 1.2 seconds in device spec. */
#define DELAY_TIME_AFTER_OCV_ISSUE 1300

#define POLL_QEN_TIMEOUT_MS 2000
#define POLL_QEN_PERIOD_MS 100

/* CONTANTS / MACROS */
#define I2C_RETRY_MAX 3			/* retry 3 times */
#define I2C_RETRY_DELAY_MS 1		/* delay in ms */
#define FUEL_GAUGE_ROM_SLAVE_ADDR 0x0B	/* 7-bit slave addr, ROM mode */
/* REGISTER ADDRESS */
#define ENTER_ROM_REG 0x00
#define ENTER_ROM_DATA 0x0F00
#define LEAVE_ROM_REG1 0x00
#define LEAVE_ROM_DATA1 0x0F
#define LEAVE_ROM_REG2 0x64
#define LEAVE_ROM_DATA2 0x0F
#define LEAVE_ROM_REG3 0x65
#define LEAVE_ROM_DATA3 0x00
#define LEAVE_ROM_DELAY_MS 4000

#define IT_ENABLE_DELAY_MS 500

/* #define DEBUG_FS */

/* Parameter update support.
 * Should not be necessary since golden file has same setting.
 */
/* #define SUPPORT_PARAMETER_UPDATE */

#ifdef DEBUG_FS
struct override_value {
	u8 active;
	int value;
};
#endif

struct bq27520_golden_info {
	u16 fw_compatible_version;
	u16 golden_file_version;
	char project_name[4];
};

struct bq27520_data {
	struct power_supply bat_ps;
	struct i2c_client *clientp;
	struct i2c_client *rom_clientp;
	int curr_mv;
	int curr_capacity;
	int curr_capacity_level;
	int curr_current;
	struct bq27520_platform_data *pdata;
	struct work_struct ext_pwr_change_work;
	struct work_struct soc_int_work;
	struct work_struct init_work;
	struct delayed_work fc_work;
	struct workqueue_struct *wq;
	int current_avg;
	int impedance;
	int flags;
	int technology;
	int bat_temp;
	int control_status;
	int app_status;
	int chg_connected;
	struct mutex lock;
	struct mutex int_lock;
	struct mutex data_flash_lock;
	int got_technology;
	int lipo_bat_max_volt;
	int lipo_bat_min_volt;
	unsigned char capacity_scaling[2];
	char *battery_dev_name;
	char *set_batt_charged_dev_name;
	int started_worker;
	int polling_lower_capacity;
	int polling_upper_capacity;
	int ocv_issue_capacity_threshold;
	u8 suspended;
	u8 resume_int;
	bool force_ocv;
	s8 sealed;
	bool run_init_after_rom;
	struct bq27520_block_table *udatap;

#ifdef DEBUG_FS
	struct override_value bat_volt_debug;
	struct override_value bat_curr_debug;
	struct override_value bat_cap_debug;
	struct override_value bat_cap_lvl_debug;
#endif
};

static atomic_t bq27520_init_ok = ATOMIC_INIT(0);

static int get_supplier_data(struct device *dev, void *data);
static int bq27520_read_control_status(struct bq27520_data *bd);
static int bq27520_write_control(struct bq27520_data *bd, int subcmd);
static int bq27520_read_bat_flags(struct power_supply *bat_ps);

#ifdef DEBUG_FS
static int read_sysfs_interface(const char *pbuf, s32 *pvalue, u8 base)
{
	long long val;
	int rc;

	rc = strict_strtoll(pbuf, base, &val);
	if (!rc)
		*pvalue = (s32)val;

	return rc;
}

static ssize_t store_voltage(struct device *pdev, struct device_attribute *attr,
			     const char *pbuf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(pdev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	int rc = count;
	s32 mv;

	if (!read_sysfs_interface(pbuf, &mv, 10) &&
	    mv >= -1 && mv <= INT_MAX) {
		mutex_lock(&bd->lock);

		bd->bat_volt_debug.active = 0;

		if (mv >= 0) {
			bd->bat_volt_debug.active = 1;
			bd->bat_volt_debug.value = mv;
		}

		mutex_unlock(&bd->lock);

		power_supply_changed(&bd->bat_ps);
	} else {
		pr_err("%s: Wrong input to sysfs set_voltage. "
		       "Expect [-1..%d]. -1 releases the debug value\n",
		       BQ27520_NAME, INT_MAX);
		rc = -EINVAL;
	}

	return rc;
}
static ssize_t store_current(struct device *pdev, struct device_attribute *attr,
			     const char *pbuf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(pdev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	int rc = count;
	s32 curr;

	if (!read_sysfs_interface(pbuf, &curr, 10) &&
	    curr >= -4001 && curr <= INT_MAX) {
		mutex_lock(&bd->lock);

		bd->bat_curr_debug.active = 0;

		if (curr >= -4000) {
			bd->bat_curr_debug.active = 1;
			bd->bat_curr_debug.value = curr;
		}

		mutex_unlock(&bd->lock);

		power_supply_changed(&bd->bat_ps);
	} else {
		pr_err("%s: Wrong input to sysfs set_current. "
		       "Expect [-4001..%d]. -4001 releases the debug value\n",
		       BQ27520_NAME, INT_MAX);
		rc = -EINVAL;
	}

	return rc;
}

static ssize_t store_capacity(struct device *pdev,
			      struct device_attribute *attr, const char *pbuf,
			      size_t count)
{
	struct power_supply *psy = dev_get_drvdata(pdev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	int rc = count;
	s32 cap;

	if (!read_sysfs_interface(pbuf, &cap, 10) &&
	    cap >= -1 && cap <= 100) {
		mutex_lock(&bd->lock);

		bd->bat_cap_debug.active = 0;

		if (cap >= 0) {
			bd->bat_cap_debug.active = 1;
			bd->bat_cap_debug.value = cap;
		}

		mutex_unlock(&bd->lock);

		power_supply_changed(&bd->bat_ps);
	} else {
		pr_err("%s: Wrong input to sysfs set_capacity. "
		       "Expect [-1..100]. -1 releases the debug value\n",
		       BQ27520_NAME);
		rc = -EINVAL;
	}

	return rc;
}

static ssize_t store_capacity_level(struct device *pdev,
			      struct device_attribute *attr, const char *pbuf,
			      size_t count)
{
	struct power_supply *psy = dev_get_drvdata(pdev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	int rc = count;
	s32 lvl;

	if (!read_sysfs_interface(pbuf, &lvl, 10) &&
	    lvl >= -1 && lvl <= POWER_SUPPLY_CAPACITY_LEVEL_FULL) {
		mutex_lock(&bd->lock);

		bd->bat_cap_lvl_debug.active = 0;

		if (lvl >= 0) {
			bd->bat_cap_lvl_debug.active = 1;
			bd->bat_cap_lvl_debug.value = lvl;
		}

		mutex_unlock(&bd->lock);

		power_supply_changed(&bd->bat_ps);
	} else {
		pr_err("%s: Wrong input to sysfs set_capacity_level. "
		       "Expect [-1..%u]. -1 releases the debug value\n",
		       BQ27520_NAME, POWER_SUPPLY_CAPACITY_LEVEL_FULL);
		rc = -EINVAL;
	}

	return rc;
}
#endif /* DEBUG_FS */

static ssize_t show_capacity(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	int capacity;

	/* Might end up here during ROM mode.
	 * Android shutsdown at 0% capacity.
	 * Prevent that by never reporting 0.
	 */
	if (bd->rom_clientp || bd->run_init_after_rom) {
		if (!bd->curr_capacity)
			capacity = 1;
		else
			capacity = bd->curr_capacity;
		return scnprintf(buf, PAGE_SIZE, "%d\n", capacity);
	} else if (!atomic_read(&bq27520_init_ok)) {
		return -EBUSY;
	}

	mutex_lock(&bd->lock);
	if (bd->capacity_scaling[0] == bd->capacity_scaling[1]) {
		capacity = bd->curr_capacity;
	} else {
		capacity = min(100,
			       (bd->curr_capacity * bd->capacity_scaling[0] +
				(bd->capacity_scaling[1] >> 1)) /
			       bd->capacity_scaling[1]);
		pr_debug("%s: Report scaled cap %d (origin %d)\n",
			 BQ27520_NAME, capacity, bd->curr_capacity);
	}
#ifdef DEBUG_FS
	if (bd->bat_cap_debug.active)
		capacity = bd->bat_cap_debug.value;
#endif
	mutex_unlock(&bd->lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", capacity);
}

/* Wrapper to i2c_smbus_read_i2c_block_data().
 * This one makes sure all 'length' bytes are read.
 */
static s32 safe_i2c_smbus_read_i2c_block_data(struct i2c_client *client,
					      u8 command, u8 length, u8 *values)
{
	s32 rc;
	u8 offs = 0;
	u8 retry = 0;
	u8 size;

	while (offs < length && retry <= I2C_RETRY_MAX) {
		if ((length - offs) > I2C_SMBUS_BLOCK_MAX)
			size = I2C_SMBUS_BLOCK_MAX;
		else
			size = length - offs;

		rc = i2c_smbus_read_i2c_block_data(client,
						   (u8)(command + offs),
						   size, values + offs);
		if (rc <= 0) {
			retry++;
			msleep(I2C_RETRY_DELAY_MS * retry);
		} else {
			retry = 0;
			offs += rc;
		}
	}

	return (retry > I2C_RETRY_MAX) ? -EPERM : offs;
}


static bool bq27520_check_if_sealed(struct bq27520_data *bd)
{
	if (bd->sealed == -1) {
		bq27520_write_control(bd, SUB_CMD_NULL);
		usleep(WAIT_ON_READ_SUB_CMD_US);
		bq27520_read_control_status(bd);
		bd->sealed = !!(bd->control_status & SEALED_MASK);
	}

	return bd->sealed;
}

static int bq27520_unseal(struct bq27520_data *bd)
{
	int rc = bq27520_write_control(bd, SUB_CMD_ENTER_CLEAR_SEAL);
	if (!rc)
		rc = bq27520_write_control(bd, SUB_CMD_CLEAR_SEALED);
	if (!rc)
		rc = bq27520_write_control(bd,
					   SUB_CMD_CLEAR_FULL_ACCESS_SEALED);
	if (!rc)
		rc = bq27520_write_control(bd,
					   SUB_CMD_CLEAR_FULL_ACCESS_SEALED);

	bd->sealed = (!rc) ? 0 : -1;

	return rc;
}

static int bq27520_get_fw_version(struct i2c_client *client,
				  u16 *fw_ver)
{
	s32 rc = i2c_smbus_write_word_data(client,
					   REG_CMD_CONTROL,
					   SUB_CMD_FW_VERSION);
	if (rc < 0)
		return rc;

	usleep(WAIT_ON_READ_SUB_CMD_US);
	rc = i2c_smbus_read_word_data(client, REG_CMD_CONTROL);
	if (rc < 0)
		return rc;

	*fw_ver = rc;

	return 0;
}

static int bq27520_setup_to_read_df_class_block(struct bq27520_data *bd,
						u8 class, u8 block)
{
	s32 rc;

	rc = i2c_smbus_write_byte_data(bd->clientp,
				       REG_EXT_CMD_BLOCK_DATA_CONTROL,
				       0x00);
	if (!rc) {
		msleep(MAX_WORD_PROGRAMMING_TIME_MS);
		rc = i2c_smbus_write_byte_data(bd->clientp,
					       REG_EXT_CMD_DATA_FLASH_CLASS,
					       class);
	}

	if (!rc) {
		msleep(MAX_WORD_PROGRAMMING_TIME_MS);
		rc = i2c_smbus_write_byte_data(bd->clientp,
					       REG_EXT_CMD_DATA_FLASH_BLOCK,
					       block);
	}

	if (!rc)
		msleep(MAX_WORD_PROGRAMMING_TIME_MS);

	return rc;
}

static int bq27520_get_golden_info(struct bq27520_data *bd,
				   struct bq27520_golden_info *gi)
{
	s32 rc;

	mutex_lock(&bd->data_flash_lock);

	/* Access Manufacturer Info block A.
	 * Two methods access depending if UNSEALED or SEALED
	 */
	if (bq27520_check_if_sealed(bd)) {
		pr_debug("%s: %s(): SEALED\n", BQ27520_NAME, __func__);
		rc = i2c_smbus_write_byte_data(bd->clientp,
					       REG_EXT_CMD_DATA_FLASH_BLOCK,
					       0x01);
		msleep(MAX_WORD_PROGRAMMING_TIME_MS);
	} else {
		pr_debug("%s: %s(): UNSEALED\n", BQ27520_NAME, __func__);
		rc = bq27520_setup_to_read_df_class_block(bd,
				  REG_EXT_CMD_DATA_FLASH_CLASS_MF, 0x00);
	}

	if (!rc) {
		rc = safe_i2c_smbus_read_i2c_block_data(bd->clientp,
							REG_EXT_CMD_BLOCK_DATA,
							sizeof(*gi), (u8 *)gi);

		/* Watch out. 'rc' here holds the number of bytes read. */
		if (rc == sizeof(*gi)) {
#ifdef DEBUG
			unsigned int i;
			for (i = 0; i < sizeof(*gi); i++)
				pr_debug("%s: Block A[%u]: 0x%.2x\n",
					 BQ27520_NAME,
					 i, *((unsigned char *)gi + i));
#endif
			rc = 0;
			/* Version is stored in big endian format in register.
			 * Convert it to little endian here.
			 */
			gi->fw_compatible_version =
				ror16(gi->fw_compatible_version, 8);
			gi->golden_file_version =
				ror16(gi->golden_file_version, 8);
		} else if (rc >= 0) {
			rc = -ENOMSG;
		}
	}

	mutex_unlock(&bd->data_flash_lock);

	if (rc)
		pr_err("%s: Failed get golden info. rc=%d\n", BQ27520_NAME, rc);

	return rc;
}

static int bq27520_read_it_enabled(struct bq27520_data *bd)
{
	s32 rc;

	if (bq27520_check_if_sealed(bd)) {
		rc = bq27520_unseal(bd);

		if (rc) {
			pr_err("%s: Failed unseal when checking IT Enable. "
			       "rc=%d\n", BQ27520_NAME, rc);
			return rc;
		}
	}

	mutex_lock(&bd->data_flash_lock);
	rc = bq27520_setup_to_read_df_class_block(bd,
				  REG_EXT_CMD_DATA_FLASH_CLASS_IT, 0x00);

	if (!rc)
		rc = i2c_smbus_read_byte_data(bd->clientp,
					      REG_EXT_CMD_BLOCK_DATA);

	mutex_unlock(&bd->data_flash_lock);


	return rc;
}

static int bq27520_make_sure_bat_is_removed(struct bq27520_data *bd)
{
	const unsigned int poll_cnt = POLL_QEN_TIMEOUT_MS / POLL_QEN_PERIOD_MS;
	int rc = bq27520_read_bat_flags(&bd->bat_ps);
	unsigned int i;

	if (!rc && bd->flags & BAT_DET_MASK) {
		pr_info("%s: Writing BAT_REMOVE\n", BQ27520_NAME);
		bq27520_write_control(bd, SUB_CMD_BAT_REMOVE);

		for (i = 0; i < poll_cnt; i++) {
			msleep(MAX_WORD_PROGRAMMING_TIME_MS);
			rc = bq27520_read_bat_flags(&bd->bat_ps);
			if (!rc && bd->flags & BAT_DET_MASK)
				msleep(POLL_QEN_PERIOD_MS);
			else
				break;
		}
	}

	if (i == poll_cnt && (bd->flags & BAT_DET_MASK))
		rc = -ETIME;

	return rc;
}

static int bq27520_make_sure_it_enabled_is_set(struct bq27520_data *bd)
{
	int rc = bq27520_read_it_enabled(bd);
	if (!rc) {
		pr_info("%s: IT Enable not set. Try to set.\n", BQ27520_NAME);

		/* Recommendation from TI:
		 * Battery must be removed before IT_ENABLED is set
		 */
		rc = bq27520_make_sure_bat_is_removed(bd);
		if (!rc) {
			rc = bq27520_write_control(bd, SUB_CMD_IT_ENABLE);
			if (!rc) {
				msleep(IT_ENABLE_DELAY_MS);
				rc = bq27520_read_it_enabled(bd);
			}
		}
	}

	if (rc < 0) {
		pr_err("%s: IT Enable check failed. rc=%d\n", BQ27520_NAME, rc);
	} else if (!rc) {
		pr_err("%s: IT Enable failed to set.\n", BQ27520_NAME);
		rc = -EFAULT;
	} else {
		pr_info("%s: IT Enable confirmed to be set.\n", BQ27520_NAME);
		rc = 0;
	}

	return rc;
}

static int bq27520_last_step_production(struct bq27520_data *bd)
{
	int rc;
	/* Document says to do following in last production step:
	 * 1. Send RESET
	 * 2. Set IT_ENABLE
	 * 3. Set SEALED mode
	 *
	 * According to TI:
	 * Reset is made when new firmware is starting to execute.
	 * So no need to reset again.
	 */

	/* SEMC golden file has IT Enabled by default from some revision.
	 * For backward compatibility read out if it is set and if not, then
	 * set it here.
	 *
	 * Wait before reading out since when setting it takes some
	 * time for the gauge to write it internally in flash.
	 */
	msleep(IT_ENABLE_DELAY_MS);
	rc = bq27520_make_sure_it_enabled_is_set(bd);

	/* Do not write SEALED command as descibed in document.
	 * We can not change battery type in SEALED mode.
	 */
	return rc;
}

static int bq27520_recover_rom_mode(struct i2c_client *clientp,
				    struct i2c_client **rom_clientp)
{
	s32 rc;

	*rom_clientp = i2c_new_dummy(clientp->adapter,
				     FUEL_GAUGE_ROM_SLAVE_ADDR);
	if (!*rom_clientp) {
		pr_err("%s: Failed creating ROM i2c access\n", BQ27520_NAME);
		return -EIO;
	}

	/* Check if i2c responds */
	rc = i2c_smbus_read_byte_data(*rom_clientp, 0x66);
	if (rc < 0) {
		/* No, not in ROM mode */
		i2c_unregister_device(*rom_clientp);
		*rom_clientp = NULL;
	} else {
		/* Yes, we are in ROM mode */
		rc = 0;
	}

	return rc;
}

static int bq27520_enter_rom_mode(struct bq27520_data *bd)
{
	s32 rc;

	if (bd->rom_clientp)
		return -EALREADY;

	pr_info("%s: Enter ROM mode\n", BQ27520_NAME);

	bd->rom_clientp = i2c_new_dummy(bd->clientp->adapter,
					FUEL_GAUGE_ROM_SLAVE_ADDR);
	if (!bd->rom_clientp) {
		pr_err("%s: Failed creating ROM i2c access\n", BQ27520_NAME);
		return -EIO;
	}
	rc = i2c_smbus_write_word_data(bd->clientp,
				       ENTER_ROM_REG, ENTER_ROM_DATA);
	if (rc < 0) {
		pr_err("%s: Fail enter ROM mode. rc=%d\n", BQ27520_NAME, rc);
		i2c_unregister_device(bd->rom_clientp);
		bd->rom_clientp = NULL;
	}

	return rc;
}

static int bq27520_exit_rom_mode(struct bq27520_data *bd)
{
	s32 rc;

	if (!bd->rom_clientp)
		return -EFAULT;

	pr_info("%s: Leave ROM mode\n", BQ27520_NAME);

	rc = i2c_smbus_write_byte_data(bd->rom_clientp,
				       LEAVE_ROM_REG1, LEAVE_ROM_DATA1);
	if (rc < 0) {
		pr_err("%s: Fail exit ROM mode. rc=%d\n", BQ27520_NAME, rc);
		goto unregister_rom;
	}
	msleep(3);
	rc = i2c_smbus_write_byte_data(bd->rom_clientp,
				       LEAVE_ROM_REG2, LEAVE_ROM_DATA2);
	if (rc < 0) {
		pr_err("%s: %s: Send Checksum for LSB rc=%d\n",
		       BQ27520_NAME, __func__, rc);
		goto unregister_rom;
	}
	rc = i2c_smbus_write_byte_data(bd->rom_clientp,
				       LEAVE_ROM_REG3, LEAVE_ROM_DATA3);
	if (rc < 0) {
		pr_err("%s: %s: Send Checksum for MSB rc=%d\n",
		       BQ27520_NAME, __func__, rc);
	}

unregister_rom:
	i2c_unregister_device(bd->rom_clientp);
	bd->rom_clientp = NULL;

	msleep(LEAVE_ROM_DELAY_MS);
	return rc;
}

static ssize_t bq27520_fg_data_write(struct kobject *kobj,
				     struct bin_attribute *bin_attr,
				     char *buf, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	u8 retry = 0;
	size_t offs = 0;
	u8 length;
	s32 rc;

	pr_debug("%s: %s(): pos 0x%x, size %u\n",
		 BQ27520_NAME, __func__, (unsigned int)pos, size);

	if ((pos + size) > (0xFF + 1)) {
		pr_err("%s: Trying to write outside register map\n",
		       BQ27520_NAME);
		return -EMSGSIZE;
	}

	while (offs < size && retry <= I2C_RETRY_MAX) {
		if ((size - offs) > I2C_SMBUS_BLOCK_MAX)
			length = I2C_SMBUS_BLOCK_MAX;
		else
			length = size - offs;

		rc = i2c_smbus_write_i2c_block_data(bd->rom_clientp,
						    (u8)(pos + offs),
						    length, buf + offs);
		if (rc < 0) {
			retry++;
			msleep(I2C_RETRY_DELAY_MS * retry);
		} else {
			retry = 0;
			offs += length;
		}
	}

	return (retry > I2C_RETRY_MAX) ? -EPERM : offs;
}

static ssize_t bq27520_fg_data_read(struct kobject *kobj,
				    struct bin_attribute *bin_attr,
				    char *buf, loff_t pos, size_t size)

{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);

	pr_debug("%s: %s(): pos 0x%x, size %u\n",
		 BQ27520_NAME, __func__, (unsigned int)pos, size);

	if ((pos + size) > (0xFF + 1)) {
		pr_err("%s: Trying to read outside register map\n",
		       BQ27520_NAME);
		return -EMSGSIZE;
	}

	return safe_i2c_smbus_read_i2c_block_data(bd->rom_clientp, pos, size,
						  buf);
}

static struct bin_attribute bq27520_fg_data = {
	.attr = {
		.name = "fg_data",
		.mode = S_IRUSR|S_IWUSR,
	},
	.size = 256,
	.read = bq27520_fg_data_read,
	.write = bq27520_fg_data_write,
};

static ssize_t store_fg_cmd(struct device *pdev,
			    struct device_attribute *attr, const char *pbuf,
			    size_t count)
{
	struct power_supply *psy = dev_get_drvdata(pdev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	int rc;
	char cmdstr[10];

	rc = sscanf(pbuf, "%9s", cmdstr);
	if (rc != 1) {
		pr_debug("%s: %s: cmd read error rc=%d\n",
			 BQ27520_NAME, __func__, rc);
		return -EBADMSG;
	}
	cmdstr[sizeof(cmdstr) - 1] = '\0';

	mutex_lock(&bd->lock);

	pr_debug("%s: %s(): (%s)\n", BQ27520_NAME, __func__, cmdstr);

	if (!strncmp(cmdstr, "start", sizeof(cmdstr))) {
		u8 tmp;
		if (bq27520_check_if_sealed(bd)) {
			pr_info("%s: Can not enter ROM mode. "
				"Must unseal device first\n", BQ27520_NAME);
			if (bq27520_unseal(bd)) {
				pr_err("%s: Can not unseal device\n",
				       BQ27520_NAME);
				goto end;
			}
		}

		if (bd->pdata->disable_algorithm)
			bd->pdata->disable_algorithm(true);

		rc = sysfs_create_bin_file(&pdev->kobj, &bq27520_fg_data);
		if (rc && rc != -EEXIST) {
			pr_err("%s: Cannot create sysfs bin file. rc=%d\n",
			       BQ27520_NAME, rc);
			goto end;
		}
		tmp = atomic_read(&bq27520_init_ok);
		bd->run_init_after_rom = true;
		atomic_set(&bq27520_init_ok, 0);
		rc = bq27520_enter_rom_mode(bd);
		if (rc && rc != -EALREADY) {
			atomic_set(&bq27520_init_ok, tmp);
			if (tmp)
				bd->run_init_after_rom = false;
			sysfs_remove_bin_file(&pdev->kobj, &bq27520_fg_data);
			if (bd->pdata->disable_algorithm)
				bd->pdata->disable_algorithm(false);
		}
	} else if (!strncmp(cmdstr, "end", sizeof(cmdstr))) {
		rc = bq27520_exit_rom_mode(bd);
		if (rc)
			goto end;
		if (bq27520_last_step_production(bd))
			pr_info("%s: Can not finalize last production step\n",
				BQ27520_NAME);
		sysfs_remove_bin_file(&pdev->kobj, &bq27520_fg_data);
		queue_work(bd->wq, &bd->init_work);
		if (bd->pdata->disable_algorithm)
			bd->pdata->disable_algorithm(false);
	} else {
		pr_debug("%s: %s: cmd not supported\n", BQ27520_NAME, __func__);
		rc = -EINVAL;
		goto end;
	}

	rc = strlen(pbuf);

end:
	mutex_unlock(&bd->lock);
	return rc;
}

static ssize_t show_fg_cmd(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);
	u16 fw_ver = 0;
	s32 rc;
	struct bq27520_golden_info gi;

	pr_debug("%s: %s()\n", BQ27520_NAME, __func__);

	if (bd->rom_clientp)
		/* In ROM mode. Return '0' */
		return scnprintf(buf, PAGE_SIZE, "0x0000 0x0000 0x0000\n");

	rc = bq27520_get_fw_version(bd->clientp, &fw_ver);
	if (rc < 0) {
		pr_err("%s: Failed getting FW version. rc=%d\n",
		       BQ27520_NAME, rc);
		return rc;
	}

	rc = bq27520_get_golden_info(bd, &gi);
	if (rc < 0) {
		pr_err("%s: Failed getting expected FW version\n",
		       BQ27520_NAME);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "0x%.4x 0x%.4x 0x%.4x", fw_ver,
			 gi.fw_compatible_version, gi.golden_file_version);
}

static struct device_attribute sysfs_attrs[] = {
	__ATTR(capacity,     S_IRUGO, show_capacity, NULL),
	__ATTR(fg_cmd,       S_IRUSR|S_IWUSR, show_fg_cmd, store_fg_cmd),
#ifdef DEBUG_FS
	__ATTR(set_voltage,  S_IWUSR, NULL, store_voltage),
	__ATTR(set_current,  S_IWUSR, NULL, store_current),
	__ATTR(set_capacity, S_IWUSR, NULL, store_capacity),
	__ATTR(set_capacity_level, S_IWUSR, NULL, store_capacity_level),
#endif /* DEBUG_FS */
};

static int sysfs_create_attrs(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sysfs_attrs); i++)
		if (device_create_file(dev, &sysfs_attrs[i]))
			goto sysfs_create_attrs_failed;

	return 0;

sysfs_create_attrs_failed:
	pr_err("%s: Failed creating sysfs attrs.\n", BQ27520_NAME);
	while (i--)
		device_remove_file(dev, &sysfs_attrs[i]);

	return -EIO;
}

static void sysfs_remove_attrs(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sysfs_attrs); i++)
		(void)device_remove_file(dev, &sysfs_attrs[i]);
}

static short conv_short(int v)
{
	return (short)v;
}

static int bq27520_read_bat_voltage(struct power_supply *bat_ps)
{
	s32 rc;
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	rc = i2c_smbus_read_word_data(bd->clientp, REG_CMD_VOLTAGE);
	if (rc < 0)
		return rc;
	bd->curr_mv = rc;
	pr_debug("%s: %s() rc=%d\n", BQ27520_NAME, __func__, bd->curr_mv);
	return 0;
}

static int bq27520_read_bat_capacity(struct power_supply *bat_ps)
{
	s32 rc;
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	rc = i2c_smbus_read_byte_data(bd->clientp, REG_CMD_SOC);
	if (rc < 0)
		return rc;
	bd->curr_capacity = rc;
	pr_debug("%s: %s() rc=%d\n", BQ27520_NAME, __func__,
						bd->curr_capacity);
	return 0;
}

static int bq27520_read_bat_current(struct power_supply *bat_ps)
{
	s32 rc;
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	rc = i2c_smbus_read_word_data(bd->clientp, REG_CMD_INS_CURRENT);
	if (rc < 0)
		return rc;
	bd->curr_current = (int)conv_short(rc);
	pr_debug("%s: %s() rc=%d\n", BQ27520_NAME, __func__,
						bd->curr_current);
	return 0;
}

static int bq27520_read_bat_current_avg(struct power_supply *bat_ps)
{
	s32 rc;
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	rc = i2c_smbus_read_word_data(bd->clientp, REG_CMD_AVG_CURRENT);
	if (rc < 0)
		return rc;
	bd->current_avg = (int)conv_short(rc);
	pr_debug("%s: %s() rc=%d\n", BQ27520_NAME, __func__,
						bd->current_avg);
	return 0;
}

static int bq27520_read_bat_flags(struct power_supply *bat_ps)
{
	s32 rc;
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	rc = i2c_smbus_read_word_data(bd->clientp, REG_CMD_FLAGS);
	if (rc < 0)
		return rc;
	bd->flags = rc;
	pr_debug("%s: %s() rc=0x%x\n", BQ27520_NAME, __func__, bd->flags);
	return 0;
}

static int bq27520_read_app_status(struct power_supply *bat_ps)
{
	s32 rc;
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	rc = i2c_smbus_read_byte_data(bd->clientp, REG_CMD_APPSTATUS);
	if (rc < 0)
		return rc;
	bd->app_status = rc;
	pr_debug("%s: %s() rc=0x%x\n", BQ27520_NAME, __func__,
						bd->app_status);
	return 0;
}

static int bq27520_read_control_status(struct bq27520_data *bd)
{
	s32 rc = i2c_smbus_read_word_data(bd->clientp, REG_CMD_CONTROL);
	if (rc < 0)
		return rc;
	bd->control_status = rc;
	pr_debug("%s: %s() rc=0x%x\n", BQ27520_NAME, __func__,
						bd->control_status);
	return 0;
}

static int bq27520_write_control(struct bq27520_data *bd, int subcmd)
{
	s32 rc = i2c_smbus_write_word_data(bd->clientp, REG_CMD_CONTROL,
					   subcmd);
	pr_debug("%s: %s() subcmd=0x%x rc=%d\n",
		 BQ27520_NAME, __func__, subcmd, rc);
	return rc;
}

static int bq27520_wait_for_qen_set(struct bq27520_data *bd)
{
	const unsigned int poll_cnt = POLL_QEN_TIMEOUT_MS / POLL_QEN_PERIOD_MS;
	int rc = 0;
	unsigned int i;

	/* TI recommendation before writing temperature after firmware load:
	 * Poll for QEN set or wait at least 2000 ms.
	 *
	 * This will do both and stop when one of these two
	 * conditions are met.
	 */
	for (i = 0; i < poll_cnt; i++) {
		bq27520_write_control(bd, SUB_CMD_NULL);
		usleep(WAIT_ON_READ_SUB_CMD_US);
		bq27520_read_control_status(bd);

		if (bd->control_status & QEN_MASK)
			break;
		else
			msleep(POLL_QEN_PERIOD_MS);
	}

	if (i == poll_cnt && !(bd->control_status & QEN_MASK))
		rc = -ETIME;

	return rc;
}

static int bq27520_write_temperature(struct bq27520_data *bd, int temp)
{
	int k = temp + A_TEMP_COEF_DEFINE;
	s32 rc = i2c_smbus_write_word_data(bd->clientp, REG_CMD_TEMPERATURE, k);
	pr_debug("%s: %s() k=%d rc=%d\n", BQ27520_NAME, __func__, k, rc);
	return rc;
}

static int bq27520_make_sure_temperature_is_set(struct bq27520_data *bd,
						int temp)
{
	unsigned int poll_cnt =
		TEMP_WRITE_TIMEOUT_MS / MAX_WORD_PROGRAMMING_TIME_MS;
	s32 rc = 0;
	int temp_check = 0;
	unsigned int i;

	for (i = 0; i < poll_cnt; i++) {
		rc = bq27520_write_temperature(bd, temp);
		if (rc < 0)
			break;
		msleep(MAX_WORD_PROGRAMMING_TIME_MS);
		rc = i2c_smbus_read_word_data(bd->clientp, REG_CMD_TEMPERATURE);
		if (rc < 0)
			break;

		temp_check = rc - A_TEMP_COEF_DEFINE;
		if (temp_check == temp)
			break;
	}

	if (rc >= 0 && i == poll_cnt && temp_check != temp)
		rc = -ETIME;

	if (rc < 0)
		pr_err("%s: Failed writing temperature. rc = %d\n",
		       BQ27520_NAME, rc);

	return rc;
}

static int bq27520_check_initialization_comp(struct bq27520_data *bd)
{
	int i;

	for (i = 0; i < RETRY_MAX; i++) {
		msleep(1000);
		bq27520_write_control(bd, SUB_CMD_NULL);
		usleep(WAIT_ON_READ_SUB_CMD_US);
		if (!bq27520_read_control_status(bd) &&
		    (bd->control_status & INIT_COMP_MASK))
			return 0;
	}
	return -ETIME;
}

static int bq27520_battery_info_setting(struct bq27520_data *bd,
					int type, int temp)
{
	int rc;
	int subcmd = 0;

	rc = bq27520_make_sure_it_enabled_is_set(bd);
	if (rc)
		return rc;

	bq27520_read_bat_flags(&bd->bat_ps);
	/* Only set BAT_INSERT if gauge sees no battery detect
	 * and the battery is identified externally.
	 *
	 * Unidentified batteries should not be set to detect in
	 * gauge. If set to detect, gauge will learn an unidentified battery.
	 *
	 * BAT_INSERT clears automatically when when gauge looses power
	 */
	if (type != POWER_SUPPLY_TECHNOLOGY_UNKNOWN &&
		!(bd->flags & BAT_DET_MASK)) {
		/* Battery was removed since last usage.
		 * Need to force OCV to find good reference
		 * for the gauge.
		 */
		bd->force_ocv = true;

		/* Recommendetion from TI:
		 * Write temperature before BAT_INSERT.
		 */
		bq27520_make_sure_temperature_is_set(bd, temp);

		pr_info("%s: Writing BAT_INSERT\n", BQ27520_NAME);
		bq27520_write_control(bd, SUB_CMD_BAT_INSERT);
	} else if (type == POWER_SUPPLY_TECHNOLOGY_UNKNOWN) {
		pr_info("%s: Writing BAT_REMOVE\n", BQ27520_NAME);
		bq27520_write_control(bd, SUB_CMD_BAT_REMOVE);
		return 0;
	}

	rc = bq27520_check_initialization_comp(bd);
	if (rc)
		return rc;

	/* Sanity to be really sure that battery will be learned */
	rc = bq27520_wait_for_qen_set(bd);
	if (rc) {
		pr_err("%s: QEN not set. Battery will not be learned. rc=%d\n",
		       BQ27520_NAME, rc);
		return rc;
	}

	if (!bd->got_technology)
		return -EINVAL;

	rc = bq27520_read_app_status(&bd->bat_ps);
	if (rc)
		return rc;

	pr_debug("%s: %s() type=%d temp=%d status=%d\n",
		BQ27520_NAME, __func__, type, temp, bd->app_status);

	if ((bd->app_status & LU_PROF_MASK) &&
		type == POWER_SUPPLY_TECHNOLOGY_LIPO)
		subcmd = SUB_CMD_CHOOSE_A;
	else if (!(bd->app_status & LU_PROF_MASK) &&
		type == POWER_SUPPLY_TECHNOLOGY_LiMn)
		subcmd = SUB_CMD_CHOOSE_B;
	else if (type == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		return -EINVAL;

	if (subcmd) {
		bq27520_write_control(bd, subcmd);
		msleep(1000);

		/* TI:
		 * No need to send BAT_INSERT since it has already been
		 * sent in top of this function.
		 */
		rc = bq27520_check_initialization_comp(bd);
	}

	return rc;
}

#ifdef SUPPORT_PARAMETER_UPDATE
static int bq27520_block_data_update(struct bq27520_data *bd)
{
	int i;
	int rc;

	for (i = 0; i < BQ27520_BTBL_MAX; i++) {
		rc = i2c_smbus_write_byte_data(bd->clientp,
			bd->udatap[i].adr,
			bd->udatap[i].data);
		if (rc < 0) {
			pr_err("%s: %s() rc=%d adr=0x%x\n",
				BQ27520_NAME, __func__, rc, bd->udatap[i].adr);
			return rc;
		}
		msleep(1);
	}

	msleep(100);
	rc = i2c_smbus_write_word_data(bd->clientp,
		REG_CMD_CONTROL, SUB_CMD_RESET);
	if (rc < 0)
		pr_err("%s: %s() rc=%d adr=0x%x\n",
		       BQ27520_NAME, __func__, rc, REG_CMD_CONTROL);
	msleep(1000);
	return rc;
}
#endif

static void bq27520_init_worker(struct work_struct *work)
{
	struct bq27520_data *bd =
		container_of(work, struct bq27520_data, init_work);
	struct power_supply *ps;
	int i;

	if (bd->battery_dev_name) {
		for (i = 0; i < RETRY_MAX; i++) {
			ps = power_supply_get_by_name(bd->battery_dev_name);
			if (ps) {
#ifdef SUPPORT_PARAMETER_UPDATE
				if (bd->udatap)
					bq27520_block_data_update(bd);
#endif
				get_supplier_data(ps->dev, &bd->bat_ps);
				bq27520_battery_info_setting(bd,
							     bd->technology,
							     bd->bat_temp);
				break;
			}
			msleep(1000);
		}
	}
	msleep(1000);
	bq27520_read_bat_capacity(&bd->bat_ps);
	bq27520_read_bat_current_avg(&bd->bat_ps);
	bq27520_read_bat_flags(&bd->bat_ps);
	if (bd->curr_capacity == 0 && !(bd->flags & SYSDOWN_MASK))
		bd->curr_capacity = 1;
	/* This if() block is not needed according to TI */
	if (bd->pdata && bd->current_avg > 0 && bd->force_ocv) {
		pr_info("%s: Detect bootup with charger.\n", BQ27520_NAME);
		if (bd->pdata->disable_algorithm)
			bd->pdata->disable_algorithm(true);
		msleep(DELAY_TIME_BEFORE_OCV_ISSUE);
		(void)bq27520_write_control(bd, SUB_CMD_OCV_CMD);
		msleep(DELAY_TIME_AFTER_OCV_ISSUE);
		if (bd->pdata->disable_algorithm)
			bd->pdata->disable_algorithm(false);
	}
	bd->force_ocv = false;
	pr_info("%s: %s() capacity=%d flags=0x%x\n", BQ27520_NAME,
		__func__, bd->curr_capacity, bd->flags);
	bd->run_init_after_rom = false;
	atomic_set(&bq27520_init_ok, 1);

	/* Notify externals that we have new data to share */
	power_supply_changed(&bd->bat_ps);
}

static int bq27520_bat_get_property(struct power_supply *bat_ps,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	int rc = 0;
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	if (!atomic_read(&bq27520_init_ok))
		return -EBUSY;

	mutex_lock(&bd->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = bq27520_read_bat_voltage(bat_ps);
		if (rc)
			break;
		val->intval = bd->curr_mv * 1000;
#ifdef DEBUG_FS
		if (bd->bat_volt_debug.active)
			val->intval = bd->bat_volt_debug.value * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = bd->lipo_bat_max_volt * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bd->lipo_bat_min_volt * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bd->curr_capacity;
#ifdef DEBUG_FS
		if (bd->bat_cap_debug.active)
			val->intval = bd->bat_cap_debug.value;
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = bd->curr_capacity_level;
#ifdef DEBUG_FS
		if (bd->bat_cap_lvl_debug.active)
			val->intval = bd->bat_cap_lvl_debug.value;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = bq27520_read_bat_current(bat_ps);
		if (rc)
			break;
		val->intval = bd->curr_current * 1000;
#ifdef DEBUG_FS
		if (bd->bat_curr_debug.active)
			val->intval = bd->bat_curr_debug.value * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		rc = bq27520_read_bat_current_avg(bat_ps);
		if (rc)
			break;
		val->intval = bd->current_avg * 1000;
#ifdef DEBUG_FS
		if (bd->bat_curr_debug.active)
			val->intval = bd->bat_curr_debug.value * 1000;
#endif
		break;
	default:
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&bd->lock);
	return rc;
}

static void start_read_fc(struct bq27520_data *bd)
{
	pr_debug("%s: %s()\n", BQ27520_NAME, __func__);
	queue_delayed_work(bd->wq, &bd->fc_work, 0);
}

static void stop_read_fc(struct bq27520_data *bd)
{
	pr_debug("%s: %s()\n", BQ27520_NAME, __func__);
	if (delayed_work_pending(&bd->fc_work))
		cancel_delayed_work_sync(&bd->fc_work);
}

static void bq27520_read_fc_worker(struct work_struct *work)
{
	int rc;
	struct delayed_work *dwork =
		container_of(work, struct delayed_work, work);
	struct bq27520_data *bd =
		container_of(dwork, struct bq27520_data, fc_work);

	mutex_lock(&bd->lock);
	rc = bq27520_read_bat_flags(&bd->bat_ps);
	mutex_unlock(&bd->lock);

	pr_debug("%s: %s() capacity=%d flags=0x%x\n", BQ27520_NAME, __func__,
		 bd->curr_capacity, bd->flags);

	if (!rc) {
		u8 changed = 0;

		mutex_lock(&bd->lock);
		if (bd->flags & FC_MASK &&
		    bd->chg_connected &&
		    bd->curr_capacity_level !=
		    POWER_SUPPLY_CAPACITY_LEVEL_FULL) {
			bd->curr_capacity_level =
				POWER_SUPPLY_CAPACITY_LEVEL_FULL;
			changed = 1;
		} else if (!(bd->flags & FC_MASK) &&
			   bd->curr_capacity_level !=
			   POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN) {
			bd->curr_capacity_level =
				POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
			changed = 1;
		}
		mutex_unlock(&bd->lock);

		if (changed)
			power_supply_changed(&bd->bat_ps);
	}

	queue_delayed_work(bd->wq, &bd->fc_work, HZ * READ_FC_TIMER);
}

static void bq27520_handle_soc_worker(struct work_struct *work)
{
	int valid_cap = 0;
	struct bq27520_data *bd =
		container_of(work, struct bq27520_data, soc_int_work);

	mutex_lock(&bd->lock);

	if (bd->got_technology &&
		bd->technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN) {
		if (!bq27520_read_bat_voltage(&bd->bat_ps)) {
			bd->curr_capacity =
			((clamp(bd->curr_mv,
			bd->lipo_bat_min_volt, bd->lipo_bat_max_volt) -
			bd->lipo_bat_min_volt) * 100) /
			(bd->lipo_bat_max_volt - bd->lipo_bat_min_volt);
			valid_cap = 1;
		}
	} else if (!bq27520_read_bat_capacity(&bd->bat_ps))
			valid_cap = 1;

	if (!bq27520_read_bat_flags(&bd->bat_ps) &&
			(bd->flags & SYSDOWN_MASK)) {
		pr_info("%s: Shutting down because of low voltage "
			"(SOC = %u%%).\n", BQ27520_NAME, bd->curr_capacity);
		bd->curr_capacity = 0;
		valid_cap = 1;
	} else if (valid_cap && bd->curr_capacity == 0) {
		bd->curr_capacity = 1;
		pr_info("%s: SOC is 0%% and no SYSDOWN.\n", BQ27520_NAME);
	}

	mutex_unlock(&bd->lock);

	if (valid_cap) {
		mutex_lock(&bd->lock);
		if (bd->chg_connected &&
		    bd->curr_capacity >= bd->polling_lower_capacity &&
			bd->curr_capacity <= bd->polling_upper_capacity) {
			if (!bd->started_worker) {
				start_read_fc(bd);
				bd->started_worker = 1;
			}
		} else {
			if (bd->started_worker) {
				stop_read_fc(bd);
				bd->started_worker = 0;
				bd->curr_capacity_level =
					POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
			}
		}
		mutex_unlock(&bd->lock);

		power_supply_changed(&bd->bat_ps);
	}
	pr_info("%s: %s() capacity=%d flags=0x%x valid=%d\n",
		BQ27520_NAME, __func__,
		bd->curr_capacity, bd->flags, valid_cap);
}

static irqreturn_t bq27520_soc_thread_irq(int irq, void *data)
{
	struct bq27520_data *bd = (struct bq27520_data *)data;

	if (atomic_read(&bq27520_init_ok)) {
		mutex_lock(&bd->int_lock);
		if (!bd->suspended)
			bq27520_handle_soc_worker(&bd->soc_int_work);
		else
			bd->resume_int = 1;
		mutex_unlock(&bd->int_lock);
	}

	return IRQ_HANDLED;
}

static int get_supplier_data(struct device *dev, void *data)
{
	struct power_supply *psy = (struct power_supply *)data;
	struct power_supply *pst = dev_get_drvdata(dev);
	unsigned int i;
	union power_supply_propval ret;
	struct bq27520_data *bd =
		container_of(psy, struct bq27520_data, bat_ps);

	mutex_lock(&bd->lock);

	for (i = 0; i < pst->num_supplicants; i++) {
		if (strcmp(pst->supplied_to[i], psy->name))
			continue;

		if (!pst->get_property(pst, POWER_SUPPLY_PROP_TEMP, &ret)) {
			if (atomic_read(&bq27520_init_ok) &&
			    bd->bat_temp != ret.intval)
				bq27520_write_temperature(bd, ret.intval);
			bd->bat_temp = ret.intval;
			pr_debug("%s: got temperature %d C\n", BQ27520_NAME,
				ret.intval);
		}

		if (!pst->get_property(pst, POWER_SUPPLY_PROP_TECHNOLOGY,
				       &ret)) {
			bd->technology = ret.intval;
			bd->got_technology = 1;
			pr_debug("%s: got technology %d\n", BQ27520_NAME,
				ret.intval);
		}
	}

	mutex_unlock(&bd->lock);

	return 0;
}

static void bq27520_ext_pwr_change_worker(struct work_struct *work)
{
	struct bq27520_data *bd =
		container_of(work, struct bq27520_data, ext_pwr_change_work);
	int chg_connected = power_supply_am_i_supplied(&bd->bat_ps);

	if (chg_connected != bd->chg_connected) {
		mutex_lock(&bd->lock);
		bd->chg_connected = chg_connected;
		pr_debug("%s: Charger %sonnected\n", BQ27520_NAME,
			 bd->chg_connected ? "c" : "disc");
		if (!chg_connected) {
			if (bd->started_worker) {
				stop_read_fc(bd);
				bd->started_worker = 0;
			}
			bd->curr_capacity_level =
				POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		} else {
			if (!bd->started_worker &&
			bd->curr_capacity >= bd->polling_lower_capacity &&
			bd->curr_capacity <= bd->polling_upper_capacity) {
				start_read_fc(bd);
				bd->started_worker = 1;
			}
		}
		mutex_unlock(&bd->lock);
	}
	class_for_each_device(power_supply_class, NULL, &bd->bat_ps,
			      get_supplier_data);

}

static void bq27520_bat_external_power_changed(struct power_supply *bat_ps)
{
	struct bq27520_data *bd =
		container_of(bat_ps, struct bq27520_data, bat_ps);

	queue_work(bd->wq, &bd->ext_pwr_change_work);
}

#ifdef CONFIG_SUSPEND
static int bq27520_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq27520_data *bd = i2c_get_clientdata(client);

	mutex_lock(&bd->int_lock);
	bd->suspended = 1;
	if (bd->got_technology &&
		bd->technology != POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		set_irq_wake(client->irq, 1);

	mutex_unlock(&bd->int_lock);
	flush_workqueue(bd->wq);

	return 0;
}

static int bq27520_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq27520_data *bd = i2c_get_clientdata(client);

	bd->suspended = 0;
	if (bd->got_technology &&
		bd->technology != POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		set_irq_wake(client->irq, 0);

	if (bd->resume_int) {
		bd->resume_int = 0;
		bq27520_handle_soc_worker(&bd->soc_int_work);
	}

	return 0;
}
#else
#define bq27520_pm_suspend	NULL
#define bq27520_pm_resume	NULL
#endif

static int __exit bq27520_remove(struct i2c_client *client)
{
	struct bq27520_data *bd = i2c_get_clientdata(client);

	free_irq(client->irq, 0);

	if (work_pending(&bd->soc_int_work))
		cancel_work_sync(&bd->soc_int_work);

	if (work_pending(&bd->ext_pwr_change_work))
		cancel_work_sync(&bd->ext_pwr_change_work);

	if (work_pending(&bd->init_work))
		cancel_work_sync(&bd->init_work);

	if (delayed_work_pending(&bd->fc_work))
		cancel_delayed_work_sync(&bd->fc_work);

	destroy_workqueue(bd->wq);

	sysfs_remove_attrs(bd->bat_ps.dev);

	power_supply_unregister(&bd->bat_ps);

	i2c_set_clientdata(client, NULL);

	kfree(bd);
	return 0;
}

static enum power_supply_property bq27520_bat_main_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_PRESENT
};

static const struct i2c_device_id bq27520_id[] = {
	{BQ27520_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bq27520_id);

static int bq27520_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rc = 0;
	u16 fw_ver = 0;
	struct bq27520_platform_data *pdata;
	struct bq27520_data *bd;
	struct bq27520_golden_info gi;
	struct i2c_client *rom_clientp = NULL;

	rc = bq27520_get_fw_version(client, &fw_ver);
	if (rc == -EIO) {
		rc = bq27520_recover_rom_mode(client, &rom_clientp);
		if (rc < 0) {
			pr_err("%s: Failed recover ROM mode\n", BQ27520_NAME);
			goto probe_exit;
		}
	} else if (rc < 0) {
		pr_err("%s: Failed getting FW version\n", BQ27520_NAME);
		goto probe_exit;
	}

	bd = kzalloc(sizeof(struct bq27520_data), GFP_KERNEL);
	if (!bd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	bd->bat_ps.name = BQ27520_NAME;
	bd->bat_ps.type = POWER_SUPPLY_TYPE_BATTERY;
	bd->bat_ps.properties = bq27520_bat_main_props;
	bd->bat_ps.num_properties = ARRAY_SIZE(bq27520_bat_main_props);
	bd->bat_ps.get_property = bq27520_bat_get_property;
	bd->bat_ps.external_power_changed =
		bq27520_bat_external_power_changed;
	bd->bat_ps.use_for_apm = 1;
	bd->clientp = client;
	bd->rom_clientp = rom_clientp;

	bd->polling_lower_capacity = 95;
	bd->polling_upper_capacity = 100;
	bd->force_ocv = false;
	bd->run_init_after_rom = false;
	bd->sealed = -1;
	pdata = client->dev.platform_data;
	if (pdata) {
		bd->pdata = pdata;
		bd->battery_dev_name = pdata->battery_dev_name;
		bd->lipo_bat_max_volt = pdata->lipo_bat_max_volt;
		bd->lipo_bat_min_volt = pdata->lipo_bat_min_volt;
		memcpy(bd->capacity_scaling, pdata->capacity_scaling,
		       sizeof(bd->capacity_scaling));
		bd->polling_lower_capacity = pdata->polling_lower_capacity;
		bd->polling_upper_capacity = pdata->polling_upper_capacity;
		bd->udatap = pdata->udatap;
		bd->ocv_issue_capacity_threshold =
			pdata->ocv_issue_capacity_threshold;
		if (pdata->supplied_to) {
			bd->bat_ps.supplied_to = pdata->supplied_to;
			bd->bat_ps.num_supplicants = pdata->num_supplicants;
		}
	}

	mutex_init(&bd->lock);
	mutex_init(&bd->int_lock);
	mutex_init(&bd->data_flash_lock);

	if (bd->rom_clientp) {
		pr_info("%s: In ROM mode\n", BQ27520_NAME);
	} else {
		rc = bq27520_get_golden_info(bd, &gi);
		if (rc < 0) {
			pr_err("%s: Failed getting expected FW version\n",
			       BQ27520_NAME);
			goto probe_exit_mem_free;
		}

		pr_info("%s: FW v%x.%x (expect v%x.%x). Golden FW v%x.%x\n",
			BQ27520_NAME,
			(fw_ver >> 8) & 0xFF, fw_ver & 0xFF,
			(gi.fw_compatible_version >> 8) & 0xFF,
			gi.fw_compatible_version & 0xFF,
			(gi.golden_file_version >> 8) & 0xFF,
			gi.golden_file_version & 0xFF);
	}

	bd->wq = create_singlethread_workqueue("batteryworker");
	if (!bd->wq) {
		pr_err("%s: Failed creating workqueue\n", BQ27520_NAME);
		rc = -EIO;
		goto probe_exit_mem_free;
	}

	INIT_WORK(&bd->init_work, bq27520_init_worker);
	INIT_WORK(&bd->ext_pwr_change_work, bq27520_ext_pwr_change_worker);
	INIT_WORK(&bd->soc_int_work, bq27520_handle_soc_worker);
	INIT_DELAYED_WORK(&bd->fc_work, bq27520_read_fc_worker);

	rc = power_supply_register(&client->dev, &bd->bat_ps);
	if (rc) {
		pr_err("%s: Failed to regist power supply\n", BQ27520_NAME);
		goto probe_exit_destroy_wq;
	}

	i2c_set_clientdata(client, bd);

	bd->got_technology = 0;
	bd->started_worker = 0;
	rc = request_threaded_irq(client->irq,
				NULL, bq27520_soc_thread_irq,
				IRQF_TRIGGER_FALLING | IRQF_DISABLED,
				BQ27520_NAME,
				bd);
	if (rc) {
		pr_err("%s: Failed requesting IRQ\n", BQ27520_NAME);
		goto probe_exit_unregister;
	}

	rc = sysfs_create_attrs(bd->bat_ps.dev);
	if (rc) {
		pr_err("%s: Complete sysfs support failed\n", BQ27520_NAME);
		goto probe_exit_unregister;
	}

	queue_work(bd->wq, &bd->init_work);
	return 0;

probe_exit_unregister:
	power_supply_unregister(&bd->bat_ps);
probe_exit_destroy_wq:
	destroy_workqueue(bd->wq);
probe_exit_mem_free:
	kfree(bd);
probe_exit:
	return rc;
}

static const struct dev_pm_ops bq27520_pm = {
	.suspend = bq27520_pm_suspend,
	.resume = bq27520_pm_resume,
};

static struct i2c_driver bq27520_driver = {
	.driver = {
		.name = BQ27520_NAME,
		.owner = THIS_MODULE,
		.pm = &bq27520_pm,
	},
	.probe = bq27520_probe,
	.remove = __exit_p(bq27520_remove),
	.id_table = bq27520_id,
};

static int __init bq27520_init(void)
{
	int rc;

	rc = i2c_add_driver(&bq27520_driver);
	if (rc) {
		pr_err("%s: FAILED: i2c_add_driver rc=%d\n", __func__, rc);
		goto init_exit;
	}
	return 0;

init_exit:
	return rc;
}

static void __exit bq27520_exit(void)
{
	i2c_del_driver(&bq27520_driver);
}

module_init(bq27520_init);
module_exit(bq27520_exit);

MODULE_AUTHOR("James Jacobsson, Imre Sunyi, Hiroyuki Namba");
MODULE_LICENSE("GPL");
