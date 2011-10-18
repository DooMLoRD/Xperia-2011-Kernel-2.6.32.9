/*
 * linux/drivers/power/max17040.c
 *
 * MAX17040 Fuel Gauge interface
 *
 * Copyright (C) 2009 James Jacobsson <james.jacobsson@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/max17040.h>

#include <asm/mach-types.h>

#define LIPO_BAT_MAX_VOLTAGE 4200
#define LIPO_BAT_MIN_VOLTAGE 3200

#define MAX17040_REG_ADDR_VCELL	0x02
#define MAX17040_REG_ADDR_SOC	0x04
#define MAX17040_REG_ADDR_MODE	0x06
#define MAX17040_REG_ADDR_RCOMP	0x0C
#define MAX17040_REG_ADDR_OCV	0x0E
#define MAX17040_REG_ADDR_CMD	0xFE
#define MAX17040_REG_ADDR_LOCK	0x3E

/* Delay in ms */
#define MAX17040_MODEL_DELAY	150 /* delay >= 150 */
#define MAX17040_OCV_DELAY	300 /* delay = [150, 600] */

#define MAX17040_TEMP_DEFAULT	200 /* 20.0C */
#define MAX17040_TEMP_DIV	10
#define MAX17040_RCOMP_MAX	0xFF
#define MAX17040_RCOMP_MIN	0x00
#define MAX17040_SOC_CHECK	5
#define MAX17040_VCELL_CHECK	3600

struct max17040_data {
	struct power_supply bat_ps;
	struct i2c_client   *clientp;
	int curr_mv;
	int curr_soc;
	struct delayed_work work;
	struct workqueue_struct *wq;
	int timer_setup;
	struct max17040_platform_data *pdata;
	int curr_temp;
	int exp_model;
	int tech;
};

static void max17040_get_vcell(struct max17040_data *data)
{
	u8 vcell[MAX17040_REG_SIZE];
	int tmp;
	s32 rc;

	rc = i2c_smbus_read_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_VCELL,
		ARRAY_SIZE(vcell),
		vcell);

	if (rc < 0) {
		dev_err(&data->clientp->dev, "%s(): Failed to read vcell, "
			"rc=%d\n", __func__, rc);
		return;
	}

	tmp = (vcell[0] * 16) + (vcell[1]>>4);
	tmp *= 125;
	tmp /= 100;

	data->curr_mv = tmp;
}

static void max17040_get_soc(struct max17040_data *data)
{
	u8 soc[MAX17040_REG_SIZE];
	int tmp;
	s32 rc;

	rc = i2c_smbus_read_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_SOC,
		ARRAY_SIZE(soc),
		soc);
	if (rc < 0) {
		dev_err(&data->clientp->dev, "%s(): Failed to read soc, "
			"rc=%d\n", __func__, rc);
		return;
	}

	tmp = soc[0];

	/* an expanded models LSB is streched to 2^9 */
	if (data->exp_model)
		tmp >>= 1;

	data->curr_soc = clamp(tmp, 0, 100); /* soc = [0,100] % */

	dev_dbg(&data->clientp->dev, "%s(): soc=%d %% (0x%x, 0x%x)\n",
			__func__, data->curr_soc, soc[0], soc[1]);
}

static s32 max17040_quick_start(struct max17040_data *data)
{
	u8 qs[MAX17040_REG_SIZE] = { 0x40, 0x00 }; /* quick-start command */
	s32 rc;

	rc = i2c_smbus_write_i2c_block_data(data->clientp,
			MAX17040_REG_ADDR_MODE,
			ARRAY_SIZE(qs), qs);
	if (rc < 0)
		dev_err(&data->clientp->dev, "%s(): Failed to quick start "
				"chipset\n", __func__);

	return rc;
}

static s32 max17040_power_on_request(struct max17040_data *data)
{
	u8 por[MAX17040_REG_SIZE] = { 0x54, 0x00 }; /* POR command */
	s32 rc;

	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_CMD,
		MAX17040_REG_SIZE,
		por);
	if (rc < 0)
		dev_err(&data->clientp->dev, "%s(): Failed to reset chip with"
			" POR\n", __func__);
	return rc;
}

static s32 max17040_lock_model(struct max17040_data *data)
{
	u8 lock[MAX17040_REG_SIZE] = { 0x00, 0x00 };
	s32 rc;

	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_LOCK,
		MAX17040_REG_SIZE,
		lock);
	if (rc < 0)
		dev_err(&data->clientp->dev, "%s(): Failed to lock model"
			" access\n", __func__);
	return rc;
}

static s32 max17040_unlock_model(struct max17040_data *data)
{
	u8 unlock[MAX17040_REG_SIZE] = { 0x4A, 0x57 };
	s32 rc;

	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_LOCK,
		MAX17040_REG_SIZE,
		unlock);
	if (rc < 0)
		dev_err(&data->clientp->dev, "%s(): Failed to unlock model"
			" access\n", __func__);
	return rc;
}

static s32 max17040_load_model(struct max17040_data *data)
{
	u8 rcomp_max[MAX17040_REG_SIZE] = { 0xFF, 0x00 };
	u8 block_addr[MAX17040_MODEL_NBR_BLKS] = { 0x40, 0x50, 0x60, 0x70 };
	u8 tmp[2 * MAX17040_REG_SIZE];
	u8 soc[MAX17040_REG_SIZE];
	s32 rc;
	int i;

	if (!data->pdata)
		return -EFAULT;

	rc = max17040_unlock_model(data);
	if (rc < 0)
		return rc;

	/* Temporary store RCOMP and OCV, modified during model load */
	rc = i2c_smbus_read_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_RCOMP,
		ARRAY_SIZE(tmp),
		tmp);
	if (rc < 0)
		return rc;

	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_OCV,
		ARRAY_SIZE(data->pdata->model_desc.ocv_test),
		data->pdata->model_desc.ocv_test);
	if (rc < 0)
		return rc;

	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_RCOMP,
		MAX17040_REG_SIZE,
		rcomp_max);
	if (rc < 0)
		return rc;

	/* Write model data */
	for (i = 0; i < MAX17040_MODEL_NBR_BLKS; i += 1) {
		rc = i2c_smbus_write_i2c_block_data(data->clientp,
			block_addr[i],
			MAX17040_MODEL_BLK_SIZE,
			data->pdata->model_desc.model_data[i]);
		if (rc < 0) {
			dev_err(&data->clientp->dev, "%s(): Failed to write "
				"block[%d] of custom model to 0x%02x\n",
				__func__, i, block_addr[i]);
			return rc;
		}
	}

	/* Sync model write */
	msleep(MAX17040_MODEL_DELAY);

	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_OCV,
		ARRAY_SIZE(data->pdata->model_desc.ocv_test),
		data->pdata->model_desc.ocv_test);
	if (rc < 0)
		return rc;

	/* Sync model calculations */
	msleep(MAX17040_OCV_DELAY);

	/* Read SOC and verify model loaded correct */
	rc = i2c_smbus_read_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_SOC,
		ARRAY_SIZE(soc),
		soc);
	if (rc < 0)
		return rc;

	if (soc[0] >= data->pdata->model_desc.soc_low &&
		soc[0] <= data->pdata->model_desc.soc_high) {
		dev_info(&data->clientp->dev, "%s(): soc (0x%.2x, 0x%.2x) "
			"verified successfully\n", __func__, soc[0], soc[1]);
	} else {
		dev_err(&data->clientp->dev, "%s(): Failed to verify soc "
			"(0x%.2x, 0x%.2x). MSB not inside boundary "
			"(low = 0x%02x, high = 0x%02x)\n", __func__,
			soc[0], soc[1],
			data->pdata->model_desc.soc_low,
			data->pdata->model_desc.soc_high);
		return rc;
	}

	/* Restore RCOMP and OCV */
	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_RCOMP,
		ARRAY_SIZE(tmp),
		tmp);
	if (rc < 0)
		return rc;

	rc = max17040_lock_model(data);

	/* Sync model */
	msleep(MAX17040_MODEL_DELAY);

	return rc;
}

static void max17040_update_rcomp(struct max17040_data *data)
{
	u8 new_rcomp[MAX17040_REG_SIZE];
	s32 rc;
	int tmp;

	if (!data->pdata)
		return;

	tmp = data->pdata->rcomp_data.rcomp0;

	/* compensate for temperature */
	if (data->curr_temp > MAX17040_TEMP_DEFAULT)
		tmp = data->pdata->rcomp_data.rcomp0 +
			(data->curr_temp - MAX17040_TEMP_DEFAULT) /
			MAX17040_TEMP_DIV *
			data->pdata->rcomp_data.temp_co_hot /
			data->pdata->rcomp_data.temp_div;
	else if (data->curr_temp < MAX17040_TEMP_DEFAULT)
		tmp = data->pdata->rcomp_data.rcomp0 +
			(data->curr_temp - MAX17040_TEMP_DEFAULT) /
			MAX17040_TEMP_DIV *
			data->pdata->rcomp_data.temp_co_cold /
			data->pdata->rcomp_data.temp_div;

	tmp = clamp_val(tmp, MAX17040_RCOMP_MIN, MAX17040_RCOMP_MAX);

	new_rcomp[0] = (u8) tmp;
	new_rcomp[1] = 0;

	/* Update RCOMP with temperature corrected value */
	rc = i2c_smbus_write_i2c_block_data(data->clientp,
		MAX17040_REG_ADDR_RCOMP,
		ARRAY_SIZE(new_rcomp),
		new_rcomp);
	if (rc < 0)
		dev_err(&data->clientp->dev, "%s(): failed to write rcomp, "
			"rc=%d\n", __func__, rc);

	dev_dbg(&data->clientp->dev, "%s(): curr_temp=%d.%d, rcomp=0x%x\n",
		__func__, data->curr_temp / 10, data->curr_temp % 10, tmp);
}

static void max17040_worker(struct work_struct *work)
{

	struct max17040_data *this;
	struct delayed_work *dwork;
	int delay;

	dwork = container_of(work, struct delayed_work, work);
	this = container_of(dwork, struct max17040_data, work);

	power_supply_changed(&this->bat_ps);

	dev_info(&this->clientp->dev, "batt:%3d%%, %d mV\n", this->curr_soc,
	       this->curr_mv);

	/* increase sample rate when closing in on 0 soc */
	delay = HZ * (this->curr_soc < 5 ? 5 : 60);

	queue_delayed_work(this->wq, &this->work, delay);
}

static int max17040_get_supplier_data(struct device *dev, void *data)
{
	struct power_supply *psy = (struct power_supply *)data;
	struct power_supply *pst = dev_get_drvdata(dev);
	struct max17040_data *this =
		container_of(psy, struct max17040_data, bat_ps);
	union power_supply_propval ret;
	int i;

	for (i = 0; i < pst->num_supplicants; i++) {
		if (strcmp(pst->supplied_to[i], psy->name))
			continue;

		if (!pst->get_property(pst, POWER_SUPPLY_PROP_TEMP, &ret)) {
			this->curr_temp = ret.intval;

			max17040_update_rcomp(this);
		}

		if (!pst->get_property(pst, POWER_SUPPLY_PROP_TECHNOLOGY, &ret))
			this->tech = ret.intval;
	}

	return 0;
}

static int max17040_is_chg(struct max17040_data *data)
{
	if (!data->pdata)
		return 0;

	return (data->curr_temp > data->pdata->chg_min_temp &&
			data->curr_temp	<= data->pdata->chg_max_temp &&
			data->tech != POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
}

static int max17040_get_status(struct max17040_data *data)
{
	if (power_supply_am_i_supplied(&data->bat_ps)) {
		if (max17040_is_chg(data))
			return POWER_SUPPLY_STATUS_CHARGING;
		else
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

static int max17040_bat_get_property(struct power_supply *bat_ps,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct max17040_data *this;
	this = container_of(bat_ps, struct max17040_data, bat_ps);

	if (!this->timer_setup) {
		/* Kick off a work queue to update the battery status */
		INIT_DELAYED_WORK(&this->work, max17040_worker);
		this->wq = create_singlethread_workqueue("batteryworker");
		if (this->wq)
			queue_delayed_work(this->wq, &this->work, HZ*60);

		this->timer_setup = 1;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max17040_get_status(this);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		max17040_get_vcell(this);

		val->intval = this->curr_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = LIPO_BAT_MAX_VOLTAGE * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = LIPO_BAT_MIN_VOLTAGE * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		max17040_get_soc(this);

		val->intval = this->curr_soc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void max17040_bat_external_power_changed(struct power_supply *bat_ps)
{
	struct max17040_data *this = container_of(bat_ps,
			struct max17040_data, bat_ps);

	class_for_each_device(power_supply_class, NULL, &this->bat_ps,
			max17040_get_supplier_data);

	power_supply_changed(bat_ps);
}

static enum power_supply_property max17040_bat_main_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_PRESENT,
};

#ifdef CONFIG_SUSPEND
static int max17040_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17040_data *this = i2c_get_clientdata(client);

	if (!cancel_delayed_work(&this->work))
		flush_workqueue(this->wq);

	return 0;
}

static int max17040_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17040_data *this = i2c_get_clientdata(client);

	queue_delayed_work(this->wq, &this->work, HZ*1);

	return 0;
}
#else
#define max17040_pm_suspend	NULL
#define max17040_pm_resume	NULL
#endif

static int __exit max17040_remove(struct i2c_client *client)
{
	struct max17040_data *this;
	this = i2c_get_clientdata(client);

	destroy_workqueue(this->wq);
	power_supply_unregister(&this->bat_ps);

	kfree(this);
	return 0;
}

static const struct i2c_device_id max17040_id[] = {
	{ MAX17040_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static int max17040_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int rc = 0;
	s32 buf[2];
	struct max17040_data *this;

	buf[0] = i2c_smbus_read_byte_data(client, 0x08);
	buf[1] = i2c_smbus_read_byte_data(client, 0x09);
	if ((buf[0] < 0) || (buf[1] < 0)) {
		printk(KERN_ERR "%s(): can't get device version\n", __func__);
		rc = -EIO;
		goto probe_exit;
	};
	printk(KERN_INFO "max17040 version %d.%d\n", buf[0], buf[1]);

	this = kzalloc(sizeof(struct max17040_data), GFP_KERNEL);
	if (!this) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	this->bat_ps.name		= MAX17040_NAME,
	this->bat_ps.type		= POWER_SUPPLY_TYPE_BATTERY,
	this->bat_ps.properties		= max17040_bat_main_props,
	this->bat_ps.num_properties	= ARRAY_SIZE(max17040_bat_main_props),
	this->bat_ps.get_property	= max17040_bat_get_property,
	this->bat_ps.external_power_changed =
			max17040_bat_external_power_changed,
	this->bat_ps.use_for_apm	= 1,

	this->clientp = client;
	this->curr_temp = MAX17040_TEMP_DEFAULT;
	this->pdata = client->dev.platform_data;

	rc = max17040_load_model(this);
	if (rc < 0) {
		dev_err(&client->dev, "%s(): Failed to load custom "
			"model, rc=%d\n", __func__, rc);

		/* reset max17040 and use default model */
		rc = max17040_power_on_request(this);
		if (rc < 0) {
			kfree(this);
			goto probe_exit;
		}
		this->exp_model = 0;
	} else {
		max17040_update_rcomp(this);

		/* did we load an expanded model? */
		if (this->pdata)
			this->exp_model = this->pdata->model_desc.exp;
	}

	max17040_get_soc(this);
	max17040_get_vcell(this);

	dev_info(&client->dev, "  vcell=%d mV\n", this->curr_mv);
	dev_info(&client->dev, "  soc=%d %%\n", this->curr_soc);

	/* verify intial soc */
	if (this->curr_soc <= MAX17040_SOC_CHECK &&
		this->curr_mv > MAX17040_VCELL_CHECK) {
		dev_info(&client->dev, "%s(): faulty soc reported, pulling a "
				"quick_start to reset soc algorithm\n",
				__func__);

		max17040_quick_start(this);
	}

	rc = power_supply_register(&client->dev, &this->bat_ps);
	if (rc) {
		kfree(this);
		goto probe_exit;
	}

	i2c_set_clientdata(client, this);

probe_exit:
	return rc;
}

static struct dev_pm_ops max17040_pm = {
	.suspend = max17040_pm_suspend,
	.resume = max17040_pm_resume,
};

static struct i2c_driver max17040_driver = {
	.driver = {
		.name = MAX17040_NAME,
		.owner = THIS_MODULE,
		.pm = &max17040_pm,
	},
	.probe = max17040_probe,
	.remove = __exit_p(max17040_remove),
	.id_table = max17040_id,
};

static int __init max17040_init(void)
{
	int rc;

	rc = i2c_add_driver(&max17040_driver);
	if (rc) {
		printk(KERN_ERR "max17040_init FAILED: i2c_add_driver rc=%d\n",
		       rc);
		goto init_exit;
	}
	return 0;

init_exit:
	return rc;
}

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_driver);
}

module_init(max17040_init);
module_exit(max17040_exit);

