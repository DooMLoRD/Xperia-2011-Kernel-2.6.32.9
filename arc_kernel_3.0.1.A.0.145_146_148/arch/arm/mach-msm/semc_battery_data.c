/* kernel/arch/arm/mach-msm/semc_battery_data.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Imre Sunyi <imre.sunyi@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/stat.h>

#include <mach/oem_rapi_client.h>
#include <mach/semc_battery_data.h>

#define OEM_RAPI_RECONNECT_S 5

#ifdef DEBUG
#define MUTEX_LOCK(x) do {						\
	pr_debug("%s: Locking mutex in %s\n", SEMC_BDATA_NAME, __func__);\
	mutex_lock(x);							\
} while (0)
#define MUTEX_UNLOCK(x) do {						\
	pr_debug("%s: Unlocking mutex in %s\n", SEMC_BDATA_NAME, __func__);\
	mutex_unlock(x);						\
} while (0)
#else
#define MUTEX_LOCK(x) mutex_lock(x)
#define MUTEX_UNLOCK(x) mutex_unlock(x)
#endif /* DEBUG */

/* #define DEBUG_FS */

#ifdef DEBUG_FS
struct override_value {
	u8 active;
	int value;
};

enum {
	BATT_ID_UNKNOWN = 0,
	BATT_ID_TYPE1,
	BATT_ID_TYPE2,
	BATT_VOLT,
};
#endif /* DEBUG_FS */

enum battery_technology {
	BATTERY_TECHNOLOGY_UNKNOWN = 0,
	BATTERY_TECHNOLOGY_TYPE1,
	BATTERY_TECHNOLOGY_TYPE2
};

struct battery_data {
	enum battery_technology technology;
	u32 cap_percent;
	s8 temp_celsius;
	s8 temp_celsius_amb;
};

struct notify_platform {
	u8 charging; /* 0 -> no, 1 -> yes */
	u8 battery_full;
	u8 power_collapse;
	u16 battery_charge_current;
	u16 charger_charge_current;
};

struct data_info {
	struct power_supply bdata_ps;
	struct battery_data bdata;
	struct notify_platform notify;
	u8 battery_ovp;
	u8 use_fuelgauge;

	u8 notify_changed;
	u8 ovp_changed;

	struct work_struct external_change_work;
	struct delayed_work oem_rapi_client_start_work;

	struct msm_rpc_client *rpc_client;
	struct mutex lock;

	void (*set_battery_health)(int health);

#ifdef DEBUG_FS
	struct override_value temperature_debug;
	struct override_value technology_debug;
	struct override_value temperature_amb_debug;
#endif
};

static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
};

static enum power_supply_property batt_props_with_fg[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
};

#ifdef DEBUG_FS
static int msm_get_event(struct data_info *di, u32 event, u8 event_switch,
			 u32 *value);
static ssize_t semc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);
static ssize_t store_temperature(struct device *pdev,
				 struct device_attribute *attr,
				 const char *pbuf,
				 size_t count);
static ssize_t store_technology(struct device *pdev,
				struct device_attribute *attr,
				const char *pbuf,
				size_t count);
static ssize_t store_temperature_ambient(struct device *pdev,
				 struct device_attribute *attr,
				 const char *pbuf,
				 size_t count);

static struct device_attribute semc_battery_attrs[] = {
	__ATTR(batt_id_unknown, S_IRUGO, semc_battery_show_property, NULL),
	__ATTR(batt_id_type1, S_IRUGO, semc_battery_show_property, NULL),
	__ATTR(batt_id_type2, S_IRUGO, semc_battery_show_property, NULL),
	__ATTR(batt_volt, S_IRUGO, semc_battery_show_property, NULL),
	__ATTR(set_temperature, S_IWUGO, NULL, store_temperature),
	__ATTR(set_technology, S_IWUGO, NULL, store_technology),
	__ATTR(set_temperature_ambient, S_IWUGO, NULL,
	       store_temperature_ambient),
};

static int semc_battery_create_attrs(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(semc_battery_attrs); i++)
		if (device_create_file(dev, &semc_battery_attrs[i]))
			goto semc_create_attrs_failed;

	return 0;

semc_create_attrs_failed:
	pr_err("%s: Failed creating semc battery attrs.\n", SEMC_BDATA_NAME);
	while (i--)
		device_remove_file(dev, &semc_battery_attrs[i]);

	return -EIO;
}

static ssize_t semc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int rc = 0;
	const ptrdiff_t offs = attr - semc_battery_attrs;
	struct power_supply *ps = dev_get_drvdata(dev);
	struct data_info *di;
	u32 event;

	if (!ps)
		return -EPERM;
	di = container_of(ps, struct data_info, bdata_ps);

	switch (offs) {
	case BATT_ID_UNKNOWN:
		rc = msm_get_event(di, OEM_RAPI_CLIENT_EVENT_BATT_ID_GET, 0,
				   &event);
		break;
	case BATT_ID_TYPE1:
		rc = msm_get_event(di, OEM_RAPI_CLIENT_EVENT_BATT_ID_TYPE1_GET,
				   0, &event);
		break;
	case BATT_ID_TYPE2:
		rc = msm_get_event(di, OEM_RAPI_CLIENT_EVENT_BATT_ID_TYPE2_GET,
				   0, &event);
		break;
	case BATT_VOLT:
		rc = msm_get_event(di, OEM_RAPI_CLIENT_EVENT_BATT_MV_GET, 0,
				   &event);
		break;
	default:
		rc = -EINVAL;
	}

	if (!rc)
		rc = scnprintf(buf, PAGE_SIZE, "%u\n", event);

	return rc;
}

static void semc_battery_remove_attrs(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(semc_battery_attrs); i++)
		(void)device_remove_file(dev, &semc_battery_attrs[i]);
}

static int read_sysfs_interface(const char *pbuf, s32 *pvalue, u8 base)
{
	long long val;
	int rc;

	if (!pbuf || !pvalue)
		return -EINVAL;

	rc = strict_strtoll(pbuf, base, &val);
	if (!rc)
		*pvalue = (s32)val;

	return rc;
}

static ssize_t store_temperature(struct device *pdev,
				 struct device_attribute *attr,
				 const char *pbuf,
				 size_t count)
{
	s32 temp;
	int rc = count;
	struct power_supply *ps = dev_get_drvdata(pdev);
	struct data_info *di = container_of(ps, struct data_info, bdata_ps);

	if (!read_sysfs_interface(pbuf, &temp, 10)
	    && temp >= -21 && temp <= 100) {
		MUTEX_LOCK(&di->lock);

		di->temperature_debug.active = 0;

		if (temp >= -20) {
			di->temperature_debug.active = 1;
			di->temperature_debug.value = temp;
		}

		MUTEX_UNLOCK(&di->lock);

		power_supply_changed(&di->bdata_ps);
	} else {
		pr_err("%s: Wrong input to sysfs set_temperature. "
		       "Expect [-21..100]. -21 releases the debug value\n",
		       SEMC_BDATA_NAME);
		rc = -EINVAL;
	}

	return rc;
}

static ssize_t store_technology(struct device *pdev,
				struct device_attribute *attr,
				const char *pbuf,
				size_t count)
{
	s32 tech;
	int rc = count;
	struct power_supply *ps = dev_get_drvdata(pdev);
	struct data_info *di = container_of(ps, struct data_info, bdata_ps);

	if (!read_sysfs_interface(pbuf, &tech, 10) &&
	    tech >= -1 && tech <= BATTERY_TECHNOLOGY_TYPE2) {
		MUTEX_LOCK(&di->lock);

		di->technology_debug.active = 0;

		if (tech >= BATTERY_TECHNOLOGY_UNKNOWN) {
			di->technology_debug.active = 1;
			di->technology_debug.value =
				(enum battery_technology)tech;
		}

		MUTEX_UNLOCK(&di->lock);

		power_supply_changed(&di->bdata_ps);
	} else {
		pr_err("%s: Wrong input to sysfs set_technology. Expect [-1..2]"
		       ". -1 releases the debug value\n", SEMC_BDATA_NAME);
		rc = -EINVAL;
	}

	return rc;
}

static ssize_t store_temperature_ambient(struct device *pdev,
					 struct device_attribute *attr,
					 const char *pbuf,
					 size_t count)
{
	s32 temp;
	int rc = count;
	struct power_supply *ps = dev_get_drvdata(pdev);
	struct data_info *di = container_of(ps, struct data_info, bdata_ps);

	if (!read_sysfs_interface(pbuf, &temp, 10)
	    && temp >= -21 && temp <= 100) {
		MUTEX_LOCK(&di->lock);

		di->temperature_amb_debug.active = 0;

		if (temp >= -20) {
			di->temperature_amb_debug.active = 1;
			di->temperature_amb_debug.value = temp;
		}

		MUTEX_UNLOCK(&di->lock);

		power_supply_changed(&di->bdata_ps);
	} else {
		pr_err("%s: Wrong input to sysfs set_temperature_ambient. "
		       "Expect [-21..100]. -21 releases the debug value\n",
		       SEMC_BDATA_NAME);
		rc = -EINVAL;
	}

	return rc;
}
#endif /* DEBUG_FS */

static int cutoff_level_cb(
		struct oem_rapi_client_streaming_func_cb_arg *arg,
		struct oem_rapi_client_streaming_func_cb_ret *ret)
{
	if (arg && OEM_RAPI_SERVER_EVENT_CUTOFF_CB_EVENT == arg->event) {
		struct data_info *di = arg->handle;

		pr_debug("%s: %s\n", SEMC_BDATA_NAME, __func__);

		if (di && di->set_battery_health)
			di->set_battery_health(POWER_SUPPLY_HEALTH_DEAD);
	}

	return 0;
}

static int bdata_change_cb(
	struct oem_rapi_client_streaming_func_cb_arg *arg,
	struct oem_rapi_client_streaming_func_cb_ret *ret)
{
	if (arg && OEM_RAPI_SERVER_EVENT_NOTIFY_BDATA_CB_EVENT == arg->event &&
	    sizeof(struct battery_data) == arg->in_len) {
		struct data_info *di = arg->handle;

		pr_debug("%s: %s\n", SEMC_BDATA_NAME, __func__);

		if (di) {
			MUTEX_LOCK(&di->lock);
			di->bdata = *(struct battery_data *)arg->input;
			MUTEX_UNLOCK(&di->lock);
			power_supply_changed(&di->bdata_ps);
		}

		pr_debug("%s: %s return\n", SEMC_BDATA_NAME, __func__);
	}

	return 0;
}

static int set_platform_callbacks(struct data_info *di)
{
	struct oem_rapi_client_streaming_func_arg client_arg = {0};
	struct oem_rapi_client_streaming_func_ret client_ret = {0};
	int rc;
	char dummy;

	MUTEX_LOCK(&di->lock);
	if (!di->rpc_client) {
		MUTEX_UNLOCK(&di->lock);
		return -ESRCH;
	}
	MUTEX_UNLOCK(&di->lock);

	/* Common setup */
	client_arg.handle = di;
	/* "input" must be set to something even if "in_len" is zero.
	 *  Otherwise the size (0) isn't sent and the server will reject the
	 * RPC message.
	 */
	client_arg.input = &dummy;

	/* Setup battery cutoff level callback */
	client_arg.event = OEM_RAPI_CLIENT_EVENT_CUTOFF_LEVEL_CB_REGISTER;
	client_arg.cb_func = cutoff_level_cb;

	rc = oem_rapi_client_streaming_function(di->rpc_client,
						&client_arg,
						&client_ret);
	if (rc) {
		pr_err("%s: Failed register cutoff level. Error %d\n",
		       SEMC_BDATA_NAME, rc);
		goto set_callbacks_exit;
	}

	/* Setup battery data change callback */
	client_arg.event = OEM_RAPI_CLIENT_EVENT_NOTIFY_BDATA_CB_REGISTER_SET;
	client_arg.cb_func = bdata_change_cb;

	rc = oem_rapi_client_streaming_function(di->rpc_client,
						&client_arg,
						&client_ret);
	if (rc)
		pr_err("%s: Failed register bdata change. Error %d\n",
		       SEMC_BDATA_NAME, rc);

set_callbacks_exit:
	return rc;
}

static void clear_platform_callbacks(struct data_info *di)
{
	struct oem_rapi_client_streaming_func_arg client_arg = {0};
	struct oem_rapi_client_streaming_func_ret client_ret = {0};
	int rc;
	char dummy;

	MUTEX_LOCK(&di->lock);
	if (!di->rpc_client) {
		MUTEX_UNLOCK(&di->lock);
		return;
	}
	MUTEX_UNLOCK(&di->lock);

	/* Common setup */
	/* "input" must be set to something even if "in_len" is zero.
	 *  Otherwise the size (0) isn't sent and the server will reject the
	 * RPC message.
	 */
	client_arg.input = &dummy;

	/* Clear battery cutoff level callback */
	client_arg.event =
		OEM_RAPI_CLIENT_EVENT_CUTOFF_LEVEL_CB_UNREGISTER_SET;
	rc = oem_rapi_client_streaming_function(di->rpc_client,
						&client_arg,
						&client_ret);
	if (rc)
		pr_err("%s: Failed unregister cutoff level. Error %d\n",
		       SEMC_BDATA_NAME, rc);

	/* Clear battery data change callback */
	client_arg.event =
		OEM_RAPI_CLIENT_EVENT_NOTIFY_BDATA_CB_UNREGISTER_SET;
	rc = oem_rapi_client_streaming_function(di->rpc_client,
						&client_arg,
						&client_ret);
	if (rc)
		pr_err("%s Failed unregister bdata change. Error %d\n",
		       SEMC_BDATA_NAME, rc);
}

static inline int get_technology(enum battery_technology tech)
{
	int ps_tech;

	switch (tech) {
	case BATTERY_TECHNOLOGY_TYPE1:
		ps_tech = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case BATTERY_TECHNOLOGY_TYPE2:
		ps_tech = POWER_SUPPLY_TECHNOLOGY_LiMn;
		break;
	case BATTERY_TECHNOLOGY_UNKNOWN:
	default:
		ps_tech = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;
	}

	return ps_tech;
}

static int bdata_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	int rc = 0;
	struct data_info *di = container_of(psy, struct data_info, bdata_ps);

	MUTEX_LOCK(&di->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	{
		enum battery_technology tech = di->bdata.technology;
#ifdef DEBUG_FS
		if (di->technology_debug.active)
			tech = (enum battery_technology)
				di->technology_debug.value;
#endif
		val->intval = get_technology(tech);
		break;
	}
	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->use_fuelgauge)
			val->intval = di->bdata.cap_percent;
		else
			rc = -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		/* Temperature in tenths of degree Celsius */
		val->intval = di->bdata.temp_celsius * 10;
#ifdef DEBUG_FS
		if (di->temperature_debug.active)
			val->intval = di->temperature_debug.value * 10;
#endif
		break;
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		/* Ambient temperature in tenths of degree Celsius */
		val->intval = di->bdata.temp_celsius_amb * 10;
#ifdef DEBUG_FS
		if (di->temperature_amb_debug.active)
			val->intval = di->temperature_amb_debug.value * 10;
#endif
		break;
	default:
		rc = -EINVAL;
		break;

	}

	MUTEX_UNLOCK(&di->lock);

	return rc;
}

#ifdef DEBUG_FS
static int msm_get_event(struct data_info *di, u32 event, u8 event_switch,
			 u32 *value)
{
	int rc = 0;
	struct oem_rapi_client_streaming_func_arg client_arg = {0};
	struct oem_rapi_client_streaming_func_ret client_ret = {0};

	MUTEX_LOCK(&di->lock);
	if (!di->rpc_client) {
		MUTEX_UNLOCK(&di->lock);
		return -ESRCH;
	}
	MUTEX_UNLOCK(&di->lock);

	client_arg.event = event;
	client_arg.in_len = sizeof(event_switch);
	client_arg.input = (char *)&event_switch;
	client_arg.out_len_valid = 1;
	client_arg.output_valid = 1;
	client_arg.output_size = sizeof(*value);

	rc = oem_rapi_client_streaming_function(di->rpc_client,
						&client_arg,
						&client_ret);
	if (rc < 0) {
		pr_err("%s: Failed getting event '%d'. Error %d\n",
		       SEMC_BDATA_NAME, event, rc);
	} else if (client_ret.out_len == NULL || client_ret.output == NULL ||
		   *client_ret.out_len == 0 ||
		   *client_ret.out_len > client_arg.output_size) {
		rc = -ENOMSG;
	} else {
		if (*client_ret.out_len == sizeof(u16))
			*value = *(u16 *)client_ret.output;
		else
			*value = *(u32 *)client_ret.output;
	}

	kfree(client_ret.out_len);
	kfree(client_ret.output);

	return rc;
}
#endif /* DEBUG_FS */

static int msm_set_ovp(struct data_info *di)
{
	int rc;
	struct oem_rapi_client_streaming_func_arg client_arg = {0};
	struct oem_rapi_client_streaming_func_ret client_ret = {0};
	u8 fet_switch_onoff = !di->battery_ovp; /* OVP 'on' -> FET 'off' and
						 * vice versa */

	MUTEX_LOCK(&di->lock);
	if (!di->rpc_client) {
		MUTEX_UNLOCK(&di->lock);
		return -ESRCH;
	}
	MUTEX_UNLOCK(&di->lock);

	client_arg.event = OEM_RAPI_CLIENT_EVENT_PM_BATT_FET_SWITCH_SET;
	client_arg.in_len = sizeof(fet_switch_onoff);
	client_arg.input = (char *)&fet_switch_onoff;
	client_arg.out_len_valid = 1;
	client_arg.output_valid = 1;
	client_arg.output_size = sizeof(u8);

	client_ret.out_len = NULL;
	client_ret.output = NULL;

	pr_info("%s: Setting OVP. Batt-FET %s\n",
		SEMC_BDATA_NAME, fet_switch_onoff ? "on" : "off");

	rc = oem_rapi_client_streaming_function(di->rpc_client,
						&client_arg,
						&client_ret);
	if (rc < 0) {
		pr_err("%s: Failed setting OVP switch. Error %d\n",
		       SEMC_BDATA_NAME, rc);
	} else if (client_ret.out_len == NULL ||
		   client_ret.output == NULL ||
		   *client_ret.out_len != client_arg.output_size) {
		return -ENOMSG;
	} else if (*(u8 *)client_ret.output) {
		rc = -ENOEXEC;
	}

	kfree(client_ret.out_len);
	kfree(client_ret.output);

	return rc;
}

static void msm_platform_notify(struct data_info *di)
{
	struct oem_rapi_client_streaming_func_arg client_arg = {0};
	struct oem_rapi_client_streaming_func_ret client_ret = {0};
	int rc;

	MUTEX_LOCK(&di->lock);
	if (!di->rpc_client) {
		MUTEX_UNLOCK(&di->lock);
		return;
	}
	MUTEX_UNLOCK(&di->lock);

	pr_debug("%s: %s\n", SEMC_BDATA_NAME, __func__);

	client_arg.event =
		OEM_RAPI_CLIENT_EVENT_NOTIFY_PLATFORM_SET;
	client_arg.in_len = sizeof(di->notify);
	client_arg.input = (char *)&di->notify;

	rc = oem_rapi_client_streaming_function(di->rpc_client,
						&client_arg,
						&client_ret);
	if (rc)
		pr_err("%s: Failed notify platform. Error %d\n",
		       SEMC_BDATA_NAME, rc);
}

static int get_ext_supplier_data(struct device *dev, void *data)
{
	struct power_supply *psy = (struct power_supply *)data;
	struct power_supply *ext = dev_get_drvdata(dev);
	struct data_info *di = container_of(psy, struct data_info, bdata_ps);
	union power_supply_propval ret;
	unsigned int i;

	for (i = 0; i < ext->num_supplicants; i++) {
		if (strcmp(ext->supplied_to[i], psy->name))
			continue;

		if (!ext->get_property(ext, POWER_SUPPLY_PROP_STATUS, &ret)) {
			struct notify_platform old_notify = di->notify;

			pr_debug("%s: got status %d\n", SEMC_BDATA_NAME,
				 ret.intval);

			di->notify.charging =
				(POWER_SUPPLY_STATUS_CHARGING == ret.intval);
			di->notify.battery_full =
				(POWER_SUPPLY_STATUS_FULL == ret.intval);
			if (memcmp(&old_notify, &di->notify,
				   sizeof(di->notify)))
				di->notify_changed = 1;
		}

		if (!ext->get_property(ext, POWER_SUPPLY_PROP_HEALTH, &ret)) {
			u8 old_ovp = di->battery_ovp;

			pr_debug("%s: got health %d\n", SEMC_BDATA_NAME,
				 ret.intval);

			di->battery_ovp =
				(POWER_SUPPLY_HEALTH_OVERVOLTAGE == ret.intval);

			if (old_ovp != di->battery_ovp)
				di->ovp_changed = 1;
		}
	}

	return 0;
}

static void bdata_external_power_changed_work(struct work_struct *work)
{
	struct data_info *di = container_of(work, struct data_info,
					    external_change_work);

	MUTEX_LOCK(&di->lock);

	di->notify_changed = 0;
	di->ovp_changed = 0;

	class_for_each_device(power_supply_class, NULL, &di->bdata_ps,
			      get_ext_supplier_data);

	MUTEX_UNLOCK(&di->lock);

	if (di->notify_changed)
		msm_platform_notify(di);
	if (di->ovp_changed)
		msm_set_ovp(di);
}

static void bdata_external_power_changed(struct power_supply *ps)
{
	struct data_info *di = container_of(ps, struct data_info, bdata_ps);

	schedule_work(&di->external_change_work);
}

static void bdata_oem_rapi_client_start_work(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work,
						  work);
	struct data_info *di = container_of(dwork, struct data_info,
					    oem_rapi_client_start_work);

	if (!di->rpc_client) {
		MUTEX_LOCK(&di->lock);
		di->rpc_client = oem_rapi_client_init();
		if (!di->rpc_client || IS_ERR(di->rpc_client)) {
			pr_err("%s: Failed initialize oem rapi client\n",
			       SEMC_BDATA_NAME);
			di->rpc_client = NULL;
			MUTEX_UNLOCK(&di->lock);
			schedule_delayed_work(&di->oem_rapi_client_start_work,
					      OEM_RAPI_RECONNECT_S * HZ);
			return;
		}
		MUTEX_UNLOCK(&di->lock);

		set_platform_callbacks(di);
	}
}

static int bdata_suspend(struct platform_device *dev, pm_message_t state)
{
	struct data_info *di = platform_get_drvdata(dev);

	pr_debug("%s: %s\n", SEMC_BDATA_NAME, __func__);

	if (work_pending(&di->external_change_work))
		flush_work(&di->external_change_work);

	if (delayed_work_pending(&di->oem_rapi_client_start_work))
		cancel_delayed_work(&di->oem_rapi_client_start_work);

	MUTEX_LOCK(&di->lock);
	di->notify.power_collapse = 1;
	MUTEX_UNLOCK(&di->lock);

	msm_platform_notify(di);

	return 0;
}

static int bdata_resume(struct platform_device *dev)
{
	struct data_info *di = platform_get_drvdata(dev);

	pr_debug("%s: %s\n", SEMC_BDATA_NAME, __func__);

	if (!di->rpc_client)
		schedule_delayed_work(&di->oem_rapi_client_start_work, 0);

	MUTEX_LOCK(&di->lock);
	di->notify.power_collapse = 0;
	MUTEX_UNLOCK(&di->lock);

	msm_platform_notify(di);

	return 0;
}

static int bdata_probe(struct platform_device *pdev)
{
	struct semc_battery_platform_data *pdata;
	struct data_info *di;

	pr_info("%s: probe\n", SEMC_BDATA_NAME);

	di = kzalloc(sizeof(struct data_info), GFP_KERNEL);
	if (!di) {
		pr_err("%s: Memory alloc fail\n", SEMC_BDATA_NAME);
		return -ENOMEM;
	}

	di->bdata_ps.name = pdev->name;
	di->bdata_ps.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bdata_ps.properties = batt_props;
	di->bdata_ps.num_properties = ARRAY_SIZE(batt_props);
	di->bdata_ps.get_property = bdata_get_property;
	di->bdata_ps.external_power_changed =
		bdata_external_power_changed;

	di->bdata.technology = BATTERY_TECHNOLOGY_UNKNOWN;
	di->bdata.cap_percent = 50;
	di->bdata.temp_celsius = 25;
	di->bdata.temp_celsius_amb = 25;

	pdata = (struct semc_battery_platform_data *)pdev->dev.platform_data;
	if (pdata) {
		di->set_battery_health = pdata->set_battery_health;
		di->use_fuelgauge = pdata->use_fuelgauge;

		if (di->use_fuelgauge) {
			di->bdata_ps.properties = batt_props_with_fg;
			di->bdata_ps.num_properties =
				ARRAY_SIZE(batt_props_with_fg);
		}

		if (pdata->supplied_to) {
			di->bdata_ps.supplied_to = pdata->supplied_to;
			di->bdata_ps.num_supplicants =
				pdata->num_supplicants;
		}
	}

	mutex_init(&di->lock);

	INIT_WORK(&di->external_change_work,
		  bdata_external_power_changed_work);
	INIT_DELAYED_WORK(&di->oem_rapi_client_start_work,
			  bdata_oem_rapi_client_start_work);

	if (power_supply_register(&pdev->dev, &di->bdata_ps)) {
		pr_err("%s: Failed to register power supply\n",
		       SEMC_BDATA_NAME);
		kfree(di);
		return -EPERM;
	}

	platform_set_drvdata(pdev, di);

	schedule_delayed_work(&di->oem_rapi_client_start_work, 0);

#ifdef DEBUG_FS
	if (semc_battery_create_attrs(di->bdata_ps.dev))
		pr_info("%s: Debug support failed\n", SEMC_BDATA_NAME);
#endif

	return 0;
}

static int __exit bdata_remove(struct platform_device *pdev)
{
	struct data_info *di = platform_get_drvdata(pdev);

	pr_info("%s: remove\n", SEMC_BDATA_NAME);

	clear_platform_callbacks(di);
	oem_rapi_client_close();
#ifdef DEBUG_FS
	semc_battery_remove_attrs(di->bdata_ps.dev);
#endif
	power_supply_unregister(&di->bdata_ps);
	kfree(di);

	return 0;
}

static struct platform_driver bdata_driver = {
	.probe = bdata_probe,
	.remove = __exit_p(bdata_remove),
	.suspend = bdata_suspend,
	.resume = bdata_resume,
	.driver = {
		.name = SEMC_BDATA_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init bdata_init(void)
{
	int rc;

	pr_debug("%s: Initializing...\n", SEMC_BDATA_NAME);

	rc = platform_driver_register(&bdata_driver);
	if (rc) {
		pr_err("%s: Failed register platform driver. rc = %d\n",
		       SEMC_BDATA_NAME, rc);
	}

	return rc;
}

static void __exit bdata_exit(void)
{
	pr_debug("%s: Exiting...\n", SEMC_BDATA_NAME);

	platform_driver_unregister(&bdata_driver);
}

module_init(bdata_init);
module_exit(bdata_exit);

MODULE_AUTHOR("Imre Sunyi");
MODULE_DESCRIPTION("Battery data handling for Sony Ericsson Mobile"
		   " Communications");

MODULE_LICENSE("GPLv2");
