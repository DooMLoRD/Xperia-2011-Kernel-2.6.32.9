/*
 * Copyright (c) 2009,2010 Sony Ericsson Mobile Comm
 *
 * Author: Courtney Cavin <courtney.cavin@sonyericsson.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/leds-as3676.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/delay.h>

#define AS3676_NAME "as3676"

enum as3676_cmode {
	AS3676_CMODE_IMMEDIATE,
	AS3676_CMODE_SCHEDULED,
};

enum as3676_register {
	AS3676_REG_CTRL,
	AS3676_LDO_VOLTAGE,
	AS3676_MODE_SWITCH,
	AS3676_MODE_SWITCH_2,

	AS3676_REG_PWM_CTRL,
	AS3676_REG_PWM_CODE,

	AS3676_REG_GPIO_CURR,

	AS3676_ADC_CTRL,

	AS3676_DCDC_CTRL_1,
	AS3676_DCDC_CTRL_2,

	AS3676_DLS_CTRL_1,
	AS3676_DLS_CTRL_2,

	AS3676_SINK_1_2_CTRL,
	AS3676_SINK_06_CTRL,
	AS3676_SINK_3X_CTRL,
	AS3676_SINK_4X_CTRL,

#ifdef CONFIG_LEDS_AS3676_HW_BLINK
	/* AS3676_REG_PATTERN_XXX must go after AS3676_SINK_4X_CTRL */
	AS3676_REG_PATTERN_DATA_0,
	AS3676_REG_PATTERN_DATA_1,
	AS3676_REG_PATTERN_DATA_2,
	AS3676_REG_PATTERN_DATA_3,
	AS3676_REG_PATTERN_CTRL,
#endif

	AS3676_SINK_01_CURR,
	AS3676_SINK_02_CURR,

	AS3676_SINK_06_CURR,

	AS3676_SINK_30_CURR,
	AS3676_SINK_31_CURR,
	AS3676_SINK_32_CURR,
	AS3676_SINK_33_CURR,

	AS3676_SINK_41_CURR,
	AS3676_SINK_42_CURR,
	AS3676_SINK_43_CURR,

	AS3676_SINK_RGB1_CURR,
	AS3676_SINK_RGB2_CURR,
	AS3676_SINK_RGB3_CURR,

	AS3676_OVERTEMP_CTRL,

	AS3676_AMB_CTRL,
	AS3676_AMB_FILTER,
	AS3676_AMB_OFFSET,
	AS3676_SINK_1_2_AMB,
	AS3676_SINK_06_AMB,
	AS3676_SINK_3X_AMB,
	AS3676_SINK_4X_AMB,
	AS3676_AMB_RGB_GRP,
	AS3676_GROUP_1_Y0,
	AS3676_GROUP_1_Y3,
	AS3676_GROUP_1_X1,
	AS3676_GROUP_1_X2,
	AS3676_GROUP_1_K1,
	AS3676_GROUP_1_K2,
	AS3676_GROUP_2_Y0,
	AS3676_GROUP_2_Y3,
	AS3676_GROUP_2_X1,
	AS3676_GROUP_2_X2,
	AS3676_GROUP_2_K1,
	AS3676_GROUP_2_K2,
	AS3676_GROUP_3_Y0,
	AS3676_GROUP_3_Y3,
	AS3676_GROUP_3_X1,
	AS3676_GROUP_3_X2,
	AS3676_GROUP_3_K1,
	AS3676_GROUP_3_K2,
	AS3676_AUDIO_CTRL_1,
	AS3676_AUDIO_CTRL_2,
	AS3676_SINK_3X_AUD_SRC,
	AS3676_AUDIO_INPUT,
	AS3676_AUDIO_OUTPUT,

	AS3676_REG_MAX,

	AS3676_SINK_RGB_CTRL = AS3676_SINK_06_CTRL,
	AS3676_SINK_RGB_AMB = AS3676_SINK_06_AMB,
};

enum as3676_ctrl_value {
	AS3676_CTRL_OFF,
	AS3676_CTRL_ON,
	AS3676_CTRL_PWM, /* controlled via PWM */
	AS3676_CTRL_PATTERN, /* controlled via pattern */
	AS3676_CTRL_EXT_CURR = 0x03,

	AS3676_CTRL_MASK = 0x03
};

enum as3676_sink_flags {
	AS3676_FLAG_DCDC_CTRL  = (1 << 0), /* led controls step up ctrlr */
	AS3676_FLAG_EXT_CURR   = (1 << 1), /* powered via external current */
};

int as3676_sink_map[] = {
	[AS3676_SINK_01] = AS3676_SINK_01_CURR,
	[AS3676_SINK_02] = AS3676_SINK_02_CURR,
	[AS3676_SINK_06] = AS3676_SINK_06_CURR,
	[AS3676_SINK_30] = AS3676_SINK_30_CURR,
	[AS3676_SINK_31] = AS3676_SINK_31_CURR,
	[AS3676_SINK_32] = AS3676_SINK_32_CURR,
	[AS3676_SINK_33] = AS3676_SINK_33_CURR,
	[AS3676_SINK_41] = AS3676_SINK_41_CURR,
	[AS3676_SINK_42] = AS3676_SINK_42_CURR,
	[AS3676_SINK_43] = AS3676_SINK_43_CURR,
	[AS3676_SINK_RGB1] = AS3676_SINK_RGB1_CURR,
	[AS3676_SINK_RGB2] = AS3676_SINK_RGB2_CURR,
	[AS3676_SINK_RGB3] = AS3676_SINK_RGB3_CURR,

};

static const struct as3676_sink {
	enum as3676_register ctrl;
	enum as3676_register amb;
	enum as3676_register dls;
	int lower_bit;
	int flags;
	int dls_bit;
} as3676_sink[] = {
	[AS3676_SINK_01_CURR] = {
		.ctrl = AS3676_SINK_1_2_CTRL,
		.amb = AS3676_SINK_1_2_AMB,
		.lower_bit = 0,
		.dls = AS3676_DLS_CTRL_2,
		.dls_bit = 0,
		.flags = AS3676_FLAG_DCDC_CTRL,
	},
	[AS3676_SINK_02_CURR] = {
		.ctrl = AS3676_SINK_1_2_CTRL,
		.amb = AS3676_SINK_1_2_AMB,
		.lower_bit = 2,
		.dls = AS3676_DLS_CTRL_2,
		.dls_bit = 1,
	},
	[AS3676_SINK_06_CURR] = {
		.ctrl = AS3676_SINK_06_CTRL,
		.amb = AS3676_SINK_06_AMB,
		.lower_bit = 6,
		.flags = AS3676_FLAG_DCDC_CTRL,
		.dls = AS3676_DLS_CTRL_2,
		.dls_bit = 7,
	},
	[AS3676_SINK_30_CURR] = {
		.ctrl = AS3676_SINK_3X_CTRL,
		.amb = AS3676_SINK_3X_AMB,
		.lower_bit = 0,
		.flags = AS3676_FLAG_EXT_CURR,
		.dls = AS3676_DLS_CTRL_1,
		.dls_bit = 0,
	},
	[AS3676_SINK_31_CURR] = {
		.ctrl = AS3676_SINK_3X_CTRL,
		.amb = AS3676_SINK_3X_AMB,
		.lower_bit = 2,
		.flags = AS3676_FLAG_EXT_CURR,
		.dls = AS3676_DLS_CTRL_1,
		.dls_bit = 1,
	},
	[AS3676_SINK_32_CURR] = {
		.ctrl = AS3676_SINK_3X_CTRL,
		.amb = AS3676_SINK_3X_AMB,
		.lower_bit = 4,
		.flags = AS3676_FLAG_EXT_CURR,
		.dls = AS3676_DLS_CTRL_1,
		.dls_bit = 2,
	},
	[AS3676_SINK_33_CURR] = {
		.ctrl = AS3676_SINK_3X_CTRL,
		.amb = AS3676_SINK_3X_AMB,
		.lower_bit = 6,
		.flags = AS3676_FLAG_EXT_CURR,
		.dls = AS3676_DLS_CTRL_1,
		.dls_bit = 3,
	},
	[AS3676_SINK_41_CURR] = {
		.ctrl = AS3676_SINK_4X_CTRL,
		.amb = AS3676_SINK_4X_AMB,
		.lower_bit = 0,
		.dls = AS3676_DLS_CTRL_2,
		.dls_bit = 2,
	},
	[AS3676_SINK_42_CURR] = {
		.ctrl = AS3676_SINK_4X_CTRL,
		.amb = AS3676_SINK_4X_AMB,
		.lower_bit = 2,
		.dls = AS3676_DLS_CTRL_2,
		.dls_bit = 3,
	},
	[AS3676_SINK_43_CURR] = {
		.ctrl = AS3676_SINK_4X_CTRL,
		.amb = AS3676_SINK_4X_AMB,
		.lower_bit = 4,
		.dls = AS3676_DLS_CTRL_2,
		.dls_bit = 4,
	},
	[AS3676_SINK_RGB1_CURR] = {
		.ctrl = AS3676_SINK_RGB_CTRL,
		.amb = AS3676_SINK_RGB_AMB,
		.lower_bit = 0,
		.dls = AS3676_DLS_CTRL_1,
		.dls_bit = 4,
	},
	[AS3676_SINK_RGB2_CURR] = {
		.ctrl = AS3676_SINK_RGB_CTRL,
		.amb = AS3676_SINK_RGB_AMB,
		.lower_bit = 2,
		.dls = AS3676_DLS_CTRL_1,
		.dls_bit = 5,
	},
	[AS3676_SINK_RGB3_CURR] = {
		.ctrl = AS3676_SINK_RGB_CTRL,
		.amb = AS3676_SINK_RGB_AMB,
		.lower_bit = 4,
		.dls = AS3676_DLS_CTRL_1,
		.dls_bit = 6,
	},
};

static const u8 as3676_i2c_registers[] = {
	[AS3676_SINK_01_CURR]   = 0x09,
	[AS3676_SINK_02_CURR]   = 0x0a,
	[AS3676_SINK_06_CURR]   = 0x2f,
	[AS3676_SINK_30_CURR]   = 0x40,
	[AS3676_SINK_31_CURR]   = 0x41,
	[AS3676_SINK_32_CURR]   = 0x42,
	[AS3676_SINK_33_CURR]   = 0x43,
	[AS3676_SINK_41_CURR]   = 0x13,
	[AS3676_SINK_42_CURR]   = 0x14,
	[AS3676_SINK_43_CURR]   = 0x15,
	[AS3676_SINK_RGB1_CURR] = 0x0b,
	[AS3676_SINK_RGB2_CURR] = 0x0c,
	[AS3676_SINK_RGB3_CURR] = 0x0d,
	[AS3676_SINK_3X_CTRL]   = 0x03,
	[AS3676_SINK_4X_CTRL]   = 0x04,
	[AS3676_SINK_1_2_CTRL]  = 0x01,
	[AS3676_SINK_RGB_CTRL]  = 0x02,
	[AS3676_REG_PWM_CTRL]   = 0x16,
	[AS3676_REG_PWM_CODE]   = 0x17,
	[AS3676_REG_CTRL]       = 0x00,
	[AS3676_LDO_VOLTAGE]    = 0x07,
	[AS3676_MODE_SWITCH]    = 0x24,
	[AS3676_MODE_SWITCH_2]  = 0x25,
	[AS3676_ADC_CTRL]       = 0x26,
	[AS3676_REG_GPIO_CURR]  = 0x2c,
	[AS3676_OVERTEMP_CTRL]  = 0x29,
#ifdef CONFIG_LEDS_AS3676_HW_BLINK
	[AS3676_REG_PATTERN_DATA_0] = 0x19,
	[AS3676_REG_PATTERN_DATA_1] = 0x1a,
	[AS3676_REG_PATTERN_DATA_2] = 0x1b,
	[AS3676_REG_PATTERN_DATA_3] = 0x1c,
	[AS3676_REG_PATTERN_CTRL] = 0x18,
#endif
	[AS3676_AUDIO_CTRL_1]   = 0x46,
	[AS3676_AUDIO_INPUT]     = 0x47,
	[AS3676_AUDIO_OUTPUT]   = 0x48,
	[AS3676_SINK_3X_AUD_SRC]  = 0x53,
	[AS3676_AUDIO_CTRL_2]   = 0x55,
	[AS3676_DLS_CTRL_1]     = 0x56,
	[AS3676_DLS_CTRL_2]     = 0x57,
	[AS3676_DCDC_CTRL_1]    = 0x21,
	[AS3676_DCDC_CTRL_2]    = 0x22,
	[AS3676_AMB_CTRL]       = 0x90,
	[AS3676_AMB_FILTER]     = 0x91,
	[AS3676_AMB_OFFSET]     = 0x92,
	[AS3676_SINK_1_2_AMB]   = 0x94,
	[AS3676_SINK_06_AMB]    = 0x95,
	[AS3676_SINK_3X_AMB]    = 0x96,
	[AS3676_SINK_4X_AMB]    = 0x97,
	[AS3676_GROUP_1_Y0]     = 0x98,
	[AS3676_GROUP_1_Y3]     = 0x99,
	[AS3676_GROUP_1_X1]     = 0x9a,
	[AS3676_GROUP_1_X2]     = 0x9c,
	[AS3676_GROUP_1_K1]     = 0x9b,
	[AS3676_GROUP_1_K2]     = 0x9d,
	[AS3676_GROUP_2_Y0]     = 0x9e,
	[AS3676_GROUP_2_Y3]     = 0x9f,
	[AS3676_GROUP_2_X1]     = 0xa0,
	[AS3676_GROUP_2_X2]     = 0xa2,
	[AS3676_GROUP_2_K1]     = 0xa1,
	[AS3676_GROUP_2_K2]     = 0xa3,
	[AS3676_GROUP_3_Y0]     = 0xa4,
	[AS3676_GROUP_3_Y3]     = 0xa5,
	[AS3676_GROUP_3_X1]     = 0xa6,
	[AS3676_GROUP_3_X2]     = 0xa8,
	[AS3676_GROUP_3_K1]     = 0xa7,
	[AS3676_GROUP_3_K2]     = 0xa9,
};

static u8 as3676_restore_regs[] = {
	AS3676_REG_GPIO_CURR,
	AS3676_DCDC_CTRL_1,
	AS3676_DCDC_CTRL_2,
	AS3676_LDO_VOLTAGE,
	AS3676_MODE_SWITCH,
	AS3676_REG_PWM_CODE,
	AS3676_REG_PWM_CTRL,
	AS3676_AMB_FILTER,
	AS3676_AMB_OFFSET,
	AS3676_SINK_1_2_AMB,
	AS3676_SINK_06_AMB,
	AS3676_SINK_3X_AMB,
	AS3676_SINK_4X_AMB,
	AS3676_AMB_RGB_GRP,
	AS3676_GROUP_1_Y0,
	AS3676_GROUP_1_Y3,
	AS3676_GROUP_1_X1,
	AS3676_GROUP_1_X2,
	AS3676_GROUP_1_K1,
	AS3676_GROUP_1_K2,
	AS3676_GROUP_2_Y0,
	AS3676_GROUP_2_Y3,
	AS3676_GROUP_2_X1,
	AS3676_GROUP_2_X2,
	AS3676_GROUP_2_K1,
	AS3676_GROUP_2_K2,
	AS3676_GROUP_3_Y0,
	AS3676_GROUP_3_Y3,
	AS3676_GROUP_3_X1,
	AS3676_GROUP_3_X2,
	AS3676_GROUP_3_K1,
	AS3676_GROUP_3_K2,
};

#define AS3676_MAX_CURRENT  38250

#define AS3676_SLOW_PATTERN_BIT_DURATION_MS  250
#define AS3676_FAST_PATTERN_BIT_DURATION_MS  31

#define AS3676_REG_CTRL_WAIT_US  5000
#define AS3676_ALS_ENABLE_WAIT_MS 2

/* You can not possibly (probably?) have more interfaces than sinks.... Well,
 * you *could* but it would be really really stupid */
#define AS3676_INTERFACE_MAX AS3676_SINK_MAX

static const struct as3676_als_config as3676_default_config = {
	.gain = AS3676_GAIN_1,
	.filter_up = AS3676_FILTER_1HZ,
	.filter_down = AS3676_FILTER_4HZ,
	.source = AS3676_ALS_SOURCE_GPIO2,
	.curve = {
		[AS3676_AMB_OFF] = {
			.y0 = 0,
			.y3 = 0,
			.k1 = 0,
			.k2 = 0,
			.x1 = 0,
			.x2 = 0,
		},
		[AS3676_AMB_GROUP_1] = {
			.y0 = 48,
			.y3 = 255,
			.k1 = 48,
			.k2 = 48,
			.x1 = 5,
			.x2 = 127,
		},
		[AS3676_AMB_GROUP_2] = {
			.y0 = 48,
			.y3 = 255,
			.k1 = 48,
			.k2 = 48,
			.x1 = 5,
			.x2 = 127,
		},
		[AS3676_AMB_GROUP_3] = {
			.y0 = 48,
			.y3 = 255,
			.k1 = 48,
			.k2 = 48,
			.x1 = 5,
			.x2 = 127,
		},
	},
};

struct as3676_interface {
	int index;
	u64 regs;
	int flags;
	int max_current;
	struct led_classdev cdev;
	struct kobject kobj;
};

enum als_suspend_state {
	AS3676_NO_SUSPEND = 0,
	AS3676_SUSPENDED,
};

struct as3676_record {
	struct i2c_client *client;
	struct as3676_interface interfaces[AS3676_INTERFACE_MAX];
	int n_interfaces;
	struct work_struct work;
	struct delayed_work als_resume_work;
	struct delayed_work als_enabled_work;
	u8 registers[AS3676_REG_MAX];
	u64 dcdcbit;
	u64 regbit;
	int als_connected;
	int als_wait;
	enum als_suspend_state als_suspend;
	int dls_connected;
	int als_enabled;
	int audio_enabled;
	enum as3676_cmode cmode;
	struct as3676_als_config als;
	struct as3676_audio_config audio;
	struct mutex lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#define as3676_lock(rd) mutex_lock(&(rd)->lock)
#define as3676_unlock(rd) mutex_unlock(&(rd)->lock)

static void as3676_als_set_enable_internal(struct as3676_record *rd, u8 enable);
static void as3676_als_set_enable(struct as3676_record *rd,
		struct as3676_interface *intf, u8 enable);
static void as3676_als_set_params(struct as3676_record *rd,
				struct as3676_als_config *param);
static void as3676_als_set_adc_ctrl(struct as3676_record *rd);
static void as3676_set_interface_brightness(struct as3676_interface *intf,
				enum led_brightness value);

static inline u8 reg_get(struct as3676_record *rd, enum as3676_register reg)
{
	u8 ret;

	ret = rd->registers[reg];

	return ret;
}

static inline int reg_isset(struct as3676_record *rd, enum as3676_register reg)
{
	return !!(rd->regbit & ((u64)1 << reg));
}

static inline void reg_set(struct as3676_record *rd,
		enum as3676_register reg, u8 val)
{
	if (rd->cmode == AS3676_CMODE_IMMEDIATE) {
		struct as3676_data {
			u8 addr;
			u8 value;
		} __attribute__((packed));
		struct as3676_data data;

		data.addr = as3676_i2c_registers[reg];
		data.value = val;
		i2c_master_send(rd->client, (u8 *)&data, sizeof(data));
		rd->registers[reg] = val;
	} else {
		rd->regbit |= ((u64)1 << reg);
		rd->registers[reg] = val;
	}
}

static void as3676_worker(struct work_struct *work)
{
	struct as3676_data {
		u8 addr;
		u8 value;
	} __attribute__((packed));
	int i;
	struct as3676_record *rd;
	struct as3676_data data;

	rd = container_of(work, struct as3676_record, work);

	as3676_lock(rd);

	if (rd->regbit == 0) {
		as3676_unlock(rd);
		return;
	}

	for (i = 0; i < AS3676_REG_MAX; ++i) {
		if (reg_isset(rd, i)) {
			data.addr = as3676_i2c_registers[i];
			data.value = rd->registers[i];
			i2c_master_send(rd->client, (u8 *)&data, sizeof(data));
		}
	}

	rd->regbit = 0;

	as3676_unlock(rd);
}

static void as3676_als_resume_worker(struct work_struct *work)
{
	struct as3676_record *rd;
	u8 val;

	rd = container_of(work, struct as3676_record, als_resume_work.work);

	as3676_lock(rd);
	val = reg_get(rd, AS3676_REG_CTRL);

	if (val) {
		enum as3676_cmode cmode = rd->cmode;

		rd->cmode = AS3676_CMODE_IMMEDIATE;
		as3676_als_set_enable_internal(rd, rd->als_enabled);
		if (!rd->audio_enabled)
			reg_set(rd, AS3676_ADC_CTRL, rd->als.source);
		rd->cmode = cmode;
	}
	as3676_unlock(rd);
}

static void as3676_als_enabled_worker(struct work_struct *work)
{
	struct as3676_record *rd;
	int i;

	rd = container_of(work, struct as3676_record, als_enabled_work.work);

	as3676_lock(rd);
	rd->als_suspend = AS3676_NO_SUSPEND;

	for (i = 0; i < rd->n_interfaces; ++i) {
		struct as3676_interface *intf = &rd->interfaces[i];
		enum led_brightness value;
		value = intf->cdev.brightness;
		as3676_set_interface_brightness(intf, value);
	}
	as3676_unlock(rd);
}

static void as3676_set_amb(struct as3676_record *rd, enum as3676_register reg,
	int group)
{
	enum as3676_register amb_reg;
	int off_bits;
	int amb_val;

	amb_reg = as3676_sink[reg].amb;
	off_bits = as3676_sink[reg].lower_bit;


	amb_val = reg_get(rd, amb_reg) & ~(AS3676_AMB_MASK << off_bits);
	amb_val |= group << off_bits;

	reg_set(rd, amb_reg, amb_val);
}

static void as3676_set_interface_amb(struct as3676_record *rd,
		struct as3676_interface *intf, int group)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(as3676_sink); ++i) {
		if (intf->regs & ((u64)1 << i))
			as3676_set_amb(rd, i, group);
	}
}

static void as3676_set_als_config(struct as3676_record *rd,
		const struct as3676_als_config *config,
		struct as3676_interface *intf)
{
	enum as3676_amb_value group;

	switch (intf->flags & AS3676_FLAG_ALS_MASK) {
	case AS3676_FLAG_ALS_GROUP1:
		group = AS3676_AMB_GROUP_1;
		break;
	case AS3676_FLAG_ALS_GROUP2:
		group = AS3676_AMB_GROUP_2;
		break;
	case AS3676_FLAG_ALS_GROUP3:
		group = AS3676_AMB_GROUP_3;
		break;
	default:
		group = AS3676_AMB_OFF;
		break;
	}

	if (!rd->audio_enabled)
		reg_set(rd, AS3676_ADC_CTRL, config->source);

	switch (group) {
	case AS3676_AMB_GROUP_1:
		reg_set(rd, AS3676_GROUP_1_Y0, config->curve[group].y0);
		reg_set(rd, AS3676_GROUP_1_Y3, config->curve[group].y3);
		reg_set(rd, AS3676_GROUP_1_X1, config->curve[group].x1);
		reg_set(rd, AS3676_GROUP_1_K1, config->curve[group].k1);
		reg_set(rd, AS3676_GROUP_1_X2, config->curve[group].x2);
		reg_set(rd, AS3676_GROUP_1_K2, config->curve[group].k2);
		break;
	case AS3676_AMB_GROUP_2:
		reg_set(rd, AS3676_GROUP_2_Y0, config->curve[group].y0);
		reg_set(rd, AS3676_GROUP_2_Y3, config->curve[group].y3);
		reg_set(rd, AS3676_GROUP_2_X1, config->curve[group].x1);
		reg_set(rd, AS3676_GROUP_2_K1, config->curve[group].k1);
		reg_set(rd, AS3676_GROUP_2_X2, config->curve[group].x2);
		reg_set(rd, AS3676_GROUP_2_K2, config->curve[group].k2);
		break;
	case AS3676_AMB_GROUP_3:
		reg_set(rd, AS3676_GROUP_3_Y0, config->curve[group].y0);
		reg_set(rd, AS3676_GROUP_3_Y3, config->curve[group].y3);
		reg_set(rd, AS3676_GROUP_3_X1, config->curve[group].x1);
		reg_set(rd, AS3676_GROUP_3_K1, config->curve[group].k1);
		reg_set(rd, AS3676_GROUP_3_X2, config->curve[group].x2);
		reg_set(rd, AS3676_GROUP_3_K2, config->curve[group].k2);
		break;
	default:
		break;
	}

	as3676_set_interface_amb(rd, intf, group);

	schedule_work(&rd->work);
}

static void as3676_get_als_config(struct as3676_record *rd,
		struct as3676_als_curve *curve, enum as3676_amb_value group)
{
	switch (group) {
	case AS3676_AMB_GROUP_1:
		curve->y0 = reg_get(rd, AS3676_GROUP_1_Y0);
		curve->y3 = reg_get(rd, AS3676_GROUP_1_Y3);
		curve->x1 = reg_get(rd, AS3676_GROUP_1_X1);
		curve->k1 = reg_get(rd, AS3676_GROUP_1_K1);
		curve->x2 = reg_get(rd, AS3676_GROUP_1_X2);
		curve->k2 = reg_get(rd, AS3676_GROUP_1_K2);
		break;
	case AS3676_AMB_GROUP_2:
		curve->y0 = reg_get(rd, AS3676_GROUP_2_Y0);
		curve->y3 = reg_get(rd, AS3676_GROUP_2_Y3);
		curve->x1 = reg_get(rd, AS3676_GROUP_2_X1);
		curve->k1 = reg_get(rd, AS3676_GROUP_2_K1);
		curve->x2 = reg_get(rd, AS3676_GROUP_2_X2);
		curve->k2 = reg_get(rd, AS3676_GROUP_2_K2);
		break;
	case AS3676_AMB_GROUP_3:
		curve->y0 = reg_get(rd, AS3676_GROUP_3_Y0);
		curve->y3 = reg_get(rd, AS3676_GROUP_3_Y3);
		curve->x1 = reg_get(rd, AS3676_GROUP_3_X1);
		curve->k1 = reg_get(rd, AS3676_GROUP_3_K1);
		curve->x2 = reg_get(rd, AS3676_GROUP_3_X2);
		curve->k2 = reg_get(rd, AS3676_GROUP_3_K2);
		break;
	default:
		break;
	}
}

static void as3676_audio_set_config(struct as3676_record *rd, u8 enable)
{
	int als_status = 0;

	rd->audio_enabled = enable;

	if (enable) {
		als_status = rd->als_enabled;
		as3676_als_set_enable_internal(rd, 0);
		as3676_als_set_adc_ctrl(rd);
		rd->als_enabled = als_status;
		reg_set(rd, AS3676_SINK_3X_AUD_SRC, rd->audio.current_3x);
		reg_set(rd, AS3676_AUDIO_CTRL_1, rd->audio.audio_control);
		reg_set(rd, AS3676_AUDIO_INPUT, rd->audio.audio_input);
		reg_set(rd, AS3676_AUDIO_OUTPUT, rd->audio.audio_output);
		reg_set(rd, AS3676_ADC_CTRL, AS3676_ALS_SOURCE_AUDIO);
	} else {
		as3676_als_set_enable_internal(rd, rd->als_enabled);
		reg_set(rd, AS3676_SINK_3X_AUD_SRC, 0x00);
		reg_set(rd, AS3676_AUDIO_CTRL_1, 0x00);
		reg_set(rd, AS3676_AUDIO_INPUT, 0x00);
		reg_set(rd, AS3676_AUDIO_OUTPUT, 0x00);
		if (rd->als_enabled)
			reg_set(rd, AS3676_ADC_CTRL, rd->als.source);
		else
			as3676_als_set_adc_ctrl(rd);
	}
}

static void as3676_set_brightness(struct as3676_record *rd,
		enum as3676_register reg, enum led_brightness value, int flags)
{
	enum as3676_register ctrl_reg;
	int off_bits;
	u8 ctrl_val;
	u8 ctrl_cur;

	if (value > LED_FULL)
		value = LED_FULL;
	if (value < LED_OFF)
		value = LED_OFF;

	if (rd->dls_connected && flags & AS3676_FLAG_DLS) {
		ctrl_reg = as3676_sink[reg].dls;
		off_bits = as3676_sink[reg].dls_bit;
		ctrl_val = reg_get(rd, ctrl_reg);
		ctrl_val |= (1 << off_bits);
		reg_set(rd, ctrl_reg, ctrl_val);
	}

	ctrl_reg = as3676_sink[reg].ctrl;
	off_bits = as3676_sink[reg].lower_bit;

	ctrl_val = reg_get(rd, ctrl_reg);
	ctrl_cur = (ctrl_val >> off_bits) & AS3676_CTRL_MASK;
	ctrl_val &= ~(AS3676_CTRL_MASK << off_bits);
	if (value == LED_OFF)
		ctrl_val |= (AS3676_CTRL_OFF << off_bits);
	else if ((flags & AS3676_FLAG_PWM_INIT) ||
			(flags & AS3676_FLAG_PWM_CTRL))
		ctrl_val |= (AS3676_CTRL_PWM << off_bits);
	else if (as3676_sink[reg].flags & AS3676_FLAG_EXT_CURR)
		ctrl_val |= (AS3676_CTRL_EXT_CURR << off_bits);
	else if (ctrl_cur == AS3676_CTRL_PATTERN) /* don't skip pattern */
		ctrl_val |= (AS3676_CTRL_PATTERN << off_bits);
	else
		ctrl_val |= (AS3676_CTRL_ON << off_bits);
	reg_set(rd, ctrl_reg, ctrl_val);

	if ((as3676_sink[reg].flags & AS3676_FLAG_EXT_CURR) &&
		(flags & AS3676_FLAG_AUDIO)) {
		if (value == LED_OFF)
			as3676_audio_set_config(rd, 0);
		else
			as3676_audio_set_config(rd, 1);
	}

	if (as3676_sink[reg].flags & AS3676_FLAG_DCDC_CTRL) {
		u8 ctrl_val = reg_get(rd, AS3676_REG_CTRL);
		u64 dcdcbit = rd->dcdcbit;
		if (value == LED_OFF)
			dcdcbit &= ~((u64)1 << reg);
		else
			dcdcbit |= ((u64)1 << reg);
		if (dcdcbit != rd->dcdcbit) {
			if (!dcdcbit) { /* Disable Step-up */
				reg_set(rd, AS3676_REG_CTRL, ctrl_val & ~0x08);
			} else if (!rd->dcdcbit) { /* Enable Step-up */
				reg_set(rd, AS3676_REG_CTRL, ctrl_val | 0x08);
				usleep(AS3676_REG_CTRL_WAIT_US);
			}
			rd->dcdcbit = dcdcbit;
		}
	}

	reg_set(rd, reg, value);
}

static void as3676_set_interface_brightness(struct as3676_interface *intf,
		enum led_brightness value)
{
	int i;
	enum as3676_cmode cmode;
	struct as3676_record *rd;
	struct as3676_platform_data *pdata;
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);
	pdata = rd->client->dev.platform_data;

	cmode = rd->cmode;
	rd->cmode = AS3676_CMODE_IMMEDIATE;

	intf->cdev.brightness = value;

	if (rd->audio_enabled || (rd->als_suspend == AS3676_NO_SUSPEND) ||
		!(pdata->leds[intf->index].flags & AS3676_FLAG_WAIT_RESUME)) {
		if (intf->max_current)
			value = (value * intf->max_current)
							/ AS3676_MAX_CURRENT;

		for (i = 0; i < ARRAY_SIZE(as3676_sink); ++i) {
			if (intf->regs & ((u64)1 << i))
				as3676_set_brightness(rd, i,
							value, intf->flags);
		}
	}
	rd->cmode = cmode;
}

static void as3676_brightness(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;

	intf = container_of(led_cdev, struct as3676_interface, cdev);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);
	dev_dbg(led_cdev->dev, "brightness i=%d, on=%d\n", intf->index, value);
	as3676_lock(rd);
	as3676_set_interface_brightness(intf, value);
	as3676_unlock(rd);
}


#ifdef CONFIG_LEDS_AS3676_HW_BLINK
static void as3676_set_blink(struct as3676_record *rd,
		enum as3676_register reg, unsigned long *on, unsigned long *off,
		int value)
{
	enum as3676_register ctrl_reg;
	int off_bits;
	u8 ctrl_val;
	int i;
	unsigned long total_time = *on + *off;

	/*
	 * Blinking period for as3676 consists of:
	 *  [delay value(led off)] [32bit pattern (0-led off, 1-led on)]
	 *   delay value is in range 0..7 seconds (x8 for slow clock)
	 *   each bit in pattern has duration 31ms (250ms for slow clock)
	 *
	 *  as3676_patterns table contains most suitable predefined settings for
	 *  all periods which can be configured
	 */
	static struct as3676_pattern {
		int period;
		int slow;
		int delay;
		int bits_per_cycle;
	} as3676_patterns[] = {
		{ 62,	0,	0,	2},
		{ 125,	0,	0,	4},
		{ 250,	0,	0,	8},
		{ 500,	0,	0,	16},
		{ 1000,	0,	0,	32},
		{ 2000,	1,	0,	8},
		{ 3000,	0,	2,	32},
		{ 4000,	1,	0,	16},
		{ 5000,	0,	4,	32},
		{ 6000,	0,	5,	32},
		{ 7000,	0,	6,	32},
		{ 8000,	1,	0,	32},
		{ 16000, 1,	1,	32},
		{ 24000, 1,	2,	32},
		{ 32000, 1,	3,	32},
		{ 40000, 1,	4,	32},
		{ 48000, 1,	5,	32},
		{ 56000, 1,	6,	32},
		{ 64000, 1,	7,	32},
	};

	if (as3676_sink[reg].flags & AS3676_FLAG_DCDC_CTRL) {
		dev_err(&rd->client->dev,
				"Request for blinking on DCDC ctrl led\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(as3676_patterns); ++i) {
		struct as3676_pattern *patt_def = &as3676_patterns[i];
		u32 pattern = 0;
		u8 curr_val, on_bits, bit, bit_duration_ms;

		/* find most suitable pattern for this period */
		if (patt_def->period < total_time)
			if (i != ARRAY_SIZE(as3676_patterns) - 1)
				continue;

		if (i > 0 && abs(total_time - as3676_patterns[i - 1].period) <
				abs(total_time-patt_def->period)) {
			/* smaller period more suitable */
			patt_def = &as3676_patterns[i - 1];
		}

		bit_duration_ms = (patt_def->slow ?
				AS3676_SLOW_PATTERN_BIT_DURATION_MS :
				AS3676_FAST_PATTERN_BIT_DURATION_MS);

		/* calculate number of bits for led_on state */
		on_bits = *on / bit_duration_ms;

		/* if on_time too small adjust to minimal value */
		if (on_bits == 0)
			on_bits = 1;

		/* return adjusted values */
		*on = on_bits * bit_duration_ms;
		*off = patt_def->period - *on;

		/* generate pattern */
		for (bit = 0; bit < 32; bit++) {
			if (bit % patt_def->bits_per_cycle < on_bits)
				pattern |= (1 << bit);
		}

		reg_set(rd, AS3676_REG_PATTERN_DATA_0, pattern & 0xff);
		reg_set(rd, AS3676_REG_PATTERN_DATA_1, (pattern >>  8) & 0xff);
		reg_set(rd, AS3676_REG_PATTERN_DATA_2, (pattern >> 16) & 0xff);
		reg_set(rd, AS3676_REG_PATTERN_DATA_3, (pattern >> 24) & 0xff);
		reg_set(rd, AS3676_REG_PATTERN_CTRL,
				(patt_def->delay & 0x3) << 1);

		curr_val = reg_get(rd, AS3676_REG_GPIO_CURR);
		curr_val &= (1 << 7);
		reg_set(rd, AS3676_REG_GPIO_CURR, curr_val |
				((patt_def->delay & 0x4) << 2) |
				(patt_def->slow << 6));
		break;
	}

	reg_set(rd, reg, value);

	ctrl_reg = as3676_sink[reg].ctrl;
	off_bits = as3676_sink[reg].lower_bit;

	ctrl_val = reg_get(rd, ctrl_reg) & ~(AS3676_CTRL_MASK << off_bits);
	reg_set(rd, ctrl_reg, ctrl_val | (AS3676_CTRL_PATTERN << off_bits));
}

static void as3676_set_interface_blink(struct as3676_interface *intf,
		unsigned long *on, unsigned long *off)
{
	struct as3676_record *rd;
	int value;
	int i;

	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	value = intf->cdev.brightness;

	/* Since blinking at 0 brightness is stupid. */
	if (value == LED_OFF)
		value = LED_FULL;

	if (intf->max_current)
		value = (value * intf->max_current) / AS3676_MAX_CURRENT;

	if (*on == 0 && *off == 0)
		*on = 500, *off = 500;

	for (i = 0; i < ARRAY_SIZE(as3676_sink); ++i) {
		if (intf->regs & ((u64)1 << i))
			as3676_set_blink(rd, i, on, off, value);
	}

	schedule_work(&rd->work);
}

static int as3676_blink(struct led_classdev *led_cdev,
		unsigned long *on, unsigned long *off)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;

	intf = container_of(led_cdev, struct as3676_interface, cdev);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);
	dev_dbg(led_cdev->dev, "blink i=%d, on=%ld, off=%ld\n",
			intf->index, *on, *off);
	as3676_lock(rd);
	as3676_set_interface_blink(intf, on, off);
	as3676_unlock(rd);
	return 0;
}
#endif

static ssize_t as3676_als_value_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;
	ssize_t ret;
	u8 val;

	intf = container_of(kobj, struct as3676_interface, kobj);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	as3676_lock(rd);
	ret = i2c_smbus_read_i2c_block_data(rd->client, 0x93, 1, &val);
	if (ret < 0) {
		as3676_unlock(rd);
		return ret;
	}
	as3676_unlock(rd);

	sprintf(buf, "%u\n", val);
	ret = strlen(buf) + 1;

	return ret;
}

static void as3676_als_set_adc_ctrl(struct as3676_record *rd)
{
	u8 val;
	int status;
	/* Sometimes AS3676 has increased 200uA current consumption in standby
	 * and need to handle it by sw. */
	reg_set(rd, AS3676_ADC_CTRL, 0x80);
	status = i2c_smbus_read_i2c_block_data(rd->client, AS3676_ADC_CTRL,
					       1, &val);
	if (status < 0)
		dev_err(&rd->client->dev, "%s:I2C read error:%d\n",
			 __func__, status);
}

static void as3676_als_set_enable(struct as3676_record *rd,
		struct as3676_interface *intf, u8 enable)
{
	as3676_als_set_enable_internal(rd, enable);

	rd->als_enabled = enable;

	/* We have to make sure to turn on/off the ALS curve here,
	 * or the brightness will still be limited by the last ALS
	 * reading (if you go from on to off).
	 */

	if (enable) {
		enum as3676_amb_value group;

		switch (intf->flags & AS3676_FLAG_ALS_MASK) {
		case AS3676_FLAG_ALS_GROUP1:
			group = AS3676_AMB_GROUP_1;
			break;
		case AS3676_FLAG_ALS_GROUP2:
			group = AS3676_AMB_GROUP_2;
			break;
		case AS3676_FLAG_ALS_GROUP3:
			group = AS3676_AMB_GROUP_3;
			break;
		default:
			group = AS3676_AMB_OFF;
			break;
		}
		as3676_set_interface_amb(rd, intf, group);
	} else {
		/* Turn off the curve */
		as3676_set_interface_amb(rd, intf, AS3676_AMB_OFF);
	}

	/* Make it happen! */
	schedule_work(&rd->work);
}

static void as3676_als_set_enable_internal(struct as3676_record *rd, u8 enable)
{
	if (enable && !rd->audio_enabled)
		reg_set(rd, AS3676_AMB_CTRL, (rd->als.gain << 1) | 0x01);
	else
		reg_set(rd, AS3676_AMB_CTRL, 0x00);

	if (!rd->audio_enabled && rd->als_suspend == AS3676_SUSPENDED) {
		/*
		 *  as3676 keeps old ALS value during sleep in amb_result
		 *  register and it affects the brightness soon after wakeup.
		 *  Need 2ms wait to update value.
		 */
		schedule_delayed_work(&rd->als_enabled_work,
				msecs_to_jiffies(AS3676_ALS_ENABLE_WAIT_MS));
	}
}

static ssize_t as3676_als_enable_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;
	unsigned long enable;
	int ret;
	u8 val;

	intf = container_of(kobj, struct as3676_interface, kobj);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	ret = strict_strtoul(buf, 0, &enable);
	if (ret < 0 || (enable != 1 && enable != 0))
		return -EINVAL;

	as3676_lock(rd);
	val = reg_get(rd, AS3676_AMB_CTRL) & 0x01;

	if (enable != val) {
		as3676_als_set_enable(rd, intf, enable);

		if (enable) {
			as3676_als_set_params(rd, &rd->als);
			if (!rd->audio_enabled)
				reg_set(rd, AS3676_ADC_CTRL, rd->als.source);
		} else {
			struct as3676_als_config param;

			memset(&param, 0x00, sizeof(struct as3676_als_config));
			as3676_als_set_params(rd, &param);
			as3676_als_set_adc_ctrl(rd);
		}
	}

	as3676_unlock(rd);
	return strlen(buf);
}


static ssize_t as3676_als_enable_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;
	u8 val;

	intf = container_of(kobj, struct as3676_interface, kobj);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	as3676_lock(rd);
	val = reg_get(rd, AS3676_AMB_CTRL);
	as3676_unlock(rd);

	return sprintf(buf, "%u\n", (val & 0x01) ? 1 : 0);
}

static ssize_t as3676_als_curve_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;
	int ret;
	struct as3676_als_curve curve;
	enum as3676_amb_value als_group;

	intf = container_of(kobj, struct as3676_interface, kobj);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	ret = sscanf(buf, "%u,%u,%u,%u,%u,%u,%u", &als_group,
			&curve.y0, &curve.y3, &curve.k1,
			&curve.k2, &curve.x1, &curve.x2);
	if (ret != 7)
		return -EINVAL;

	if (als_group & ~AS3676_AMB_MASK)
		return -EINVAL;
	/* if any of the values are > 255 or < 0, error out */
	if ((curve.y0 | curve.y3) & ~0xff)
		return -EINVAL;
	if ((curve.k1 | curve.k2) & ~0xff)
		return -EINVAL;
	if ((curve.x1 | curve.x2) & ~0xff)
		return -EINVAL;

	as3676_lock(rd);
	intf->flags &= ~AS3676_FLAG_ALS_MASK;
	switch (als_group) {
	case AS3676_AMB_GROUP_1:
		intf->flags |= AS3676_FLAG_ALS_GROUP1;
		break;
	case AS3676_AMB_GROUP_2:
		intf->flags |= AS3676_FLAG_ALS_GROUP2;
		break;
	case AS3676_AMB_GROUP_3:
		intf->flags |= AS3676_FLAG_ALS_GROUP3;
		break;
	default:
		break;
	}

	memcpy(&rd->als.curve[als_group], &curve,
				sizeof(struct as3676_als_curve));

	as3676_set_als_config(rd, &rd->als, intf);
	as3676_unlock(rd);

	return count;
}

static ssize_t as3676_als_curve_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;
	struct as3676_als_curve curve;
	char *curve_str = buf;
	int length = 0;
	int i;

	intf = container_of(kobj, struct as3676_interface, kobj);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	as3676_lock(rd);
	length = sprintf(curve_str, "curve,y0,y3,k1,k2,x1,x2\n");
	curve_str += length;

	for (i = 1; i < AS3676_AMB_MAX; i++) {
		memset(&curve, 0x00, sizeof(struct as3676_als_curve));
		as3676_get_als_config(rd, &curve, i);

		length = snprintf(curve_str, PAGE_SIZE - (curve_str - buf),
			"group_%u,%u,%u,%u,%u,%u,%u\n",
			i, curve.y0, curve.y3, curve.k1,
			curve.k2, curve.x1, curve.x2);
		curve_str += length;
	}
	as3676_unlock(rd);

	return strlen(buf);
}

static void as3676_als_set_params(struct as3676_record *rd,
				struct as3676_als_config *param)
{
	u8 ctrl = reg_get(rd, AS3676_AMB_CTRL);

	/* amb_gain 0x90 */
	ctrl &= ~0x06;
	reg_set(rd, AS3676_AMB_CTRL, ((param->gain << 1) | ctrl));
	/* als_filter 0x91 */
	reg_set(rd, AS3676_AMB_FILTER,
		(param->filter_up | (param->filter_down << 4)));
	/* als offs 0x92 */
	reg_set(rd, AS3676_AMB_OFFSET, param->offset);

	schedule_work(&rd->work);
}

static ssize_t as3676_als_params_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;
	struct as3676_als_config param;
	int count;

	intf = container_of(kobj, struct as3676_interface, kobj);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	count = sscanf(buf, "%u,%u,%u,%u", &param.gain, &param.filter_up,
					&param.filter_down, &param.offset);
	if (count != 4)
		return -EINVAL;

	if ((param.gain >= AS3676_GAIN_MAX) ||
		(param.filter_up >= AS3676_FILTER_MAX) ||
		(param.filter_down >= AS3676_FILTER_MAX) ||
		(param.offset >= 0xFF))
		return -EINVAL;

	as3676_lock(rd);
	rd->als.gain = param.gain;
	rd->als.filter_up = param.filter_up;
	rd->als.filter_down = param.filter_down;
	rd->als.offset = param.offset;

	as3676_als_set_params(rd, &rd->als);
	as3676_unlock(rd);

	return strlen(buf);
}

static ssize_t as3676_als_params_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct as3676_interface *intf;
	struct as3676_record *rd;

	intf = container_of(kobj, struct as3676_interface, kobj);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	return sprintf(buf, "gain,filter_up,filter_down,offset\n"
		"%u,%u,%u,%u\n", rd->als.gain, rd->als.filter_up,
		rd->als.filter_down, rd->als.offset);
}

static struct kobj_attribute as3676_als_attr_value =
	__ATTR(value, 0444, as3676_als_value_show, NULL);
static struct kobj_attribute as3676_als_attr_enable =
	__ATTR(enable, 0644, as3676_als_enable_show, as3676_als_enable_store);
static struct kobj_attribute as3676_als_attr_curve =
	__ATTR(curve, 0644, as3676_als_curve_show, as3676_als_curve_store);
static struct kobj_attribute as3676_als_attr_params =
	__ATTR(params, 0644, as3676_als_params_show, as3676_als_params_store);

static ssize_t as3676_max_current_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct as3676_record *rd;
	struct as3676_interface *intf;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	intf = container_of(led_cdev, struct as3676_interface, cdev);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	return snprintf(buf, PAGE_SIZE, "%d\n", intf->max_current);
}

static ssize_t as3676_max_current_store(struct device *dev,
                 struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	unsigned long curr_val;
	struct as3676_record *rd;
	struct as3676_interface *intf;
        struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct as3676_platform_data *pdata;

	intf = container_of(led_cdev, struct as3676_interface, cdev);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	ret = strict_strtoul(buf, 10, &curr_val);

	if (ret != 0 || curr_val == 0)
		return -EINVAL;

	as3676_lock(rd);
	pdata = rd->client->dev.platform_data;

	if (curr_val > pdata->leds[intf->index].max_current)
		curr_val = pdata->leds[intf->index].max_current;

	intf->max_current = (int)curr_val;
	as3676_unlock(rd);

	return size;
}

static void as3676_als_set_mode(struct as3676_record *rd,
			enum as3676_register reg, u8 mode)
{
	enum as3676_register ctrl_reg;
	int off_bits;
	u8 ctrl_val;

	ctrl_reg = as3676_sink[reg].ctrl;
	off_bits = as3676_sink[reg].lower_bit;
	ctrl_val = reg_get(rd, ctrl_reg) & ~(AS3676_CTRL_MASK << off_bits);

	reg_set(rd, ctrl_reg, ctrl_val | (mode << off_bits));
}

static ssize_t as3676_mode_store(struct device *dev,
                 struct device_attribute *attr, const char *buf, size_t size)
{
	int ret, i;
	unsigned long mode;
	struct as3676_record *rd;
	struct as3676_interface *intf;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	intf = container_of(led_cdev, struct as3676_interface, cdev);
	rd = container_of(intf, struct as3676_record, interfaces[intf->index]);

	ret = strict_strtoul(buf, 10, &mode);

	if ((ret != 0) || (mode > AS3676_CTRL_PWM))
		return -EINVAL;

	as3676_lock(rd);
	intf->flags &= ~AS3676_FLAG_PWM_CTRL;
	if (mode == AS3676_CTRL_PWM)
		intf->flags |= AS3676_FLAG_PWM_CTRL;

	for (i = 0; i < ARRAY_SIZE(as3676_sink); ++i) {
		if (intf->regs & ((u64)1 << i))
			as3676_als_set_mode(rd, i, (u8)mode);
	}
	as3676_unlock(rd);

	return size;
}

static DEVICE_ATTR(max_current, 0600, as3676_max_current_show, as3676_max_current_store);
static DEVICE_ATTR(mode, 0200, NULL, as3676_mode_store);

static void dummy_kobj_release(struct kobject *kobj)
{ }

static struct kobj_type dummy_kobj_ktype = {
	.release	= dummy_kobj_release,
	.sysfs_ops	= &kobj_sysfs_ops,
};

static struct attribute *as3676_als_attrs[] = {
	&as3676_als_attr_value.attr,
	&as3676_als_attr_enable.attr,
	&as3676_als_attr_curve.attr,
	&as3676_als_attr_params.attr,
	NULL
};

static struct attribute_group as3676_als_attr_group = {
	.attrs = as3676_als_attrs,
};

static int as3676_create_als_tree(struct as3676_record *rd,
		struct as3676_interface *intf)
{
	int rc;

	rc = kobject_init_and_add(&intf->kobj, &dummy_kobj_ktype,
			&intf->cdev.dev->kobj, "als");
	if (rc)
		return rc;

	rc = sysfs_create_group(&intf->kobj, &as3676_als_attr_group);
	if (rc)
		kobject_put(&intf->kobj);

	return rc;
}

static void as3676_restore(struct as3676_record *rd)
{
	unsigned i;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(as3676_restore_regs); i++) {
		reg = as3676_restore_regs[i];
		reg_set(rd, reg, rd->registers[reg]);
	}
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int as3676_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3676_record *rd = i2c_get_clientdata(client);

	dev_info(dev, "Suspending AS3676\n");

	as3676_lock(rd);
	rd->cmode = AS3676_CMODE_IMMEDIATE;
	reg_set(rd, AS3676_REG_CTRL, 0x00);

	if (rd->als_connected) {
		as3676_als_set_enable_internal(rd, 0);
		if (!rd->audio_enabled)
			as3676_als_set_adc_ctrl(rd);
		rd->als_suspend = AS3676_SUSPENDED;
	}

	as3676_unlock(rd);

	return 0;
}

static int as3676_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3676_record *rd = i2c_get_clientdata(client);

	dev_info(dev, "Resuming AS3676\n");

	as3676_lock(rd);
	as3676_restore(rd);
	reg_set(rd, AS3676_REG_CTRL, 0x0d);

	if (rd->als_connected) {
		if (rd->als_wait) {
			schedule_delayed_work(&rd->als_resume_work,
					msecs_to_jiffies(rd->als_wait));
		} else {
			as3676_als_set_enable_internal(rd, rd->als_enabled);
			if (!rd->audio_enabled)
				reg_set(rd, AS3676_ADC_CTRL, rd->als.source);
		}
	}
	rd->cmode = AS3676_CMODE_SCHEDULED;

	as3676_unlock(rd);

	return 0;
}
#else
#define as3676_pm_suspend	NULL
#define as3676_pm_resume	NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void as3676_early_suspend(struct early_suspend *handler)
{
	struct as3676_record *rd =
		container_of(handler, struct as3676_record, early_suspend);

	dev_info(&rd->client->dev, "%s\n", __func__);

	as3676_lock(rd);
	rd->cmode = AS3676_CMODE_IMMEDIATE;
	reg_set(rd, AS3676_REG_CTRL, 0x00);

	if (rd->als_connected) {
		as3676_als_set_enable_internal(rd, 0);
		if (!rd->audio_enabled)
			as3676_als_set_adc_ctrl(rd);
		rd->als_suspend = AS3676_SUSPENDED;
	}

	as3676_unlock(rd);
}

static void as3676_late_resume(struct early_suspend *handler)
{
	struct as3676_record *rd =
		container_of(handler, struct as3676_record, early_suspend);

	dev_info(&rd->client->dev, "%s\n", __func__);

	as3676_lock(rd);
	as3676_restore(rd);
	reg_set(rd, AS3676_REG_CTRL, 0x0d);

	if (rd->als_connected) {
		if (rd->als_wait) {
			schedule_delayed_work(&rd->als_resume_work,
					msecs_to_jiffies(rd->als_wait));
		} else {
			as3676_als_set_enable_internal(rd, rd->als_enabled);
			if (!rd->audio_enabled)
				reg_set(rd, AS3676_ADC_CTRL, rd->als.source);
		}
	}

	rd->cmode = AS3676_CMODE_SCHEDULED;

	as3676_unlock(rd);
}
#endif

static void as3676_shutdown(struct i2c_client *client)
{
	struct as3676_record *rd = i2c_get_clientdata(client);
	int i;

	dev_info(&client->dev, "Shutting down AS3676\n");

	as3676_lock(rd);
	rd->cmode = AS3676_CMODE_IMMEDIATE;

	for (i = 0; i < rd->n_interfaces; ++i)
		as3676_set_interface_brightness(&rd->interfaces[i], 0);

	reg_set(rd, AS3676_REG_CTRL, 0x00);
	reg_set(rd, AS3676_MODE_SWITCH, 0x00);
	reg_set(rd, AS3676_MODE_SWITCH_2, 0x00);
	reg_set(rd, AS3676_OVERTEMP_CTRL, 0x10);
	if (rd->als_connected)
		reg_set(rd, AS3676_AMB_CTRL, 0x0);

	as3676_unlock(rd);
}

static int __devexit as3676_remove(struct i2c_client *client)
{
	struct as3676_record *rd = i2c_get_clientdata(client);
	int i;

	dev_info(&client->dev, "Removing AS3676 driver\n");

	for (i = 0; i < rd->n_interfaces; ++i) {
		struct as3676_interface *intf = &rd->interfaces[i];
		device_remove_file(intf->cdev.dev, &dev_attr_max_current);
		device_remove_file(intf->cdev.dev, &dev_attr_mode);

		kobject_put(&intf->kobj);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rd->early_suspend);
#endif

	cancel_delayed_work_sync(&rd->als_resume_work);
	cancel_delayed_work_sync(&rd->als_enabled_work);
	rd->cmode = AS3676_CMODE_IMMEDIATE;
	reg_set(rd, AS3676_REG_CTRL, 0x00);

	device_init_wakeup(&client->dev, 0);

	kfree(rd);

	return 0;
}

static int __devinit as3676_probe(struct i2c_client *client,
		const struct i2c_device_id *id);

static const struct i2c_device_id as3676_idtable[] = {
	{AS3676_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, as3676_idtable);

static const struct dev_pm_ops as3676_pm = {
	.suspend = as3676_pm_suspend,
	.resume = as3676_pm_resume,
};

static struct i2c_driver as3676_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AS3676_NAME,
		.pm = &as3676_pm,
	},
	.probe   = as3676_probe,
	.remove  = __devexit_p(as3676_remove),
	.shutdown = as3676_shutdown,
	.id_table = as3676_idtable,
};

static int __devinit as3676_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct as3676_record *rd = 0;
	struct as3676_platform_data *pdata;
	int err;
	int i, j;
	int ldo_val;

	pdata = client->dev.platform_data;

	if (!pdata || pdata->num_leds == 0) {
		dev_err(&client->dev, "no/bad pdata provided\n");
		return -EFAULT;
	}

	if (pdata->num_leds > AS3676_INTERFACE_MAX) {
		dev_err(&client->dev, "pdata specifies too many leds\n");
		return -EFAULT;
	}

	rd = kzalloc(sizeof(struct as3676_record), GFP_KERNEL);
	if (!rd)
		return -ENOMEM;

	rd->als_connected = pdata->als_connected;
	rd->als_wait = pdata->als_wait;
	rd->dls_connected = pdata->dls_connected;

	client->driver = &as3676_driver;

	mutex_init(&rd->lock);
	INIT_WORK(&rd->work, as3676_worker);
	INIT_DELAYED_WORK(&rd->als_resume_work, as3676_als_resume_worker);
	INIT_DELAYED_WORK(&rd->als_enabled_work, as3676_als_enabled_worker);

	/* We will need the i2c device later */
	rd->client = client;

	rd->cmode = AS3676_CMODE_IMMEDIATE;

	i2c_set_clientdata(client, rd);
	dev_set_drvdata(&client->dev, rd);

	rd->n_interfaces = pdata->num_leds;

	for (i = 0; i < rd->n_interfaces; ++i) {
		struct as3676_platform_led *led = &pdata->leds[i];
		struct as3676_interface *intf = &rd->interfaces[i];
		intf->cdev.name           = led->name;
		intf->cdev.brightness     = led->default_brightness;
		intf->cdev.brightness_set = as3676_brightness;
#ifdef CONFIG_LEDS_AS3676_HW_BLINK
		if (led->flags & AS3676_FLAG_BLINK)
			intf->cdev.blink_set = as3676_blink;
#endif
		intf->flags = led->flags;
		intf->max_current = led->max_current;
		intf->index = i;

		for (j = 0; j < AS3676_SINK_MAX; ++j) {
			if (led->sinks & BIT(j)) {
				intf->regs |= ((u64)1 << as3676_sink_map[j]);
				if (as3676_sink[as3676_sink_map[j]].flags &
						AS3676_FLAG_DCDC_CTRL) {
					reg_set(rd, AS3676_DCDC_CTRL_1, 0xc2);
					reg_set(rd, AS3676_DCDC_CTRL_2, 0x8C);
				}
			}
		}
		err = led_classdev_register(&client->dev, &intf->cdev);
		if (err < 0) {
			dev_info(&client->dev, "Failed to add %s\n",
					intf->cdev.name);
		}
		if (rd->als_connected && (intf->flags & AS3676_FLAG_ALS_MASK))
			as3676_create_als_tree(rd, intf);

		err = device_create_file(intf->cdev.dev, &dev_attr_max_current);
		if (err)
			dev_err(&client->dev,
				"create dev_attr_max_current failed\n");
		err = device_create_file(intf->cdev.dev, &dev_attr_mode);
		if (err)
			dev_err(&client->dev,
				"create dev_attr_mode failed\n");
	}

	/* Enable charge pump and connect all leds to it */
	/* TODO: double check that these are appropriate according to pdata */
	reg_set(rd, AS3676_REG_CTRL, 0x05);

	if (pdata->ldo_mV <= 0) {
		ldo_val = 0x1f;
	} else if ((pdata->ldo_mV < AS3676_LDO_MIN) ||
			(pdata->ldo_mV > AS3676_LDO_MAX)) {
		dev_err(&client->dev, "ldo_mV in pdata is out-of-range\n");
		err = -EINVAL;
		goto error;
	} else {
		ldo_val = (pdata->ldo_mV - AS3676_LDO_MIN) / 50;
	}

	reg_set(rd, AS3676_LDO_VOLTAGE, ldo_val);

	reg_set(rd, AS3676_MODE_SWITCH, 0x70);
	/* Allow dimming up */
	reg_set(rd, AS3676_REG_PWM_CODE, 0);
	reg_set(rd, AS3676_REG_PWM_CTRL, 4<<3 | 1<<1);

	if (pdata->als_config) {
		memcpy(&rd->als, pdata->als_config,
				sizeof(struct as3676_als_config));
	} else {
		memcpy(&rd->als, &as3676_default_config,
				sizeof(struct as3676_als_config));
	}

	if (pdata->audio_config) {
		memcpy(&rd->audio, pdata->audio_config,
				sizeof(struct as3676_audio_config));
	} else {
		memset(&rd->audio, 0,
				sizeof(struct as3676_audio_config));
	}

	if (rd->als_connected) {
		as3676_als_set_params(rd, &rd->als);
		/* By default, ALS should be enabled */
		rd->als_enabled = 1;
		as3676_als_set_enable_internal(rd, rd->als_enabled);
		rd->als_suspend = AS3676_NO_SUSPEND;
	}

	if (rd->dls_connected)
		reg_set(rd, AS3676_REG_GPIO_CURR, 1 << 7);

	for (i = 0; i < rd->n_interfaces; ++i) {
		struct as3676_interface *intf = &rd->interfaces[i];
		as3676_set_interface_brightness(intf, intf->cdev.brightness);
		intf->flags &= ~AS3676_FLAG_PWM_INIT;
		if ((rd->als_connected) && (intf->flags & AS3676_FLAG_ALS_MASK))
			as3676_set_als_config(rd, &rd->als, intf);
	}

	rd->cmode = AS3676_CMODE_SCHEDULED;

#ifdef CONFIG_HAS_EARLYSUSPEND
	rd->early_suspend.suspend = as3676_early_suspend;
	rd->early_suspend.resume = as3676_late_resume;
	register_early_suspend(&rd->early_suspend);
#endif
	return 0;

error:
	kfree(rd);
	return err;
}

static int __init as3676_init(void)
{
	return i2c_add_driver(&as3676_driver);
}

static void __exit as3676_exit(void)
{
	i2c_del_driver(&as3676_driver);
}

module_init(as3676_init);
module_exit(as3676_exit);

MODULE_AUTHOR("Courtney Cavin <courtney.cavin@sonyericsson.com>");
MODULE_DESCRIPTION("AS3676 I2C LED driver");
MODULE_LICENSE("GPL v2");
