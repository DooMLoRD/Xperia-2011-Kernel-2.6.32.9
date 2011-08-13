/* /kernel/arch/arm/mach-msm/charger-anzu.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/battery_chargalg.h>

static struct ambient_temperature_limit limit_tbl = {
	{0,	42,	43,	127},	/* ambient temp: base */
	{4,	1,	37,	0},	/* ambient temp: hysteresis */
};

struct device_data device_data = {
	.limit_tbl = &limit_tbl,
	.battery_capacity_mah = 1500,
	.maximum_charging_current_ma = 1050,
};
