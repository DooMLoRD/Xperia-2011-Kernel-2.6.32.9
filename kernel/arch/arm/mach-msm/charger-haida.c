/* /kernel/arch/arm/mach-msm/charger-haida.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/battery_chargalg.h>

struct device_data device_data = {
	.battery_capacity_mah = 1500,
	.maximum_charging_current_ma = 1050,
};
