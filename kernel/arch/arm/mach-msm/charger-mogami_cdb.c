/* /kernel/arch/arm/mach-msm/charger-mogami_cdb.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/battery_chargalg.h>

#ifdef CONFIG_BATTERY_CHARGALG_ENABLE_STEP_CHARGING
static struct step_charging step_charging[] = {
	/* Imax, Vmax, Hyst up, Hyst down */
	{{2, 1}, 4000, 25, 0},
	{{1, 1}, 4100, 25, 200},
	{{1, 2}, 4200, 0,  200},
};
#endif

struct device_data device_data = {
#ifdef CONFIG_BATTERY_CHARGALG_ENABLE_STEP_CHARGING
	.step_charging = &step_charging[0],
	.num_step_charging = ARRAY_SIZE(step_charging),
#endif
	.battery_capacity_mah = 1500,
	.maximum_charging_current_ma = 1050,
};
