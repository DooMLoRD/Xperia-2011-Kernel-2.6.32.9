/* Copyright (c) 2011, Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/mfd/pmic8058.h>
#include <linux/input.h>
#include "keypad-pmic-mogami.h"

static struct pm8058_gpio pm_gpio_config = {
	.direction    = PM_GPIO_DIR_IN,
	.pull         = PM_GPIO_PULL_UP_31P5,
	.vin_sel      = PM_GPIO_VIN_S3,
	.function     = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol  = 0,
};

static struct keypad_pmic_mogami_key keymap_pmic[] = {
	{
		.code	= KEY_VENDOR,
		.irq	= PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, 26 - 1),
		.gpio	= 26,
		.wake	= 1,
	},
};

struct keypad_pmic_mogami_platform_data keypad_pmic_platform_data = {
	.keymap = keymap_pmic,
	.keymap_size = ARRAY_SIZE(keymap_pmic),
	.input_name = "keypad-pmic-azusa",
	.pm_gpio_config = &pm_gpio_config,
};
