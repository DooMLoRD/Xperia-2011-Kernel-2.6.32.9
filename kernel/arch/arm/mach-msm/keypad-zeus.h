/*
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
 */

#ifndef _KEYPAD_ZEUS_H
#define _KEYPAD_ZEUS_H

#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/platform_device.h>
#include <linux/mfd/pmic8058.h>

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)	(sys_gpio - NR_GPIO_IRQS)


static struct gpio_event_direct_entry keypad_zeus_gpio_map[] = {
	{ 19, BTN_TL},
	{ 88, BTN_TR},
	{ 95, BTN_Y},
	{ 96, BTN_B},
	{ 97, BTN_A},
	{ 98, BTN_X},
	{108, KEY_MEDIA},
/*	{ 99, KEY_UP},
	{100, KEY_DOWN},
	{101, KEY_LEFT},
	{102, KEY_RIGHT}, */
	/* Rotate the D-Pad 90 degrees */
	{ 99, KEY_RIGHT},
	{101, KEY_UP},
	{100, KEY_LEFT},
	{102, KEY_DOWN}
};

static struct gpio_event_direct_entry keypad_zeus_pmic_gpio_map_nwake[] = {
	{ PM8058_GPIO_PM_TO_SYS(17 - 1), KEY_BACK },
	{ PM8058_GPIO_PM_TO_SYS(18 - 1), KEY_MENU },
	{ PM8058_GPIO_PM_TO_SYS(26 - 1), KEY_SEARCH },
};

static struct gpio_event_direct_entry keypad_zeus_pmic_gpio_map_wake[] = {
	{ PM8058_GPIO_PM_TO_SYS(19 - 1), KEY_HOME },
	{ PM8058_GPIO_PM_TO_SYS(31 - 1), KEY_VENDOR },
};

static const struct gpio_event_direct_entry switch_zeus_gpio_map[] = {
	{ 180, SW_LID},
};

static const struct gpio_event_direct_entry lidswitch_zeus_gpio_map[] = {
	{ 79, MSC_RAW},
};

#if defined(CONFIG_PHF_TESTER)

static const struct gpio_event_direct_entry phf_gpio_map[] = {
	{ 26, BTN_THUMB},	// The mic
	{ 94, BTN_PINKIE},	// The headset
};

#endif

#endif
