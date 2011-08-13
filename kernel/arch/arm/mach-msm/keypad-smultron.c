/* /kernel/arch/arm/mach-msm/keypad-smultron.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>
#include "board-semc_mogami-keypad.h"

static const unsigned int keymap[] = {
	KEY(0, 1, KEY_HOME),
	KEY(0, 3, KEY_VOLUMEUP),
	KEY(0, 4, KEY_VOLUMEDOWN),
	KEY(0, 5, KEY_CAMERA_FOCUS),
	KEY(0, 6, KEY_CAMERA),
};

static struct resource resources_keypad[] = {
	{
		.start	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct matrix_keymap_data keymap_data = {
	.keymap_size	= ARRAY_SIZE(keymap),
	.keymap		= keymap,
};

static struct pmic8058_keypad_data keypad_data = {
	.input_name		= PM8058_KEYPAD_DEV,
	.input_phys_device	= PM8058_KEYPAD_PHYS,
	.num_rows		= 5,
	.num_cols		= 7,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 122000,
	.wakeup			= 1,
	.keymap_data		= &keymap_data,
};

static struct mfd_cell keypad_dev = {
	.name		= PM8058_KEYPAD_DEV,
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_keypad),
	.resources	= resources_keypad,
	.platform_data	= &keypad_data,
	.data_size	= sizeof(keypad_data),
};

struct mfd_cell *get_pm8058_keypad_dev(void)
{
	return &keypad_dev;
}
