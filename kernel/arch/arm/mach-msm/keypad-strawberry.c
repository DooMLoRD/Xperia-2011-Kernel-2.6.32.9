/* /kernel/arch/arm/mach-msm/keypad-strawberry.c
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
	KEY(0, 1, KEY_CAMERA_FOCUS),        /* camera AF */
	KEY(0, 2, KEY_DOWN),
	KEY(0, 3, KEY_SPACE),
	KEY(0, 4, KEY_A),
	KEY(0, 5, KEY_COMMA),
	KEY(0, 6, KEY_Z),

	KEY(1, 0, KEY_HOME),
	KEY(1, 1, KEY_CAMERA),
	KEY(1, 3, KEY_ENTER),
	KEY(1, 4, KEY_L),
	KEY(1, 5, KEY_S),
	KEY(1, 6, KEY_COMPOSE), /* SYMBOL */

	KEY(2, 1, KEY_VOLUMEDOWN),
	KEY(2, 3, KEY_B),
	KEY(2, 5, KEY_X),
	KEY(2, 7, KEY_LEFTALT), /* FUNCTION */

	KEY(3, 0, KEY_LANGUAGE),
	KEY(3, 1, KEY_VOLUMEUP),
	KEY(3, 2, KEY_K),
	KEY(3, 3, KEY_DOT),
	KEY(3, 4, KEY_UP),
	KEY(3, 5, KEY_O),
	KEY(3, 7, KEY_QUESTION),

	KEY(4, 1, KEY_BACKSPACE),
	KEY(4, 3, KEY_I),
	KEY(4, 4, KEY_M),
	KEY(4, 5, KEY_U),
	KEY(4, 6, KEY_RIGHT),
	KEY(4, 7, KEY_N),

	KEY(5, 1, KEY_J),
	KEY(5, 2, KEY_LEFT),
	KEY(5, 4, KEY_Y),
	KEY(5, 5, KEY_H),
	KEY(5, 6, KEY_LEFTSHIFT), /* SHIFT */
	KEY(5, 7, KEY_APOSTROPHE), /* QUATATION */

	KEY(6, 1, KEY_T),
	KEY(6, 2, KEY_V),
	KEY(6, 3, KEY_G),
	KEY(6, 5, KEY_R),
	KEY(6, 6, KEY_F),
	KEY(6, 7, KEY_C),

	KEY(7, 0, KEY_P),
	KEY(7, 1, KEY_SLASH),
	KEY(7, 2, KEY_E),
	KEY(7, 4, KEY_D),
	KEY(7, 6, KEY_Q),
	KEY(7, 7, KEY_W),
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
	.num_rows		= 8,
	.num_cols		= 8,
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

