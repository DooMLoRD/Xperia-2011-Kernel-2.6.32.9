/* /kernel/arch/arm/mach-msm/board-semc_mogami-keypad.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _BOARD_SEMC_MOGAMI_KEYPAD_H
#define _BOARD_SEMC_MOGAMI_KEYPAD_H

#define PM8058_KEYPAD_DEV	"pm8058-keypad"
#define PM8058_KEYPAD_PHYS	"sys/bus/i2c/devices/6-0000"

struct mfd_cell *get_pm8058_keypad_dev(void);

#endif
