/* kernel/include/linux/oneseg_tunerpm.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Akira Numata <Akira.Numata@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _ONESEG_TUNERPM_H_
#define _ONESEG_TUNERPM_H_

/* module name of tuner pm driver */
#define D_ONESEG_TUNERPM_DRIVER_NAME	"oneseg_tunerpm"

/* oneseg tuner : power on */
#define D_ONESEG_TUNER_POWER_ON       1

/* oneseg tuner : power off */
#define D_ONESEG_TUNER_POWER_OFF      0

struct oneseg_tunerpm_platform_data {
	int gpio_rst;
	int gpio_pwr;
};

#endif /* _ONESEG_TUNERPM_H_ */
