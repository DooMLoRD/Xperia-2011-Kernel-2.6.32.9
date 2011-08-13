/* /kernel/arch/arm/mach-msm/board-semc_mogami-gpio.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _BOARD_SEMC_MOGAMI_GPIO_H
#define _BOARD_SEMC_MOGAMI_GPIO_H

extern struct msm_gpio qup_i2c_gpios_hw[];
extern struct msm_gpio qup_i2c_gpios_io[];
extern int qup_i2c_gpios_hw_size;

extern struct msm_gpio qsd_spi_gpio_config_data[];
extern int qsd_spi_gpio_config_data_size;

extern struct msm_gpio msm_i2c_gpios_hw[];
extern struct msm_gpio msm_i2c_gpios_io[];
extern int msm_i2c_gpios_hw_size;

#endif /* _BOARD_SEMC_MOGAMI_GPIO_H */
