/* [kernel/arch/arm/mach-msm/include/mach/simple_remote_msm7x30_pf.h]
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * Authors: Takashi Shiina <takashi.shiina@sonyericsson.com>
 *          Tadashi Kubo <tadashi.kubo@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef LINUX_SIMPLE_REMOTE_MSM7X30_PLATFORM
#define LINUX_SIMPLE_REMOTE_MSM7X30_PLATFORM

#include <mach/pmic.h>

#define SIMPLE_REMOTE_PF_NAME "simple_remote_pf"

struct vreg;

struct simple_remote_platform_regulators {
	const char *name;
	struct vreg *reg;
};

struct simple_remote_platform_data {
	unsigned int headset_detect_enable_pin;
	unsigned int headset_detect_read_pin;
	int headset_mode_switch_pin;  /* Set to -1 if not supported by HW */

	int (*initialize)(struct simple_remote_platform_data *);
	void (*deinitialize)(struct simple_remote_platform_data *);

	struct simple_remote_platform_regulators *regs;
	u16 num_regs;

	enum hsed_controller controller;
	int invert_plug_det; /* Default is 0 = plug inserted */
};

void simple_remote_pf_button_handler(uint32_t key, uint32_t event);

#endif /* LINUX_SIMPLE_REMOTE_MSM7X30_PLATFORM */
