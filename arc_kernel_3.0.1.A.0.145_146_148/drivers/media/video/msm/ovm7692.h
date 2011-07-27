/*
 * Copyright (c) 2009,2010 Sony Ericsson Mobile Communications
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */

#ifndef _OVM7692_H_
#define _OVM7692_H_

#include <linux/types.h>
#include <mach/camera.h>

struct regval_list {
	uint8_t reg_num;
	uint8_t value;
};

extern const struct regval_list ovm7692_ref_vga_regs[];
extern uint32_t size_ovm7692_ref_vga_regs;

extern const struct regval_list ovm7692_device_init_regs[];
extern uint32_t size_ovm7692_device_init_regs;

extern const struct regval_list ovm7692_framerate_30_regs[];
extern uint32_t size_ovm7692_framerate_30_regs;

extern const struct regval_list ovm7692_framerate_15_regs[];
extern uint32_t size_ovm7692_framerate_15_regs;

enum camera_devmode_type {
	CAMERA_MODE_STANDBY,
	CAMERA_MODE_MONITOR,
};

#endif /* _OVM7692_H_ */
