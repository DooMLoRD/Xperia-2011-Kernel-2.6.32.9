/* drivers/media/video/msm/mt9v114.h
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _MT9V114_H_
#define _MT9V114_H_

#include <linux/types.h>
#include <mach/camera.h>

enum camera_devmode_type {
	CAMERA_MODE_STANDBY,
	CAMERA_MODE_MONITOR,
};

#define MT9V114_CHIP_ID			0x2283
#define MT9V114_SENSOR_VERSION	0xFFFF

/* VGA size */
#define MT9V114_VGA_SIZE_WIDTH	640
#define MT9V114_VGA_SIZE_HEIGHT	480

#define OP_WRITE_I2C			0xFFFF0000
#define OP_WRITE_I2C_WORD		0xFFFE0000
#define OP_READ_I2C				0xFFFD0000
#define OP_READ_I2C_WORD		0xFFFC0000
#define OP_RETURN				0xF0000000
#define CAM_OP_MASK				0xFFFF0000
#define CAM_LOAD_TABLES_MAX		512

#define I2C_RW_BASIC_LENGTH_MAX 32
#define Q8 0x00000100

struct mt9v114_table_t {
	uint32_t address;
	uint32_t value;
	uint32_t action;
};

extern const struct mt9v114_table_t mt9v114_cam_init[];
extern const struct mt9v114_table_t mt9v114_pll_setting[];

#endif /* _MT9V114_H_ */
