/*
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#ifndef _CAMSENSOR_SEIX006_CAM_DEVDRV_H_
#define _CAMSENSOR_SEIX006_CAM_DEVDRV_H_

#define SEIX006_DEFAULT_CLOCK_RATE  19200000
#define SEIX006_I2C_WRITE_FOOTPRINT 2
#define SEIX006_I2C_MAX_BYTES       60

#define TRUE  1
#define FALSE 0
#define boolean uint8_t

#define HIGH_HIGH_BIT_32  0xFF000000
#define HIGH_LOW_BIT_32   0x00FF0000
#define LOW_HIGH_BIT_32   0x0000FF00
#define LOW_LOW_BIT_32    0x000000FF
#define LOW_BIT_32        0x0000FFFF
#define HIGH_BIT_32       0xFFFF0000

#define BYTE_1  1
#define BYTE_2  2
#define BYTE_4  4

#define SHIFT_8  8
#define SHIFT_16 16
#define SHIFT_24 24

#define SEIX006_USERCTRL_0000 0x0000
#define SEIX006_USERCTRL_0250 0x0250
#define SEIX006_USERCTRL_0254 0x0254
#define SEIX006_USERCTRL_025C 0x025C
#define SEIX006_USERCTRL_0258 0x0258
#define SEIX006_USERCTRL_0260 0x0260
#define SEIX006_USERCTRL_0264 0x0264

#define SEIX006_ADJ_4A04 0x4A04
#define SEIX006_ADJ_4A06 0x4A06
#define SEIX006_ADJ_4A08 0x4A08
#define SEIX006_ADJ_4A0A 0x4A0A

#define SEIX006_WB_DATA_OTPB0_2_MASK      0x00000003
#define SEIX006_AF_MASK                   0x000003FF
#define SEIX006_SHD_MASK                  0x0000000F

#define SEIX006_AF_4876 0x4876
#define SEIX006_AF_487A 0x487A
#define SEIX006_AF_487E 0x487E
#define SEIX006_AF_4878 0x4878
#define SEIX006_AF_487C 0x487C
#define SEIX006_AF_4880 0x4880
#define SEIX006_AF_486C 0x486C
#define SEIX006_AF_486E 0x486E
#define SEIX006_AF_4870 0x4870
#define SEIX006_AF_4872 0x4872
#define SEIX006_AF_4844 0x4844
#define SEIX006_AF_486A 0x486A
#define SEIX006_AF_495E 0x495E
#define SEIX006_AF_4960 0x4960
#define SEIX006_AF_4822 0x4822
#define SEIX006_AF_4824 0x4824
#define SEIX006_AF_4838 0x4838

#define SEIX006_USERCTRL_0010     0x0010
#define SEIX006_SOUT_6D76     0x6D76
#define SEIX006_USERCTRL_0004  0x0004

#define SEIX006_SCAN_RANGE_REG_AUTO     0x01D3
#define SEIX006_SCAN_RANGE_REG_TWILIGHT 0x01D8
#define SEIX006_SCAN_RANGE_REG_SPORTS   0x01D7
#define SEIX006_SCAN_RANGE_AUTO         0x04
#define SEIX006_SCAN_RANGE_MACRO        0x00

#define SEIX006_V_FLAG_OTP_MASK        0x00000010
#define SEIX006_VENDOR_ID_OTP_MASK     0x01000000

#define SEIX006_CAM_REV_ES1 0x3009
#define SEIX006_CAM_REV_ES2 0x3013

#define SEIX006_INTSTS_JPEG_UPDATE_STS_MASK 0x04
#define SEIX006_INTSTS_CM_CHANGED_STS_MASK 0x02

#define SEIX006_MSTS_MON_VAL 0x00
#define SEIX006_MSTS_HR_VAL 0x01
#define SEIX006_MSTS_CAP_VAL 0x02
#define SEIX006_MSTS_MOV_VAL 0x03
#define SEIX006_AFSTS_VAL 0x03
#define SEIX006_BSTS_VAL 0x01

#define SEIX006_PICT_OUT_INFO_NOW_PICTOUTINFO_NOW_MASK 0x03

#define SEIX006_PICT_OUT_INFO_NOW_MONITOR 0
#define SEIX006_PICT_OUT_INFO_NOW_HARF_RELEASE 1
#define SEIX006_PICT_OUT_INFO_NOW_CAPTURE 2
#define SEIX006_PICT_OUT_INFO_NOW_MOVIE 3

#define SEIX006_POLLING_PERIOD 10
#define SEIX006_POLLING_TIMES 110

#define SEIX006_FLASH_NEEDED_AE_MIN 28000

enum seix006_vendor
{
	VENDOR_ID_0,
	VENDOR_ID_1,
	VENDOR_ID_2
};

enum seix006_otp
{
	OTP_0 = 0,
	OTP_1,
	OTP_NO_DATA_WRITTEN
};

enum camera_devmode_type
{
	CAMERA_MODE_STANDBY,
	CAMERA_MODE_MONITOR,
	CAMERA_MODE_HALF_RELEASE,
	CAMERA_MODE_CAPTURE,
	CAMERA_MODE_MOVIE
};

struct seix006_work {
	struct work_struct work;
};

struct seix006_calibration_data
{
	int16_t normr;
	int16_t normb;
	uint16_t awbprer;
	uint16_t awbpreb;
	uint16_t otp_inf;
	uint16_t otp_macro;
	uint16_t af_c;
	uint16_t af_d;
	uint16_t af_e;
	uint16_t af_g_k;
	uint16_t af_g_s;
	uint16_t af_i;
	uint16_t af_j;
	uint16_t af_k;
	uint16_t af_l;
	uint16_t af_m;
	uint8_t shd_index;
};

struct seix006_ctrl_t
{
	int8_t opened;
	struct msm_camera_sensor_info *sensordata;
	struct seix006_cam_devdrv_work *sensorw;
	struct i2c_client *client;
	struct msm_sensor_resp *resp;

	struct seix006_calibration_data calibration_data;

	uint16_t camera_revision;
	enum seix006_vendor vendor_id;

	int8_t init_complete;
	uint16_t focus_mode;

	enum camera_devmode_type dev_mode;
	enum camera_scene scene;
	bool autoflash_used;
	bool autoflash_assist_light_on;
	bool autoflash_poll_reg_x2AA;
	int16_t autoflash_cmds[3][2];
	uint16_t autoflash_reg_x288;
	int16_t autoflash_reg_x284;
	uint16_t autoflash_reg_x26A;
	uint16_t autoflash_reg_x26C;
	uint16_t af_4838_val;
	uint16_t scan_range_reg;
	uint16_t scan_range_val;
	unsigned long aeawb_timeout;
};

#endif /* _CAMSENSOR_SEIX006_CAM_DEVDRV_H_ */
