/*
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/board.h>

#include <mach/vreg.h>

#include "seix006.h"
#include "seix006_cam_devdrv_table.h"

/* Some Robyn specific functions, also dependent on addition to board.h, not
 * necessary for Zeus at the moment are removed via the following define. */
#define USE_ZEUS_POWER_MANAGEMENT

#ifdef SEDBG
#ifdef CDBG
#undef CDBG
#endif

#define CDBG(fmt, args...) printk(KERN_INFO "msm_cam_seix006: " fmt, ##args)
#endif /* SEDBG */

#define SEIX006_I2C_NAME "seix006"
#define SEIX006_MSM_CAMERA_NAME "msm_camera_seix006"

/* ******** Local functions ************* */
static int32_t seix006_gpio_access(int gpio_pin, int dir);
#ifndef USE_ZEUS_POWER_MANAGEMENT
	static int32_t seix006_resource_enable(struct msm_camera_sensor_pwr *resource);
	static int32_t seix006_resource_disable(struct msm_camera_sensor_pwr *resource);
#endif
static int32_t seix006_sensor_on(void);
static int32_t seix006_sensor_init(void);
static void seix006_sensor_off(void);
static int seix006_sensor_open(const struct msm_camera_sensor_info *data);
static int seix006_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit seix006_i2c_remove(struct i2c_client *client);
static int32_t seix006_i2c_write(uint16_t address, uint8_t data_length, const uint8_t * data);
static int32_t seix006_i2c_read(uint16_t address, uint8_t length, uint8_t * data);
static int32_t seix006_set_sensor_mode(struct sensor_cfg_data cfg_data);
static int32_t seix006_monitor_config(void);
static int32_t seix006_raw_snapshot_config(void);
static int32_t seix006_raw_rgb_snapshot_config(void);
static int32_t seix006_raw_snapshot_start(void);
static int32_t seix006_snapshot_config(void);
static int32_t seix006_half_release_config(void);
static int32_t seix006_set_test_pattern(enum set_test_pattern_t mode);
static int32_t seix006_set_iso(uint16_t iso_mode);
static int32_t seix006_set_focus_mode(enum camera_focus_mode focus_mode);
static int32_t seix006_update_focus_mode(enum camera_focus_mode focus_mode);
static int32_t seix006_set_scene(enum camera_scene scene);
static int32_t seix006_update_scene(enum camera_scene start_scene);
static int32_t seix006_set_dimensions(struct camera_dimension_t dimension);
static int32_t seix006_set_preview_dimension(struct camera_preview_dimension_t dimension);
static int32_t seix006_get_af_status(enum camera_af_status *status);
static int32_t seix006_get_exif(struct cam_ctrl_exif_params_t *exif);
static int32_t seix006_read_vendor_data(void);
static void seix006_add_bytes_in_switched_endian(uint8_t * p_package_buf, uint8_t pkg_position,
						uint8_t size_of_data, uint8_t * Data);
static int32_t seix006_send_reg_table(const struct reg_entry *table, uint32_t tables_to_send);
static int32_t seix006_check_msts(uint8_t value, uint32_t timeout);
static int32_t seix006_check_bsts(uint8_t value, uint32_t timeout);
static int32_t seix006_check_afsts(uint8_t value, uint32_t timeout);
static int32_t seix006_refresh_monitor(uint32_t timeout);
static int32_t seix006_cam_is_assist_light_needed(int *result);
static int32_t seix006_set_framerate(uint16_t fps);
static int32_t seix006_write_calibration_data(struct seix006_calibration_data);
static int init_thread(void* data);
static int autoflash_enable(int onoff);
static int autoflash_strobe(int onoff);
static int autoflash_adjust(void);
static int32_t seix006_movie_config(void);
static int32_t seix006_raw_rgb_stream_config(void);

static long seix006_set_effect(int mode, int effect);
static long seix006_set_exposure_mode(int exp_mode);
static long seix006_set_wb(int wb_type);
static long seix006_set_sharpness(uint8_t value);
static long seix006_set_contrast(uint8_t value);
static long seix006_set_img_quality(uint8_t value);
static long seix006_set_brightness(uint8_t value);
static long seix006_set_exposure_compensation(int8_t value);
static int seix006_get_capture_started(void);
static long seix006_set_flash(uint8_t flash_mode);
static long setix006_set_led_state(uint8_t led_state);

/* ********** Local variables/structs ************ */

static struct seix006_ctrl_t *seix006_ctrl = NULL;
static DECLARE_WAIT_QUEUE_HEAD(seix006_wait_queue);
DECLARE_MUTEX(seix006_sem);
static int is_capture_started = 0;
enum {
	FLASH_OFF,
	FLASH_AUTO,
	FLASH_ON
};
static int camera_flash = FLASH_OFF;
static int camera_variable_frame_rate = 1;
/* 1 if continous auto focus is active */
static int camera_continous_autofocus;
DEFINE_MUTEX(seix006_capture_lock);

/*
 *  Switches places of Most Significant Bit (MSB) and Least Significant  *
 *  Bit (LSB) before adding the data to the package buffer               *
 */
static void seix006_add_bytes_in_switched_endian(uint8_t *p_package_buf,
	uint8_t pkg_position, uint8_t size_of_data, uint8_t *Data)
{
	int MSB_byte_number, byte_number;
	for (MSB_byte_number = size_of_data - 1, byte_number = 0; MSB_byte_number
		>= 0; --MSB_byte_number, ++byte_number) {
		memcpy(p_package_buf + pkg_position + byte_number, Data
			+ MSB_byte_number, sizeof(uint8_t));
	}
}

/**
 * I2C Device ID Structure Body. *
 */
static const struct i2c_device_id seix006_id[] =
{
	{ SEIX006_I2C_NAME, 0},
	{ }
};

/**
 * I2C Device Structure Body.
 *
 */
static struct i2c_driver seix006_driver =
{
	.id_table = seix006_id,
	.probe = seix006_i2c_probe,
	.remove = __exit_p(seix006_i2c_remove),
	.driver =
	{
		.name = SEIX006_I2C_NAME,
	},
};

/**
 * Precess IOCTL messages.
 *
 */
int seix006_sensor_config(void __user *argp)
{
	int32_t ret = 0;
	struct sensor_cfg_data cfg_data;

	CDBG("seix006_sensor_config [S]\n");

	ret = copy_from_user(&cfg_data, (void *) argp,
		sizeof(struct sensor_cfg_data));
	if (ret) {
		CDBG("seix006_sensor_config copy_from_user failed\n");
		return -EFAULT;
	}

	if (cfg_data.cfgtype == CFG_SET_MODE &&
		cfg_data.mode == SENSOR_PREVIEW_MODE &&
		seix006_ctrl->dev_mode == CAMERA_MODE_MONITOR &&
		seix006_ctrl->init_complete) {
		CDBG("Already running in monitor");
		return 0;
	}

	down(&seix006_sem);

	CDBG("seix006_sensor_config cfgtype = %d\n", cfg_data.cfgtype);
	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		ret = seix006_set_sensor_mode(cfg_data);
		break;

	case CFG_SET_TEST_PATTERN:
		ret = seix006_set_test_pattern(cfg_data.cfg.set_test_pattern);
		break;

	case CFG_SET_ISO:
		ret = seix006_set_iso(cfg_data.cfg.iso_mode);
		break;

	case CFG_GET_AF_STATUS:
		ret = seix006_get_af_status(&cfg_data.cfg.af_status);
		break;

	case CFG_GET_EXIF:
		ret = seix006_get_exif(&cfg_data.cfg.exif);
		break;

	case CFG_SET_SCENE:
		ret = seix006_set_scene(cfg_data.cfg.scene);
		break;

	case CFG_SET_DIMENSION:
		ret = seix006_set_dimensions(cfg_data.cfg.dimension);
		break;

	case CFG_SET_SENSOR_DIMENSION:
		ret = seix006_set_preview_dimension(cfg_data.cfg.preview_dimension);
		break;

	case CFG_GET_AF_ASSIST_LIGHT:
		ret = seix006_cam_is_assist_light_needed((int *) &cfg_data.rs);
		break;

	case CFG_SET_FPS:
		ret = seix006_set_framerate(cfg_data.cfg.fps.f_mult);
		break;

	case CFG_SET_EFFECT:
		ret = seix006_set_effect(cfg_data.mode, cfg_data.cfg.effect);
		break;

	case CFG_SET_CONTRAST:
		ret = seix006_set_contrast(cfg_data.cfg.contrast);
		break;

	case CFG_SET_BRIGHTNESS:
		ret = seix006_set_brightness(cfg_data.cfg.brightness);
		break;

	case CFG_SET_SHARPNESS:
		ret = seix006_set_sharpness(cfg_data.cfg.sharpness);
		break;

	case CFG_SET_WB:
		ret = seix006_set_wb(cfg_data.cfg.wb_type);
		break;

	case CFG_SET_IMG_QUALITY:
		ret = seix006_set_img_quality(cfg_data.cfg.quality);
		break;

	case CFG_SET_EXPOSURE_COMPENSATION:
		ret = seix006_set_exposure_compensation(cfg_data.cfg.ev);
		break;

	case CFG_SET_EXPOSURE_MODE:
		ret = seix006_set_exposure_mode(cfg_data.cfg.exp_mode);
		break;

	case CFG_SET_FLASH:
		ret = seix006_set_flash(cfg_data.cfg.flashled);
		break;
	default:
		CDBG("seix006_sensor_config cfgtype failed\n");
		ret = -EFAULT;
		break;
	}

	up(&seix006_sem);

	ret = copy_to_user((void *) argp, &cfg_data,
		sizeof(struct sensor_cfg_data));
	if (ret) {
		CDBG("seix006_sensor_config copy_to_user failed\n");
		return -EFAULT;
	}

	CDBG("seix006_sensor_config [E]\n");

	return ret;
}

/**
 * Release
 *
 */
int seix006_sensor_release(void)
{
	CDBG("seix006_sensor_release [S]\n");

	down(&seix006_sem);

	if(seix006_ctrl->opened) {
		seix006_sensor_off();
		seix006_ctrl->opened = 0;
	}

	seix006_ctrl->dev_mode = CAMERA_MODE_STANDBY;

	up(&seix006_sem);

	CDBG("seix006_sensor_release [E]\n");

	return 0;
}

/**
 * Exit
 *
 */
void seix006_exit(void)
{
	CDBG("seix006_exit [S]\n");

	i2c_del_driver(&seix006_driver);

	CDBG("seix006_exit [E]\n");
}

/**
 * Probe
 *
 */
static int seix006_camera_probe(const struct msm_camera_sensor_info *info, struct msm_sensor_ctrl *s)
{
	int rc = 0;

	CDBG("seix006_camera_probe [S]\n");

	seix006_ctrl = kzalloc(sizeof(struct seix006_ctrl_t), GFP_KERNEL);
	if (NULL == seix006_ctrl || NULL == info) {
		CDBG("seix006_cam_probe memory allocation failed\n");
		rc = -EINVAL;
		goto probe_done;
	}

	seix006_ctrl->sensordata = (struct msm_camera_sensor_info*) info;
	rc = i2c_add_driver(&seix006_driver);
	if (IS_ERR_VALUE(rc)) {
		CDBG("seix failed i2c_add_driver\n");
		kfree(seix006_ctrl);
		goto probe_done;
	}
	CDBG("seix pass i2c_add_driver\n");

	seix006_ctrl->dev_mode = CAMERA_MODE_STANDBY;

	/* Power on sensor */
	rc = seix006_sensor_on();
	if (rc < 0) {
		CDBG("seix006_camera_probe sensor_on failed\n");
		goto probe_done;
	}

	/* sensor power on - gpio */
	rc = seix006_gpio_access(seix006_ctrl->sensordata->sensor_reset, 0);
	if (rc) {
		CDBG("seix006_probe CAM_RESET_N release failed\n");
		goto probe_done;
	}
	udelay(5);
	/* (STANDBY_N = low) */
	rc = seix006_gpio_access(seix006_ctrl->sensordata->sensor_pwd, 0);
	if (rc) {
		CDBG("sexi006_probe STANDBY failed\n");
		goto probe_done;
	}
	mdelay(2);
	/* CAM_RESET_N = high */
	rc = seix006_gpio_access(seix006_ctrl->sensordata->sensor_reset, 1);
	if (rc) {
		CDBG("seix006_probe CAM_RESET_N release failed\n");
		goto probe_done;
	}

	mdelay(20);

	rc = seix006_gpio_access(seix006_ctrl->sensordata->sensor_pwd, 1);
	if (rc) {
		CDBG("sexi006_probe STANDBY failed\n");
		goto probe_done;
	}

	mdelay(10);

	rc = seix006_read_vendor_data();
	if (rc != 0) {
		CDBG("seix006_read_vendr_data failed\n");
		goto probe_done;
	}

	CDBG("<<<<<<<<<< Camera vendor [%d] >>>>>>>>>>>\n", seix006_ctrl->vendor_id);
	CDBG("<<<<<<<<<< Camera revision[%d] >>>>>>>>>>>\n", seix006_ctrl->camera_revision);

	s->s_init = seix006_sensor_open;
	s->s_release = seix006_sensor_release;
	s->s_config = seix006_sensor_config;
	s->s_get_capture_started = seix006_get_capture_started;

probe_done:
	if (seix006_ctrl) {
		seix006_sensor_off();
	}
	CDBG("seix006_camera_probe [E] %d\n ", rc);
	return rc;
}
static int32_t seix006_read_vendor_data(void)
{
	int32_t rc = 0;
	uint32_t data = 0;
	uint8_t v_flag_otp0 = 0;
	uint8_t v_flag_otp1 = 0;
	enum seix006_otp otp_id = OTP_NO_DATA_WRITTEN;
	uint32_t c_nr_1 = 0;
	uint32_t c_nr_2 = 0;
	uint8_t c_nr_res = 0;
	uint32_t c_nb = 0;
	uint32_t c_pr = 0;
	uint32_t c_pb = 0;
	uint16_t inormr_k = 0x10FB;
	uint16_t inormb_k = 0x10AB;
	uint16_t iawbprer_k = 0x013C;
	uint16_t iawbpreb_k = 0x0241;
	uint16_t inormr_s = 0x10BD;
	uint16_t inormb_s = 0x0EE0;
	uint16_t iawbprer_s = 0x0146;
	uint16_t iawbpreb_s = 0x0239;


	CDBG("seix006_read_vendor_data [S}\n");

	rc = seix006_i2c_read(SEIX006_USERCTRL_0000, BYTE_2, (uint8_t *) & data);
	seix006_ctrl->camera_revision = data & LOW_BIT_32;
	CDBG("ROM 0x0000: 0x%x", seix006_ctrl->camera_revision);

	rc += seix006_i2c_read(SEIX006_USERCTRL_0250, BYTE_4, (uint8_t *) & data);
	if ((data & SEIX006_V_FLAG_OTP_MASK) > 0)
		v_flag_otp0 = 1;
	CDBG("---------------0\n");
	rc += seix006_i2c_read(SEIX006_USERCTRL_025C, BYTE_4, (uint8_t *) & data);
	if ((data & SEIX006_V_FLAG_OTP_MASK) > 0)
		v_flag_otp1 = 1;

	if (v_flag_otp0 == 0) {
		if (v_flag_otp1 == 0)
			otp_id = OTP_NO_DATA_WRITTEN;
		else
			otp_id = OTP_1;
	} else {
		if (v_flag_otp1 == 0)
			otp_id = OTP_0;
		else
			otp_id = OTP_1;
	}
	CDBG("---------------v_flag_otp0=%d v_flag_otp1=%d 1\n", v_flag_otp0, v_flag_otp1);

	switch (otp_id) {
	case OTP_NO_DATA_WRITTEN:
		CDBG("seix006_read_vendor_data failed! No valid OTP\n");
		return -1;
	case OTP_0:
		rc += seix006_i2c_read(SEIX006_USERCTRL_0258, BYTE_4, (uint8_t *) & data);
		CDBG("seix006_read_vendor_data OTP_0\n");
		break;
	case OTP_1:
		rc += seix006_i2c_read(SEIX006_USERCTRL_0264, BYTE_4, (uint8_t *) & data);
		CDBG("seix006_read_vendor_data OTP_1\n");
		break;
	default:
		return -1;
	}

	if ((data & SEIX006_VENDOR_ID_OTP_MASK) == 0) {
		seix006_ctrl->vendor_id = VENDOR_ID_0;
	} else {
		seix006_ctrl->vendor_id = VENDOR_ID_1;
	}
	CDBG("%s seix006_ctrl->vendor_id=%d", __FUNCTION__, seix006_ctrl->vendor_id);


	if (seix006_ctrl->camera_revision != SEIX006_CAM_REV_ES1) {
		switch (otp_id) {
		case OTP_NO_DATA_WRITTEN:
			CDBG("seix006_read_initial_values failed! No valid OTP\n");
			return -1;
		case OTP_0:
			rc += seix006_i2c_read(SEIX006_USERCTRL_0258, BYTE_4, (uint8_t *) & data);
			seix006_ctrl->calibration_data.shd_index = ((data >> 20) & SEIX006_SHD_MASK);
			rc += seix006_i2c_read(SEIX006_USERCTRL_0250, BYTE_4, (uint8_t *) & data);
			c_nr_1 = (data >> 26);
			rc += seix006_i2c_read(SEIX006_USERCTRL_0254, BYTE_4, (uint8_t *) & data);
			c_nr_2 = ((data & SEIX006_WB_DATA_OTPB0_2_MASK) << 6);
			break;
		case OTP_1:
			rc += seix006_i2c_read(SEIX006_USERCTRL_0264, BYTE_4, (uint8_t *) & data);
			seix006_ctrl->calibration_data.shd_index = ((data >> 20) & SEIX006_SHD_MASK);
			rc += seix006_i2c_read(SEIX006_USERCTRL_025C, BYTE_4, (uint8_t *) & data);
			c_nr_1 = (data >> 26);
			rc += seix006_i2c_read(SEIX006_USERCTRL_0260, BYTE_4, (uint8_t *) & data);
			c_nr_2 = ((data & SEIX006_WB_DATA_OTPB0_2_MASK) << 6);
			break;
		default:
			return -1;
			break;
		}

		c_nr_res = (c_nr_1 + c_nr_2);
		c_nb = ((data >> 2) & LOW_LOW_BIT_32);
		c_pr = ((data >> 10) & LOW_LOW_BIT_32);
		c_pb = ((data >> 18) & LOW_LOW_BIT_32);
		if (seix006_ctrl->vendor_id == VENDOR_ID_0) {
			seix006_ctrl->calibration_data.normr = (inormr_k * (128 + c_nr_res)) / 256;
			seix006_ctrl->calibration_data.normb = (inormb_k * (128 + c_nb)) / 256;
			seix006_ctrl->calibration_data.awbprer = (iawbprer_k * (128 + c_pr)) / 256;
			seix006_ctrl->calibration_data.awbpreb = (iawbpreb_k * (128 + c_pb)) / 256;
		} else if (seix006_ctrl->vendor_id == VENDOR_ID_1) {
			seix006_ctrl->calibration_data.normr = (inormr_s * (128 + c_nr_res)) / 256;
			seix006_ctrl->calibration_data.normb = (inormb_s * (128 + c_nb)) / 256;
			seix006_ctrl->calibration_data.awbprer = (iawbprer_s * (128 + c_pr)) / 256;
			seix006_ctrl->calibration_data.awbpreb = (iawbpreb_s * (128 + c_pb)) / 256;
		}

		switch (otp_id) {
		case OTP_NO_DATA_WRITTEN:
			CDBG("seix006_read_initial_values failed! No valid OTP\n");
			return -1;
		case OTP_0:
			rc += seix006_i2c_read(SEIX006_USERCTRL_0250, BYTE_4, (uint8_t *) & data);
			seix006_ctrl->calibration_data.otp_inf = ((data >> 5) & SEIX006_AF_MASK);
			seix006_ctrl->calibration_data.otp_macro = ((data >> 16) & SEIX006_AF_MASK);
			break;
		case OTP_1:
			rc += seix006_i2c_read(SEIX006_USERCTRL_025C, BYTE_4, (uint8_t *) & data);
			seix006_ctrl->calibration_data.otp_inf = ((data >> 5) & SEIX006_AF_MASK);
			seix006_ctrl->calibration_data.otp_macro = ((data >> 16) & SEIX006_AF_MASK);
			break;
		default:
			return -1;
		}

		seix006_ctrl->calibration_data.af_c =
			(((((8 * seix006_ctrl->calibration_data.otp_macro) -
			(8 * seix006_ctrl->calibration_data.otp_inf)) / 6) + 6) / 8);
		seix006_ctrl->calibration_data.af_d = seix006_ctrl->calibration_data.af_c / 4;
		seix006_ctrl->calibration_data.af_e =
			seix006_ctrl->calibration_data.otp_inf +
			(seix006_ctrl->calibration_data.af_c * 8);
		seix006_ctrl->calibration_data.af_i =
			seix006_ctrl->calibration_data.otp_inf +
			(seix006_ctrl->calibration_data.af_c * 5);
		seix006_ctrl->calibration_data.af_j = seix006_ctrl->calibration_data.af_c * 2;
		seix006_ctrl->calibration_data.af_k = seix006_ctrl->calibration_data.af_d / 2;
		seix006_ctrl->calibration_data.af_l = seix006_ctrl->calibration_data.af_d * 4;
		seix006_ctrl->calibration_data.af_m =
			seix006_ctrl->calibration_data.otp_inf +
			(seix006_ctrl->calibration_data.af_c * 6);
		seix006_ctrl->calibration_data.af_g_k = 1023;
		seix006_ctrl->calibration_data.af_g_s = 255;
	}
	CDBG("--------------- 6\n");
	return rc;
}

/**
 * Set calibration data
 *
 */

static int32_t seix006_write_calibration_data(struct seix006_calibration_data calibration_data)
{
	int32_t ret = 0;

	seix006_i2c_write(SEIX006_ADJ_4A04, BYTE_2, (uint8_t *) & calibration_data.normr);
	seix006_i2c_write(SEIX006_ADJ_4A06, BYTE_2, (uint8_t *) & calibration_data.normb);
	seix006_i2c_write(SEIX006_ADJ_4A08, BYTE_2, (uint8_t *) & calibration_data.awbprer);
	seix006_i2c_write(SEIX006_ADJ_4A0A, BYTE_2, (uint8_t *) & calibration_data.awbpreb);

	seix006_i2c_write(SEIX006_AF_4876, BYTE_2, (uint8_t *) & calibration_data.otp_inf);
	seix006_i2c_write(SEIX006_AF_487A, BYTE_2, (uint8_t *) & calibration_data.otp_inf);
	seix006_i2c_write(SEIX006_AF_486C, BYTE_2, (uint8_t *) & calibration_data.af_c);
	seix006_i2c_write(SEIX006_AF_4870, BYTE_2, (uint8_t *) & calibration_data.af_c);
	seix006_i2c_write(SEIX006_AF_486E, BYTE_2, (uint8_t *) & calibration_data.af_d);
	seix006_i2c_write(SEIX006_AF_4872, BYTE_2, (uint8_t *) & calibration_data.af_d);

	if (seix006_ctrl->vendor_id == VENDOR_ID_0) {
		if (calibration_data.af_e > calibration_data.af_g_k) {
			calibration_data.af_e = calibration_data.af_g_k;
		}
		seix006_i2c_write(SEIX006_AF_4878, BYTE_2, (uint8_t *) & calibration_data.af_e);
		seix006_i2c_write(SEIX006_AF_4880, BYTE_2, (uint8_t *) & calibration_data.af_m);
		seix006_i2c_write(SEIX006_AF_495E, BYTE_2, (uint8_t *) & calibration_data.af_e);
	}
	if (seix006_ctrl->vendor_id == VENDOR_ID_1) {
		if (calibration_data.af_e > calibration_data.af_g_s) {
			calibration_data.af_e = calibration_data.af_g_s;
		}
		seix006_i2c_write(SEIX006_AF_4878, BYTE_2, (uint8_t *) & calibration_data.af_e);
		seix006_i2c_write(SEIX006_AF_4880, BYTE_2, (uint8_t *) & calibration_data.af_m);
		seix006_i2c_write(SEIX006_AF_495E, BYTE_2, (uint8_t *) & calibration_data.af_e);
	}

	seix006_i2c_write(SEIX006_AF_487E, BYTE_2, (uint8_t *) & calibration_data.otp_inf);
	seix006_i2c_write(SEIX006_AF_487C, BYTE_2, (uint8_t *) & calibration_data.af_i);
	seix006_i2c_write(SEIX006_AF_4844, BYTE_2, (uint8_t *) & calibration_data.af_j);
	seix006_i2c_write(SEIX006_AF_486A, BYTE_2, (uint8_t *) & calibration_data.otp_inf);
	seix006_i2c_write(SEIX006_AF_4960, BYTE_2, (uint8_t *) & calibration_data.otp_inf);
	seix006_i2c_write(SEIX006_AF_4822, BYTE_2, (uint8_t *) & calibration_data.af_k);
	seix006_i2c_write(SEIX006_AF_4824, BYTE_2, (uint8_t *) & calibration_data.af_d);
	seix006_i2c_write(SEIX006_AF_4838, BYTE_2, (uint8_t *) & calibration_data.af_l);

	return ret;
}

/**
 * Set sensor mode
 *
 */
static int32_t seix006_set_sensor_mode(struct sensor_cfg_data cfg_data)
{
	int32_t ret = 0;

	CDBG("seix006_set_sensor_mode [S]\n");

	switch (cfg_data.mode) {
	case SENSOR_PREVIEW_MODE:
		ret = seix006_monitor_config();
		CDBG("seix006_set_sensor_mode SENSOR_PREVIEW_MODE done\n");
		break;
	case SENSOR_SNAPSHOT_MODE:
		ret = seix006_snapshot_config();
		CDBG("seix006_set_sensor_mode SENSOR_SNAPSHOT_MODE done\n");
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		ret = seix006_raw_snapshot_config();
		CDBG("seix006_set_sensor_mode SENSOR_RAW_SNAPSHOT_MODE\n");
		break;
	case SENSOR_RAW_SNAPSHOT_START:
		ret = seix006_raw_snapshot_start();
		CDBG("seix006_set_sensor_mode SENSOR_RAW_SNAPSHOT_START\n");
		break;
	case SENSOR_HALF_RELEASE_MODE:
		if (seix006_ctrl->dev_mode == CAMERA_MODE_HALF_RELEASE) {
			CDBG("seix006_set_sensor_mode CAMERA_MODE_HALF_RELEASE\
					already in half release mode\n");
			/* This occurs if application calls autofocus again
			 * without leaving half release mode. The sensor
			 * requires us to go to monitor mode before a new
			 * focus can be performed */
			seix006_monitor_config();
			/* Delay is necessary to allow a few frames to reach
			 * the sensor so it can adapt tochanged light
			 * conditions etc */
			msleep(1000);
		}
		seix006_set_focus_mode(cfg_data.cfg.focus_mode);
		/* check if asked to set focus mode only or start auto focus */
		if (cfg_data.rs)
			break;

		ret = seix006_half_release_config();
		CDBG("seix006_set_sensor_mode SENSOR_HALF_RELEASE_MODE done\n");
		break;
	case SENSOR_RAW_RGB_STREAM_MODE:
		ret = seix006_raw_rgb_stream_config();
		CDBG("seix006_set_sensor_mode SENSOR_RAW_RGB_STREAM_MODE\n");
		break;
	case SENSOR_MOVIE_MODE:
		ret = seix006_movie_config();
		CDBG("seix006_set_sensor_mode SENSOR_MOVIE_MODE done\n");
		break;
	case SENSOR_RAW_RGB_SNAPSHOT_MODE:
		ret = seix006_raw_rgb_snapshot_config();
		CDBG("seix006_set_sensor_mode SENSOR_RAW_RGB_SNAPSHOT_MODE\n");
		break;
	default:
		CDBG("seix006_set_sensor_mode failed\n");
		ret = -EINVAL;
		break;
	}

	CDBG("seix006_set_sensor_mode [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Set monitor mode
 *
 */
static int32_t seix006_monitor_config()
{
	int32_t ret = 0;
	uint8_t afclr = 0x01;
	char data;
	uint8_t datawb = 0;

	CDBG("seix006_monitor_config [S]\n");

	if (seix006_ctrl->dev_mode == CAMERA_MODE_MONITOR) {
		CDBG("seix006_monitor_config Already in monitor mode. Do nothing...\n");
		goto monitor_done;
	}

	ret = seix006_i2c_read(SEIX006_USERCTRL_0010, BYTE_1, &data);

	if (data == SEIX006_MSTS_HR_VAL ||
		seix006_ctrl->autoflash_assist_light_on) {
		switch (seix006_ctrl->scene) {
		case SENSOR_SCENE_TWILIGHT:
		case SENSOR_SCENE_TWILIGHT_PORTRAIT:
			if (seix006_ctrl->autoflash_assist_light_on)
				ret = setix006_set_led_state(MSM_CAMERA_LED_OFF);

			ret = seix006_set_regs(seix006_hr_reset);
			if (ret) {
				CDBG("seix006_hr_reset failed\n");
				return ret;
			}
			break;
		case SENSOR_SCENE_AUTO:
		case SENSOR_SCENE_SPORTS:
		case SENSOR_SCENE_BEACH:
		case SENSOR_SCENE_SNOW:
		case SENSOR_SCENE_PORTRAIT:
			if (seix006_ctrl->autoflash_assist_light_on) {
				ret = setix006_set_led_state(MSM_CAMERA_LED_OFF);

				ret = seix006_i2c_read(0x0102, BYTE_1, &datawb);
				if (datawb == 0) {
					datawb = 0x20;
					ret |= seix006_i2c_write(0x0102, BYTE_1, &datawb);
				}
				ret |= seix006_set_regs(seix006_hr_LED_reset);
				if (ret) {
					CDBG("seix006_hr_LED_reset failed\n");
					return ret;
				}
			} else {
				ret = seix006_set_regs(seix006_hr_auto_reset);
				if (ret) {
					CDBG("seix006_hr__auto_reset failed\n");
					return ret;
				}
			}
			break;
		default:
			CDBG("seix006_hr_release already set\n");
		}
		seix006_i2c_write(0x4885, BYTE_1, &afclr);
		ret = seix006_check_afsts(SEIX006_AFSTS_VAL, SEIX006_POLLING_TIMES);
		if (ret) {
			CDBG("seix006_monitor_config seix006_check_afsts failed\n");
		}
	}

	ret = seix006_set_regs(seix006_mode_monitor);
	if (ret) {
		CDBG("seix006_monitor_config failed\n");
		return ret;
	}

	/* set frame rate mode in case overwritten by snapshot */
	/* snapshot sets it to variable */
	if (camera_variable_frame_rate == 0)
		seix006_set_regs(seix006_framerate_fixed);

	/*  wait MODESEL_FIX to 0 */
	ret = seix006_check_msts(SEIX006_MSTS_MON_VAL, SEIX006_POLLING_TIMES);
	if (ret)
		CDBG("seix006_monitor_config seix006_check_msts failed\n");
	else
		CDBG("seix006_monitor_config Now in monitor\n");

	seix006_ctrl->dev_mode = CAMERA_MODE_MONITOR;
	seix006_ctrl->autoflash_used = FALSE;
	seix006_ctrl->autoflash_assist_light_on = FALSE;
	seix006_ctrl->autoflash_poll_reg_x2AA = FALSE;

monitor_done:

	if (!seix006_ctrl->init_complete) {
		kthread_run(init_thread, NULL, "sensor_init");
	}

	CDBG("seix006_monitor_config [E] ret[%d]\n", ret);

return ret;
}

/**
 * RGB RAW snapshot config
 *
 */
static int32_t seix006_raw_rgb_snapshot_config()
{
	int32_t ret;
	uint8_t data = 0;
	int32_t count = 0;

	CDBG("seix006_raw_rgb_snapshot_config [S]\n");

	/* Change to capture mode */
	ret = seix006_send_reg_table(seix006_mode_capture_RGB, sizeof_seix006_mode_capture_RGB/sizeof(struct reg_entry));
	if(ret) {
		CDBG("seix006_raw_rgb_snapshot_config send_reg_table failed\n");
		return ret;
	}

	/* wait MODESEL_FIX to 2 */
	ret = seix006_check_msts(SEIX006_MSTS_CAP_VAL, SEIX006_POLLING_TIMES);
	if(ret) {
		CDBG	("seix006_raw_rgb_snapshot_config seix006_check_msts failed\n");
		/* Continue silently */
	}

	while(++count <= 20) {
		if (count == 20) {
			CDBG("seix006_raw_rgb_snapshot_config move to RGB mode failed\n");
			return -EFAULT;
		}
        ret = seix006_i2c_read(0x004, BYTE_1, &data);
        if (ret) {
            CDBG("seix006_raw_rgb_snapshot_config: i2c_read failed\n");
                return ret;
        }
        CDBG("seix006_raw_rgb_snapshot_config: data = 0x%x\n",data);
        if (0 != (data & 0x10)) {
            /*Sensor has moved to RGB snapshot mode*/
            break;
        }
        mdelay(10);
	}
	is_capture_started = 1;
	CDBG("seix006_raw_rgb_snapshot_config [E] ret[%d]\n",ret);
	seix006_ctrl->dev_mode = CAMERA_MODE_CAPTURE;
	return ret;
}
/**
 * RAW snapshot config
 *
 */
static int32_t seix006_raw_snapshot_config()
{
	int ret = 0;
	CDBG("seix006_raw_snapshot_config [S]\n");

	ret = seix006_set_regs(seix006_prepare_mode_capture);
	if (ret) {
		CDBG("seix006_raw_snapshot_config failed\n");
		return ret;
	}
	if (seix006_ctrl->scene == SENSOR_SCENE_TWILIGHT ||
			seix006_ctrl->scene == SENSOR_SCENE_TWILIGHT_PORTRAIT) {
		ret = seix006_check_bsts(SEIX006_BSTS_VAL, SEIX006_POLLING_TIMES);
		if (ret) {
			CDBG("seix006_monitor_config seix006_check_bsts failed\n");
			/* continue silently */
		}
	}

	is_capture_started = 0;
	mdelay(250);
	CDBG("seix006_raw_snapshot_config [E]\n");
	return ret;
}
/* Capture started flag*/
static int seix006_get_capture_started()
{
	int capture_started = 1;

	mutex_lock(&seix006_capture_lock);
#ifdef DBG_SEMC_READ_CAP_STS
	ret = seix006_i2c_read(0x004, BYTE_1, &data);
	if (ret) {
		CDBG("seix006_get_capture_started: i2c_read failed\n");
		mutex_unlock(&seix006_capture_lock);
		return capture_started;
	}
	CDBG("seix006_get_capture_started: data = 0x%x\n",data);
	capture_started =  ((data & 0x3) == 2);
#else
	capture_started = is_capture_started;
	CDBG("seix006_get_capture_started %d\n",capture_started);
#endif
	mutex_unlock(&seix006_capture_lock);
	return capture_started;
}

/**
 * RAW snapshot start
 *
 */
static int32_t seix006_raw_snapshot_start()
{
	int32_t i = 0;
	int32_t ret = 0;

	CDBG("seix006_raw_snapshot_start [S]\n");

	/* if flash used, and autoflash_adjust() calculation was done */
	if (seix006_ctrl->autoflash_assist_light_on && !seix006_ctrl->autoflash_poll_reg_x2AA) {
	/**
	* USE calculated values computed in autoflash_adjust()
	*/
		CDBG(".. FLASH is on: sending AE/AWB offset values to register"
			" 0x282=%d, 0x445c=%d, 0x445e=%d\n",
			seix006_ctrl->autoflash_cmds[0][1],
			seix006_ctrl->autoflash_cmds[1][1], seix006_ctrl->autoflash_cmds[2][1]);

		for (i = 0; i < 3; i++) {
			ret = seix006_i2c_write(
					seix006_ctrl->autoflash_cmds[i][0],
					BYTE_2,
					(uint8_t *) &(seix006_ctrl->autoflash_cmds[i][1]));

			if (ret) {
				CDBG("Failed writing I2C register 0x%4.4x: %d",
					seix006_ctrl->autoflash_cmds[i][0], ret);

				return ret;
			}
		}
	}

	mutex_lock(&seix006_capture_lock);

	ret = seix006_set_regs(seix006_mode_capture);
	if (ret) {
		CDBG("seix006_raw_snapshot_start set_regs failed\n");
		mutex_unlock(&seix006_capture_lock);
		return ret;
	}

	/* wait MODESEL_FIX to 2 */
	ret = seix006_check_msts(SEIX006_MSTS_CAP_VAL, SEIX006_POLLING_TIMES);
	if(ret) {
		CDBG	("seix006_raw_snapshot_config seix006_check_msts failed\n");
		/* Continue silently */
		ret = 0;
	}

	seix006_ctrl->dev_mode = CAMERA_MODE_CAPTURE;
	is_capture_started = 1;
	mutex_unlock(&seix006_capture_lock);
	CDBG("seix006_raw_snapshot_start [E] ret[%d]\n", ret);
	return ret;
}

/**
 * Snapshot config - config sensor as output YUV format
 *
 */
static int32_t seix006_snapshot_config()
{
	int32_t ret;

	CDBG("seix006_snapshot_config [S]\n");

	/* Change to capture mode */
	ret = seix006_set_regs(seix006_mode_capture_YUV);
	if (ret) {
		CDBG("snapshot_config: set_regs failed\n");
		return ret;
	}

	/* wait MODESEL_FIX to 2 */
	ret = seix006_check_msts(SEIX006_MSTS_CAP_VAL, SEIX006_POLLING_TIMES);
	if(ret) {
		CDBG	("seix006_snapshot_config seix006_check_msts failed\n");
		/* Continue silently */
	}

	is_capture_started = 1;

	CDBG("seix006_snapshot_config [E] ret[%d]\n", ret);
	seix006_ctrl->dev_mode = CAMERA_MODE_CAPTURE;
	return ret;
}


/**
 * Half release config
 *
 */
static int32_t seix006_half_release_config()
{
	int32_t ret = 0;
	int onoff = 0;
	uint8_t datawb = 0;

	CDBG("seix006_half_release_config [S]\n");
	/**
	* Only Flash LED related code, runs only if CFG_GET_AF_ASSIST_LIGHT was invoked
	*/
	if (seix006_ctrl->autoflash_used) {
		onoff = seix006_ctrl->autoflash_assist_light_on;

		ret = autoflash_enable(onoff);
		if(ret) {
			CDBG("autoflash_enable(%d) failed, ret: %d", onoff, ret);
		}

		ret = autoflash_strobe(onoff);
		if (ret) {
			CDBG("autoflash_strobe(%d) failed, ret: %d", onoff, ret);
		}
		/* if flash needed, then AF polling will continue sequence, set flag to TRUE */
		if (onoff) {
			CDBG("autoflash_assist_light_on, set autoflash_poll_reg_x2AA to TRUE!\n");

			seix006_ctrl->autoflash_poll_reg_x2AA = TRUE;
			seix006_ctrl->aeawb_timeout = jiffies + 2 * HZ;
		}
	} else {
		seix006_ctrl->autoflash_poll_reg_x2AA = FALSE;

		ret = autoflash_enable(FALSE);
		if (ret) {
			CDBG("autoflash_enable(OFF) failed, ret: %d", ret);
		}
	}

	switch (seix006_ctrl->scene) {
	case SENSOR_SCENE_TWILIGHT:
	case SENSOR_SCENE_TWILIGHT_PORTRAIT:
		if (seix006_ctrl->autoflash_assist_light_on)
					ret = setix006_set_led_state(MSM_CAMERA_LED_HIGH);

		ret = seix006_set_regs(seix006_hr_twilight);
		if (ret) {
			CDBG("seix006_hr_twilight failed\n");
			return ret;
		}
		break;
	case SENSOR_SCENE_AUTO:
	case SENSOR_SCENE_SPORTS:
	case SENSOR_SCENE_BEACH:
	case SENSOR_SCENE_SNOW:
	case SENSOR_SCENE_LANDSCAPE:
	case SENSOR_SCENE_PORTRAIT:
		CDBG("seix006_half_release_config focus %d\n",
				seix006_ctrl->focus_mode);

		if (seix006_ctrl->focus_mode == SENSOR_FOCUS_MODE_FIXED) {
			if (seix006_ctrl->autoflash_assist_light_on) {
				ret = setix006_set_led_state(MSM_CAMERA_LED_HIGH);
				mdelay(1500);
			}
			CDBG("seix006_half_relase_config SENSOR_FOCUS_MODE_FIXED [E]\n");
			datawb = 0x00;
			seix006_i2c_write(0x6D77, BYTE_1, &datawb);
			return ret;
		}

		if (seix006_ctrl->autoflash_assist_light_on) {
			ret = setix006_set_led_state(MSM_CAMERA_LED_HIGH);
			ret |= seix006_i2c_read(0x0102, BYTE_1, &datawb);
			if (datawb == 0x20) {
				datawb = 0;
				ret |= seix006_i2c_write(0x0102, BYTE_1, &datawb);
			}
			ret |= seix006_set_regs(seix006_hr_LED);
			if (ret) {
				CDBG("seix006_hr_LED failed\n");
				return ret;
			}
		} else {
			ret = seix006_set_regs(seix006_hr_auto_start);
			if (ret) {
				CDBG("seix006_hr_auto_start failed\n");
				return ret;
			}
		}
		break;
	default:
		ret = seix006_set_regs(seix006_mode_half_release);
		if(ret) {
			CDBG("seix006_half_release_config\n");
			return ret;
		}
		break;
	}

	/* wait MODESEL_FIX to 1 */
	ret = seix006_check_msts(SEIX006_MSTS_HR_VAL, SEIX006_POLLING_TIMES);
	if(ret) {
		CDBG("seix006_half_release_config seix006_check_msts failed\n");
		return ret;
	}

	seix006_ctrl->dev_mode = CAMERA_MODE_HALF_RELEASE;

	CDBG("seix006_half_relase_config [E]\n");

	return ret;
}

/**
 * Get AF status
 *
 */
static int32_t seix006_get_af_status(enum camera_af_status* status)
{
	int8_t data = -1;
	uint8_t datawb = 0;
	int32_t ret;

	CDBG("seix006_get_af_status [S]\n");

	*status = SENSOR_AF_IN_PROGRESS;

	if (seix006_ctrl->autoflash_poll_reg_x2AA) {

		if (time_before(jiffies, seix006_ctrl->aeawb_timeout)) {

			ret = autoflash_adjust();

			return ret;

		} else {
			CDBG(".. seix006_get_af_status: TIMED OUT waiting for AWB/AE, do AF status poll only!");
		}
	}

	ret = seix006_i2c_read(0x6D77, 1, &data);
	if(ret) {
		CDBG("seix006_get_af_status failed\n");
	} else {
		if (data != 0x02){
			switch (seix006_ctrl->scene) {
			case SENSOR_SCENE_TWILIGHT:
			case SENSOR_SCENE_TWILIGHT_PORTRAIT:
				ret = setix006_set_led_state(MSM_CAMERA_LED_OFF);
				ret |= seix006_i2c_read(0x0102, BYTE_1, &datawb);
				if (datawb == 0x20) {
					datawb = 0;
					ret |= seix006_i2c_write(0x0102, BYTE_1, &datawb);
				}
				ret |= seix006_set_regs(seix006_hr_reset);
				if(ret) {
						CDBG("seix006_hr_reset failed\n");
						return ret;
				}
				break;
			case SENSOR_SCENE_AUTO:
			case SENSOR_SCENE_SPORTS:
			case SENSOR_SCENE_BEACH:
			case SENSOR_SCENE_SNOW:
			case SENSOR_SCENE_PORTRAIT:
				if (seix006_ctrl->autoflash_assist_light_on) {
					ret = setix006_set_led_state(MSM_CAMERA_LED_OFF);
					ret |= seix006_set_regs(seix006_hr_LED_reset);
					if (ret) {
						CDBG("seix006_hr_LED_reset failed\n");
						return ret;
					}
				} else {
					ret = seix006_set_regs(seix006_hr_auto_reset);
					if (ret) {
						CDBG("seix006_hr__auto_reset failed\n");
						return ret;
					}
				}
				break;
			default:
				CDBG("seix006_hr_release already set\n");
			}
		}

		switch (data) {
		case 0x00:
		default:
			*status = SENSOR_AF_FAILED;
			CDBG("seix006_get_af_status : SENSOR_AF_FAILED\n");
			break;
		case 0x01:
			CDBG("seix006_get_af_status : SENSOR_AF_SUCCESS\n");
			*status = SENSOR_AF_SUCCESS;
			break;
		case 0x02:
			CDBG("seix006_get_af_status : SENSOR_AF_IN_PROGRESS\n");
			*status = SENSOR_AF_IN_PROGRESS;
			break;
		}
	}

	CDBG("seix006_get_af_status [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Get EXIF data
 *
 */
static int32_t seix006_get_exif(struct cam_ctrl_exif_params_t *exif)
{
	int32_t ret = 0;
	int8_t data_8 = -1;
	uint16_t data_16_low = -1;
	uint16_t data_16_high = -1;

	CDBG("seix006_get_exif [S]\n");

	ret = seix006_i2c_read(0x00F0, BYTE_1, &data_8);
	if(ret) {
		CDBG("seix006_get_exif failed\n");
		return ret;
	}

	exif->iso_speed_index = data_8;

	ret = seix006_i2c_read(0x00F2, BYTE_2, (int8_t*)&data_16_low);
	if(ret) {
		CDBG("seix006_get_exif failed\n");
		return ret;
	}

	ret = seix006_i2c_read(0x00F4, BYTE_2, (int8_t*)&data_16_high);
	if(ret) {
		CDBG("seix006_get_exif failed\n");
		return ret;
	}

	CDBG("data_16_low %d\n", data_16_low);
	CDBG("data_16_high %d\n", data_16_high);

	exif->shutter_speed = (data_16_high << SHIFT_16) | data_16_low;
	CDBG("Shutter speed %d us\n", exif->shutter_speed);

	exif->camera_revision = seix006_ctrl->camera_revision;
	CDBG("Camera revision %d\n", exif->camera_revision);

	exif->flash_fired = seix006_ctrl->autoflash_assist_light_on;
	CDBG("Flash fired %d\n", exif->flash_fired);

	CDBG("seix006_get_exif [X]\n");
	return ret;
}

/**
 * Set test pattern on/off
 *
 */
static int32_t seix006_set_test_pattern(enum set_test_pattern_t mode)
{
	int32_t ret;

	CDBG("seix006_set_test_pattern [S]\n");

	if (mode == TEST_PATTERN_ON) {
		ret = seix006_set_regs(seix006_test_pattern_on);
	} else {
		ret = seix006_set_regs(seix006_test_pattern_off);
	}

	if (ret) {
		CDBG("seix006_set_test_pattern send_reg_table failed\n");
		return ret;
	}

	CDBG("seix006_set_test_pattern [E]\n");

	return ret;
}

/**
 * Set ISO mode
 *
 */
static int32_t seix006_set_iso(uint16_t iso_mode)
{
	int32_t ret = 0;
    
    CDBG("seix006_set_iso [S] iso=%d\n", iso_mode);

	/* iso_mode is set to the actual ISO value, e.g. 200, 800 ... */
	/* if 0 or any value not supported should set the sensor to AUTO (0 is used in user space to represent AUTO) */

	switch (iso_mode) {
	case 100:
	  ret = seix006_set_regs(seix006_iso_100);
	break;
	case 200:
	  ret = seix006_set_regs(seix006_iso_200);
	break;
	case 400:
	  ret = seix006_set_regs(seix006_iso_400);
	break;
	case 800:
	  ret = seix006_set_regs(seix006_iso_800);
	break;
	case 1600:
	  ret = seix006_set_regs(seix006_iso_1600);
	break;
	case 0:
	default:
	  ret = seix006_set_regs(seix006_iso_auto);
	break;
	}

	CDBG("seix006_set_iso [E]\n");
	return ret;
}

static int32_t seix006_update_scan_range(enum camera_focus_mode focus_mode,
		enum camera_scene scene)
{

	int32_t ret = 0;
	int update_register = 0;

	CDBG("seix006_update_scan_range [S]\n");

	switch (scene) {
	case SENSOR_SCENE_AUTO:
	case SENSOR_SCENE_BEACH:
	case SENSOR_SCENE_SNOW:
	case SENSOR_SCENE_LANDSCAPE:
	case SENSOR_SCENE_PORTRAIT:
	case SENSOR_SCENE_DOCUMENT:
		if (seix006_ctrl->scan_range_reg != SEIX006_SCAN_RANGE_REG_AUTO) {
			seix006_ctrl->scan_range_reg = SEIX006_SCAN_RANGE_REG_AUTO;
			update_register = 1;
		}
	break;
	case SENSOR_SCENE_TWILIGHT:
	case SENSOR_SCENE_TWILIGHT_PORTRAIT:
		if (seix006_ctrl->scan_range_reg != SEIX006_SCAN_RANGE_REG_TWILIGHT) {
			seix006_ctrl->scan_range_reg = SEIX006_SCAN_RANGE_REG_TWILIGHT;
			update_register = 1;
		}
	break;
	case SENSOR_SCENE_SPORTS:
		if (seix006_ctrl->scan_range_reg != SEIX006_SCAN_RANGE_REG_SPORTS) {
			seix006_ctrl->scan_range_reg = SEIX006_SCAN_RANGE_REG_SPORTS;
			update_register = 1;
		}
	break;
	default:
		CDBG("seix006_update_scan_range scene %d not supported.\n", scene);
	break;
	}

	if (focus_mode == SENSOR_FOCUS_MODE_MACRO) {
		if (seix006_ctrl->scan_range_val != SEIX006_SCAN_RANGE_MACRO) {
			seix006_ctrl->scan_range_val = SEIX006_SCAN_RANGE_MACRO;
			update_register = 1;
		}
	} else {
		if (seix006_ctrl->scan_range_val != SEIX006_SCAN_RANGE_AUTO) {
			seix006_ctrl->scan_range_val = SEIX006_SCAN_RANGE_AUTO;
			update_register = 1;
		}
	}

	if (update_register == 1) {
		CDBG("seix006_update_scan_range setting reg %d to value %d\n",
				seix006_ctrl->scan_range_reg, seix006_ctrl->scan_range_val);
		ret = seix006_i2c_write(seix006_ctrl->scan_range_reg, BYTE_2,
				(uint8_t *) &seix006_ctrl->scan_range_val);
	}

	CDBG("seix006_update_scan_range [E]\n");

	return ret;
}

/**
 * Set focus mode
 *
 */
static int32_t seix006_set_focus_mode(enum camera_focus_mode focus_mode)
{
	int32_t ret = 0;

	CDBG("seix006_set_focus_mode [S]\n");

	if (seix006_ctrl->focus_mode != focus_mode) {
		seix006_ctrl->focus_mode = focus_mode;

		if (seix006_ctrl->init_complete)
			ret = seix006_update_focus_mode(focus_mode);
	}
	CDBG("seix006_set_focus_mode [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Update focus_mode
 *
 */
static int32_t seix006_update_focus_mode(enum camera_focus_mode focus_mode)
{
	int32_t ret = 0;

	CDBG("seix006_update_focus_mode [S]\n");

	switch (focus_mode) {
	case SENSOR_FOCUS_MODE_AUTO:
	case SENSOR_FOCUS_MODE_MACRO:
	default:
		CDBG("seix006_update_focus_mode setting focus mode auto\n");
		ret = seix006_set_regs(seix006_focus_mode_auto);
		if (!ret) {
			ret = seix006_set_regs(seix006_primary_focus_window_auto);
			CDBG("seix006_update_focus_mode setting focus window auto\n");
		}
		if (seix006_ctrl->af_4838_val !=
				seix006_ctrl->calibration_data.af_d * 4) {
			seix006_ctrl->af_4838_val = seix006_ctrl->calibration_data.af_d * 4;
			seix006_i2c_write(SEIX006_AF_4838, BYTE_2,
					(uint8_t *) &seix006_ctrl->af_4838_val);
		}
		camera_continous_autofocus = 0;
	break;

	case SENSOR_FOCUS_MODE_CONTINUOUS:
		CDBG("seix006_update_focus_mode setting focus mode continuous\n");
		ret = seix006_set_regs(seix006_focus_mode_continuous);
		if (!ret) {
			ret = seix006_set_regs(seix006_primary_focus_window_continuous);
			CDBG("seix006_update_focus_mode setting focus window continuous\n");
			camera_continous_autofocus = 1;
		}
		if (seix006_ctrl->af_4838_val !=
				seix006_ctrl->calibration_data.af_d * 2) {
			seix006_ctrl->af_4838_val = seix006_ctrl->calibration_data.af_d * 2;
			seix006_i2c_write(SEIX006_AF_4838, BYTE_2,
					(uint8_t *) &seix006_ctrl->af_4838_val);
		}
	break;

	case SENSOR_FOCUS_MODE_FIXED:
		CDBG("seix006_update_focus_mode focus mode not supported by camera\n");
	break;
	}

	seix006_update_scan_range(focus_mode, seix006_ctrl->scene);

	CDBG("seix006_update_focus_mode [E]\n");

	return ret;
}

/**
 * Set scene
 *
 */
static int32_t seix006_set_scene(enum camera_scene scene)
{
	int32_t ret = 0;

	CDBG("seix006_set_scene [S]\n");

	if (seix006_ctrl->scene != scene) {
		seix006_ctrl->scene = scene;
		if (seix006_ctrl->init_complete) {
			ret = seix006_update_scene(scene);
		}
	}

	CDBG("seix006_set_scene [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Update scene
 *
 */
static int32_t seix006_update_scene(enum camera_scene scene)
{
	int32_t ret = 0;

	CDBG("seix006_update_scene [S]\n");

	CDBG("seix006_update_scene Requested scene = %d\n",scene);
	switch (scene) {
	case SENSOR_SCENE_AUTO:
	default:
		CDBG("seix006 Setting normal scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_normal);
		break;
	case SENSOR_SCENE_MACRO:
		CDBG("seix006 Setting macro scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_macro);
		break;
	case SENSOR_SCENE_TWILIGHT:
		CDBG("seix006 Setting twilight landscape scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_twilight);
		break;
	case SENSOR_SCENE_SPORTS:
		CDBG("seix006 Setting sports scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_sports);
		break;
	case SENSOR_SCENE_BEACH:
	case SENSOR_SCENE_SNOW:
		CDBG("seix006 Setting beach/snow scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_beach_and_snow);
		break;
	case SENSOR_SCENE_LANDSCAPE:
		CDBG("seix006 Setting landscape scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_landscape);
		break;
	case SENSOR_SCENE_PORTRAIT:
		CDBG("seix006 Setting portrait scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_portrait);
		break;
	case SENSOR_SCENE_TWILIGHT_PORTRAIT:
		CDBG("seix006 Setting twilight portrait scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_twilight_portrait);
		break;
	case SENSOR_SCENE_DOCUMENT:
		CDBG("seix006 Setting document scene mode\n");
		ret = seix006_set_regs(seix006_GEN_scene_document);
		break;
	}

	CDBG("seix006_update_scene [E] ret[%d]\n", ret);

	return ret;
}

static int seix006_cam_is_assist_light_needed(int *result)
{
	int32_t ret = 0;
	uint16_t reg_x288, reg_x26A, reg_x26C;
	int16_t reg_x284;
	int A;

	CDBG("seix006_cam_is_assist_light_needed [S]\n");

	ret = seix006_i2c_read(0x288, BYTE_2, (int8_t *) & reg_x288);
	if (!ret)
		ret = seix006_i2c_read(0x284, BYTE_2, (int8_t *) & reg_x284);
	if (!ret)
		ret = seix006_i2c_read(0x26A, BYTE_2, (int8_t *) & reg_x26A);
	if (!ret)
		ret = seix006_i2c_read(0x26C, BYTE_2, (int8_t *) & reg_x26C);

	CDBG("seix006_cam_is_assist_light_needed, reg 0x288: %d, 0x284: %d, "
		"0x26A: %d, 0x26C: %d\n", reg_x288, reg_x284, reg_x26A, reg_x26C);

	if (ret) {
		CDBG("seix006_cam_is_assist_light_needed failed, result=%d\n", ret);
		return ret;
	}

	A = reg_x288 + reg_x284;
	CDBG("seix006_cam_is_assist_light_needed: A = (reg x288 + reg x284) = %d\n", A);

	seix006_ctrl->autoflash_used = TRUE;
	seix006_ctrl->autoflash_assist_light_on = camera_flash == FLASH_AUTO ? (A < SEIX006_FLASH_NEEDED_AE_MIN) : camera_flash;
	seix006_ctrl->autoflash_reg_x288 = reg_x288;
	seix006_ctrl->autoflash_reg_x284 = reg_x284;
	seix006_ctrl->autoflash_reg_x26A = reg_x26A;
	seix006_ctrl->autoflash_reg_x26C = reg_x26C;

	/* return TRUE or FALSE */
	*result = seix006_ctrl->autoflash_assist_light_on;

	CDBG("seix006_cam_is_assist_light_needed: %s, [E] ret[%d]\n", (*result ? "TRUE" : "FALSE"), ret);

	return ret;
}

static int32_t seix006_set_preview_dimension(struct camera_preview_dimension_t dimension)
{
	int32_t ret = 0;
	uint16_t current_width = 0;
	CDBG("seix006_set_preview_dimension [S]\n");

	seix006_i2c_read(0x0022, BYTE_2, (uint8_t *) &current_width);

	if (dimension.sensor_width == 640 && dimension.sensor_height == 480) {
		if (current_width == 640) {
			CDBG("VGA preview size already set\n");
		} else {
			CDBG("Setting VGA preview size\n");
			ret = seix006_send_reg_table(
					seix006_vf_resolution_640x480,
					sizeof_seix006_vf_resolution_640x480
					/ sizeof(struct reg_entry));
			if (ret)
				CDBG("Setting VGA preview size failed\n");
			else
				ret = seix006_refresh_monitor(SEIX006_POLLING_TIMES);
		}
	} else if (dimension.sensor_width == 800 &&
			dimension.sensor_height == 480) {
		if (current_width == 800) {
			CDBG("WVGA preview size already set\n");
		} else {
			CDBG("Setting WVGA preview size\n");
			ret = seix006_send_reg_table(
					seix006_vf_resolution_800x480,
					sizeof_seix006_vf_resolution_800x480
					/ sizeof(struct reg_entry));
			if (ret)
				CDBG("Setting WVGA preview size failed\n");
			else
				ret = seix006_refresh_monitor(SEIX006_POLLING_TIMES);
		}
	} else if (dimension.sensor_width == 1280 &&
				dimension.sensor_height == 720) {
		if (current_width == 1280) {
			CDBG("HD720 preview size already set\n");
		} else {
			CDBG("Setting HD720 preview size 1280x720\n");
			ret = seix006_send_reg_table(
					seix006_vf_resolution_1280x720,
					sizeof_seix006_vf_resolution_1280x720
					/ sizeof(struct reg_entry));
			if (ret)
				CDBG("Setting HD720 preview size failed\n");
			else
				ret = seix006_refresh_monitor(
						SEIX006_POLLING_TIMES);
		}
	}

	seix006_i2c_read(0x0022, BYTE_2, (int8_t *) &current_width);

	CDBG("seix006_set_preview_dimension New sensor output is %dx%d",
			current_width, (current_width == 1280 ? 720 : 480));

	CDBG("seix006_set_preview_dimension [E] ret[%d]\n", ret);

	return ret;
}

static int32_t seix006_set_dimensions(struct camera_dimension_t dimension)
{
	int32_t ret = 0;

	CDBG("seix006_set_dimensions [S]\n");

	if (dimension.picture_width == 640 && dimension.picture_height == 480) {
		CDBG("Setting VGA snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_640x480,
			sizeof_seix006_snapshot_resolution_640x480
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 800 && dimension.picture_height == 480) {
		CDBG("Setting WVGA snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_800x480,
			sizeof_seix006_snapshot_resolution_800x480
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 176 && dimension.picture_height == 144) {
		CDBG("Setting 176x144 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_176x144,
			sizeof_seix006_snapshot_resolution_176x144
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 320 && dimension.picture_height == 240) {
		CDBG("Setting 320x240 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_320x240,
			sizeof_seix006_snapshot_resolution_320x240
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 352 && dimension.picture_height == 288) {
		CDBG("Setting 352x288 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_352x288,
			sizeof_seix006_snapshot_resolution_352x288
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 800 && dimension.picture_height == 600) {
		CDBG("Setting 800x600 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_800x600,
			sizeof_seix006_snapshot_resolution_800x600
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 1024 && dimension.picture_height == 768) {
		CDBG("Setting 1024x768 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_1024x768,
			sizeof_seix006_snapshot_resolution_1024x768
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 1280 && dimension.picture_height == 720) {
		CDBG("Setting 1280x720 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_1280x720,
			sizeof_seix006_snapshot_resolution_1280x720
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 1280 && dimension.picture_height == 960) {
		CDBG("Setting 1280x960 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_1280x960,
			sizeof_seix006_snapshot_resolution_1280x960
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 1632 && dimension.picture_height == 1224) {
		CDBG("Setting 1632x1224 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_1632x1224,
			sizeof_seix006_snapshot_resolution_1632x1224
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 1280 && dimension.picture_height == 768) {
		CDBG("Setting 1280x768 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_1280x768,
			sizeof_seix006_snapshot_resolution_1280x768
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 1600 && dimension.picture_height == 1200) {
		CDBG("Setting 1600x1200 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_1600x1200,
			sizeof_seix006_snapshot_resolution_1600x1200
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 1920 && dimension.picture_height == 1080) {
		CDBG("Setting 1920x1080 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_1920x1080,
			sizeof_seix006_snapshot_resolution_1920x1080
				/ sizeof(struct reg_entry));
	} else if (dimension.picture_width == 2048 && dimension.picture_height == 1536) {
		CDBG("Setting 2048x1536 snapshot size\n");
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_2048x1536,
			sizeof_seix006_snapshot_resolution_2048x1536
				/ sizeof(struct reg_entry));
	} else {
		CDBG("Setting 5MP snapshot size, input %d x %d\n", dimension.picture_width, dimension.picture_height);
		ret = seix006_send_reg_table(
			seix006_snapshot_resolution_2592x1944,
			sizeof_seix006_snapshot_resolution_2592x1944
				/ sizeof(struct reg_entry));
	}
	if (dimension.thumbnail_width == 800 && dimension.thumbnail_height == 480) {
		CDBG("Setting WVGA thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_WVGA,
			sizeof_seix006_thumbnail_size_WVGA
				/ sizeof(struct reg_entry));
	} else if (dimension.thumbnail_width == 640 && dimension.thumbnail_height == 480) {
		CDBG("Setting VGA thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_VGA,
			sizeof_seix006_thumbnail_size_VGA
				/ sizeof(struct reg_entry));
	} else if (dimension.thumbnail_width == 176 && dimension.thumbnail_height == 144) {
		CDBG("Setting QCIF thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_QCIF,
			sizeof_seix006_thumbnail_size_QCIF
				/ sizeof(struct reg_entry));
	}

	else if (dimension.thumbnail_width == 512 && dimension.thumbnail_height == 384) {
		CDBG("Setting 512x384 thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_512x384,
			sizeof_seix006_thumbnail_size_512x384
				/ sizeof(struct reg_entry));
	} else if (dimension.thumbnail_width == 512 && dimension.thumbnail_height == 288) {
		CDBG("Setting 512x288 thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_512x288,
			sizeof_seix006_thumbnail_size_512x288
				/ sizeof(struct reg_entry));
	} else if (dimension.thumbnail_width == 480 && dimension.thumbnail_height == 288) {
		CDBG("Setting 480x288 thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_480x288,
			sizeof_seix006_thumbnail_size_480x288
				/ sizeof(struct reg_entry));
	} else if (dimension.thumbnail_width == 432 && dimension.thumbnail_height == 288) {
		CDBG("Setting 432x288 thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_432x288,
			sizeof_seix006_thumbnail_size_432x288
				/ sizeof(struct reg_entry));
	} else if (dimension.thumbnail_width == 352 && dimension.thumbnail_height == 288) {
		CDBG("Setting 352x288 thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_352x288,
			sizeof_seix006_thumbnail_size_352x288
				/ sizeof(struct reg_entry));
	} else if (dimension.thumbnail_width == 400 && dimension.thumbnail_height == 240) {
		CDBG("Setting WQVGA thumbnail size\n");
		ret = seix006_send_reg_table(seix006_thumbnail_size_WQVGA,
			sizeof_seix006_thumbnail_size_WQVGA
				/ sizeof(struct reg_entry));
	} else {
		CDBG("Setting QVGA thumbnail size, input %d x %d\n", dimension.thumbnail_width, dimension.thumbnail_height);
		ret = seix006_send_reg_table(seix006_thumbnail_size_QVGA,
			sizeof_seix006_thumbnail_size_QVGA
				/ sizeof(struct reg_entry));
	}

	CDBG("seix006_set_dimensions [E] ret[%d]\n", ret);

	return ret;
}

static int32_t seix006_set_framerate(uint16_t fps)
{
	int32_t ret = 0;
	uint8_t register_value = 0x0;
	CDBG("seix006_set_framerate fps=%d [S]\n", fps);

	/* Block reset of focus on state change */
	if (camera_continous_autofocus == 1) {
		seix006_i2c_write(0x4884, BYTE_1, &register_value);
		CDBG("seix006_set_framerate block refocus");
	}

	if (fps == 0) {
		CDBG("seix006_set_framerate set variable");
		camera_variable_frame_rate  = 1;
		ret = seix006_set_regs(seix006_framerate_variable);
	} else if (fps == 30) {
		CDBG("seix006_set_framerate set 30");
		camera_variable_frame_rate  = 0;
		ret = seix006_set_regs(seix006_framerate_30);
		ret |= seix006_set_regs(seix006_framerate_fixed);
	} else if (fps == 15) {
		CDBG("seix006_set_framerate set 15");
		camera_variable_frame_rate  = 0;
		ret = seix006_set_regs(seix006_framerate_15);
		ret |= seix006_set_regs(seix006_framerate_fixed);
	} else {
		CDBG("seix006_set_framerate error, %d fps not supported by camera\n", fps);
		ret = -EFAULT;
	}

	/* Start and wait for state change */
	CDBG("seix006_set_framerate restart");
	seix006_refresh_monitor(1000);

	/* Restore reset focus setting on state change */
	register_value = 0x1;
	seix006_i2c_write(0x4884, BYTE_1, &register_value);

	CDBG("seix006_set_framerate [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Access GPIO
*/
static int32_t seix006_gpio_access(int gpio_pin, int dir)
{
	int rc = 0;

	CDBG("seix006_gpio_access [S]\n");

	rc = gpio_request(gpio_pin, "seix006_camera");
	if (!rc) {
		gpio_direction_output(gpio_pin, dir);
	}
	gpio_free(gpio_pin);

	CDBG("seix006_gpio_access [E] rc[%d]\n", rc);

	return rc;
}

/* Robyn specific functions, also dependant on addition to board.h, not necessary for Zeus at the moment */
#ifndef USE_ZEUS_POWER_MANAGEMENT
/**
 * Enable a resource (GPIO or VREG)
*/
static int32_t seix006_resource_enable(struct msm_camera_sensor_pwr *resource)
{
	int32_t ret;

	if (!resource) {
		CDBG("seix006_resource_enable argument is NULL.\n");
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		CDBG("seix006_resource_enable GPIO[%d]\n", resource->resource.number);
		ret = seix006_gpio_access(resource->resource.number, TRUE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		CDBG("seix006_resource_enable VREG[%s]\n", resource->resource.name);
		ret = vreg_enable(vreg_get(0, resource->resource.name));
		break;
	default:
		CDBG("seix006_resource_enable invalid resource type[%d]\n", resource->type);
		ret = 1;
		break;
	}

	return ret;
}

/**
 * Disable a resource (GPIO or VREG)
*/
static int32_t seix006_resource_disable(struct msm_camera_sensor_pwr *resource)
{
	int32_t ret;

	if (!resource) {
		CDBG("seix006_resource_disable argument is NULL.\n");
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		CDBG("seix006_resource_disable GPIO[%d]\n", resource->resource.number);
		ret = seix006_gpio_access(resource->resource.number, FALSE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		CDBG("seix006_resource_disable VREG[%s]\n", resource->resource.name);
		ret = vreg_disable(vreg_get(0, resource->resource.name));
		break;
	default:
		CDBG("seix006_resource_disable invalid resource type[%d]\n", resource->type);
		ret = 1;
		break;
	}

	return ret;
}
#endif /* USE_ZEUS_POWER_MANAGEMENT */

/**
 * Power on sensor
 *
 */
static int32_t seix006_sensor_on(void)
{
#ifdef USE_ZEUS_POWER_MANAGEMENT
	CDBG("seix006_sensor_on [S]\n");

	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);

	/* Output CAM_MCLK(19.2MHz) */
	msm_camio_clk_rate_set(SEIX006_DEFAULT_CLOCK_RATE);
	CDBG("seix006_sensor_on [E]\n");
	return 0;
#else
	int32_t ret = 0;

	CDBG("seix006_sensor_on [S]\n");

	/* Power on VCAM_SD12(GPI117 = High) 1,2V Core */
	ret = seix006_resource_enable(&seix006_ctrl->sensordata->vcam_sd12);
	if (ret) {
		CDBG("seix006_sensor_on Power on VCAM_SD12 failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on VCAM_IO(PM7540/REG_GP4) 2,6V */
	ret = seix006_resource_enable(&seix006_ctrl->sensordata->vcam_io);
	if (ret) {
		CDBG("seix006_sensor_on Power on VCAM_IO failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on VCAM_SA28(PM7540/RFRX2) */
	ret = seix006_resource_enable(&seix006_ctrl->sensordata->vcam_sa28);
	if (ret) {
		CDBG("seix006_sensor_on Power on VCAM_SA28 failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on AF(PM7540/RFTX) */
	ret = seix006_resource_enable(&seix006_ctrl->sensordata->vcam_af30);
	if (ret) {
		CDBG("seix006_sensor_on Power on VCAM_AF failed\n");
		return ret;
	}

	mdelay(5);

	CDBG("seix006_sensor_on [E] ret[%d]\n", ret);

	return ret;
#endif /* USE_ZEUS_POWER_MANAGEMENT */
}

/**
 * Initialize sensor
 *
 */
static int32_t seix006_sensor_init(void)
{
	int32_t ret = 0;

	CDBG("seix006_sensor_init [S] %d\n",seix006_ctrl->camera_revision);

	if (seix006_ctrl->camera_revision == SEIX006_CAM_REV_ES1) {
		ret = seix006_set_regs(seix006_GEN_period_1_ES1);
		if (ret) {
			CDBG("seix006_set_regs seix006_GEN_Period_1_ES1 failed\n");
		}
	} else if (seix006_ctrl->camera_revision == SEIX006_CAM_REV_ES2) {
		ret = seix006_set_regs(seix006_GEN_period_1_ES2);
		mdelay(5);
		if (ret) {
			CDBG("seix006_set_regs seix006_GEN_Period_1_ES2 failed\n");
		}
		ret = seix006_write_calibration_data(seix006_ctrl->calibration_data);
		if (ret) {
			CDBG("seix006_write_calibration_data failed\n");
		}
	}

	seix006_ctrl->dev_mode = CAMERA_MODE_MONITOR;
	seix006_ctrl->scene = SENSOR_SCENE_AUTO;
	seix006_ctrl->focus_mode = SENSOR_FOCUS_MODE_AUTO;
	seix006_ctrl->init_complete = 0;
	seix006_ctrl->af_4838_val = seix006_ctrl->calibration_data.af_l;
	seix006_ctrl->scan_range_reg = SEIX006_SCAN_RANGE_REG_AUTO;
	seix006_ctrl->scan_range_val = SEIX006_SCAN_RANGE_AUTO;

	/* set continous autofocus flag to off by default */
	camera_continous_autofocus = 0;

	CDBG("seix006_sensor_init [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Power off sensor
 *
 */
static void seix006_sensor_off(void)
{
#ifdef USE_ZEUS_POWER_MANAGEMENT
	int32_t ret = 0;

	CDBG("seix006_sensor_off [S]\n");

	/* Power off = LOW */
	ret = seix006_gpio_access(seix006_ctrl->sensordata->sensor_pwd, 0);
	if(ret) {
		CDBG("seix006_sensor_off Power off STANDBY failed\n");
	}

	mdelay(5);

	/* CAM_RESET_N = LOW */
	ret = seix006_gpio_access(seix006_ctrl->sensordata->sensor_reset, 0);
	if(ret) {
		CDBG("seix006_sensor_off CAM_RESET_N release failed\n");
	}

	mdelay(20);

	/* Output CAM_MCLK(0MHz) */
	msm_camio_clk_rate_set(0);

	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);

	mdelay(5);

	CDBG("seix006_sensor_off [E]\n");
#else
	int32_t ret = 0;

	CDBG("seix006_sensor_off [S]\n");

	/* Power off STANDBY (GPIO2 = LOW) */
	ret = seix006_resource_disable(&seix006_ctrl->sensordata->standby);
	if (ret) {
		CDBG("seix006_sensor_off Power off STANDBY failed\n");
}

	mdelay(5);

	/* CAM_RESET_N release(GPIO0 = LOW) */
	ret = seix006_gpio_access(seix006_ctrl->sensordata->sensor_reset, FALSE);
	if (ret) {
		CDBG("seix006_sensor_off CAM_RESET_N release failed\n");
	}

	msleep(20);

	/* Output CAM_MCLK(0MHz) */
	msm_camio_clk_rate_set(0);

	msm_camio_clk_disable(CAMIO_VFE_CLK);

	mdelay(5);

	/* Power off VCAM_AF */
	ret = seix006_resource_disable(&seix006_ctrl->sensordata->vcam_af30);
	if (ret) {
		CDBG("seix006_sensor_off Power off VCAM_AF failed\n");
	}
	mdelay(5);
	/* Power off VCAM_L2(GPIO43 = LOW) */
	ret = seix006_resource_disable(&seix006_ctrl->sensordata->vcam_sa28);
	if (ret) {
		CDBG("seix006_sensor_off Power off VCAM_L2 failed\n");
	}
	msleep(250);
	/* Power off VCAM_IO(PM7540/REG_RFRX2) */
	ret = seix006_resource_disable(&seix006_ctrl->sensordata->vcam_io);
	if (ret) {
		CDBG("seix006_sensor_off Power off VCAM_IO failed\n");
	}

	msleep(150);

	/* Power off VCAM_SD(GPIO142 = LOW) */
	ret = seix006_resource_disable(&seix006_ctrl->sensordata->vcam_sd12);
	if (ret) {
		CDBG("seix006_sensor_off Power off VCAM_SD failed\n");
	}

	CDBG("seix006_sensor_off [E]\n");
#endif /* USE_ZEUS_POWER_MANAGEMENT */
}

/**
 * Open Processing.
 *
 */
static int seix006_sensor_open(const struct msm_camera_sensor_info *data)
{
#ifdef USE_ZEUS_POWER_MANAGEMENT
	int32_t ret = 0;

	CDBG("%s <--\n", __FUNCTION__);

	down(&seix006_sem);

	CDBG("seix006_open [S]\n");

	if (seix006_ctrl->opened)
	{
		CDBG("seix006_open already opened\n");
		ret = 0;
		goto open_done;
	}

	ret = seix006_sensor_on();
	if (ret)
	{
		CDBG("seix006_open sensor_on failed\n");
		goto open_done;
	}

	msm_camio_camif_pad_reg_reset(); /* Moved from vfe_31_init(...) */

	mdelay(40);

	/* STANDBY low then high before reg settings */
	ret = seix006_gpio_access(seix006_ctrl->sensordata->sensor_pwd, 0);
	if(ret)
	{
		CDBG("seix006_sensor_open STANDBY failed\n");
		goto open_done;
	}
	mdelay(5);
	/* CAM_RESET_N release(GPI89 = High) */
	ret = seix006_gpio_access(seix006_ctrl->sensordata->sensor_reset, 1);
	if (ret)
	{
		CDBG("seix006_sensor_open CAM_RESET_N release failed\n");
		goto open_done;
	}

	mdelay(20);

	ret = seix006_sensor_init();
	if (ret) {
		CDBG("seix006_open sensor_init failed\n");
		goto open_done;
	}

	mdelay(5);

	/**
	 * End of Init sensor
	 */

	/* STANDBY (GPIO0 = High) */
	ret = seix006_gpio_access(seix006_ctrl->sensordata->sensor_pwd, 1);
	mdelay(10);
	if(ret)
	{
		CDBG("seix006_sensor_open STANDBY failed\n");
		/* goto open_done; */
	}

open_done:

	if(ret)
	{
		CDBG("seix006_sensor_going to be off\n");
		seix006_sensor_off();
		ret = -EFAULT;
	}
	else
	{
		seix006_ctrl->opened = 1;
	}

	up(&seix006_sem);

	CDBG("%s <-- ret[%d]\n",__FUNCTION__, ret);

	return ret;
#else
	int32_t ret = 0;

	down(&seix006_sem);

	CDBG("seix006_open [S]\n");

	if (seix006_ctrl->opened) {
		CDBG("seix006_open already opened\n");
		ret = 0;
		goto open_done;
	}

	ret = seix006_sensor_on();
	if (ret) {
		CDBG("seix006_open sensor_on failed\n");
		goto open_done;
	}

	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);

	/* Output CAM_MCLK(19.2MHz) */
	msm_camio_clk_rate_set(SEIX006_DEFAULT_CLOCK_RATE);

	msm_camio_camif_pad_reg_reset();

	msleep(40);

	/* CAM_RESET_N release(GPI89 = High) */
	ret = seix006_gpio_access(seix006_ctrl->sensordata->sensor_reset, TRUE);
	if (ret) {
		CDBG("seix006_sensor_open CAM_RESET_N release failed\n");
		goto open_done;
	}

	msleep(20);

	ret = seix006_sensor_init();
	if (ret) {
		CDBG("seix006_open sensor_init failed\n");
		goto open_done;
	}

	mdelay(5);

	/* STANDBY (GPIO0 = High) */
	ret = seix006_resource_enable(&seix006_ctrl->sensordata->standby);
	mdelay(10);
	if (ret) {
		CDBG("seix006_sensor_open STANDBY failed\n");
		goto open_done;
	}

 open_done:

	if (ret) {
		seix006_sensor_off();
		ret = -EFAULT;
	} else {
		seix006_ctrl->opened = 1;
	}

	up(&seix006_sem);

	CDBG("seix006_open [E]\n");

	return ret;
#endif /* USE_ZEUS_POWER_MANAGEMENT */
}

/**
 * Probe Processing.
 *
 */
static int seix006_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	CDBG("seix006_i2c_probe [S]\n");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("seix006_probe i2c_check_functionality failed\n");
		kfree(seix006_ctrl->sensorw);
		seix006_ctrl->sensorw = NULL;

		return -ENOTSUPP;
	}

	seix006_ctrl->sensorw = kzalloc(sizeof(struct seix006_work), GFP_KERNEL);

	if(NULL == seix006_ctrl->sensorw) {
		CDBG("seix006_probe sensorw failed\n");
		kfree(seix006_ctrl->sensorw);
		seix006_ctrl->sensorw = NULL;

		return -ENOMEM;
	}

	i2c_set_clientdata(client, seix006_ctrl->sensorw);

	seix006_ctrl->client = client;
	CDBG("seix006_seix006_ctrl->client->addr %d\n", seix006_ctrl->client->addr);

	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&seix006_wait_queue);

	CDBG("seix006_probe [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Remove Processing.
 *
 */
static int __exit seix006_i2c_remove(struct i2c_client *client)
{
	struct seix006_work_t *sensorw = i2c_get_clientdata(client);

	CDBG("seix006_remove [S]\n");

	free_irq(client->irq, sensorw);
	seix006_ctrl->client = NULL;
	kfree(sensorw);

	CDBG("seix006_remove [E]\n");

	return 0;
}

/**
 * Single or Sequential Write to Random Location.
 *
 */
static int32_t seix006_i2c_write(uint16_t address, uint8_t data_length, const uint8_t * data)
{
	uint8_t i2c_package[SEIX006_I2C_MAX_BYTES + SEIX006_I2C_WRITE_FOOTPRINT];
	uint8_t package_length = 0;

	if (data_length > SEIX006_I2C_MAX_BYTES) {
		CDBG("seix006_i2c_write length[%d] failed\n", data_length);
		return -EFAULT;
	}
	/* Add 2 byte Register Address in switched endian */
	seix006_add_bytes_in_switched_endian(i2c_package, 0, SEIX006_I2C_WRITE_FOOTPRINT,
						(uint8_t *) & address);
	if (data != NULL) {
		memcpy(i2c_package + SEIX006_I2C_WRITE_FOOTPRINT, data, data_length);	/* Add Data */
	}

	package_length = SEIX006_I2C_WRITE_FOOTPRINT + data_length;

	if (i2c_master_send(seix006_ctrl->client, i2c_package, package_length) != package_length) {
		CDBG("seix006_i2c_write i2c_master_send failed\n");
		return -EIO;
	}

	return 0;
}

/**
 * Single or Sequential Read to Random Location.
 *
 */
static int32_t seix006_i2c_read(uint16_t address, uint8_t length, uint8_t * data)
{
	int32_t ret = 0;

	CDBG("seix006_i2c_read [S]\n");

	if (!data) {
		CDBG("seix006_i2c_read *data failed\n");
		return -EFAULT;
	}

	ret = seix006_i2c_write(address, 0, NULL);
	if (ret < 0) {
		CDBG("seix006_i2c_read i2c_write failed\n");
		return ret;
	}

	if (i2c_master_recv(seix006_ctrl->client, data, length) < 0) {
		CDBG("seix006_i2c_read i2c_master_recv failed\n");
		return -EIO;
	}

	CDBG("seix006_i2c_read [E]\n");

	return 0;
}

static int32_t seix006_send_reg_table(const struct reg_entry *table, uint32_t tables_to_send)
{
	int32_t   ret         = 0;
	uint32_t  table_index = 0;
	uint32_t  data_index  = 0;
	uint32_t  j;
	uint8_t   data_buffer[SEIX006_I2C_MAX_BYTES] = {0};
	uint32_t  grouped_data = 0;         /* The amount of continued data to group with one i2c write */
	uint8_t   grouped_tables = 0;

	CDBG("seix006_send_reg_table [tables: %d] [S]\n", tables_to_send);

	if(!table || tables_to_send == 0) {
		CDBG("seix006_send_reg_table *table tables_to_send[%d] failed\n", tables_to_send);
		return -EFAULT;
	}

	/* Will loop thru entire table and send all data, if possible data will be grouped */
	while(table_index < tables_to_send) {
		grouped_data = 0;
		grouped_tables = 0;

		switch(table[table_index].reg_bits) {
			case REG_BITS_8:
			{
				/* Gather the amount of data we can send in one i2c stream, if next address is one incremented
				 the i2c write will continue to write at next address automaticly when sending more data then
				 one register can hold. Only group 8 bits registers since we can't be sure that when writing
				 32 bits register that is is really 32 bits or 4 grouped 8 bits.*/
				while( ((table_index + grouped_tables + 1) < tables_to_send)  &&      /* Only if there are more tables to be found */
					   (grouped_data < (SEIX006_I2C_MAX_BYTES - 1))            && /* Make sure we only send MAX allowed bytes */
					   ((table[table_index + grouped_tables + 1].address)     ==  /* Only if next tables address is */
					   (table[table_index + grouped_tables].address + 1))    &&   /* one incremented address as current */
					   (table[table_index + grouped_tables + 1].reg_bits == REG_BITS_8) /* Only if next table is the same amount of bit holder */
					 ) {
					grouped_data++;
					grouped_tables++;
				}

				/* Load all tables, default to one */
				for(j = 0; j < grouped_tables + 1; j++)
					data_buffer[j] = table[table_index + j].data & LOW_LOW_BIT_32;
			}
			break;
			case REG_BITS_16:
				while( ((table_index + grouped_tables + 1) < tables_to_send)  &&      /* Only if there are more tables to be found */
					   (grouped_data < (SEIX006_I2C_MAX_BYTES - 2))            && /* Make sure we only send MAX allowed bytes */
					   ((table[table_index + grouped_tables + 1].address)     ==  /* Only if next tables address is */
					   (table[table_index + grouped_tables].address + 2))    &&   /* one incremented address as current */
					   (table[table_index + grouped_tables + 1].reg_bits == REG_BITS_16) /* Only if next table is the same amount of bit holder */
					 ) {
					grouped_data +=2;
					grouped_tables++;
				}

				/* Load all tables, default to one */
				for (j = 0, data_index = 0; j < grouped_tables + 1; j++, data_index += 2) {
					data_buffer[data_index] =
						(table[table_index + j].data & LOW_HIGH_BIT_32) >> SHIFT_8;
					data_buffer[data_index + 1] =
						(table[table_index + j].data & LOW_LOW_BIT_32);
				}

				grouped_data++;  /* To hold the "extra" byte compared to REG_BITS_8 */
			break;
			case REG_BITS_32:
				while( ((table_index + grouped_tables + 1) < tables_to_send)  &&      /* Only if there are more tables to be found */
					   (grouped_data < (SEIX006_I2C_MAX_BYTES - 4))            && /* Make sure we only send MAX allowed bytes */
					   ((table[table_index + grouped_tables + 1].address)     ==  /* Only if next tables address is */
					   (table[table_index + grouped_tables].address + 4))     &&  /* one incremented address as current */
					   (table[table_index + grouped_tables + 1].reg_bits == REG_BITS_32) /* Only if next table is the same amount of bit holder */
					 ) {
					grouped_data += 4;
					grouped_tables++;
				}

				/* Load all tables, default to one*/
				for(j = 0, data_index = 0; j < grouped_tables + 1; j++, data_index += 4) {
					data_buffer[data_index] =
					    (table[table_index + j].data & HIGH_HIGH_BIT_32) >> SHIFT_24;
					data_buffer[data_index + 1] =
					    (table[table_index + j].data & HIGH_LOW_BIT_32) >> SHIFT_16;
					data_buffer[data_index + 2] =
					    (table[table_index + j].data & LOW_HIGH_BIT_32) >> SHIFT_8;
					data_buffer[data_index + 3] =
					    (table[table_index + j].data & LOW_LOW_BIT_32);
				}

				grouped_data += 3; /* To hold the "three extra" bytes compared to REG_BITS_8 */
			break;
			default:
				CDBG("seix006_send_reg_table wrong reg_bits\n");
			break;
		}

		ret = seix006_i2c_write(table[table_index].address,
				       grouped_data + 1, &data_buffer[0]);
		if(ret) {
			CDBG("seix006_send_reg_table i2c_write failed\n");
			break;
		}

		mdelay(4);
		table_index += grouped_tables + 1;
	}

	CDBG("seix006_send_reg_table [E] ret[%d]\n", ret);

	return ret;
}

static int32_t seix006_check_msts(uint8_t value, uint32_t timeout)
{
	char data;
	uint32_t nRetry = 0;
	int32_t ret = 0;

	while (nRetry < timeout) {
		ret = seix006_i2c_read(SEIX006_USERCTRL_0010, BYTE_1, &data);
		if (ret) {
			CDBG("seix006_check_msts: i2c_read failed\n");
			return ret;
		}

		CDBG("seix006_check_msts: %X\n", data);
		if ( data == value ) {
			CDBG("seix006_check_msts match %X\n", value);
			return 0;
		}
		mdelay(SEIX006_POLLING_PERIOD);
		nRetry++;
	}

	CDBG("seix006_check_msts: timeout \n");
	return 1;
}

static int32_t seix006_refresh_monitor(uint32_t timeout)
{
	char data;
	uint32_t nRetry = 0;
	int32_t ret = 0;

	/* tell sensor to restart monitor mode after settings has been changed */
	ret = seix006_send_reg_table(seix006_MONI_REFRESH_F,
					sizeof_seix006_MONI_REFRESH_F/sizeof(struct reg_entry));
	if (ret) {
		CDBG("seix006_refresh_monitor: i2c_write failed\n");
		return ret;
	}

	/* 0x0012 returns to 0 after the sensor has finished the restart */
	while (nRetry < timeout) {
		ret = seix006_i2c_read(0x0012, BYTE_1, (int8_t *) &data);

		if (ret) {
			CDBG("seix006_refresh_monitor: i2c_read failed\n");
			return ret;
		}

		if (data == 0x0) {
			CDBG("seix006_refresh_monitor done\n");
			return 0;
		}
		mdelay(SEIX006_POLLING_PERIOD);
		nRetry++;
	}

	CDBG("seix006_refresh_monitor: timeout \n");
	return 1;
}

static int32_t seix006_check_bsts(uint8_t value, uint32_t timeout)
{
	char data;
	uint32_t nRetry = 0;
	int32_t ret = 0;

	while (nRetry < timeout) {
		ret = seix006_i2c_read(SEIX006_USERCTRL_0004, BYTE_1, &data);
		if (ret) {
			CDBG("seix006_check_bsts: i2c_read failed\n");
			return ret;
		}

		CDBG("seix006_check_bsts: %X\n", data);
		if (data == value) {
			CDBG("seix006_check_bsts match %X\n", value);
			return 0;
		}
		mdelay(SEIX006_POLLING_PERIOD);
		nRetry++;
	}

	CDBG("seix006_check_bsts: timeout \n");
	return 1;
}

static int32_t seix006_check_afsts(uint8_t value, uint32_t timeout)
{
	char data;
	uint32_t nRetry = 0;
	int32_t ret = 0;

	while (nRetry < timeout) {
		ret = seix006_i2c_read(SEIX006_SOUT_6D76, BYTE_1, &data);
		if (ret) {
			CDBG("seix006_check_afsts: i2c_read failed\n");
			return ret;
		}

		CDBG("seix006_check_afsts: %X\n", data);
		if (data == value) {
			CDBG("seix006_check_afsts match %X\n", value);
			return 0;
		}
		mdelay(SEIX006_POLLING_PERIOD);
		nRetry++;
	}

	CDBG("seix006_check_afsts: timeout \n");
	return 1;
}

static int autoflash_enable(int onoff)
{
	const uint16_t cmds[3][2] = { {0x027D, 0x05}, {0x028C, 0x01}, {0x0097, 0x02} };
	uint8_t zero;
	int nret, i;

	nret = -1;
	zero = 0;

	for (i = 0; i < 3; i++) {

		if (onoff) {
			/* enable */
			nret = seix006_i2c_write(cmds[i][0], BYTE_1, (uint8_t *) & (cmds[i][1]));
		} else {
			/* disable - all are 0 */
			nret = seix006_i2c_write(cmds[i][0], BYTE_1, &zero);
		}

		/* break if ioctl fails */
		if (nret) {
			break;
		}
	}

	return nret;
}

static int autoflash_adjust()
{
	static const uint16_t CMDS[3][2] = { {0x0282, 0xffff}, {0x445c, 0xffff}, {0x445e, 0xffff} };
	uint8_t reg_x2AA;
	uint16_t reg_x28A;
	int16_t reg_x286;
	uint16_t reg_x26E;
	uint16_t reg_x270;
	uint16_t reg_6C26;
	uint16_t reg_6C28;
	int nret, AEO, SR, SB;

	nret = seix006_i2c_read(0x02AA, BYTE_1, &reg_x2AA);

	if (nret) {
		CDBG("autoflash_adjust: Failed reading I2C reg 0x02AA: %d\n", nret);
		return nret;
	}

	if (reg_x2AA != 0) {
		CDBG("autoflash_adjust: Camera is still busy, reg. 0x02AA=%d\n", reg_x2AA);
		return 0;
	}

	reg_x28A = 0;
	reg_x286 = 0;
	reg_x26E = 0;
	reg_x270 = 0;
	reg_6C26 = 0;
	reg_6C28 = 0;
	AEO = 0;
	SR = 0;
	SB = 0;

	CDBG("autoflash_adjust: Camera finished AE adjust, reg. 0x02AA=%d\n", reg_x2AA);

	if ((nret = seix006_i2c_read(0x28A, BYTE_2, (uint8_t *) & reg_x28A)) ||
	    (nret = seix006_i2c_read(0x286, BYTE_2, (uint8_t *) & reg_x286)) ||
	    (nret = seix006_i2c_read(0x26E, BYTE_2, (uint8_t *) & reg_x26E)) ||
	    (nret = seix006_i2c_read(0x270, BYTE_2, (uint8_t *) & reg_x270)) ||
	    (nret = seix006_i2c_read(0x6C26, BYTE_2, (uint8_t *) & reg_6C26)) ||
	    (nret = seix006_i2c_read(0x6C28, BYTE_2, (uint8_t *) & reg_6C28))) {
		CDBG("autoflash_adjust: Failed reading camera I2C register: %d", nret);

		return nret;
	}

	CDBG(".. set autoflash_poll_reg_x2AA to FALSE!\n");
	seix006_ctrl->autoflash_poll_reg_x2AA = FALSE;
	memcpy(&(seix006_ctrl->autoflash_cmds[0][0]), CMDS, sizeof(CMDS));

	{
		int A, B, C, D, E, F, CR, CB, R1, R2, KL;
		int K, RF, RM;
		A = seix006_ctrl->autoflash_reg_x288 + seix006_ctrl->autoflash_reg_x284;
		C = seix006_ctrl->autoflash_reg_x26A;
		D = seix006_ctrl->autoflash_reg_x26C;
		B = reg_x28A + reg_x286;
		E = reg_x26E;
		F = reg_x270;
		CR = reg_6C26;
		CB = reg_6C28;
		R1 = 4000;
		R2 = 10000;
		KL = 500;

		if ((B - A) >= 5000) {
			AEO = -2317 - reg_x286;
		} else {
			AEO = -AEO_table[(B - A) / 10] - reg_x286;
		}

		RF = (1000 * (B - A)) / A;
		RM = ((373 * RF * RF) - (4137 * RF)) / 1000 + 224;
		if (RM < R1) {
			K = 1000;
		}
		if (RM > R1 && RM < R2) {
			K = 1000 - (((RM - R1) * (1000 - KL)) / (R2 - R1));
		}
		if (RM > R2) {
			K = KL;
		}
		SR = ((K * (1172 - ((C * 334) / 1000))) / 1000) - 200;
		SB = 1788 - ((D * 345) / 1000);

	}

	/* these are stored in device header structure, and reused in .._raw_snapshot_config */
	seix006_ctrl->autoflash_cmds[0][1] = AEO;
	seix006_ctrl->autoflash_cmds[1][1] = SR;
	seix006_ctrl->autoflash_cmds[2][1] = SB;

	return 0;
}

static int autoflash_strobe(int onoff)
{
	int nret;
	uint8_t tmp;

	tmp = (onoff ? 0x09 : 0x08);

	nret = seix006_i2c_write(0x069, BYTE_1, &tmp);
	return nret;
}

static int __seix006_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "%s \n", __func__);
	return msm_camera_drv_start(pdev, seix006_camera_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __seix006_probe,
	.driver = {
		.name = SEIX006_MSM_CAMERA_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init seix006_init(void)
{
	printk(KERN_INFO "%s \n", __func__);

	return platform_driver_register(&msm_camera_driver);
}

module_init(seix006_init);


static int init_thread(void* data)
{
	int32_t ret = 0;

	CDBG("Camera init thread started\n");

	down(&seix006_sem);

	if ( seix006_ctrl->vendor_id == VENDOR_ID_0 ) {
		if (seix006_ctrl->camera_revision == SEIX006_CAM_REV_ES1) {
			ret = seix006_set_regs(seix006_vendor_0_period_2_ES1);
			if(ret) {
				CDBG("seix006_set_regs seix006_KM0_period_2_ES1 failed\n");
			}

			ret = seix006_set_regs(seix006_vendor_0_period_3_ES1);
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_0_period_3_ES1 failed\n");
			}
		} else if (seix006_ctrl->camera_revision == SEIX006_CAM_REV_ES2) {
			if (seix006_ctrl->calibration_data.shd_index == 1) {
				ret = seix006_set_regs(seix006_vendor_0_SHD_1_ES2);
			}
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_0_SHD_1_ES2 failed\n");
			}
			if (seix006_ctrl->calibration_data.shd_index == 2) {
				ret = seix006_set_regs(seix006_vendor_0_SHD_2_ES2);
			}
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_0_SHD_2_ES2 failed\n");
			}
			if (seix006_ctrl->calibration_data.shd_index == 3) {
				ret = seix006_set_regs(seix006_vendor_0_SHD_3_ES2);
			}
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_0_SHD_3_ES2 failed\n");
			}

			ret = seix006_set_regs(seix006_vendor_0_period_2_ES2);
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_0_period_2_ES2 failed\n");
			}

			ret = seix006_set_regs(seix006_vendor_0_period_3_ES2);
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_0_period_3_ES2 failed\n");
			}
			CDBG("seix006_send_reg_table seix006_vendor_0_period_3_ES2 GOT IT\n");
		}

	} else if (seix006_ctrl->vendor_id == VENDOR_ID_1) {
		if ( seix006_ctrl->camera_revision == SEIX006_CAM_REV_ES1) {
			ret = seix006_set_regs(seix006_vendor_1_period_2_ES1);
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_1_period_2_ES1 failed\n");
			}

			ret = seix006_set_regs(seix006_vendor_1_period_3_ES1);
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_1_period_3_ES1 failed\n");
			}
		} else if (seix006_ctrl->camera_revision == SEIX006_CAM_REV_ES2) {
			if (seix006_ctrl->calibration_data.shd_index == 1) {
				ret = seix006_set_regs(seix006_vendor_1_SHD_1_ES2);
			}
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_1_SHD_1_ES2 failed\n");
			}
			if (seix006_ctrl->calibration_data.shd_index == 2) {
				ret = seix006_set_regs(seix006_vendor_1_SHD_2_ES2);
			}
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_1_SHD_2_ES2 failed\n");
			}
			if (seix006_ctrl->calibration_data.shd_index == 3) {
				ret = seix006_set_regs(seix006_vendor_1_SHD_3_ES2);
			}
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_1_SHD_3_ES2 failed\n");
			}
			ret = seix006_set_regs(seix006_vendor_1_period_2_ES2);
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_1_period_2_ES2 failed\n");
			}

			ret = seix006_set_regs(seix006_vendor_1_period_3_ES2);
			if(ret) {
				CDBG("seix006_set_regs seix006_vendor_1_period_3_ES2 failed\n");
			}
		}
	}
	ret = seix006_update_scene(seix006_ctrl->scene);
	seix006_ctrl->init_complete = 1;

	up(&seix006_sem);

	CDBG("Camera init thread end");
	return ret;
}


static int32_t seix006_raw_rgb_stream_config()
{
	int32_t ret;
	/* config the rgb stream regitser */
	uint8_t data = 0x04; /* RGB mode */

	CDBG("seix006_raw_rgb_stream_config [S]\n");

	seix006_i2c_write(0x001E, BYTE_1, &data);

	ret = seix006_send_reg_table(seix006_mode_movie,
		sizeof_seix006_mode_movie/sizeof(struct reg_entry));
	if(ret) {
		CDBG("seix006_raw_rgb_stream_config failed\n");
		return ret;
	}

	/* wait MODESEL_FIX to 3 */
	ret = seix006_check_msts(SEIX006_MSTS_MOV_VAL, SEIX006_POLLING_TIMES);
	if(ret) {
		CDBG("seix006_raw_rgb_stream_config "
				"seix006_check_msts failed\n");
		/* Continue silently */
	}

	seix006_ctrl->dev_mode = CAMERA_MODE_MOVIE;

	CDBG("seix006_raw_rgb_stream_config [E] ret[%d]\n", ret);

	return ret;
}


static int32_t seix006_movie_config()
{
	int32_t ret = 0;

	CDBG("seix006_movie_config [S]\n");

	if (seix006_ctrl->dev_mode == CAMERA_MODE_MOVIE) {
		CDBG("seix006_movie_config Already in movie mode. Do nothing...\n");
		goto movie_done;
	}

	ret = seix006_send_reg_table(seix006_mode_movie, sizeof_seix006_mode_movie/sizeof(struct reg_entry));
	if(ret) {
		CDBG("seix006_movie_config failed\n");
		return ret;
	}

	/* wait MODESEL_FIX to 3 */
	ret = seix006_check_msts(SEIX006_MSTS_MOV_VAL, SEIX006_POLLING_TIMES);
	if(ret) {
		CDBG("seix006_movie_config  seix006_check_msts failed\n");
		/* Continue silently */
	}

	seix006_ctrl->dev_mode = CAMERA_MODE_MOVIE;

movie_done:

	if (!seix006_ctrl->init_complete) {
		kthread_run(init_thread, NULL, "sensor_init");
	}

	CDBG("seix006_movie_config [E] ret[%d]\n",ret);

	return ret;
}

static long seix006_set_exposure_compensation(int8_t value)
{
	long rc = 0;
	uint8_t register_value = 0;
	CDBG("seix006_set_exposure_compensation [S] value=%d\n",value);
	if(value < -2 || value > 2) {
		CDBG("seix006_set_exposure_compensation %d is an invalid value, has to be between -2 to +2 \n",value);
		return -EINVAL;
	}
	register_value = value * 3;
	rc = seix006_i2c_write(0x0080, BYTE_1, &register_value);
	CDBG("seix006_set_exposure_compensation [E] ret[%ld]\n",rc);
	return rc;
}

static long seix006_set_brightness(uint8_t value)
{
	long rc = 0;
	CDBG("seix006_set_brightness [S] %d\n",value);

	rc = seix006_i2c_write(0x0060, BYTE_1, &value);
	CDBG("seix006_set_brightness [E] ret[%ld]\n",rc);
	return rc;
}


static long seix006_set_img_quality(uint8_t value)
{
	long rc = 0;
	CDBG("seix006_set_img_quality [S] %d\n",value);

	switch(value)
	{
	case 0:
		CDBG("seix006_set_img_quality STANDARD\n");
		break;
	case 1:
		CDBG("seix006_set_img_quality FINE\n");
		break;
	case 2:
		CDBG("seix006_set_img_quality SUPER FINE\n");
		break;
	default:
		CDBG("seix006_set_img_quality Incorrect value\n");
		rc = -EINVAL;
	}

	if(rc == 0)
	{
		rc = seix006_i2c_write(0x204, BYTE_1, &value);
	}

	CDBG("seix006_set_img_quality [E] ret[%ld]\n",rc);
	return rc;
}

static long seix006_set_contrast(uint8_t value)
{
	long rc = 0;
	CDBG("seix006_set_contrast [S] %d\n",value);

	rc = seix006_i2c_write(0x0061, BYTE_1, &value);
	CDBG("seix006_set_contrast [E] ret[%ld]\n",rc);
	return rc;
}

static long seix006_set_sharpness(uint8_t value)
{
	long rc = 0;
	CDBG("seix006_set_sharpness [S] %d\n",value);

	rc = seix006_i2c_write(0x0062, BYTE_1, &value);
	CDBG("seix006_set_sharpness [E] ret[%ld]\n",rc);
	return rc;
}

static long seix006_set_wb(int wb_type)
{
	uint8_t data = 0;
	long rc = 0;
	CDBG("seix006_set_wb [S] wb_type %d\n",wb_type);

	switch (wb_type) {
	case CAMERA_WBTYPE_AUTO:
		data = 0x20;
		break;

	case CAMERA_WBTYPE_INCANDESCENT:
		data = 0x28;
		break;

	case CAMERA_WBTYPE_FLUORESCENT:
		data = 0x27;
		break;

	case CAMERA_WBTYPE_DAYLIGHT:
		data = 0x24;
		break;

	case CAMERA_WBTYPE_CLOUDY_DAYLIGHT:
		data = 0x26;
		break;

	default:
		data = 0x20;
	}

	rc = seix006_i2c_write(0x0102, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0102 data %d ret[%ld]\n",data, rc);
		return rc;
	}
	rc = seix006_i2c_write(0x0107, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0107 data %d ret[%ld]\n",data, rc);
		return rc;
	}
	rc = seix006_i2c_write(0x010C, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x010C data %d ret[%ld]\n",data, rc);
		return rc;
	}
	rc = seix006_i2c_write(0x0111, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0111 data %d ret[%ld]\n",data, rc);
		return rc;
	}
	rc = seix006_i2c_write(0x0116, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0116 data %d ret[%ld]\n",data, rc);
		return rc;
	}
	rc = seix006_i2c_write(0x011B, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x011B data %d ret[%ld]\n",data, rc);
		return rc;
	}
	rc = seix006_i2c_write(0x0120, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0120 data %d ret[%ld]\n",data, rc);
		return rc;
	}
	rc = seix006_i2c_write(0x0125, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0125 data %d ret[%ld]\n",data, rc);
		return rc;
	}
	return rc;
}

static long seix006_set_exposure_mode(int exp_mode)
{
	uint8_t data = 0;
	uint8_t mode = 0;
	long rc = 0;
	CDBG("seix006_set_exposure_mode [S] exp_mode %d\n",exp_mode);

	switch (exp_mode) {
	case CAMERA_AUTO_EXPOSURE_FRAME_AVG:
		mode = 0x0;
		break;

	case CAMERA_AUTO_EXPOSURE_CENTER_WEIGHTED:
		mode = 0x1;
		break;

	case CAMERA_AUTO_EXPOSURE_SPOT_METERING:
		mode = 0x2;
		break;

	default:
		return rc;
	}

	data = 0x40 + mode;
	rc = seix006_i2c_write(0x0104, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0102 data %d ret[%ld]\n",data, rc);
		return rc;
	}

	data = 0x4C + mode;
	rc = seix006_i2c_write(0x0109, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0107 data %d ret[%ld]\n",data, rc);
		return rc;
	}

	data = 0x40 + mode;
	rc = seix006_i2c_write(0x010E, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x010C data %d ret[%ld]\n",data, rc);
		return rc;
	}

	data = 0x40 + mode;
	rc = seix006_i2c_write(0x0113, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0111 data %d ret[%ld]\n",data, rc);
		return rc;
	}

	data = mode;
	rc = seix006_i2c_write(0x0118, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0116 data %d ret[%ld]\n",data, rc);
		return rc;
	}

	data = 0xCC + mode;
	rc = seix006_i2c_write(0x011D, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x011B data %d ret[%ld]\n",data, rc);
		return rc;
	}

	data = mode;
	rc = seix006_i2c_write(0x0122, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0120 data %d ret[%ld]\n",data, rc);
		return rc;
	}

	data = 0xC0 + mode;
	rc = seix006_i2c_write(0x0127, BYTE_1, &data);
	if(rc) {
		CDBG("seix006_set_wb [X] Failed 0x0125 data %d ret[%ld]\n",data, rc);
		return rc;
	}
	return rc;
}

static long seix006_set_effect(int mode, int effect)
{
	uint8_t data = 0;
	long rc = 0;
	CDBG("seix006_set_effect [S] effect %d\n",effect);

	switch (effect) {
	case CAMERA_EFFECT_OFF:
		data = 0x0;
		break;

	case CAMERA_EFFECT_MONO:
		data = 0x4;
		break;

	case CAMERA_EFFECT_NEGATIVE:
		data = 0x2;
		break;

	case CAMERA_EFFECT_SOLARIZE:
		data = 0x1;
		break;

	case CAMERA_EFFECT_SEPIA:
		data = 0x3;
		break;

	default:
		data = 0x0;
	}

	rc = seix006_i2c_write(0x005F, BYTE_1, &data);
	CDBG("seix006_set_effect [E] data %d ret[%ld]\n",data, rc);
	return rc;
}

static long seix006_set_flash(uint8_t flash_mode)
{
	long rc = 0;
	CDBG("seix006_set_flash [S] data %d ret[%ld]\n", flash_mode, rc);

	camera_flash = flash_mode;

	return rc;
}

static long setix006_set_led_state(uint8_t led_state)
{
	long rc = 0;
	CDBG("setix006_set_led_state [S] led state %d\n", led_state);

	if (seix006_ctrl->sensordata->flash_data->flash_src == NULL)
		return -ENODEV;

	rc = msm_camera_flash_set_led_state(seix006_ctrl->sensordata->flash_data,
			MSM_CAMERA_LED_OFF);

	if (rc)
		pr_err("setix006_set_led_state ret[%ld]\n", rc);

	rc = msm_camera_flash_set_led_state(seix006_ctrl->sensordata->flash_data,
			led_state);

	CDBG("setix006_set_led_state [E] led state %d ret[%ld]\n", led_state, rc);
	return rc;
}

