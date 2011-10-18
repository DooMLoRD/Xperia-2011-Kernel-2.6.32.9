/* drivers/media/video/msm/mt9v114.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9v114.h"

#ifdef CONFIG_MSM_CAMERA_DEBUG
	#ifdef CDBG
	#undef CDBG
	#endif
	#define CDBG(fmt, args...) \
		printk(KERN_ERR "msm_cam_mt9v114: " fmt, ##args)
#endif

struct mt9v114_work {
	struct work_struct work;
};

static struct mt9v114_work *mt9v114_sensorw;
static struct i2c_client *mt9v114_client;
static s8 mt9v114_opened;
static s8 mt9v114_probe_completed;

struct mt9v114_ctrl {
	const struct msm_camera_sensor_info *sensordata;
	enum camera_devmode_type            dev_mode;
};

static struct mt9v114_ctrl *mt9v114_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(mt9v114_wait_queue);
DEFINE_MUTEX(mt9v114_mutex);

static s16 mt9v114_effect = CAMERA_EFFECT_OFF;
static s16 mt9v114_fps = 30;

/*=============================================================*/

static int mt9v114_power_on(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	CDBG("mt9v114_power_on [S]\n");

	if (mt9v114_probe_completed) {
		/* only enable after sensor probe */
		rc = msm_camio_clk_enable(CAMIO_MDC_CLK);
		rc = msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	}
	msleep(5);

	/* Input MCLK = 19.2MHz */
	msm_camio_clk_rate_set(data->pdata->ioclk.mclk_clk_rate);

	msleep(5);

	/* pull reset/standby low to bring sensor out of hard standby */
	rc = gpio_request(data->sensor_pwd, "mt9v114");
	if (rc == 0) {
		/* GPIO PIN turn on */
		rc = gpio_direction_output(data->sensor_pwd, 1); /* inactive */
		msleep(5);
		rc = gpio_direction_output(data->sensor_pwd, 0); /* active */
	}
	gpio_free(data->sensor_pwd);

	CDBG("mt9v114_power_on [E]\n");
	return rc;
}

static int mt9v114_power_off(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	CDBG("mt9v114_power_off [S]\n");

	/* Output CAM_MCLK(0MHz) */
	msm_camio_clk_rate_set(0);

	/* .sensor_pwd = 31, */
	rc = gpio_request(data->sensor_pwd, "mt9v114");
	if (rc == 0)
		rc = gpio_direction_output(data->sensor_pwd, 1); /* inactive */

	gpio_free(data->sensor_pwd);

	if (mt9v114_probe_completed) {	/* only enable after sensor probe */
		rc = msm_camio_clk_disable(CAMIO_MDC_CLK);
		rc = msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
	}

	CDBG("mt9v114_sensor_poweroff [E]\n");
	return rc;
}

static int mt9v114_i2c_read(
	uint32_t address,
	uint8_t length,
	uint8_t *data)
{
	uint8_t sbuf[SENSOR_I2C_ADDR_4BYTE];
	uint8_t rbuf[I2C_RW_BASIC_LENGTH_MAX];
	int32_t i;
	/* length of address is always 2 bytes for mt9v114 */
	uint8_t address_type = 2;
	int rc = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = mt9v114_client->addr,
			.flags = 0,
			.len = address_type,
			.buf = &sbuf[0],
		},
		{
			.addr = mt9v114_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = &rbuf[0],
		},
	};
	struct i2c_msg *pmsg = msgs;
	uint8_t msg_length = 2;

	CDBG("mt9v114_i2c_read [S]\n");

	if (!data) {
		pr_err("mt9v114_i2c_read *data failed\n");
		return -EFAULT;
	}

	if (length > I2C_RW_BASIC_LENGTH_MAX) {
		pr_err("mt9v114_i2c_read length[%d] failed\n", length);
		return -EFAULT;
	}

	memset(rbuf, 0, sizeof(rbuf));

	sbuf[0] = (uint8_t)((address & 0xff00) >> 8);
	sbuf[1] = (uint8_t)((address & 0x00ff) >> 0);

	if (rc < 0)
		return rc;

	rc = i2c_transfer(mt9v114_client->adapter, pmsg, msg_length);
	if (rc < 0) {
		pr_err("mt9v114_i2c_read transfer failed\n");
		return -EIO;
	}

	for (i = 0; i < length; i++)
		data[i] = rbuf[i];

	return 0;
}

static int mt9v114_i2c_write(
	uint32_t address,
	uint8_t length,
	uint8_t *data)
{
	uint8_t sbuf[I2C_RW_BASIC_LENGTH_MAX];
	int32_t i;
	int rc = 0;
	/* length of address is always 2 bytes for mt9v114 */
	uint8_t address_type = 2;

	struct i2c_msg msg[] = {
		{
			.addr = mt9v114_client->addr,
			.flags = 0,
			.len = length + address_type,
			.buf = &sbuf[0],
		},
	};

	CDBG("mt9v114_i2c_write [S] %04x,%02x,%02x\n",
		address, address_type, length);

	if (!data) {
		pr_err("mt9v114_i2c_write *data failed\n");
		return -EFAULT;
	}

	if (length + address_type > I2C_RW_BASIC_LENGTH_MAX) {
		CDBG("mt9v114_i2c_write length[%d] failed\n", length);
		return -EFAULT;
	}

	sbuf[0] = (uint8_t)((address & 0xff00) >> 8);
	sbuf[1] = (uint8_t)((address & 0x00ff) >> 0);

	if (rc < 0)
		return rc;

	for (i = 0; i < length; i++)
		sbuf[i + address_type] = data[i];

	rc = i2c_transfer(mt9v114_client->adapter, msg, 1);
	if (rc < 0) {
		pr_err("mt9v114_i2c_write transfer failed\n");
		return -EIO;
	}

	return 0;
}

static long mt9v114_load_tables(const struct mt9v114_table_t *regs)
{
	uint8_t tmp[2];
	long rc = 0;
	int i = 0;

	CDBG("mt9v114_load_tables [S]");

	if (!regs)
		return -EINVAL;

	while (i < CAM_LOAD_TABLES_MAX)	{
		switch (regs[i].action & CAM_OP_MASK) {
		case OP_WRITE_I2C:
			tmp[0] = regs[i].value;
			rc = mt9v114_i2c_write(regs[i].address, 1, tmp);
			if (rc < 0)	{
				pr_err("mt9v114_load_tables failed "
						"rc=%ld\n", rc);
				return rc;
			}
			break;

		case OP_WRITE_I2C_WORD:
			tmp[0] = (regs[i].value & 0xff00) >> 8;
			tmp[1] = (regs[i].value & 0x00ff) >> 0;
			rc = mt9v114_i2c_write(regs[i].address, 2, tmp);
			if (rc < 0)	{
				pr_err("mt9v114_load_tables failed "
						"rc=%ld\n", rc);
				return rc;
			}
			break;

		case OP_RETURN:
			return 0;

		default:
			pr_err("mt9v114_load_tables invalid command found");
			return -EINVAL;
		}
		i++;
	}

	CDBG("mt9v114_load_tables [E]");
	return 0;
}


static long mt9v114_set_effect(int mode, int effect)
{
	long rc = 0;
	uint8_t data = 0;

	switch (effect) {
	case CAMERA_EFFECT_MONO:
		data = 0x01;
		break;
	case CAMERA_EFFECT_NEGATIVE:
		data = 0x03;
		break;
	case CAMERA_EFFECT_SEPIA:
		data = 0x02;
		break;
	case CAMERA_EFFECT_OFF:
		data = 0x00;
		break;
	default:
		/* All other but above invalid */
		pr_err("mt9v114_set_effect effect=%d "
				"is not a valid value\n", effect);
		return -EINVAL;
	}

	rc = mt9v114_i2c_write(0xA010, 1, &data);

	if (rc == 0)
		mt9v114_effect = effect;
	else
		pr_err("mt9v114_set_effect failed");

	CDBG("mt9v114_set_effect effect=%d rc=%ld\n", effect, rc);

	return rc;
}

static long mt9v114_set_framerate(uint16_t fps)
{
	long rc = 0;
	uint32_t nom = 1000000 * Q8;
	uint32_t row_time = 66; /* us */
	uint32_t reg_value;
	uint8_t data[4];

	CDBG("mt9v114_set_framerate [S] fps=%d\n", fps);

	if (fps != 15 && fps != 30) {
		pr_err("mt9v114_set_framerate %d fps "
				"not supported by camera\n", fps);
		rc = -EFAULT;
	}
	fps = fps * Q8;
	/* set max fps */
	reg_value = nom / row_time / fps;
	data[1] = (uint8_t)reg_value;
	data[0] = (uint8_t)(reg_value >> 8);

	rc = mt9v114_i2c_write(0x300A, 2, data);

	if (rc == 0) {
		/* set min fps */
		data[0] = 0;
		data[1] = (uint8_t)(100 * Q8 / fps); /* 50 Hz */
		data[2] = 0;
		data[3] = (uint8_t)(120 * Q8 / fps); /* 60 Hz */
		rc = mt9v114_i2c_write(0xA076, 4, data);
		if (rc == 0) {
			/* Set dwell point */
			rc |= mt9v114_i2c_write(0xA01A, 2, data);
			if (!rc)
				pr_err("mt9v114_sensor_set_min_fps() dwell "
					"success %x %x ", data[0], data[1]);
			else
				pr_err("mt9v114_sensor_set_min_fps() failure");
		}
	}

	if (rc < 0)
		pr_err("mt9v114_set_framerate failed rc=%ld\n", rc);
	else
		mt9v114_fps = (fps / Q8);

	CDBG("mt9v114_set_framerate [E]\n");

	return rc;
}

static long mt9v114_set_sensor_mode(int mode)
{
	long rc = 0;
	int n = 0;
	uint8_t data[2];

	CDBG("mt9v114_set_sensor_mode [S]\n");

	switch (mode) {
	case SENSOR_SNAPSHOT_MODE:
		/* Skip reprogramming if sensor is already running */
		if (mt9v114_ctrl->dev_mode == CAMERA_MODE_MONITOR)
			break;
	case SENSOR_PREVIEW_MODE:
	case SENSOR_MOVIE_MODE:
		rc = mt9v114_load_tables(mt9v114_pll_setting);

		if (rc != 0) {
			pr_err("mt9v114_set_sensor_mode failed "
					"to send pll rc=%ld\n", rc);
			return rc;
		}

		msleep(2);
		do {
			rc = mt9v114_i2c_read(0x0018,  2, data);
			if (rc != 0) {
				pr_err("mt9v114_set_sensor_mode poll"
						" error rc=%ld\n", rc);
				return -EINVAL;
			}
			msleep(2); /* 2 ms */
			n++;
			CDBG("mt9v114_set_sensor_mode poll values "
					"n=%d data[0]=%d", n, data[0]);
		} while (0x40 == data[0] && n < 5000);

		rc = mt9v114_load_tables(mt9v114_cam_init);
		if (rc != 0) {
			pr_err("mt9v114_set_sensor_mode "
					"failed to send init rc=%ld\n", rc);
			return rc;
		}
		msleep(2);

		/* Re-apply effects */
		if (mt9v114_effect != CAMERA_EFFECT_OFF)
			rc |= mt9v114_set_effect(mode, mt9v114_effect);

		/* Re-apply framerate */
		rc |= mt9v114_set_framerate(mt9v114_fps);

		if (rc == 0)
			mt9v114_ctrl->dev_mode = CAMERA_MODE_MONITOR;
		break;
	default:
		pr_err("mt9v114_set_sensor_mode Unknown mode %d\n", mode);
		return -EINVAL;
	}

	CDBG("mt9v114_set_sensor_mode [E] mode=%d, rc=%ld\n", mode, rc);
	return rc;
}

static int mt9v114_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	uint16_t model_id = 0;
	uint8_t i2c_buf[2];
	int rc = 0;

	pr_err("mt9v114_sensor_init_probe [S]\n");

	/* power on camera */
	rc = mt9v114_power_on(data);
	if (rc < 0) {
		pr_err("pwd failed!\n");
		goto init_probe_fail;
	}

	/* get Product ID */

	rc = mt9v114_i2c_read(0x0000, 2, i2c_buf);
	if (rc < 0) {
		pr_err("mt9v114_sensor_init_probe i2c read of PID failed\n");
		goto init_probe_fail;
	}

	model_id = (i2c_buf[0] << 8) | (i2c_buf[1]);

	pr_err("mt9v114 model_id = 0x%x\n", model_id);

	pr_err("mt9v114_sensor_init_probe [E]\n");
	return rc;

init_probe_fail:
	mt9v114_power_off(data);
	pr_err("mt9v114_sensor_init_probe failed rc=%d\n", rc);
	return rc;
}

static int mt9v114_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	pr_err("mt9v114_sensor_init [S]\n");

	mutex_lock(&mt9v114_mutex);
	if (mt9v114_opened) {
		pr_err("mt9v114_open already opened\n");
		rc = -EBUSY;
		goto init_done;
	}

	mt9v114_ctrl = kzalloc(sizeof(struct mt9v114_ctrl), GFP_KERNEL);
	if (NULL == mt9v114_ctrl) {
		pr_err("mt9v114_sensor_init memory allocation failed\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9v114_ctrl->sensordata = data;
	mt9v114_ctrl->dev_mode = CAMERA_MODE_STANDBY;
	mt9v114_effect = CAMERA_EFFECT_OFF;

	msm_camio_camif_pad_reg_reset(); /* msm_io_vfe31 */

	rc = mt9v114_sensor_init_probe(data);
	if (rc < 0)
		goto init_fail;

	mt9v114_opened = 1;
	pr_err("mt9v114_sensor_init [E]\n");

init_done:
	mutex_unlock(&mt9v114_mutex);
	return rc;

init_fail:
	pr_err("mt9v114_sensor_init failed!\n");
	kfree(mt9v114_ctrl);
	mt9v114_ctrl = NULL;
	mutex_unlock(&mt9v114_mutex);
	return rc;
}

static int mt9v114_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9v114_wait_queue);
	return 0;
}

static int mt9v114_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;

	if (copy_from_user(&cfg_data, (void *)argp,
			sizeof(struct sensor_cfg_data))) {
		return -EFAULT;
	}

	mutex_lock(&mt9v114_mutex);

	CDBG("mt9v114_sensor_config [S]  cfgtype=%d, mode=%d\n",
			cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = mt9v114_set_sensor_mode(cfg_data.mode);
		CDBG("mt9v114_sensor_config CFG_SET_MODE rc=%d\n", (int)rc);
		break;

	case CFG_SET_EFFECT:
		rc = mt9v114_set_effect(cfg_data.mode, cfg_data.cfg.effect);
		CDBG("mt9v114_sensor_config CFG_SET_EFFECT rc=%d\n", (int)rc);
		break;
	case CFG_SET_FPS:
		rc = mt9v114_set_framerate(cfg_data.cfg.fps.f_mult);
		CDBG("mt9v114_sensor_config CFG_SET_FPS rc=%d\n", (int)rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&mt9v114_mutex);

	CDBG("mt9v114_sensor_config [E]\n");

	return rc;
}

static int mt9v114_sensor_release(void)
{
	int rc = 0;

	mutex_lock(&mt9v114_mutex);
	if (mt9v114_opened) {
		mt9v114_power_off(mt9v114_ctrl->sensordata);
		mt9v114_opened = 0;
	}

	kfree(mt9v114_ctrl);
	mt9v114_ctrl = NULL;
	mutex_unlock(&mt9v114_mutex);

	return rc;
}

static int mt9v114_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	CDBG("mt9v114_i2c_probe [S]\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	CDBG("pass mt9v114_ctrl kzalloc\n");

	mt9v114_sensorw =
		kzalloc(sizeof(struct mt9v114_work), GFP_KERNEL);

	if (!mt9v114_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9v114_sensorw);
	mt9v114_init_client(client);
	mt9v114_client = client;

	CDBG("mt9v114_i2c_probe [E] succeeded\n");

	return 0;

probe_failure:
	kfree(mt9v114_sensorw);
	kfree(mt9v114_ctrl);
	mt9v114_ctrl = NULL;
	pr_err("mt9v114_i2c_probe [E] failed rc=%d\n", rc);
	return rc;
}

static const struct i2c_device_id mt9v114_i2c_id[] = {
	{ "mt9v114", 0},
	{ },
};

static struct i2c_driver mt9v114_i2c_driver = {
	.id_table = mt9v114_i2c_id,
	.probe = mt9v114_i2c_probe,
	.remove = __exit_p(mt9v114_i2c_remove),
	.driver = {
		.name = "mt9v114",
	},
};

static int mt9v114_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = 0;

	CDBG("mt9v114_sensor_probe [S]\n");

	if (NULL == info || NULL == s) {
		pr_err("mt9v114_sensor_probe memory invalid argument\n");
		rc = -EINVAL;
		goto probe_done;
	}

	rc = i2c_add_driver(&mt9v114_i2c_driver);
	if (rc < 0 || mt9v114_client == NULL) {
		pr_err("mt9v114_sensor_probe failed mt9v114 i2c_add_driver\n");
		rc = (rc == 0) ? -EINVAL : rc;
		goto probe_done;
	}

	CDBG("mt9v114_sensor_probe pass i2c_add_driver\n");

	rc = mt9v114_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = mt9v114_sensor_init;
	s->s_release = mt9v114_sensor_release;
	s->s_config = mt9v114_sensor_config;

probe_done:
	if (rc == 0) {
		mt9v114_power_off(info);
		mt9v114_probe_completed = 1;
		CDBG("mt9v114_sensor_probe probe_done"
				" should power off sensor\n");
	} else {
		kfree(mt9v114_sensorw);
		kfree(mt9v114_ctrl);
		mt9v114_ctrl = NULL;
	}
	CDBG("%s %s:%d -->\n", __FILE__, __func__, __LINE__);
	CDBG("mt9v114_sensor_probe [E]\n");

	return rc;
}

static int __mt9v114_remove(struct platform_device *pdev)
{
	kfree(mt9v114_sensorw);
	return 0;
}

static int __mt9v114_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9v114_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9v114_probe,
	.remove = __devexit_p(__mt9v114_remove),
	.driver = {
		.name = "msm_camera_mt9v114",
		.owner = THIS_MODULE,
	},
};

static int __init mt9v114_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9v114_init);
