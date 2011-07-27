/*
 * Copyright (c) 2009,2010 Sony Ericsson Mobile Communications
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "ovm7692.h"

/* OVM OVM7692 Registers and their values */
/* Sensor Core Registers */
#define  REG_OVM7692_PID_H 0x000A
#define  REG_OVM7692_PID_L 0x000B
#define  OVM7692_PRODUCT_ID     0x7692

#define REG_12			0x12	/* REG 12 control */
#define COM12_RESET		0x80	/* Register reset */
#define COM12_RGB		0x02	/* bits 0 and 2 - RGB format */
#define COM12_YUV		0x00	/* YUV */
#define COM12_RGB565	0x06


#ifdef CONFIG_MSM_CAMERA_DEBUG
	#ifdef CDBG
	#undef CDBG
	#endif
	#define CDBG(fmt, args...) printk(KERN_ERR "msm_cam_ovm7692: " fmt, ##args)
#endif

struct ovm7692_work {
	struct work_struct work;
};

static struct ovm7692_work *ovm7692_sensorw;
static struct i2c_client *ovm7692_client;
static s8 ovm7692_opened;
static s8 ovm7692_probe_completed;

struct ovm7692_ctrl {
	const struct msm_camera_sensor_info       *sensordata;
	enum camera_devmode_type                  dev_mode;
};

static struct ovm7692_ctrl *ovm7692_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(ovm7692_wait_queue);
DEFINE_MUTEX(ovm7692_mutex);

static s16 ovm7692_effect = CAMERA_EFFECT_OFF;
static s16 ovm7692_fps = 30;

/*=============================================================*/
/* VT only have pwd pin */
static int ovm7692_power_on(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	if (ovm7692_probe_completed) {
		/* only enable after sensor probe */
		rc = msm_camio_clk_enable(CAMIO_MDC_CLK);
		rc = msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	}

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	rc = gpio_request(data->sensor_pwd, "ovm7692");
	if (rc == 0) {
		/* GPIO PIN turn on */
		rc = gpio_direction_output(data->sensor_pwd, 1); /* inactive */
		mdelay(5);
		rc = gpio_direction_output(data->sensor_pwd, 0); /* active */
	}
	gpio_free(data->sensor_pwd);

	CDBG("ovm7692_power_on [e]\n");
	return rc;
}


/* VT only have pwd pin */
static int ovm7692_power_off(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	rc = gpio_request(data->sensor_pwd, "ovm7692");
	if (rc == 0)
		rc = gpio_direction_output(data->sensor_pwd, 1); /* inactive */

	gpio_free(data->sensor_pwd);

	if (ovm7692_probe_completed) {	/* only enable after sensor probe */
		rc = msm_camio_clk_disable(CAMIO_MDC_CLK);
		rc = msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
	}

	/* Output CAM_MCLK(0MHz) */
	msm_camio_clk_rate_set(0);

	CDBG("ovm7692_sensor_poweroff [e]\n");
	return rc;
}


/*
 * Low-level register I/O.
 */

static int ovm7692_read(struct i2c_client *c, u8 reg,
		u8 *value)
{
	int ret;

	ret = i2c_smbus_read_byte_data(c, reg);
	if (ret >= 0) {
		*value = (u8) ret;
		ret = 0;
	}
	return ret;
}

static int ovm7692_write_array(struct i2c_client *c,
		const struct regval_list *vals, u32 list_size)
{
	int ret = 0;
	int index = 0;
	while (index < list_size) {
		ret = i2c_smbus_write_byte_data(c, vals->reg_num, vals->value);
		if (ret < 0) {
			pr_err("%s failed to write to i2c\n", __func__);
			return ret;
		}
		vals++;
		index++;
		mdelay(1);
	}
	CDBG("%s reg_table=%d, write array size = %d to i2c\n", __func__, list_size, index);
	return 0;
}


static long ovm7692_set_effect(int mode, int effect)
{
	long rc = 0;
	u8 i2c_buf = 0;

	switch (effect) {
	case CAMERA_EFFECT_MONO:
		rc = ovm7692_read(ovm7692_client, 0x0081, &i2c_buf);

		i2c_buf |= 0x20;
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x81, i2c_buf);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x28, 0x02);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xd2, 0x18);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xda, 0x80);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xdb, 0x80);

		break;
	case CAMERA_EFFECT_NEGATIVE:
		rc = ovm7692_read(ovm7692_client, 0x0081, &i2c_buf);

		i2c_buf |= 0x20;
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x81, i2c_buf);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x28, 0x82);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xd2, 0x00);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xda, 0x80);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xdb, 0x80);

		break;
	case CAMERA_EFFECT_SEPIA:
		rc = ovm7692_read(ovm7692_client, 0x0081, &i2c_buf);

		i2c_buf |= 0x20;
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x81, i2c_buf);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x28, 0x02);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xd2, 0x18);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xda, 0x40);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xdb, 0xa0);

		break;
	case CAMERA_EFFECT_OFF:
		rc = ovm7692_read(ovm7692_client, 0x0081, &i2c_buf);
		i2c_buf &= 0xdf;
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x81, i2c_buf);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0x28, 0x02);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xd2, 0x00);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xda, 0x80);
		rc |= i2c_smbus_write_byte_data(ovm7692_client, 0xdb, 0x80);

		break;
	default:
			/* All other but above invalid */
			return -EINVAL;
	}

	if (rc == 0)
		ovm7692_effect = effect;

	CDBG("ovm7692_set_effect effect=%d rc=%ld\n", effect, rc);
	return rc;
}

static long ovm7692_set_framerate(uint16_t fps)
{
	long rc = 0;
	CDBG("seix006_set_framerate fps=%d [S]\n", fps);

	if (fps == 30) {
		rc |= ovm7692_write_array(ovm7692_client, ovm7692_framerate_30_regs,
					size_ovm7692_framerate_30_regs);
	} else if (fps == 15) {
		 rc |= ovm7692_write_array(ovm7692_client, ovm7692_framerate_15_regs,
					size_ovm7692_framerate_15_regs);
	} else {
		pr_err("ovm7692_set_framerate %d fps not supported by camera\n", fps);
		rc = -EFAULT;
	}

	if (rc < 0)
		pr_err("ovm7692_set_framerate failed rc=%ld\n", rc);
	else
		ovm7692_fps = fps;

	return rc;
}

static long ovm7692_set_sensor_mode(int mode)
{
	u16 clock;
	int rc = 0;

	switch (mode) {
	case SENSOR_SNAPSHOT_MODE:
		/* Skip reprogramming if sensor is already running */
		if (ovm7692_ctrl->dev_mode == CAMERA_MODE_MONITOR)
			break;
	case SENSOR_PREVIEW_MODE:
	case SENSOR_MOVIE_MODE:
		/* WRITE ARRAY */
		clock = 2400;
		rc |= i2c_smbus_write_byte_data(ovm7692_client, REG_12, 0);
		mdelay(2);

		/* Omnivision default VGA settings */
		rc |= i2c_smbus_write_byte_data(ovm7692_client, REG_12,  COM12_RESET);
		mdelay(2);
		rc |= ovm7692_write_array(ovm7692_client, ovm7692_ref_vga_regs,
					size_ovm7692_ref_vga_regs);

		rc |= ovm7692_write_array(ovm7692_client, ovm7692_device_init_regs,
					size_ovm7692_device_init_regs);

		/* Re-apply effects */
		if (ovm7692_effect != CAMERA_EFFECT_OFF)
			rc |= ovm7692_set_effect(mode, ovm7692_effect);

		/* Re-apply framerate */
		rc |= ovm7692_set_framerate(ovm7692_fps);

		if (rc == 0)
			ovm7692_ctrl->dev_mode = CAMERA_MODE_MONITOR;

		CDBG("ovm7692_set_sensor_mode = %d, rc:%d\n", mode, rc);
		break;
	default:
		pr_err("ovm7692_set_sensor_mode Unknown mode %d\n", mode);
		return -EINVAL;
	}

	return rc;
}

static int ovm7692_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	u16 model_id = 0;
	u8 i2c_buf = 0;
	int rc = 0;

	/* power on camera */
	rc = ovm7692_power_on(data);
	if (rc < 0) {
		pr_err("pwd failed!\n");
		goto init_probe_fail;
	}

	/* get Product ID */
	rc = ovm7692_read(ovm7692_client, REG_OVM7692_PID_H, &i2c_buf);
	if (rc < 0) {
		pr_err("ovm7692_i2c_read PID H failed\n");
		goto init_probe_fail;
	}

	CDBG("ovm7692_i2c_read PID H:0x%x\n", i2c_buf);
	model_id = (model_id | i2c_buf) << 8;
	i2c_buf = 0;

	rc = ovm7692_read(ovm7692_client, REG_OVM7692_PID_L, &i2c_buf);
	if (rc < 0) {
		pr_err("ovm7692_i2c_read PID L fialed\n");
		goto init_probe_fail;
	}

	CDBG("ovm7692_i2c_read PID L:0x%x\n", i2c_buf);
	model_id = (model_id & 0xFF00) | i2c_buf;
	CDBG("ovm7692_i2c_read model_id L:0x%x\n", model_id);

	CDBG("ovm7692 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != OVM7692_PRODUCT_ID) {
		rc = -EINVAL;
		goto init_probe_fail;
	}

	/* test write */
	i2c_buf = 0;
	rc = ovm7692_read(ovm7692_client, REG_12, &i2c_buf);
	CDBG("ovm7692 REG12 = 0x%x\n", i2c_buf);
	rc = i2c_smbus_write_byte_data(ovm7692_client, REG_12,  COM12_RGB | COM12_RGB565);
	if (rc < 0) {
		pr_err("ovm7692_i2c_write PID L fialed\n");
		goto init_probe_fail;
	}
	CDBG("ovm7692 pass write\n");
	i2c_buf = 0;
	rc = ovm7692_read(ovm7692_client, REG_12, &i2c_buf);
	if (rc < 0) {
		pr_err("ovm7692_i2c_read PID L fialed\n");
		goto init_probe_fail;
	}
	CDBG("ovm7692 after write REG12 = 0x%x\n", i2c_buf);

	return rc;

init_probe_fail:
	ovm7692_power_off(data);
	pr_err("ovm7692 init_probe_fail\n");
	return rc;
}

static int ovm7692_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	mutex_lock(&ovm7692_mutex);
	if (ovm7692_opened) {
		pr_err("ovm7692_open already opened\n");
		rc = -EBUSY;
		goto init_done;
	}

	ovm7692_ctrl = kzalloc(sizeof(struct ovm7692_ctrl), GFP_KERNEL);
	if (NULL == ovm7692_ctrl) {
		pr_err("%s memory allocation failed\n", __func__);
		rc = -ENOMEM;
		goto init_done;
	}

	ovm7692_ctrl->sensordata = data;
	ovm7692_ctrl->dev_mode = CAMERA_MODE_STANDBY;
	ovm7692_effect = CAMERA_EFFECT_OFF;

	msm_camio_camif_pad_reg_reset(); /* msm_io_vfe31 */

	rc = ovm7692_sensor_init_probe(data);
	if (rc < 0) {
		pr_err("ovm7692_sensor_init failed!\n");
		goto init_fail;
	}

	ovm7692_opened = 1;
	CDBG("ovm7692_sensor_init done\n");

init_done:
	mutex_unlock(&ovm7692_mutex);
	return rc;

init_fail:
	kfree(ovm7692_ctrl);
	ovm7692_ctrl = NULL;
	mutex_unlock(&ovm7692_mutex);
	return rc;
}

static int ovm7692_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ovm7692_wait_queue);
	return 0;
}

static int ovm7692_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data, (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	mutex_lock(&ovm7692_mutex);

	CDBG("ovm7692_ioctl, cfgtype = %d, mode = %d\n",
	cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = ovm7692_set_sensor_mode(cfg_data.mode);
		CDBG("ovm7692_sensor_config CFG_SET_MODE rc = %d\n", (int)rc);
		break;

	case CFG_SET_EFFECT:
		rc = ovm7692_set_effect(cfg_data.mode, cfg_data.cfg.effect);
		CDBG("ovm7692_sensor_config CFG_SET_EFFECT rc = %d\n", (int)rc);
		break;
	case CFG_SET_FPS:
		rc = ovm7692_set_framerate(cfg_data.cfg.fps.f_mult);
		CDBG("ovm7692_sensor_config CFG_SET_FPS rc = %d\n", (int)rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&ovm7692_mutex);

	return rc;
}

static int ovm7692_sensor_release(void)
{
	int rc = 0;

	mutex_lock(&ovm7692_mutex);
	if (ovm7692_opened) {
		ovm7692_power_off(ovm7692_ctrl->sensordata);
		ovm7692_opened = 0;
	}

	kfree(ovm7692_ctrl);
	ovm7692_ctrl = NULL;
	mutex_unlock(&ovm7692_mutex);

	return rc;
}

static int ovm7692_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	CDBG("pass ovm7692_ctrl kzalloc\n");

	ovm7692_sensorw =
		kzalloc(sizeof(struct ovm7692_work), GFP_KERNEL);

	if (!ovm7692_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ovm7692_sensorw);
	ovm7692_init_client(client);
	ovm7692_client = client;

	CDBG("ovm7692_i2c_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(ovm7692_sensorw);
	kfree(ovm7692_ctrl);
	ovm7692_ctrl = NULL;
	pr_err("ovm7692_i2c_probe failed!\n");
	return rc;
}

static const struct i2c_device_id ovm7692_i2c_id[] = {
	{ "ovm7692", 0},
	{ },
};

static struct i2c_driver ovm7692_i2c_driver = {
	.id_table = ovm7692_i2c_id,
	.probe  = ovm7692_i2c_probe,
	.remove = __exit_p(ovm7692_i2c_remove),
	.driver = {
		.name = "ovm7692",
	},
};

static int ovm7692_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = 0;

	if (NULL == info || NULL == s) {
		pr_err("ovm7692_sensor_probe memory invalid argument\n");
		rc = -EINVAL;
		goto probe_done;
	}

	rc = i2c_add_driver(&ovm7692_i2c_driver);
	if (rc < 0 || ovm7692_client == NULL) {
		pr_err("failed ovm7692 i2c_add_driver\n");
		rc = (rc == 0) ? -EINVAL : rc;
		goto probe_done;
	}

	CDBG("ovm7692 pass i2c_add_driver\n");

	rc = ovm7692_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = ovm7692_sensor_init;
	s->s_release = ovm7692_sensor_release;
	s->s_config  = ovm7692_sensor_config;

probe_done:
	if (rc == 0) {
		ovm7692_power_off(info);
		ovm7692_probe_completed = 1;
		CDBG("ovm7692_sensor_probe probe_done should power off sensor\n");
	} else {
		kfree(ovm7692_sensorw);
		kfree(ovm7692_ctrl);
		ovm7692_ctrl = NULL;
	}
	CDBG("%s %s:%d -->\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __ovm7692_remove(struct platform_device *pdev)
{
	kfree(ovm7692_sensorw);
	return 0;
}

static int __ovm7692_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ovm7692_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ovm7692_probe,
	.remove = __devexit_p(__ovm7692_remove),
	.driver = {
		.name = "msm_camera_ovm7692",
		.owner = THIS_MODULE,
	},
};

static int __init ovm7692_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ovm7692_init);
