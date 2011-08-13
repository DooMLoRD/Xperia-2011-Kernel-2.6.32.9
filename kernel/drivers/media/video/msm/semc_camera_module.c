/* drivers/media/video/msm/semc_camera_module.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include "semc_camera_module.h"

#define I2C_NAME "semc_camera"
#define GPIO_NAME "semc_camera"
#define MSM_CAMERA_NAME "msm_camera_semc_camera"
#define MODULE_KMO05BN0 "KMO05BN0"
#define MODULE_STW05BN0 "STW05BN0"

#define GPIO_HI 1
#define GPIO_LO 0

#define SUCCESS 0

#define I2C_RW_BASIC_LENGTH_MAX 32

/* #define DEFAULT_CLOCK_RATE 25869474 */
#define DEFAULT_CLOCK_RATE 8000000

#define ROM_DEV_NO 0x50
#define ROM_AREA_MAX_BYTE 0x0800
#define ROM_READ_MAX_BYTE 0x0020
#define ROM_READ_ADDR_MASK 0x0700

#define CSLAVE_ADDR			0x1A
#define REG_MODE_SELECT			0x0100
#define REG_MODE_SELECT_STOP		0x00
#define STOP_WAIT			100

#define ENABLE_SEMC_LOGE
/* #define ENABLE_SEMC_LOGD */
/* #define ENABLE_SEMC_LOGV */

#ifdef ENABLE_SEMC_LOGE
#define SEMC_LOGE(f, a...) printk(KERN_ERR "[CAM KERN DD-MAIN]" f, ##a)
#else
#define SEMC_LOGE(f, a...)
#endif /* ENABLE_SEMC_LOGE */

#ifdef ENABLE_SEMC_LOGD
#define SEMC_LOGD(f, a...) printk(KERN_DEBUG "[CAM KERN DD-MAIN]" f, ##a)
#else
#define SEMC_LOGD(f, a...)
#endif /* ENABLE_SEMC_LOGD */

#ifdef ENABLE_SEMC_LOGV
#define SEMC_LOGV(f, a...) printk(KERN_DEBUG "[CAM KERN DD-MAIN]" f, ##a)
#else
#define SEMC_LOGV(f, a...)
#endif /* ENABLE_SEMC_LOGV */

#define SEMC_CHK_ERR(x) (x < 0)

struct semc_camera_work_type {
	struct work_struct work;
};

struct semc_camera_ctrl_type {
	int8_t opened;
	const struct msm_camera_sensor_info *sensordata;
	struct semc_camera_work_type *sensorw;
	struct i2c_client *client;
};

static uint8_t *g_rom_save_area;
static uint32_t g_camera_clock_rate = DEFAULT_CLOCK_RATE;
static struct semc_camera_ctrl_type *g_camera_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(g_camera_wait_queue);
static DECLARE_MUTEX(g_camera_sem);

static int semc_camera_i2c_read(
	uint8_t slave_addr,
	uint32_t address,
	uint8_t address_type,
	uint8_t length,
	uint8_t *data)
{
	uint8_t sbuf[SENSOR_I2C_ADDR_4BYTE];
	uint8_t rbuf[I2C_RW_BASIC_LENGTH_MAX];
	int32_t i;
	int ret = SUCCESS;

#ifdef ENABLE_SEMC_LOGV
	uint8_t dbgbuf[I2C_RW_BASIC_LENGTH_MAX * 2 + 1];
#endif /* ENABLE_SEMC_LOGV */

	struct i2c_msg msgs[] =
		{
			{
				.addr = slave_addr,
				.flags = 0,
				.len = address_type,
				.buf = &sbuf[0],
			},
			{
				.addr = slave_addr,
				.flags = I2C_M_RD,
				.len = length,
				.buf = &rbuf[0],
			},
		};
	struct i2c_msg *pmsg = msgs;
	uint8_t msg_length = 2;

	SEMC_LOGD("camera_i2c_read [S]\n");

	if (!data) {
		SEMC_LOGE("camera_i2c_read *data failed\n");
		return -EFAULT;
	}

	if (length > I2C_RW_BASIC_LENGTH_MAX) {
		SEMC_LOGE("camera_i2c_read length[%d] failed\n", length);
		return -EFAULT;
	}

	memset(rbuf, 0, sizeof(rbuf));

	switch (address_type) {
	case SENSOR_I2C_ADDR_0BYTE:
		pmsg = &msgs[1];
		msg_length = 1;
		SEMC_LOGV("camera_i2c_read (I2C_ADDR_0BYTE)\n");
		break;
	case SENSOR_I2C_ADDR_1BYTE:
		sbuf[0] = (uint8_t)((address & 0xff) >> 0);
		SEMC_LOGV("camera_i2c_read (addr[0x%02X], data[0x%02X])\n",
			msgs[0].addr, msgs[0].buf[0]);
		break;
	case SENSOR_I2C_ADDR_2BYTE:
		sbuf[0] = (uint8_t)((address & 0xff00) >> 8);
		sbuf[1] = (uint8_t)((address & 0x00ff) >> 0);
		SEMC_LOGV("camera_i2c_read (addr[0x%02X], data[0x%02X%02X])\n",
			msgs[0].addr, msgs[0].buf[0], msgs[0].buf[1]);
		break;
	case SENSOR_I2C_ADDR_4BYTE:
		sbuf[0] = (uint8_t)((address & 0xff000000) >> 24);
		sbuf[1] = (uint8_t)((address & 0x00ff0000) >> 16);
		sbuf[2] = (uint8_t)((address & 0x0000ff00) >> 8);
		sbuf[3] = (uint8_t)((address & 0x000000ff) >> 0);
		SEMC_LOGV(
			"camera_i2c_read (a[0x%02X], d[0x%02X%02X%02X%02X])\n",
			msgs[0].addr, msgs[0].buf[0], msgs[0].buf[1],
			msgs[0].buf[2], msgs[0].buf[3]);
		break;
	default:
		SEMC_LOGE("camera_i2c_read invalid address_type[%d]\n",
			address_type);
		ret = -EINVAL;
		break;
	}

	if (SEMC_CHK_ERR(ret))
		return ret;

	ret = i2c_transfer(g_camera_ctrl->client->adapter, pmsg, msg_length);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_i2c_read transfer failed\n");
		return -EIO;
	}

#ifdef ENABLE_SEMC_LOGV
	for (i = 0; i < msgs[1].len; i++)
		sprintf(&dbgbuf[i * 2], "%02X", msgs[1].buf[i]);
	dbgbuf[i * 2] = '\0';
	SEMC_LOGV("camera_i2c_read read(readlen[%d], data[0x%s])\n",
		msgs[1].len, &dbgbuf[0]);
#endif /* ENABLE_SEMC_LOGV */

	for (i = 0; i < length; i++)
		data[i] = rbuf[i];

	SEMC_LOGD("camera_i2c_read [E]\n");

	return SUCCESS;
}

static int semc_camera_i2c_write(
	uint8_t slave_addr,
	uint32_t address,
	uint8_t address_type,
	uint8_t length,
	uint8_t *data)
{
	uint8_t sbuf[I2C_RW_BASIC_LENGTH_MAX];
	int32_t i;
	int ret = SUCCESS;

#ifdef ENABLE_SEMC_LOGV
	uint8_t dbgbuf[I2C_RW_BASIC_LENGTH_MAX * 2 + 1];
#endif /* ENABLE_SEMC_LOGV */

	struct i2c_msg msg[] =
		{
			{
				.addr = slave_addr,
				.flags = 0,
				.len = length + address_type,
				.buf = &sbuf[0],
			},
		};

	SEMC_LOGD("camera_i2c_write [S] %04x,%02x,%02x\n",
		address, address_type, length);

	if (!data) {
		SEMC_LOGE("camera_i2c_write *data failed\n");
		return -EFAULT;
	}

	if (length + address_type > I2C_RW_BASIC_LENGTH_MAX) {
		SEMC_LOGE("camera_i2c_write length[%d] failed\n", length);
		return -EFAULT;
	}

	switch (address_type) {
	case SENSOR_I2C_ADDR_1BYTE:
		sbuf[0] = (uint8_t)((address & 0xff) >> 0);
		break;
	case SENSOR_I2C_ADDR_2BYTE:
		sbuf[0] = (uint8_t)((address & 0xff00) >> 8);
		sbuf[1] = (uint8_t)((address & 0x00ff) >> 0);
		break;
	case SENSOR_I2C_ADDR_4BYTE:
		sbuf[0] = (uint8_t)((address & 0xff000000) >> 24);
		sbuf[1] = (uint8_t)((address & 0x00ff0000) >> 16);
		sbuf[2] = (uint8_t)((address & 0x0000ff00) >> 8);
		sbuf[3] = (uint8_t)((address & 0x000000ff) >> 0);
		break;
	default:
		SEMC_LOGE("camera_i2c_write invalid address_type[%d]\n",
			address_type);
		ret = -EINVAL;
		break;
	}

	if (SEMC_CHK_ERR(ret))
		return ret;

	for (i = 0; i < length; i++)
		sbuf[i + address_type] = data[i];

#ifdef ENABLE_SEMC_LOGV
	for (i = 0; i < msg[0].len; i++)
		sprintf(&dbgbuf[i * 2], "%02X", msg[0].buf[i]);
	dbgbuf[i * 2] = '\0';
	SEMC_LOGV("camera_i2c_write (addr[0x%2X], data[0x%s])\n",
		msg[0].addr, &dbgbuf[0]);
#endif /* ENABLE_SEMC_LOGV */

	ret = i2c_transfer(g_camera_ctrl->client->adapter, msg, 1);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_i2c_write transfer failed\n");
		return -EIO;
	}

	SEMC_LOGD("camera_i2c_write [E]\n");

	return SUCCESS;
}

static int semc_camera_rom_in(
	uint16_t address,
	uint16_t length,
	uint8_t __user *save_area)
{
	int ret = SUCCESS;

	SEMC_LOGD("semc_camera_rom_in [S]\n");

	if (!g_rom_save_area) {
		SEMC_LOGE("semc_camera_rom_in read failed\n");
		return -EFAULT;
	}

	if ((address >= ROM_AREA_MAX_BYTE) ||
		(address + length - 1 >= ROM_AREA_MAX_BYTE) ||
		(length == 0)) {
		SEMC_LOGE(
			"semc_camera_rom_in parameter addr=[%d],lng=[%d]\n",
			address, length);
		return -EINVAL;
	}

	ret = copy_to_user(save_area, g_rom_save_area + address, length);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("semc_camera_rom_in copy_to_user failed\n");
		return -EFAULT;
	}

	SEMC_LOGD("semc_camera_rom_in [E]\n");

	return ret;
}

static int semc_camera_gpio_access(int gpio_pin, int dir)
{
	int ret = SUCCESS;

	SEMC_LOGD("camera_gpio_access [S]\n");

	ret = gpio_request(gpio_pin, GPIO_NAME);
	if (!SEMC_CHK_ERR(ret)) {
		ret = gpio_direction_output(gpio_pin, dir);
		gpio_free(gpio_pin);
	}

	SEMC_LOGD("camera_gpio_access [E] ret[%d]\n", ret);

	return ret;
}

static int semc_camera_gpio_ctrl(struct sensor_gpio_ctrl *gpio_ctrl)
{
	int ret = SUCCESS;

	SEMC_LOGD("camera_gpio_ctrl [S]\n");

	switch (gpio_ctrl->gpio) {
	case SENSOR_GPIO_CTRL_RESET:
		ret = semc_camera_gpio_access(
			g_camera_ctrl->sensordata->sensor_reset,
			gpio_ctrl->value);
		break;
	case SENSOR_GPIO_CTRL_STANBY:
	default:
		ret = -ENODEV;
		break;
	}

	SEMC_LOGD("camera_gpio_ctrl [E]\n");

	return ret;
}

static int semc_camera_resource_enable(
	const struct msm_camera_sensor_pwr *resource,
	unsigned int level)
{
	int ret = SUCCESS;
	struct vreg *reg = NULL;

	if (!resource) {
		SEMC_LOGE("camera_resource_enable argument is NULL.\n");
		return -EPERM;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		SEMC_LOGV("camera_resource_enable GPIO[%d]\n",
			resource->resource.number);
		ret = semc_camera_gpio_access(
			resource->resource.number, GPIO_HI);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		SEMC_LOGV("camera_resource_enable VREG[%s]=%d\n",
			resource->resource.name, level);
		reg = vreg_get(NULL, resource->resource.name);
		if (!reg) {
			SEMC_LOGE("camera_resource_enable VREG name err %s\n",
				resource->resource.name);
			ret = -ENOENT;
			break;
		}
		ret = vreg_set_level(reg, level);
		if (SEMC_CHK_ERR(ret)) {
			SEMC_LOGE("camera_resource_enable vreg=%s level=%d\n",
				resource->resource.name, level);
			break;
		}
		ret = vreg_enable(reg);
		if (SEMC_CHK_ERR(ret)) {
			SEMC_LOGE("camera_resource_enable vreg=%s level=%d\n",
			resource->resource.name, level);
			break;
		}
		break;
	default:
		SEMC_LOGE("camera_resource_enable invalid resource type[%d]\n",
			resource->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int semc_camera_resource_disable(
	const struct msm_camera_sensor_pwr *resource)
{
	int ret = SUCCESS;

	if (!resource) {
		SEMC_LOGE("camera_resource_disable argument is NULL.\n");
		return -EPERM;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		SEMC_LOGV("camera_resource_disable GPIO[%d]\n",
			resource->resource.number);
		ret = semc_camera_gpio_access(
			resource->resource.number, GPIO_LO);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		SEMC_LOGV("camera_resource_disable VREG[%s]\n",
			resource->resource.name);
		ret = vreg_disable(vreg_get(0, resource->resource.name));
		break;
	default:
		SEMC_LOGE("camera_resource_disable invalid rsc type[%d]\n",
			resource->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int semc_camera_init_client(struct i2c_client *client)
{
	SEMC_LOGD("camera_init_client [S]\n");

	if (!client) {
		SEMC_LOGE("camera_init_client *client failed\n");
		return -EFAULT;
	}

	/* Initialize the MSM_CAMI2C chip */
	init_waitqueue_head(&g_camera_wait_queue);

	SEMC_LOGD("camera_init_client [E]\n");

	return SUCCESS;
}

static int semc_camera_param_init(void)
{
	if (!g_camera_ctrl)
		return -EPERM;

	return SUCCESS;
}

static int semc_camera_sensor_on(void)
{
	int ret = SUCCESS;

	SEMC_LOGD("camera_sensor_on [S]\n");

#if defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	/* VGA_RST_N release(GPIO31 = HIGH) */
	ret = semc_camera_gpio_access(
		g_camera_ctrl->sensordata->sub_sensor_reset, GPIO_HI);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_sensor_on VGA_RST_N release failed\n");
		return ret;
	}

	/* 5ms wait */
	mdelay(5);
#endif

	if (g_rom_save_area &&
		((strncmp(MODULE_KMO05BN0, g_rom_save_area, 8) == 0) ||
		(strncmp(MODULE_STW05BN0, g_rom_save_area, 8) == 0))) {
		/* Power on VCAM_AF(3.0V) */
		ret = semc_camera_resource_enable(
			&g_camera_ctrl->sensordata->vcam_af, 3000);
	} else {
		/* CAM_RESET_N release(GPIO0 = High) */
		ret = semc_camera_gpio_access(
			g_camera_ctrl->sensordata->sensor_reset, GPIO_HI);
		if (SEMC_CHK_ERR(ret)) {
			SEMC_LOGE("camera_sensor_on CAM_RESET_N release failed\n");
			return ret;
		}

		/* 1ms wait */
		mdelay(1);

		/* Power on VCAM_AF(2.9V) */
		ret = semc_camera_resource_enable(
			&g_camera_ctrl->sensordata->vcam_af, 2900);
	}
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_sensor_on Power on VCAM_AF failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	/* Power on VCAM_SD(1.2V) */
	ret = semc_camera_resource_enable(
		&g_camera_ctrl->sensordata->vcam_sd, 1200);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_sensor_on Power on VCAM_SD failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	/* Power on VCAM_IO(1.8V) */
	ret = semc_camera_resource_enable(
		&g_camera_ctrl->sensordata->vcam_io, 1800);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_sensor_on Power on VCAM_IO failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	/* Power on VCAM_SA(2.8V) */
	ret = semc_camera_resource_enable(
		&g_camera_ctrl->sensordata->vcam_sa, 2800);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_sensor_on Power on VCAM_SA failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	if (g_rom_save_area &&
		((strncmp(MODULE_KMO05BN0, g_rom_save_area, 8) == 0) ||
		 (strncmp(MODULE_STW05BN0, g_rom_save_area, 8) == 0))) {
		/* CAM_RESET_N release(GPIO0 = High) */
		ret = semc_camera_gpio_access(
			g_camera_ctrl->sensordata->sensor_reset, GPIO_HI);
		if (SEMC_CHK_ERR(ret)) {
			SEMC_LOGE("camera_sensor_on CAM_RESET_N release failed\n");
			return ret;
		}

		/* 5ms wait */
		mdelay(5);
	}

	/* Set CAM_MCLK(8MHz) */
	msm_camio_cam_mclk_enable(g_camera_clock_rate);

	/* 5ms wait */
	mdelay(5);

	SEMC_LOGD("camera_sensor_on [E] ret[%d]\n", ret);

	return ret;
}

static int semc_camera_probe_sensor_on(void)
{
	int ret = SUCCESS;

	SEMC_LOGD("camera_probe_sensor_on [S]\n");

#if defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	/* VGA_RST_N release(GPIO31 = HIGH) */
	ret = semc_camera_gpio_access(
		g_camera_ctrl->sensordata->sub_sensor_reset, GPIO_HI);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_sensor_on VGA_RST_N release failed\n");
		return ret;
	}

	/* 5ms wait */
	mdelay(5);
#endif

	/* Power on VCAM_AF(2.9V) */
	ret = semc_camera_resource_enable(
		&g_camera_ctrl->sensordata->vcam_af, 2900);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_probe_sensor_on Power on VCAM_AF failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	/* Power on VCAM_SD(1.2V) */
	ret = semc_camera_resource_enable(
		&g_camera_ctrl->sensordata->vcam_sd, 1200);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_probe_sensor_on Power on VCAM_SD failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	/* Power on VCAM_IO(1.8V) */
	ret = semc_camera_resource_enable(
		&g_camera_ctrl->sensordata->vcam_io, 1800);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_probe_sensor_on Power on VCAM_IO failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	/* Power on VCAM_SA(2.8V) */
	ret = semc_camera_resource_enable(
		&g_camera_ctrl->sensordata->vcam_sa, 2800);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_probe_sensor_on Power on VCAM_SA failed\n");
		return ret;
	}

	/* 1ms wait */
	mdelay(1);

	/* CAM_RESET_N release(GPIO0 = High) */
	ret = semc_camera_gpio_access(
		g_camera_ctrl->sensordata->sensor_reset, GPIO_HI);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_probe_sensor_on CAM_RESET_N release failed\n");
		return ret;
	}

	/* 5ms wait */
	mdelay(5);

	/* Set CAM_MCLK(8MHz) */
	msm_camio_cam_mclk_enable(g_camera_clock_rate);

	/* 5ms wait */
	mdelay(5);

	SEMC_LOGD("camera_probe_sensor_on [E] ret[%d]\n", ret);

	return ret;
}

static void semc_camera_sensor_exit(void)
{
	int ret = SUCCESS;
	uint8_t iodt = REG_MODE_SELECT_STOP;

	SEMC_LOGD("camera_sensor_exit [S]\n");

	semc_camera_i2c_write(
				CSLAVE_ADDR,
				REG_MODE_SELECT,
				SENSOR_I2C_ADDR_2BYTE,
				1,
				&iodt);
	mdelay(STOP_WAIT);

	/* Stop CAM_MCLK */
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);

	/* 1ms wait */
	mdelay(1);

	/* CAM_RESET_N release(GPIO0 = LOW) */
	ret = semc_camera_gpio_access(
		g_camera_ctrl->sensordata->sensor_reset, GPIO_LO);
	if (SEMC_CHK_ERR(ret))
		SEMC_LOGE("camera_sensor_exit CAM_RESET_N release failed\n");

	/* 1ms wait */
	mdelay(1);

	/* Power off VCAM_SA(0V) */
	ret = semc_camera_resource_disable(
		&g_camera_ctrl->sensordata->vcam_sa);
	if (SEMC_CHK_ERR(ret))
		SEMC_LOGE("camera_sensor_exit Power off VCAM_SA failed\n");

	/* Power off VCAM_AF(0V) */
	ret = semc_camera_resource_disable(
		&g_camera_ctrl->sensordata->vcam_af);
	if (SEMC_CHK_ERR(ret))
		SEMC_LOGE("camera_sensor_exit Power off VCAM_AF failed\n");

	/* 10ms wait */
	mdelay(10);

	/* Power off VCAM_IO(0V) */
	ret = semc_camera_resource_disable(
		&g_camera_ctrl->sensordata->vcam_io);
	if (SEMC_CHK_ERR(ret))
		SEMC_LOGE("camera_sensor_exit Power off VCAM_IO failed\n");

	/* 50ms wait */
	mdelay(50);

	/* Power off VCAM_SD(0V) */
	ret = semc_camera_resource_disable(
		&g_camera_ctrl->sensordata->vcam_sd);
	if (SEMC_CHK_ERR(ret))
		SEMC_LOGE("camera_sensor_exit Power off VCAM_SD failed\n");



#if defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	/* 5ms wait */
	mdelay(5);

	/* VGA_RST_N release(GPIO31 = LOW) */
	ret = semc_camera_gpio_access(
		g_camera_ctrl->sensordata->sub_sensor_reset, GPIO_LO);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_sensor_exit VGA_RST_N release failed\n");
	}
#endif

	/* 10ms wait */
	mdelay(10);

	SEMC_LOGD("camera_sensor_exit [E]\n");
}

static int semc_camera_sensor_open_init(
	const struct msm_camera_sensor_info *data)
{
	int ret = SUCCESS;

	SEMC_LOGD("sensor_open_init [S]\n");

	down(&g_camera_sem);

	if (g_camera_ctrl->opened) {
		SEMC_LOGE("sensor_open_init already opened\n");
		goto open_done;
	}

	ret = semc_camera_param_init();
	if (SEMC_CHK_ERR(ret))
		SEMC_LOGE("sensor_open_init param_init return failed\n");

	ret = semc_camera_sensor_on();
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("sensor_open_init sensor_on failed\n");
		goto open_done;
	}

	if (!data->csi_if) {
		msm_camio_camif_pad_reg_reset();

		/* 20ms wait */
		mdelay(20);
	}

open_done:
	if (SEMC_CHK_ERR(ret)) {
		semc_camera_sensor_exit();
		ret = -EFAULT;
	} else {
		g_camera_ctrl->opened = 1;
	}

	up(&g_camera_sem);

	SEMC_LOGD("sensor_open_init [E]\n");

	return ret;
}

static int semc_camera_sensor_config(void __user *argp)
{
	int ret = SUCCESS;
	struct sensor_cfg_data cfg_data;
	uint8_t slave_addr;
	uint32_t address;
	uint8_t address_type;
	uint8_t length;
	uint8_t data[32];

	SEMC_LOGD("sensor_config [S]\n");

	ret = copy_from_user(&cfg_data, (void *)argp,
		sizeof(struct sensor_cfg_data));
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("sensor_config copy_from_user failed\n");
		return -EFAULT;
	}

	down(&g_camera_sem);

	SEMC_LOGD("sensor_config cfgtype = %d\n", cfg_data.cfgtype);
	switch (cfg_data.cfgtype) {
	case CFG_I2C_WRITE:
		slave_addr = cfg_data.cfg.i2c_io.slave_addr;
		address = cfg_data.cfg.i2c_io.address;
		address_type = cfg_data.cfg.i2c_io.address_type;
		length = cfg_data.cfg.i2c_io.length;
		ret = copy_from_user((void *)data,
			(void *)cfg_data.cfg.i2c_io.data, length);
		if (SEMC_CHK_ERR(ret))
			break;
		ret = semc_camera_i2c_write(
			slave_addr, address, address_type, length, data);
		break;
	case CFG_I2C_READ:
		slave_addr = cfg_data.cfg.i2c_io.slave_addr;
		address = cfg_data.cfg.i2c_io.address;
		address_type = cfg_data.cfg.i2c_io.address_type;
		length = cfg_data.cfg.i2c_io.length;
		ret = semc_camera_i2c_read(
			slave_addr, address, address_type, length, data);
		if (SEMC_CHK_ERR(ret))
			break;
		ret = copy_to_user((void *)cfg_data.cfg.i2c_io.data,
			(void *)data, length);
		break;
	case CFG_GPIO_CTRL:
		ret = semc_camera_gpio_ctrl(&cfg_data.cfg.gpio_ctrl);
		break;
	case CFG_CSI_CTRL:
		ret = msm_camio_csi_config(
		(struct msm_camera_csi_params *)&cfg_data.cfg.csi_ctrl);
		break;
	case CFG_ROM_READ:
		ret = semc_camera_rom_in(
			cfg_data.cfg.rom_in.address,
			cfg_data.cfg.rom_in.length,
			cfg_data.cfg.rom_in.data);
		break;
	default:
		SEMC_LOGE("sensor_config cfgtype failed\n");
		ret = -EFAULT;
		break;
	}

	up(&g_camera_sem);

	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("sensor_config failed\n");
		return -EFAULT;
	}

	ret = copy_to_user(
		(void *)argp, &cfg_data, sizeof(struct sensor_cfg_data));
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("sensor_config copy_to_user failed\n");
		return -EFAULT;
	}

	SEMC_LOGD("sensor_config [E]\n");

	return ret;
}

static int semc_camera_sensor_release(void)
{
	SEMC_LOGD("sensor_release [S]\n");

	down(&g_camera_sem);

	if (g_camera_ctrl->opened) {
		semc_camera_sensor_exit();
		g_camera_ctrl->opened = 0;
	}

	up(&g_camera_sem);

	SEMC_LOGD("sensor_release [E]\n");

	return SUCCESS;
}

static int semc_camera_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = SUCCESS;

	SEMC_LOGD("camera_probe [S]\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SEMC_LOGE("camera_probe i2c_check_functionality failed\n");
		kfree(g_camera_ctrl->sensorw);
		g_camera_ctrl->sensorw = NULL;
		return -ENOTSUPP;
	}

	g_camera_ctrl->sensorw =
		kzalloc(sizeof(struct semc_camera_work_type), GFP_KERNEL);

	if (!g_camera_ctrl->sensorw) {
		SEMC_LOGE("camera_probe sensorw failed\n");
		kfree(g_camera_ctrl->sensorw);
		g_camera_ctrl->sensorw = NULL;
		return -ENOMEM;
	}

	i2c_set_clientdata(client, g_camera_ctrl->sensorw);

	semc_camera_init_client(client);
	g_camera_ctrl->client = client;

	SEMC_LOGD("camera_probe [E] ret[%d]\n", ret);

	return ret;
}

static int __exit semc_camera_remove(struct i2c_client *client)
{
	struct semc_camera_work_type *work = i2c_get_clientdata(client);

	SEMC_LOGD("camera_remove [S]\n");

	free_irq(client->irq, work);
	g_camera_ctrl->client = NULL;
	kfree(work);

	SEMC_LOGD("camera_remove [E]\n");

	return SUCCESS;
}

static const struct i2c_device_id g_semc_camera_id[] = {
	{ I2C_NAME, 0 },
	{ }
};

static struct i2c_driver g_semc_camera_driver = {
	.id_table = g_semc_camera_id,
	.probe = semc_camera_probe,
	.remove = __exit_p(semc_camera_remove),
	.driver = {
		.name = I2C_NAME,
	},
};

static int semc_camera_init(const struct msm_camera_sensor_info *data)
{
	int ret = SUCCESS;

	SEMC_LOGD("camera_init [S]\n");

	g_camera_ctrl = kzalloc(
		sizeof(struct semc_camera_ctrl_type), GFP_KERNEL);
	if (!g_camera_ctrl) {
		SEMC_LOGE("camera_init *g_camera_ctrl failed\n");
		return -ENOMEM;
	}

	if (data) {
		g_camera_ctrl->sensordata = data;
		ret = i2c_add_driver(&g_semc_camera_driver);
		if (SEMC_CHK_ERR(ret)) {
			kfree(g_camera_ctrl);
			g_camera_ctrl = NULL;
		}
	}

	SEMC_LOGD("camera_init [E] ret[%d]\n", ret);

	return ret;
}

static int semc_camera_probe_init(
	const struct msm_camera_sensor_info *info,
	struct msm_sensor_ctrl *s)
{
	int ret = SUCCESS;
	int i;
	uint8_t slv_address;
	uint32_t adress;

	SEMC_LOGD("camera_probe_init [S]\n");

	ret = semc_camera_init(info);
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_probe_init driver initialization failed\n");
		goto probe_done;
	}

	s->s_init = semc_camera_sensor_open_init;
	s->s_release = semc_camera_sensor_release;
	s->s_config = semc_camera_sensor_config;

	ret = semc_camera_probe_sensor_on();
	if (SEMC_CHK_ERR(ret)) {
		SEMC_LOGE("camera_probe_init power on failed\n");
		goto probe_done;
	}

	if (!g_rom_save_area) {
		g_rom_save_area = kzalloc(ROM_AREA_MAX_BYTE, GFP_KERNEL);
		if (!g_rom_save_area) {
			SEMC_LOGE("camera_probe_init power on rom get err\n");
			goto probe_done;
		}
	}

	for (i = 0; i < ROM_AREA_MAX_BYTE; i = i + ROM_READ_MAX_BYTE) {
		slv_address = ROM_DEV_NO + ((i & ROM_READ_ADDR_MASK) >> 8);
		adress = i & 0x00FF;
		ret = semc_camera_i2c_read(slv_address, adress,
			SENSOR_I2C_ADDR_1BYTE,
			ROM_READ_MAX_BYTE, g_rom_save_area + i);
		if (SEMC_CHK_ERR(ret)) {
			kfree(g_rom_save_area);
			g_rom_save_area = NULL;
			SEMC_LOGE("camera_probe_init power on rom read err\n");
			goto probe_done;
		}
	}

	/* The sensor found !! */
	SEMC_LOGD("camera_probe_init SEMC module found %c%c%c%c%c%c%c%c\n",
		g_rom_save_area[0],
		g_rom_save_area[1],
		g_rom_save_area[2],
		g_rom_save_area[3],
		g_rom_save_area[4],
		g_rom_save_area[5],
		g_rom_save_area[6],
		g_rom_save_area[7]);

probe_done:
	semc_camera_sensor_exit();

	SEMC_LOGD("camera_probe_init [E]\n");

	return ret;
}

static int __semc_camera_probe(struct platform_device *pdev)
{
	SEMC_LOGD("_camera_probe [S]\n");

	return msm_camera_drv_start(pdev, semc_camera_probe_init);
}

static struct platform_driver g_msm_camera_driver = {
	.probe = __semc_camera_probe,
	.driver = {
		.name = MSM_CAMERA_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init semc_camera_driver_init(void)
{
	SEMC_LOGD("camera_driver_init [S]\n");

	return platform_driver_register(&g_msm_camera_driver);
}

module_init(semc_camera_driver_init);
