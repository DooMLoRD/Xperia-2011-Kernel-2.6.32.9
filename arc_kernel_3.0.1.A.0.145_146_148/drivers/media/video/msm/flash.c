/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/hrtimer.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#ifdef CONFIG_MACH_SEMC_ZEUS
#include <linux/delay.h>
#endif /* CONFIG_MACH_SEMC_ZEUS */
#include <mach/gpio.h>

struct timer_list timer_flash;

static int msm_camera_flash_pwm(
	struct msm_camera_sensor_flash_pwm *pwm,
	unsigned led_state)
{
	int rc = 0;
	int PWM_PERIOD = USEC_PER_SEC / pwm->freq;

	static struct pwm_device *flash_pwm;

	if (!flash_pwm) {
		flash_pwm = pwm_request(pwm->channel, "camera-flash");
		if (flash_pwm == NULL || IS_ERR(flash_pwm)) {
			pr_err("%s: FAIL pwm_request(): flash_pwm=%p\n",
			       __func__, flash_pwm);
			flash_pwm = NULL;
			return -ENXIO;
		}
	}

	switch (led_state) {
	case MSM_CAMERA_LED_LOW:
		rc = pwm_config(flash_pwm,
			(PWM_PERIOD/pwm->max_load)*pwm->low_load,
			PWM_PERIOD);
		if (rc >= 0)
			rc = pwm_enable(flash_pwm);
		break;

	case MSM_CAMERA_LED_HIGH:
		rc = pwm_config(flash_pwm,
			(PWM_PERIOD/pwm->max_load)*pwm->high_load,
			PWM_PERIOD);
		if (rc >= 0)
			rc = pwm_enable(flash_pwm);
		break;

	case MSM_CAMERA_LED_OFF:
		pwm_disable(flash_pwm);
		break;

	default:
		rc = -EFAULT;
		break;
	}

	return rc;
}

int msm_camera_flash_pmic(
	struct msm_camera_sensor_flash_pmic *pmic,
	unsigned led_state)
{
	int rc = 0;

	switch (led_state) {
	case MSM_CAMERA_LED_OFF:
		rc = pmic->pmic_set_current(pmic->led_src_1, 0);
		break;

	case MSM_CAMERA_LED_LOW:
		rc = pmic->pmic_set_current(pmic->led_src_1,
				pmic->low_current);
		break;

	case MSM_CAMERA_LED_HIGH:
		if (pmic->num_of_src == 2) {
			rc = pmic->pmic_set_current(pmic->led_src_1,
				pmic->high_current);
			rc = pmic->pmic_set_current(pmic->led_src_2,
				pmic->high_current);
		} else
			rc = pmic->pmic_set_current(pmic->led_src_1,
				pmic->high_current);
		break;

	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);

	return rc;
}

#ifdef CONFIG_MACH_SEMC_ZEUS
/**
 * Access GPIO
*/
static int32_t gpio_access(unsigned gpio_pin, int dir)
{
	int rc = 0;

	rc = gpio_request(gpio_pin, "camera_flash");
	if (!rc) {
		gpio_direction_output(gpio_pin, dir);
	}
	gpio_free(gpio_pin);

	return rc;
}

int msm_camera_flash_gpio(
	struct gpio_led_platform_data *gpio_leds,
	unsigned led_state)
{
	int rc = 0;
	int led_num = 0;

	switch (led_state) {
	case MSM_CAMERA_LED_OFF:
		for (led_num = 0; led_num < gpio_leds->num_leds; led_num++) {
			rc |= gpio_access(gpio_leds->leds[led_num].gpio,
					gpio_leds->leds[led_num].active_low);
			udelay(2);
		}
		break;

	case MSM_CAMERA_LED_LOW:
		rc = gpio_access(gpio_leds->leds[0].gpio,
				!gpio_leds->leds[0].active_low);
		udelay(2);
		rc = gpio_access(gpio_leds->leds[1].gpio,
				gpio_leds->leds[1].active_low);
		udelay(2);
		break;

	case MSM_CAMERA_LED_HIGH:
		rc = gpio_access(gpio_leds->leds[0].gpio,
				gpio_leds->leds[0].active_low);
		udelay(2);
		rc = gpio_access(gpio_leds->leds[1].gpio,
				!gpio_leds->leds[1].active_low);
		udelay(2);
		break;

	default:
		rc = -EFAULT;
		break;
	}

	CDBG("flash_set_led_state: return %d\n", rc);

	return rc;
}
#endif /* CONFIG_MACH_SEMC_ZEUS */

int32_t msm_camera_flash_set_led_state(
	struct msm_camera_sensor_flash_data *fdata, unsigned led_state)
{
	int32_t rc;

	CDBG("flash_set_led_state: %d flash_sr_type=%d\n", led_state,
	    fdata->flash_src->flash_sr_type);

	if (fdata->flash_type != MSM_CAMERA_FLASH_LED)
		return -ENODEV;

	switch (fdata->flash_src->flash_sr_type) {
	case MSM_CAMERA_FLASH_SRC_PMIC:
		rc = msm_camera_flash_pmic(&fdata->flash_src->_fsrc.pmic_src,
			led_state);
		break;

	case MSM_CAMERA_FLASH_SRC_PWM:
		rc = msm_camera_flash_pwm(&fdata->flash_src->_fsrc.pwm_src,
			led_state);
		break;
#ifdef CONFIG_MACH_SEMC_ZEUS
	case MSM_CAMERA_FLASH_SRC_LED:
			rc = msm_camera_flash_gpio(fdata->flash_src->_fsrc.gpio_led_src,
				led_state);
			break;
#endif /* CONFIG_MACH_SEMC_ZEUS */
	default:
		rc = -ENODEV;
		break;
	}

	return rc;
}

static int msm_strobe_flash_xenon_charge(
		int32_t flash_charge, int32_t charge_enable)
{
	gpio_direction_output(flash_charge, charge_enable);
	/* add timer for the recharge */
	add_timer(&timer_flash);

	return 0;
}

static void strobe_flash_xenon_recharge_handler(unsigned long data)
{
	unsigned long flags;
	struct msm_camera_sensor_strobe_flash_data *sfdata =
		(struct msm_camera_sensor_strobe_flash_data *)data;

	spin_lock_irqsave(&sfdata->timer_lock, flags);
	msm_strobe_flash_xenon_charge(sfdata->flash_charge, 1);
	spin_unlock_irqrestore(&sfdata->timer_lock, flags);

	return;
}

static irqreturn_t strobe_flash_charge_ready_irq(int irq_num, void *data)
{
	struct msm_camera_sensor_strobe_flash_data *sfdata =
		(struct msm_camera_sensor_strobe_flash_data *)data;

	/* put the charge signal to low */
	gpio_direction_output(sfdata->flash_charge, 0);

	return IRQ_HANDLED;
}

static int msm_strobe_flash_xenon_init(
	struct msm_camera_sensor_strobe_flash_data *sfdata)
{
	int rc = 0;

	rc = request_irq(sfdata->irq, strobe_flash_charge_ready_irq,
			IRQF_TRIGGER_FALLING, "charge_ready", sfdata);
	if (rc < 0) {
		pr_err("%s: request_irq failed %d\n", __func__, rc);
		return rc;
	}
	rc = gpio_request(sfdata->flash_charge, "charge");
	if (rc < 0) {
		pr_err("%s: gpio_request failed\n", __func__);
		free_irq(sfdata->irq, sfdata);
		return rc;
	}
	spin_lock_init(&sfdata->timer_lock);
	/* setup timer */
	init_timer(&timer_flash);
	timer_flash.function = strobe_flash_xenon_recharge_handler;
	timer_flash.data = (unsigned long)sfdata;
	timer_flash.expires = jiffies +
		msecs_to_jiffies(sfdata->flash_recharge_duration);

	return rc;
}

static int msm_strobe_flash_xenon_release
	(struct msm_camera_sensor_strobe_flash_data *sfdata)
{
	free_irq(sfdata->irq, sfdata);
	gpio_free(sfdata->flash_charge);
	del_timer_sync(&timer_flash);
	return 0;
}

static void msm_strobe_flash_xenon_fn_init
	(struct msm_strobe_flash_ctrl *strobe_flash_ptr)
{
	strobe_flash_ptr->strobe_flash_init =
				msm_strobe_flash_xenon_init;
	strobe_flash_ptr->strobe_flash_charge =
				msm_strobe_flash_xenon_charge;
	strobe_flash_ptr->strobe_flash_release =
				msm_strobe_flash_xenon_release;
}

int msm_strobe_flash_init(struct msm_sync *sync, uint32_t sftype)
{
	int rc = 0;
	switch (sftype) {
	case MSM_CAMERA_STROBE_FLASH_XENON:
		msm_strobe_flash_xenon_fn_init(&sync->sfctrl);
		rc = sync->sfctrl.strobe_flash_init(
			sync->sdata->strobe_flash_data);
		break;
	default:
		rc = -ENODEV;
	}
	return rc;
}
