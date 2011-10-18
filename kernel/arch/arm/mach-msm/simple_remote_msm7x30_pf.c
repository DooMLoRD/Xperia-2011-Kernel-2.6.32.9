/* kernel/arch/arm/mach-msm/simple_remote_msm7x30_pf.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * Authors: Takashi Shiina <takashi.shiina@sonyericsson.com>
 *          Tadashi Kubo <tadashi.kubo@sonyericsson.com>
 *          Joachim Holst <joachim.holst@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <asm/atomic.h>
#include <linux/bitops.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/mfd/msm-adie-codec.h>

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <asm/atomic.h>
#include <linux/mutex.h>
#include <linux/bitops.h>
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */

#ifdef CONFIG_CRADLE_SUPPORT
#include <linux/switch.h>
#endif

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
#include <linux/timer.h>
/* Add all includes specifically required by the adaptor here */
#endif

#include "proc_comm.h"
#include <linux/simple_remote.h>
#include <mach/simple_remote_msm7x30_pf.h>

#define HEADSET_BUTTON_ID       0x84
#define HEADSET_BUTTON_PRESS    0x00
#define HEADSET_BUTTON_RELEASE  0xFF

#define IS_BTN_PRESSED BIT(1)
#define DET_INTERRUPT_ENABLED BIT(2)

#ifdef CONFIG_CRADLE_SUPPORT
#define CRADLE_ADC_MAX 1630
#define CRADLE_ADC_MIN 900
#define NUM_DETECT_ITERATIONS 15 /* Detection will run max X * 200ms */
#endif /* CONFIG_CRADLE_SUPPORT */

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
#define CRADLE_VALID_ACC_MAX 1699
#define ACC_CONVERT_DETECT_RETRIES 5
#endif

#define LOCK(x)							\
	do {								\
		dev_dbg(loc_dat->dev, "%s - %d Locking mutex\n", __func__, \
			__LINE__);					\
		mutex_lock(x);						\
	} while (0)							\

#define UNLOCK(x)							\
	do {								\
		dev_dbg(loc_dat->dev, "%s - %d Unlocking mutex\n",	\
		       __func__, __LINE__);				\
		mutex_unlock(x);					\
	} while (0)							\

#define TRY_LOCK(x)							\
	mutex_trylock(x)						\

struct params {
	unsigned int hr_value;
	unsigned int enum_value;
};

static const struct params period_time_vals[] = {
	{
		.hr_value = 1,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_1_CLK_CYCLES,
	},
	{
		.hr_value = 2,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_2_CLK_CYCLES,
	},
	{
		.hr_value = 3,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_3_CLK_CYCLES,
	},
	{
		.hr_value = 4,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_4_CLK_CYCLES,
	},
	{
		.hr_value = 5,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_5_CLK_CYCLES,
	},
	{
		.hr_value = 6,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_6_CLK_CYCLES,
	},
	{
		.hr_value = 7,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_7_CLK_CYCLES,
	},
	{
		.hr_value = 8,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_8_CLK_CYCLES,
	},
	{
		.hr_value = 9,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_9_CLK_CYCLES,
	},
	{
		.hr_value = 10,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_10_CLK_CYCLES,
	},
	{
		.hr_value = 11,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_11_CLK_CYCLES,
	},
	{
		.hr_value = 12,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_12_CLK_CYCLES,
	},
	{
		.hr_value = 13,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_13_CLK_CYCLES,
	},
	{
		.hr_value = 14,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_14_CLK_CYCLES,
	},
	{
		.hr_value = 15,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_15_CLK_CYCLES,
	},
	{
		.hr_value = 16,
		.enum_value = (unsigned int)PM_HSED_PERIOD_TIME_16_CLK_CYCLES,
	},
};

static const struct params hyst_time_vals[] = {
	{
		.hr_value = 1,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_1_CLK_CYCLES
	},
	{
		.hr_value = 2,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_2_CLK_CYCLES
	},
	{
		.hr_value = 3,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_3_CLK_CYCLES
	},
	{
		.hr_value = 4,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_4_CLK_CYCLES
	},
	{
		.hr_value = 5,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_5_CLK_CYCLES
	},
	{
		.hr_value = 6,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_6_CLK_CYCLES
	},
	{
		.hr_value = 7,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_7_CLK_CYCLES
	},
	{
		.hr_value = 8,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_8_CLK_CYCLES
	},
	{
		.hr_value = 9,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_9_CLK_CYCLES
	},
	{
		.hr_value = 10,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_10_CLK_CYCLES
	},
	{
		.hr_value = 11,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_11_CLK_CYCLES
	},
	{
		.hr_value = 12,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_12_CLK_CYCLES
	},
	{
		.hr_value = 13,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_13_CLK_CYCLES
	},
	{
		.hr_value = 14,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_14_CLK_CYCLES
	},
	{
		.hr_value = 15,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_15_CLK_CYCLES
	},
	{
		.hr_value = 16,
		.enum_value = (unsigned int)PM_HSED_HYST_TIME_16_CLK_CYCLES
	},
};

static const struct params period_freq_vals[] = {
	{
		.hr_value = 4,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_256,
	},
	{
		.hr_value = 8,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_128,
	},
	{
		.hr_value = 16,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_64,
	},
	{
		.hr_value = 32,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_32,
	},
	{
		.hr_value = 64,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_16,
	},
	{
		.hr_value = 128,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_8,
	},
	{
		.hr_value = 256,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_4,
	},
	{
		.hr_value = 512,
		.enum_value = (unsigned int)PM_HSED_PERIOD_PRE_DIV_2,
	},
};

static const struct params hyst_freq_vals[] = {
	{
		.hr_value = 8,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_128,
	},
	{
		.hr_value = 16,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_64,
	},
	{
		.hr_value = 32,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_32,
	},
	{
		.hr_value = 64,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_16,
	},
	{
		.hr_value = 128,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_8,
	},
	{
		.hr_value = 256,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_4,
	},
	{
		.hr_value = 512,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_2,
	},
	{
		.hr_value = 1024,
		.enum_value = (unsigned int)PM_HSED_HYST_PRE_DIV_1,
	},
};

#ifdef CONFIG_CRADLE_SUPPORT
struct cradle_data {
	/* Since Work func will sleep, this should be placed on it's own WQ */
	struct delayed_work cradle_work;
	struct workqueue_struct *cradle_queue;

	int detect_iterations;
	int allow_suspend; /* Will be set to -EAGAIN if suspend not allowed */
	int dock_state;

	struct switch_dev cradle_dev;
};
#endif /* CONFIG_CRADLE_SUPPORT */

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
struct acc_convert_data {
	struct mutex acc_detect_lock;
	struct delayed_work irq_work;
	struct delayed_work btn_work;

	struct workqueue_struct *irq_wq;
	struct workqueue_struct *btn_wq;

	atomic_t acc_detected;
	atomic_t docked;
	atomic_t init_done;

	atomic_t hold; /* Is set when btn detection should hold */

	int det_retries;
	int plug_det_retry;

	/* When we use accessory adapter support, we need to intercept some
	 * interrupts. For this purpose, we need to store away the original
	 * IRQ handler so we can call it when required. */
	irq_handler_t real_plug_det_cb_func;
	void *real_plug_det_data;

	irq_handler_t real_btn_det_cb_func;
	void *real_btn_det_data;

	struct timer_list init_timer;
};
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */

struct local_data {
	struct simple_remote_platform_data *jack_pf;
	long unsigned int simple_remote_pf_flags;
	struct mutex lock;

	irq_handler_t simple_remote_btn_det_cb_func;
	void *simple_remote_vad_data;

	atomic_t mic_bias_enable_counter;
	atomic_t hsed_enable_counter;

	unsigned int trigger_level;
	enum hsed_period_pre_div period_pre_div;
	enum hsed_period_time period_time;
	enum hsed_hyst_pre_div hyst_pre_div;
	enum hsed_hyst_time hyst_time;

	struct device *dev;

	u8 mic_bias_first_enable;

	u8 x33_orig_val;
	u8 x34_orig_val;
	u8 x38_orig_val;
	u8 x39_orig_val;

	bool hpamp_enabled;

#ifdef CONFIG_CRADLE_SUPPORT
	struct cradle_data cradle;
#endif /* CONFIG_CRADLE_SUPPORT */

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
	struct acc_convert_data acc_convert;
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */
};

static struct local_data *loc_dat;

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
static int simple_remote_pf_get_current_plug_status(u8 *status);
static int simple_remote_pf_register_hssd_button_interrupt(irq_handler_t func,
							   void *data);
static void simple_remote_pf_unregister_hssd_button_interrupt(void *data);
static int simple_remote_pf_enable_mic_bias(unsigned int enable);
static int simple_remote_pf_enable_button_isr(unsigned int enable);
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */

static inline int simple_remote_pf_get_real_gpio_state(void)
{
	u8 gpiovalue;

	gpiovalue = gpio_get_value(loc_dat->jack_pf->headset_detect_read_pin);
	if (loc_dat->jack_pf->invert_plug_det)
		gpiovalue = !gpiovalue;

	return gpiovalue;
}


#ifdef CONFIG_CRADLE_SUPPORT
static int get_cradle_adc(uint *adc_value)
{
	/* Read twice. First value is not valid. Buffered ADC reads */
	int err = msm_proc_comm(PCOM_OEM_GET_CRADLE_ADC_VALUE, adc_value, 0);
	err = msm_proc_comm(PCOM_OEM_GET_CRADLE_ADC_VALUE, adc_value, 0);

	dev_dbg(loc_dat->dev, "%s - Cradle ADC value = %u\n",
		__func__, *adc_value);

	return err;
}


/* The work function is responsible for detecting if the cradle is attached
 * or not. If it's attached, it will wait until the generic layer is done
 * with plug and accessory detection and then initialize HSED to handle new
 * plug detection. Whend HSED is used for plug detection, we need to intercept
 * the HSED interrupt in order to trigger the generic layers plug detection.
 * That way, the generic layer will perform it's work and correctly detect
 * the accessory that has been inserted into the cradle. */
static void cradle_detect_work(struct work_struct *work)
{
	uint adc_val = 0;
	u8 gpiovalue;

	get_cradle_adc(&adc_val);

	/* Detecting if cradle is connected. */

	gpiovalue = simple_remote_pf_get_real_gpio_state();
	if (adc_val <= CRADLE_ADC_MAX && adc_val >= CRADLE_ADC_MIN &&
				!gpiovalue) {
		dev_dbg(loc_dat->dev, "Cradle detected.\n");
		loc_dat->cradle.dock_state = 1;
		switch_set_state(&loc_dat->cradle.cradle_dev,
				 loc_dat->cradle.dock_state);
	} else {
		dev_dbg(loc_dat->dev, "Cradle not detected\n");
		/* If no cradle is detected here, restart the detection scheme
		 * a bit later just to try again. */
		if (NUM_DETECT_ITERATIONS > loc_dat->cradle.detect_iterations) {
			queue_delayed_work(loc_dat->cradle.cradle_queue,
					   &loc_dat->cradle.cradle_work,
					   msecs_to_jiffies(200));
			loc_dat->cradle.detect_iterations++;
			loc_dat->cradle.dock_state = 0;
			switch_set_state(&loc_dat->cradle.cradle_dev,
					loc_dat->cradle.dock_state);
			return;
		}
	}

	loc_dat->cradle.allow_suspend = 0;
}


static ssize_t cradle_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Cradle : %s\n",
		       switch_get_state(&loc_dat->cradle.cradle_dev) ?
		       "Attached" : "Detached");
}


static int init_cradle_data(struct cradle_data *dat)
{
	int ret = 0;

	dat->cradle_queue = create_singlethread_workqueue("cradle_work");
	INIT_DELAYED_WORK(&dat->cradle_work, cradle_detect_work);

	dat->cradle_dev.name = "dock";
	dat->cradle_dev.print_name = cradle_print_name;

	ret = switch_dev_register(&dat->cradle_dev);
	if (ret)
		dev_err(loc_dat->dev, "switch_dev_register for cradle device"
			" failed.\n");

	switch_set_state(&loc_dat->cradle.cradle_dev,
			 loc_dat->cradle.dock_state);

	return ret;
}

/* Here, we destroy and delete all thing required by cradle support.
 * It's better to create a function for this than to add everything
 * within ifdefs in the destroy function., */
static int destroy_cradle_data(struct cradle_data *dat)
{
	destroy_workqueue(dat->cradle_queue);
	switch_dev_unregister(&dat->cradle_dev);

	return 0;
}

int cradle_suspend(struct platform_device *pdev, pm_message_t state)
{
	return loc_dat->cradle.allow_suspend;
}
#endif /* CONFIG_CRADLE_SUPPORT */

#ifdef CONFIG_ACC_CONVERTER_SUPPORT

#define CRADLE_DETECT_WORK_DELAY 300
static void acc_convert_detect_tmr_func(unsigned long func_data);
static int simple_remote_pf_read_hsd_adc(unsigned int *adc_value);

static void acc_converter_work(struct work_struct *work)
{

	uint adc_val;
	u8 gpiovalue;

	dev_vdbg(loc_dat->dev, "%s - Called\n", __func__);

	atomic_inc(&loc_dat->acc_convert.init_done);

	/* We always need the real value here, so we don't use the supplied
	 * function to get GPIO value. */
	gpiovalue = simple_remote_pf_get_real_gpio_state();

#ifdef CONFIG_CRADLE_SUPPORT
		dev_vdbg(loc_dat->dev,
			 "%s - We have cradle support. Starting cradle "
			 "detection\n", __func__);
		loc_dat->cradle.detect_iterations = 0;
		loc_dat->cradle.allow_suspend = -EAGAIN;

		if (delayed_work_pending(&loc_dat->cradle.cradle_work))
			cancel_delayed_work(
				&loc_dat->cradle.cradle_work);

		queue_delayed_work(loc_dat->cradle.cradle_queue,
				   &loc_dat->cradle.cradle_work,
				   msecs_to_jiffies(500));
#endif

	/* We need to assume that all accessories are
	 * connected via the adaptor in order to make sure that
	 * we can detect removal of accessories in all cases, so
	 * set state to docked if something is inserted.
	 * This will sadly keep MIC Bias enabled all the time
	 * which is not quite good, because if someone inserts
	 * a headphone, we will generate quite a large current
	 * consumption due to shorting MIC Bias to ground.
	 * This is however unavoidable.*/
	if (!gpiovalue) { /* Accessory connected */
		dev_dbg(loc_dat->dev, "%s - Something is connected", __func__);

		/* Enable MIC Bias in order to be able to read
		 * headset ADC value if we are not in docked state */
		if (simple_remote_pf_enable_mic_bias(1)) {
			dev_err(loc_dat->dev,
				"%s - Failed to enable MIC Bias\n",
				__func__);
			goto do_rest;
		}

		if (simple_remote_pf_read_hsd_adc(&adc_val)) {
			dev_err(loc_dat->dev,
				"%s - Failed to read headset ADC. Aborting\n",
				__func__);
			(void)simple_remote_pf_enable_mic_bias(0);
			goto do_rest;
		}

		dev_dbg(loc_dat->dev, "%s - Read ADC value = %u\n",
			__func__, adc_val);

		if (simple_remote_pf_enable_button_isr(1)) {
			dev_err(loc_dat->dev,
				"%s - Failed to enable HSED "
				"interrupt\n", __func__);
		} else {
			atomic_set(&loc_dat->acc_convert.docked, 1);

			if (CRADLE_VALID_ACC_MAX > adc_val) {
				dev_dbg(loc_dat->dev,
					"%s - Accessory detected\n",
				       __func__);
				atomic_set(
					&loc_dat->acc_convert.acc_detected, 1);
			} else {
				dev_dbg(loc_dat->dev,
				       "%s - Accessory not detected\n",
				       __func__);
				atomic_set(
					&loc_dat->acc_convert.acc_detected, 0);

				if (ACC_CONVERT_DETECT_RETRIES >
				    loc_dat->acc_convert.plug_det_retry) {
					loc_dat->acc_convert.plug_det_retry++;
					queue_delayed_work(
						loc_dat->acc_convert.irq_wq,
						&loc_dat->acc_convert.irq_work,
						msecs_to_jiffies(100));
					return;
				} else {
					loc_dat->acc_convert.plug_det_retry = 0;
					goto do_rest;
				}
			}
		}
	} else {
		dev_dbg(loc_dat->dev,
			"%s - Nothing connected. Shutting down,\n",
		       __func__);
		/* If nothing was connected, we need to turn off
		 * MIC Bias and HSED IRQ again */
		atomic_set(&loc_dat->acc_convert.docked, 0);
		atomic_set(&loc_dat->acc_convert.acc_detected, 0);
		simple_remote_pf_enable_mic_bias(0);
		simple_remote_pf_enable_button_isr(0);
	}

do_rest:

	if (1 < atomic_read(&loc_dat->acc_convert.init_done)) {
		dev_dbg(loc_dat->dev, "%s - Passing IRQ to logic layer\n",
		       __func__);
		if (loc_dat->acc_convert.real_plug_det_cb_func)
			loc_dat->acc_convert.real_plug_det_cb_func(1,
				   loc_dat->acc_convert.real_plug_det_data);
		else
			dev_err(loc_dat->dev, "%s - Oops! No IRQ handler "
			       "registered\n", __func__);
	} else {
		dev_dbg(loc_dat->dev,
			"%s - Init not done. Not sendig IRQ to logic"
		       " layer\n", __func__);
		atomic_inc(&loc_dat->acc_convert.init_done);
	}

	atomic_set(&loc_dat->acc_convert.hold, 0);

	dev_vdbg(loc_dat->dev, "%s - Done\n", __func__);
}

/* If we get a plug_detect IRQ, we need to start the workqueue that will handle
 * detection of the docking accessory converter. */
static irqreturn_t acc_convert_irq_plug_det_irq_interceptor(int irq,
							    void *dev_id)
{
	irqreturn_t err = IRQ_HANDLED;

	atomic_set(&loc_dat->acc_convert.hold, 1);
	dev_vdbg(loc_dat->dev, "%s - Called\n", __func__);

	/* This will filter out unwanter IRQ's */
	if (delayed_work_pending(&loc_dat->acc_convert.irq_work))
		cancel_delayed_work(&loc_dat->acc_convert.irq_work);

	queue_delayed_work(loc_dat->acc_convert.irq_wq,
			   &loc_dat->acc_convert.irq_work,
			   msecs_to_jiffies(CRADLE_DETECT_WORK_DELAY));

	dev_vdbg(loc_dat->dev, "%s - Done\n", __func__);

	return err;
}


/* This function is used to re-route a button IRQ to a plug_det IRQ in the
 * generic layer. This is required by cradle support in order to handle plug
 * detection in the cradle */
static void acc_convert_irq_btn_det_work(struct work_struct *work)
{
	/* We need to re-route button detect IRQ's to plug detect IRQ's if
	 * we have a cradle connected. We do this here. */
	unsigned int adc_value;
	u8 gpiovalue;

	dev_vdbg(loc_dat->dev, "%s - Called\n", __func__);

	dev_dbg(loc_dat->dev, "%s - acc_convert.hold = %d\n", __func__,
		(int)atomic_read(&loc_dat->acc_convert.hold));

	while (atomic_read(&loc_dat->acc_convert.hold))
		msleep(10);

	dev_dbg(loc_dat->dev, "%s - acc_convert.hold released\n", __func__);

	if (!TRY_LOCK(&loc_dat->acc_convert.acc_detect_lock)) {
		dev_dbg(loc_dat->dev,
			"%s - Failed to acquire lock. Aborting\n",
			__func__);
		return;
	}

	dev_dbg(loc_dat->dev, "%s - Successfully locked mutex\n", __func__);

	/* We always need the real value here, so we don't use the supplied
	 * function to get GPIO value. */
	gpiovalue = simple_remote_pf_get_real_gpio_state();

	if (simple_remote_pf_read_hsd_adc(&adc_value)) {
		dev_err(loc_dat->dev,
			"%s - Failed to read ADC value. Aborting\n",
			__func__);
		UNLOCK(&loc_dat->acc_convert.acc_detect_lock);
		return;
	}

	dev_dbg(loc_dat->dev,
		"%s - Read ADC value = %u\n", __func__, adc_value);
	dev_dbg(loc_dat->dev, "%s - Max ACC value = %d\n", __func__,
	       CRADLE_VALID_ACC_MAX);


	/* Check plug ADC. If greater than plug max, plug is removed
	 * in that case, call plug_det handler. Otherwise, call btn_det
	 * handler. */

	/* Accessory removed */
	if (CRADLE_VALID_ACC_MAX < adc_value) {
		dev_dbg(loc_dat->dev, "%s - Accessory removed.\n", __func__);
		if (ACC_CONVERT_DETECT_RETRIES >
		    loc_dat->acc_convert.det_retries) {
			dev_dbg(loc_dat->dev, "%s - Retrying detection\n",
				__func__);
			queue_delayed_work(loc_dat->acc_convert.btn_wq,
					   &loc_dat->acc_convert.btn_work,
					   msecs_to_jiffies(100));
			loc_dat->acc_convert.det_retries++;
		} else {
			atomic_set(&loc_dat->acc_convert.acc_detected, 0);
			dev_dbg(loc_dat->dev,
				"%s - Sending IRQ ot logic layer\n", __func__);
			goto out;
		}
	} else if (!atomic_read(&loc_dat->acc_convert.acc_detected)) {
		/* Acessory newly inserted */
		dev_dbg(loc_dat->dev, "%s - Accessory just inserted."
			" Calling plug detect IRQ handler\n", __func__);
		atomic_set(&loc_dat->acc_convert.acc_detected, 1);
		goto out;
	} else if (!gpiovalue) { /* Accessory already inserted */
		dev_dbg(loc_dat->dev, "%s - Accessory inserted. Calling "
			"btn_detect IRQ handler\n", __func__);
		if (loc_dat->simple_remote_btn_det_cb_func)
			loc_dat->simple_remote_btn_det_cb_func(1,
				       loc_dat->simple_remote_vad_data);
		else
			dev_err(loc_dat->dev, "%s - BTN_DET IRQ handler not "
				"registered\n", __func__);
	} else {
		dev_info(loc_dat->dev, "%s - Received interrupt although "
			 " accessory was removed. Spurious interrupt!."
			 " Ignoring IRQ\n", __func__);
	}

	UNLOCK(&loc_dat->acc_convert.acc_detect_lock);

	return;
out:
	loc_dat->acc_convert.det_retries = 0;

	UNLOCK(&loc_dat->acc_convert.acc_detect_lock);

	if (loc_dat->acc_convert.real_plug_det_cb_func)
		loc_dat->acc_convert.real_plug_det_cb_func(1,
			loc_dat->acc_convert.real_plug_det_data);
	else
		dev_err(loc_dat->dev, "%s - Plug det IRQ handler not "
			"registered\n", __func__);

	return;
}

static void init_acc_convert_data(struct acc_convert_data *acc)
{
	mutex_init(&acc->acc_detect_lock);
	acc->irq_wq = create_singlethread_workqueue("acc_irq_work");
	INIT_DELAYED_WORK(&acc->irq_work, acc_converter_work);

	acc->btn_wq = create_singlethread_workqueue("acc_btn_work");
	INIT_DELAYED_WORK(&acc->btn_work, acc_convert_irq_btn_det_work);

	acc->init_timer.function = acc_convert_detect_tmr_func;
	init_timer(&acc->init_timer);
}


static void destroy_acc_convert_data(struct acc_convert_data *acc)
{
	del_timer(&acc->init_timer);
	if (delayed_work_pending(&acc->irq_work))
		cancel_delayed_work_sync(&acc->irq_work);

	if (delayed_work_pending(&acc->btn_work))
		cancel_delayed_work_sync(&acc->btn_work);

	mutex_destroy(&acc->acc_detect_lock);
	destroy_workqueue(acc->irq_wq);
	destroy_workqueue(acc->btn_wq);
}

static void acc_convert_detect_tmr_func(unsigned long func_data)
{
	dev_dbg(loc_dat->dev, "%s - Generic detection done. "
		"Queueing adaptor detection\n", __func__);

	queue_delayed_work(loc_dat->acc_convert.irq_wq,
			   &loc_dat->acc_convert.irq_work, 0);

}
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */

static int get_param_value(const struct params *parm, int parm_size,
			   u8 enum_val, unsigned int value)
{
	int i;

	for (i = 0; i < parm_size; i++) {
		if (enum_val) {
			if (parm[i].hr_value >= value)
				return parm[i].enum_value;
		} else {
			if (value == parm[i].enum_value)
				return parm[i].hr_value;
		}
	}

	return -EINVAL;
}


static int simple_remote_pf_enable_button_isr(unsigned int enable)
{
	int err = 0;

	dev_dbg(loc_dat->dev, "%s - %s HSED interrupt\n", __func__,
		enable ? "Enabling" : "Disabling");

	LOCK(&loc_dat->lock);
	if (enable) {
		if (!atomic_read(&loc_dat->hsed_enable_counter)) {
			err = msm_proc_comm(PCOM_OEM_ENABLE_HSED_ISR,
					    &enable, 0);
			if (!err)
				dev_dbg(loc_dat->dev, "%s - HSED IRQ Enabled\n",
					__func__);
			else
				dev_dbg(loc_dat->dev,
					"%s - Failed to disable HSED IRQ\n",
					__func__);
		}
		if (!err) {
			dev_vdbg(loc_dat->dev,
				 "%s - increasing HSED_ENABLE_COUNTER from "
				 "value %d to ", __func__,
				 (int)atomic_read(
					 &loc_dat->hsed_enable_counter));
			atomic_inc(&loc_dat->hsed_enable_counter);
			dev_vdbg(loc_dat->dev,  "%d\n",
				 (int)atomic_read(
					 &loc_dat->hsed_enable_counter));
		}
	} else {
		if (1 == atomic_read(&loc_dat->hsed_enable_counter)) {
			err = msm_proc_comm(PCOM_OEM_ENABLE_HSED_ISR,
					    &enable, 0);
			dev_dbg(loc_dat->dev, "%s - HSED IRQ Disabled\n",
				__func__);
		}
		if (1 <= atomic_read(&loc_dat->hsed_enable_counter)) {
			dev_vdbg(loc_dat->dev,
				 "%s - Decreasing HSED_ENABLE_COUNTER "
				 "from value %u to ", __func__,
				 atomic_read(&loc_dat->hsed_enable_counter));
			atomic_dec(&loc_dat->hsed_enable_counter);
			dev_vdbg(loc_dat->dev, "%u\n",
				 atomic_read(&loc_dat->hsed_enable_counter));
		} else {
			dev_vdbg(loc_dat->dev,
				 "%s - HSED_ENABLE_COUNTER = %u. No "
				 "need to decrease\n", __func__,
				 atomic_read(&loc_dat->hsed_enable_counter));
		}
	}

	UNLOCK(&loc_dat->lock);
	return err;
}


static int simple_remote_pf_read_hsd_adc(unsigned int *adc_value)
{
	int err;

	LOCK(&loc_dat->lock);

	/* Read twice. First value is not valid. Buffered ADC reads */
	msm_proc_comm(PCOM_OEM_GET_HEADSET_ADC_VALUE, adc_value, 0);
	err = msm_proc_comm(PCOM_OEM_GET_HEADSET_ADC_VALUE, adc_value, 0);

	UNLOCK(&loc_dat->lock);

	dev_dbg(loc_dat->dev, "%s - PHF Adc value = %u\n", __func__,
		*adc_value);

	return err;
}


static int simple_remote_pf_enable_mic_bias(unsigned int enable)
{
	int err = 0;

	LOCK(&loc_dat->lock);

	dev_dbg(loc_dat->dev, "%s - %s MIC Bias\n", __func__,
		enable ? "Enabling" : "Disabling");

	dev_vdbg(loc_dat->dev, "%s - MIC_BIAS_COUNTER = %d\n", __func__,
		 (int)atomic_read(&loc_dat->mic_bias_enable_counter));

	if (enable) {
		if (!atomic_read(&loc_dat->mic_bias_enable_counter)) {
			err = pmic_hsed_enable(loc_dat->jack_pf->controller,
					       PM_HSED_ENABLE_ALWAYS);
			if (err)
				dev_err(loc_dat->dev,
					"%s - Failed to enable MIC "
					"Bias\n", __func__);
			else
				dev_dbg(loc_dat->dev, "%s - MIC Bias enabled\n",
					__func__);
		}
		if (!err) {
			dev_vdbg(loc_dat->dev,
				"%s - Increasing MIC_BIAS_COUNTER\n",
				__func__);
			atomic_inc(&loc_dat->mic_bias_enable_counter);
			dev_vdbg(loc_dat->dev,
				 "%s - MIC_BIAS_COUNTER = %u\n", __func__,
				 atomic_read(&loc_dat->
					     mic_bias_enable_counter));
		}
	} else {
		if (1 == atomic_read(&loc_dat->mic_bias_enable_counter)) {
			err = pmic_hsed_enable(loc_dat->jack_pf->controller,
					       PM_HSED_ENABLE_OFF);
			if (!err)
				dev_dbg(loc_dat->dev,
					"%s - MIC Bias disabled\n",
					__func__);
			else
				dev_dbg(loc_dat->dev,
					"%s - Failed to disable MIC Bias\n",
					__func__);
		}
		if (1 <= atomic_read(&loc_dat->mic_bias_enable_counter)) {
			dev_vdbg(loc_dat->dev,
				"%s - Decreasing MIC_BIAS_COUNTER\n",
				__func__);
			atomic_dec(&loc_dat->mic_bias_enable_counter);
			dev_vdbg(loc_dat->dev, "%s - MIC_BIAS_COUNTER = %d\n",
			       __func__,
			       (int)
			       atomic_read(&loc_dat->mic_bias_enable_counter));
		} else {
			dev_vdbg(loc_dat->dev, "%s - No need to decrease "
				"MIC_BIAS_COUNTER\n", __func__);
			dev_vdbg(loc_dat->dev, "%s - MIC_BIAS_COUNTER = %u\n",
				 __func__,
				 atomic_read(&loc_dat->mic_bias_enable_counter)
				);
		}
	}

	if (err)
		dev_err(loc_dat->dev, "Unable to toggle MIC Bias\n");

	UNLOCK(&loc_dat->lock);

	return err;
}


static int simple_remote_pf_set_period_freq(unsigned int value)
{
	int err = -EINVAL;
	int ret_val = get_param_value(period_freq_vals,
				ARRAY_SIZE(period_freq_vals), 1, value);

	if (ret_val < 0)
		return ret_val;

	err = pmic_hsed_set_period(loc_dat->jack_pf->controller,
				   (enum hsed_period_pre_div)ret_val,
				   loc_dat->period_time);

	if (!err)
		loc_dat->period_pre_div = (enum hsed_period_pre_div)ret_val;
	else
		dev_err(loc_dat->dev, "%s - Failed to set PMIC value (%u)\n",
			__func__, ret_val);
	return err;
}


static int simple_remote_pf_set_period_time(unsigned int value)
{
	int err = -EINVAL;
	int ret_val = get_param_value(period_time_vals,
				ARRAY_SIZE(period_time_vals), 1, value);

	if (ret_val < 0)
		return ret_val;

	err = pmic_hsed_set_period(loc_dat->jack_pf->controller,
				   loc_dat->period_pre_div,
				   (enum hsed_period_time)ret_val);

	if (!err)
		loc_dat->period_time = (enum hsed_period_time)ret_val;

	return err;
}


static int simple_remote_pf_set_hysteresis_freq(unsigned int value)
{
	int err = -EINVAL;
	int ret_val = get_param_value(hyst_freq_vals,
				      ARRAY_SIZE(hyst_freq_vals),
				      1, value);

	if (ret_val < 0)
		return ret_val;

	err = pmic_hsed_set_hysteresis(loc_dat->jack_pf->controller,
				       (enum hsed_hyst_pre_div)ret_val,
				       loc_dat->hyst_time);

	if (!err)
		loc_dat->hyst_pre_div = (enum hsed_hyst_pre_div)ret_val;

	return err;
}


static int simple_remote_pf_set_hysteresis_time(unsigned int value)
{
	int err = -EINVAL;
	int ret_val = get_param_value(hyst_time_vals,
				      ARRAY_SIZE(hyst_time_vals),
				      1, value);

	if (ret_val < 0)
		return ret_val;

	err = pmic_hsed_set_hysteresis(loc_dat->jack_pf->controller,
				       loc_dat->hyst_pre_div,
				       (enum hsed_hyst_time)ret_val);

	if (!err)
		loc_dat->hyst_time = (enum hsed_hyst_time)ret_val;

	return err;
}


static int simple_remote_pf_set_trig_level(unsigned int value)
{
	int ret = -EINVAL;

	if (value >= 200 && value <= 1700) {
		loc_dat->trigger_level = value;
		ret = pmic_hsed_set_current_threshold(
			loc_dat->jack_pf->controller,
			PM_HSED_SC_SWITCH_TYPE,
			loc_dat->trigger_level);
	} else {
		dev_warn(loc_dat->dev, "Trig level out of range\n");
	}

	return ret;
}


static int simple_remote_pf_get_period_freq(unsigned int *value)
{
	int val = get_param_value(period_freq_vals,
				  ARRAY_SIZE(period_freq_vals), 0,
				  (unsigned int)loc_dat->period_pre_div);

	if (0 > val)
		return val;

	*value = (unsigned int)val;

	return 0;
}


static int simple_remote_pf_get_period_time(unsigned int *value)
{
	int val = get_param_value(period_time_vals,
				  ARRAY_SIZE(period_time_vals), 0,
				  (unsigned int)loc_dat->period_time);

	if (0 > val)
		return val;

	*value = (unsigned int)val;

	return 0;
}


static int simple_remote_pf_get_hysteresis_freq(unsigned int *value)
{
	int val = get_param_value(hyst_freq_vals, ARRAY_SIZE(hyst_freq_vals),
				  0, (unsigned int)loc_dat->hyst_pre_div);

	if (0 > val)
		return val;

	*value = (unsigned int)val;

	return 0;
}


static int simple_remote_pf_get_hysteresis_time(unsigned int *value)
{
	int val = get_param_value(hyst_time_vals, ARRAY_SIZE(hyst_time_vals),
				  0, (unsigned int)loc_dat->hyst_time);

	if (0 > val)
		return val;

	*value = (unsigned int)val;

	return 0;
}


static int simple_remote_pf_get_trig_level(unsigned int *value)
{
	*value = loc_dat->trigger_level;
	return 0;
}


static int simple_remote_pf_get_current_plug_status(u8 *status)
{
#ifdef CONFIG_ACC_CONVERTER_SUPPORT
	/* Only send false values if we have detected a converter
	 * accessory */
	if (atomic_read(&loc_dat->acc_convert.docked)) {
		if (atomic_read(&loc_dat->acc_convert.acc_detected)) {
			dev_dbg(loc_dat->dev, "%s - Reporting GPIO status 0\n",
			       __func__);
			*status = 0;
			return 0;
		} else {
			dev_dbg(loc_dat->dev, "%s - Reporting GPIO status 1\n",
			       __func__);
			*status = 1;
			return 1;
		}
	}

	/* This timer will kick in once the generic layer stops  requesting
	 * GPIO values from headset. This will then trigger the work function
	 * to try and detect if an adaptor is attached. This is only run on
	 * system startup */
	if (!atomic_read(&loc_dat->acc_convert.init_done)) {
		dev_vdbg(loc_dat->dev, "%s - Resetting timer\n", __func__);
		mod_timer(&loc_dat->acc_convert.init_timer,
			  jiffies + msecs_to_jiffies(300));
	}

#endif /* CONFIG_ACC_CONVERTER_SUPPORT */

	*status = simple_remote_pf_get_real_gpio_state();

	dev_dbg(loc_dat->dev,
		"%s - Read GPIO status = %u\n", __func__, *status);

	return 0;
}


static int simple_remote_pf_enable_vregs(u8 enable)
{
	int i;
	dev_dbg(loc_dat->dev, "%s - %s vregs\n", __func__,
		enable ? "Enabling" : "Disabling");

	if (enable) {
		/* Turn them on in one direction */
		for (i = 0; i < loc_dat->jack_pf->num_regs; i++) {
			if (vreg_enable(loc_dat->jack_pf->regs[i].reg)) {
				dev_err(loc_dat->dev,
					"%s - Failed to enable regulator %s\n",
					__func__,
					loc_dat->jack_pf->regs[i].name);
				return -EFAULT;
			}
		}
	} else {
		/* And turn them off the other way */
		for (i = loc_dat->jack_pf->num_regs - 1; i >= 0; i--) {
			if (vreg_disable(loc_dat->jack_pf->regs[i].reg)) {
				dev_err(loc_dat->dev,
					"%s - Failed to enable regulator %s\n",
					__func__,
					loc_dat->jack_pf->regs[i].name);
				return -EFAULT;
			}
		}
	}

	return 0;
}


static int simple_remote_pf_enable_hp_amp(u8 enable)
{
	int rc = 0;
	struct marimba config = { .mod_id = MARIMBA_SLAVE_ID_CDC };
	u8 x34_pa_enable = 0;
	u8 x33_pa_enable = 0;
	u8 x38_pa_cfg = 0;
	u8 x39_pa_cfg = 0;

	dev_dbg(loc_dat->dev, "%s - %s HP amp\n", __func__,
	       enable ? "Enabling" : "Disabling");

	LOCK(&loc_dat->lock);
	if (enable) {
		if (loc_dat->hpamp_enabled)
			goto same_state;

		simple_remote_pf_enable_vregs(1);

		adie_codec_powerup(1);

		if (0 > marimba_read(&config, 0x33,
				     &loc_dat->x33_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to read register 0x33\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_read(&config, 0x34,
				     &loc_dat->x34_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to read register 0x34\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_read(&config, 0x38,
				     &loc_dat->x38_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to read register 0x38\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_read(&config, 0x39,
				     &loc_dat->x39_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to read register 0x39\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		dev_dbg(loc_dat->dev, "%s - reg 0x33 = 0x%02x\n", __func__,
			loc_dat->x33_orig_val);
		dev_dbg(loc_dat->dev, "%s - reg 0x34 = 0x%02x\n", __func__,
			loc_dat->x34_orig_val);
		dev_dbg(loc_dat->dev, "%s - reg 0x38 = 0x%02X\n", __func__,
			loc_dat->x38_orig_val);
		dev_dbg(loc_dat->dev, "%s - reg 0x39 = 0x%02X\n", __func__,
			loc_dat->x39_orig_val);

		/* Enable the HP AMP */
		x33_pa_enable = (loc_dat->x33_orig_val | 0x80);
		x34_pa_enable = (loc_dat->x34_orig_val | 0xF0);
		x38_pa_cfg = (loc_dat->x38_orig_val | 0x02);
		x39_pa_cfg = (loc_dat->x39_orig_val | 0x02);

		if (0 > marimba_write(&config, 0x38, &x38_pa_cfg, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x38\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_write(&config, 0x39, &x39_pa_cfg, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x39\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_write(&config, 0x33, &x33_pa_enable, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x33\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_write(&config, 0x34, &x34_pa_enable, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x34\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		/* wait a bit to give the HP amp time to start */
		msleep(20);
		loc_dat->hpamp_enabled = true;
	} else {
		if (!loc_dat->hpamp_enabled)
			goto same_state;

		if (0 > marimba_write(&config, 0x33,
				      &loc_dat->x33_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x33\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_write(&config, 0x34,
				      &loc_dat->x34_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x34\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_write(&config, 0x38,
				      &loc_dat->x38_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x38\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		if (0 > marimba_write(&config, 0x39,
				      &loc_dat->x39_orig_val, 1)) {
			dev_warn(loc_dat->dev,
				 "%s - Failed to write register 0x39\n",
				 __func__);
			rc = -EIO;
			goto error;
		}

		adie_codec_powerup(0);
		simple_remote_pf_enable_vregs(0);
		loc_dat->hpamp_enabled = false;
	}

same_state:
	dev_info(loc_dat->dev,
		"%s - HP Amp already %s\n", __func__,
		enable ? "enabled" : "disabled");

error:
	if (rc < 0)
		dev_err(loc_dat->dev,
			"%s - Failed to fully %s HP Amp\n", __func__,
			enable ? "enable" : "disable");

	UNLOCK(&loc_dat->lock);

	return rc;
}


static int simple_remote_pf_enable_alternate_mode(u8 enable)
{

	if (0 <= loc_dat->jack_pf->headset_mode_switch_pin) {
		u8 status;
		dev_dbg(loc_dat->dev, "%s - We support headset mode switch\n",
			__func__);
		status = gpio_get_value(
			loc_dat->jack_pf->headset_mode_switch_pin);
		gpio_set_value(loc_dat->jack_pf->headset_mode_switch_pin,
			       !status);
		return 0;
	}

	dev_info(loc_dat->dev,
		 "%s - This hardware doesn't support headset CTIA/OMTP mode "
		 "switch\n", __func__);
	return -ENODEV;
}


static int simple_remote_pf_register_plug_detect_interrupt(irq_handler_t func,
							   void *data)
{
	int err = -EALREADY;
	int irq = 0;
	irq_handler_t regfunc = func;

	if (test_bit(DET_INTERRUPT_ENABLED, &loc_dat->simple_remote_pf_flags))
		return err;

	dev_dbg(loc_dat->dev,
		"%s - Interrupt not enabled. Enabling\n", __func__);

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
	dev_dbg(loc_dat->dev, "%s - We have support for accessory converter\n",
		__func__);
	/* Swap IRQ handlers so that we intercept the plug detect IRQ
	 * in order to start the cradle detect functionality. Also
	 * store the original function so that we can forward the
	 * IRQ to this handler. */
	regfunc = acc_convert_irq_plug_det_irq_interceptor;
	dev_dbg(loc_dat->dev, "%s - regfunc = local function\n", __func__);
	loc_dat->acc_convert.real_plug_det_cb_func = func;
	loc_dat->acc_convert.real_plug_det_data = data;
	dev_dbg(loc_dat->dev, "%s - Stored away real IRQ handler\n", __func__);
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */

	irq = gpio_to_irq(loc_dat->jack_pf->headset_detect_read_pin);
	if (0 <= irq) {
		err = request_threaded_irq(irq, NULL, regfunc,
					   IRQF_TRIGGER_FALLING |
					   IRQF_TRIGGER_RISING,
					   "simple_remote_plug_detect",
					   data);

		if (err) {
#ifdef CONFIG_ACC_CONVERTER_SUPPORT
			loc_dat->acc_convert.real_plug_det_cb_func = NULL;
			loc_dat->acc_convert.real_plug_det_data = NULL;
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */
			dev_crit(loc_dat->dev, "Failed to subscribe to plug "
				 "detect interrupt\n");
			return err;
		}
	} else {
#ifdef CONFIG_ACC_CONVERTER_SUPPORT
		loc_dat->acc_convert.real_plug_det_cb_func = NULL;
		loc_dat->acc_convert.real_plug_det_data = NULL;
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */
		dev_crit(loc_dat->dev, "Failed to register interrupt for GPIO "
			 "(%d). GPIO Does not exist\n",
			 loc_dat->jack_pf->headset_detect_read_pin);
		return irq;
	}

	/*
	 * Setting interrupt enabled here, will in worst case generate
	 * a "unmatched irq_wake" print in the kernel log when shutting
	 * down the system, but at least some detection will work.
	 */
	set_bit(DET_INTERRUPT_ENABLED, &loc_dat->simple_remote_pf_flags);

	err = enable_irq_wake(irq);
	if (err)
		dev_crit(loc_dat->dev,
			 "Failed to enable wakeup on interrupt\n");
	else
		dev_dbg(loc_dat->dev,
			"%s - Interrupt successfully registered\n",
			__func__);
	return err;
}


static void simple_remote_pf_unregister_plug_detect_interrupt(void *data)
{
	int irq;
	void *ldat = data;
	if (!test_bit(DET_INTERRUPT_ENABLED, &loc_dat->simple_remote_pf_flags))
		return;

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
	ldat = loc_dat->acc_convert.real_plug_det_data;
#endif /* CONFIG_ACC_CONVERTER_SUPPORT */

	irq = gpio_to_irq(loc_dat->jack_pf->headset_detect_read_pin);
	if (0 <= irq) {
		disable_irq_wake(irq);
		free_irq(irq, ldat);
		clear_bit(DET_INTERRUPT_ENABLED,
			  &loc_dat->simple_remote_pf_flags);
	} else {
		dev_crit(loc_dat->dev, "Failed to disable plug detect interrupt"
			 ". GPIO (%d) does not exist\n",
			 loc_dat->jack_pf->headset_detect_read_pin);
	}
}


static int simple_remote_pf_register_hssd_button_interrupt(irq_handler_t func,
						    void *data)
{
	int err;

	dev_dbg(loc_dat->dev, "%s - Setting btn_det IRQ handler\n", __func__);


	if (loc_dat->simple_remote_btn_det_cb_func) {
		dev_info(loc_dat->dev, "%s - Interrupt already registered\n",
		       __func__);
		return -EFAULT;
	}

	LOCK(&loc_dat->lock);
	loc_dat->simple_remote_btn_det_cb_func = func;
	loc_dat->simple_remote_vad_data = data;
	UNLOCK(&loc_dat->lock);

	err = simple_remote_pf_enable_button_isr(1);

	if (err)
		dev_err(loc_dat->dev, "%s - Failed to enable HSED interrupt\n",
			__func__);

	return err;

}


static void simple_remote_pf_unregister_hssd_button_interrupt(void *data)
{
	LOCK(&loc_dat->lock);
	loc_dat->simple_remote_btn_det_cb_func = NULL;
	loc_dat->simple_remote_vad_data = NULL;
	UNLOCK(&loc_dat->lock);

	simple_remote_pf_enable_button_isr(0);

}


void simple_remote_pf_button_handler(uint32_t key, uint32_t event)
{
	dev_dbg(loc_dat->dev, "%s - Called\n", __func__);
	dev_dbg(loc_dat->dev, "%s - key = 0x%X, event = 0x%X\n", __func__,
		 key, event);

#ifndef CONFIG_ACC_CONVERTER_SUPPORT
	/* Ignore this check for adaptor support. We only start a work
	 * thread if adaptor is supported. Check is done elsewhere */
	if (loc_dat->simple_remote_btn_det_cb_func == NULL) {
		dev_err(loc_dat->dev, "%s - Button callback not registered\n",
			__func__);
		return;
	}
#endif

	if (HEADSET_BUTTON_ID != key)
		return;


	switch (event) {
	case HEADSET_BUTTON_PRESS:
		if (!test_bit(IS_BTN_PRESSED,
			      &loc_dat->simple_remote_pf_flags)) {
			dev_dbg(loc_dat->dev, "%s - HEADSET_BUTTON_PRESS\n",
				__func__);
			set_bit(IS_BTN_PRESSED,
				&loc_dat->simple_remote_pf_flags);
			}
			break;
	case HEADSET_BUTTON_RELEASE:
		dev_dbg(loc_dat->dev, "%s - HEADSET_BUTTON_RELEASE\n",
			__func__);
		clear_bit(IS_BTN_PRESSED, &loc_dat->simple_remote_pf_flags);
		break;
	}

#ifdef CONFIG_ACC_CONVERTER_SUPPORT

	if (!delayed_work_pending(&loc_dat->acc_convert.btn_work)) {
		dev_dbg(loc_dat->dev, "%s - Queueing button work\n", __func__);

		queue_delayed_work(loc_dat->acc_convert.btn_wq,
				   &loc_dat->acc_convert.btn_work,
				   msecs_to_jiffies(40));
	} else {
		dev_dbg(loc_dat->dev, "%s - Work pending. Ignoring IRQ\n",
			__func__);
	}
#else
	dev_dbg(loc_dat->dev,
		"%s - Sending interrupt to registered IRQ handler\n",
		__func__);
	loc_dat->simple_remote_btn_det_cb_func(1,
		loc_dat->simple_remote_vad_data);
#endif
}


static struct simple_remote_pf_interface interface = {
	.read_hs_adc = simple_remote_pf_read_hsd_adc,
	.enable_mic_bias = simple_remote_pf_enable_mic_bias,
	.get_current_plug_status = simple_remote_pf_get_current_plug_status,
	.enable_alternate_adc_mode = simple_remote_pf_enable_hp_amp,
	.enable_alternate_headset_mode = simple_remote_pf_enable_alternate_mode,
	.set_period_freq = simple_remote_pf_set_period_freq,
	.set_period_time = simple_remote_pf_set_period_time,
	.set_hysteresis_freq = simple_remote_pf_set_hysteresis_freq,
	.set_hysteresis_time = simple_remote_pf_set_hysteresis_time,
	.set_trig_level = simple_remote_pf_set_trig_level,

	.get_period_freq = simple_remote_pf_get_period_freq,
	.get_period_time = simple_remote_pf_get_period_time,
	.get_hysteresis_freq = simple_remote_pf_get_hysteresis_freq,
	.get_hysteresis_time = simple_remote_pf_get_hysteresis_time,
	.get_trig_level = simple_remote_pf_get_trig_level,

	.register_plug_detect_interrupt =
		simple_remote_pf_register_plug_detect_interrupt,

	.unregister_plug_detect_interrupt =
		simple_remote_pf_unregister_plug_detect_interrupt,

	.register_hssd_button_interrupt =
		simple_remote_pf_register_hssd_button_interrupt,

	.unregister_hssd_button_interrupt =
		simple_remote_pf_unregister_hssd_button_interrupt,
};


static struct platform_device simple_remote_device = {
	.name = SIMPLE_REMOTE_NAME,
	.dev = {
		.platform_data = &interface,
	},
};


static int simple_remote_pf_probe(struct platform_device *pdev)
{
	int ret;
	struct platform_device *n_pdev = &simple_remote_device;

	loc_dat = kzalloc(sizeof(*loc_dat), GFP_KERNEL);
	if (!loc_dat)
		return -ENOMEM;

	loc_dat->jack_pf = pdev->dev.platform_data;
	loc_dat->dev = &pdev->dev;

	ret = loc_dat->jack_pf->initialize(loc_dat->jack_pf);
	if (ret)
		goto out;

	mutex_init(&loc_dat->lock);

	(void)simple_remote_pf_enable_mic_bias(0);
	(void)simple_remote_pf_enable_button_isr(0);

	ret = platform_add_devices(&n_pdev, 1);
	if (ret)
		goto out;

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
	init_acc_convert_data(&loc_dat->acc_convert);
#endif

#ifdef CONFIG_CRADLE_SUPPORT
	if (init_cradle_data(&loc_dat->cradle))
		dev_err(loc_dat->dev, "Failed to initialize cradle support."
			"cradle will not work\n");
	else
#endif /* CONFIG_CRADLE_SUPPORT */
		dev_info(loc_dat->dev, "Successfully registered\n");

	return ret;

out:
	kfree(loc_dat);
	dev_err(&pdev->dev, "Failed to register driver\n");
	return ret;
}


static int simple_remote_pf_remove(struct platform_device *pdev)
{
	struct platform_device *n_pdev = &simple_remote_device;

	(void)simple_remote_pf_enable_mic_bias(0);
	(void)simple_remote_pf_enable_button_isr(0);
	mutex_destroy(&loc_dat->lock);

	platform_device_unregister(n_pdev);

	loc_dat->jack_pf->deinitialize(loc_dat->jack_pf);

#ifdef CONFIG_CRADLE_SUPPORT
	destroy_cradle_data(&loc_dat->cradle);
#endif /* CONFIG_CRADLE_SUPPORT */

#ifdef CONFIG_ACC_CONVERTER_SUPPORT
	destroy_acc_convert_data(&loc_dat->acc_convert);
#endif

	kfree(loc_dat);
	return 0;
}


static struct platform_driver simple_remote_pf = {
	.probe		= simple_remote_pf_probe,
	.remove		= simple_remote_pf_remove,
	.driver		= {
		.name		= SIMPLE_REMOTE_PF_NAME,
		.owner		= THIS_MODULE,
	},
#ifdef CONFIG_CRADLE_SUPPORT
	.suspend = cradle_suspend,
#endif /* CONFIG_CRADLE_SUPPORT */
};


static int __init simple_remote_pf_init(void)
{
	return platform_driver_register(&simple_remote_pf);
}


static void __exit simple_remote_pf_exit(void)
{
	platform_driver_unregister(&simple_remote_pf);
}

module_init(simple_remote_pf_init);
module_exit(simple_remote_pf_exit);

MODULE_AUTHOR("Joachim Holst, Takashi Shiina, Tadashi Kubo");
MODULE_DESCRIPTION("3.5mm audio jack platform driver");
MODULE_LICENSE("GPL");
