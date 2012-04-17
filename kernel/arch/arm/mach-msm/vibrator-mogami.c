/*
 * Copyright (C) 2009 Sony Ericsson Mobile Corporation.
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>

#define PM_LIBPROG 0x30000061

#if defined(CONFIG_ARCH_QSD8X50) || defined(CONFIG_ARCH_MSM7X30)
#define PM_VIB_MOT_SET_VOLT_PROC 22
#else
#define PM_VIB_MOT_SET_VOLT_PROC 21
#endif
#define PM_VIBRATOR_LEVEL CONFIG_MOGAMI_VIBRATOR_ON_VOLTAGE

#define STRONG_VIBRATION_TIME 50
#define VIBRATOR_LEVEL_STRONG 3100

enum vib_task {
	TASK_KICK_START,
	TASK_START,
	TASK_STOP,
	TASK_FORCE_STOP,
	TASK_NONE,
};

static struct work_struct vibrator_work;
static struct workqueue_struct *vibrator_workqueue;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static enum vib_task vibe_state;
static int long_vibe_time;

static const uint32_t pm_rpc_versions[] = {
	0x30001,
	0x30002,
	0x30003,
	0x30004,
	0x30005,
};

static void set_pmic_vibrator(void)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct pm_vib_mot_set_volt_args {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;
	int rc;
	unsigned long flags;
	int kick_time;

	if (!vib_endpoint) {
		int i;
		for (i = 0; i < ARRAY_SIZE(pm_rpc_versions); i++) {
			vib_endpoint = msm_rpc_connect(PM_LIBPROG,
					pm_rpc_versions[i], 0);
			if (IS_ERR(vib_endpoint))
				printk(KERN_INFO \
				"init vib rpc version %d failed!\n", \
				pm_rpc_versions[i]);
			else
				break;
		}
	}

	if (IS_ERR(vib_endpoint)) {
		printk(KERN_ERR "init vib rpc failed!\n");
		vib_endpoint = 0;
		return;
	}
	pr_debug("%s: ON=%d\n", __func__, vibe_state);

	spin_lock_irqsave(&vibe_lock, flags);
	switch (vibe_state) {
	case TASK_KICK_START:
		if (long_vibe_time > STRONG_VIBRATION_TIME) {
			kick_time = STRONG_VIBRATION_TIME;
			long_vibe_time -= STRONG_VIBRATION_TIME;
			vibe_state = TASK_START;
		} else {
			kick_time = long_vibe_time;
			vibe_state = TASK_STOP;
			long_vibe_time = 0;
		}
		req.data = cpu_to_be32(VIBRATOR_LEVEL_STRONG);
		hrtimer_start(&vibe_timer,
			ktime_set(kick_time / 1000,
			(kick_time % 1000) * 1000000), HRTIMER_MODE_REL);

		pr_debug("%s: KICK START for %d ms\n", __func__, kick_time);
		break;
	case TASK_START:
		req.data = cpu_to_be32(PM_VIBRATOR_LEVEL);
		pr_debug("%s: START for %d ms\n", __func__, long_vibe_time);
		hrtimer_start(&vibe_timer,
			ktime_set((long_vibe_time) / 1000,
			((long_vibe_time) % 1000) * 1000000), HRTIMER_MODE_REL);
		long_vibe_time = 0;
		vibe_state = TASK_STOP;
		break;
	case TASK_STOP:
	case TASK_FORCE_STOP:
	default:
		vibe_state = TASK_NONE;
		req.data = cpu_to_be32(0);
	}
	spin_unlock_irqrestore(&vibe_lock, flags);

	rc = msm_rpc_call(vib_endpoint, PM_VIB_MOT_SET_VOLT_PROC, &req,
		sizeof(req), 5 * HZ);
	if (rc)
		printk(KERN_ERR "vib rpc failed! rc=%d\n", rc);
}

static void update_vibrator(struct work_struct *work)
{
	set_pmic_vibrator();
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long	flags;

	spin_lock_irqsave(&vibe_lock, flags);
	hrtimer_cancel(&vibe_timer);

	if (value == 0) {
		vibe_state = TASK_FORCE_STOP;
	} else {
		long_vibe_time = value;
		vibe_state = TASK_KICK_START;
		pr_debug("%s: Value=%d\n", __func__, value);
	}
	spin_unlock_irqrestore(&vibe_lock, flags);

	queue_work(vibrator_workqueue, &vibrator_work);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&vibe_lock, flags);
	switch (vibe_state) {
	case TASK_KICK_START:
		ret = long_vibe_time;
		break;
	case TASK_START:
	case TASK_STOP:
		ret = long_vibe_time;
		if (hrtimer_active(&vibe_timer)) {
			ktime_t r = hrtimer_get_remaining(&vibe_timer);
			ret += r.tv.sec * 1000 + r.tv.nsec / 1000000;
		}
		break;
	case TASK_FORCE_STOP:
	case TASK_NONE:
	default:
		ret = 0;
		break;
	}
	spin_unlock_irqrestore(&vibe_lock, flags);
	return ret;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	queue_work(vibrator_workqueue, &vibrator_work);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init msm_init_pmic_vibrator(void)
{
	INIT_WORK(&vibrator_work, update_vibrator);

	vibrator_workqueue = create_workqueue("vibrator_workqueue");
	if (vibrator_workqueue == NULL) {
		printk(KERN_ERR "vibrator_workqueue=NULL\n");
		return;
	}

	spin_lock_init(&vibe_lock);
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
	printk(KERN_INFO "Vibrator: enable voltage %d(mV)\n", PM_VIBRATOR_LEVEL);
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");
