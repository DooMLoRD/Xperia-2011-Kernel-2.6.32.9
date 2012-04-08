/* arch/arm/mach-msm/cpufreq.c
 *
 * MSM architecture cpufreq driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2010, Code Aurora Forum. All rights reserved.
 * Author: Mike A. Chan <mikechan@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include "acpuclock.h"
#include "cpufreq.h"

#ifdef CONFIG_SMP
struct cpufreq_work_struct {
	struct work_struct work;
	struct cpufreq_policy *policy;
	struct completion complete;
	int frequency;
	int status;
};

static DEFINE_PER_CPU(struct cpufreq_work_struct, cpufreq_work);
#endif

static DEFINE_MUTEX(msm_cpufreq_voter_lock);
static int msm_cpufreq_vote = MSM_CPUFREQ_IDLE;
static LIST_HEAD(msm_cpufreq_voters);

#define dprintk(msg...) \
		cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, "cpufreq-msm", msg)

static int msm_cpufreq_check_votes(void)
{
	struct msm_cpufreq_voter *voter;
	int vote = MSM_CPUFREQ_IDLE;

	list_for_each_entry(voter, &msm_cpufreq_voters, item)
		vote |= voter->vote(voter);

	return vote;
}

void msm_cpufreq_voter_update(struct msm_cpufreq_voter *v)
{
	mutex_lock(&msm_cpufreq_voter_lock);
	msm_cpufreq_vote = msm_cpufreq_check_votes();
	mutex_unlock(&msm_cpufreq_voter_lock);
}
EXPORT_SYMBOL_GPL(msm_cpufreq_voter_update);

int msm_cpufreq_register_voter(struct msm_cpufreq_voter *v)
{
	if (v == NULL || v->vote == NULL)
		return -EINVAL;

	mutex_lock(&msm_cpufreq_voter_lock);
	list_add(&v->item, &msm_cpufreq_voters);
	msm_cpufreq_vote = msm_cpufreq_check_votes();
	mutex_unlock(&msm_cpufreq_voter_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(msm_cpufreq_register_voter);

void msm_cpufreq_unregister_voter(struct msm_cpufreq_voter *v)
{
	if (v == NULL)
		return;

	mutex_lock(&msm_cpufreq_voter_lock);
	list_del(&v->item);
	msm_cpufreq_vote = msm_cpufreq_check_votes();
	mutex_unlock(&msm_cpufreq_voter_lock);
}
EXPORT_SYMBOL_GPL(msm_cpufreq_unregister_voter);

static int set_cpu_freq(struct cpufreq_policy *policy, unsigned int new_freq)
{
	int ret = 0;
	struct cpufreq_freqs freqs;

	/* race condition ok */
	if (msm_cpufreq_vote == MSM_CPUFREQ_ACTIVE)
		new_freq = policy->max;

	freqs.old = policy->cur;
	freqs.new = new_freq;
	freqs.cpu = policy->cpu;
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	ret = acpuclk_set_rate(policy->cpu, new_freq, SETRATE_CPUFREQ);
	if (!ret)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

#ifdef CONFIG_SMP
static void set_cpu_work(struct work_struct *work)
{
	struct cpufreq_work_struct *cpu_work =
		container_of(work, struct cpufreq_work_struct, work);

	cpu_work->status = set_cpu_freq(cpu_work->policy, cpu_work->frequency);
	complete(&cpu_work->complete);
}
#endif

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int ret = -EFAULT;
	int index;
	struct cpufreq_frequency_table *table;

	table = cpufreq_frequency_get_table(policy->cpu);
	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		return -EINVAL;
	}

#ifdef CONFIG_CPU_FREQ_DEBUG
	dprintk("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
		policy->cpu, target_freq, relation,
		policy->min, policy->max, table[index].frequency);
#endif

#ifdef CONFIG_SMP
	if (get_cpu() == policy->cpu) {
		/* Issue a direct call, since we are on the same cpu */
		ret = set_cpu_freq(policy, table[index].frequency);
		put_cpu();
	} else {
		struct cpufreq_work_struct *cpu_work = NULL;

		put_cpu();
		cpu_work = &per_cpu(cpufreq_work, policy->cpu);
		cpu_work->policy = policy;
		cpu_work->frequency = table[index].frequency;

		init_completion(&cpu_work->complete);
		cancel_work_sync(&cpu_work->work);
		schedule_work_on(policy->cpu, &cpu_work->work);
		wait_for_completion(&cpu_work->complete);
		ret = cpu_work->status;
	}
#else
	ret = set_cpu_freq(policy, table[index].frequency);
#endif

	return ret;
}

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
	return 0;
}

static int __cpuinit msm_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table;
#ifdef CONFIG_SMP
	struct cpufreq_work_struct *cpu_work = NULL;
#endif

	table = cpufreq_frequency_get_table(policy->cpu);
	policy->cur = acpuclk_get_rate(policy->cpu);
	if (cpufreq_frequency_table_cpuinfo(policy, table)) {
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
		policy->cpuinfo.min_freq = CONFIG_MSM_CPU_FREQ_MIN;
		policy->cpuinfo.max_freq = CONFIG_MSM_CPU_FREQ_MAX;
#endif
	}
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	policy->min = CONFIG_MSM_CPU_FREQ_MIN;
	policy->max = CONFIG_MSM_CPU_FREQ_MAX;
#endif

	policy->cpuinfo.transition_latency =
		acpuclk_get_switch_time() * NSEC_PER_USEC;
#ifdef CONFIG_SMP
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	INIT_WORK(&cpu_work->work, set_cpu_work);
#endif
	policy->min = 249600;
	policy->max = 1017600;

	return 0;
}

static struct freq_attr *msm_cpufreq_attr[] = {  
        &cpufreq_freq_attr_scaling_available_freqs,  
        NULL,
};

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculations are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.name		= "msm",
        .attr		= msm_cpufreq_attr,
};

static int __init msm_cpufreq_register(void)
{
	return cpufreq_register_driver(&msm_cpufreq_driver);
}

late_initcall(msm_cpufreq_register);

