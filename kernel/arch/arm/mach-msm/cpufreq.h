#ifndef __ARCH_ARM_MACH_MSM_CPUFREQ_H
#define __ARCH_ARM_MACH_MSM_CPUFREQ_H

#include <linux/list.h>

enum {
	MSM_CPUFREQ_IDLE,
	MSM_CPUFREQ_ACTIVE,
};

struct msm_cpufreq_voter {
	int (*vote)(struct msm_cpufreq_voter *);
	struct list_head item;
};

void msm_cpufreq_voter_update(struct msm_cpufreq_voter *);

int msm_cpufreq_register_voter(struct msm_cpufreq_voter *);
void msm_cpufreq_unregister_voter(struct msm_cpufreq_voter *);

#endif
