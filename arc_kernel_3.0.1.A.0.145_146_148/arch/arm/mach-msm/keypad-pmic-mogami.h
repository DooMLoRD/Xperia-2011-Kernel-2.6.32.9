#ifndef __ARCH_ARM_MACH_MSM_KEYPAD_PMIC_MOGAMI_H
#define __ARCH_ARM_MACH_MSM_KEYPAD_PMIC_MOGAMI_H

#include <linux/mfd/pmic8058.h>

#define KP_NAME "keypad-pmic"
#define KP_DEVICE "dev/keypad-pmic"

struct keypad_pmic_mogami_key {
	unsigned int code;
	int irq;
	int gpio;
	int wake;
};

struct keypad_pmic_mogami_platform_data {
	struct keypad_pmic_mogami_key *keymap;
	int keymap_size;
	char *input_name;
	struct pm8058_gpio *pm_gpio_config;
};

extern struct keypad_pmic_mogami_platform_data keypad_pmic_platform_data;
#endif
