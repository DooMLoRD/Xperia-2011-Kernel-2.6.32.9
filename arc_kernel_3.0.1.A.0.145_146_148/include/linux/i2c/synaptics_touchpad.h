#ifndef __LINUX_I2C_SYNAPTICS_TOUCHPAD_H
#define __LINUX_I2C_SYNAPTICS_TOUCHPAD_H

#include <linux/ioctl.h>

#define SYNIO                   0xA5

#define SYN_IOCTL_SET_ACTIVE	_IOW(SYNIO, 0x0E, int)


struct synaptics_touchpad_platform_data {
	int (* gpio_setup)(void);
	void (* gpio_teardown)(void);
};

#endif /* __LINUX_I2C_SYNAPTICS_TOUCHPAD_H */
