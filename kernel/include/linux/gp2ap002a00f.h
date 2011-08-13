#ifndef _GP2AP002A00F_H_
#define _GP2AP002A00F_H_

#define GP2A_GPIO_NAME  "gp2a_vo"
#define GP2A_I2C_NAME   "gp2ap002a00f"

struct gp2a_platform_data {
	int gpio;
	int wake;
	int (*gpio_setup)(void);
	int (*gpio_shutdown)(void);
};

#endif
