#ifndef _AK8975_H_
#define _AK8975_H_

struct ak8975_platform_data {
	int gpio;
	int (* gpio_setup)(void);
	int (* gpio_shutdown)(void);
};

#endif
