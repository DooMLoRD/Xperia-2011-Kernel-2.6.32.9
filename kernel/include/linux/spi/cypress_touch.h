#ifndef __LINUX_SPI_CYPRESS_TOUCH_H
#define __LINUX_SPI_CYPRESS_TOUCH_H

#include <linux/ioctl.h>

/* most of this stuff is shared between the cy8c204 and cy8ctma300 devices */

struct cypress_callback {
	void (*cb)(void *, int);
};

struct cypress_touch_platform_data {
	u8	gpio_irq_pin;
	u8	gpio_reset_pin;

	u16	x_min, x_max;
	u16	y_min, y_max;

	int	irq;
	int	irq_polarity;
	int	reset_polarity;
	int	no_fw_update;
	void	(*register_cb)(struct cypress_callback *);
};

struct cy8c204_touch_ioctl_clbr {
	u8 clbr_num;        // TODO: To check&fix. The this menber is temporary correspondence.
	u8 resolution;
	u8 scanspeed;
	u8 prescaler;
	u8 debounce;
	u8 hysteresis;
	u8 noisethreshold;
	u8 fingerthreshold;
	u8 negnoisethreshold;
	u8 lowbaselinereset;
	u8 blupdatethreshold;
};

#define IOC_MAGIC 't'

#define IOCTL_VALSET 	_IOW(IOC_MAGIC, 1, struct cy8c204_touch_ioctl_clbr)
#define IOCTL_VALGET 	_IOR(IOC_MAGIC, 2, struct cy8c204_touch_ioctl_clbr)
#define IOCTL_REFLASH	_IOW(IOC_MAGIC, 3, int)

#endif /* __LINUX_SPI_CYPRESS_TOUCH_H */
