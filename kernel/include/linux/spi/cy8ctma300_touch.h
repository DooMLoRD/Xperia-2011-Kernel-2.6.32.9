#ifndef __LINUX_SPI_CY8CTMA300_TOUCH_H
#define __LINUX_SPI_CY8CTMA300_TOUCH_H

#include <linux/ioctl.h>

struct cypress_touch_platform_data {
	__u8 gpio_irq_pin;
	__u8 gpio_reset_pin;
	__u16 x_max;
	__u16 y_max;
	__u16 z_max;
	int (*gpio_init)(void);
	__u8 esd_mode;
	int (*spi_cs_set)(bool val);
};

struct cy8ctma300_touch_ioctl_clbr {
	__u8 clbr_num;
	__u8 prescaler;
	__u8 subconversion;
	__u8 shift;
	__u8 noisethreshold;
	__u8 fingerthreshold;
	__u8 ninttimeout;
	__u8 initmovestep;
	__u8 movestep;
};

struct cy8ctma300_touch_reg_read_req {
	__u8 reg;
	__u8 *buf;
	size_t len;
};

struct cy8ctma300_touch_reg_write_req {
	__u8 reg;
	__u8 *buf;
	size_t len;
};

#define IOC_MAGIC 't'

#define IOCTL_VALSET _IOW(IOC_MAGIC, 1, struct cy8ctma300_touch_ioctl_clbr)
#define IOCTL_VALGET _IOR(IOC_MAGIC, 2, struct cy8ctma300_touch_ioctl_clbr)
#define IOCTL_TOUCH_REG_READ \
		_IOWR(IOC_MAGIC, 3, struct cy8ctma300_touch_reg_read_req)
#define IOCTL_TOUCH_REG_WRITE \
		_IOW(IOC_MAGIC, 4, struct cy8ctma300_touch_reg_write_req)

#endif /* __LINUX_SPI_CY8CTMA300_TOUCH_H */
