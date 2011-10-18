/* include/linux/clearpad.h
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * Author: Courtney Cavin <courtney.cavin@sonyericsson.com>
 *         Yusuke Yoshimura <Yusuke.Yoshimura@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __LINUX_CLEARPAD_H
#define __LINUX_CLEARPAD_H

#define CLEARPAD_NAME "clearpad"
#define CLEARPADI2C_NAME "clearpad-i2c"

enum synaptics_funcarea_kind {
	SYN_FUNCAREA_INSENSIBLE,
	SYN_FUNCAREA_POINTER,
	SYN_FUNCAREA_BOTTOM_EDGE,
	SYN_FUNCAREA_BUTTON,
	SYN_FUNCAREA_BTN_INBOUND,
	SYN_FUNCAREA_END,
};

struct synaptics_funcarea {
	int x1;
	int y1;
	int x2;
	int y2;
	enum synaptics_funcarea_kind func;
	void *data;
};

struct synaptics_button {
	int type;
	int code;
	bool down;
	bool down_report;
};

struct clearpad_platform_data {
	int irq;
	struct synaptics_funcarea *funcarea; /* last one = SYN_FUNCAREA_END */
	int (*gpio_configure)(int);
	void (*vreg_off)(void);
};

struct clearpad_bus_data {
	__u16 bustype;
	int (*read)(struct device *dev, u8 reg, u8 *buf, u8 len);
	int (*write)(struct device *dev, u8 reg, const u8 *buf, u8 len);
};

struct clearpad_data {
	struct clearpad_platform_data *pdata;
	struct clearpad_bus_data *bdata;
};
#endif
