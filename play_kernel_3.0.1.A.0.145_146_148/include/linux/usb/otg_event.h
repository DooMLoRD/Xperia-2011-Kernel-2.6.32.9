 /* include/linux/usb/otg_event.h
 *
 * OTG event handling function
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#ifndef __OTG_EVENT_H__
#define __OTG_EVENT_H__

#include <linux/usb/otg.h>

extern int otg_event_driver_register(struct otg_transceiver *xceiv);
extern void otg_event_driver_unregister(void);

#endif
