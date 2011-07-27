/* drivers/misc/lm356x-status.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Shougo Watanabe <Shougo.X.Watanebe@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
*/

#ifndef __LM356X_STATUS_H__
#define __LM356X_STATUS_H__

/* Camera-light status define */
#define	LM356X_FLASH_TIMEOUT	0x01	/* FLASH Time out */
#define	LM356X_THERMAL_SHUTDOWN	0x02	/* Temperature over */
#define	LM356X_LED_FAULT	0x04	/* LED is Open or Short */
#define	LM356X_TX1_INT		0x08	/* TX1 has changed state */
#define	LM356X_TX2_INT		0x10	/* TX2 has changed state */
#define	LM356X_NTC_FAULT	0x20	/* Temperature warning */
#define	LM356X_VIN_FLASH	0x40
#define	LM356X_VIN_MON		0x80

#endif /* __LM356X_STATUS_H__ */
