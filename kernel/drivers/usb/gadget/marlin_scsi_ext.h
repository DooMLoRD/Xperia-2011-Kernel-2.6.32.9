/* drivers/usb/function/marlin.h
 *
 * Marlin USB SCSI Command Extension
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _DRIVERS_USB_FUNCTION_MARLIN_H_
#define _DRIVERS_USB_FUNCTION_MARLIN_H_

#define USB_MARLIN_IOC_MAGIC    0xFE
#define USB_MARLIN_IOC_INIT     _IOR(USB_MARLIN_IOC_MAGIC, 0x20, int)
#define USB_MARLIN_IOC_FIN      _IOR(USB_MARLIN_IOC_MAGIC, 0x21, int)

int mldd_init(void);
int mldd_fin(void);
void mldd_enable(void);
void mldd_disable(void);
void mldd_mount(void);
void mldd_unmount(void);
int mldd_do_send_key(int cmd_size,
			u8 *cmd_p,
			u16 buf_size,
			u8 *bh_buf_p,
			u32 *sense_code_p);
int mldd_do_report_key(int cmd_size,
			u8 *cmd_p,
			u16 buf_size,
			u8 *bh_buf_p,
			u32 *sense_code_p);
int mldd_do_mode_sense(int all_pages,
			int page_code,
			int subpage_code,
			u8 *bh_buf_p);
#endif
