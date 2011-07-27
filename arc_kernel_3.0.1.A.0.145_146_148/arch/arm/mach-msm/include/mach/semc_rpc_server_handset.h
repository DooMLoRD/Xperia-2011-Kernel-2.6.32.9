/* arch/arm/mach-msm/include/mach/semc_rpc_server_handset.h
 *
 * Copyright (C) 2010 SonyEricsson Mobile Communications AB
 *
 * Based on rpc_server_handset.h, Copyright (c) 2008-2009,
 * Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 *
 * Author: Joachim Holst (joachim.holst@sonyericsson.com)
 *
 */

#ifndef __ASM_ARCH_MSM_SEMC_RPC_SERVER_HANDSET_H
#define __ASM_ARCH_MSM_SEMC_RPC_SERVER_HANDSET_H

typedef void (* const *handset_callback_t) (uint32_t, uint32_t);
typedef void (* const handset_cb_array_t[])(uint32_t, uint32_t);

#define SEMC_HANDSET_DRIVER_NAME "semc-handset"

/*
 * Names and value definitions of different keys/events.
 * Decided to keep them for easy handling by clients.
 * These were previously defined in rpc_server_handset.c
 */
#define HS_PWR_K		0x6F	/* Power key */
#define HS_END_K		0x51	/* End key or Power key */
#define HS_STEREO_HEADSET_K	0x82    /* Headset inserted */
#define HS_HEADSET_SWITCH_K	0x84    /* Headset button */
#define HS_HEADSET_SWITCH_OFF_K 0x85    /* Headset button */
#define HS_REL_K		0xFF	/* Key release */

struct semc_handset_data {
	u32 num_callbacks;
	handset_callback_t callbacks;
};

/* Used by SEport to tell AMSS side hs driver to enable hssd detection */
void report_headset_status(bool connected);

#endif /* __ASM_ARCH_MSM_SEMC_RPC_SERVER_HANDSET_H */
