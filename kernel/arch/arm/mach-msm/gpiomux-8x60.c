/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <mach/msm_iomap.h>
#include "tlmm-msm8660.h"
#include "gpiomux.h"

#define CONSOLE_UART	(GPIOMUX_FUNC_2 | GPIOMUX_DRV_8MA | GPIOMUX_VALID)

#ifdef CONFIG_I2C_QUP
#define GSBI3	(GPIOMUX_FUNC_1 | GPIOMUX_DRV_8MA | GPIOMUX_VALID)
#ifdef CONFIG_MSM_CAMERA
#define GSBI4	(GPIOMUX_FUNC_1 | GPIOMUX_DRV_8MA | GPIOMUX_VALID)
#else
#define GSBI4	0
#endif
#define GSBI7	(GPIOMUX_FUNC_1 | GPIOMUX_DRV_8MA | GPIOMUX_VALID)
#define GSBI8	(GPIOMUX_FUNC_1 | GPIOMUX_VALID)
#else
#define GSBI3	0
#define GSBI4	0
#define GSBI7	0
#define GSBI8	0
#endif

#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
#define GSBI1	(GPIOMUX_FUNC_1 | GPIOMUX_DRV_8MA | GPIOMUX_VALID)
#else
#define GSBI1	0
#endif

#define PS_HOLD	(GPIOMUX_FUNC_1 | GPIOMUX_DRV_12MA | GPIOMUX_VALID)

#define EBI2_A_D	(GPIOMUX_FUNC_1 | GPIOMUX_PULL_UP | GPIOMUX_DRV_8MA |\
			 GPIOMUX_VALID)
#define EBI2_OE		(GPIOMUX_FUNC_1 | GPIOMUX_PULL_UP | GPIOMUX_DRV_8MA |\
			 GPIOMUX_VALID)
#define EBI2_WE		(GPIOMUX_FUNC_1 | GPIOMUX_PULL_UP | GPIOMUX_DRV_8MA |\
			 GPIOMUX_VALID)
#define EBI2_CS2	(GPIOMUX_FUNC_2 | GPIOMUX_PULL_UP | GPIOMUX_DRV_8MA |\
			 GPIOMUX_VALID)
#define EBI2_CS3	(GPIOMUX_FUNC_1 | GPIOMUX_PULL_UP | GPIOMUX_DRV_8MA |\
			 GPIOMUX_VALID)
#define EBI2_ADV	(GPIOMUX_FUNC_1 | GPIOMUX_PULL_UP | GPIOMUX_DRV_8MA |\
			 GPIOMUX_VALID)

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	#define SDCC1_DAT_0_3_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_1 | GPIOMUX_DRV_10MA)
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	#define SDCC1_DAT_4_7_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_1 | GPIOMUX_DRV_10MA)
#else
	#define SDCC1_DAT_4_7_CMD_ACTV_CFG 0
#endif
	#define SDCC1_CLK_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_NONE\
					| GPIOMUX_FUNC_1 | GPIOMUX_DRV_16MA)
#else
	#define SDCC1_DAT_0_3_CMD_ACTV_CFG 0
	#define SDCC1_DAT_4_7_CMD_ACTV_CFG 0
	#define SDCC1_CLK_ACTV_CFG 0
#endif

#define SDCC1_SUSPEND_CONFIG (GPIOMUX_VALID | GPIOMUX_PULL_UP)

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	#define SDCC2_DAT_0_3_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	#define SDCC2_DAT_4_7_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#else
	#define SDCC2_DAT_4_7_CMD_ACTV_CFG 0
#endif
	#define SDCC2_CLK_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_NONE\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_16MA)
#else
	#define SDCC2_DAT_0_3_CMD_ACTV_CFG 0
	#define SDCC2_DAT_4_7_CMD_ACTV_CFG 0
	#define SDCC2_CLK_ACTV_CFG 0
#endif

#define SDCC2_SUSPEND_CONFIG (GPIOMUX_VALID | GPIOMUX_PULL_DOWN)

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	#define SDCC5_DAT_0_3_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#ifdef CONFIG_MMC_MSM_SDC5_8_BIT_SUPPORT
	#define SDCC5_DAT_4_7_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#else
	#define SDCC5_DAT_4_7_CMD_ACTV_CFG 0
#endif
	#define SDCC5_CLK_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_NONE\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_16MA)
#else
	#define SDCC5_DAT_0_3_CMD_ACTV_CFG 0
	#define SDCC5_DAT_4_7_CMD_ACTV_CFG 0
	#define SDCC5_CLK_ACTV_CFG 0
#endif

#define SDCC5_SUSPEND_CONFIG (GPIOMUX_VALID | GPIOMUX_PULL_DOWN)

struct msm_gpiomux_config msm_gpiomux_configs[GPIOMUX_NGPIOS] = {
	[33] = {
		.suspended = GSBI1,
	},
	[34] = {
		.suspended = GSBI1,
	},
	[35] = {
		.suspended = GSBI1,
	},
	[36] = {
		.suspended = GSBI1,
	},
	[40] = {
		.suspended = EBI2_CS2,
	},
	[43] = {
		.suspended = GSBI3,
	},
	[44] = {
		.suspended = GSBI3,
	},
	[47] = {
		.suspended = GSBI4,
	},
	[48] = {
		.suspended = GSBI4,
	},
	[59] = {
		.suspended = GSBI7,
	},
	[60] = {
		.suspended = GSBI7,
	},
	[64] = {
		.suspended = GSBI8,
	},
	[65] = {
		.suspended = GSBI8,
	},
	[92] = {
		.suspended = PS_HOLD,
	},
	[115] = {
		.suspended = CONSOLE_UART,
	},
	[116] = {
		.suspended = CONSOLE_UART,
	},
	[117] = {
		.suspended = CONSOLE_UART,
	},
	[118] = {
		.suspended = CONSOLE_UART,
	},
	[123] = {
		.suspended = EBI2_A_D,
	},
	[124] = {
		.suspended = EBI2_A_D,
	},
	[125] = {
		.suspended = EBI2_A_D,
	},
	[126] = {
		.suspended = EBI2_A_D,
	},
	[127] = {
		.suspended = EBI2_A_D,
	},
	[128] = {
		.suspended = EBI2_A_D,
	},
	[129] = {
		.suspended = EBI2_A_D,
	},
	[130] = {
		.suspended = EBI2_A_D,
	},
	[133] = {
		.suspended = EBI2_CS3,
	},
	[135] = {
		.suspended = EBI2_A_D,
	},
	[136] = {
		.suspended = EBI2_A_D,
	},
	[137] = {
		.suspended = EBI2_A_D,
	},
	[138] = {
		.suspended = EBI2_A_D,
	},
	[139] = {
		.suspended = EBI2_A_D,
	},
	[140] = {
		.suspended = EBI2_A_D,
	},
	[141] = {
		.suspended = EBI2_A_D,
	},
	[142] = {
		.suspended = EBI2_A_D,
	},
	[143] = {
		.suspended = EBI2_A_D,
	},
	[144] = {
		.suspended = EBI2_A_D,
	},
	[145] = {
		.suspended = EBI2_A_D,
	},
	[146] = {
		.suspended = EBI2_A_D,
	},
	[147] = {
		.suspended = EBI2_A_D,
	},
	[148] = {
		.suspended = EBI2_A_D,
	},
	[149] = {
		.suspended = EBI2_A_D,
	},
	[150] = {
		.suspended = EBI2_A_D,
	},
	[151] = {
		.suspended = EBI2_OE,
	},
	[153] = {
		.suspended = EBI2_ADV,
	},
	[157] = {
		.suspended = EBI2_WE,
	},
	/* SDCC1 data[0] */
	[159] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[1] */
	[160] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[2] */
	[161] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[3] */
	[162] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[4] */
	[163] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[5] */
	[164] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[6] */
	[165] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[7] */
	[166] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 CLK */
	[167] = {
		.active = SDCC1_CLK_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 CMD */
	[168] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
};

void __msm_gpiomux_write(unsigned gpio, gpiomux_config_t val)
{
	writel(val & ~GPIOMUX_CTL_MASK, GPIO_CONFIG(gpio));
}
