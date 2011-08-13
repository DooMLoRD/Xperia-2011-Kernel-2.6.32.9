/* kernel/arch/arm/mach-msm/semc_mogami_felica.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Hiroaki.Kuriyama <Hiroaki.Kuriyama@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <mach/semc_mogami_felica.h>
#include <mach/vreg.h>
#include <mach/dma.h>
#include <mach/irqs-7x30.h>

/* Macros assume PMIC GPIOs start at 0 */
#define PRT_NAME "Mogami FeliCa support"
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM_GPIO_FELICA_M_RX   36  /* PM8058 GPIO #37 */
#define PM_GPIO_FELICA_M_TX   35
#define PM_GPIO_FELICA_RX1    32
#define PM_GPIO_FELICA_RX2    33
#define PM_GPIO_FELICA_RX3    34
#define PM_GPIO_FELICA_TX1    20
#define PM_GPIO_FELICA_TX2    21
#define PM_GPIO_FELICA_TX3    22
#define PM_GPIO_FELICA_PON    16
#define PM_GPIO_FELICA_RFS    17
#define PM_GPIO_FELICA_INT    18
#define FELICA_MSM_UART2DM_PHYS  (0xA3200000) /* MSM7x30 UART2DM for FeliCa */
#define VREG_TVDD_NAME  "wlan2"
#define VREG_TVDD_LEVEL 3050

static struct msm_gpio felica_uart[] = {
	{ GPIO_CFG(85, 3, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
							"UART2DM_Rx" },
	{ GPIO_CFG(87, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
							"UART2DM_Tx" },
};

struct pm8058_gpio pm_felica_m_tx = {
	.direction      = PM_GPIO_DIR_IN,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,
	.out_strength   = PM_GPIO_STRENGTH_NO,
	.function       = PM_GPIO_FUNC_NORMAL,
};

struct pm8058_gpio pm_felica_tx = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,
	.out_strength   = PM_GPIO_STRENGTH_MED,
	.function       = PM_GPIO_FUNC_2,
};

struct pm8058_gpio pm_felica_m_rx = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,
	.out_strength   = PM_GPIO_STRENGTH_MED,
	.function       = PM_GPIO_FUNC_2,
};

struct pm8058_gpio pm_felica_rx = {
	.direction      = PM_GPIO_DIR_IN,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,
	.out_strength   = PM_GPIO_STRENGTH_NO,
	.function       = PM_GPIO_FUNC_NORMAL,
};

struct pm8058_gpio pm_felica_pon = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,
	.out_strength   = PM_GPIO_STRENGTH_MED,
	.function       = PM_GPIO_FUNC_NORMAL,
};

struct pm8058_gpio pm_felica_rfs = {
	.direction      = PM_GPIO_DIR_IN,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,
	.out_strength   = PM_GPIO_STRENGTH_NO,
	.function       = PM_GPIO_FUNC_NORMAL,
};

struct pm8058_gpio pm_felica_int = {
	.direction      = PM_GPIO_DIR_IN,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,
	.out_strength   = PM_GPIO_STRENGTH_NO,
	.function       = PM_GPIO_FUNC_NORMAL,
};

struct felica_pm_gpio_config {
	int                gpio;
	struct pm8058_gpio *param;
};

static struct felica_pm_gpio_config felica_pm_gpio[] = {
	{PM_GPIO_FELICA_TX1, &pm_felica_tx},
	{PM_GPIO_FELICA_TX3, &pm_felica_tx},
	{PM_GPIO_FELICA_RX1, &pm_felica_rx},
	{PM_GPIO_FELICA_RX3, &pm_felica_rx},
	{PM_GPIO_FELICA_M_TX, &pm_felica_m_tx},
	{PM_GPIO_FELICA_M_RX, &pm_felica_m_rx},
	{PM_GPIO_FELICA_PON, &pm_felica_pon},
	{PM_GPIO_FELICA_RFS, &pm_felica_rfs},
	{PM_GPIO_FELICA_INT, &pm_felica_int},

};

static int semc_mogami_felica_tvdd_on(void)
{
	struct vreg *reg = NULL;
	int rc = 0;

	reg = vreg_get(NULL, VREG_TVDD_NAME);
	if (reg == NULL) {
		pr_err(PRT_NAME "Unable to resolve TVDD name\n");
		return rc;
	}

	rc = vreg_set_level(reg, VREG_TVDD_LEVEL);
	if (rc) {
		pr_err(PRT_NAME "Unable to set TVDD level\n");
		return rc;
	}

	rc = vreg_enable(reg);
	if (rc) {
		pr_err(PRT_NAME "Unable to enable TVDD\n");
		return rc;
	}

	return rc;
}

static void semc_mogami_felica_tvdd_off(void)
{
	struct vreg *reg = NULL;
	int rc;

	reg = vreg_get(NULL, VREG_TVDD_NAME);
	if (reg == NULL) {
		pr_err(PRT_NAME "Unable to resolve TVDD\n");
		return;
	}

	rc = vreg_disable(reg);
	if (rc) {
		pr_err(PRT_NAME "Unable to disable TVDD\n");
		return;
	}
}

static int semc_mogami_felica_gpio_init(void)
{
	int ret;
	int i, len;

	ret = 0;

	/* Configure PM8058 GPIO*/
	len = ARRAY_SIZE(felica_pm_gpio);
	for (i = 0; i < len; i++) {
		ret = pm8058_gpio_config(felica_pm_gpio[i].gpio,
						felica_pm_gpio[i].param);
		if (ret) {
			pr_err("%s PM_GPIO_FELICA[%d] config failed\n",
				 __func__, i);
			return ret;
		}

	}

	/* Configure MSM UART2DM GPIO*/
	ret = msm_gpios_request_enable(felica_uart, ARRAY_SIZE(felica_uart));
	if (ret) {
		pr_err("%s enable uart2dm gpios failed\n", __func__);
		return ret;
	}

	return 0;
}

extern struct platform_device msm_device_uart_dm2;

struct felica_platform_data semc_mogami_felica_pfdata = {
	.uart_pfdata = {
		.uartdm_pfdata = {
			.paddr_uartdm = (char *) FELICA_MSM_UART2DM_PHYS,
			.irq_uartdm = INT_UART2DM_IRQ,
			.chan_uartdm_tx = DMOV_HSUART2_TX_CHAN,
			.crci_uartdm_tx = DMOV_HSUART2_TX_CRCI,
			.chan_uartdm_rx = DMOV_HSUART2_RX_CHAN,
			.crci_uartdm_rx = DMOV_HSUART2_RX_CRCI,
			.clk_str = "uartdm_clk",
			.clk_dev = &msm_device_uart_dm2.dev,
			.workqueue_name = "felica_uartdm",
			.iomem_name = "felica_uartdm",
			.callback_rcv_1byte = NULL,
			.callback_rx_complete = NULL,
			.callback_rx_error = NULL,
			.callback_tx_complete = NULL,
			.callback_tx_error = NULL,
		},
		.uartmux_neutral = PM8058_UART_MUX_NO,
		.uartmux_felica = PM8058_UART_MUX_1,
	},
	.pon_pfdata = {
		.gpio_pon = PM8058_GPIO_PM_TO_SYS(PM_GPIO_FELICA_PON),
		.tvdd_on = semc_mogami_felica_tvdd_on,
		.tvdd_off = semc_mogami_felica_tvdd_off,
	},
	.rfs_pfdata = {
		.gpio_rfs = PM8058_GPIO_PM_TO_SYS(PM_GPIO_FELICA_RFS),
	},
	.int_pfdata = {
		.gpio_int = PM8058_GPIO_PM_TO_SYS(PM_GPIO_FELICA_INT),
		.irq_int = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE,
						PM_GPIO_FELICA_INT),
	},
	.gpio_init = semc_mogami_felica_gpio_init,
};

struct platform_device semc_mogami_felica_device = {
	.name = "semc_felica",
	.id = 0,
	.dev  = {
		.platform_data = &semc_mogami_felica_pfdata,
	},
};
