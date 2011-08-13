/* kernel/arch/arm/mach-msm/include/mach/semc_mogami_felica.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Hiroaki Kuriyama <Hiroaki.Kuriyama@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SEMC_MOGAMI_FELICA_H
#define _SEMC_MOGAMI_FELICA_H

extern struct platform_device semc_mogami_felica_device;

/* FeliCa Platform Data structures */
struct msm_uartdm_pfdata {
	char	*paddr_uartdm;
	int	irq_uartdm;
	char	irq_name[16];
	int	chan_uartdm_tx;
	int	crci_uartdm_tx;
	int	chan_uartdm_rx;
	int	crci_uartdm_rx;
	char	clk_str[16];
	struct device	*clk_dev;
	char	workqueue_name[16];
	char	iomem_name[16];
	void	(*callback_rcv_1byte)(void);
	void	(*callback_rx_complete)(void);
	void	(*callback_rx_error)(void);
	void	(*callback_tx_complete)(void);
	void	(*callback_tx_error)(void);
};

struct felica_uart_pfdata {
	struct msm_uartdm_pfdata	uartdm_pfdata;
	int	uartmux_neutral;
	int	uartmux_felica;
};

struct felica_cen_pfdata {
};

struct felica_pon_pfdata {
	int	gpio_pon;
	int	(*tvdd_on)(void);
	void	(*tvdd_off)(void);
};

struct felica_rfs_pfdata {
	int	gpio_rfs;
};

struct felica_int_pfdata {
	int	gpio_int;
	int	irq_int;
};

struct felica_platform_data {
	struct felica_uart_pfdata	uart_pfdata;
	struct felica_cen_pfdata	cen_pfdata;
	struct felica_pon_pfdata	pon_pfdata;
	struct felica_rfs_pfdata	rfs_pfdata;
	struct felica_int_pfdata	int_pfdata;
	int (*gpio_init)(void);
};

#endif
