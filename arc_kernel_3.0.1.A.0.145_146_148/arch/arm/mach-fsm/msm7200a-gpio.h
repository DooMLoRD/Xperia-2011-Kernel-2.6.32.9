/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __LINUX_MSM7200A_GPIO_H
#define __LINUX_MSM7200A_GPIO_H

/**
 * struct msm7200a_gpio_regs - addresess of iomapped GPIO registers
 * @in:		GPIO_IN_n
 * @out:	GPIO_OUT_n
 * @oe:		GPIO_OE_n
 * @int_status:	GPIO_INT_STATUS_n
 * @int_clear:	GPIO_INT_CLEAR_n
 * @int_en:	GPIO_INT_EN_n
 * @int_edge:	GPIO_INT_EDGE_n
 * @int_pos:	GPIO_INT_POS_n
 *
 * Registers are not guaranteed to be packed in memory, or even
 * located in a predictable pattern.
 */
struct msm7200a_gpio_regs {
	void __iomem *in;
	void __iomem *out;
	void __iomem *oe;
	void __iomem *int_status;
	void __iomem *int_clear;
	void __iomem *int_en;
	void __iomem *int_edge;
	void __iomem *int_pos;
};

/**
 * struct msm7200a_gpio_platform_data - configuration for msm7200a-gpio
 * @gpio_base:	The first gpio to be assigned to the device.  Corresponds
 *		directly to gpio_chip.base.
 * @ngpio:	The number of gpio lines to be managed by the device.
 *		Must be <= 32.  Corresponds directly to gpio_chip.ngpio.
 * @irq_base:	The first irq to be assigned to the device.  The gpio
 *		at 'gpio_base' will be assigned irq 'irq_base',
 *		gpio 'gpio_base + 1' will receive irq 'irq_base + 1',
 *		and so on.
 * @irq_summary:	The summary irq line which will be used by the device
 *			to notify the kernel when an interrupt occurs on
 *			any of its gpio lines.  Most MSM SoCs have more
 *			than one gpio device sharing each of these.
 * @latch_level_irqs:	The MSM gpio hardware latches level interrupts,
 *			which is atypical.  Setting this flag to false
 *			makes the driver compensate for this and produce
 *			the traditional unlatched behavior for level irqs.
 * @regs:	Addresses of the registers which control the gpios
 *		to be managed by the device.
 */
struct msm7200a_gpio_platform_data {
	unsigned gpio_base;
	unsigned ngpio;
	unsigned irq_base;
	unsigned irq_summary;
	bool latch_level_irqs;
	struct msm7200a_gpio_regs regs;
};

#endif
