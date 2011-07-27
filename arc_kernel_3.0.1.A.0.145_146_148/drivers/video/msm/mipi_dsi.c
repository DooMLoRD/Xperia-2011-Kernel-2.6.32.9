/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clk.h>

#include "msm_fb.h"
#include "mipi_dsi.h"

static int mipi_dsi_probe(struct platform_device *pdev);
static int mipi_dsi_remove(struct platform_device *pdev);

static int mipi_dsi_off(struct platform_device *pdev);
static int mipi_dsi_on(struct platform_device *pdev);

#ifdef CONFIG_PM
static int mipi_dsi_suspend(struct platform_device *pdev, pm_message_t state);
static int mipi_dsi_resume(struct platform_device *pdev);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mipi_dsi_early_suspend(struct early_suspend *h);
static void mipi_dsi_early_resume(struct early_suspend *h);
#endif

static struct clk *dsi_byte_div_clk;
static struct clk *dsi_esc_clk;
static struct clk *dsi_m_pclk;
static struct clk *amp_pclk;

static char *mmss_cc_base;	/* mutimedia sub system clock control */
static char *mmss_sfpb_base;	/* mutimedia sub system sfpb */

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;
static struct mipi_dsi_platform_data *mipi_dsi_pdata;

static struct platform_driver mipi_dsi_driver = {
	.probe = mipi_dsi_probe,
	.remove = mipi_dsi_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM
	.suspend = mipi_dsi_suspend,
	.resume = mipi_dsi_resume,
#endif
#endif
	.shutdown = NULL,
	.driver = {
		   .name = "mipi_dsi",
		   },
};

struct device dsi_dev;


/*
 * mipi_dsi_mxo_selected() -
 *
 *	determine if PXO or MXO should be used as clk_ref for the
 *	DSI PLL.
 *
 *	Note - this function can only be called after clk_enable()
 *	is called for the dsi_esc_clk, as that is what ensures that
 *	the MXO/PXO slection has been made for the dsi_esc_clk.  This
 *	function simply reads what the kernel's clock manager has
 *	configured for dsi_esc_clk.
 *
 *	Returns 1 if MXO should be used, 0 if PXO should be used
 *
 */
static int mipi_dsi_mxo_selected(void)
{
	uint32_t data = MIPI_INP(mmss_cc_base + 0x005c);

	return (data & BIT(14)) != 0;
}


#ifdef DSI_CLK_CALCULATE
static void mipi_dsi_clk(int on, struct dsi_clk_desc *clk)
{
	uint32 *cc, *ns, *md;
	uint32 data, val;

	cc = (uint32 *)(mmss_cc_base + 0x004c);
	md = (uint32 *)(mmss_cc_base + 0x0050);
	ns = (uint32 *)(mmss_cc_base + 0x0054);

	val = clk->d * 2;
	data = (~val) & 0x0ff;
	data |= clk->m << 8;
	MIPI_OUTP(md, data);

	val = clk->n - clk->m;
	data = (~val) & 0x0ff;
	data <<= 24;
	data |= clk->src;
	MIPI_OUTP(ns, data);

	/*
	 * mxo, bypass, mnd_en, root_en, clk_en
	 * */
	MIPI_OUTP(cc, 0x0145);
}
#else

static uint32_t dsi_cc_data = 0x25;

static void mipi_dsi_clk(int on)
{
	char	*cc, *ns, *md;

	cc = mmss_cc_base + 0x004c;
	md = mmss_cc_base + 0x0050;
	ns = mmss_cc_base + 0x0054;

	MIPI_OUTP(cc, dsi_cc_data);
	wmb();
	MIPI_OUTP(md, 0x1fd);
	wmb();
	MIPI_OUTP(ns, 0xff000003);
	wmb();
}

#endif

static void mipi_dsi_sfpb_cfg(void)
{
	char *sfpb;
	int data;

	sfpb = mmss_sfpb_base + 0x058;

	data = MIPI_INP(sfpb);
	data |= 0x01800;
	MIPI_OUTP(sfpb, data);
	wmb();
}


static void mipi_dsi_pclk(int on)
{
	char	*cc, *ns, *md;

	cc = mmss_cc_base + 0x0130;
	md = mmss_cc_base + 0x0134;
	ns = mmss_cc_base + 0x0138;

	MIPI_OUTP(cc, 0x2a5);
	wmb();
	MIPI_OUTP(md, 0x1fb);
	wmb();
	MIPI_OUTP(ns, 0xfd0003);
	wmb();
}

static void mipi_dsi_ahb_en(void)
{
	char	*ahb;

	ahb = mmss_cc_base + 0x08;

	printk(KERN_INFO "%s: ahb=%x %x\n", __func__, (int) ahb, MIPI_INP(ahb));
}

static void mipi_dsi_calibration(void)
{
	uint32 data;

	MIPI_OUTP(MIPI_DSI_BASE + 0xf4, 0x0000ff11); /* cal_ctrl */
	MIPI_OUTP(MIPI_DSI_BASE + 0xf8, 0x00a105a1); /* cal_hw_ctrl */
	MIPI_OUTP(MIPI_DSI_BASE + 0xf0, 0x01); /* cal_hw_trigger */

	while (1) {
		data = MIPI_INP(MIPI_DSI_BASE + 0xfc); /* cal_status */
		if ((data & 0x10000000) == 0)
			break;

		udelay(10);
	}
}

static struct dsi_phy_ctrl dsi_phy_db[DSI_PANEL_MAX] = {
	/* 480*854, RGB888, 2 Lane 60 fps video mode */
	{
		{0x03, 0x01, 0x01, 0x00},	/* regulator */
		/* timing   */
		{0x64, 0x1e, 0x14, 0x00, 0x2d, 0x23, 0x1e, 0x1c,
		0x0b, 0x13, 0x04},
		{0x7f, 0x00, 0x00, 0x00},	/* phy ctrl */
		{0xee, 0x03, 0x86, 0x03},	/* strength */
		/* pll control */

#define DSI_BIT_CLK_380MHZ

#if defined(DSI_BIT_CLK_366MHZ)
		{0x41, 0xdb, 0xb2, 0xf5, 0x00, 0x50, 0x48, 0x63,
		0x31, 0x0f, 0x07,
		0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
#elif defined(DSI_BIT_CLK_380MHZ)
		{0x41, 0xf7, 0xb2, 0xf5, 0x00, 0x50, 0x48, 0x63,
		0x31, 0x0f, 0x07,
		0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
#elif defined(DSI_BIT_CLK_400MHZ)
		{0x41, 0x8f, 0xb1, 0xda, 0x00, 0x50, 0x48, 0x63,
		0x31, 0x0f, 0x07,
		0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
#else		/* 200 mhz */
		{0x41, 0x8f, 0xb1, 0xda, 0x00, 0x50, 0x48, 0x63,
		0x33, 0x1f, 0x0f,
		0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
#endif
		0x0e0f		/* clock out timing */
	},
	{ /* 480*854, RGB888, 2 Lane 60 fps cmd mode */
		{0x03, 0x01, 0x01, 0x00},          /* regulator */
		/* timing */
		{0x52, 0x16, 0x10, 0x00, 0x22, 0x16, 0x22, 0x1c,
		0x1c, 0x1b, 0x1c },
		{0x7f, 0x00, 0x00, 0x00},           /* phy ctrl */
		{0x88, 0x03, 0x86, 0x03},           /* strength */
		/* pll control */
		{0x41, 0x57, 0xba, 0xda, 0x00, 0x50, 0x48, 0x63,
		0x31, 0x1f, 0x1f,
		0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
		0x0f0e		/* clock out timing   */
	}
};

void mipi_dsi_phy_init(int panel_ndx)
{
	struct dsi_phy_ctrl *pd;
	int i, off;

	MIPI_OUTP(MIPI_DSI_BASE + 0x128, 0x0001);/* start phy sw reset */
	msleep(100);
	MIPI_OUTP(MIPI_DSI_BASE + 0x128, 0x0000);/* end phy w reset */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2cc, 0x0003);/* regulator_ctrl_0 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2d0, 0x0001);/* regulator_ctrl_1 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2d4, 0x0001);/* regulator_ctrl_2 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2d8, 0x0000);/* regulator_ctrl_3 */
#ifdef DSI_POWER
	MIPI_OUTP(MIPI_DSI_BASE + 0x2dc, 0x0100);/* regulator_ctrl_4 */
#endif

	pd = &dsi_phy_db[panel_ndx];

	off = 0x02cc;	/* regulator ctrl 0 */
	for (i = 0; i < 4; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->regulator[i]);
		wmb();
		off += 4;
	}

	off = 0x0260;	/* phy timig ctrl 0 */
	for (i = 0; i < 11; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->timing[i]);
		wmb();
		off += 4;
	}

	/* clock out ctrl */
	MIPI_OUTP(MIPI_DSI_BASE + 0xc0, pd->clkout);
	wmb();

	off = 0x0290;	/* ctrl 0 */
	for (i = 0; i < 4; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->ctrl[i]);
		wmb();
		off += 4;
	}

	off = 0x02a0;	/* strength 0 */
	for (i = 0; i < 4; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->strength[i]);
		wmb();
		off += 4;
	}

	off = 0x0204;	/* pll ctrl 1, skip 0 */
	for (i = 1; i < 21; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->pll[i]);
		wmb();
		off += 4;
	}

	/* pll ctrl 0 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x200, pd->pll[0]);
	wmb();
}

static int mipi_dsi_off(struct platform_device *pdev)
{
	int ret = 0;

	ret = panel_next_off(pdev);

	if (mipi_dsi_pdata && mipi_dsi_pdata->dsi_power_save)
		mipi_dsi_pdata->dsi_power_save(0);

	return ret;
}

static int mipi_dsi_on(struct platform_device *pdev)
{
	int ret = 0;
	u32 clk_rate;
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct fb_var_screeninfo *var;
	u32 hbp, hfp, vbp, vfp, hspw, vspw, width, height;

	mfd = platform_get_drvdata(pdev);
	fbi = mfd->fbi;
	var = &fbi->var;

	if (mipi_dsi_pdata && mipi_dsi_pdata->dsi_power_save)
		mipi_dsi_pdata->dsi_power_save(1);

	clk_rate = mfd->fbi->var.pixclock;
	clk_rate = min(clk_rate, mfd->panel_info.clk_max);

	if (clk_set_rate(dsi_byte_div_clk, 1) < 0)	/* divided by 1 */
		printk(KERN_ERR "%s: clk_set_rate failed\n",
			__func__);

	clk_enable(amp_pclk);
	clk_enable(dsi_m_pclk);
	clk_enable(dsi_byte_div_clk);
	clk_enable(dsi_esc_clk);

	hbp = var->left_margin;
	hfp = var->right_margin;
	vbp = var->upper_margin;
	vfp = var->lower_margin;
	hspw = var->hsync_len;
	vspw = var->vsync_len;
	width = mfd->panel_info.xres;
	height = mfd->panel_info.yres;

	mipi_dsi_ahb_en();
	mipi_dsi_sfpb_cfg();
	mipi_dsi_clk(1);
	mipi_dsi_pclk(1);

	mipi_dsi_phy_init(0); /* toshiba video */

	enable_irq(DSI_IRQ);

	/*
	 * turn esc, byte, dsi, pclk, sclk, hclk on
	 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x118, 0x23f);	/* DSI_CLK_CTRL */

	if (mfd->panel_info.type == MIPI_VIDEO_PANEL) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x20,
			((hbp + width - 1) << 16 | (hbp - 1)));
		MIPI_OUTP(MIPI_DSI_BASE + 0x24,
			((vbp + height - 1) << 16 | (vbp - 1)));
		MIPI_OUTP(MIPI_DSI_BASE + 0x28,
			(vbp + height + vfp - 1) << 16 |
				(hbp + width + hfp - 1));
		MIPI_OUTP(MIPI_DSI_BASE + 0x2c, (hspw - 1) << 16);
		MIPI_OUTP(MIPI_DSI_BASE + 0x30, 0);
		MIPI_OUTP(MIPI_DSI_BASE + 0x34, (vspw - 1) << 16);

		/* there has hardware problem
		 * the color channel between dsi and mdp are swapped
		 */
		MIPI_OUTP(MIPI_DSI_BASE + 0x1c, 0x2000); /* rGB --> BGR */

		/* embedded  mode, lpm */
		MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x14000000);

		/* non burst sync pulse, 888RGB */
		MIPI_OUTP(MIPI_DSI_BASE + 0x0c, 0x11119030);
	}

	MIPI_OUTP(MIPI_DSI_BASE + 0x0080, 0x04);/* sw trigger */
	/* command mode */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0, 0x135);

	wmb();

	ret = panel_next_on(pdev);

	/* video mode */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0, 0x133);
	wmb();

	return ret;
}

static int mipi_dsi_resource_initialized;

static int mipi_dsi_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct platform_device *mdp_dev = NULL;
	struct msm_fb_panel_data *pdata = NULL;
	int rc;
	resource_size_t size ;

	if ((pdev->id == 0) && (pdev->num_resources >= 0)) {
		mipi_dsi_pdata = pdev->dev.platform_data;

		size =  resource_size(&pdev->resource[0]);
		mipi_dsi_base =  ioremap(pdev->resource[0].start, size);

		MSM_FB_INFO("mipi_dsi base phy_addr = 0x%x virt = 0x%x\n",
				pdev->resource[0].start, (int) mipi_dsi_base);

		if (!mipi_dsi_base)
			return -ENOMEM;

		mmss_cc_base =  ioremap(MMSS_CC_BASE_PHY, 0x200);
		MSM_FB_INFO("mmss_cc base phy_addr = 0x%x virt = 0x%x\n",
				MMSS_CC_BASE_PHY, (int) mmss_cc_base);

		if (!mmss_cc_base)
			return -ENOMEM;

		mmss_sfpb_base =  ioremap(MMSS_SFPB_BASE_PHY, 0x100);
		MSM_FB_INFO("mmss_sfpb  base phy_addr = 0x%x virt = 0x%x\n",
				MMSS_SFPB_BASE_PHY, (int) mmss_sfpb_base);

		if (!mmss_cc_base)
			return -ENOMEM;

		rc = request_irq(DSI_IRQ, mipi_dsi_isr, IRQF_DISABLED,
						"MIPI_DSI", 0);
		if (rc) {
			printk(KERN_ERR "mipi_dsi_host request_irq() failed!\n");
			return rc;
		}

		disable_irq(DSI_IRQ);

		mipi_dsi_calibration();

		if (mipi_dsi_mxo_selected())
			dsi_cc_data |= BIT(8);	/* use MXO for DSI PLL clkref */
		else
			dsi_cc_data &= ~BIT(8);	/* use PXO */

		mipi_dsi_resource_initialized = 1;

		return 0;
	}

	if (!mipi_dsi_resource_initialized)
		return -EPERM;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;


	mdp_dev = platform_device_alloc("mdp", pdev->id);
	if (!mdp_dev)
		return -ENOMEM;

	/*
	 * link to the latest pdev
	 */
	mfd->pdev = mdp_dev;
	mfd->dest = DISPLAY_LCD;

	/*
	 * alloc panel device data
	 */
	if (platform_device_add_data
	    (mdp_dev, pdev->dev.platform_data,
	     sizeof(struct msm_fb_panel_data))) {
		printk(KERN_ERR "mipi_dsi_probe: platform_device_add_data failed!\n");
		platform_device_put(mdp_dev);
		return -ENOMEM;
	}
	/*
	 * data chain
	 */
	pdata = mdp_dev->dev.platform_data;
	pdata->on = mipi_dsi_on;
	pdata->off = mipi_dsi_off;
	pdata->next = pdev;

	/*
	 * get/set panel specific fb info
	 */
	mfd->panel_info = pdata->panel_info;


#ifdef MSMFB_FRAMEBUF_32
	if (mfd->index == 0)
		mfd->fb_imgType = MDP_RGBA_8888; /* primary */
	else
		mfd->fb_imgType = MDP_RGB_565;	/* secondary */
#else
	mfd->fb_imgType = MDP_RGB_565;
#endif

	fbi = mfd->fbi;
	fbi->var.pixclock = mfd->panel_info.clk_rate;
	fbi->var.left_margin = mfd->panel_info.lcdc.h_back_porch;
	fbi->var.right_margin = mfd->panel_info.lcdc.h_front_porch;
	fbi->var.upper_margin = mfd->panel_info.lcdc.v_back_porch;
	fbi->var.lower_margin = mfd->panel_info.lcdc.v_front_porch;
	fbi->var.hsync_len = mfd->panel_info.lcdc.h_pulse_width;
	fbi->var.vsync_len = mfd->panel_info.lcdc.v_pulse_width;

#ifdef DSI_CLK
	clk_rate = mfd->panel_info.clk_max;
	if (clk_set_max_rate(mipi_dsi_clk, clk_rate) < 0)
		printk(KERN_ERR "%s: clk_set_max_rate failed\n", __func__);
	mfd->panel_info.clk_rate = mfd->panel_info.clk_min;
#endif

	/*
	 * set driver data
	 */
	platform_set_drvdata(mdp_dev, mfd);

	/*
	 * register in mdp driver
	 */
	rc = platform_device_add(mdp_dev);
	if (rc)
		goto mipi_dsi_probe_err;

	pdev_list[pdev_list_cnt++] = pdev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	mfd->mipi_dsi_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	mfd->mipi_dsi_early_suspend.suspend = mipi_dsi_early_suspend;
	mfd->mipi_dsi_early_suspend.resume = mipi_dsi_early_resume;
	register_early_suspend(&mfd->mipi_dsi_early_suspend);
#endif

	return 0;

mipi_dsi_probe_err:
	platform_device_put(mdp_dev);
	return rc;
}

#ifdef CONFIG_PM
static int mipi_dsi_is_in_suspend;

static int mipi_dsi_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (mipi_dsi_is_in_suspend)
		return 0;

	mipi_dsi_is_in_suspend = 1;


	/*
	 * set clock rate to 0
	 * disable clocks
	 */

	if (mipi_dsi_pdata && mipi_dsi_pdata->dsi_power_save)
		mipi_dsi_pdata->dsi_power_save(0);

	return 0;
}

static int mipi_dsi_resume(struct platform_device *pdev)
{
	if (!mipi_dsi_is_in_suspend)
		return 0;

	mipi_dsi_is_in_suspend = 0;

	/* clocks enable */

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mipi_dsi_early_suspend(struct early_suspend *h)
{
	pm_message_t state;
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
							mipi_dsi_early_suspend);

	state.event = PM_EVENT_SUSPEND;
	mipi_dsi_suspend(mfd->pdev, state);
}

static void mipi_dsi_early_resume(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
							mipi_dsi_early_suspend);
	mipi_dsi_resume(mfd->pdev);
}
#endif

static int mipi_dsi_remove(struct platform_device *pdev)
{
	iounmap(msm_pmdh_base);

	return 0;
}

static int mipi_dsi_register_driver(void)
{
	return platform_driver_register(&mipi_dsi_driver);
}

static int __init mipi_dsi_driver_init(void)
{
	int ret;

	amp_pclk = clk_get(NULL, "amp_pclk");
	if (IS_ERR(amp_pclk)) {
		printk(KERN_ERR "can't find amp_pclk\n");
		return PTR_ERR(amp_pclk);
	}

	dsi_m_pclk = clk_get(NULL, "dsi_m_pclk");
	if (IS_ERR(dsi_m_pclk)) {
		printk(KERN_ERR "can't find dsi_m_pclk\n");
		return PTR_ERR(dsi_m_pclk);
	}

	dsi_byte_div_clk = clk_get(NULL, "dsi_byte_div_clk");
	if (IS_ERR(dsi_byte_div_clk)) {
		printk(KERN_ERR "can't find dsi_byte_div_clk\n");
		return PTR_ERR(dsi_byte_div_clk);
	}


	dsi_esc_clk = clk_get(NULL, "dsi_esc_clk");
	if (IS_ERR(dsi_esc_clk)) {
		printk(KERN_ERR "can't find dsi_byte_div_clk\n");
		return PTR_ERR(dsi_byte_div_clk);
	}

	ret = mipi_dsi_register_driver();

	device_initialize(&dsi_dev);

	if (ret) {
		clk_disable(amp_pclk);
		clk_put(amp_pclk);
		clk_disable(dsi_m_pclk);
		clk_put(dsi_m_pclk);
		clk_disable(dsi_byte_div_clk);
		clk_put(dsi_byte_div_clk);
		clk_disable(dsi_esc_clk);
		clk_put(dsi_esc_clk);
		printk(KERN_ERR "mipi_dsi_register_driver() failed!\n");
		return ret;
	}

	return ret;
}

module_init(mipi_dsi_driver_init);
