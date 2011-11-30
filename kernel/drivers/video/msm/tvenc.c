/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#include <mach/hardware.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_qos_params.h>
#include <mach/msm_reqs.h>

#define TVENC_C
#include "tvenc.h"
#include "msm_fb.h"

#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
/* NPA Flow ID */
#define MSM_SYSTEM_BUS_RATE	MSM_AXI_FLOW_MDP_DTV_720P_2BPP
#else
/* AXI rate in KHz */
#define MSM_SYSTEM_BUS_RATE	128000
#endif

static int tvenc_probe(struct platform_device *pdev);
static int tvenc_remove(struct platform_device *pdev);

static int tvenc_off(struct platform_device *pdev);
static int tvenc_on(struct platform_device *pdev);

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;

static struct clk *tvenc_clk;
static struct clk *tvdac_clk;
#ifdef CONFIG_FB_MSM_MDP40
static struct clk *tv_src_clk;
#endif

static int tvenc_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int tvenc_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static struct dev_pm_ops tvenc_dev_pm_ops = {
	.runtime_suspend = tvenc_runtime_suspend,
	.runtime_resume = tvenc_runtime_resume,
};

static struct platform_driver tvenc_driver = {
	.probe = tvenc_probe,
	.remove = tvenc_remove,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "tvenc",
		   .pm = &tvenc_dev_pm_ops
		   },
};

static struct tvenc_platform_data *tvenc_pdata;

static int tvenc_off(struct platform_device *pdev)
{
	int ret = 0;

	ret = panel_next_off(pdev);

	if (ret)
		printk(KERN_ERR "%s: pm_vid_en(off) failed! %d\n",
		__func__, ret);

#ifdef CONFIG_FB_MSM_MDP40
	clk_disable(tv_src_clk);
#endif
	clk_disable(tvdac_clk);
	clk_disable(tvenc_clk);

	if (tvenc_pdata && tvenc_pdata->pm_vid_en)
		ret = tvenc_pdata->pm_vid_en(0);

	pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ , "tvenc",
				  PM_QOS_DEFAULT_VALUE);

	if (ret)
		printk(KERN_ERR "%s: pm_vid_en(off) failed! %d\n",
		__func__, ret);

	return ret;
}

static int tvenc_on(struct platform_device *pdev)
{
	int ret = 0;

	pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ , "tvenc",
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
				  MSM_SYSTEM_BUS_RATE);
#else
				  65000);
#endif
	if (tvenc_pdata && tvenc_pdata->pm_vid_en)
		ret = tvenc_pdata->pm_vid_en(1);

	if (ret) {
		printk(KERN_ERR "%s: pm_vid_en(on) failed! %d\n",
		__func__, ret);
		return ret;
	}
	ret = clk_enable(tvenc_clk);
    if (ret) {
		printk(KERN_ERR "%s: tvenc_clk enable failed! %d\n",
			__func__, ret);
		goto tvenc_err;
	}
    ret = clk_enable(tvdac_clk);
    if (ret) {
		printk(KERN_ERR "%s: tvdac_clk enable failed! %d\n",
			__func__, ret);
		goto tvdac_err;
	}
#ifdef CONFIG_FB_MSM_MDP40
    ret = clk_enable(tv_src_clk);
    if (ret) {
		printk(KERN_ERR "%s: tvsrc_clk enable failed! %d\n",
			__func__, ret);
		goto tvsrc_err;
    }
#endif
    ret = panel_next_on(pdev);
    return ret;

#ifdef CONFIG_FB_MSM_MDP40
tvsrc_err:
    clk_disable(tvdac_clk);
#endif
tvdac_err:
    clk_disable(tvenc_clk);
tvenc_err:
    return ret;
}

void tvenc_gen_test_pattern(struct msm_fb_data_type *mfd)
{
	uint32 reg = 0, i;

	reg = readl(MSM_TV_ENC_CTL);
	reg |= TVENC_CTL_TEST_PATT_EN;

	for (i = 0; i < 3; i++) {
		TV_OUT(TV_ENC_CTL, 0);	/* disable TV encoder */

		switch (i) {
			/*
			 * TV Encoder - Color Bar Test Pattern
			 */
		case 0:
			reg |= TVENC_CTL_TPG_CLRBAR;
			break;
			/*
			 * TV Encoder - Red Frame Test Pattern
			 */
		case 1:
			reg |= TVENC_CTL_TPG_REDCLR;
			break;
			/*
			 * TV Encoder - Modulated Ramp Test Pattern
			 */
		default:
			reg |= TVENC_CTL_TPG_MODRAMP;
			break;
		}

		TV_OUT(TV_ENC_CTL, reg);
		mdelay(5000);

		switch (i) {
			/*
			 * TV Encoder - Color Bar Test Pattern
			 */
		case 0:
			reg &= ~TVENC_CTL_TPG_CLRBAR;
			break;
			/*
			 * TV Encoder - Red Frame Test Pattern
			 */
		case 1:
			reg &= ~TVENC_CTL_TPG_REDCLR;
			break;
			/*
			 * TV Encoder - Modulated Ramp Test Pattern
			 */
		default:
			reg &= ~TVENC_CTL_TPG_MODRAMP;
			break;
		}
	}
}

static int tvenc_resource_initialized;

static int tvenc_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct platform_device *mdp_dev = NULL;
	struct msm_fb_panel_data *pdata = NULL;
	int rc;

	if (pdev->id == 0) {
		tvenc_base = ioremap(pdev->resource[0].start,
					pdev->resource[0].end -
					pdev->resource[0].start + 1);
		if (!tvenc_base) {
			printk(KERN_ERR
				"tvenc_base ioremap failed!\n");
			return -ENOMEM;
		}
		tvenc_pdata = pdev->dev.platform_data;
		tvenc_resource_initialized = 1;
		return 0;
	}

	if (!tvenc_resource_initialized)
		return -EPERM;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;

	if (tvenc_base == NULL)
		return -ENOMEM;

	mdp_dev = platform_device_alloc("mdp", pdev->id);
	if (!mdp_dev)
		return -ENOMEM;

	/*
	 * link to the latest pdev
	 */
	mfd->pdev = mdp_dev;
	mfd->dest = DISPLAY_TV;

	/*
	 * alloc panel device data
	 */
	if (platform_device_add_data
	    (mdp_dev, pdev->dev.platform_data,
	     sizeof(struct msm_fb_panel_data))) {
		printk(KERN_ERR "tvenc_probe: platform_device_add_data failed!\n");
		platform_device_put(mdp_dev);
		return -ENOMEM;
	}
	/*
	 * data chain
	 */
	pdata = mdp_dev->dev.platform_data;
	pdata->on = tvenc_on;
	pdata->off = tvenc_off;
	pdata->next = pdev;

	/*
	 * get/set panel specific fb info
	 */
	mfd->panel_info = pdata->panel_info;
#ifdef CONFIG_FB_MSM_MDP40
	mfd->fb_imgType = MDP_RGB_565;  /* base layer */
#else
	mfd->fb_imgType = MDP_YCRYCB_H2V1;
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
		goto tvenc_probe_err;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);



	pdev_list[pdev_list_cnt++] = pdev;
	return 0;

tvenc_probe_err:
	platform_device_put(mdp_dev);
	return rc;
}

static int tvenc_remove(struct platform_device *pdev)
{
	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ , "tvenc");

	pm_runtime_disable(&pdev->dev);
	return 0;
}

static int tvenc_register_driver(void)
{
	return platform_driver_register(&tvenc_driver);
}

static int __init tvenc_driver_init(void)
{
	tvenc_clk = clk_get(NULL, "tv_enc_clk");
	tvdac_clk = clk_get(NULL, "tv_dac_clk");

#ifdef CONFIG_FB_MSM_MDP40
	tv_src_clk = clk_get(NULL, "tv_src_clk");
	if (IS_ERR(tv_src_clk))
		tv_src_clk = tvenc_clk; /* Fallback to slave */
#endif

	if (IS_ERR(tvenc_clk)) {
		printk(KERN_ERR "error: can't get tvenc_clk!\n");
		return IS_ERR(tvenc_clk);
	}

	if (IS_ERR(tvdac_clk)) {
		printk(KERN_ERR "error: can't get tvdac_clk!\n");
		return IS_ERR(tvdac_clk);
	}

	pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ , "tvenc",
			       PM_QOS_DEFAULT_VALUE);
	return tvenc_register_driver();
}

module_init(tvenc_driver_init);
