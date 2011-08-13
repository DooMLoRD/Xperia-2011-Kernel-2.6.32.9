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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_toshiba.h"

static struct msm_panel_common_pdata *mipi_toshiba_pdata;

static struct dsi_buf toshiba_tx_buf;
static struct dsi_buf toshiba_rx_buf;


char mcap_off[2] = {0xb2, 0x00};
char ena_test_reg[3] = {0xEF, 0x01, 0x01};
char two_lane[3] = {0xEF, 0x60, 0x63};
char one_lane[3] = {0xEF, 0x60, 0x62};
char non_burst_sync_pulse[3] = {0xef, 0x61, 0x09};
char dmode_wvga[2] = {0xB3, 0x00};
char dmode_wqvga[2] = {0xB3, 0x01};
char intern_wr_clk1_wvga[3] = {0xef, 0x2f, 0xcc};
char intern_wr_clk2_wvga[3] = {0xef, 0x6e, 0xdd};
char intern_wr_clk1_wqvga[3] = {0xef, 0x2f, 0x22};
char intern_wr_clk2_wqvga[3] = {0xef, 0x6e, 0x33};
char hor_addr_2A_wvga[5] = {0x2A, 0x00, 0x00, 0x01, 0xdf};
char hor_addr_2B_wvga[5] = {0x2B, 0x00, 0x00, 0x03, 0x55};
char hor_addr_2A_wqvga[5] = {0x2A, 0x00, 0x00, 0x00, 0xef};
char hor_addr_2B_wqvga[5] = {0x2B, 0x00, 0x00, 0x01, 0xaa};
char if_sel_video[2] = {0x53, 0x01};
char if_sel_cmd[2] = {0x53, 0x00};
char exit_sleep[2] = {0x11, 0x00};
char display_on[2] = {0x29, 0x00};
char enter_sleep[2] = {0x10, 0x00};
char max_pktsize[2] = {0x00, 0x04};	/* LSB tx first */

static struct dsi_cmd_desc toshiba_mcmd_set[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(mcap_off), mcap_off},
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(ena_test_reg), ena_test_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(two_lane), two_lane},
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(non_burst_sync_pulse),
					non_burst_sync_pulse},
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(dmode_wvga), dmode_wvga},
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(intern_wr_clk1_wvga),
					intern_wr_clk1_wvga},
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(intern_wr_clk2_wvga),
					intern_wr_clk2_wvga},
	{DTYPE_DCS_LWRITE, 1, 0, 0, sizeof(hor_addr_2A_wvga),
					hor_addr_2A_wvga},
	{DTYPE_DCS_LWRITE, 1, 0, 0, sizeof(hor_addr_2B_wvga),
					hor_addr_2B_wvga},
	{DTYPE_GEN_LWRITE, 1, 0, 0, sizeof(if_sel_video), if_sel_video},
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, sizeof(max_pktsize), max_pktsize},
	{DTYPE_DCS_WRITE, 1, 0, 0, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, sizeof(display_on), display_on}
};

#define TOSHIBA_MCMD_MAX (sizeof(toshiba_mcmd_set)/sizeof(struct dsi_cmd_desc))

void mipi_toshiba_manufacture_cmds(void)
{
	struct dsi_buf *dp;
	struct dsi_cmd_desc *cm;
	int i;

	dp = &toshiba_tx_buf;
	mipi_dsi_buf_init(dp);
	for (i = 0; i < TOSHIBA_MCMD_MAX; i++) {
		cm = &toshiba_mcmd_set[i];
		mipi_dsi_buf_init(dp);
		mipi_dsi_dma_cmd_add(dp, cm);
		mipi_dsi_dma_cmd_tx(dp);
	}
}

static int mipi_toshiba_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmd_mode_ctrl(1);	/* enable cmd mode */
	mipi_toshiba_manufacture_cmds();
	mipi_dsi_cmd_mode_ctrl(0);	/* disable cmd mode */

	return 0;
}

static int mipi_toshiba_lcd_off(struct platform_device *pdev)
{
	return 0;
}

static int __init mipi_toshiba_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_toshiba_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_toshiba_lcd_probe,
	.driver = {
		.name   = "mipi_toshiba",
	},
};

static struct msm_fb_panel_data toshiba_panel_data = {
	.on		= mipi_toshiba_lcd_on,
	.off		= mipi_toshiba_lcd_off,
};

static int ch_used[3];

int mipi_toshiba_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_toshiba", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	toshiba_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &toshiba_panel_data,
		sizeof(toshiba_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_toshiba_lcd_init(void)
{
	mipi_dsi_buf_alloc(&toshiba_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&toshiba_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_toshiba_lcd_init);
