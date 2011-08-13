/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#ifndef MIPI_DSI_H
#define MIPI_DSI_H

#ifdef BIT
#undef BIT
#endif

#define BIT(x)  (1<<(x))

#define MMSS_CC_BASE_PHY 0x04000000	/* mmss clcok control */
#define MMSS_SFPB_BASE_PHY 0x05700000	/* mmss SFPB CFG */

#define MIPI_DSI_BASE mipi_dsi_base

#define MIPI_OUTP(addr, data) writel((data), (addr))
#define MIPI_INP(addr) readl(addr)

#define MIPI_DSI_PRIM 1
#define MIPI_DSI_SECD 2

#define MIPI_DSI_PANEL_VGA	0
#define MIPI_DSI_PANEL_WVGA	1
#define MIPI_DSI_PANEL_WVGA_PT	2

#define DSI_PANEL_MAX	2



#define DSI_ERROR_STAT BIT(24)

extern struct device dsi_dev;

struct dsi_clk_desc {
	uint32 src;
	uint32 m;
	uint32 n;
	uint32 d;
};

struct dsi_phy_ctrl {
	uint32 regulator[4];
	uint32 timing[12];
	uint32 ctrl[4];
	uint32 strength[4];
	uint32 pll[21];
	uint32 clkout;
};

#define DSI_HOST_HDR_SIZE	4
#define DSI_HDR_LAST		BIT(31)
#define DSI_HDR_LONG_PKT	BIT(30)
#define DSI_HDR_BTA		BIT(29)
#define DSI_HDR_VC(vc)		(((vc) & 0x03) << 22)
#define DSI_HDR_DTYPE(dtype)	(((dtype) & 0x03f) << 16)
#define DSI_HDR_DATA2(data)	(((data) & 0x0ff) << 8)
#define DSI_HDR_DATA1(data)	((data) & 0x0ff)
#define DSI_HDR_WC(wc)		((wc) & 0x0ffff)

#define DSI_BUF_SIZE	2048

struct dsi_buf {
	uint32 *hdr;	/* dsi host header */
	char *start;	/* buffer start addr */
	char *end;	/* buffer end addr */
	int size;	/* size of buffer */
	char *data;	/* buffer */
	int len;	/* data length */
	dma_addr_t dmap; /* mapped dma addr */
};

/* dcs read/write */
#define DTYPE_DCS_WRITE		0x05	/* short write, 0 parameter */
#define DTYPE_DCS_WRITE1	0x15	/* short write, 1 parameter */
#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_LWRITE	0x39	/* long write */

/* generic read/write */
#define DTYPE_GEN_WRITE		0x03	/* short write, 0 parameter */
#define DTYPE_GEN_WRITE1	0x13	/* short write, 1 parameter */
#define DTYPE_GEN_WRITE2	0x23	/* short write, 2 parameter */
#define DTYPE_GEN_LWRITE	0x29	/* long write */
#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_READ1		0x14	/* long read, 1 parameter */
#define DTYPE_GEN_READ2		0x24	/* long read, 2 parameter */

#define DTYPE_MAX_PKTSIZE	0x37	/* set max packet size */
#define DTYPE_NULL_PKT		0x09	/* null packet, no data */
#define DTYPE_BLANK_PKT		0x19	/* blankiing packet, no data */

#define DTYPE_CM_ON		0x02	/* color mode off */
#define DTYPE_CM_OFF		0x12	/* color mode on */
#define DTYPE_PERIPHERAL_OFF	0x22
#define DTYPE_PERIPHERAL_ON	0x32


struct dsi_cmd_desc {
	int dtype;
	int last;
	int vc;
	int ack;	/* ask ACK from peripheral */
	int dlen;
	char *payload;
};

char *mipi_dsi_buf_reserve_hdr(struct dsi_buf *dp, int hlen);
char *mipi_dsi_buf_init(struct dsi_buf *dp);
int mipi_dsi_buf_alloc(struct dsi_buf *, int size);
int mipi_dsi_dma_cmd_add(struct dsi_buf *dp, struct dsi_cmd_desc *cm);
int mipi_dsi_dma_cmd_tx(struct dsi_buf *dp);
int mipi_dsi_cmd_reg_tx(uint32 data);
void mipi_dsi_host_init(void);
void mipi_dsi_cmd_mode_ctrl(int enable);

irqreturn_t mipi_dsi_isr(int irq, void *ptr);

#endif /* MIPI_DSI_H */
