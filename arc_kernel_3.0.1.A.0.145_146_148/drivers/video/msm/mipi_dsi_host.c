
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
#include <mach/dma.h>

#include "msm_fb.h"
#include "mipi_dsi.h"


/*
 * mipi dsi buf mechanism
 */
char *mipi_dsi_buf_reserve(struct dsi_buf *dp, int len)
{
	dp->data += len;
	return dp->data;
}

char *mipi_dsi_buf_unreserve(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	return dp->data;
}

char *mipi_dsi_buf_push(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	dp->len += len;
	return dp->data;
}

char *mipi_dsi_buf_reserve_hdr(struct dsi_buf *dp, int hlen)
{
	dp->hdr = (uint32 *)dp->data;
	return mipi_dsi_buf_reserve(dp, hlen);
}

char *mipi_dsi_buf_init(struct dsi_buf *dp)
{
	int off;

	dp->data = dp->start;
	off = (int)dp->data;
	/* 8 byte align */
	off &= 0x07;
	if (off)
		off = 8 - off;
	dp->data += off;
	dp->len = 0;
	return dp->data;
}

int mipi_dsi_buf_alloc(struct dsi_buf *dp, int size)
{

	dp->start = kmalloc(size, GFP_KERNEL);
	if (dp->start == NULL) {
		printk(KERN_ERR "%s:%u\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dp->end = dp->start + size;
	dp->size = size;

	if ((int)dp->start & 0x07)
		printk(KERN_ERR "%s: buf NOT 8 bytes aligned\n", __func__);

	dp->data = dp->start;
	dp->len = 0;
	return size;
}

/*
 * mipi dsi gerneric long write
 */
static int mipi_dsi_generic_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	char *bp;
	uint32 *hp;
	int i, len;

	bp = mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	/* fill up payload */
	if (cm->payload) {
		len = cm->dlen;
		len += 3;
		len &= ~0x03;	/* multipled by 4 */
		for (i = 0; i < cm->dlen; i++)
			*bp++ = cm->payload[i];

		/* append 0xff to the end */
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	/* fill up header */
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_GEN_LWRITE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}

/*
 * mipi dsi gerneric short write with 0, 1 2 parameters
 */
static int mipi_dsi_generic_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->dlen && cm->payload == 0) {
		printk(KERN_ERR "%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->last)
		*hp |= DSI_HDR_LAST;


	len = (cm->dlen > 2) ? 2 : cm->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

/*
 * mipi dsi gerneric read with 0, 1 2 parameters
 */
static int mipi_dsi_generic_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->dlen && cm->payload == 0) {
		printk(KERN_ERR "%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	len = (cm->dlen > 2) ? 2 : cm->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return dp->len;	/* 4 bytes */
}

/*
 * mipi dsi dcs long write
 */
static int mipi_dsi_dcs_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	char *bp;
	uint32 *hp;
	int i, len;

	bp = mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	/*
	 * fill up payload
	 * dcs command byte (first byte) followed by payload
	 */
	if (cm->payload) {
		len = cm->dlen;
		len += 3;
		len &= ~0x03;	/* multipled by 4 */
		for (i = 0; i < cm->dlen; i++)
			*bp++ = cm->payload[i];

		/* append 0xff to the end */
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	/* fill up header */
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_LWRITE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}

/*
 * mipi dsi dcs short write with 0 parameters
 */
static int mipi_dsi_dcs_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->payload == 0) {
		printk(KERN_ERR "%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->ack)		/* ask ACK trigger msg from peripeheral */
		*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	len = (cm->dlen > 1) ? 1 : cm->dlen;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs command byte */
	*hp |= DSI_HDR_DATA2(0);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return dp->len;
}

/*
 * mipi dsi dcs short write with 1 parameters
 */
static int mipi_dsi_dcs_swrite1(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->dlen < 2 || cm->payload == 0) {
		printk(KERN_ERR "%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->ack)		/* ask ACK trigger msg from peripeheral */
		*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE1);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs comamnd byte */
	*hp |= DSI_HDR_DATA1(cm->payload[1]);	/* parameter */

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}
/*
 * mipi dsi dcs read with 0 parameters
 */

static int mipi_dsi_dcs_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->payload == 0) {
		printk(KERN_ERR "%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_BTA;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_READ);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs command byte */
	*hp |= DSI_HDR_DATA2(0);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_cm_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_ON);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_cm_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_OFF);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_peripheral_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_ON);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_peripheral_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_OFF);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_set_max_pktsize(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->payload == 0) {
		printk(KERN_ERR "%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_MAX_PKTSIZE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);
	*hp |= DSI_HDR_DATA2(cm->payload[1]);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_null_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_NULL_PKT);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_blank_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_BLANK_PKT);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

/*
 * prepare cmd buffer to be txed
 */
int mipi_dsi_dma_cmd_add(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	int len = 0;

	switch (cm->dtype) {
	case DTYPE_GEN_WRITE:
	case DTYPE_GEN_WRITE1:
	case DTYPE_GEN_WRITE2:
		len = mipi_dsi_generic_swrite(dp, cm);
		break;
	case DTYPE_GEN_LWRITE:
		len = mipi_dsi_generic_lwrite(dp, cm);
		break;
	case DTYPE_GEN_READ:
	case DTYPE_GEN_READ1:
	case DTYPE_GEN_READ2:
		len = mipi_dsi_generic_read(dp, cm);
		break;
	case DTYPE_DCS_LWRITE:
		len = mipi_dsi_dcs_lwrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE:
		len = mipi_dsi_dcs_swrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE1:
		len = mipi_dsi_dcs_swrite1(dp, cm);
		break;
	case DTYPE_DCS_READ:
		len = mipi_dsi_dcs_read(dp, cm);
		break;
	case DTYPE_MAX_PKTSIZE:
		len = mipi_dsi_set_max_pktsize(dp, cm);
		break;
	case DTYPE_NULL_PKT:
		len = mipi_dsi_null_pkt(dp, cm);
		break;
	case DTYPE_BLANK_PKT:
		len = mipi_dsi_blank_pkt(dp, cm);
		break;
	case DTYPE_CM_ON:
		len = mipi_dsi_cm_on(dp, cm);
		break;
	case DTYPE_CM_OFF:
		len = mipi_dsi_cm_off(dp, cm);
		break;
	case DTYPE_PERIPHERAL_ON:
		len = mipi_dsi_peripheral_on(dp, cm);
		break;
	case DTYPE_PERIPHERAL_OFF:
		len = mipi_dsi_peripheral_off(dp, cm);
		break;
	default:
		printk(KERN_INFO "%s: dtype=%x NOT supported\n",
					__func__, cm->dtype);
		break;

	}

	return len;
}

int mipi_dsi_cmd_reg_tx(uint32 data)
{
#ifdef DSI_HOST_DEBUG
	int i;
	char *bp;

	bp = (char *)&data;
	printk(KERN_INFO "%s: ", __func__);
	for (i = 0; i < 4; i++)
		printk(KERN_INFO "%x ", *bp++);

	printk(KERN_INFO "\n");
#endif

	MIPI_OUTP(MIPI_DSI_BASE + 0x0080, 0x04);/* sw trigger */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0, 0x135);

	wmb();

	MIPI_OUTP(MIPI_DSI_BASE + 0x038, data);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x08c, 0x01);	/* trigger */
	wmb();

	udelay(300);

	return 4;
}


static struct completion dsi_comp;

void mipi_dsi_cmd_mode_ctrl(int enable)
{
	uint32 data;

	MIPI_OUTP(MIPI_DSI_BASE + 0x0080, 0x04);/* sw trigger */
	data = MIPI_INP(MIPI_DSI_BASE + 0x0);
	if (enable)
		data |= 0x04;	/* cmd mode enable */
	else
		data &= ~0x04; /* cmd disable */

	MIPI_OUTP(MIPI_DSI_BASE + 0x0, data);
	wmb();
}

int mipi_dsi_dma_cmd_tx(struct dsi_buf *dp)
{
	int len;

#ifdef DSI_HOST_DEBUG
	int i;
	char *bp;

	bp = dp->data;

	printk(KERN_INFO "%s: ", __func__);
	for (i = 0; i < dp->len; i++)
		printk(KERN_INFO "%x ", *bp++);

	printk(KERN_INFO "\n");
#endif

	len = dp->len;
	len += 3;
	len &= ~0x03;	/* multipled by 4 */

	dp->dmap = dma_map_single(&dsi_dev, dp->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(&dsi_dev, dp->dmap))
		printk(KERN_ERR "%s: dmap mapp failed\n", __func__);

	init_completion(&dsi_comp);

	MIPI_OUTP(MIPI_DSI_BASE + 0x044, dp->dmap);
	MIPI_OUTP(MIPI_DSI_BASE + 0x048, len);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x08c, 0x01);	/* trigger */
	wmb();

	wait_for_completion(&dsi_comp);

	dma_unmap_single(&dsi_dev, dp->dmap, len, DMA_TO_DEVICE);
	dp->dmap = 0;
	return dp->len;
}

irqreturn_t mipi_dsi_isr(int irq, void *ptr)
{
	uint32 isr;

	isr = MIPI_INP(MIPI_DSI_BASE + 0x010c);/* DSI_INTR_CTRL */
	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, isr);

	if (isr & DSI_ERROR_STAT) {
		/*
		* do something  here
		*/
	}

	complete(&dsi_comp);

	return IRQ_HANDLED;
}

