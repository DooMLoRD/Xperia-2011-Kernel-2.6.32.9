/* Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2008 HTC Corporation
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */

#include <mach/debug_mm.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>

#include "msm8x60-pcm.h"

/* Audrec Queue command sent macro's */
#define audrec_send_bitstreamqueue(audio, cmd, len) \
	msm_adsp_write(audio->audrec, ((audio->queue_id & 0xFFFF0000) >> 16),\
		cmd, len)

#define audrec_send_audrecqueue(audio, cmd, len) \
	msm_adsp_write(audio->audrec, (audio->queue_id & 0x0000FFFF),\
		cmd, len)

static int alsa_in_enc_config(struct msm_audio *audio, int enable);

int intcnt;
struct audio_frame {
	uint16_t count_low;
	uint16_t count_high;
	uint16_t bytes;
	uint16_t unknown;
	unsigned char samples[];
} __attribute__ ((packed));

int alsa_audio_configure(struct msm_audio *prtd)
{
	if (prtd->enabled)
		return 0;

	MM_DBG("\n");
	if (prtd->dir == SNDRV_PCM_STREAM_PLAYBACK)
		prtd->out_weight = 100;

	if (prtd->dir == SNDRV_PCM_STREAM_CAPTURE)
		alsa_in_enc_config(prtd, 1);

	prtd->enabled = 1;
	return 0;
}
EXPORT_SYMBOL(alsa_audio_configure);

ssize_t alsa_send_buffer(struct msm_audio *prtd, const char __user *buf,
			  size_t count, loff_t *pos)
{
	unsigned long flag = 0;
	const char __user *start = buf;
	struct buffer *frame;
	size_t xfer;
	int ret = 0;

	MM_DBG("\n");
	mutex_lock(&the_locks.write_lock);
	while (count > 0) {
		frame = prtd->out + prtd->out_head;
		ret = wait_event_interruptible(the_locks.write_wait,
					      (frame->used == 0)
					      || (prtd->stopped));
		if (ret < 0)
			break;
		if (prtd->stopped) {
			ret = -EBUSY;
			break;
		}
		xfer = count > frame->size ? frame->size : count;
		if (copy_from_user(frame->data, buf, xfer)) {
			ret = -EFAULT;
			break;
		}
		frame->used = xfer;
		prtd->out_head ^= 1;
		count -= xfer;
		buf += xfer;

		spin_lock_irqsave(&the_locks.write_dsp_lock, flag);
		frame = prtd->out + prtd->out_tail;
		if (frame->used && prtd->out_needed) {
			alsa_dsp_send_buffer(prtd, prtd->out_tail,
					      frame->used);
			/* Reset eos_ack flag to avoid stale
			 * PCMDMAMISS been considered
			 */
			prtd->eos_ack = 0;
			prtd->out_tail ^= 1;
			prtd->out_needed--;
		}
		spin_unlock_irqrestore(&the_locks.write_dsp_lock, flag);
	}
	mutex_unlock(&the_locks.write_lock);
	if (buf > start)
		return buf - start;
	return ret;
}
EXPORT_SYMBOL(alsa_send_buffer);

int alsa_audio_disable(struct msm_audio *prtd)
{
	if (prtd->enabled) {
		MM_DBG("\n");
		mutex_lock(&the_locks.lock);
		prtd->enabled = 0;
		audio_dsp_out_enable(prtd, 0);
		wake_up(&the_locks.write_wait);
		prtd->out_needed = 0;
		mutex_unlock(&the_locks.lock);
	}
	return 0;
}
EXPORT_SYMBOL(alsa_audio_disable);

int alsa_audrec_disable(struct msm_audio *prtd)
{
	if (prtd->enabled) {
		prtd->enabled = 0;
		alsa_in_enc_config(prtd, 0);
		wake_up(&the_locks.read_wait);
		prtd->out_needed = 0;
	}
	return 0;
}
EXPORT_SYMBOL(alsa_audrec_disable);

static int alsa_in_enc_config(struct msm_audio *prtd, int enable)
{
	return 0;
}

int alsa_in_record_config(struct msm_audio *prtd, int enable)
{
	return 0;
}

int audio_dsp_out_enable(struct msm_audio *prtd, int yes)
{
	return 0;
}

int alsa_buffer_read(struct msm_audio *prtd, void __user *buf,
		      size_t count, loff_t *pos)
{
	unsigned long flag;
	void *data;
	uint32_t index;
	uint32_t size;
	int ret = 0;

	mutex_lock(&the_locks.read_lock);
	while (count > 0) {
		ret = wait_event_interruptible(the_locks.read_wait,
					      (prtd->in_count > 0)
					      || prtd->stopped ||
						  prtd->abort);

		if (ret < 0)
			break;

		if (prtd->stopped) {
			ret = -EBUSY;
			break;
		}

		if (prtd->abort) {
			MM_DBG(" prtd->abort ! \n");
			ret = -EPERM; /* Not permitted due to abort */
			break;
		}

		index = prtd->in_tail;
		data = (uint8_t *) prtd->in[index].data;
		size = prtd->in[index].size;
		if (count >= size) {
			if (copy_to_user(buf, data, size)) {
				ret = -EFAULT;
				break;
			}
			spin_lock_irqsave(&the_locks.read_dsp_lock, flag);
			if (index != prtd->in_tail) {
				/* overrun: data is invalid, we need to retry */
				spin_unlock_irqrestore(&the_locks.read_dsp_lock,
						       flag);
				continue;
			}
			prtd->in[index].size = 0;
			prtd->in_tail = (prtd->in_tail + 1) & (FRAME_NUM - 1);
			prtd->in_count--;
			spin_unlock_irqrestore(&the_locks.read_dsp_lock, flag);
			count -= size;
			buf += size;
		} else {
			break;
		}
	}
	mutex_unlock(&the_locks.read_lock);
	return ret;
}
EXPORT_SYMBOL(alsa_buffer_read);

int alsa_dsp_send_buffer(struct msm_audio *prtd,
					unsigned idx, unsigned len)
{
	return 0;
}


