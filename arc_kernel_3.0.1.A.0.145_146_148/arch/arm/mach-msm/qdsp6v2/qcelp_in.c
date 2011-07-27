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
 *
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio_qcp.h>
#include <asm/atomic.h>
#include <asm/ioctls.h>

#include <mach/debug_mm.h>
#include "apr_audio.h"
#include "q6asm.h"

#define AUDQCELP_EOS_SET  0x01
#define TUNNEL_MODE     0x0000
#define NON_TUNNEL_MODE 0x0001

#define NO_BUF_ALLOC	0x00
#define BUF_ALLOC_IN    0x01
#define BUF_ALLOC_OUT   0x02
#define BUF_ALLOC_INOUT 0x03

struct timestamp{
	unsigned long lowpart;
	unsigned long highpart;
} __attribute__ ((packed));

struct meta_in{
	unsigned short offset;
	struct timestamp ntimestamp;
	unsigned int nflags;
} __attribute__ ((packed));

struct meta_out_dsp{
	u32 offset_to_frame;
	u32 frame_size;
	u32 encoded_pcm_samples;
	u32 msw_ts;
	u32 lsw_ts;
	u32 nflags;
} __attribute__ ((packed));

struct meta_out{
	unsigned char num_of_frames;
	struct meta_out_dsp meta_out_dsp[];
} __attribute__ ((packed));

#define PCM_BUF_COUNT		(2)
/* Buffer with meta*/
#define PCM_BUF_SIZE		(16384 + sizeof(struct meta_in))

/* FRAME_NUM must be a power of two */
#define FRAME_NUM		(8)
/* Maximum 10 frames in buffer with meta */
#define FRAME_SIZE		(1 + ((35+sizeof(struct meta_out_dsp)) * 10))

struct q6audio_qcelp{
	spinlock_t			dsp_lock;
	atomic_t			in_bytes;
	atomic_t			in_samples;

	struct mutex			lock;
	struct mutex			read_lock;
	struct mutex			write_lock;
	wait_queue_head_t		read_wait;
	wait_queue_head_t		write_wait;
	wait_queue_head_t		cmd_wait;

	struct audio_client             *ac;
	struct msm_audio_stream_config  str_cfg;
	struct msm_audio_qcelp_enc_config enc_cfg;
	struct msm_audio_buf_cfg        buf_cfg;
	struct msm_audio_config     pcm_cfg;

	/* number of buffers available to read/write */
	atomic_t			in_count;
	atomic_t			out_count;

	/* first idx: num of frames per buf, second idx: offset to frame */
	uint32_t			out_frame_info[FRAME_NUM][2];
	int				eos_ack;
	int				eos_rsp;
	int				opened;
	int				enabled;
	int				stopped;
	int				pcm_feedback;
	int				rflush;
	int				wflush;
	int				buf_alloc;
};

static void  qcelp_in_get_dsp_frames(struct q6audio_qcelp *audio,
		uint32_t token,	uint32_t *payload);
static int qcelp_in_wait_for_eos_ack(struct q6audio_qcelp *audio);
static int qcelp_in_flush(struct q6audio_qcelp *ac);
static int qcelp_in_pause(struct q6audio_qcelp *ac);

void q6asm_qcelp_in_cb(uint32_t opcode, uint32_t token,
		uint32_t *payload, void *priv)
{
	struct q6audio_qcelp * audio = (struct q6audio_qcelp *)priv;
	unsigned long flags;

	MM_DBG("opcode - %d\n", opcode);

	spin_lock_irqsave(&audio->dsp_lock, flags);
	switch (opcode) {
	case ASM_DATA_EVENT_EOS:
		audio->eos_ack = 1;
		break;
	case ASM_DATA_EVENT_READ_DONE:
		qcelp_in_get_dsp_frames(audio, token, payload);
		break;
	case ASM_DATA_EVENT_WRITE_DONE:
		atomic_inc(&audio->in_count);
		wake_up(&audio->write_wait);
		break;
	case ASM_DATA_CMDRSP_EOS:
		audio->eos_rsp = 1;
		wake_up(&audio->read_wait);
		break;
	case ASM_STREAM_CMDRSP_GET_ENCDEC_PARAM:
		break;
	case ASM_STREAM_CMDRSP_GET_PP_PARAMS:
		break;
	case ASM_SESSION_EVENT_TX_OVERFLOW:
		MM_ERR("ASM_SESSION_EVENT_TX_OVERFLOW\n");
		break;
	default:
		MM_ERR("Ignore opcode[0x%x]\n", opcode);
		break;
	}
	spin_unlock_irqrestore(&audio->dsp_lock, flags);
}

static void  qcelp_in_get_dsp_frames(struct q6audio_qcelp *audio,
	uint32_t token,	uint32_t *payload)
{
	uint32_t index;

	index = token;
	MM_DBG("index=%d nr frames=%d offset[%d]\n", token,
			payload[7], payload[3]);
	MM_DBG("timemsw=%d lsw=%d\n", payload[4], payload[5]);
	MM_DBG("uflags=0x%8x uid=0x%8x\n", payload[6],
			payload[8]);
	MM_DBG("enc frame size=0x%8x\n", payload[2]);

	audio->out_frame_info[index][0] = payload[7];
	audio->out_frame_info[index][1] = payload[3];

	/* statistics of read */
	atomic_add(payload[2], &audio->in_bytes);
	atomic_add(payload[7], &audio->in_samples);

	if (atomic_read(&audio->out_count) <= audio->str_cfg.buffer_count) {
		atomic_inc(&audio->out_count);
		wake_up(&audio->read_wait);
	}
}

/* must be called with audio->lock held */
static int qcelp_in_enable(struct q6audio_qcelp  *audio)
{
	if (audio->enabled)
		return 0;

	/* 2nd arg: 0 -> run immediately
		3rd arg: 0 -> msw_ts, 4th arg: 0 ->lsw_ts */
	return q6asm_run(audio->ac, 0x00, 0x00, 0x00);
}

/* must be called with audio->lock held */
static int qcelp_in_disable(struct q6audio_qcelp  *audio)
{
	int rc = 0;
	if (audio->enabled) {
		audio->enabled = 0;
		MM_INFO("inbytes[%d] insamples[%d]\n",
					atomic_read(&audio->in_bytes),
					atomic_read(&audio->in_samples));

		rc = q6asm_cmd(audio->ac, CMD_CLOSE);
		if (rc < 0)
			MM_ERR("Failed to close the session rc=%d\n", rc);
		audio->stopped = 1;
		memset(audio->out_frame_info, 0,
				sizeof(audio->out_frame_info));
		wake_up(&audio->cmd_wait);
		wake_up(&audio->read_wait);
		wake_up(&audio->write_wait);
	}
	MM_DBG("enabled[%d]\n", audio->enabled);
	return rc;
}

static int qcelp_in_flush(struct q6audio_qcelp  *audio)
{
	int rc;

	MM_DBG("flush\n");
	/* Implicitly issue a pause to the decoder before flushing */
	rc = qcelp_in_pause(audio);
	if (rc < 0) {
		MM_ERR("pause cmd failed rc=%d\n", rc);
		return rc;
	}

	rc = q6asm_cmd(audio->ac, CMD_FLUSH);
	if (rc < 0) {
		MM_ERR("flush cmd failed rc=%d\n", rc);
		return rc;
	}
	audio->rflush = 1;
	audio->wflush = 1;
	memset(audio->out_frame_info, 0, sizeof(audio->out_frame_info));
	wake_up(&audio->read_wait);
	/* get read_lock to ensure no more waiting read thread */
	mutex_lock(&audio->read_lock);
	audio->rflush = 0;
	mutex_unlock(&audio->read_lock);
	wake_up(&audio->write_wait);
	/* get write_lock to ensure no more waiting write thread */
	mutex_lock(&audio->write_lock);
	audio->wflush = 0;
	mutex_unlock(&audio->write_lock);
	MM_DBG("in_bytes %d\n", atomic_read(&audio->in_bytes));
	MM_DBG("in_samples %d\n", atomic_read(&audio->in_samples));
	atomic_set(&audio->in_bytes, 0);
	atomic_set(&audio->in_samples, 0);
	return 0;
}

static int qcelp_in_pause(struct q6audio_qcelp  *audio)
{
	int rc;

	rc = q6asm_cmd(audio->ac, CMD_PAUSE);
	if (rc < 0)
		MM_ERR("pause cmd failed rc=%d\n", rc);

	if (qcelp_in_wait_for_eos_ack(audio) < 0) {
		MM_ERR("Wait for eos ack failed rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static int qcelp_in_wait_for_eos_ack(struct q6audio_qcelp *audio)
{
	int rc;
	rc = wait_event_interruptible_timeout(audio->cmd_wait,
				/*wait for eos ack for pause */
				(audio->eos_ack == 1 || audio->stopped), 1*HZ);
	if (rc < 0)
		MM_ERR("Wait for eos ack timedout,rc=%d\n", rc);
	audio->eos_ack = 0;

	return rc;
}

static int qcelp_buf_alloc(struct q6audio_qcelp *audio)
{
	int rc = 0;

	switch (audio->buf_alloc) {
	case NO_BUF_ALLOC:
		if (audio->pcm_feedback == NON_TUNNEL_MODE) {
			rc = q6asm_audio_client_buf_alloc(IN,
					audio->ac,
					audio->pcm_cfg.buffer_size,
					audio->pcm_cfg.buffer_count);
			if (rc < 0) {
				MM_ERR("Buffer Alloc failed\n");
				rc = -ENOMEM;
				break;
			}
			audio->buf_alloc |= BUF_ALLOC_IN;
		}
		rc = q6asm_audio_client_buf_alloc(OUT, audio->ac,
					audio->str_cfg.buffer_size,
					audio->str_cfg.buffer_count);
		if (rc < 0) {
			MM_ERR("Buffer Alloc failed rc=%d\n", rc);
			rc = -ENOMEM;
			break;
		}
		audio->buf_alloc |= BUF_ALLOC_OUT;
		break;
	case BUF_ALLOC_IN:
		rc = q6asm_audio_client_buf_alloc(OUT, audio->ac,
					audio->str_cfg.buffer_size,
					audio->str_cfg.buffer_count);
		if (rc < 0) {
			MM_ERR("Buffer Alloc failed rc=%d\n", rc);
			rc = -ENOMEM;
			break;
		}
		audio->buf_alloc |= BUF_ALLOC_OUT;
		break;
	case BUF_ALLOC_OUT:
		if (audio->pcm_feedback == NON_TUNNEL_MODE) {
			rc = q6asm_audio_client_buf_alloc(IN, audio->ac,
					audio->pcm_cfg.buffer_size,
					audio->pcm_cfg.buffer_count);
			if (rc < 0) {
				MM_ERR("Buffer Alloc failed\n");
				rc = -ENOMEM;
				break;
			}
			audio->buf_alloc |= BUF_ALLOC_IN;
		}
		break;
	default:
		MM_DBG("buf[%d]\n", audio->buf_alloc);
	}

	return rc;
}
/* ------------------- device --------------------- */
static long qcelp_in_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct q6audio_qcelp  *audio = file->private_data;
	int rc = 0;
	int cnt = 0;

	if (cmd == AUDIO_GET_STATS) {
		struct msm_audio_stats stats;
		stats.byte_count = atomic_read(&audio->in_bytes);
		stats.sample_count = atomic_read(&audio->in_samples);
		if (copy_to_user((void *) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return rc;
	}

	mutex_lock(&audio->lock);
	switch (cmd) {
	case AUDIO_START: {
		MM_DBG("default buf alloc[%d]\n", audio->buf_alloc);
		rc = qcelp_buf_alloc(audio);
		if (rc < 0) {
			MM_ERR("buffer allocation failed\n");
			break;
		}

		/* reduced_rate_level, rate_modulation_cmd set to zero
			 currently not configurable from user space */
		rc = q6asm_enc_cfg_blk_qcelp(audio->ac,
			audio->buf_cfg.frames_per_buf,
			audio->enc_cfg.min_bit_rate,
			audio->enc_cfg.max_bit_rate, 0, 0);

		if (rc < 0) {
			MM_ERR("cmd qcelp media format block failed\n");
			break;
		}
		if (audio->pcm_feedback == NON_TUNNEL_MODE) {
			rc = q6asm_media_format_block_pcm(audio->ac,
				audio->pcm_cfg.sample_rate,
				audio->pcm_cfg.channel_count);

			if (rc < 0) {
				MM_ERR("media format block failed\n");
				break;
			}
		}
		MM_DBG("AUDIO_START enable[%d]\n", audio->enabled);
		rc = qcelp_in_enable(audio);
		if (!rc) {
			audio->enabled = 1;
		} else {
			audio->enabled = 0;
			MM_ERR("Audio Start procedure failed rc=%d\n", rc);
			break;
		}
		while (cnt++ < audio->str_cfg.buffer_count)
			q6asm_read(audio->ac); /* Push buffer to DSP */
		rc = 0;
		MM_INFO("AUDIO_START success enable[%d]\n", audio->enabled);
		break;
	}
	case AUDIO_STOP: {
		MM_INFO("AUDIO_STOP\n");
		rc = qcelp_in_disable(audio);
		if (rc  < 0) {
			MM_ERR("Audio Stop procedure failed rc=%d\n", rc);
			break;
		}
		break;
	}
	case AUDIO_FLUSH: {
		/* Make sure we're stopped and we wake any threads
		* that might be blocked holding the read_lock.
		* While audio->stopped read threads will always
		* exit immediately.
		*/
		wake_up(&audio->cmd_wait);
		rc = qcelp_in_flush(audio);
		if (rc < 0)
			MM_ERR("Flush Fail rc=%d\n", rc);
		break;
	}
	case AUDIO_PAUSE: {
		MM_DBG("AUDIO_PAUSE\n");
		if (audio->enabled)
			qcelp_in_pause(audio);
		break;
	}
	case AUDIO_GET_STREAM_CONFIG: {
		struct msm_audio_stream_config cfg;
		memset(&cfg, 0, sizeof(cfg));
		cfg.buffer_size = audio->str_cfg.buffer_size;
		cfg.buffer_count = audio->str_cfg.buffer_count;
		if (copy_to_user((void *)arg, &cfg, sizeof(cfg)))
			rc = -EFAULT;
		MM_DBG("AUDIO_GET_STREAM_CONFIG %d %d\n", cfg.buffer_size,
			cfg.buffer_count);
		break;
	}
	case AUDIO_SET_STREAM_CONFIG: {
		struct msm_audio_stream_config cfg;
		if (copy_from_user(&cfg, (void *)arg, sizeof(cfg))) {
			rc = -EFAULT;
			break;
		}
		/* Allow minimum single frame, but with in maximum frames */
		if ((cfg.buffer_size < (35+sizeof(struct meta_out_dsp))) ||
			(cfg.buffer_count < FRAME_NUM)) {
			rc = -EINVAL;
			break;
		}
		audio->str_cfg.buffer_size = cfg.buffer_size;
		audio->str_cfg.buffer_count = cfg.buffer_count;
		rc = q6asm_audio_client_buf_alloc(OUT, audio->ac,
				audio->str_cfg.buffer_size,
				audio->str_cfg.buffer_count);
		if (rc < 0) {
			MM_ERR("Buffer Alloc failed rc=%d\n", rc);
			rc = -ENOMEM;
			break;
		}
		audio->buf_alloc |= BUF_ALLOC_OUT;
		rc = 0;
		MM_DBG("AUDIO_SET_STREAM_CONFIG %d %d\n",
			audio->str_cfg.buffer_size,
			audio->str_cfg.buffer_count);
		break;
	}
	case AUDIO_GET_QCELP_ENC_CONFIG: {
		if (copy_to_user((void *)arg, &audio->enc_cfg,
			sizeof(struct msm_audio_qcelp_enc_config)))
			rc = -EFAULT;
		break;
	}
	case AUDIO_SET_QCELP_ENC_CONFIG: {
		struct msm_audio_qcelp_enc_config cfg;

		if (copy_from_user(&cfg, (void *) arg,
				sizeof(struct msm_audio_qcelp_enc_config))) {
			rc = -EFAULT;
			break;
		}

		if (cfg.min_bit_rate > 4 ||
			 cfg.min_bit_rate < 1) {
			MM_ERR("invalid min bitrate\n");
			rc = -EINVAL;
			break;
		}
		if (cfg.max_bit_rate > 4 ||
			 cfg.max_bit_rate < 1) {
			MM_ERR("invalid max bitrate\n");
			rc = -EINVAL;
			break;
		}
		audio->enc_cfg.min_bit_rate = cfg.min_bit_rate;
		audio->enc_cfg.max_bit_rate = cfg.max_bit_rate;
		MM_DBG("min_bit_rate= 0x%x max_bit_rate=0x%x\n",\
						audio->enc_cfg.min_bit_rate,
						audio->enc_cfg.max_bit_rate);
		break;
	}
	case AUDIO_GET_SESSION_ID: {
		if (copy_to_user((void *) arg, &audio->ac->session,
			sizeof(unsigned short))) {
			rc = -EFAULT;
		}
		break;
	}
	case AUDIO_SET_BUF_CFG: {
		struct msm_audio_buf_cfg  cfg;
		if (copy_from_user(&cfg, (void *)arg, sizeof(cfg))) {
			rc = -EFAULT;
			break;
		}
		if ((audio->pcm_feedback == NON_TUNNEL_MODE) &&
			!cfg.meta_info_enable) {
			rc = -EFAULT;
			break;
		}

		/* Restrict the num of frames per buf to 10 to coincide with
		 * default buf size */
		if (cfg.frames_per_buf > 10) {
			rc = -EFAULT;
			break;
		}
		audio->buf_cfg.meta_info_enable = cfg.meta_info_enable;
		audio->buf_cfg.frames_per_buf = cfg.frames_per_buf;
		MM_DBG("Set-buf-cfg: meta[%d] framesperbuf[%d]\n",
					cfg.meta_info_enable,
					cfg.frames_per_buf);
		break;
	}
	case AUDIO_GET_BUF_CFG: {
		MM_DBG("Get-buf-cfg: meta[%d] framesperbuf[%d]\n",
				audio->buf_cfg.meta_info_enable,
				audio->buf_cfg.frames_per_buf);

		if (copy_to_user((void *)arg, &audio->buf_cfg,
					sizeof(struct msm_audio_buf_cfg)))
			rc = -EFAULT;
		break;
	}
	case AUDIO_GET_CONFIG: {
		if (copy_to_user((void *)arg, &audio->pcm_cfg,
					sizeof(struct msm_audio_config)))
			rc = -EFAULT;
		break;

	}
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config cfg;
		if (copy_from_user(&cfg, (void *)arg, sizeof(cfg))) {
			rc = -EFAULT;
			break;
		}
		if (audio->pcm_feedback != NON_TUNNEL_MODE) {
			MM_ERR("Not sufficient permission to"
					"change the playback mode\n");
			rc = -EACCES;
			break;
		}
		if ((cfg.buffer_count > PCM_BUF_COUNT) ||
				(cfg.buffer_count == 1))
			cfg.buffer_count = PCM_BUF_COUNT;

		audio->pcm_cfg.buffer_count = cfg.buffer_count;
		audio->pcm_cfg.buffer_size  = cfg.buffer_size;
		audio->pcm_cfg.channel_count = cfg.channel_count;
		audio->pcm_cfg.sample_rate = cfg.sample_rate;
		rc = q6asm_audio_client_buf_alloc(IN, audio->ac,
			audio->pcm_cfg.buffer_size,
			audio->pcm_cfg.buffer_count);
		if (rc < 0) {
			MM_ERR("Buffer Alloc failed\n");
			rc = -ENOMEM;
			break;
		}
		audio->buf_alloc |= BUF_ALLOC_IN;
		rc = 0;
		MM_DBG("AUDIO_SET_CONFIG %d %d\n",
			audio->pcm_cfg.buffer_count,
			audio->pcm_cfg.buffer_size);
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&audio->lock);
	return rc;
}

static ssize_t qcelp_in_read(struct file *file,
				char __user *buf,
				size_t count, loff_t *pos)
{
	struct q6audio_qcelp  *audio = file->private_data;
	const char __user *start = buf;
	unsigned char *data;
	uint32_t offset = 0;
	uint32_t size = 0;
	int rc = 0;
	uint32_t idx;
	struct meta_out_dsp meta;
	uint32_t bytes_to_copy = 0;
	uint32_t mfield_size = (audio->buf_cfg.meta_info_enable == 0) ? 0 :
		(sizeof(unsigned char) +
		(sizeof(struct meta_out_dsp)*(audio->buf_cfg.frames_per_buf)));

	MM_DBG("read - %d\n", count);
	mutex_lock(&audio->read_lock);
	while (count > 0) {
		rc = wait_event_interruptible(
			audio->read_wait,
			((atomic_read(&audio->out_count) > 0) ||
			(audio->stopped) ||
			 audio->rflush || audio->eos_rsp));

		if (rc < 0)
			break;

		if ((audio->stopped && !(atomic_read(&audio->out_count))) ||
			audio->rflush) {
			MM_DBG("driver in stop state or flush,\
					No more buf to read");
			rc = 0;/* End of File */
			break;
		}
		if (!(atomic_read(&audio->out_count)) &&
			(audio->eos_rsp == 1) &&
			(count >= (sizeof(unsigned char) +
				sizeof(struct meta_out_dsp)))) {
			unsigned char num_of_frames;
			MM_INFO("eos %d at output\n", audio->eos_rsp);
			if (buf != start)
				break;
			num_of_frames = 0xFF;
			if (copy_to_user(buf, &num_of_frames,
					sizeof(unsigned char))) {
				rc = -EFAULT;
				break;
			}
			buf += sizeof(unsigned char);
			meta.frame_size = 0xFFFF;
			meta.encoded_pcm_samples = 0xFFFF;
			meta.msw_ts = 0x00;
			meta.lsw_ts = 0x00;
			meta.nflags = AUDQCELP_EOS_SET;
			audio->eos_rsp = 0;
			if (copy_to_user(buf, &meta, sizeof(meta))) {
				rc = -EFAULT;
				break;
			}
			buf += sizeof(meta);
			break;
		}
		data = (unsigned char *)q6asm_is_cpu_buf_avail(OUT, audio->ac,
						&size, &idx);
		if ((count >= (size + mfield_size)) && data) {
			if (audio->buf_cfg.meta_info_enable) {
				if (copy_to_user(buf,
					&audio->out_frame_info[idx][0],
					sizeof(unsigned char))) {
					rc = -EFAULT;
					break;
				}
				bytes_to_copy =
					(size + audio->out_frame_info[idx][1]);
				/* Number of frames information copied */
				buf += sizeof(unsigned char);
				count -= sizeof(unsigned char);
			} else {
				offset = audio->out_frame_info[idx][1];
				bytes_to_copy = size;
			}

			MM_DBG("offset=%d nr of frames= %d\n",
					audio->out_frame_info[idx][1],
					audio->out_frame_info[idx][0]);

			if (copy_to_user(buf, &data[offset], bytes_to_copy)) {
				rc = -EFAULT;
				break;
			}
			atomic_dec(&audio->out_count);
			q6asm_read(audio->ac);
			count -= bytes_to_copy;
			buf += bytes_to_copy;
		} else {
			MM_ERR("short read data[%p] size[%d]\n", data, size);
			break;
		}
		break;
	}
	mutex_unlock(&audio->read_lock);

	MM_DBG("read: %d bytes\n", (buf-start));
	if (buf > start)
		return buf - start;
	return rc;
}

static int extract_meta_info(char *buf, unsigned long *msw_ts,
		unsigned long *lsw_ts, unsigned int *flags)
{
	struct meta_in *meta = (struct meta_in *)buf;
	*msw_ts = meta->ntimestamp.lowpart;
	*lsw_ts = meta->ntimestamp.highpart;
	*flags = meta->nflags;
	return 0;
}

static ssize_t qcelp_in_write(struct file *file,
				const char __user *buf,
				size_t count, loff_t *pos)
{
	struct q6audio_qcelp *audio = file->private_data;
	const char __user *start = buf;
	size_t xfer = 0;
	char *cpy_ptr;
	int rc = 0;
	unsigned char *data;
	uint32_t size = 0;
	uint32_t idx = 0;
	uint32_t nflags = 0;
	unsigned long msw_ts = 0;
	unsigned long lsw_ts = 0;
	uint32_t mfield_size = (audio->buf_cfg.meta_info_enable == 0) ? 0 :
			sizeof(struct meta_in);

	MM_DBG("to write[%d]\n", count);
	mutex_lock(&audio->write_lock);

	while (count > 0) {
		rc = wait_event_interruptible(audio->write_wait,
				     ((atomic_read(&audio->in_count) > 0) ||
				      (audio->stopped) ||
				      (audio->wflush)));
		if (rc < 0)
			break;
		if (audio->stopped || audio->wflush) {
			MM_DBG("stop or flush\n");
			rc = -EBUSY;
			break;
		}
		data = (unsigned char *)q6asm_is_cpu_buf_avail(IN, audio->ac,
						&size, &idx);
		if (!data) {
			MM_DBG("No buf available\n");
			continue;
		}
		cpy_ptr = data;
		if (audio->buf_cfg.meta_info_enable) {
			if (buf == start) {
				/* Processing beginning of user buffer */
				if (copy_from_user(cpy_ptr, buf, mfield_size)) {
					rc = -EFAULT;
					break;
				}
				/* Check if EOS flag is set and buffer has
				* contains just meta field
				*/
				extract_meta_info(cpy_ptr, &msw_ts, &lsw_ts,
						&nflags);
				buf += mfield_size;
				if (count == mfield_size) {
					/* send the EOS and return */
					MM_DBG("send EOS 0x%8x\n", nflags);
					break;
				}
				count -= mfield_size;
			} else {
				MM_DBG("continuous buffer\n");
			}
		}
		xfer = (count > (audio->pcm_cfg.buffer_size)) ?
				(audio->pcm_cfg.buffer_size) : count;

		if (copy_from_user(cpy_ptr, buf, xfer)) {
			rc = -EFAULT;
			break;
		}
		rc = q6asm_write(audio->ac, xfer, msw_ts, lsw_ts, 0x00);
		if (rc < 0) {
			rc = -EFAULT;
			break;
		}
		atomic_dec(&audio->in_count);
		count -= xfer;
		buf += xfer;
	}
	mutex_unlock(&audio->write_lock);
	MM_DBG("eos_condition 0x%8x buf[0x%x] start[0x%x]\n", nflags,
			(int) buf, (int) start);
	if (nflags & AUDQCELP_EOS_SET) {
		rc = q6asm_cmd(audio->ac, CMD_EOS);
		MM_INFO("eos %d at input\n", audio->eos_rsp);
	}
	MM_DBG("Written %d Avail Buf[%d]",
					(buf - start - mfield_size),
					atomic_read(&audio->in_count));
	if (!rc) {
		if (buf > start)
			return buf - start;
	}
	return rc;
}

static int qcelp_in_release(struct inode *inode, struct file *file)
{
	struct q6audio_qcelp  *audio = file->private_data;
	MM_INFO("\n");
	mutex_lock(&audio->lock);
	if (audio->enabled)
		qcelp_in_disable(audio);
	q6asm_audio_client_free(audio->ac);
	mutex_unlock(&audio->lock);
	kfree(audio);
	MM_DBG("COMPLETED\n");
	return 0;
}

static int qcelp_in_open(struct inode *inode, struct file *file)
{
	struct q6audio_qcelp *audio = NULL;
	int rc = 0;

	audio = kzalloc(sizeof(struct q6audio_qcelp), GFP_KERNEL);

	if (audio == NULL) {
		MM_ERR("Could not allocate memory for qcelp driver\n");
		return -ENOMEM;
	}

	mutex_init(&audio->lock);
	mutex_init(&audio->read_lock);
	mutex_init(&audio->write_lock);
	spin_lock_init(&audio->dsp_lock);
	init_waitqueue_head(&audio->cmd_wait);
	init_waitqueue_head(&audio->read_wait);
	init_waitqueue_head(&audio->write_wait);

	/* Settings will be re-config at AUDIO_SET_CONFIG,
	* but at least we need to have initial config
	*/
	audio->str_cfg.buffer_size = FRAME_SIZE;
	audio->str_cfg.buffer_count = FRAME_NUM;
	audio->pcm_cfg.buffer_size = PCM_BUF_SIZE;
	audio->pcm_cfg.buffer_count = PCM_BUF_COUNT;
	audio->enc_cfg.min_bit_rate = 4;
	audio->enc_cfg.max_bit_rate = 4;
	audio->pcm_cfg.channel_count = 1;
	audio->pcm_cfg.sample_rate = 8000;
	audio->buf_cfg.meta_info_enable = 0x01;
	audio->buf_cfg.frames_per_buf = 0x01;

	audio->ac = q6asm_audio_client_alloc((app_cb)q6asm_qcelp_in_cb,
				(void *)audio);

	if (!audio->ac) {
		MM_ERR("Could not allocate memory for audio client\n");
		kfree(audio);
		return -ENOMEM;
	}

	/* open qcelp encoder in T/NT mode */
	if ((file->f_mode & FMODE_WRITE) &&
		(file->f_mode & FMODE_READ)) {
		audio->pcm_feedback = NON_TUNNEL_MODE;
		rc = q6asm_open_read_write(audio->ac, FORMAT_V13K,
					FORMAT_LINEAR_PCM);
		if (rc < 0) {
			MM_ERR("NT mode Open failed rc=%d\n", rc);
			rc = -ENODEV;
			goto fail;
		}
	} else if (!(file->f_mode & FMODE_WRITE) &&
				(file->f_mode & FMODE_READ)) {
		audio->pcm_feedback = TUNNEL_MODE;
		rc = q6asm_open_read(audio->ac, FORMAT_V13K);
		if (rc < 0) {
			MM_ERR("T mode Open failed rc=%d\n", rc);
			rc = -ENODEV;
			goto fail;
		}
		/* register for tx overflow (valid for tunnel mode only) */
		rc = q6asm_reg_tx_overflow(audio->ac, 0x01);
		if (rc < 0) {
			MM_ERR("TX Overflow registration failed rc=%d\n", rc);
			rc = -ENODEV;
			goto fail;
		}
	} else {
		rc = -EACCES;
		goto fail;
	}

	audio->opened = 1;
	atomic_set(&audio->in_count, PCM_BUF_COUNT);
	atomic_set(&audio->out_count, 0x00);
	file->private_data = audio;
	MM_INFO("success\n");
	return 0;
fail:
	q6asm_audio_client_free(audio->ac);
	kfree(audio);
	return rc;
}

static const struct file_operations audio_in_fops = {
	.owner		= THIS_MODULE,
	.open		= qcelp_in_open,
	.release	= qcelp_in_release,
	.read		= qcelp_in_read,
	.write		= qcelp_in_write,
	.unlocked_ioctl	= qcelp_in_ioctl,
};

struct miscdevice audio_qcelp_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_qcelp_in",
	.fops	= &audio_in_fops,
};

static int __init qcelp_in_init(void)
{
	return misc_register(&audio_qcelp_in_misc);
}

device_initcall(qcelp_in_init);
