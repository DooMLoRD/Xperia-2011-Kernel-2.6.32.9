/*  Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/jiffies.h>

#include "apr_audio.h"

struct afe_ctl {
	atomic_t ref_cnt;
	void *apr;
	atomic_t state;
	wait_queue_head_t wait;
};

static struct afe_ctl this_afe;

#define TIMEOUT_MS 1000

static int32_t afe_callback(struct apr_client_data *data, void *priv)
{
	if (data->payload_size) {
		uint32_t *payload;
		payload = data->payload;
		pr_info("%s: cmd = 0x%x status = 0x%x\n", __func__,
					payload[0], payload[1]);
		if (data->opcode == APR_BASIC_RSP_RESULT) {
			switch (payload[0]) {
			case AFE_PORT_AUDIO_IF_CONFIG:
			case AFE_PORT_CMD_STOP:
			case AFE_PORT_CMD_START:
				atomic_set(&this_afe.state, 0);
				wake_up(&this_afe.wait);
				break;
			default:
				pr_err("Unknown cmd 0x%x\n",
						payload[0]);
				break;
			}
		}
	}
	return 0;
}


int afe_open_pcmif(struct afe_port_pcm_cfg cfg)
{
	struct afe_port_start_command start;
	struct afe_audioif_config_command config;
	int ret;

	if (atomic_read(&this_afe.ref_cnt) == 0) {
		this_afe.apr = apr_register("ADSP", "AFE", afe_callback,
					0xFFFFFFFF, &this_afe);
		pr_info("%s: Register AFE\n", __func__);
		if (this_afe.apr == NULL) {
			pr_err("%s: Unable to register AFE\n", __func__);
			ret = -ENODEV;
			goto fail_cmd;
		}
	}
	config.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	config.hdr.pkt_size = sizeof(config);
	config.hdr.src_port = 0;
	config.hdr.dest_port = 0;
	config.hdr.token = 0;
	config.hdr.opcode = AFE_PORT_AUDIO_IF_CONFIG;
	config.port.pcm = cfg;


	atomic_set(&this_afe.state, 1);
	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &config);
	if (ret < 0) {
		pr_err("%s: AFE enable for port %d failed\n", __func__,
				cfg.port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail_cmd;
	}

	start.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	start.hdr.pkt_size = sizeof(start);
	start.hdr.src_port = 0;
	start.hdr.dest_port = 0;
	start.hdr.token = 0;
	start.hdr.opcode = AFE_PORT_CMD_START;
	start.port_id = cfg.port_id;
	start.gain = 0x4000;
	start.sample_rate = 8000;

	atomic_set(&this_afe.state, 1);

	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &start);
	if (ret < 0) {
		pr_err("%s: AFE enable for port %d failed\n", __func__,
				cfg.port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	atomic_inc(&this_afe.ref_cnt);
	return 0;
fail_cmd:
	if (atomic_read(&this_afe.ref_cnt) == 0)
		apr_deregister(this_afe.apr);
	return ret;
}


int afe_open(int port_id, int rate, int channel_mode)
{
	struct afe_port_start_command start;
	struct afe_audioif_config_command config;
	int ret = 0;

	pr_info("%s: %d %d %d\n", __func__, port_id, rate, channel_mode);

	if (atomic_read(&this_afe.ref_cnt) == 0) {
		this_afe.apr = apr_register("ADSP", "AFE", afe_callback,
					0xFFFFFFFF, &this_afe);
		pr_info("%s: Register AFE\n", __func__);
		if (this_afe.apr == NULL) {
			pr_err("%s: Unable to register AFE\n", __func__);
			ret = -ENODEV;
			return ret;
		}
	}

	config.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	config.hdr.pkt_size = sizeof(config);
	config.hdr.src_port = 0;
	config.hdr.dest_port = 0;
	config.hdr.token = 0;
	config.hdr.opcode = AFE_PORT_AUDIO_IF_CONFIG;

	if ((port_id == PRIMARY_I2S_RX) || (port_id == PRIMARY_I2S_TX)) {

		pr_info("%s: I2S %d %d %d\n", __func__, port_id, rate,
				channel_mode);

		config.port.mi2s.port_id = port_id;
		config.port.mi2s.bitwidth = 16;
		config.port.mi2s.line = 1;
		config.port.mi2s.channel = channel_mode;
		config.port.mi2s.ws = 1;
	} else if (port_id == HDMI_RX) {

		pr_info("%s: HDMI %d %d %d\n", __func__, port_id, rate,
				channel_mode);
		config.port.hdmi.port_id = port_id;
		config.port.hdmi.bitwidth = 16;
		config.port.hdmi.channel_mode = channel_mode;
		config.port.hdmi.data_type = 0;
	} else {
		pr_err("%s: Failed : Invalid Port id = %d\n", __func__,
				port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	atomic_set(&this_afe.state, 1);

	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &config);
	if (ret < 0) {
		pr_err("%s: AFE enable for port %d failed\n", __func__,
				port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	start.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	start.hdr.pkt_size = sizeof(start);
	start.hdr.src_port = 0;
	start.hdr.dest_port = 0;
	start.hdr.token = 0;
	start.hdr.opcode = AFE_PORT_CMD_START;
	start.port_id = port_id;
	start.gain = 0x4000;
	start.sample_rate = rate;

	atomic_set(&this_afe.state, 1);
	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &start);
	if (ret < 0) {
		pr_err("%s: AFE enable for port %d failed\n", __func__,
				port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	atomic_inc(&this_afe.ref_cnt);
	return 0;
fail_cmd:
	if (atomic_read(&this_afe.ref_cnt) == 0)
		apr_deregister(this_afe.apr);
	return ret;
}

int afe_close(int port_id)
{
	struct afe_port_stop_command stop;
	int ret = 0;

	if (atomic_read(&this_afe.ref_cnt) <= 0) {
		pr_err("AFE is already closed\n");
		ret = -EINVAL;
		goto fail_cmd;
	}

	stop.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	stop.hdr.pkt_size = sizeof(stop);
	stop.hdr.src_port = 0;
	stop.hdr.dest_port = 0;
	stop.hdr.token = 0;
	stop.hdr.opcode = AFE_PORT_CMD_STOP;
	stop.port_id = port_id;
	stop.reserved = 0;

	atomic_set(&this_afe.state, 1);
	ret = apr_send_pkt(this_afe.apr, (uint32_t *) &stop);

	if (ret < 0) {
		pr_err("AFE close failed\n");
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_afe.wait,
			(atomic_read(&this_afe.state) == 0),
					msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	atomic_dec(&this_afe.ref_cnt);
	if (atomic_read(&this_afe.ref_cnt) == 0) {
		pr_debug("%s: Deregister AFE\n", __func__);
		apr_deregister(this_afe.apr);
	}
fail_cmd:
	return ret;
}

static int __init afe_init(void)
{
	pr_info("%s:\n", __func__);
	init_waitqueue_head(&this_afe.wait);
	atomic_set(&this_afe.state, 0);
	atomic_set(&this_afe.ref_cnt, 0);
	return 0;
}

device_initcall(afe_init);
