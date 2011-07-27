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
 */

#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <asm/atomic.h>
#include "apr_audio.h"


#define TIMEOUT_MS 1000
#define AUDIO_RX 0x0
#define AUDIO_TX 0x1

struct adm_ctl {
	atomic_t ref_cnt;
	void *apr;
	atomic_t copp_id[AFE_MAX_PORTS];
	atomic_t copp_cnt[AFE_MAX_PORTS];
	atomic_t copp_state[AFE_MAX_PORTS];
	atomic_t copp_test[AFE_MAX_PORTS];
	wait_queue_head_t wait;
};

static struct adm_ctl this_adm;

static int32_t adm_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *payload;
	payload = data->payload;
	pr_info("%s: code = 0x%x %x %x size = %d\n", __func__,
			data->opcode, payload[0], payload[1],
					data->payload_size);

	if (data->payload_size) {
		if (data->opcode == APR_BASIC_RSP_RESULT) {
			switch (payload[0]) {
			case ADM_CMD_COPP_CLOSE:
			case ADM_CMD_MEMORY_MAP:
			case ADM_CMD_MEMORY_UNMAP:
			case ADM_CMD_MEMORY_MAP_REGIONS:
			case ADM_CMD_MEMORY_UNMAP_REGIONS:
			case ADM_CMD_MATRIX_MAP_ROUTINGS:
				atomic_set(&this_adm.copp_test[data->token],
									1);
				wake_up(&this_adm.wait);
				break;
			default:
				pr_err("Unknown Cmd: 0x%x\n", payload[0]);
				break;
			}
			return 0;
		}

		switch (data->opcode) {
		case ADM_CMDRSP_COPP_OPEN: {
			struct adm_copp_open_respond *open = data->payload;
			atomic_set(&this_adm.copp_id[data->token],
							open->copp_id);
			atomic_set(&this_adm.copp_state[data->token], 1);
			wake_up(&this_adm.wait);
			}
			break;
		default:
			pr_err("Unknown cmd:0x%x\n", data->opcode);
			break;
		}
	}
	return 0;
}

int adm_open(int port_id, int session_id , int path,
				int rate, int channel_mode)
{
	struct adm_copp_open_command open;
	struct adm_routings_command route;
	int ret = 0;

	pr_info("%s: port %d session 0x%x path:%d rate:%d mode:%d\n", __func__,
				port_id, session_id, path, rate, channel_mode);

	if (port_id >= AFE_MAX_PORTS)
		return -ENODEV;

	if (atomic_read(&this_adm.ref_cnt) == 0) {
		this_adm.apr = apr_register("ADSP", "ADM", adm_callback,
						0xFFFFFFFF, &this_adm);
		if (this_adm.apr == NULL) {
			pr_err("Unable to register ADM\n");
			ret = -ENODEV;
			return ret;
		}
	}

	/* Create a COPP if port id are not enabled */
	if (atomic_read(&this_adm.copp_cnt[port_id]) == 0) {

		open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		open.hdr.pkt_size = sizeof(open);
		open.hdr.src_svc = APR_SVC_ADM;
		open.hdr.src_domain = APR_DOMAIN_APPS;
		open.hdr.src_port = port_id;
		open.hdr.dest_svc = APR_SVC_ADM;
		open.hdr.dest_domain = APR_DOMAIN_ADSP;
		open.hdr.dest_port = port_id;
		open.hdr.token = port_id;
		open.hdr.opcode = ADM_CMD_COPP_OPEN;

		open.mode = path;
		open.endpoint_id1 = port_id & 0x00FF;
		open.endpoint_id2 = 0xFFFF;
		open.topology_id = DEFAULT_TOPOLOGY;
		open.channel_config = channel_mode & 0x00FF;
		open.rate  = rate;

		pr_debug("channel_config=%d port_id=%d\n",
				open.channel_config, open.endpoint_id1);

		atomic_set(&this_adm.copp_state[port_id], 0);

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&open);
		if (ret < 0) {
			pr_err("ADM enable for port %d failed\n", port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
		/* Wait for the callback with copp id */
		ret = wait_event_timeout(this_adm.wait,
			atomic_read(&this_adm.copp_state[port_id]),
			msecs_to_jiffies(TIMEOUT_MS));
		if (ret < 0) {
			pr_err("ADM open failed for port %d\n", port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
	}

	atomic_inc(&this_adm.copp_cnt[port_id]);


	if (atomic_read(&this_adm.copp_state[port_id])) {

		route.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		route.hdr.pkt_size = sizeof(route);
		route.hdr.src_svc = 0;
		route.hdr.src_domain = APR_DOMAIN_APPS;
		route.hdr.src_port = atomic_read(&this_adm.copp_id[port_id]);
		route.hdr.dest_svc = APR_SVC_ADM;
		route.hdr.dest_domain = APR_DOMAIN_ADSP;
		route.hdr.dest_port = atomic_read(&this_adm.copp_id[port_id]);
		route.hdr.token = port_id;
		route.hdr.opcode = ADM_CMD_MATRIX_MAP_ROUTINGS;
		route.num_sessions = 1;
		route.sessions[0].id = session_id;
		route.sessions[0].num_copps = 1;
		route.sessions[0].copp_id[0] =
				atomic_read(&this_adm.copp_id[port_id]);

		switch (path) {
		case 0x1:
			route.path = AUDIO_RX;
			break;
		case 0x2:
		case 0x3:
			route.path = AUDIO_TX;
			break;
		default:
				pr_err("Wrong path set\n");
			break;
		}

		atomic_set(&this_adm.copp_test[port_id], 0);
		atomic_set(&this_adm.copp_state[port_id], 0);

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&route);
		if (ret < 0) {
			pr_err("ADM routing for port %d failed\n", port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}

		ret = wait_event_timeout(this_adm.wait,
				atomic_read(&this_adm.copp_test[port_id]),
				msecs_to_jiffies(TIMEOUT_MS));
		if (ret < 0) {
			pr_err("ADM cmd Route failed for port %d\n", port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
	}

	atomic_inc(&this_adm.ref_cnt);
	return 0;

fail_cmd:
	if (atomic_read(&this_adm.ref_cnt) == 0)
		apr_deregister(this_adm.apr);
	return ret;
}

int adm_memory_map_regions(uint32_t *buf_add, uint32_t mempool_id,
				uint32_t *bufsz, uint32_t bufcnt)
{
	struct  adm_cmd_memory_map_regions *mmap_regions = NULL;
	struct  adm_memory_map_regions *mregions = NULL;
	void    *mmap_region_cmd = NULL;
	void    *payload = NULL;
	int     ret = 0;
	int     i = 0;
	int     cmd_size = 0;

	pr_info("%s\n", __func__);
	if (this_adm.apr == NULL) {
		pr_err("APR handle NULL\n");
		return -EINVAL;
	}

	cmd_size = sizeof(struct adm_cmd_memory_map_regions)
			+ sizeof(struct adm_memory_map_regions) * bufcnt;

	mmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	mmap_regions = (struct adm_cmd_memory_map_regions *)mmap_region_cmd;
	mmap_regions->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
								APR_PKT_VER);
	mmap_regions->hdr.pkt_size = cmd_size;
	mmap_regions->hdr.src_port = 0;
	mmap_regions->hdr.dest_port = 0;
	mmap_regions->hdr.token = 0;
	mmap_regions->hdr.opcode = ADM_CMD_MEMORY_MAP_REGIONS;
	mmap_regions->mempool_id = mempool_id & 0x00ff;
	mmap_regions->nregions = bufcnt & 0x00ff;
	pr_debug("map_regions->nregions = %d\n",
				mmap_regions->nregions);
	payload = ((u8 *) mmap_region_cmd +
				sizeof(struct adm_cmd_memory_map_regions));
	mregions = (struct adm_memory_map_regions *)payload;

	for (i = 0; i < bufcnt; i++) {
		mregions->phys = buf_add[i];
		mregions->buf_size = bufsz[i];
		++mregions;
	}

	atomic_set(&this_adm.copp_test[0], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *) mmap_region_cmd);
	if (ret < 0) {
		pr_err("mmap_regions op[0x%x]rc[%d]\n",
				mmap_regions->hdr.opcode, ret);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_adm.wait,
			atomic_read(&this_adm.copp_test[0]), 5 * HZ);
	if (ret < 0) {
		pr_err("timeout. waited for memory_map\n");
		ret = -EINVAL;
		goto fail_cmd;
	}
fail_cmd:
	kfree(mmap_region_cmd);
	return ret;
}

int adm_memory_unmap_regions(uint32_t *buf_add, uint32_t *bufsz,
						uint32_t bufcnt)
{
	struct  adm_cmd_memory_unmap_regions *unmap_regions = NULL;
	struct  adm_memory_unmap_regions *mregions = NULL;
	void    *unmap_region_cmd = NULL;
	void    *payload = NULL;
	int     ret = 0;
	int     i = 0;
	int     cmd_size = 0;

	pr_info("%s\n", __func__);

	if (this_adm.apr == NULL) {
		pr_err("APR handle NULL\n");
		return -EINVAL;
	}

	cmd_size = sizeof(struct adm_cmd_memory_unmap_regions)
			+ sizeof(struct adm_memory_unmap_regions) * bufcnt;

	unmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	unmap_regions = (struct adm_cmd_memory_unmap_regions *)
						unmap_region_cmd;
	unmap_regions->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
							APR_PKT_VER);
	unmap_regions->hdr.pkt_size = cmd_size;
	unmap_regions->hdr.src_port = 0;
	unmap_regions->hdr.dest_port = 0;
	unmap_regions->hdr.token = 0;
	unmap_regions->hdr.opcode = ADM_CMD_MEMORY_UNMAP_REGIONS;
	unmap_regions->nregions = bufcnt & 0x00ff;
	unmap_regions->reserved = 0;
	pr_debug("unmap_regions->nregions = %d\n",
				unmap_regions->nregions);
	payload = ((u8 *) unmap_region_cmd +
			sizeof(struct adm_cmd_memory_unmap_regions));
	mregions = (struct adm_memory_unmap_regions *)payload;

	for (i = 0; i < bufcnt; i++) {
		mregions->phys = buf_add[i];
		++mregions;
	}
	atomic_set(&this_adm.copp_test[0], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *) unmap_region_cmd);
	if (ret < 0) {
		pr_err("mmap_regions op[0x%x]rc[%d]\n",
					unmap_regions->hdr.opcode, ret);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_adm.wait,
			atomic_read(&this_adm.copp_test[0]), 5 * HZ);
	if (ret < 0) {
		pr_err("timeout. waited for memory_unmap\n");
		ret = -EINVAL;
		goto fail_cmd;
	}
fail_cmd:
	kfree(unmap_region_cmd);
	return ret;
}

int adm_close(int port_id)
{
	struct apr_hdr close;

	int ret = 0;

	pr_info("%s\n", __func__);

	if (atomic_read(&this_adm.ref_cnt) <= 0) {
		pr_err("ADM is already closed\n");
		ret = -EINVAL;
		return ret;
	}

	atomic_dec(&this_adm.copp_cnt[port_id]);
	if (!(atomic_read(&this_adm.copp_cnt[port_id]))) {

		close.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		close.pkt_size = sizeof(close);
		close.src_svc = APR_SVC_ADM;
		close.src_domain = APR_DOMAIN_APPS;
		close.src_port = port_id;
		close.dest_svc = APR_SVC_ADM;
		close.dest_domain = APR_DOMAIN_ADSP;
		close.dest_port = atomic_read(&this_adm.copp_id[port_id]);
		close.token = port_id;
		close.opcode = ADM_CMD_COPP_CLOSE;

		atomic_set(&this_adm.copp_id[port_id], 0);
		atomic_set(&this_adm.copp_state[port_id], 0);
		atomic_set(&this_adm.copp_test[port_id], 0);

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&close);
		if (ret < 0) {
			pr_err("ADM close failed\n");
			ret = -EINVAL;
			goto fail_cmd;
		}

		ret = wait_event_timeout(this_adm.wait,
				atomic_read(&this_adm.copp_test[port_id]),
				msecs_to_jiffies(TIMEOUT_MS));
		if (ret < 0) {
			pr_info("%s: ADM cmd Route failed for port %d\n",
							__func__, port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
	}

fail_cmd:
	atomic_dec(&this_adm.ref_cnt);
	if (atomic_read(&this_adm.ref_cnt) == 0)
		apr_deregister(this_adm.apr);

	return ret;
}

static int __init adm_init(void)
{
	int i = 0;
	pr_info("%s\n", __func__);
	init_waitqueue_head(&this_adm.wait);
	atomic_set(&this_adm.ref_cnt, 0);

	for (i = 0; i < AFE_MAX_PORTS; i++) {
		atomic_set(&this_adm.copp_id[i], 0);
		atomic_set(&this_adm.copp_cnt[i], 0);
		atomic_set(&this_adm.copp_state[i], 0);
		atomic_set(&this_adm.copp_test[i], 0);
	}
	return 0;
}

device_initcall(adm_init);
