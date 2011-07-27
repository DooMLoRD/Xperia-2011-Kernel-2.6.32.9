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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/msm_audio.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <mach/qdsp6v2/audio_dev_ctl.h>
#include <mach/dal.h>
#include <mach/debug_mm.h>
#include "q6voice.h"

#define TIMEOUT_MS 3000

struct voice_data {
	int voc_state;/*INIT, CHANGE, RELEASE, RUN */
	struct task_struct *task;

	wait_queue_head_t mvm_wait;
	wait_queue_head_t cvs_wait;
	wait_queue_head_t cvp_wait;

	uint32_t device_events;

	/* cache the values related to Rx and Tx */
	struct device_data dev_rx;
	struct device_data dev_tx;

	/* these default values are for all devices */
	uint32_t default_mute_val;
	uint32_t default_vol_val;
	uint32_t default_sample_val;

	/* call status */
	int v_call_status; /* Start or End */

	void *apr_mvm;
	void *apr_cvs;
	void *apr_cvp;

	u32 mvm_state;
	u32 cvs_state;
	u32 cvp_state;

	/* handle */
	u16 mvm_handle;
	u16 cvs_handle;
	u16 cvp_handle;
};

struct voice_data voice;

static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data);

static int32_t modem_mvm_callback(struct apr_client_data *data, void *priv);
static int32_t modem_cvs_callback(struct apr_client_data *data, void *priv);
static int32_t modem_cvp_callback(struct apr_client_data *data, void *priv);

static int voice_apr_register(struct voice_data *v)
{
	int rc = 0;

	MM_DBG("into voice_apr_register_callback\n");
	/* register callback to APR */
	if (v->apr_mvm == NULL) {
		MM_DBG("start to register MVM callback\n");
		v->apr_mvm = apr_register("MODEM", "MVM", modem_mvm_callback,
								0xFFFFFFFF, v);
		if (v->apr_mvm == NULL) {
			MM_ERR("Unable to register MVM\n");
			rc = -ENODEV;
			goto done;
		}
	}
	if (v->apr_cvs == NULL) {
		MM_DBG("start to register CVS callback\n");
		v->apr_cvs = apr_register("MODEM", "CVS", modem_cvs_callback,
								0xFFFFFFFF, v);
		if (v->apr_cvs == NULL) {
			MM_ERR("Unable to register CVS\n");
			rc = -ENODEV;
			goto err;
		}
	}
	if (v->apr_cvp == NULL) {
		MM_DBG("start to register CVP callback\n");
		v->apr_cvp = apr_register("MODEM", "CVP", modem_cvp_callback,
								0xFFFFFFFF, v);
		if (v->apr_cvp == NULL) {
			MM_ERR("Unable to register CVP\n");
			rc = -ENODEV;
			goto err1;
		}
	}
	return 0;

err1:
	apr_deregister(v->apr_cvs);
err:
	apr_deregister(v->apr_mvm);

done:
	return rc;

}

static int voice_create_mvm_cvs_session(struct voice_data *v)
{
	int ret = 0;
	struct mvm_create_passive_ctl_session_cmd mvm_session_cmd;
	struct cvs_create_passive_ctl_session_cmd cvs_session_cmd;

	/* start to ping if modem service is up */
	MM_DBG("in voice_create_mvm_cvs_session, mvm_hdl=%d, cvs_hdl=%d\n",
					v->mvm_handle, v->cvs_handle);
	/* send cmd to create mvm session and wait for response */
	if (!v->mvm_handle) {
		mvm_session_cmd.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		mvm_session_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(mvm_session_cmd) - APR_HDR_SIZE);
		MM_INFO("send mvm create session pkt size = %d\n",
					mvm_session_cmd.hdr.pkt_size);
		mvm_session_cmd.hdr.src_port = 0;
		mvm_session_cmd.hdr.dest_port = 0;
		mvm_session_cmd.hdr.token = 0;
		mvm_session_cmd.hdr.opcode =
				VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION;
		v->mvm_state = 1;
		ret = apr_send_pkt(v->apr_mvm, (uint32_t *) &mvm_session_cmd);
		if (ret < 0) {
			MM_ERR("Fail in sending MVM_CONTROL_SESSION\n");
			goto fail;
		}
		ret = wait_event_timeout(v->mvm_wait, (v->mvm_state == 0),
						msecs_to_jiffies(TIMEOUT_MS));
		if (ret < 0) {
			MM_ERR("%s: wait_event timeout\n", __func__);
			goto fail;
		}
	}

	/* send cmd to create cvs session */
	if (!v->cvs_handle) {
		cvs_session_cmd.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		cvs_session_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(cvs_session_cmd) - APR_HDR_SIZE);
		MM_INFO("send stream create session pkt size = %d\n",
					cvs_session_cmd.hdr.pkt_size);
		cvs_session_cmd.hdr.src_port = 0;
		cvs_session_cmd.hdr.dest_port = 0;
		cvs_session_cmd.hdr.token = 0;
		cvs_session_cmd.hdr.opcode =
				VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION;
		strcpy(cvs_session_cmd.cvs_session.name, "default modem voice");

		v->cvs_state = 1;
		ret = apr_send_pkt(v->apr_cvs, (uint32_t *) &cvs_session_cmd);
		if (ret < 0) {
			MM_ERR("Fail in sending STREAM_CONTROL_SESSION\n");
			goto fail;
		}
		ret = wait_event_timeout(v->cvs_wait, (v->cvs_state == 0),
						msecs_to_jiffies(TIMEOUT_MS));
		if (ret < 0) {
			MM_ERR("%s: wait_event timeout\n", __func__);
			goto fail;
		}
	}
	return 0;

fail:
	apr_deregister(v->apr_mvm);
	apr_deregister(v->apr_cvs);
	apr_deregister(v->apr_cvp);
	v->cvp_handle = 0;
	v->cvs_handle = 0;

	return -EINVAL;
}

static int voice_start_modem_voice(struct voice_data *v)
{
	struct cvp_create_full_ctl_session_cmd cvp_session_cmd;
	struct apr_hdr cvp_enable_cmd;
	struct mvm_attach_vocproc_cmd mvm_a_vocproc_cmd;
	struct apr_hdr mvm_start_voice_cmd;
	int ret = 0;

	/* create cvp session and wait for response */
	cvp_session_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_session_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_session_cmd) - APR_HDR_SIZE);
	MM_INFO(" send create cvp session, pkt size = %d\n",
				cvp_session_cmd.hdr.pkt_size);
	cvp_session_cmd.hdr.src_port = 0;
	cvp_session_cmd.hdr.dest_port = 0;
	cvp_session_cmd.hdr.token = 0;
	cvp_session_cmd.hdr.opcode =
		VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION;
	cvp_session_cmd.cvp_session.tx_topology_id =
			VSS_IVOCPROC_TOPOLOGY_ID_TX_SM_ECNS;
	cvp_session_cmd.cvp_session.rx_topology_id =
			VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT;
	cvp_session_cmd.cvp_session.direction = 2; /*tx and rx*/
	cvp_session_cmd.cvp_session.network_id = VSS_NETWORK_ID_DEFAULT;
	cvp_session_cmd.cvp_session.tx_port_id = v->dev_tx.dev_port_id;
	cvp_session_cmd.cvp_session.rx_port_id = v->dev_rx.dev_port_id;
	MM_INFO("net_id=%d, dir=%d tx_port_id=%d, rx_port_id=%d\n",
			cvp_session_cmd.cvp_session.network_id,
			cvp_session_cmd.cvp_session.direction,
			cvp_session_cmd.cvp_session.tx_port_id,
			cvp_session_cmd.cvp_session.rx_port_id);

	v->cvp_state = 1;
	ret = apr_send_pkt(v->apr_cvp, (uint32_t *) &cvp_session_cmd);
	if (ret < 0) {
		MM_ERR("Fail in sending VOCPROC_FULL_CONTROL_SESSION\n");
		goto fail;
	}
	MM_DBG("wait for cvp create session event\n");
	ret = wait_event_timeout(v->cvp_wait, (v->cvp_state == 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		MM_ERR("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* enable vocproc and wait for respose */
	cvp_enable_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_enable_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_enable_cmd) - APR_HDR_SIZE);
	MM_DBG("cvp_enable_cmd pkt size = %d, cvp_handle=%d\n",
			cvp_enable_cmd.pkt_size, v->cvp_handle);
	cvp_enable_cmd.src_port = 0;
	cvp_enable_cmd.dest_port = v->cvp_handle;
	cvp_enable_cmd.token = 0;
	cvp_enable_cmd.opcode = VSS_IVOCPROC_CMD_ENABLE;

	v->cvp_state = 1;
	ret = apr_send_pkt(v->apr_cvp, (uint32_t *) &cvp_enable_cmd);
	if (ret < 0) {
		MM_ERR("Fail in sending VSS_IVOCPROC_CMD_ENABLE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->cvp_wait, (v->cvp_state == 0),
					msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		MM_ERR("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* attach vocproc and wait for response */
	mvm_a_vocproc_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_a_vocproc_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_a_vocproc_cmd) - APR_HDR_SIZE);
	MM_INFO("send mvm_a_vocproc_cmd pkt size = %d\n",
				mvm_a_vocproc_cmd.hdr.pkt_size);
	mvm_a_vocproc_cmd.hdr.src_port = 0;
	mvm_a_vocproc_cmd.hdr.dest_port = v->mvm_handle;
	mvm_a_vocproc_cmd.hdr.token = 0;
	mvm_a_vocproc_cmd.hdr.opcode = VSS_ISTREAM_CMD_ATTACH_VOCPROC;
	mvm_a_vocproc_cmd.mvm_attach_cvp_handle.handle = v->cvp_handle;

	v->mvm_state = 1;
	ret = apr_send_pkt(v->apr_mvm, (uint32_t *) &mvm_a_vocproc_cmd);
	if (ret < 0) {
		MM_ERR("Fail in sending VSS_ISTREAM_CMD_ATTACH_VOCPROC\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait, (v->mvm_state == 0),
						msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		MM_ERR("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* start voice and wait for response */
	mvm_start_voice_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_start_voice_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_start_voice_cmd) - APR_HDR_SIZE);
	MM_INFO("send mvm_start_voice_cmd pkt size = %d\n",
				mvm_start_voice_cmd.pkt_size);
	mvm_start_voice_cmd.src_port = 0;
	mvm_start_voice_cmd.dest_port = v->mvm_handle;
	mvm_start_voice_cmd.token = 0;
	mvm_start_voice_cmd.opcode = VSS_IMVM_CMD_START_VOICE;

	v->mvm_state = 1;
	ret = apr_send_pkt(v->apr_mvm, (uint32_t *) &mvm_start_voice_cmd);
	if (ret < 0) {
		MM_ERR("Fail in sending VSS_IMVM_CMD_START_VOICE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait, (v->mvm_state == 0),
					msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		MM_ERR("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	return 0;
fail:
	apr_deregister(v->apr_mvm);
	apr_deregister(v->apr_cvs);
	apr_deregister(v->apr_cvp);

	v->mvm_handle = 0;
	v->cvs_handle = 0;
	v->cvp_handle = 0;

	return -EINVAL;
}

static int voice_stop_modem_voice(struct voice_data *v)
{
	struct apr_hdr mvm_stop_voice_cmd;
	struct mvm_detach_vocproc_cmd mvm_d_vocproc_cmd;
	struct apr_hdr cvp_destroy_session_cmd;
	int ret = 0;

	/* stop voice and wait for the response from mvm */
	mvm_stop_voice_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_stop_voice_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_stop_voice_cmd) - APR_HDR_SIZE);
	MM_INFO("send mvm_stop_voice_cmd pkt size = %d\n",
				mvm_stop_voice_cmd.pkt_size);
	mvm_stop_voice_cmd.src_port = 0;
	mvm_stop_voice_cmd.dest_port = v->mvm_handle;
	mvm_stop_voice_cmd.token = 0;
	mvm_stop_voice_cmd.opcode = VSS_IMVM_CMD_STOP_VOICE;

	v->mvm_state = 1;
	ret = apr_send_pkt(v->apr_mvm, (uint32_t *) &mvm_stop_voice_cmd);
	if (ret < 0) {
		MM_ERR("Fail in sending VSS_IMVM_CMD_STOP_VOICE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait, (v->mvm_state == 0),
						msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		MM_ERR("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* detach VOCPROC and wait for response from mvm */
	mvm_d_vocproc_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_d_vocproc_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_d_vocproc_cmd) - APR_HDR_SIZE);
	MM_INFO("mvm_d_vocproc_cmd  pkt size = %d\n",
				mvm_d_vocproc_cmd.hdr.pkt_size);
	mvm_d_vocproc_cmd.hdr.src_port = 0;
	mvm_d_vocproc_cmd.hdr.dest_port = v->mvm_handle;
	mvm_d_vocproc_cmd.hdr.token = 0;
	mvm_d_vocproc_cmd.hdr.opcode = VSS_ISTREAM_CMD_DETACH_VOCPROC;
	mvm_d_vocproc_cmd.mvm_detach_cvp_handle.handle = v->cvp_handle;

	v->mvm_state = 1;
	ret = apr_send_pkt(v->apr_mvm, (uint32_t *) &mvm_d_vocproc_cmd);
	if (ret < 0) {
		MM_ERR("Fail in sending VSS_ISTREAM_CMD_DETACH_VOCPROC\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait, (v->mvm_state == 0),
						msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		MM_ERR("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	/* destrop cvp session */
	cvp_destroy_session_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_destroy_session_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_destroy_session_cmd) - APR_HDR_SIZE);
	MM_INFO("cvp_destroy_session_cmd pkt size = %d\n",
				cvp_destroy_session_cmd.pkt_size);
	cvp_destroy_session_cmd.src_port = 0;
	cvp_destroy_session_cmd.dest_port = v->cvp_handle;
	cvp_destroy_session_cmd.token = 0;
	cvp_destroy_session_cmd.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

	v->cvp_state = 1;
	ret = apr_send_pkt(v->apr_cvp, (uint32_t *) &cvp_destroy_session_cmd);
	if (ret < 0) {
		MM_ERR("Fail in sending APRV2_IBASIC_CMD_DESTROY_SESSION\n");
		goto fail;
	}
	ret = wait_event_timeout(v->cvp_wait, (v->cvp_state == 0),
						msecs_to_jiffies(TIMEOUT_MS));
	if (ret < 0) {
		MM_ERR("%s: wait_event timeout\n", __func__);
		goto fail;
	}

	v->cvp_handle = 0;

	return 0;

fail:
	apr_deregister(v->apr_mvm);
	apr_deregister(v->apr_cvs);
	apr_deregister(v->apr_cvp);

	return -EINVAL;
}

static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data)
{
	struct voice_data *v = &voice;

	MM_INFO("auddev_cb_function, evt_id=%d,\n", evt_id);
	if ((evt_id != AUDDEV_EVT_START_VOICE) ||
			(evt_id != AUDDEV_EVT_END_VOICE)) {
		if (evt_payload == NULL) {
			MM_ERR(" evt_payload is NULL pointer\n");
			return;
		}
	}

	switch (evt_id) {
	case AUDDEV_EVT_START_VOICE:
		if ((v->voc_state == VOC_INIT) ||
				(v->voc_state == VOC_RELEASE)) {
			v->v_call_status = VOICE_CALL_START;
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED)
				&& (v->dev_tx.enabled == VOICE_DEV_ENABLED)) {
				voice_apr_register(v);
				voice_create_mvm_cvs_session(v);
				voice_start_modem_voice(v);
				v->voc_state = VOC_RUN;
			}
		}
		break;
	case AUDDEV_EVT_DEV_CHG_VOICE:
		v->dev_rx.enabled = VOICE_DEV_DISABLED;
		v->dev_tx.enabled = VOICE_DEV_DISABLED;
		if (v->voc_state == VOC_RUN) {
			/* send cmd to modem to do voice device change */
			voice_stop_modem_voice(v);
			v->voc_state = VOC_CHANGE;
		}
		break;
	case AUDDEV_EVT_DEV_RDY:
		if (v->voc_state == VOC_CHANGE) {
			/* get port Ids */
			if (evt_payload->voc_devinfo.dev_type == DIR_RX) {
				v->dev_rx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_rx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_rx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_rx.enabled = VOICE_DEV_ENABLED;
			} else {
				v->dev_tx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_tx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_tx.enabled = VOICE_DEV_ENABLED;
				v->dev_tx.dev_id =
				evt_payload->voc_devinfo.dev_id;
			}
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED) &&
				(v->dev_tx.enabled == VOICE_DEV_ENABLED)) {
				voice_start_modem_voice(v);
				v->voc_state = VOC_RUN;
			}
		} else if ((v->voc_state == VOC_INIT) ||
			(v->voc_state == VOC_RELEASE)) {
			/* get AFE ports */
			if (evt_payload->voc_devinfo.dev_type == DIR_RX) {
				/* get rx port id */
				v->dev_rx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_rx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_rx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_rx.enabled = VOICE_DEV_ENABLED;
			} else {
				/* get tx port id */
				v->dev_tx.dev_port_id =
					evt_payload->voc_devinfo.dev_port_id;
				v->dev_tx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_tx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_tx.enabled = VOICE_DEV_ENABLED;
			}
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED) &&
				(v->dev_tx.enabled == VOICE_DEV_ENABLED) &&
				(v->v_call_status == VOICE_CALL_START)) {
				voice_apr_register(v);
				voice_create_mvm_cvs_session(v);
				voice_start_modem_voice(v);
				v->voc_state = VOC_RUN;
			}
		}
		break;
	case AUDDEV_EVT_DEVICE_VOL_MUTE_CHG:
		/* cache the mute and volume index value */
		if (evt_payload->voc_devinfo.dev_type == DIR_TX) {
			v->dev_tx.mute =
				evt_payload->voc_vm_info.dev_vm_val.mute;
		}
		break;
	case AUDDEV_EVT_REL_PENDING:
		/* recover the tx mute and rx volume to the default values */
		if (v->voc_state == VOC_RUN) {
			/* send stop voice to modem */
			voice_stop_modem_voice(v);
			v->voc_state = VOC_RELEASE;
		}
		if (evt_payload->voc_devinfo.dev_type == DIR_RX)
			v->dev_rx.enabled = VOICE_DEV_DISABLED;
		else
				v->dev_tx.enabled = VOICE_DEV_DISABLED;

		break;
	case AUDDEV_EVT_END_VOICE:
		/* recover the tx mute and rx volume to the default values */
		v->dev_tx.mute = v->default_mute_val;
		v->dev_rx.volume = v->default_vol_val;

		if (v->voc_state == VOC_RUN) {
			/* call stop modem voice */
			voice_stop_modem_voice(v);
			v->voc_state = VOC_RELEASE;
		}
			v->v_call_status = VOICE_CALL_END;

		break;
	default:
		MM_ERR("UNKNOWN EVENT\n");
	}
	return;
}
EXPORT_SYMBOL(voice_auddev_cb_function);

static int32_t modem_mvm_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr;
	struct voice_data *v = &voice;

	MM_DBG("%s\n", __func__);
	MM_DBG("%s: Payload Length = %d, opcode=%x\n", __func__,
				data->payload_size, data->opcode);

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size) {
			ptr = data->payload;

			MM_INFO("%x %x\n", ptr[0], ptr[1]);
			/* ping mvm service ACK */

			if (ptr[0] ==
			 VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION) {
				MM_DBG("%s: cmd = 0x%x\n", __func__, ptr[0]);
				if (!ptr[1]) {
					v->mvm_handle = data->src_port;
				} else
					MM_INFO("got NACK for sending \
							MVM create session \n");
				v->mvm_state = 0;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_IMVM_CMD_START_VOICE) {
				MM_DBG("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->mvm_state = 0;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_ATTACH_VOCPROC) {
				MM_DBG("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->mvm_state = 0;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_IMVM_CMD_STOP_VOICE) {
				v->mvm_state = 0;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_DETACH_VOCPROC) {
				v->mvm_state = 0;
				wake_up(&v->mvm_wait);
			} else if (ptr[0] == VSS_ISTREAM_CMD_SET_TTY_MODE) {
				v->mvm_state = 0;
				wake_up(&v->mvm_wait);
			} else
				MM_DBG("%s: not match cmd = 0x%x\n",
							__func__, ptr[0]);
		}
	}

	return 0;
}

static int32_t modem_cvs_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr;
	struct voice_data *v = &voice;

	MM_DBG("%s\n", __func__);
	MM_DBG("%s: Payload Length = %d, opcode=%x\n", __func__,
					data->payload_size, data->opcode);

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size) {
			ptr = data->payload;

			MM_INFO("%x %x\n", ptr[0], ptr[1]);
			/*response from modem CVS */
			if (ptr[0] ==
			VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION) {
				if (!ptr[1]) {
					v->cvs_handle = data->src_port;
				} else
					MM_INFO("got NACK for sending \
							CVS create session \n");
				v->cvs_state = 0;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] ==
				VSS_ISTREAM_CMD_CACHE_CALIBRATION_DATA) {
				v->cvs_state = 0;
				wake_up(&v->cvs_wait);
			} else if (ptr[0] ==
					VSS_ISTREAM_CMD_SET_MUTE) {
				v->cvs_state = 0;
				wake_up(&v->cvs_wait);
			} else
				MM_DBG("%s: cmd = 0x%x\n", __func__, ptr[0]);
		}
	}
	return 0;
}

static int32_t modem_cvp_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr;
	struct voice_data *v = &voice;

	MM_DBG("%s\n", __func__);
	MM_DBG("%s: Payload Length = %d, opcode=%x\n", __func__,
				data->payload_size, data->opcode);

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size) {
			ptr = data->payload;

			MM_INFO("%x %x\n", ptr[0], ptr[1]);
			/*response from modem CVP */
			if (ptr[0] ==
				VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION) {
				MM_DBG("%s: cmd = 0x%x\n", __func__, ptr[0]);
				if (!ptr[1]) {
					v->cvp_handle = data->src_port;
					MM_DBG("cvp hdl=%d\n", data->src_port);
				} else
					MM_INFO("got NACK from CVP create \
						session response\n");
				v->cvp_state = 0;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] ==
				VSS_IVOCPROC_CMD_CACHE_CALIBRATION_DATA) {
				MM_DBG("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->cvp_state = 0;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] ==
					VSS_IVOCPROC_CMD_SET_RX_VOLUME_INDEX) {
				v->cvp_state = 0;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] == VSS_IVOCPROC_CMD_ENABLE) {
				v->cvp_state = 0;
				wake_up(&v->cvp_wait);
			} else if (ptr[0] == APRV2_IBASIC_CMD_DESTROY_SESSION) {
				v->cvp_state = 0;
				wake_up(&v->cvp_wait);
			} else
				MM_DBG("%s: not match cmd = 0x%x\n",
							__func__, ptr[0]);
		}
	}
	return 0;
}


static int __init voice_init(void)
{
	int rc = 0;
	struct voice_data *v = &voice;
	MM_INFO("%s\n", __func__); /* Macro prints the file name and function */

	/* set default value */
	v->default_mute_val = 1;  /* default is mute */
	v->default_vol_val = 0;
	v->default_sample_val = 8000;

	/* initialize dev_rx and dev_tx */
	memset(&v->dev_tx, 0, sizeof(struct device_data));
	memset(&v->dev_rx, 0, sizeof(struct device_data));
	v->dev_rx.volume = v->default_vol_val;
	v->dev_tx.mute = v->default_mute_val;

	v->voc_state = VOC_INIT;
	init_waitqueue_head(&v->mvm_wait);
	init_waitqueue_head(&v->cvs_wait);
	init_waitqueue_head(&v->cvp_wait);

	v->mvm_handle = 0;
	v->cvs_handle = 0;
	v->cvp_handle = 0;

	v->apr_mvm = NULL;
	v->apr_cvs = NULL;
	v->apr_cvp = NULL;


	v->device_events = AUDDEV_EVT_DEV_CHG_VOICE |
			AUDDEV_EVT_DEV_RDY |
			AUDDEV_EVT_REL_PENDING |
			AUDDEV_EVT_START_VOICE |
			AUDDEV_EVT_END_VOICE |
			AUDDEV_EVT_DEVICE_VOL_MUTE_CHG |
			AUDDEV_EVT_FREQ_CHG;

	MM_DBG("to register call back\n");
	/* register callback to auddev */
	auddev_register_evt_listner(v->device_events, AUDDEV_CLNT_VOC,
				0, voice_auddev_cb_function, v);

	return rc;
}

device_initcall(voice_init);
