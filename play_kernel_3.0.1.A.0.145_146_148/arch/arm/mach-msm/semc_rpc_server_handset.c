/* arch/arm/mach-msm/semc_rpc_server_handset.c
 *
 * Copyright (C) 2010 SonyEricsson Mobile Communications AB
 *
 * Based on rpc_server_handset.c, Copyright (c) 2008-2009,
 * Code Aurora Forum. All rights reserved.
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
 *
 * Author: Joachim Holst (joachim.holst@sonyericsson.com)
 *
 */

#ifdef CONFIG_MSM_RPCSERVER_HANDSET
#error "CONFIG_MSM_RPCSERVER_HANDSET can not be set with this driver"
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <asm/mach-types.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <mach/semc_rpc_server_handset.h>

/* Defined for RPC connection */
#define HS_SERVER_PROG 0x30000062
#define HS_SERVER_VERS 0x00010001

#define HS_RPC_PROG 0x30000091

#define HS_SUBSCRIBE_SRVC_PROC 0x03
#define HS_REPORT_EVNT_PROC    0x05
#define HS_EVENT_CB_PROC	1
#define HS_EVENT_DATA_VER	1

#define RPC_KEYPAD_NULL_PROC 0
#define RPC_KEYPAD_PASS_KEY_CODE_PROC 2
#define RPC_KEYPAD_SET_PWR_KEY_STATE_PROC 3

void dispatch_event(struct work_struct *work);

static DECLARE_WORK(dispatch_work, dispatch_event);
static LIST_HEAD(semc_rpc_srv_hs_event_list);
static DEFINE_MUTEX(semc_rpc_srv_hs_event_lock);
static struct msm_rpc_client *rpc_client;
static struct semc_handset_data *s_hs_data;

enum hs_event {
	HS_EVNT_EXT_PWR = 0,	/* External Power status        */
	HS_EVNT_HSD,		/* Headset Detection            */
	HS_EVNT_HSTD,		/* Headset Type Detection       */
	HS_EVNT_HSSD,		/* Headset Switch Detection     */
	HS_EVNT_KPD,
	HS_EVNT_FLIP,		/* Flip / Clamshell status (open/close) */
	HS_EVNT_CHARGER,	/* Battery is being charged or not */
	HS_EVNT_ENV,		/* Events from runtime environment like DEM */
	HS_EVNT_REM,		/* Events received from HS counterpart on a
				   remote processor*/
	HS_EVNT_DIAG,		/* Diag Events  */
	HS_EVNT_LAST,		/* Should always be the last event type */
	HS_EVNT_MAX		/* Force enum to be an 32-bit number */
};


enum hs_src_state {
	HS_SRC_STATE_UNKWN = 0,
	HS_SRC_STATE_LO,
	HS_SRC_STATE_HI,
};

struct hs_event_data {
	uint32_t	ver;		/* Version number */
	enum hs_event	event_type;     /* Event Type	*/
	enum hs_event	enum_disc;      /* discriminator */
	uint32_t	data_length;	/* length of the next field */
	enum hs_src_state	data;   /* Pointer to data */
	uint32_t	data_size;	/* Elements to be processed in data */
};

enum hs_return_value {
	HS_EKPDLOCKED     = -2,	/* Operation failed because keypad is locked */
	HS_ENOTSUPPORTED  = -1,	/* Functionality not supported */
	HS_FALSE          =  0, /* Inquired condition is not true */
	HS_FAILURE        =  0, /* Requested operation was not successful */
	HS_TRUE           =  1, /* Inquired condition is true */
	HS_SUCCESS        =  1, /* Requested operation was successful */
	HS_MAX_RETURN     =  0x7FFFFFFF/* Force enum to be a 32 bit number */
};

struct hs_key_data {
	uint32_t ver;        /* Version number to track sturcture changes */
	uint32_t code;       /* which key? */
	uint32_t parm;       /* key status. Up/down or pressed/released */
};

enum hs_subs_srvc {
	HS_SUBS_SEND_CMD = 0, /* Subscribe to send commands to HS */
	HS_SUBS_RCV_EVNT,     /* Subscribe to receive Events from HS */
	HS_SUBS_SRVC_MAX
};

enum hs_subs_req {
	HS_SUBS_REGISTER,    /* Subscribe   */
	HS_SUBS_CANCEL,      /* Unsubscribe */
	HS_SUB_STATUS_MAX
};

enum hs_event_class {
	HS_EVNT_CLASS_ALL = 0, /* All HS events */
	HS_EVNT_CLASS_LAST,    /* Should always be the last class type   */
	HS_EVNT_CLASS_MAX
};

enum hs_cmd_class {
	HS_CMD_CLASS_LCD = 0, /* Send LCD related commands              */
	HS_CMD_CLASS_KPD,     /* Send KPD related commands              */
	HS_CMD_CLASS_LAST,    /* Should always be the last class type   */
	HS_CMD_CLASS_MAX
};

/* Add newer versions at the top of array */
static const unsigned int rpc_vers[] = {
	0x00030001,
	0x00020001,
	0x00010001,
};

/*
 * Receive events or send command
 */
union hs_subs_class {
	enum hs_event_class	evnt;
	enum hs_cmd_class	cmd;
};

struct hs_subs {
	uint32_t                ver;
	enum hs_subs_srvc	srvc;  /* commands or events */
	enum hs_subs_req	req;   /* subscribe or unsubscribe  */
	uint32_t		host_os;
	enum hs_subs_req	disc;  /* discriminator    */
	union hs_subs_class     id;
};

struct hs_event_cb_recv {
	uint32_t cb_id;
	uint32_t hs_key_data_ptr;
	struct hs_key_data key;
};

struct semc_rpc_srv_hs_key_event {
	uint32_t key;
	uint32_t event;
};

struct semc_rpc_srv_hs_key_event_node {
	struct semc_rpc_srv_hs_key_event *event;
	struct list_head list;
};

static
struct semc_rpc_srv_hs_key_event *semc_rpc_srv_hs_get_next_key_event(void)
{
	struct semc_rpc_srv_hs_key_event_node *node;
	struct semc_rpc_srv_hs_key_event *ev = NULL;

	mutex_lock(&semc_rpc_srv_hs_event_lock);
	if (!list_empty(&semc_rpc_srv_hs_event_list)) {
		node = list_first_entry(&semc_rpc_srv_hs_event_list,
					struct semc_rpc_srv_hs_key_event_node,
					list);
		ev = node->event;
		list_del(&node->list);
		kfree(node);
	}

	mutex_unlock(&semc_rpc_srv_hs_event_lock);

	return ev;
}

static int semc_rpc_srv_hs_add_key_event(struct semc_rpc_srv_hs_key_event *ev)
{

	struct semc_rpc_srv_hs_key_event_node *evn;

	if(s_hs_data->num_callbacks <= 0)
		goto cleanup_ev;

	evn = kmalloc(sizeof(struct semc_rpc_srv_hs_key_event_node),
		      GFP_ATOMIC);

	if (NULL == evn) {
		pr_err("%s : Failed to allocate memory "	\
		       "for button event node!\n"
		       , __func__);
		goto cleanup_evn;
	}

	evn->event = ev;

	mutex_lock(&semc_rpc_srv_hs_event_lock);
	list_add_tail(&evn->list, &semc_rpc_srv_hs_event_list);
	mutex_unlock(&semc_rpc_srv_hs_event_lock);

	(void)schedule_work(&dispatch_work);

	return 0;

cleanup_evn:
	kfree(evn);
cleanup_ev:
	kfree(ev);

	return -EFAULT;
}

static void semc_rpc_srv_hs_clear_event_list(void)
{
	struct list_head *ptr;
	struct list_head *n;
	struct semc_rpc_srv_hs_key_event_node *evn;

	mutex_lock(&semc_rpc_srv_hs_event_lock);
	if (!list_empty(&semc_rpc_srv_hs_event_list)) {
		list_for_each_safe(ptr, n, &semc_rpc_srv_hs_event_list) {
			evn = list_entry(ptr,
					 struct semc_rpc_srv_hs_key_event_node,
					 list);
			list_del(ptr);
			kfree(evn->event);
			kfree(evn);
		}
	}
	mutex_unlock(&semc_rpc_srv_hs_event_lock);
}


/*
 * Dispatch events in a separate work thread.
 * We don't want to block RPC calls while clients
 * are handling the events.
 */
void dispatch_event(struct work_struct *work)
{

	struct semc_rpc_srv_hs_key_event *ev;
	int i;

	while((ev = semc_rpc_srv_hs_get_next_key_event())) {
		for (i = 0; i < s_hs_data->num_callbacks; i++) {
			if (s_hs_data->callbacks[i])
				s_hs_data->callbacks[i](ev->key, ev->event);
		}

		kfree(ev);
	}
}


/*
 * tuple format: (key_code, key_param)
 * key-press = (key_code, 0)
 * key-release = (key_code, 0xff)
 */
static void report_hs_key(uint32_t key_code, uint32_t key_parm)
{
	struct semc_rpc_srv_hs_key_event *ev;

	ev = kmalloc(sizeof(struct semc_rpc_srv_hs_key_event),
		     GFP_ATOMIC);

	if (NULL != ev) {
		ev->key = key_code;
		ev->event = key_parm;
		(void)semc_rpc_srv_hs_add_key_event(ev);
	} else {
		pr_err("***** %s - Failed to allocate memory\n", __func__);
	}
}

static int handle_hs_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	struct rpc_keypad_pass_key_code_args {
		uint32_t key_code;
		uint32_t key_parm;
	};

	switch (req->procedure) {
	case RPC_KEYPAD_NULL_PROC:
		return 0;

	case RPC_KEYPAD_PASS_KEY_CODE_PROC: {
		struct rpc_keypad_pass_key_code_args *args;

		args = (struct rpc_keypad_pass_key_code_args *)(req + 1);
		args->key_code = be32_to_cpu(args->key_code);
		args->key_parm = be32_to_cpu(args->key_parm);

		report_hs_key(args->key_code, args->key_parm);

		return 0;
	}

	case RPC_KEYPAD_SET_PWR_KEY_STATE_PROC:
		/* This RPC function must be available for the ARM9
		 * to function properly.  This function is redundant
		 * when RPC_KEYPAD_PASS_KEY_CODE_PROC is handled. So
		 * input_report_key is not needed.
		 */
		return 0;
	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_SERVER_PROG,
	.vers		= HS_SERVER_VERS,
	.rpc_call	= handle_hs_rpc_call,
};

static int process_subs_srvc_callback(struct hs_event_cb_recv *recv)
{
	if (!recv)
		return -ENODATA;

	report_hs_key(be32_to_cpu(recv->key.code), be32_to_cpu(recv->key.parm));

	return 0;
}

static void process_hs_rpc_request(uint32_t proc, void *data)
{
	if (proc == HS_EVENT_CB_PROC)
		process_subs_srvc_callback(data);
	else
		pr_err("%s: unknown rpc proc %d\n", __func__, proc);
}

static int hs_rpc_report_event_arg(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	struct hs_event_rpc_req {
		uint32_t hs_event_data_ptr;
		struct hs_event_data data;
	};

	struct hs_event_rpc_req *req = buffer;

	req->hs_event_data_ptr	= cpu_to_be32(0x1);
	req->data.ver		= cpu_to_be32(HS_EVENT_DATA_VER);
	req->data.event_type	= cpu_to_be32(HS_EVNT_HSD);
	req->data.enum_disc	= cpu_to_be32(HS_EVNT_HSD);
	req->data.data_length	= cpu_to_be32(0x1);
	req->data.data		= cpu_to_be32(*(enum hs_src_state *)data);
	req->data.data_size	= cpu_to_be32(sizeof(enum hs_src_state));

	return sizeof(*req);
}

static int hs_rpc_report_event_res(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	enum hs_return_value result;

	result = be32_to_cpu(*(enum hs_return_value *)buffer);
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	if (result == HS_SUCCESS)
		return 0;

	return 1;
}

void report_headset_status(bool connected)
{
	int rc = -1;
	enum hs_src_state status;

	if (connected == true)
		status = HS_SRC_STATE_HI;
	else
		status = HS_SRC_STATE_LO;

	rc = msm_rpc_client_req(rpc_client, HS_REPORT_EVNT_PROC,
				hs_rpc_report_event_arg, &status,
				hs_rpc_report_event_res, NULL, -1);

	if (rc)
		pr_err("%s: couldn't send rpc client request\n", __func__);
}
EXPORT_SYMBOL(report_headset_status);

static int hs_rpc_register_subs_arg(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	struct hs_subs_rpc_req {
		uint32_t hs_subs_ptr;
		struct hs_subs hs_subs;
		uint32_t hs_cb_id;
		uint32_t hs_handle_ptr;
		uint32_t hs_handle_data;
	};

	struct hs_subs_rpc_req *req = buffer;

	req->hs_subs_ptr	= cpu_to_be32(0x1);
	req->hs_subs.ver	= cpu_to_be32(0x1);
	req->hs_subs.srvc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	req->hs_subs.req	= cpu_to_be32(HS_SUBS_REGISTER);
	req->hs_subs.host_os	= cpu_to_be32(0x4); /* linux */
	req->hs_subs.disc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	req->hs_subs.id.evnt	= cpu_to_be32(HS_EVNT_CLASS_ALL);

	req->hs_cb_id		= cpu_to_be32(0x1);

	req->hs_handle_ptr	= cpu_to_be32(0x1);
	req->hs_handle_data	= cpu_to_be32(0x0);

	return sizeof(*req);
}

static int hs_rpc_register_subs_res(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	uint32_t result;

	result = be32_to_cpu(*((uint32_t *)buffer));
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}

static int hs_cb_func(struct msm_rpc_client *client, void *buffer, int in_size)
{
	int rc = -1;

	struct rpc_request_hdr *hdr = buffer;

	hdr->type = be32_to_cpu(hdr->type);
	hdr->xid = be32_to_cpu(hdr->xid);
	hdr->rpc_vers = be32_to_cpu(hdr->rpc_vers);
	hdr->prog = be32_to_cpu(hdr->prog);
	hdr->vers = be32_to_cpu(hdr->vers);
	hdr->procedure = be32_to_cpu(hdr->procedure);

	process_hs_rpc_request(hdr->procedure,
			    (void *) (hdr + 1));

	msm_rpc_start_accepted_reply(client, hdr->xid,
				     RPC_ACCEPTSTAT_SUCCESS);
	rc = msm_rpc_send_accepted_reply(client, 0);
	if (rc) {
		pr_err("%s: sending reply failed: %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int __init hs_rpc_cb_init(void)
{
	int rc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(rpc_vers); i++) {
		rpc_client = msm_rpc_register_client("hs",
			HS_RPC_PROG, rpc_vers[i], 0, hs_cb_func);

		if (IS_ERR(rpc_client))
			pr_err("%s: couldn't open rpc client with version %d"
				" err %ld\n",
				 __func__, rpc_vers[i], PTR_ERR(rpc_client));
		else
			break;
	}

	if (IS_ERR(rpc_client)) {
		pr_err("%s: Incompatible RPC version error\n", __func__);
		return PTR_ERR(rpc_client);
	}
	rc = msm_rpc_client_req(rpc_client, HS_SUBSCRIBE_SRVC_PROC,
				hs_rpc_register_subs_arg, NULL,
				hs_rpc_register_subs_res, NULL, -1);
	if (rc) {
		pr_err("%s: couldn't send rpc client request\n", __func__);
		msm_rpc_unregister_client(rpc_client);
	}

	return rc;
}

static int __devinit hs_rpc_init(void)
{
	int rc;

	rc = hs_rpc_cb_init();
	if (rc)
		pr_err("%s: failed to initialize rpc client\n", __func__);

	rc = msm_rpc_create_server(&hs_rpc_server);
	if (rc < 0)
		pr_err("%s: failed to create rpc server\n", __func__);

	return 0;
}

static void __devexit hs_rpc_deinit(void)
{
	if (rpc_client)
		msm_rpc_unregister_client(rpc_client);

	semc_rpc_srv_hs_clear_event_list();
}

static int __devinit hs_probe(struct platform_device *pdev)
{
	int rc;
	if (NULL == pdev->dev.platform_data)
		return -EFAULT;

	s_hs_data = pdev->dev.platform_data;

	rc = hs_rpc_init();
	if (rc)
		goto err_hs_rpc_init;

	return 0;


err_hs_rpc_init:
	return rc;
}

static int __devexit hs_remove(struct platform_device *pdev)
{
	hs_rpc_deinit();
	return 0;
}

static struct platform_driver hs_driver = {
	.probe		= hs_probe,
	.remove		= __devexit_p(hs_remove),
	.driver		= {
		.name	= SEMC_HANDSET_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init hs_init(void)
{
	return platform_driver_register(&hs_driver);
}
late_initcall(hs_init);

static void __exit hs_exit(void)
{
	platform_driver_unregister(&hs_driver);
}
module_exit(hs_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:semc-handset");
