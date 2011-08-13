 /* drivers/usb/gadget/marlin_scsi_ext.c
 *
 * Marlin USB SCSI Command Extension
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/poll.h>
#include <asm/atomic.h>
#include <asm/byteorder.h>
#include <linux/uaccess.h>

#include "marlin_scsi_ext.h"

#ifdef DEBUG
#define MLOG_DUM_MAX_LEN		(64)
#define MLOG_PRINT_LINE_LEN		(64)
#define MLOG_PRINT_MLDD(data_p)		\
	mldd_print_mldd(__func__, __LINE__, (data_p))
#define MLOG_DUM(nm, sz, data_p)	\
	mltl_data_dump(__func__, __LINE__, (nm), (sz), (data_p))
#else
#define MLOG_PRINT_MLDD(data_p)
#define MLOG_DUM(nm, sz, data_p)
#endif

static char *cmd_type_str[] = {
	"SENDKEY_CHECK",
	"SENDKEY_SIZE",
	"SENDKEY_DATA",
	"REPORTKEY_SIZE",
	"REPORTKEY_DATA",
	"RESETTED",
	"UNLOADED",
	"SYS_ERR"
};

static char *proc_res_str[] = {
	"RES_OK",
	"RES_INVALID_CMD",
	"RES_CMD_CANCELLED",
	"RES_SYS_ERR",
	"RES_READY",
	"RES_NOT_READY",
	"RES_INVALID_CMD_OP_CODE",
	"RES_SYS_RESOURCE_FAILURE"
};

static char *buf_state_str[] = {
	"BUF_STATE_NONE",		/* 0x00 */
	"BUF_STATE_READ_READY",		/* 0x01 */
	"BUF_STATE_UNKNOWN",		/* 0x02 */
	"BUF_STATE_WRITE_READY",	/* 0x03 */
	"BUF_STATE_UNKNOWN",		/* 0x04 */
	"BUF_STATE_UNKNOWN",		/* 0x05 */
	"BUF_STATE_UNKNOWN",		/* 0x06 */
	"BUF_STATE_UNKNOWN",		/* 0x07 */
	"BUF_STATE_UNKNOWN",		/* 0x08 */
	"BUF_STATE_UNKNOWN",		/* 0x09 */
	"BUF_STATE_UNKNOWN",		/* 0x0A */
	"BUF_STATE_UNKNOWN",		/* 0x0B */
	"BUF_STATE_UNKNOWN",		/* 0x0C */
	"BUF_STATE_UNKNOWN",		/* 0x0D */
	"BUF_STATE_UNKNOWN",		/* 0x0E */
	"BUF_STATE_UNKNOWN",		/* 0x0F */
	"BUF_STATE_WRITE_COMPLETED",	/* 0x10 */
	"BUF_STATE_READ_ERR",		/* 0x11 */
	"BUF_STATE_WRITE_ERR",		/* 0x12 */
	"BUF_STATE_SYS_ERR"		/* 0x13 */
};

static char *dd_state_str[] = {
	"STATE_NONE",			/* 0x00 */
	"STATE_TL_READY",		/* 0x01 */
	"STATE_UNKNOWN",		/* 0x02 */
	"STATE_UNKNOWN",		/* 0x03 */
	"STATE_UNKNOWN",		/* 0x04 */
	"STATE_UNKNOWN",		/* 0x05 */
	"STATE_UNKNOWN",		/* 0x06 */
	"STATE_UNKNOWN",		/* 0x07 */
	"STATE_UNKNOWN",		/* 0x08 */
	"STATE_UNKNOWN",		/* 0x09 */
	"STATE_UNKNOWN",		/* 0x0A */
	"STATE_UNKNOWN",		/* 0x0B */
	"STATE_UNKNOWN",		/* 0x0C */
	"STATE_UNKNOWN",		/* 0x0D */
	"STATE_UNKNOWN",		/* 0x0E */
	"STATE_UNKNOWN",		/* 0x0F */
	"STATE_DD_READY",		/* 0x10 */
	"STATE_ML_READY",		/* 0x11 */
	"STATE_UNKNOWN",		/* 0x12 */
	"STATE_ML_INPROGRESS"		/* 0x13 */
};

#define MLDD_CMDTYPE_MAX	(MLTL_DD_CMD_TYPE_SYS_ERR)
#define MLDD_CMDTYPE2STR(type)			\
	((((uint)type) > MLDD_CMDTYPE_MAX) ?	\
	"UNKNOWN" : cmd_type_str[(type)])

#define MLDD_PROCRES_MAX	(MLTL_DD_PROC_RES_SYS_RESOURCE_FAILURE)
#define MLDD_PROCRES2STR(res)			\
	((((uint)res) > MLDD_PROCRES_MAX) ?	\
	"RES_UNKNOWN" : proc_res_str[(res)])

#define MLDD_BUFSTATE_MAX	(MLDD_BUF_STATE_SYS_ERR)
#define MLDD_BUFSTATE2STR(state)		\
	((((uint)state) > MLDD_BUFSTATE_MAX) ?	\
		"BUF_STATE_UNKNOWN" : buf_state_str[(state)])

#define MLDD_DDSTATE_MAX	(MLDD_STATE_ML_INPROGRESS)
#define MLDD_DDSTATE2STR(state)			\
	((((uint)state) > MLDD_DDSTATE_MAX) ?	\
	"STATE_UNKNOWN" : dd_state_str[(state)])

/**
 * Data buffer state
 */
#define MLDD_BUF_STATE_RESP_BITMASK	(0xF0)
#define MLDD_COND_WAIT_RESP	\
	((the_marlin.mldd_data.buf_state & MLDD_BUF_STATE_RESP_BITMASK) > 0)
#define MLDD_BUF_STATE_GET()		(the_marlin.mldd_data.buf_state)
#define MLDD_BUF_STATE_GET_STR()	\
	MLDD_BUFSTATE2STR(the_marlin.mldd_data.buf_state)
#define MLDD_BUF_STATE_SET(state) {\
	pr_debug("Buf State: %s -> %s\n",	\
		MLDD_BUF_STATE_GET_STR(), MLDD_BUFSTATE2STR(state));	\
	the_marlin.mldd_data.buf_state = (state);	\
}

/**
 * Driver state
 */
#define MLDD_STATE_GUARD_TIME	(5)	/* ms */
#define MLDD_STATE_GET()	atomic_read(&the_marlin.mldd_state)
#define MLDD_STATE_GET_STR()	MLDD_DDSTATE2STR(MLDD_STATE_GET())
#define MLDD_STATE_SET(state) {\
	pr_debug("DD State: %s -> %s\n",	\
		MLDD_STATE_GET_STR(), MLDD_DDSTATE2STR(state));	\
	atomic_set(&the_marlin.mldd_state, (state));	\
}
#define MLDD_STATE_IS_TLREADY()	((MLDD_STATE_GET() & MLDD_STATE_TL_READY) > 0)
#define MLDD_STATE_IS_DDREADY()	((MLDD_STATE_GET() & MLDD_STATE_DD_READY) > 0)


/**
 * Marlin SCSI command extentions
 */
#define MARLIN_KEY_FORMAT_MASK			(0x3F)
#define MARLIN_KEY_FROMAT_SEND_KEY_SIZE		(0x3A)
#define MARLIN_KEY_FROMAT_SEND_KEY_DATA		(0x3B)
#define MARLIN_KEY_FROMAT_SEND_KEY_CHECK	(0x3F)
#define MARLIN_KEY_FROMAT_REPORT_KEY_SIZE	(0x3C)
#define MARLIN_KEY_FROMAT_REPORT_KEY_DATA	(0x3D)
#define MARLIN_KEY_FROMAT_REPORT_KEY_CAP	(0x3E)

#define MARLIN_KEY_FORMAT(x)			((u8) ((x) & 0x3F))
#define MARLIN_AGID(x)				((u8) (((x) >> 6) & 0x03))

#define MARLIN_SS_NO_SENSE			(0x000000)
#define MARLIN_SS_INVALID_COMMAND		(0x052000)
#define MARLIN_SS_SYSTEM_RESOURCE_FAILURE	(0x055500)
#define MARLIN_SS_INVALID_FIELD_IN_CDB		(0x052400)
#define MARLIN_SS_LOGICAL_UNIT_NOT_SUPPORTED	(0x052500)
#define MARLIN_SS_DEVICE_NOT_READY		(0x020407)


#define DBS_D2H (0x4000 - 0x100)
#define DBS_H2D (0x4000 - 0x100)


/**
 * Command type from driver to transport
 */
enum mltl_dd_cmd_type_t {
	MLTL_DD_CMD_TYPE_SENDKEY_CHECK,
	MLTL_DD_CMD_TYPE_SENDKEY_SIZE,
	MLTL_DD_CMD_TYPE_SENDKEY_DATA,
	MLTL_DD_CMD_TYPE_REPORTKEY_SIZE,
	MLTL_DD_CMD_TYPE_REPORTKEY_DATA,
	MLTL_DD_CMD_TYPE_RESETTED,
	MLTL_DD_CMD_TYPE_UNLOADED,
	MLTL_DD_CMD_TYPE_SYS_ERR
};


/**
 * Process result from transport to driver.
 */
enum mltl_dd_proc_res_t {
	MLTL_DD_PROC_RES_OK,
	MLTL_DD_PROC_RES_INVALID_CMD,
	MLTL_DD_PROC_RES_CMD_CANCELLED,
	MLTL_DD_PROC_RES_SYS_ERR,
	MLTL_DD_PROC_RES_READY,
	MLTL_DD_PROC_RES_NOT_READY,
	MLTL_DD_PROC_RES_INVALID_CMD_OP_CODE,
	MLTL_DD_PROC_RES_SYS_RESOURCE_FAILURE
};

enum mldd_ioctl_res_t {
	MLDD_RES_OK,
	MLDD_RES_ERR_NOT_OPENNED,
	MLDD_RES_ERR_ALREADY_OPENNED,
	MLDD_RES_ERR_INVALID_SEQ,
	MLDD_RES_ERR_INVALID_REQUEST,
};

/**
 * Response data of ioctl()
 */
struct mltl_ioctl_resp_t {
	enum mldd_ioctl_res_t	result;
	u16			dbs_d2h;
	u16			dbs_h2d;
} __attribute__ ((packed));

/**
 * R/W buffer state
 */
enum mldd_buf_state_t {
	MLDD_BUF_STATE_NONE		= 0x00,
	MLDD_BUF_STATE_READ_READY	= 0x01,
	MLDD_BUF_STATE_WRITE_READY	= 0x03,
	MLDD_BUF_STATE_WRITE_COMPLETED	= 0x10,
	MLDD_BUF_STATE_READ_ERR	= 0x11,
	MLDD_BUF_STATE_WRITE_ERR	= 0x12,
	MLDD_BUF_STATE_SYS_ERR		= 0x13
};


/**
 * Command data from Device Handler to Character Device
 */
struct mldd_cmd_t {
	enum mltl_dd_cmd_type_t		cmd_type;
	u16				size;
	u_char				*data_p;
} __attribute__ ((packed));


/**
 * Response data from Character Device to Device Handler
 */
struct mldd_resp_t {
	enum mltl_dd_proc_res_t		res_type;
	u16				size;
	u_char				*data_p;
} __attribute__ ((packed));


/**
 * Data used between Device Handler and Character Device
 */
struct mldd_data_t {
	enum mldd_buf_state_t	buf_state;
	struct mldd_cmd_t	cmd;
	struct mldd_resp_t	resp;
};


/**
 * Marlin USB Driver state
 */
enum mldd_state_t {
	MLDD_STATE_NONE			= 0x00,
	MLDD_STATE_TL_READY		= 0x01,
	MLDD_STATE_DD_READY		= 0x10,
	MLDD_STATE_ML_READY		= 0x11,
	MLDD_STATE_ML_INPROGRESS	= 0x13
};


/**
 * Process result of functions
 */
enum mldd_proc_res_t {
	MLDD_PROC_RES_OK,
	MLDD_PROC_RES_INVALID_STATE,
	MLDD_PROC_RES_SYS_ERR
};

/**
 * Global Variable
 */
struct marlin_dev {
	int init_completed;
	wait_queue_head_t mldd_wq;
	atomic_t mldd_state;
	struct mldd_data_t mldd_data;

	struct semaphore mldd_state_sem;

	struct work_struct marlin_enable;
	struct work_struct marlin_disable;
	struct work_struct marlin_mount;
	struct work_struct marlin_unmount;
};

static struct marlin_dev the_marlin = {
	.mldd_state = ATOMIC_INIT(MLDD_STATE_NONE),
};

/**
 * Static Functions
 */

#ifdef DEBUG
static void mltl_data_dump(const char *func, int line,
		char *name, uint size, u_char *data_p)
{
	int i;
	int end_num = 0;
	int next_idx = 0;
	uint head_len = size;
	uint tail_idx = 0;
	char *pre_str = "  ";

	if (!data_p)
		return;

	pr_debug("[%s(%d)] [DUM] %s (0x%08X, %04d):\n",
			func, line, name, (int)data_p, size);

	if (size > MLOG_DUM_MAX_LEN) {
		if (MLOG_DUM_MAX_LEN % 2 == 0) {
			head_len = (MLOG_DUM_MAX_LEN / 2) - 1;
			tail_idx = size - head_len;
		} else {
			head_len = (MLOG_DUM_MAX_LEN / 2);
			tail_idx = size - head_len + 1;
		}
	}

	while (next_idx < size) {
		end_num = ((size - next_idx) < MLOG_PRINT_LINE_LEN) ?
				(size - next_idx) : MLOG_PRINT_LINE_LEN;
		pr_debug("%s<0x%08X> ",
			pre_str,
			(int)(&data_p[next_idx]));
		for (i = 0; i < end_num; i++) {
			if (next_idx < head_len || next_idx >= tail_idx)
				pr_debug("%02X ", data_p[next_idx]);
			else if (next_idx == head_len ||
					next_idx == head_len + 1)
				pr_debug(".. ");
			else
				end_num++;

			next_idx++;
			if (next_idx >= size)
				break;
		}
		pr_debug("\n");
	}
}


static void mldd_print_mldd(const char *func,
		int line, struct mldd_data_t *mldd_data_p)
{
	pr_debug("[%s(%d)] mldd_data:\n", func, line);
	if (!mldd_data_p)
		return;

	pr_debug("buf_state=%s cmd.cmd_type=%s cmd.size=%d"
			" resp.res_type=%s resp.size=%d\n",
			MLDD_BUFSTATE2STR(mldd_data_p->buf_state),
			MLDD_CMDTYPE2STR(mldd_data_p->cmd.cmd_type),
			mldd_data_p->cmd.size,
			MLDD_PROCRES2STR(mldd_data_p->resp.res_type),
			mldd_data_p->resp.size
		);
}
#endif


static int mldd_wait_process_completed(void)
{
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(1000);
	while (MLDD_STATE_GET() == MLDD_STATE_ML_INPROGRESS) {
		pr_debug("wait process completed...\n");
		msleep(MLDD_STATE_GUARD_TIME);
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}

	return 0;
}

/**
 * Send command data to transport and get the reponse.
 */
static enum mldd_proc_res_t mldd_raise_data(
		enum mltl_dd_cmd_type_t cmd_type,
		uint cmd_size,
		u8 *cmd_data_p,
		u16 *resp_size_p,
		u8 *resp_data_p)
{
	/* Send command data */
	pr_debug("cmd_type=%s, cmd_size=%d\n",
			MLDD_CMDTYPE2STR(cmd_type),
			cmd_size);
	if (MLDD_STATE_GET() != MLDD_STATE_ML_READY) {
		pr_err("Invalid dd state. (%s)\n", MLDD_STATE_GET_STR());
		return MLDD_PROC_RES_INVALID_STATE;
	}

	/* Set driver state to MLDD_STATE_ML_INPROGRESS */
	down(&the_marlin.mldd_state_sem);
	MLDD_STATE_SET(MLDD_STATE_ML_INPROGRESS);
	up(&the_marlin.mldd_state_sem);

	/* Setup data buffer */
	the_marlin.mldd_data.buf_state = MLDD_BUF_STATE_READ_READY;
	the_marlin.mldd_data.cmd.cmd_type = cmd_type;
	the_marlin.mldd_data.cmd.size = cmd_size;
	the_marlin.mldd_data.cmd.data_p = cmd_data_p;
	the_marlin.mldd_data.resp.res_type = MLTL_DD_PROC_RES_OK;
	the_marlin.mldd_data.resp.size = (resp_size_p != NULL) ?
						(*resp_size_p) : 0;
	the_marlin.mldd_data.resp.data_p = resp_data_p;
	MLOG_PRINT_MLDD(&the_marlin.mldd_data);

	/* Send data to transport */
	wake_up_interruptible(&the_marlin.mldd_wq);

	/* Wait response data */
	wait_event(the_marlin.mldd_wq, MLDD_COND_WAIT_RESP);

	/* check error */
	if (MLDD_BUF_STATE_GET() != MLDD_BUF_STATE_WRITE_COMPLETED) {
		pr_err("%s process failed. (%s)\n",
				MLDD_CMDTYPE2STR(cmd_type),
				MLDD_BUF_STATE_GET_STR());
		return MLDD_PROC_RES_SYS_ERR;
	}

	if (resp_size_p) {
		*resp_size_p = the_marlin.mldd_data.resp.size;
		pr_debug("resp_size=%d\n", *resp_size_p);
	}

	/* Set driver state to MLDD_STATE_ML_READY */
	down(&the_marlin.mldd_state_sem);
	MLDD_STATE_SET(MLDD_STATE_ML_READY);
	up(&the_marlin.mldd_state_sem);

	return MLDD_PROC_RES_OK;
}


/**
 * Send command data to transport and get the reponse.
 */
static enum mldd_proc_res_t mldd_raise_event(enum mltl_dd_cmd_type_t cmd_type)
{
	return mldd_raise_data(cmd_type, 0, NULL, NULL, NULL);
}


/**
 * Character Device Handler
 */
static int mldd_ioctl(struct inode *nd, struct file *fp,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	enum mldd_state_t new_state = MLDD_STATE_NONE;
	struct mltl_ioctl_resp_t ioctl_resp;

	down(&the_marlin.mldd_state_sem);

	/* Init response data */
	ioctl_resp.result = MLDD_RES_OK;
	ioctl_resp.dbs_d2h = DBS_D2H;
	ioctl_resp.dbs_h2d = DBS_H2D;
	new_state = MLDD_STATE_GET();

	pr_debug("cmd=%d, state=%s)\n", cmd, MLDD_STATE_GET_STR());
	if (cmd == USB_MARLIN_IOC_INIT) {
		pr_debug("USB_MARLIN_IOC_INIT\n");
		switch (MLDD_STATE_GET()) {
		case MLDD_STATE_NONE:
			new_state = MLDD_STATE_TL_READY;
			break;

		case MLDD_STATE_DD_READY:
			new_state = MLDD_STATE_ML_READY;
			break;

		case MLDD_STATE_TL_READY:
		case MLDD_STATE_ML_READY:
		case MLDD_STATE_ML_INPROGRESS:
			ioctl_resp.result = MLDD_RES_ERR_ALREADY_OPENNED;
			break;

		default:
			pr_err("Invalid state: %s\n", MLDD_STATE_GET_STR());
			ioctl_resp.result = MLDD_RES_ERR_INVALID_SEQ;
			break;
		}
	} else if (cmd == USB_MARLIN_IOC_FIN) {
		pr_debug("USB_MARLIN_IOC_FIN\n");
		switch (MLDD_STATE_GET()) {
		case MLDD_STATE_NONE:
		case MLDD_STATE_DD_READY:
			ioctl_resp.result = MLDD_RES_ERR_NOT_OPENNED;
			break;

		case MLDD_STATE_TL_READY:
			new_state = MLDD_STATE_NONE;
			break;

		case MLDD_STATE_ML_READY:
			new_state = MLDD_STATE_DD_READY;
			break;

		case MLDD_STATE_ML_INPROGRESS:
			new_state = MLDD_STATE_DD_READY;
			ioctl_resp.result = MLDD_RES_ERR_INVALID_SEQ;
			break;

		default:
			pr_err("Invalid state: %s\n", MLDD_STATE_GET_STR());
			ioctl_resp.result = MLDD_RES_ERR_INVALID_SEQ;
			break;
		}
	} else {
		ioctl_resp.result = MLDD_RES_ERR_INVALID_REQUEST;
		pr_err("Invalid request: %d\n", cmd);
	}

	pr_debug("Response data: result=%d, D2H=%d, H2D=%d\n",
			ioctl_resp.result, DBS_D2H, DBS_H2D);
	ret = copy_to_user((char *)arg, &ioctl_resp, sizeof(ioctl_resp));
	if (ret != 0) {
		pr_err("copy_to_user() failed. (%d)\n", ret);
		ret = -EFAULT;
	} else {
		MLDD_STATE_SET(new_state);
	}

	up(&the_marlin.mldd_state_sem);

	return ret;
}

static ssize_t mldd_read(struct file *fp, char __user *buf,
		size_t count, loff_t *pos)
{
	ssize_t read_size = 0;
	int ret;

	if (!MLDD_STATE_IS_TLREADY()) {
		pr_err("state is not TL_READY. (state: %d)\n",
				MLDD_STATE_GET());
		return -EIO;
	}

	ret = wait_event_interruptible(
			the_marlin.mldd_wq,
			MLDD_BUF_STATE_GET() == MLDD_BUF_STATE_READ_READY);
	if (ret < 0) {
		pr_err("wait_event_interruptible() failed. (%d)\n", ret);
		return ret;
	}

	/* type */
	ret = copy_to_user(buf, &the_marlin.mldd_data.cmd.cmd_type,
			sizeof(the_marlin.mldd_data.cmd.cmd_type));
	if (ret != 0) {
		pr_err("copy_to_user() failed. (%d)\n", ret);
		goto error;
	}
	read_size += sizeof(the_marlin.mldd_data.cmd.cmd_type);

	/* size */
	ret = copy_to_user(buf + read_size,
			&the_marlin.mldd_data.cmd.size,
			sizeof(the_marlin.mldd_data.cmd.size));
	if (ret != 0) {
		pr_err("copy_to_user() failed. (%d)\n", ret);
		goto error;
	}
	read_size += sizeof(the_marlin.mldd_data.cmd.size);

	/* data */
	if (the_marlin.mldd_data.cmd.size) {
		u32 *addr = (u32 *)(buf + read_size);
		int ok;

		ok = access_ok(VERIFY_WRITE, addr, sizeof(u32));
		if (!ok) {
			pr_err("access_ok() failed. (%d)\n", ok);
			goto error;
		}

		if (the_marlin.mldd_data.cmd.size > count) {
			pr_err("Too many data. (max: %d, current: %d)\n",
					count, the_marlin.mldd_data.cmd.size);
			goto error;
		}

		ret = copy_to_user((void *)(*addr),
					the_marlin.mldd_data.cmd.data_p,
					the_marlin.mldd_data.cmd.size);
		if (ret != 0) {
			pr_err("copy_to_user() failed. (%d)\n", ret);
			goto error;
		}
		read_size += the_marlin.mldd_data.cmd.size;
	}

	MLDD_BUF_STATE_SET(MLDD_BUF_STATE_WRITE_READY);

	return read_size;

error:
	MLDD_BUF_STATE_SET(MLDD_BUF_STATE_READ_ERR);
	pr_err("End. (buf state=%d)\n", MLDD_BUF_STATE_GET());
	return -EFAULT;
}


static ssize_t mldd_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *pos)
{
	int ret;
	u16 buf_size = the_marlin.mldd_data.resp.size;
	ssize_t write_size = 0;

	if (MLDD_STATE_GET() != MLDD_STATE_ML_INPROGRESS) {
		pr_err("Invalid DD state: %s)\n", MLDD_STATE_GET_STR());
		return -EIO;
	}

	/* type */
	ret = copy_from_user(&the_marlin.mldd_data.resp.res_type,
			buf, sizeof(the_marlin.mldd_data.resp.res_type));
	if (ret != 0) {
		pr_err("copy_from_user() failed. (%d)\n", ret);
		goto error;
	}
	write_size += sizeof(the_marlin.mldd_data.resp.res_type);
	pr_debug("resp.res_type=%s\n",
			MLDD_PROCRES2STR(the_marlin.mldd_data.resp.res_type));

	/* size */
	ret = copy_from_user(&the_marlin.mldd_data.resp.size,
				buf + write_size,
				sizeof(the_marlin.mldd_data.resp.size));
	if (ret != 0) {
		pr_err("copy_from_user() failed. (%d)\n", ret);
		goto error;
	}
	write_size += sizeof(the_marlin.mldd_data.resp.size);
	pr_debug("resp.size=%d\n", the_marlin.mldd_data.resp.size);

	/* data */
	if (the_marlin.mldd_data.resp.size) {
		u32 *addr = (u32 *)(buf + write_size);
		int ok;

		ok = access_ok(VERIFY_READ, addr, sizeof(u32));
		if (!ok) {
			pr_err("access_ok() failed. (%d)\n", ok);
			goto error;
		}

		if (the_marlin.mldd_data.resp.size > buf_size) {
			pr_err("Too many data. (max: %d, current: %d)\n",
					buf_size,
					the_marlin.mldd_data.resp.size);
			goto error;
		}

		ret = copy_from_user(the_marlin.mldd_data.resp.data_p,
					(void *)(*addr),
					the_marlin.mldd_data.resp.size);
		if (ret != 0) {
			pr_err("copy_from_user() failed. (%d)\n", ret);
			goto error;
		}
		write_size += the_marlin.mldd_data.resp.size;
	}

	MLDD_BUF_STATE_SET(MLDD_BUF_STATE_WRITE_COMPLETED);
	wake_up(&the_marlin.mldd_wq);

	return write_size;

error:
	MLDD_BUF_STATE_SET(MLDD_BUF_STATE_WRITE_ERR);
	wake_up(&the_marlin.mldd_wq);
	return -EFAULT;
}

static unsigned int mldd_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &the_marlin.mldd_wq, wait);
	if (MLDD_BUF_STATE_GET() == MLDD_BUF_STATE_READ_READY)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}


/* file operations needed when we register this driver */
static const struct file_operations marlin_fops = {
	.owner =	 THIS_MODULE,
	.ioctl =	 mldd_ioctl,
	.read =		 mldd_read,
	.write =	 mldd_write,
	.poll =		 mldd_poll,
};

static struct miscdevice marlin_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "marlin_usb",
	.fops = &marlin_fops,
};


/**
 * Event Handler
 */
static void mldd_do_enable(struct work_struct *work)
{
	int ret;

	/* Wait the remaining process completed. */
	ret = mldd_wait_process_completed();
	if (ret != 0) {
		pr_err("process time out\n");
		mldd_fin();
		mldd_init();
	}

	/* Update driver state to DD_READY. */
	pr_debug("DD state: %s\n", MLDD_STATE_GET_STR());
	down(&the_marlin.mldd_state_sem);
	switch (MLDD_STATE_GET()) {
	case MLDD_STATE_NONE:
		MLDD_STATE_SET(MLDD_STATE_DD_READY);
		break;

	case MLDD_STATE_TL_READY:
		MLDD_STATE_SET(MLDD_STATE_ML_READY);
		break;

	default:
		/* Do nothing */
		break;
	}
	up(&the_marlin.mldd_state_sem);
}


static void mldd_do_disable(struct work_struct *work)
{
	int ret;

	/* Wait the remaining process completed. */
	ret = mldd_wait_process_completed();
	if (ret != 0) {
		pr_err("process time out\n");
		mldd_fin();
		mldd_init();
	}

	/* Notify transport that driver is disabled */
	pr_debug("DD state: %s\n", MLDD_STATE_GET_STR());
	if (MLDD_STATE_GET() == MLDD_STATE_ML_READY) {
		mldd_raise_event(MLTL_DD_CMD_TYPE_RESETTED);
		down(&the_marlin.mldd_state_sem);
		MLDD_STATE_SET(MLDD_STATE_TL_READY);
		up(&the_marlin.mldd_state_sem);
	}
}


static void mldd_do_mount_unmount(struct work_struct *work)
{
	int ret;

	/* Wait the remaining process completed. */
	ret = mldd_wait_process_completed();
	if (ret != 0) {
		pr_err("process time out\n");
		mldd_fin();
		mldd_init();
	}

	/* Notify transport to reset it's context. */
	pr_debug("DD state: %s\n", MLDD_STATE_GET_STR());
	if (MLDD_STATE_IS_TLREADY())
		mldd_raise_event(MLTL_DD_CMD_TYPE_RESETTED);
}


/**
 * This function will be called by module_init()
 */
int mldd_init(void)
{
	int ret = 0;

	if (the_marlin.init_completed)
		return 0;

	/* Create wait queue for data exchaning. */
	init_waitqueue_head(&the_marlin.mldd_wq);

	/* Init works for interrupt handler. */
	INIT_WORK(&the_marlin.marlin_enable, mldd_do_enable);
	INIT_WORK(&the_marlin.marlin_disable, mldd_do_disable);
	INIT_WORK(&the_marlin.marlin_mount, mldd_do_mount_unmount);
	INIT_WORK(&the_marlin.marlin_unmount, mldd_do_mount_unmount);

	sema_init(&the_marlin.mldd_state_sem, 1);


	/* Register Character Device Driver. */
	ret = misc_register(&marlin_device);
	if (ret) {
		pr_err("misc_register() failed. (%d)\n", ret);
		return ret;
	}

	/* Initialize driver state to MLDD_STATE_NONE. */
	memset(&the_marlin.mldd_data, 0, sizeof(the_marlin.mldd_data));
	MLDD_STATE_SET(MLDD_STATE_NONE);

	the_marlin.init_completed = 1;

	return 0;
}

/**
 * This function will be called by module_exit()
 */
int mldd_fin(void)
{
	int ret;

	/* Wait the remaining process completed. */
	pr_debug("DD state: %s\n", MLDD_STATE_GET_STR());
	ret = mldd_wait_process_completed();
	if (ret != 0)
		pr_err("process time out\n");

	the_marlin.init_completed = 0;

	/* Update driver state to MLDD_STATE_NONE. */
	MLDD_STATE_SET(MLDD_STATE_NONE);

	/* Unregister Character Device Driver. */
	misc_deregister(&marlin_device);

	return 0;
}


/**
 * This function will be called by fsg_configure()
 */
void mldd_enable(void)
{
	schedule_work(&the_marlin.marlin_enable);
}


/**
 * This function will be called by fsg_disconnect()
 */
void mldd_disable(void)
{
	schedule_work(&the_marlin.marlin_disable);
}


/**
 * This function will be called by fsg_configure()
 */
void mldd_mount(void)
{
	schedule_work(&the_marlin.marlin_mount);
}


/**
 * This function will be called by fsg_disconnect()
 */
void mldd_unmount(void)
{
	schedule_work(&the_marlin.marlin_unmount);
}


/*****************************************************
 * Command handler
 ****************************************************/

/**
 * Get SCSI Sense Key by process result
 */
static u32 mldd_restype2ss(enum mltl_dd_proc_res_t res_type)
{

	u32 sense_code = MARLIN_SS_NO_SENSE;

	switch (res_type) {
	case MLTL_DD_PROC_RES_INVALID_CMD:
	case MLTL_DD_PROC_RES_CMD_CANCELLED:
	case MLTL_DD_PROC_RES_SYS_ERR:
	case MLTL_DD_PROC_RES_SYS_RESOURCE_FAILURE:
		sense_code = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
		pr_err("%s -> MARLIN_SS_SYSTEM_RESOURCE_FAILURE.\n",
				MLDD_PROCRES2STR(res_type));
		break;

	case MLTL_DD_PROC_RES_INVALID_CMD_OP_CODE:
		sense_code = MARLIN_SS_INVALID_COMMAND;
		pr_err("%s -> MARLIN_SS_INVALID_COMMAND.\n",
				MLDD_PROCRES2STR(res_type));
		break;

	case MLTL_DD_PROC_RES_NOT_READY:
		sense_code = MARLIN_SS_DEVICE_NOT_READY;
		pr_debug("%s -> MARLIN_SS_DEVICE_NOT_READY.\n",
				MLDD_PROCRES2STR(res_type));
		break;

	default:
		/* Do nothing */
		pr_debug("%s -> MARLIN_SS_NO_SENSE.\n",
				MLDD_PROCRES2STR(res_type));
		break;
	}

	return sense_code;
}


/**
 * REPORT_KEY command handler
 */
int mldd_do_report_key(int cmd_size, u8 *cmd_p,
		u16 buf_size, u8 *bh_buf_p, u32 *sense_code_p)
{
	u16 len = 0;
	u8 key_format;
	enum mldd_proc_res_t proc_res = MLDD_PROC_RES_OK;

	if (buf_size < 2 || cmd_p == NULL || bh_buf_p == NULL
		|| sense_code_p == NULL) {
		pr_err("Invalid parameter.\n");
		return -EINVAL;
	}

	len = buf_size - 2;

	if (cmd_p[7] != 0x6D) {
		pr_err("Invalid key class. (%d)\n", cmd_p[7]);
		*sense_code_p = MARLIN_SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	key_format = MARLIN_KEY_FORMAT(cmd_p[10]);
	pr_debug("[REPORT_KEY Command] key format: %d\n", key_format);
	*sense_code_p = MARLIN_SS_NO_SENSE;
	switch (key_format) {
	case MARLIN_KEY_FROMAT_REPORT_KEY_SIZE:
		pr_debug("[<--] [REPORT_KEY_SIZE]\n");
		proc_res = mldd_raise_data(MLTL_DD_CMD_TYPE_REPORTKEY_SIZE,
						0,
						NULL,
						&len,
						&bh_buf_p[2]);
		if (proc_res == MLDD_PROC_RES_OK) {
			pr_debug("[REPORT_KEY_SIZE] "
					"report key size=%d, len=%d\n",
					*((u32 *)&bh_buf_p[2]), len);
			*((u16 *)&bh_buf_p[0]) = cpu_to_be16(len);
			*((u32 *)&bh_buf_p[2]) =
				cpu_to_be32(*((u32 *)&bh_buf_p[2]));
			pr_debug("[-->] [REPORT_KEY_SIZE] size: %d\n",
					be32_to_cpu(*((u32 *)&bh_buf_p[2])));
			len += 2;
		} else {
			*sense_code_p = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
		}
		break;

	case MARLIN_KEY_FROMAT_REPORT_KEY_DATA:
		pr_debug("[<--] [REPORT_KEY_DATA]\n");
		proc_res = mldd_raise_data(MLTL_DD_CMD_TYPE_REPORTKEY_DATA,
						0,
						NULL,
						&len,
						&bh_buf_p[2]);
		if (proc_res == MLDD_PROC_RES_OK) {
			pr_debug("[Response REPORT_KEY_DATA] "
					"report key data size=%d\n",
					len);
			*((u16 *)&bh_buf_p[0]) = cpu_to_be16(len);
			pr_debug("[-->] [REPORT_KEY_DATA] size: %d\n", len);
			MLOG_DUM("[-->] [REPORT_KEY_DATA]", len, &bh_buf_p[2]);
			len += 2;
		} else {
			*sense_code_p = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
		}
		break;

	case MARLIN_KEY_FROMAT_REPORT_KEY_CAP:
		pr_debug("[<--][REPORT_KEY_CAP]\n");
		memset(bh_buf_p, 0, 64);

		*((u16 *)&bh_buf_p[0]) = cpu_to_be16(64);
					/* Report Key Data Length */
		bh_buf_p[4] = 1;	/* Data Format Version */
		bh_buf_p[5] = 3;	/* Marlin SCSI Command Version */
		*((u16 *)&bh_buf_p[6]) = cpu_to_be16(0);
					/* Marlin SCSI Device Type */
		*((u16 *)&bh_buf_p[8]) = cpu_to_be16(54);
					/* Device Capability Length */
		bh_buf_p[10] = 3;	/* Device Capability Field */
		len = 64;
		pr_debug("[-->] [REPORT_KEY_CAP] size: %d\n", len);
		MLOG_DUM("[-->] [REPORT_KEY_CAP]", len, bh_buf_p);
		break;

	default:
		pr_err("Invalid key format. (%d)\n", key_format);
		*sense_code_p = MARLIN_SS_INVALID_FIELD_IN_CDB;
		break;
	}

	if (*sense_code_p == MARLIN_SS_NO_SENSE)
		*sense_code_p =
			mldd_restype2ss(the_marlin.mldd_data.resp.res_type);

	pr_debug("[Response REPORT_KEY] sense code=0x%08X, data len=%d\n",
			*sense_code_p, len);
	if (*sense_code_p == MARLIN_SS_DEVICE_NOT_READY) {
		return -EINVAL;
	} else if (*sense_code_p != MARLIN_SS_NO_SENSE) {
		pr_err("Process failed. (sense code: 0x%02X)\n",
				*sense_code_p);
		return -EINVAL;
	}

	return len;
}


/**
 * SEND_KEY command handler
 */
int mldd_do_send_key(int cmd_size, u8 *cmd_p,
		u16 buf_size, u8 *bh_buf_p, u32 *sense_code_p)
{
	uint kdata_len;
	uint data_size;
	u8 key_format;
	enum mldd_proc_res_t proc_res = MLDD_PROC_RES_OK;

	if (cmd_p == NULL || bh_buf_p == NULL || sense_code_p == NULL) {
		pr_err("Invalid parameter.\n");
		return -EINVAL;
	}

	if (cmd_p[7] != 0x6D) {
		pr_err("Invalid key class. (%d)\n", cmd_p[7]);
		*sense_code_p = MARLIN_SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	key_format = MARLIN_KEY_FORMAT(cmd_p[10]);
	pr_debug("[SEND_KEY Command] key format: %d\n", key_format);
	*sense_code_p = MARLIN_SS_NO_SENSE;
	switch (key_format) {
	case MARLIN_KEY_FROMAT_SEND_KEY_SIZE:
		kdata_len = be16_to_cpu(*((u16 *)bh_buf_p));
		if ((kdata_len + 2) > buf_size) {
			pr_err("[SEND_KEY_SIZE] Invalid data. "
					"(kdata_len=%d, buf_size=%d)\n",
					 kdata_len,
					 buf_size);
			*sense_code_p = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
			break;
		}

		data_size = be32_to_cpu(*((u32 *)&bh_buf_p[2]));
		pr_debug("[<--] [SEND_KEY_SIZE] size: %d\n", data_size);
		proc_res = mldd_raise_data(MLTL_DD_CMD_TYPE_SENDKEY_SIZE,
						kdata_len,
						(u8 *)&data_size,
						NULL,
						NULL);
		if (proc_res != MLDD_PROC_RES_OK)
			*sense_code_p = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
		break;

	case MARLIN_KEY_FROMAT_SEND_KEY_DATA:
		kdata_len = be16_to_cpu(*((u16 *)bh_buf_p));
		if ((kdata_len + 2) > buf_size) {
			pr_err("[SEND_KEY_DATA] Invalid data. "
					"(kdata_len=%d, buf_size=%d)\n",
					kdata_len,
					buf_size);
			*sense_code_p = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
			break;
		}

		pr_debug("[<--] [SEND_KEY_DATA] size=%d\n", kdata_len);
		MLOG_DUM("[<--] [SEND_KEY_DATA]", kdata_len, &bh_buf_p[2]);
		proc_res = mldd_raise_data(MLTL_DD_CMD_TYPE_SENDKEY_DATA,
						kdata_len,
						&bh_buf_p[2],
						NULL,
						NULL);
		if (proc_res != MLDD_PROC_RES_OK)
			*sense_code_p = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
		break;

	case MARLIN_KEY_FROMAT_SEND_KEY_CHECK:
		pr_debug("[<--] [SEND_KEY_CHECK]\n");
		proc_res = mldd_raise_event(MLTL_DD_CMD_TYPE_SENDKEY_CHECK);
		if (proc_res == MLDD_PROC_RES_INVALID_STATE)
			*sense_code_p = MARLIN_SS_SYSTEM_RESOURCE_FAILURE;
		break;

	default:
		pr_err("[<--] Invalid key format. (%d)\n", key_format);
		*sense_code_p = MARLIN_SS_INVALID_FIELD_IN_CDB;
		break;
	}

	if (*sense_code_p == MARLIN_SS_NO_SENSE)
		*sense_code_p =
			mldd_restype2ss(the_marlin.mldd_data.resp.res_type);

	pr_debug("[-->] [SEND_KEY] sense code: 0x%08X\n", *sense_code_p);
	if (*sense_code_p == MARLIN_SS_DEVICE_NOT_READY) {
		return -EINVAL;
	} else if (*sense_code_p != MARLIN_SS_NO_SENSE) {
		pr_err("Process failed. (sense code: 0x%02X)\n",
				*sense_code_p);
		return -EINVAL;
	}

	return 0;
}

int mldd_do_mode_sense(int all_pages, int page_code,
		int subpage_code, u8 *bh_buf_p)
{
	if (bh_buf_p == NULL) {
		pr_err("Invalid parameter.\n");
		return 0;
	}

	if (!((page_code == 0x3D || all_pages) && (subpage_code == 0x00))) {
		pr_debug("Not Marlin command. (page_code: %d, "
				"subpage_code: %d)\n",
				page_code,
				subpage_code);
		return 0;
	}

	pr_debug("[<--] [MODE_SENSE]\n");
	memset(bh_buf_p, 0, 64);
	bh_buf_p[0] = 0x3D;
	bh_buf_p[1] = 0x3E;
	memcpy(&bh_buf_p[2], "MARLIN", 6);
	bh_buf_p[8] = 0x03;	/* Command Version */
	*((u16 *)&bh_buf_p[10]) = cpu_to_be16(DBS_D2H);
	*((u16 *)&bh_buf_p[12]) = cpu_to_be16(DBS_H2D);

	pr_debug("[-->] [MODE_SENSE] size: %d\n", 64);
	MLOG_DUM("[-->] [MODE_SENSE]", 64, bh_buf_p);

	return 64;
}

