/* linux/arch/arm/mach-msm/last_amsslog.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Christian Lindeberg <christian.lindeberg@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include "smd_private.h"

#define LOG_SIZE_MAX 4096

static char *last_amsslog;
static ssize_t last_amsslog_size;

static ssize_t last_amsslog_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos >= last_amsslog_size)
		return 0;

	count = min(len, (size_t)(last_amsslog_size - pos));
	if (copy_to_user(buf, last_amsslog + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations last_amsslog_file_ops = {
	.read = last_amsslog_read,
};

static int create_last_amsslog_proc_entry(void)
{
	struct proc_dir_entry *entry;
	char *log;
	unsigned log_size;

	log = smem_get_entry(SMEM_ERR_CRASH_LOG, &log_size);
	if (!log) {
		printk(KERN_ERR "last_amsslog: Log entry not found\n");
		return 0;
	}
	if (log[0] == '\0') {
		printk(KERN_INFO "last_amsslog: Log not present\n");
		return 0;
	}

	last_amsslog_size = (log_size > LOG_SIZE_MAX) ? LOG_SIZE_MAX : log_size;
	last_amsslog = kmalloc(last_amsslog_size, GFP_KERNEL);
	if (last_amsslog == NULL) {
		printk(KERN_ERR "last_amsslog: Failed to allocate buffer\n");
		return 0;
	}

	memcpy(last_amsslog, log, last_amsslog_size);

	entry = create_proc_entry("last_amsslog", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "last_amsslog: Failed to create proc entry\n");
		kfree(last_amsslog);
		last_amsslog = NULL;
		return 0;
	}

	entry->proc_fops = &last_amsslog_file_ops;
	entry->size = last_amsslog_size;
	return 0;
}

static int last_amsslog_init(void)
{
	create_last_amsslog_proc_entry();
	return 0;
}

MODULE_AUTHOR("Christian Lindeberg christian.lindeberg@sonyericsson.com");
MODULE_DESCRIPTION("AMSS error log");
MODULE_LICENSE("GPL");

module_init(last_amsslog_init);
