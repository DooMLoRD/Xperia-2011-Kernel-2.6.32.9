/*
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: minghui.wang <minghui.wang@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/console.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#define MAX_LEN 4

unsigned long console_value = 1;
EXPORT_SYMBOL(console_value);

static struct proc_dir_entry  *console_control_file;
static DEFINE_MUTEX(console_lock);

static int console_control_read(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len;

	mutex_lock(&console_lock);
	len = snprintf(page, count, "%lu\n", console_value);
	mutex_unlock(&console_lock);
	if (len >= count)
		return count;
	return len + 1;
}

static int console_control_write(struct file *file, const char *buffer,
			     unsigned long count, void *data)
{
	unsigned long len = MAX_LEN;
	unsigned long set_console_value;
	struct console *con;
	int ret;
	char kbuf[MAX_LEN + 1];

	if (count < MAX_LEN)
		len = count;

	ret = copy_from_user(kbuf, buffer, len);
	if (ret) {
		ret = -EFAULT;
		goto error;
	}
	kbuf[len] = '\0';

	ret = strict_strtoul(kbuf, 0, &set_console_value);
	if (ret) {
		printk(KERN_ERR "strict_strtoul failed!\n");
		ret = -EINVAL;
		goto error;
	}

	/*
	 * 0 - disable console except for ram_console
	 * 1 - enable console except for ram_console
	 */
	if (set_console_value == 0) {
		acquire_console_sem();
		for (con = console_drivers; con != NULL; con = con->next) {
			if (strcmp(con->name, "ram")) {
				con->flags = con->flags & ~CON_ENABLED;
				console_value = 0;
			}
		}
		release_console_sem();
	} else if (set_console_value == 1) {
		acquire_console_sem();
		for (con = console_drivers; con != NULL; con = con->next) {
			if (strcmp(con->name, "ram")) {
				con->flags = con->flags | CON_ENABLED;
				console_value = 1;
			}
		}
		release_console_sem();
	} else {
		ret = -EINVAL;
		goto error;
	}
	return len;

error:
	return ret;
}

static int __init init_console_control(void)
{
	console_control_file = create_proc_entry("console_control", 0644, NULL);
	if (console_control_file == NULL) {
		printk(KERN_ERR "console_control proc create failed!\n");
		return -ENOMEM;
	}
	console_control_file->read_proc = console_control_read;
	console_control_file->write_proc = console_control_write;

	return 0;
}

static void __exit cleanup_console_control(void)
{
	remove_proc_entry("console_control", NULL);
}

module_init(init_console_control);
module_exit(cleanup_console_control);

MODULE_AUTHOR("SonyEricsson");
MODULE_DESCRIPTION("console_control");
MODULE_LICENSE("GPL");
