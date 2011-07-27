/*
   Read MSM AMSS RAM from KDUMP Capture Kernel

   Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License, version 2, as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/elf.h>
#include <../../arch/arm/mach-msm/smd_private.h>
#include "../../arch/arm/mach-msm/proc_comm.h"
#include "../../arch/arm/mach-msm/include/mach/msm_iomap.h"
#define PCOM_KDUMP_EXTENSION_START 0x20000000
#define VIBRATION_ON	2500
#define VIBRATION_OFF	0

/* enum extension of proc_comm.h for capk */
enum {
	PCOM_KDUMP_VIBRATION = PCOM_KDUMP_EXTENSION_START,
};

#define AMSSCORE_NAME		"amsscore"
#define SMEMCORE_NAME		"smemcore"
#define ADSPCORE_NAME		"adspcore"

struct memory_range {
	unsigned long start;
	unsigned long end;
	unsigned long size;
	unsigned long offset;
};

struct elfheader_info {
	void *hdr;
	unsigned long header_len;
};

struct coredump_info {
	const char *name;
	ssize_t total_size; /* A sum of elfhdr_size and total_range_size. */
	ssize_t elfhdr_size; /* ELF header size. */
	ssize_t total_range_size; /* Total size of memory ranges. */
	unsigned int num_ranges;
	struct memory_range *ranges;
	const struct file_operations *fops;
	struct elfheader_info *elfinfo;
};

enum {
	AMSSCORE = 0,
	SMEMCORE,
	ADSPCORE,
	NUM_ENTRIES,
};

static const struct file_operations proc_amsscore_operations;
static const struct file_operations proc_smemcore_operations;
static const struct file_operations proc_adspcore_operations;

static struct coredump_info coredumpinfo[NUM_ENTRIES] = {
	{"amsscore", 0, 0, 0, 0, NULL, &proc_amsscore_operations, NULL},
	{"smemcore", 0, 0, 0, 0, NULL, &proc_smemcore_operations, NULL},
	{"adspcore", 0, 0, 0, 0, NULL, &proc_adspcore_operations, NULL},
};

Elf32_Ehdr common_ehdr = {
	.e_ident     = {[EI_MAG0]    = ELFMAG0,
			[EI_MAG1]    = ELFMAG1,
			[EI_MAG2]    = ELFMAG2,
			[EI_MAG3]    = ELFMAG3,
			[EI_CLASS]   = ELFCLASS32,
			[EI_DATA]    = ELFDATA2LSB,
			[EI_VERSION] = EV_CURRENT,
			[EI_OSABI]   = ELFOSABI_NONE },
	.e_type      = ET_CORE,
	.e_machine   = EM_ARM,
	.e_version   = EV_CURRENT,
	.e_phoff     = sizeof(Elf32_Ehdr),
	.e_ehsize    = sizeof(Elf32_Ehdr),
	.e_phentsize = sizeof(Elf32_Phdr),
};

Elf32_Phdr common_phdr = {
	.p_type   = PT_LOAD,
	.p_flags  = (PF_R | PF_W | PF_X)
};

static int map_offset_to_range_addr(loff_t offset, size_t *bytes_read,
			struct coredump_info *dumpinfo, unsigned long *start)
{
	int i;
	struct memory_range *range = NULL;
	size_t range_offset, range_remain, acc_size = 0;

	if (unlikely(!dumpinfo || (offset >= dumpinfo->total_size)))
		return -1;

	acc_size = dumpinfo->elfhdr_size;

	for (i = 0; i < dumpinfo->num_ranges; i++) {
		acc_size += dumpinfo->ranges[i].size;
		if (offset < acc_size) {
			range = &dumpinfo->ranges[i];
			break;
		}
	}

	if (range == NULL)
		return -1;

	range_remain = acc_size - offset;
	range_offset = range->size - range_remain;
	if (bytes_read && (*bytes_read > range_remain))
		*bytes_read = range_remain;

	*start = range->start + range_offset;
	return 0;
}

static ssize_t read_elfheader(struct file *file, char __user *buffer,
			      size_t buflen, loff_t *fpos,
			      struct elfheader_info *elfinfo)
{
	size_t remaining_bytes = 0;

	if (buflen == 0 || *fpos >= elfinfo->header_len)
		return 0;

	if (buflen > elfinfo->header_len)
		buflen = elfinfo->header_len;

	remaining_bytes = elfinfo->header_len - *fpos;

	buflen = (buflen <= remaining_bytes) ? buflen : remaining_bytes;

	if (copy_to_user(buffer, elfinfo->hdr + *fpos, buflen))
		return -EFAULT;

	return buflen;
}

static ssize_t read_core(struct file *file, char __user *buffer,
		size_t buflen, loff_t *fpos, struct coredump_info *dumpinfo)
{
	unsigned long start = 0;
	void *vaddr;
	size_t bytes_read = 0;
	size_t acc_bytes = 0;
	size_t remaining_bytes = 0;
	int res;

	if (buflen == 0 || *fpos >= dumpinfo->total_size)
		return 0;

	remaining_bytes = dumpinfo->total_size - *fpos;
	/* trim buflen to not to go beyond EOF */
	if (buflen > remaining_bytes)
		buflen = remaining_bytes;

	bytes_read = min(remaining_bytes, (size_t)PAGE_SIZE);
	while (buflen) {
		res = map_offset_to_range_addr(*fpos, &bytes_read,
						dumpinfo, &start);
		if (res < 0) {
			printk(KERN_WARNING "Invalid range addr %lx\n", start);
			return -EFAULT;
		}

		vaddr = ioremap(start, PAGE_SIZE);
		if (vaddr == NULL) {
			printk(KERN_WARNING "Error in ioremaping %lx\n", start);
			return -EFAULT;
		}

		if (copy_to_user(buffer, vaddr, bytes_read)) {
			iounmap(vaddr);
			printk(KERN_WARNING "Error in copy_to_user \n");
			return -EFAULT;
		}

		iounmap(vaddr);
		buflen -= bytes_read;
		*fpos += bytes_read;
		buffer += bytes_read;
		acc_bytes += bytes_read;
		remaining_bytes -= bytes_read;
		bytes_read = min(remaining_bytes, (size_t)PAGE_SIZE);
	}
	return acc_bytes;
}

static ssize_t read_amsscore(struct file *file, char __user *buffer,
				size_t buflen, loff_t *fpos)
{
	ssize_t ret;

	if (*fpos < (coredumpinfo[AMSSCORE].elfinfo)->header_len) {
		ret = read_elfheader(file, buffer, buflen, fpos,
				     coredumpinfo[AMSSCORE].elfinfo);
		if (ret < 0)
			return ret;

		*fpos += ret;
		return ret;
	}

	ret = read_core(file, buffer, buflen, fpos, &coredumpinfo[AMSSCORE]);
	return ret;
}

static ssize_t read_smemcore(struct file *file, char __user *buffer,
				size_t buflen, loff_t *fpos)
{
	ssize_t ret;

	if (*fpos < (coredumpinfo[SMEMCORE].elfinfo)->header_len) {
		ret = read_elfheader(file, buffer, buflen, fpos,
				     coredumpinfo[SMEMCORE].elfinfo);
		if (ret < 0)
			return ret;

		*fpos += ret;
		return ret;
	}

	ret = read_core(file, buffer, buflen, fpos, &coredumpinfo[SMEMCORE]);
	return ret;
}

static ssize_t read_adspcore(struct file *file, char __user *buffer,
				size_t buflen, loff_t *fpos)
{
	ssize_t ret = 0;

	if (*fpos < (coredumpinfo[ADSPCORE].elfinfo)->header_len) {
		ret = read_elfheader(file, buffer, buflen, fpos,
				     coredumpinfo[ADSPCORE].elfinfo);
		if (ret < 0)
			return ret;

		*fpos += ret;
		return ret;
	}

	ret = read_core(file, buffer, buflen, fpos, &coredumpinfo[ADSPCORE]);
	return ret;
}

static int create_elfheader(struct coredump_info *dumpinfo,
			    struct elfheader_info **elfinfo)
{
	int i;
	Elf32_Ehdr   ehdr = common_ehdr;
	Elf32_Phdr   phdr = common_phdr;
	Elf32_Phdr  *p_phdr = NULL;
	unsigned int total_offset = 0;

	/* Alocate memory regions. */
	*elfinfo = kmalloc(sizeof(struct elfheader_info), GFP_KERNEL);
	if (*elfinfo == NULL)
		return -ENOMEM;

	(*elfinfo)->header_len
		= sizeof(Elf32_Ehdr)
		+ sizeof(Elf32_Phdr) * dumpinfo->num_ranges;

	(*elfinfo)->hdr = kmalloc((*elfinfo)->header_len, GFP_KERNEL);
	if (!(*elfinfo)->hdr) {
		kfree(*elfinfo);
		return -ENOMEM;
	}

	/* Set the number of program headers. */
	ehdr.e_phnum = dumpinfo->num_ranges;

	/* Copy an ELF header to buffer. */
	memcpy((*elfinfo)->hdr, &ehdr, sizeof(Elf32_Ehdr));

	/* Copy program headers buffer. */
	total_offset = ehdr.e_phoff + ehdr.e_phentsize * ehdr.e_phnum;

	for (i = 0, p_phdr = (*elfinfo)->hdr + sizeof(Elf32_Ehdr);
	     i < dumpinfo->num_ranges; i++, p_phdr++) {
		memcpy(p_phdr, &phdr, sizeof(Elf32_Phdr));
		p_phdr->p_offset = total_offset;
		p_phdr->p_vaddr  = dumpinfo->ranges[i].start;
		p_phdr->p_paddr  = dumpinfo->ranges[i].start;
		p_phdr->p_filesz = dumpinfo->ranges[i].size;
		p_phdr->p_memsz  = dumpinfo->ranges[i].size;

		total_offset += p_phdr->p_memsz;
	}

	return 0;
}

static void remove_elfheader(struct elfheader_info **elfinfo)
{
	if (elfinfo == NULL || *elfinfo == NULL)
		return;

	kfree((*elfinfo)->hdr);
	kfree(*elfinfo);
}

static ssize_t write_vibrator(struct file *file, const char __user *buffer,
				size_t buflen, loff_t *fops)
{
	int volt = VIBRATION_OFF;
	if (buflen) {
		char c;
		if (get_user(c, buffer))
			return -EFAULT;
		if (c == '1')
			volt = VIBRATION_ON;
		else
			volt = VIBRATION_OFF;
	}
	return msm_proc_comm(PCOM_KDUMP_VIBRATION, &volt, NULL);
}

static const struct file_operations proc_amsscore_operations = {
	.read		= read_amsscore,
};

static const struct file_operations proc_smemcore_operations = {
	.read		= read_smemcore,
};

static const struct file_operations proc_adspcore_operations = {
	.read		= read_adspcore,
};

static const struct file_operations proc_vibrator_operations = {
	.write		= write_vibrator,
};

static void setup_memory_range(struct coredump_info *dumpinfo,
			       struct resource *res, unsigned int idx)
{
	dumpinfo->ranges[idx].start = res->start;
	dumpinfo->ranges[idx].end   = res->end;
	dumpinfo->ranges[idx].size  = res->end - res->start + 1;
}

static int core_dumpinfo_init(struct platform_device *pdev,
				struct coredump_info *dumpinfo)
{
	int i;
	unsigned int index = 0;
	struct resource *res;
	struct proc_dir_entry *proc_entry;

	/* scan resources and get the number of ranges. */
	for (i = 0; i < pdev->num_resources; i++) {
		if (strncmp(pdev->resource[i].name, dumpinfo->name,
					strlen(dumpinfo->name)) == 0) {
			dumpinfo->num_ranges++;
		}
	}

	/* Allocate memory regions for memory ranges. */
	dumpinfo->ranges
		= kmalloc(sizeof(struct memory_range) * dumpinfo->num_ranges,
					GFP_KERNEL);
	if (!dumpinfo->ranges)
		return -ENOMEM;

	/* Construct memory range data structure. */
	for (i = 0, res = &pdev->resource[0];
		i < pdev->num_resources; i++, res++) {
		if (strncmp(res->name, dumpinfo->name,
			    strlen(dumpinfo->name)) == 0) {
			setup_memory_range(dumpinfo, res, index);
			dumpinfo->total_range_size
				+= dumpinfo->ranges[index].size;
			index++;
		}
	}
	dumpinfo->elfhdr_size = sizeof(Elf32_Ehdr)
		+ sizeof(Elf32_Phdr) * dumpinfo->num_ranges;
	dumpinfo->total_size
		+= dumpinfo->elfhdr_size + dumpinfo->total_range_size;

	proc_entry = proc_create(dumpinfo->name, S_IRUSR, NULL,
					dumpinfo->fops);
	if (!proc_entry) {
		kfree(dumpinfo->ranges);
		return -ENOMEM;
	}

	proc_entry->size = dumpinfo->total_size;
	return 0;
}

static int create_misc_proc_entries(void)
{
	proc_create("vibrator", S_IRUSR, NULL,
			&proc_vibrator_operations);
	return 0;
}

static int kdump_amsscoredump_driver_probe(struct platform_device *pdev)
{
	int i, ret;

	for (i = 0; i < NUM_ENTRIES; i++) {
		ret = core_dumpinfo_init(pdev, &coredumpinfo[i]);
		if (ret < 0) {
			printk(KERN_ERR "KDUMP: %s init failed\n",
						coredumpinfo[i].name);
			continue;
		}

		ret = create_elfheader(&coredumpinfo[i],
				       &(coredumpinfo[i].elfinfo));
		if (ret < 0) {
			printk(KERN_ERR "Failed creating %s elf header"
			       " information\n", coredumpinfo[i].name);
			return ret;
		}
	}

	ret = create_misc_proc_entries();
	if (ret < 0) {
		printk(KERN_ERR "Failed creating misc proc entries\n");
		return ret;
	}

	return 0;
}

static int kdump_amsscoredump_driver_remove(struct platform_device *pdev)
{
	int i;

	/* Free allocated memory regions. */
	for (i = 0; i < NUM_ENTRIES; i++) {
		kfree(coredumpinfo[i].ranges);
		remove_elfheader(&(coredumpinfo[i].elfinfo));
	}

	return 0;
}

static void kdump_amss_comm_exit(void)
{
	printk(KERN_INFO "Exit of kdump amss comm Module\n");
}

static struct platform_driver amsscoredump_driver = {
	.probe  = kdump_amsscoredump_driver_probe,
	.remove = __devexit_p(kdump_amsscoredump_driver_remove),
	.driver = {
		.name = "amsscoredump"
	},
};

static int kdump_amss_comm_init(void)
{
	int err;

	err = platform_driver_register(&amsscoredump_driver);
	if (err) {
		printk(KERN_ERR "KDUMP: Error registering platform driver\n");
		return err;
	}

	printk(KERN_INFO "KDUMP msm_amss_coredump module Initialized\n");
	return 0;
}

MODULE_AUTHOR("Vivek Pallantla vivek.pallantla@sonyericsson.com");
MODULE_DESCRIPTION("MSM AMSS Ram DUMP driver");
MODULE_LICENSE("GPL");

module_init(kdump_amss_comm_init);
module_exit(kdump_amss_comm_exit);
