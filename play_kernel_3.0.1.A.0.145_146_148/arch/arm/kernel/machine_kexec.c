/*
 * machine_kexec.c - handle transition of Linux booting another kernel
 */

#include <linux/mm.h>
#include <linux/kexec.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/mmu_context.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#include <asm/setup.h>
#include "../mach-msm/smd_private.h"

extern const unsigned char relocate_new_kernel[];
extern const unsigned int relocate_new_kernel_size;

extern void setup_mm_for_reboot(char mode);

extern unsigned long kexec_start_address;
extern unsigned long kexec_indirection_page;
extern unsigned long kexec_mach_type;
extern unsigned long kexec_boot_atags;

/*
 * Provide a dummy crash_notes definition while crash dump arrives to arm.
 * This prevents breakage of crash_notes attribute in kernel/ksysfs.c.
 */

int machine_kexec_prepare(struct kimage *image)
{
	return 0;
}

void machine_kexec_cleanup(struct kimage *image)
{
}

void machine_shutdown(void)
{
}

void machine_crash_shutdown(struct pt_regs *regs)
{
	/* The kernel is broken so disable interrupts */
	local_irq_disable();
	crash_save_cpu(regs, 0);
#ifdef CONFIG_CACHE_L2X0
	l2x0_suspend();
#endif
	smsm_notify_apps_crashdump();
}

void setup_mm_for_kdump(char mode)
{
	unsigned long base_pmdval;
	pgd_t *pgd;
	int i;

	pgd = init_mm.pgd;

	cpu_switch_mm(pgd, &init_mm);

	base_pmdval = PMD_SECT_AP_WRITE | PMD_SECT_AP_READ | PMD_TYPE_SECT;

	for (i = 0; i < FIRST_USER_PGD_NR + USER_PTRS_PER_PGD; i++, pgd++) {
		unsigned long pmdval = (i << PGDIR_SHIFT) | base_pmdval;
		pmd_t *pmd;

		pmd = pmd_offset(pgd, i << PGDIR_SHIFT);
		pmd[0] = __pmd(pmdval);
		pmd[1] = __pmd(pmdval + (1 << (PGDIR_SHIFT - 1)));
		flush_pmd_entry(pmd);
	}
}

static void append_crash_params_cmdline(void)
{
	struct tag *tag = (struct tag *)kexec_boot_atags;
	char crashtime[32];
	unsigned int size;
	int remaining_size, required_size;

	do {
		if (tag->hdr.tag == ATAG_CMDLINE)
			break;
		else
			tag = tag_next(tag);

	} while (tag->hdr.size > 0);

	sprintf(crashtime, " crashtime=%lu", get_seconds());
	/* Modify command line only if there is space left */
	remaining_size = COMMAND_LINE_SIZE - strlen(tag->u.cmdline.cmdline);
	required_size = strlen(crashtime);
	if (unlikely(remaining_size < required_size))
		return;
	strncat(tag->u.cmdline.cmdline, crashtime, sizeof(crashtime));
	size = (strlen(tag->u.cmdline.cmdline) + sizeof(struct tag_header));
	tag->hdr.size = ALIGN(size, sizeof(unsigned int))/sizeof(unsigned int);
	tag = tag_next(tag);
	tag->hdr.tag = ATAG_NONE;
	tag->hdr.size = 0;
}

void machine_kexec(struct kimage *image)
{
	unsigned long page_list;
	unsigned long reboot_code_buffer_phys;
	void *reboot_code_buffer;


	page_list = image->head & PAGE_MASK;

	/* we need both effective and real address here */
	reboot_code_buffer_phys =
	    page_to_pfn(image->control_code_page) << PAGE_SHIFT;
	reboot_code_buffer = page_address(image->control_code_page);

	/* Prepare parameters for reboot_code_buffer*/
	kexec_start_address = image->start;
	kexec_indirection_page = page_list;
	kexec_mach_type = machine_arch_type;
	kexec_boot_atags = image->start - KEXEC_ARM_ZIMAGE_OFFSET + KEXEC_ARM_ATAGS_OFFSET;

	/* copy our kernel relocation code to the control code page */
	memcpy(reboot_code_buffer,
	       relocate_new_kernel, relocate_new_kernel_size);


	flush_icache_range((unsigned long) reboot_code_buffer,
			   (unsigned long) reboot_code_buffer + KEXEC_CONTROL_PAGE_SIZE);
	printk(KERN_INFO "Bye!\n");

	cpu_proc_fin();
	setup_mm_for_kdump(0); /* mode is not used, so just pass 0*/
	append_crash_params_cmdline();
	cpu_reset(reboot_code_buffer_phys);
}
