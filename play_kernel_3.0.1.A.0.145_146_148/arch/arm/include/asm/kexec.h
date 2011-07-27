#ifndef _ARM_KEXEC_H
#define _ARM_KEXEC_H

#ifdef CONFIG_KEXEC

/* Maximum physical address we can use pages from */
#define KEXEC_SOURCE_MEMORY_LIMIT (-1UL)
/* Maximum address we can reach in physical address mode */
#define KEXEC_DESTINATION_MEMORY_LIMIT (-1UL)
/* Maximum address we can use for the control code buffer */
#define KEXEC_CONTROL_MEMORY_LIMIT (-1UL)

#define KEXEC_CONTROL_PAGE_SIZE	4096

#define KEXEC_ARCH KEXEC_ARCH_ARM

#define KEXEC_ARM_ATAGS_OFFSET  0x1000
#define KEXEC_ARM_ZIMAGE_OFFSET 0x8000

#ifndef __ASSEMBLY__

struct kimage;
/* Provide a dummy definition to avoid build failures. */
static inline void crash_setup_regs(struct pt_regs *newregs,
					struct pt_regs *oldregs)
{
	if (oldregs) {
		memcpy(newregs, oldregs, sizeof(*newregs));
	} else {
	/* dump critical general registers first */
		__asm__ __volatile__("str fp, %0"
				: "=m"(newregs->ARM_fp));
		__asm__ __volatile__("str sp, %0"
				: "=m"(newregs->ARM_sp));
		__asm__ __volatile__("str pc, %0"
				: "=m"(newregs->ARM_pc));
		__asm__ __volatile__("str lr, %0"
				: "=m"(newregs->ARM_lr));
			/* dump general registers that will be used later */
		__asm__ __volatile__("str r0, %0"
				: "=m"(newregs->ARM_r0));
		__asm__ __volatile__("str r1, %0"
				: "=m"(newregs->ARM_r1));
		__asm__ __volatile__("str r2, %0"
				: "=m"(newregs->ARM_r2));
		__asm__ __volatile__("str r3, %0"
				: "=m"(newregs->ARM_r3));
		__asm__ __volatile__("str r4, %0"
				: "=m"(newregs->ARM_r4));
		__asm__ __volatile__("str r5, %0"
				: "=m"(newregs->ARM_r5));
		__asm__ __volatile__("str r6, %0"
				: "=m"(newregs->ARM_r6));
		__asm__ __volatile__("str r7, %0"
				: "=m"(newregs->ARM_r7));
		__asm__ __volatile__("str r8, %0"
				: "=m"(newregs->ARM_r8));
		__asm__ __volatile__("str r9, %0"
				: "=m"(newregs->ARM_r9));
		__asm__ __volatile__("str r10, %0"
				: "=m"(newregs->ARM_r10));
		__asm__ __volatile__("str ip, %0"
				: "=m"(newregs->ARM_ip));
	/* The registers involved with processor states and cp states
	* will not be changed in the above operation, so it is safe
	* to dump them at last
	*/
	/* dump cpsr register */
		__asm__ __volatile__("mrs %0, cpsr"
				: "=r"(newregs->ARM_cpsr));
	}

}

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_KEXEC */

#endif /* _ARM_KEXEC_H */
