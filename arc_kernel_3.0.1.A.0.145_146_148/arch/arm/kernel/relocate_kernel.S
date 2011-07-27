/*
 * relocate_kernel.S - put the kernel image in place to boot
 */

#include <asm/kexec.h>

	.globl relocate_new_kernel
relocate_new_kernel:

	mov 	r0,#0

	/*Switch Over to Supervisor mode with interrupts disabled*/
	mov     r5,#0x00d3                      @ F_BIT|I_BIT|SVC_MODE
	msr     cpsr_c,r5			@ reset CPSR
	mov     r0,#0
	mcr     p15, 0, r0, c8, c7, 0		@ invalidate I + D TLBs
	mrc     p15, 0, r5, c1, c0, 0		@ Read ctrl register
	bic     r5, r5, #0x0086			@ Disable I,D-Cache,
					@ Branch Prediction, MMU Protection
	bic     r5, r5, #0x1900
	mcr     p15, 0, r5, c1, c0, 0

	bl      __cache_flush

	/* Jump to relocated kernel */
	ldr 	r1,kexec_mach_type
	ldr 	r2,kexec_boot_atags
	ldr     pc,kexec_start_address

#ifdef CONFIG_CPU_V6
__cache_flush:
        mov     r1, #0
        mcr     p15, 0, r1, c7, c14, 0  @ clean+invalidate D
        mcr     p15, 0, r1, c7, c5, 0   @ invalidate I+BTB
        mcr     p15, 0, r1, c7, c15, 0  @ clean+invalidate unified
        mcr     p15, 0, r1, c7, c10, 4  @ drain WB
        mov     pc, lr
        mov     pc, lr
#endif

#ifdef CONFIG_CPU_V7
__cache_flush:
        mrc     p15, 0, r10, c0, c1, 5  @ read ID_MMFR1
        tst     r10, #0xf << 16         @ hierarchical cache (ARMv7)
        beq     hierarchical
        mov     r10, #0
        mcr     p15, 0, r10, c7, c14, 0 @ clean+invalidate D
        b       iflush
hierarchical:
        stmfd   sp!, {r0-r5, r7, r9-r11}
        mrc     p15, 1, r0, c0, c0, 1   @ read clidr
        ands    r3, r0, #0x7000000      @ extract loc from clidr
        mov     r3, r3, lsr #23         @ left align loc bit field
        beq     finished                @ if loc is 0, then no need to clean
        mov     r10, #0                 @ start clean at cache level 0
loop1:
        add     r2, r10, r10, lsr #1    @ work out 3x current cache level
        mov     r1, r0, lsr r2          @ extract cache type bits from clidr
        and     r1, r1, #7              @ maskof the bits for current cache only
        cmp     r1, #2                  @ see what cache we have at this level
        blt     skip                    @ skip if no cache, or just i-cache
        mcr     p15, 2, r10, c0, c0, 0  @ select current cache level in cssr
        mcr     p15, 0, r10, c7, c5, 4  @ isb to sych the new cssr&csidr
        mrc     p15, 1, r1, c0, c0, 0   @ read the new csidr
        and     r2, r1, #7              @ extract the length of the cache lines
        add     r2, r2, #4              @ add 4 (line length offset)
        /*ldr   r4, =0x3ff*/
        mov     r4, #0
        add     r4, r4, #0x0300
        add     r4, r4, #0x00ff
        ands    r4, r4, r1, lsr #3      @ find maximum number on the way size
        .word   0xe16f5f14              @ clz r5, r4 -
					@ find bit position of waysize increment
        /*ldr   r7, =0x7fff*/
        mov     r7, #0
        add     r7, r7, #0x7f00
        add     r7, r7, #0x00ff
        ands    r7, r7, r1, lsr #13     @ extract max number of the index size
loop2:
        mov     r9, r4                  @ create working copy of max way size
loop3:
        orr     r11, r10, r9, lsl r5    @ factor way and cache number into r11
        orr     r11, r11, r7, lsl r2    @ factor index number into r11
        mcr     p15, 0, r11, c7, c14, 2 @ clean & invalidate by set/way
        subs    r9, r9, #1              @ decrement the way
        bge     loop3
        subs    r7, r7, #1              @ decrement the index
        bge     loop2
skip:
        add     r10, r10, #2            @ increment cache number
        cmp     r3, r10
        bgt     loop1
finished:
        mov     r10, #0                 @ swith back to cache level 0
        mcr     p15, 2, r10, c0, c0, 0  @ select current cache level in cssr
        ldmfd   sp!, {r0-r5, r7, r9-r11}
iflush:
        mcr     p15, 0, r10, c7, c5, 0  @ invalidate I+BTB
        mcr     p15, 0, r10, c7, c10, 4 @ drain WB
        mov     pc, lr

#endif

	.globl kexec_start_address
kexec_start_address:
	.long	0x0

	.globl kexec_indirection_page
kexec_indirection_page:
	.long	0x0

	.globl kexec_mach_type
kexec_mach_type:
	.long	0x0

	/* phy addr of the atags for the new kernel */
	.globl kexec_boot_atags
kexec_boot_atags:
	.long	0x0

relocate_new_kernel_end:

	.globl relocate_new_kernel_size
relocate_new_kernel_size:
	.long relocate_new_kernel_end - relocate_new_kernel


