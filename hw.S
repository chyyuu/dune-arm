#include "trap.h"
.text
.align 4

/*
 *	v7_flush_dcache_all()
 *
 *	Flush the whole D-cache.
 *
 *	Corrupted registers: r0-r7, r9-r11 (r6 only in Thumb mode)
 *
 *	- mm    - mm_struct describing address space
 */
.global v7_flush_dcache_all
v7_flush_dcache_all:
	dmb					@ ensure ordering with previous memory accesses
	mrc	p15, 1, r0, c0, c0, 1		@ read clidr
	ands	r3, r0, #0x7000000		@ extract loc from clidr
	mov	r3, r3, lsr #23			@ left align loc bit field
	beq	finished			@ if loc is 0, then no need to clean
	mov	r10, #0				@ start clean at cache level 0
flush_levels:
	add	r2, r10, r10, lsr #1		@ work out 3x current cache level
	mov	r1, r0, lsr r2			@ extract cache type bits from clidr
	and	r1, r1, #7			@ mask of the bits for current cache only
	cmp	r1, #2				@ see what cache we have at this level
	blt	skip				@ skip if no cache, or just i-cache
#ifdef CONFIG_PREEMPT
	save_and_disable_irqs_notrace r9	@ make cssr&csidr read atomic
#endif
	mcr	p15, 2, r10, c0, c0, 0		@ select current cache level in cssr
	isb					@ isb to sych the new cssr&csidr
	mrc	p15, 1, r1, c0, c0, 0		@ read the new csidr
#ifdef CONFIG_PREEMPT
	restore_irqs_notrace r9
#endif
	and	r2, r1, #7			@ extract the length of the cache lines
	add	r2, r2, #4			@ add 4 (line length offset)
	ldr	r4, =0x3ff
	ands	r4, r4, r1, lsr #3		@ find maximum number on the way size
	clz	r5, r4				@ find bit position of way size increment
	ldr	r7, =0x7fff
	ands	r7, r7, r1, lsr #13		@ extract max number of the index size
loop1:
	mov	r9, r7				@ create working copy of max index
loop2:
 	orr	r11, r10, r4, lsl r5		@ factor way and cache number into r11
 	orr	r11, r11, r9, lsl r2		@ factor index number into r11
	mcr	p15, 0, r11, c7, c14, 2		@ clean & invalidate by set/way
	subs	r9, r9, #1			@ decrement the index
	bge	loop2
	subs	r4, r4, #1			@ decrement the way
	bge	loop1
skip:
	add	r10, r10, #2			@ increment cache number
	cmp	r3, r10
	bgt	flush_levels
finished:
	mov	r10, #0				@ swith back to cache level 0
	mcr	p15, 2, r10, c0, c0, 0		@ select current cache level in cssr
	dsb
	isb
	mov	pc, lr

.global v7_flush_kern_cache_all
v7_flush_kern_cache_all:
 	stmfd	sp!, {r4-r5, r7, r9-r11, lr}
	bl	v7_flush_dcache_all
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0	@ I+BTB cache invalidate
 	ldmfd	sp!, {r4-r5, r7, r9-r11, lr}
	mov	pc, lr


/*
 * dcache_line_size - get the minimum D-cache line size from the CTR register
 * on ARMv7.
 */
	.macro	dcache_line_size, reg, tmp
	mrc	p15, 0, \tmp, c0, c0, 1		@ read ctr
	lsr	\tmp, \tmp, #16
	and	\tmp, \tmp, #0xf		@ cache line size encoding
	mov	\reg, #4			@ bytes per word
	mov	\reg, \reg, lsl \tmp		@ actual cache line size
	.endm


/*
 *	v7_flush_kern_dcache_area(void *addr, size_t size)
 *
 *	Ensure that the data held in the page kaddr is written back
 *	to the page in question.
 *
 *	- addr	- kernel address
 *	- size	- region size
 */
.global v7_flush_kern_dcache_area
v7_flush_kern_dcache_area:
	dcache_line_size r2, r3
	add	r1, r0, r1
	sub	r3, r2, #1
	bic	r0, r0, r3
1:
	mcr	p15, 0, r0, c7, c14, 1		@ clean & invalidate D line / unified line
	add	r0, r0, r2
	cmp	r0, r1
	blo	1b
	dsb
	mov	pc, lr

.global v7_mmu_cache_on
v7_mmu_cache_on:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4	@ drain write buffer
	tst	r11, #0xf		@ VMSA
	mcrne	p15, 0, r0, c8, c7, 0	@ flush I,D TLBs

	mrc	p15, 0, r0, c1, c0, 0	@ read control reg
	bic	r0, r0, #1 << 28	@ clear SCTLR.TRE
	orr	r0, r0, #0x5000		@ I-cache enable, RR cache replacement
	orr	r0, r0, #1 << 11	@ Branch prediction
	orr	r0, r0, #0x003c		@ write buffer
	bic	r0, r0, #2		@ A (no unaligned access fault)
	orr	r0, r0, #1 << 22	@ U (v6 unaligned access model)
					@ (needed for ARM1176)
	orr	r0, r0, #1<<13		@ V
	mrc	p15, 0, r2, c2, c0, 2   @ read ttb control reg
	orr	r0, r0, #1		@ MMU enabled
	mov	r1, #0xfffffffd		@ domain 0 = client
	bic     r2, r2, #1 << 31        @ 32-bit translation system
	bic     r2, r2, #3 << 0         @ use only ttbr0

	mcr	p15, 0, r1, c3, c0, 0	@ load domain access control
	mcr     p15, 0, r2, c2, c0, 2   @ load ttb control
	mcr	p15, 0, r0, c7, c5, 4	@ ISB
	mcr	p15, 0, r0, c1, c0, 0	@ load control register
	mrc	p15, 0, r0, c1, c0, 0	@ and read it back
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 4	@ ISB

	mov    pc, lr

.align 4
.globl switch_to
switch_to:                      # switch_to(from, to)
# TODO: select relevant registers to keep

# save from's registers
# in the structure pointed by r0
	stmia r0!, {r4-r10, fp, ip, sp, lr}
	mrs   r4, cpsr
	stmia r0!, {r4}

# restore to's registers
# in the structure pointed by r1
	ldr   r4, [r1, #+44]
	msr   cpsr, r4

	ldmia r1!, {r4-r10, fp, ip, sp, lr}

# return
	mov pc, lr

