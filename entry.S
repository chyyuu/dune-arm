#include "mmu.h"
#include "pgtable-hwdef.h"
#include "param.h"


#ifdef CONFIG_XIP_KERNEL
#define KERNEL_START	XIP_VIRT_ADDR(CONFIG_XIP_PHYS_ADDR)
#define KERNEL_END	_edata_loc
#else
#define KERNEL_START	KERNEL_RAM_VADDR
#define KERNEL_END	_end
#endif


#define KERNEL_RAM_VADDR	(PAGE_OFFSET + TEXT_OFFSET)

##if (KERNEL_RAM_VADDR & 0xffff) != 0x8000
##error KERNEL_RAM_VADDR must start at 0xXXXX8000
##endif


.equ DISABLE_IRQ, 0x80
.equ DISABLE_FIQ, 0x40
.equ SYS_MOD, 0x1f
.equ IRQ_MOD, 0x12
.equ FIQ_MOD, 0x11
.equ SVC_MOD, 0x13
.equ ABT_MOD, 0x17
.equ UND_MOD, 0x1b

.equ MEM_SIZE, SDRAM0_SIZE 
.equ TEXT_BASE, SDRAM0_START


.macro	pgtbl, rd
	ldr	\rd, =(__virt_to_phys(KERNEL_RAM_VADDR - 0x4000))
.endm

.macro _debug_break, nr
	ldr r5, =\nr
	ldr r1, =0xf0000000
	str r5, [r1]
.endm

.text

.global kern_entry 
kern_entry:
	mrs r0, cpsr
	bic r0, r0, #(DISABLE_FIQ|DISABLE_IRQ)
	msr cpsr, r0

	/* enable swp,flow prediction */
	ldr r1, =(1<<11)|(1<<10)
	MCR p15, 0, r1, c1, c0, 0

#if 1
    # enable the FPU
    mrc p15, 0, r0, c1, c0, 2
    orr r0, r0, #0x300000            /* single precision */
    orr r0, r0, #0xC00000            /* double precision */
    mcr p15, 0, r0, c1, c0, 2
    isb
    mov r0, #0x40000000
    //fmxr fpexc,r0
    mcr     p10,#0x7,r0,c8,c0,#0
#endif



	/* L1 Cache */


# Stack inititialization - starts in SVC mode
# only 12bytes are used in these stacks, see trapentry.S
	msr cpsr_c,#(DISABLE_IRQ|DISABLE_FIQ|IRQ_MOD)
	ldr sp,=irq_stack
	msr cpsr_c,#(DISABLE_IRQ|DISABLE_FIQ|FIQ_MOD)
	ldr sp,=fiq_stack
	msr cpsr_c,#(DISABLE_IRQ|DISABLE_FIQ|ABT_MOD)
	ldr sp,=abt_stack
	msr cpsr_c,#(DISABLE_IRQ|DISABLE_FIQ|UND_MOD)
	ldr sp,=und_stack
	msr cpsr_c,#(DISABLE_IRQ|DISABLE_FIQ|SYS_MOD)
	ldr sp,=sys_stacktop
	msr cpsr_c,#(DISABLE_IRQ|DISABLE_FIQ|SVC_MOD)

#in SVC mode
relocated:
	ldr sp,=bootstacktop

#if 0
ttest:
	bl __create_pgtbl

	b __enable_mmu
#endif

	ldr r0, =kern_init

	# goto kern_init
	mov pc, r0



###################################################################

mm_mmuflag:
.long	PMD_TYPE_SECT | PMD_SECT_AP_WRITE | PMD_SECT_AP_READ
# | PMD_FLAGS_SMP
#.long   PMD_TYPE_SECT | PMD_SECT_BUFFERABLE | PMD_SECT_CACHEABLE | PMD_BIT4 | PMD_SECT_AP_WRITE | PMD_SECT_AP_READ
#.long   PMD_TYPE_SECT | PMD_BIT4 | PMD_SECT_AP_WRITE | PMD_SECT_AP_READ

control_access_value:
.long	DOM3CLT

control_access_mask:
.long	CHANGEALLDOM


mmu_value:
.long	ENABLEMMU | ENABLEICACHE | ENABLEDCACHE | ENABLEHIGHEVT
mmu_mask:
.long	CHANGEMMU | CHANGEICACHE | CHANGEDCACHE | CHANGEHIGHEVT

__create_pgtbl:
	pgtbl	r4

	mov r0, r4
	mov r3, #0
	add r6, r0, #0x4000
1:	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	teq	r0, r6
	bne	1b

#	mov r7, #mm_mmuflag
#	ldr r7, [r7, #0]
	ldr r7, =mm_mmuflag
	eor r7, r7, #0xc << 28
	ldr r7, [r7]

	# map 0x0 to 0x0
	ldr r6, =0
	orr r6, r6, r7
	str r6, [r4]

	# map first section
	mov r6, pc, lsr #20
	orr r3, r7, r6, lsl #20
	str r3, [r4, r6, lsl #2]

	# map first kernel section
	add r0, r4, #(KERNEL_RAM_VADDR & 0xff000000) >> 18
	str r3, [r0, #(KERNEL_RAM_VADDR & 0x00f00000) >> 18]!

#	ldr r6, =(end - PAGE_OFFSET - 1)
#	ldr r6, =(end - PAGE_OFFSET - 1)
	ldr r6, =(KMEMSIZE - 1)
	mov r6, r6, lsr #20

	#add some more memory for dynamically allocation
#	add r6, r6, #23

1:	add r3, r3, #1 << 20
	str r3, [r0, #4]!
	subs r6, r6, #1
	bgt 1b

	add r0, r4, #PAGE_OFFSET >> 18
	orr r6, r7, #PHYS_OFFSET
	str r6, [r0]

	# map io space to new page table
	ldr r6, =IO_SPACE_START
	mov r6, r6, lsr #20
	mov r0, r6
	orr r3, r7, r6, lsl #20
#	str r3, [r4, r6, lsl #2]

	add r0, r4, r0, lsl #2

	ldr r6, =IO_SPACE_SIZE
	mov r6, r6, lsr #20

1:	str r3, [r0]
	add r0, r0, #4
	add r3, r3, #1 << 20
	subs r6, r6, #1
	bgt 1b

	mov pc, lr


	.ltorg
	.align
__enable_mmu_loc:
	.long	.
	.long	__enable_mmu
	.long	__enable_mmu_end

/*
   r0, r1, r2
*/
__enable_mmu:

	mcr	p15, 0, r4, c2, c0, 0		@ load page table pointer

	#set domain access
	mrc p15, 0, r0, c3, c0, 0
	ldr r1, control_access_value
	ldr r2, control_access_mask
	mvn r2, r2
	and r0, r0, r2
	orr	r0, r0, r1
	mcr p15, 0, r0, c3, c0, 0

	#enable cache
	mrc p15, 0, r0, c1, c0, 0
	ldr r1, mmu_value
	ldr r2, mmu_mask
	mvn r2, r2
	and r0, r0, r2
	orr r0, r0, r1
	mcr p15, 0, r0, c1, c0, 0

	ldr r0, =kern_init

	# initialize the stack top
	ldr sp,=bootstacktop

	mov r0, r0
	mov r0, r0
	mov r0, r0

	# goto kern_init
	mov pc, r0

__enable_mmu_end:

#.globl kern_init
#kern_init:
#	ldr r0, =0x08
#	ldr r1, =0xf0000000
#	str r0, [r1]
	
    .data
    .align 6
# There might be a alignment problem, as should be aligned to a page size
irq_stack:
.space 64
irq_stacktop:
fiq_stack:
.space 64
fiq_stacktop:
abt_stack:
.space 64
abt_stacktop:
und_stack:
.space 64
und_stacktop:
sys_stack:
.space KSTACKSIZE
.global sys_stacktop
sys_stacktop:
    .globl bootstack
bootstack:
    .space KSTACKSIZE
    .globl bootstacktop
bootstacktop:


