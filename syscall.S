
#include "trap.h"

.text
.align 4

/* void syscall_passthrough(struct trapframe *) */
/* XXX do hvc? */
.global syscall_passthrough
syscall_passthrough:
	ldr r2, =0xf0000004
	str r0, [r2]
	mov pc, lr


/* void syscall_passthrough_fast(struct pushregs*) */
.global syscall_passthrough_fast
syscall_passthrough_fast:
	stmfd	sp!, {r4-r9}
	mov r9, r0
	ldmfd   r0, {r1-r8}
	ldr r0, =(0x95c1ba5e+4)
	hvc #0
	str r0, [r9]
 	ldmfd	sp!, {r4-r9}
	mov	pc, lr

.global bsc_passthrough_fast
bsc_passthrough_fast:
	ldr r0, =(0x95c1ba5e+5)
	hvc #0
	mov	pc, lr


/* void hyper_map_region(struct elf_info*) */
.global hyper_map_region
hyper_map_region:
	ldr r1, =0xf0000008
	str r0, [r1]
	dsb
	isb
	mov pc, lr


