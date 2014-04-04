#ifndef HW_H_CMXUMTHE
#define HW_H_CMXUMTHE

#include "trap.h"
extern void v7_flush_kern_dcache_area(void *addr, size_t size);
void syscall_passthrough(struct trapframe*);
void syscall_passthrough_fast(struct pushregs*);

static inline void v7_flush_icache(){
	int zero = 0;
	asm volatile(
		"mcr	p15, 0, %0, c7, c5, 0	@ I+BTB cache invalidate\n"
		"isb\n"
		::"r"(zero):
	);
}



#endif /* end of include guard: HW_H_CMXUMTHE */

