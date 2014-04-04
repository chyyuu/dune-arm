#ifndef HW_H_CMXUMTHE
#define HW_H_CMXUMTHE

#include "trap.h"
extern void v7_flush_kern_dcache_area(void *addr, size_t size);
void syscall_passthrough(struct trapframe*);
void syscall_passthrough_fast(struct pushregs*);


struct context {
	uint32_t r4;
	uint32_t r5;
	uint32_t r6;
	uint32_t r7;
	uint32_t r8;
	uint32_t r9;
	uint32_t sl;
	uint32_t fp;
	uint32_t eic;
	uint32_t esp;
	uint32_t epc;		//lr
	uint32_t e_cpsr;
};

void switch_to(struct context *from, struct context *to);

static inline void v7_flush_icache(){
	int zero = 0;
	asm volatile(
		"mcr	p15, 0, %0, c7, c5, 0	@ I+BTB cache invalidate\n"
		"isb\n"
		::"r"(zero):
	);
}

static inline void __print_hex(uint32_t hex){
	asm volatile(
		"ldr r0, =0xf0000000;"
		"mov r1, %0;"
		"str r1, [r0];"
		:
		:"r"(hex)
		:"r0","r1"
	);
}


void panic(int id){
	__print_hex(0xffff0000);
	__print_hex(id);
	while(1);
}
#define assert(x) do{if(!(x)) panic(-1);}while(0)




#endif /* end of include guard: HW_H_CMXUMTHE */

