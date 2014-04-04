#include "lib.h"
#include "mmu.h"
#include "trap.h"
#include "param.h"
#include <string.h>
#include <sys/queue.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <elf.h>
#include "hw.h"
#include "kfiber.h"
#include "../kvm_test/elf_loader.h"
#include "../kvm_test/syscall_nr.h"
#include "batch_sc.h"


static void clear_bss(){
	extern char edata[], end[];
	unsigned int *ptr = (unsigned int*)edata;
	while(ptr < (unsigned int*)end)
		*ptr++ = 0;
}

static inline void exception_vector_init(void)
{
	extern char __vector_table, __vector_table_end;
	memcpy((void *)PHYS_OFFSET, (void *)&__vector_table,
	       &__vector_table_end - &__vector_table);
}

/* page manager */
SLIST_HEAD(page_head, page);           
typedef SLIST_ENTRY(page) page_entry_t;
static struct page_head pages_free;

struct page {
	page_entry_t link;
	uint32_t ref;
};

struct page *pages;             
int num_pages;
int num_kern_pages;

static inline uintptr_t page2kva(struct page *pg){
	return PAGE_OFFSET + ((uintptr_t)(pg - pages) << PGSHIFT);
}

void page_init(){
	extern char end[];
	int i;
	size_t maxpa = KMEMSIZE;
	size_t end_pa = __virt_to_phys(ROUNDUP((uintptr_t)end, PAGE_SIZE));
	size_t page_size = ROUNDUP(MAX_PAGES * sizeof(struct page), PAGE_SIZE);

	pages = (struct page*)ROUNDUP((void*)end, PAGE_SIZE);

	int start = (end_pa - PHYS_OFFSET + ROUNDUP(MAX_PAGES*sizeof(struct page), PAGE_SIZE)) / PAGE_SIZE;
	num_pages = KMEMSIZE / PAGE_SIZE;
	num_kern_pages = start;

	SLIST_INIT(&pages_free);
	for(i = start; i < num_pages ;i++){
		pages[i].ref = 0;
		SLIST_INSERT_HEAD(&pages_free, &pages[i], link);
	}

}

struct page* page_alloc(){
	if(SLIST_EMPTY(&pages_free))
		return NULL;
	struct page *pg = SLIST_FIRST(&pages_free);
	SLIST_REMOVE_HEAD(&pages_free, link);

	pg->ref++;
	return pg;
}

void page_free(struct page* pg){
	SLIST_INSERT_HEAD(&pages_free, pg, link);
}

/* task manager(kfiber) */
#define MAX_N_KFIBER 64

typedef TAILQ_HEAD(task_head, task_struct) task_head_t;
typedef TAILQ_ENTRY(task_struct) task_entry_t;

static task_head_t free_tasks;
static task_head_t ready_tasks;
static task_head_t wait_tasks;

struct task_struct{
	volatile long state;
	int runs;		// the running times of Proces
	int tid;
	uintptr_t kstack;	// Process kernel stack
	struct page* kstack_page;
	struct context context;
	struct trapframe *tf;

	int exit_code;		// return value when exit

	task_entry_t link;
};

struct task_struct *current;

#define TASK_STATE_FREE 1
#define TASK_STATE_READY 2
#define TASK_STATE_WAIT 3
#define TASK_STATE_DIE 4

static struct task_struct __task_pool[MAX_N_KFIBER];

struct task_struct *alloc_task(){
	if(TAILQ_EMPTY(&free_tasks))
		return NULL;
	struct task_struct *ts = TAILQ_FIRST(&free_tasks);
	TAILQ_REMOVE(&free_tasks, ts, link);
	return ts;
};

void free_task(struct task_struct* ts){
	assert(0);
	if(!ts)
		return;
	//if(ts->kstack)
//		free_page(kva2page(ts->kstack));
	if(ts->state == TASK_STATE_WAIT)
		TAILQ_REMOVE(&wait_tasks, ts, link);
	if(ts->state == TASK_STATE_READY)
		TAILQ_REMOVE(&ready_tasks, ts, link);
	ts->state = TASK_STATE_FREE;
	TAILQ_INSERT_HEAD(&free_tasks, ts, link);
}

void init_task(struct task_struct *ts){
	//memset(ts, 0, sizeof(*ts));
	ts->state = TASK_STATE_READY;
	ts->runs = 0;
	ts->tid = 0;
	ts->kstack = (uintptr_t)page2kva(ts->kstack_page) + PAGE_SIZE;
	ts->tf = 0;
	memset(&ts->context, 0, sizeof(ts->context));
}

void task_enqueue(struct task_struct *ts){
	ts->state = TASK_STATE_READY;
	TAILQ_INSERT_TAIL(&ready_tasks, ts, link);
}

struct task_struct * task_dequeue(){
	if(TAILQ_EMPTY(&ready_tasks))
		return NULL;
	struct task_struct *ts = TAILQ_FIRST(&ready_tasks);
	TAILQ_REMOVE(&ready_tasks, ts, link);
	return ts;
}

void kfiber_init(){
	int i;
	TAILQ_INIT(&ready_tasks);
	TAILQ_INIT(&wait_tasks);
	TAILQ_INIT(&free_tasks);

	for(i=0;i<MAX_N_KFIBER;i++){
		__task_pool[i].state = TASK_STATE_FREE;
		/* XXX hack */
		__task_pool[i].kstack_page = page_alloc();
		TAILQ_INSERT_HEAD(&free_tasks, &__task_pool[i], link);
	}

	/* idle */
	struct task_struct *ts = alloc_task();
	init_task(ts);
	//task_enqueue(ts);
	current = ts;
}


void kfiber_run_next(){
	struct task_struct *ts = task_dequeue();
	if(!ts){
		panic(0x131);
		return;
	}
	struct task_struct  *prev = current;
	current = ts;
	switch_to(&prev->context, &ts->context);
}

void kfiber_idle(void){
	while(1)
		kfiber_run_next();
}

/* mmu */
static unsigned char __boot_pgtlb[32*1024];
pde_t *boot_pgdir;

pte_t *get_pte(pde_t * pgdir, uintptr_t la, int create){
	pde_t *pdt_entry = &pgdir[PDX(la)];
	pde_t *pdt = NULL;
	pde_t *pdt_entry_low = &pgdir[PDX(la) & ~3];
	if(!ptep_present(pdt_entry)){
		if(!create)
			return NULL;
		struct page *pg = page_alloc();
		if(!pg)
			return NULL;
		pdt = (pde_t *)page2kva(pg);
		memset(pdt, 0, PAGE_SIZE);
		pdep_map(pdt_entry_low, (uintptr_t)PADDR(pdt));
		pdep_map(pdt_entry_low+1, (uintptr_t)PADDR(pdt)+1024);
		pdep_map(pdt_entry_low+2, (uintptr_t)PADDR(pdt)+1024*2);
		pdep_map(pdt_entry_low+3, (uintptr_t)PADDR(pdt)+1024*3);

		v7_flush_kern_dcache_area(pdt, PAGE_SIZE);
		v7_flush_kern_dcache_area(pdt_entry_low, 4*sizeof(pde_t));
	}

	pdt = (pde_t*)KADDR(PDE_ADDR(*pdt_entry));

	pte_t *ptep = (pte_t*)&pdt[PTX(la)];
	return ptep;
}

int map_page(pde_t *pgdir, uintptr_t la, struct page* pg, uint32_t perm){
	uintptr_t pa = PADDR(page2kva(pg));
	la = ROUNDDOWN(la, PAGE_SIZE);
	pa = ROUNDDOWN(pa, PAGE_SIZE);
	pte_t *ptep = get_pte(pgdir, la, 1);
	if(!ptep)
		return -1;
	ptep_map(ptep, pa);
	ptep_set_perm(ptep, PTE_P | perm);
	v7_flush_kern_dcache_area(ptep, sizeof(pte_t));
}

//boot_map_segment - setup&enable the paging mechanism
// parameters
//  la:   linear address of this memory need to map
//  size: memory size
//  pa:   physical address of this memory
//  perm: permission of this memory  
void
boot_map_segment(pde_t * pgdir, uintptr_t la, size_t size, uintptr_t pa,
		 uint32_t perm)
{
	if (size == 0)
		return;
	//assert(PGOFF(la) == PGOFF(pa));
	size_t n = ROUNDUP(size + PGOFF(la), PAGE_SIZE) / PAGE_SIZE;
	la = ROUNDDOWN(la, PAGE_SIZE);
	pa = ROUNDDOWN(pa, PAGE_SIZE);
	for (; n > 0; n--, la += PAGE_SIZE, pa += PAGE_SIZE) {
		pte_t *ptep = get_pte(pgdir, la, 1);
		if(!ptep){
			return;
		}
		//assert(ptep != NULL);
		ptep_map(ptep, pa);
		ptep_set_perm(ptep, PTE_P | perm);
		v7_flush_kern_dcache_area(ptep, sizeof(pte_t));
	}
}

/* 16 domains */
static void domainAccessSet(uint32_t value, uint32_t mask)
{
	uint32_t c3format;
	asm volatile ("MRC p15, 0, %0, c3, c0, 0"	/* read domain register */
		      :"=r" (c3format)
	    );
	c3format &= ~mask;	/* clear bits that change */
	c3format |= value;	/* set bits that change */
	asm volatile ("MCR p15, 0, %0, c3, c0, 0"	/* write domain register */
		      ::"r" (c3format)
	    );
}
static void controlSet(uint32_t value, uint32_t mask)
{
	uint32_t c1format;
	asm volatile ("MRC p15, 0, %0, c1, c0, 0"	/* read control register */
		      :"=r" (c1format)
	    );
	c1format &= ~mask;	/* clear bits that change */
	c1format |= value;	/* set bits that change */
	asm volatile ("MCR p15, 0, %0, c1, c0, 0"	/* write control register */
		      ::"r" (c1format)
	    );
}

/* mmu_init - initialize the virtual memory management */
// Supposed already attached and initialized
/* 1. Initialize the page tables in main memory by filling them with FAULT entries.
 * 2. Fill in the page tables with translations that map regions to physical memory.
 * 3. Activate the page tables.
 * 4. Assign domain access rights.
 * 5. Enable the memory management unit and cache hardware.
 * */
extern void v7_mmu_cache_on();
void mmu_init(void)
{
	uint32_t enable, change;

	/* Part 4 Set Domain Access */
	domainAccessSet(DOM3CLT, CHANGEALLDOM);	/* set Domain Access */

	/* Part 5 Enable MMU, caches and write buffer */
	enable = ENABLEMMU | ENABLEICACHE | ENABLEDCACHE | ENABLEHIGHEVT;
	change = CHANGEMMU | CHANGEICACHE | CHANGEDCACHE | CHANGEHIGHEVT;
#ifdef __MACH_ARM_ARMV6
	enable |= ENABLENEWPT;
	change |= CHANGENEWPT;
#endif

	/* enable cache and MMU */
	//controlSet(enable, change);
	v7_mmu_cache_on();
}

void boot_pg_init(){
	boot_pgdir = (pde_t*)ROUNDUP((uintptr_t)__boot_pgtlb, 4*PAGE_SIZE);
	memset(boot_pgdir, 0, 4*PAGE_SIZE);

	boot_map_segment(boot_pgdir, PAGE_OFFSET, KMEMSIZE,
			PHYS_OFFSET, PTE_W);

	boot_map_segment(boot_pgdir, IO_SPACE_START, IO_SPACE_SIZE, IO_SPACE_START, PTE_W | PTE_IOMEM);

	boot_map_segment(boot_pgdir, 0xFFFF0000, PAGE_SIZE, SDRAM0_START, PTE_PWT | PTE_W );	// high location of vector table

	ttbSet((uint32_t)PADDR(boot_pgdir));
	//ttbSet(0x4000);

	mmu_init();
	tlb_invalidate_all();

}

/*
static void load_linker(){
	extern char _binary_linker_start[], _binary_linker_end[];
	size_t _linker_len = _binary_linker_end - _binary_linker_start;
	Elf32_Ehdr *elf = (Elf32_Ehdr*)_binary_linker_start;
	if(elf->e_ident[0] != EI_MAG0
	|| elf->e_ident[1] != EI_MAG1
	|| elf->e_ident[2] != EI_MAG2
	|| elf->e_ident[3] != EI_MAG3
	)
		return;
	uint32_t real_entry;
	uint32_t is_dynamic = 0, interp_idx;
	uint32_t base_addr = 0;
	uint32_t vm_flags, phnum;
	real_entry = elf->e_entry;
	for (phnum = 0; phnum < elf->e_phnum; phnum++) {
		size_t phoff = elf->e_phoff + sizeof(Elf32_Phdr) * phnum;
		Elf32_Phdr *phdr = (Elf32_Phdr*)(phoff + _binary_linker_start);

	}
}
*/

void hyper_map_region(struct elf_info *);
extern void switch_to_sys(struct trapframe*);
extern void __sys_entry();
extern char sys_stacktop[];

#define ELF_MAPPING_ENCRYPTED_START 0xE0000000
static void build_elf_mapping(pde_t *pgdir, struct elf_info *info)
{
	int i;
	for(i = 0; i < info->nmap; i++){
		//decrypt on prefetch
		if(info->mapping[i].flags & ELF_PROG_ENCRYPTED){
			boot_map_segment(pgdir, info->mapping[i].addr+ELF_MAPPING_ENCRYPTED_START, info->mapping[i].limit, info->mapping[i].addr, 0);
		}else{
			boot_map_segment(pgdir, info->mapping[i].addr, info->mapping[i].limit, info->mapping[i].addr, PTE_W);
		}
	}
	/* visible to HW tlb walk */
	v7_flush_kern_cache_all();
	tlb_invalidate_all();
}

/* batch syscall */
static void *__bsc_slots = 0;
static int bsc_enabled = 0;

void setup_batch_syscall(){
	struct page *pg = page_alloc();
	uintptr_t pa = PADDR(page2kva(pg));
	struct bsc_superblk *sb = (struct bsc_superblk*)page2kva(pg);
	sb->len = 0;
	sb->id = 1;
	__bsc_slots = (void*)page2kva(pg);

	//*(unsigned int*)(0xf000000c) = pa;
}

void bsc_passthrough_fast();

static inline void __flush_all_bsc(){
	struct bsc_superblk *sb = BSC_SB();
	if(!sb->len)
		return;
	v7_flush_kern_dcache_area(__bsc_slots, PAGE_SIZE);
	bsc_passthrough_fast();
	sb->len = 0;
}

inline uint64_t bsc_add_request(uint32_t nr, uint32_t a0, uint32_t a1, uint32_t a2,
		uint32_t a3, uint32_t a4, uint32_t a5){
	struct bsc_superblk *sb = BSC_SB();
	if(sb->len >= BSC_MAX_NR_CALL)
		__flush_all_bsc();
	uint64_t id = sb->id++;
	struct bsc_request *r = BSC_SLOT(sb->len);
	r->nr_syscall = nr;
	r->status = 0;
	r->args[0] = a0;
	r->args[1] = a1;
	r->args[2] = a2;
	r->args[3] = a3;
	r->args[4] = a4;
	r->args[5] = a5;
	sb->len++;
}

static struct elf_info elf_info;
static inline __mark_guest_start(){
	*(unsigned int*)(0xf000000c) = 0;
}

static struct task_struct* kfiber_create(uintptr_t entry, uintptr_t stacktop, void* data){
	//__print_hex(*(int*)elf_info.stacktop);

	//switch to sys mode
	struct task_struct *usermain = alloc_task();
	if(!usermain)
		return NULL;
	init_task(usermain);
	struct trapframe *tf = usermain->kstack - sizeof(struct trapframe);
	usermain->tf = tf;
	//struct trapframe tf;
	memset(tf, 0, sizeof(*tf));
	tf->tf_regs.reg_r[0] = (uint32_t)data;
	tf->tf_regs.ARM_sp = stacktop;
	//__print_hex(elf_info.stacktop);
	tf->tf_regs.ARM_pc = entry;
	//tf.tf_regs.ARM_pc = (uintptr_t)__sys_entry;
	//XXX enable int
	tf->tf_sr = ARM_SR_MODE_SYS;
	//tf.tf_sr = ARM_SR_MODE_USR;
	
	//kernel context
	usermain->context.e_cpsr = ARM_SR_MODE_SVC;
	usermain->context.esp = (uintptr_t)tf;
	usermain->context.epc = (uintptr_t)switch_to_sys;

	return usermain;
}

static void user_init(){
	struct task_struct *usermain = kfiber_create((uintptr_t)elf_info.entry, 
			(uintptr_t)elf_info.stacktop, NULL);
	task_enqueue(usermain);
}

void kern_init(){
	clear_bss();

	exception_vector_init();
	//*(int*)0xf0000000 = 0x1236;

	page_init();

	boot_pg_init();

	hyper_map_region((struct elf_info*)PADDR(&elf_info));
	//v7_flush_kern_cache_all();
	v7_flush_kern_dcache_area(&elf_info, sizeof(elf_info));

	build_elf_mapping(boot_pgdir, &elf_info);

	setup_batch_syscall();

	kfiber_init();

	user_init();

	__mark_guest_start();
	//switch_to_sys(&tf);

	kfiber_idle();
	return;
}

void sys_entry(){
	__print_hex((int*)elf_info.entry);
	while(1);
}

struct elf_mapping* pgfault_find_mapping(uint32_t far){
	int i;
	for(i = 0;i < elf_info.nmap;i++){
		uintptr_t start = elf_info.mapping[i].addr;
		uintptr_t end = start + elf_info.mapping[i].limit;
		if(far >= start && far < end)
			return &elf_info.mapping[i];
	}
	return NULL;
}

void decrypt_page(void *dst, void *src){
	uint32_t len = PAGE_SIZE;
	uint32_t *pd = (uint32_t*)dst;
	uint32_t *ps = (uint32_t*)src;
	while(len > 0){
		*pd++ = (*ps++) ^ ELF_SIMPLE_KEY;
		len -= 4;
	}
}

static int pgfault_handler(struct trapframe *tf){
	uint32_t badaddr = 0;
	if (tf->tf_trapno == T_PABT) {
		badaddr = tf->tf_epc;
	} else {
		badaddr = far();
	}
	__print_hex(badaddr);
	//XXX
	struct elf_mapping *mapping = pgfault_find_mapping(badaddr);
	if(!mapping){
		__print_hex(badaddr);
		panic(0x1);
	}
	//decrypt
	if(mapping->flags & ELF_PROG_ENCRYPTED){
		struct page *pg = page_alloc();
		assert(pg);
		uintptr_t start = ROUNDDOWN(badaddr, PAGE_SIZE);
		decrypt_page((void*)page2kva(pg), (void*)(start + ELF_MAPPING_ENCRYPTED_START));
		map_page(boot_pgdir, start, pg, 0);
		v7_flush_icache();
	}else{
		panic(0x2);
	}
	return 0;
}

static int __sys_brk(struct trapframe *tf){
	int heap_idx = ELF_GET_HEAP_IDX(elf_info);
	uintptr_t oldbrk = elf_info.mapping[heap_idx].addr + elf_info.mapping[heap_idx].limit;

	syscall_passthrough(PADDR(tf));
	v7_flush_kern_dcache_area(tf, sizeof(*tf));

	uintptr_t newbrk = tf->tf_regs.reg_r[0];
	if(oldbrk < newbrk){
		boot_map_segment(boot_pgdir, oldbrk, newbrk - oldbrk, oldbrk, PTE_W);

	}else if(oldbrk > newbrk){
		//TODO
		__print_hex(0x1231213);
		while(1);
	}/* else do nothing */
	size_t new_size = newbrk - elf_info.mapping[heap_idx].addr;
	elf_info.mapping[heap_idx].limit = new_size;
	return 0;
}

static int __sys_mmap2(struct trapframe *tf){
	syscall_passthrough(PADDR(tf));
	v7_flush_kern_dcache_area(tf, sizeof(*tf));
	void * ret = (void*)tf->tf_regs.reg_r[0]; 
	if(ret == MAP_FAILED)
		return -1;
	uint32_t perm = 0;
	int operm = tf->tf_regs.reg_r[2];
	if(operm & PROT_WRITE)
		perm |= PTE_W;

	uintptr_t la = (uintptr_t)ret;
	size_t size = tf->tf_regs.reg_r[1];
	boot_map_segment(boot_pgdir, la, size, la, perm);

	return 0;
}


void __sys_kfiber_create(struct trapframe* tf){
	int r = -1; 
	struct task_struct *ts = kfiber_create((uintptr_t)tf->tf_regs.reg_r[0],
			(uintptr_t)tf->tf_regs.reg_r[1],
			(void*)tf->tf_regs.reg_r[2]);
	if(ts)
		r = ts->tid;
	tf->tf_regs.reg_r[0] = r;
	task_enqueue(ts);
}

void __sys_kfiber_exit(struct trapframe* tf){
	current->state = TASK_STATE_FREE;
	TAILQ_INSERT_TAIL(&free_tasks, current, link);
	kfiber_run_next();
}

void __sys_kfiber_yield(struct trapframe* tf){
	current->state = TASK_STATE_READY;
	TAILQ_INSERT_TAIL(&ready_tasks, current, link);
	kfiber_run_next();
}

static inline void do_syscall(struct trapframe *tf){
	int num = tf->tf_regs.reg_r[7];
	switch(num){
		case __NR_brk:
			__sys_brk(tf);
			break;
		case __NR_mmap2:
			__sys_mmap2(tf);
			break;
		/* hook */
		case __NR_exit:
			syscall_passthrough(&tf->tf_regs);
			/* never return */
			break;
		case __NR_kfiber_create:
			__sys_kfiber_create(tf);
			bsc_enabled = 1;
			break;
		default:
			//v7_flush_kern_cache_all();
			//syscall_passthrough(PADDR(tf));
			syscall_passthrough_fast(&tf->tf_regs);
			v7_flush_kern_dcache_area(tf, sizeof(*tf));
	}
}

static inline void do_bsc_syscall(struct trapframe *tf){
	int num = tf->tf_regs.reg_r[7];
	switch(num){
		case __NR_kfiber_create:
			__sys_kfiber_create(tf);
			break;
		case __NR_kfiber_exit:
			__sys_kfiber_exit(tf);
			break;
		case __NR_kfiber_yield:
			__sys_kfiber_yield(tf);
			break;
		default:
			do_syscall(tf);
	}
}

static inline void trap_dispatch(struct trapframe *tf)
{
	int ret;
	switch(tf->tf_trapno){
		case T_PABT:
		case T_DABT:
			pgfault_handler(tf);
			break;
		case T_SWI:
			if(bsc_enabled)
				do_syscall(tf);
			else
				do_bsc_syscall(tf);
			break;
		case T_IRQ:
			__print_hex(0x32421);
			while(1);
			break;
		case T_UNDEF:{
			uint32_t inst = *(uint32_t*)(tf->tf_epc - 4);
			__print_hex(0x32423);
			__print_hex(inst);
			while(1);
			break;
			     }
		default:
			break;
	}
}

void trap(struct trapframe *tf){
	trap_dispatch(tf);
}


