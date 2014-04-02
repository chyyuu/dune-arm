#include "lib.h"
#include "mmu.h"
#include "trap.h"
#include "param.h"
#include <string.h>
#include <sys/queue.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <elf.h>
#include "../kvm_test/elf_loader.h"
#include "../kvm_test/syscall_nr.h"
#include "batch_sc.h"

void __print_hex(uint32_t hex){
	asm volatile(
		"ldr r0, =0xf0000000;"
		"mov r1, %0;"
		"str r1, [r0];"
		:
		:"r"(hex)
		:"r0","r1"
	);
}


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

/* controlSet
 * sets the control bits register in CP15:c1 */
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
	controlSet(enable, change);
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
extern void v7_flush_kern_dcache_area(void *addr, size_t size);

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
	tlb_invalidate_all();
}

/* batch syscall */
static void *__bsc_slots = 0;

void setup_batch_syscall(){
	struct page *pg = page_alloc();
	uintptr_t pa = PADDR(page2kva(pg));
	struct bsc_superblk *sb = (struct bsc_superblk*)page2kva(pg);
	sb->len = 0;
	sb->id = 1;
	__bsc_slots = page2kva(pg);

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

	//__print_hex(*(int*)elf_info.stacktop);

	//switch to sys mode
	struct trapframe tf;
	memset(&tf, 0, sizeof(tf));
	tf.tf_regs.reg_r[0] = 0;
	tf.tf_regs.ARM_sp = (uintptr_t)elf_info.stacktop;
	//__print_hex(elf_info.stacktop);
	tf.tf_regs.ARM_pc = (uintptr_t)elf_info.entry;
	//tf.tf_regs.ARM_pc = (uintptr_t)__sys_entry;
	//XXX enable int
	//tf.tf_sr = ARM_SR_MODE_SYS;
	tf.tf_sr = ARM_SR_MODE_SYS;
	switch_to_sys(&tf);

	return;
}

void sys_entry(){
	__print_hex(*(int*)elf_info.entry);
	while(1);
}

void syscall_passthrough(struct trapframe*);
void syscall_passthrough_fast(struct pushregs*);

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

void panic(int id){
	__print_hex(0xffff0000);
	__print_hex(id);
	while(1);
}
#define assert(x) do{if(!(x)) panic(-1);}while(0)

void decrypt_page(void *dst, void *src){
	uint32_t len = PAGE_SIZE;
	uint32_t *pd = (uint32_t*)dst;
	uint32_t *ps = (uint32_t*)src;
	while(len > 0){
		*pd++ = (*ps++) ^ ELF_SIMPLE_KEY;
		len -= 4;
	}
}

static inline v7_flush_icache(){
	int zero = 0;
	asm volatile(
		"mcr	p15, 0, %0, c7, c5, 0	@ I+BTB cache invalidate\n"
		"isb\n"
		::"r"(zero):
	);
}


static int pgfault_handler(struct trapframe *tf){
	uint32_t badaddr = 0;
	if (tf->tf_trapno == T_PABT) {
		badaddr = tf->tf_epc;
	} else {
		badaddr = far();
	}
	//__print_hex(badaddr);
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

static inline void do_syscall(struct trapframe *tf){
	int num = tf->tf_regs.reg_r[7];
	switch(num){
		case __NR_brk:
			__sys_brk(tf);
			break;
		case __NR_mmap2:
			__sys_mmap2(tf);
			break;
		case __NR_write:
		case __NR_read:
			v7_flush_kern_dcache_area(tf->tf_regs.reg_r[1], tf->tf_regs.reg_r[2]);
			syscall_passthrough_fast(&tf->tf_regs);
			v7_flush_kern_dcache_area(tf, sizeof(*tf));
			break;
		case __NR_gettimeofday:
			if(tf->tf_regs.reg_r[0])
				v7_flush_kern_dcache_area(tf->tf_regs.reg_r[0], sizeof(struct timeval));
			if(tf->tf_regs.reg_r[1])
				v7_flush_kern_dcache_area(tf->tf_regs.reg_r[1], sizeof(struct timezone));
			syscall_passthrough_fast(&tf->tf_regs);
			v7_flush_kern_dcache_area(tf, sizeof(*tf));
			break;
		case __NR_getpid:
			syscall_passthrough_fast(&tf->tf_regs);
			v7_flush_kern_dcache_area(tf, sizeof(*tf));
			break;
		default:
			v7_flush_kern_cache_all();
			//syscall_passthrough(PADDR(tf));
			syscall_passthrough_fast(&tf->tf_regs);
			v7_flush_kern_dcache_area(tf, sizeof(*tf));
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
			do_syscall(tf);
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


