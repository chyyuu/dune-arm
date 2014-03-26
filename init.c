#include "lib.h"
#include "mmu.h"
#include "trap.h"
#include "param.h"
#include <string.h>
#include <sys/queue.h>
#include <elf.h>
#include "../kvm_test/elf_loader.h"

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

static void build_elf_mapping(pde_t *pgdir, struct elf_info *info)
{
	int i;
	for(i = 0; i < info->nmap; i++){
		boot_map_segment(pgdir, info->mapping[i].addr, info->mapping[i].limit, info->mapping[i].addr, PTE_W);
	}
	tlb_invalidate_all();
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

static int pgfault_handler(struct trapframe *tf){
	uint32_t badaddr = 0;
	if (tf->tf_trapno == T_PABT) {
		badaddr = tf->tf_epc;
	} else {
		badaddr = far();
	}
	__print_hex(badaddr);
	//XXX
	while(1);
	return 0;
}

static void trap_dispatch(struct trapframe *tf)
{
	int ret;
	switch(tf->tf_trapno){
		case T_PABT:
		case T_DABT:
			pgfault_handler(tf);
			break;
		case T_SWI:
			v7_flush_kern_cache_all();
			syscall_passthrough(PADDR(tf));
			v7_flush_kern_dcache_area(tf, sizeof(*tf));
			break;
		case T_IRQ:
		case T_UNDEF:
			__print_hex(0x32423);
			break;
		default:
			break;
	}
}

void trap(struct trapframe *tf){
	trap_dispatch(tf);
}

