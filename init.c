#include "lib.h"
#include "mmu.h"
#include "trap.h"
#include "param.h"
#include <string.h>
#include <sys/queue.h>

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
	memcpy((void *)0, (void *)&__vector_table,
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

	int start = (end_pa + ROUNDUP(MAX_PAGES*sizeof(struct page), PAGE_SIZE)) / PAGE_SIZE;
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



void boot_pg_init(){
	boot_pgdir = (pde_t*)ROUNDUP((uintptr_t)__boot_pgtlb, 4*PAGE_SIZE);
	memset(boot_pgdir, 0, 4*PAGE_SIZE);

	boot_map_segment(boot_pgdir, PAGE_OFFSET, num_kern_pages * PAGE_SIZE,
			PHYS_OFFSET, PTE_W);

	boot_map_segment(boot_pgdir, IO_SPACE_START, IO_SPACE_SIZE, IO_SPACE_START, PTE_W | PTE_IOMEM);

	boot_map_segment(boot_pgdir, 0xFFFF0000, PAGE_SIZE, SDRAM0_START, PTE_PWT | PTE_W );	// high location of vector table

	ttbSet((uint32_t)PADDR(boot_pgdir));
	//ttbSet(0x4000);

	tlb_invalidate_all();
	__print_hex(0x124);

}

void kern_init(){
	clear_bss();

	exception_vector_init();
	//*(int*)0xf0000000 = 0x1236;

	page_init();

	boot_pg_init();

	//switch to sys mode

	write(1, "A",1);
	*(int*)0= 0x1238;
	while(1);
	return;
}

static void trap_dispatch(struct trapframe *tf)
{
	switch(tf->tf_trapno){
		case T_PABT:
		case T_DABT:
		case T_SWI:
		case T_IRQ:
		case T_UNDEF:
		default:
			break;
	}
}

void trap(struct trapframe *tf){
	*(int*)0xf0000000 = 0x1237;
	trap_dispatch(tf);
	while(1);
}

