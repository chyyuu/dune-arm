#ifndef PARAM_H_FREVBYYC
#define PARAM_H_FREVBYYC

#define PAGE_OFFSET		0xc0000000
#define TEXT_OFFSET		0x8000
#define PHYS_OFFSET		0xc0000000

//XXX
#define SDRAM0_START PHYS_OFFSET
#define PGSHIFT 12
#define PAGE_SIZE (1<<PGSHIFT)
#define KSTACKPAGE          2	// # of pages in kernel stack
#define KSTACKSIZE          (KSTACKPAGE * PAGE_SIZE)	// sizeof kernel stack
#define KMEMSIZE    (64 << 20)
#define IO_SPACE_START 0xf0000000
#define IO_SPACE_SIZE  0x00100000

#if 0
#define __virt_to_phys(x)	((x) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt(x)	((x) - PHYS_OFFSET + PAGE_OFFSET)
#else
#define __virt_to_phys(x)	(x)
#define __phys_to_virt(x)	(x)
#endif

#define KADDR(pa) __phys_to_virt((uintptr_t)pa)
#define PADDR(va) __virt_to_phys((uintptr_t)va)

#if (SDRAM0_START & 0x001fffff)
#error "SDRAM0_START must be at an even 2MiB boundary!"
#endif

#define MAX_PAGES (1ul << 20) /* 4g */

#endif /* end of include guard: PARAM_H_FREVBYYC */

