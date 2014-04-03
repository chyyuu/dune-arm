#ifndef __KERN_MM_MMU_H__
#define __KERN_MM_MMU_H__

/*  MMU definition for ARM9 */
/*  Chen Yuheng 2012/3/20 */

/* PSR flags */
#define PSR_N 		1 << 31
#define PSR_Z 		1 << 30
#define PSR_C 		1 << 29
#define PSR_V 		1 << 28
#define PSR_I 		0x00000080
#define PSR_F 		0x00000040
#define PSR_T 		0x00000020

/* MMU activation */
#define CHANGEALLDOM 	0xffffffff
#define ENABLEMMU 		0x00000001
#define ENABLEDCACHE 	0x00000004
#define ENABLEICACHE 	0x00001000
#define ENABLEHIGHEVT	0x00002000
// In ARMv6, this enables new page table format
#define ENABLENEWPT   0x00800000
#define CHANGEMMU 		0x00000001
#define CHANGEDCACHE 	0x00000004
#define CHANGEICACHE 	0x00001000
#define CHANGEHIGHEVT	0x00002000
#define CHANGENEWPT   0x00800000
#define ENABLEWB 		0x00000008
#define CHANGEWB 		0x00000008

#define DOM3CLT 		0x00000001	// Critical value

/* type */
#define FAULT 	0
#define COARSE 	1
#define MASTER	2

/* Pagesize in kb */
#define SECTIONPAGE 1024
#define LARGEPAGE 64
#define SMALLPAGE 4

// A linear address 'la' has a three-part structure as follows:
//
// +-------12-------+--------8-------+---------12----------+
// | L1 MasterTable | L2 Page Table  | Offset within Page  |
// |      Index     |     Index      |                     |
// +----------------+----------------+---------------------+
//  \--- PDX(la) --/ \--- PTX(la) --/ \---- PGOFF(la) ----/
//  \----------- PPN(la) -----------/
//
// The PDX, PTX, PGOFF, and PPN macros decompose linear addresses as shown.
// To construct a linear address la from PDX(la), PTX(la), and PGOFF(la),
// use PGADDR(PDX(la), PTX(la), PGOFF(la)).
#define NPDEENTRY       4096	// page directory entries per page directory (L1 MASTER)
#define NPTEENTRY       256	// page table entries per page table (L2 COARSE)


#define PTSHIFT         20	// log2(PTSIZE)

#define PTXSHIFT        12	// offset of PTX in a linear address
#define PDXSHIFT        20	// offset of PDX in a linear address

// page directory index (L1)
#define PDX(la) ((((uintptr_t)(la)) >> PDXSHIFT) & 0xFFF)

// page table index (L2)
#define PTX(la) ((((uintptr_t)(la)) >> PTXSHIFT) & 0xFF)

// page number field of address
#define PPN(la) (((uintptr_t)(la)) >> PTXSHIFT)

// offset in page
#define PGOFF(la) (((uintptr_t)(la)) & 0xFFF)

// construct linear address from indexes and offset
#define PGADDR(d, t, o) ((uintptr_t)((d) << PDXSHIFT | (t) << PTXSHIFT | (o)))

// address in page table or page directory entry
#define PTE_ADDR(pte)   ((uintptr_t)(pte) & ~0xFFF)
#define PDE_ADDR(pde)   ((uintptr_t)(pde) & ~0x3FF)

// PTE_xxx are the ucore flags
// PTEX_xxx and PTE_LX_xxx are the hardware flags
//#define PTE_STATUS(pte) (pte + 512)

/* page table/directory entry flags used for the bit status PT */
#define PTE_P           0x001	// Present
#define PTE_W           0x002	// Writeable
#define PTE_U           0x004	// User
#define PTE_PWT         0x008	// Write-Through
#define PTE_PCD         0x010	// Cache-Disable
#define PTE_A           0x020	// Accessed
#define PTE_D           0x040	// Dirty
#define PTE_PS          0x080	// Page Size
#define PTE_MBZ         0x180	// Bits must be zero
#define PTE_AVAIL       0xE00	// Available for software use
						// The PTE_AVAIL bits aren't used by the kernel or interpreted by the
						// hardware, so user processes are allowed to set them arbitrarily.
#define PTE_IOMEM       0x10000

#define PTE_USER        (PTE_U | PTE_W | PTE_P)

/* page directory (L1) / table (L2) entry flags */
#define PTEX_P 0x3		/* Present (if one bit of [1:0] is set, page exists */
/* PTE type */
/* AP */
/* NA = no access, RO = read only, RW = read/write */
#define PTEX_NANA 0x00
#define PTEX_RWNA 0x01
#define PTEX_RWRO 0x02
#define PTEX_RWRW 0x03
/* CB */
#define PTEX_cb 0x0		/* cb = not cached/not buffered */
#define PTEX_cB 0x1		/* cB = not cached/Buffered */
#define PTEX_WT 0x2		/* WT = Write Through cache */
#define PTEX_WB 0x3		/* WB = write back cache */
/* Typical */
#define PTEX_PWT (PTEX_WT << 2)	// Write Through
#define PTEX_PIO (PTEX_cb << 2)	// Write Through
#define PTEX_PWB (PTEX_WB << 2)	// Write back

/* ARMv7 */
#define PTEX_R   0x210		// Supervisor/Readonly
#define PTEX_W   0x010		// Supervisor/Write
#define PTEX_U   0x020		// Supervisor/Write _ User/Read Only
#define PTEX_UW  0x030		// Supervisor/Write _ User/Write

/* Chen Yuheng */
#define PTEX_PROTECT_MASK 0xFF0
#define PTEX_CB_MASK 0xC
#define PTEX_L1_PDTYPE 0x1	//fine
#define PTEX_L2_PGTYPE 0x2	//small page

#ifndef __ASSEMBLER__

#include <stdint.h>
#include <stddef.h>

typedef uintptr_t pte_t;
typedef uintptr_t pde_t;
typedef uint32_t pte_perm_t;

inline static void v7_flush_icache_all(){
	asm volatile(
		"mov r0, #0;"
		"mcr p15, 0, r0, c7, c1, 0;"
		"mcr p15, 0, r0, c7, c5, 0;"
		::);
}

#define isb() __asm__ __volatile__ ("isb" : : : "memory")
#define dsb() __asm__ __volatile__ ("dsb" : : : "memory")
#define dmb() __asm__ __volatile__ ("dmb" : : : "memory")

inline static void ttbSet(uint32_t ttb)
{
	//flush_clean_cache();
	ttb &= 0xffffc000;
	asm volatile ("MCR p15, 0, %0, c2, c0, 0"	/* set translation table base */
		      ::"r" (ttb)
	    );
	isb();
}

inline static void tlb_invalidate_all()
{
	// tlb_invalidate(0,0);
	const int zero = 0;
	dsb();
	asm volatile (
			"MCR p15, 0, %0, c8, c7, 0;"	/* invalidate TLB */
		      "MCR p15, 0, %0, c8, c6, 0;"	/* invalidate TLB */
			"MCR p15, 0, %0, c8, c5, 0;"	/* invalidate TLB */
		      "mcr p15, 0, %0, c7, c5, 6"
		      ::"r" (zero):"cc");
	dsb();
	isb();
}

static inline uint32_t far(void)
{
	uint32_t c6format;
	asm volatile ("MRC p15, 0, %0, c6, c0, 0;"	//read data in fault address register
		      :"=r" (c6format));
	return c6format;
}


// L2 PTE setter
// Set the ucore flags directly, and the hardware flags under condition
// flags are PTE_xxx
static inline void _arm_pte_setflags(pte_t * pte, pte_perm_t flags)
{
	// hardware flags
	//*pte &= 0xFFFFF000;
	if (flags & PTE_P)
		*pte |= PTEX_L2_PGTYPE;
	*pte &= ~PTEX_PROTECT_MASK;

	if (flags & PTE_U) {
		if (flags & PTE_W)	// Write permission is accorded only if page is writeable and dirty
		{
			*pte |= PTEX_UW;
		} else {
			*pte |= PTEX_U;
		}
	} else {
		if (flags & PTE_W)
			*pte |= PTEX_W;	// kernel will be read/write or no access
		else
			*pte |= PTEX_R;	//kernel readonly
	}

	if (flags & PTE_PWT) {
		*pte &= ~PTEX_CB_MASK;
		*pte |= PTEX_PWT;
	} else if (flags & PTE_IOMEM) {
		*pte &= ~PTEX_CB_MASK;
		*pte |= PTEX_PIO;
	}else{
		*pte &= ~PTEX_CB_MASK;
		//*pte |= PTEX_PWT;
		//writealloc, shared
		*pte |= (1<<6) | PTEX_PWB | (1<<10);
	}

}

#define _SET_PTE_BITS(pte, perm) do{pte_perm_t oldf = *(PTE_STATUS(pte));\
                _arm_pte_setflags(pte, oldf|perm);}while(0);

//Assuming cr1 SR = 1 0
static inline void ptep_map(pte_t * ptep, uintptr_t pa)
{
	//*ptep = pa|(PTEX_WB<<2);
	//*ptep = pa | (PTEX_PWT << 2);	//write through cache by default
	*ptep = pa | PTEX_L2_PGTYPE;
	//_arm_pte_setflags(ptep, PTE_P);
}

static inline void pdep_map(pde_t * pdep, uintptr_t pa)
{
	*pdep = pa | PTEX_L1_PDTYPE;
}

static inline void ptep_unmap(pte_t * ptep)
{
	*ptep = 0;
}

static inline int ptep_invalid(pte_t * ptep)
{
	return (*ptep == 0);
}

static inline int ptep_present(pte_t * ptep)
{
	return (*ptep & PTEX_P);
}

static inline int ptep_s_read(pte_t * ptep)
{
//      return (*ptep & PTEX_W);
	return 1;
}

static inline int ptep_s_write(pte_t * ptep)
{
	uint32_t p = *ptep & PTEX_PROTECT_MASK;
	return (p != PTEX_R);
}

static inline int ptep_s_exec(pte_t * ptep)
{
//      return (*ptep & PTE_E);
	return 1;
}

/* user readable */
static inline int ptep_u_read(pte_t * ptep)
{
	//return (*ptep & PTEX_U);
	uint32_t p = *ptep & PTEX_PROTECT_MASK;
	return (p == PTEX_U) || (p == PTEX_UW);
}

/* user writable */
static inline int ptep_u_write(pte_t * ptep)
{
	uint32_t p = *ptep & PTEX_PROTECT_MASK;
	return (p == PTEX_UW);
	//return (*ptep & PTEX_UW);
}

static inline int ptep_u_exec(pte_t * ptep)
{
//      return (*ptep & PTE_E);
	return ptep_u_read(ptep);
}

static inline void ptep_set_s_read(pte_t * ptep)
{
	//*ptep |= PTE_SPR_R;
	//supervisor mode always readable
}

static inline void ptep_set_s_write(pte_t * ptep)
{
	//if(!ptep_u_read(ptep))
	*ptep |= PTE_W;
	//_SET_PTE_BITS(ptep, PTE_W);
}

static inline void ptep_set_s_exec(pte_t * ptep)
{
	//*ptep |= PTE_E;
}

static inline void ptep_set_u_read(pte_t * ptep)
{
	//_SET_PTE_BITS(ptep, PTE_U);
	*ptep |= PTE_U;
}

static inline void ptep_set_u_write(pte_t * ptep)
{
	//*ptep |= PTEX_UW|PTEX_W;
	//_SET_PTE_BITS(ptep, PTE_U|PTE_W);
	*ptep |= PTE_U | PTE_W;
}

static inline void ptep_set_perm(pte_t * ptep, pte_perm_t perm)
{
	//*PTE_STATUS(ptep) |= perm;
	_arm_pte_setflags(ptep, perm);
}

#endif

#endif /* !__KERN_MM_MMU_H__ */
