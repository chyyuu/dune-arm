#ifndef LIB_H_MUWN0Z7C
#define LIB_H_MUWN0Z7C

#include <stdint.h>
#include <stddef.h>

void *__memset(void *s, char c, size_t n) __attribute__ ((always_inline));
void *__memmove(void *dst, const void *src, size_t n)
    __attribute__ ((always_inline));
void *__memcpy(void *dst, const void *src, size_t n)
    __attribute__ ((always_inline));

#if 0
//#define memset __memset
#define memcpy __memcpy

static inline void *memset(void *s, int c, size_t n)
{
	unsigned char* p=s;
	while(n--)
		*p++ = (unsigned char)c;
	return s;
}
#endif

/* *
 * Rounding operations (efficient when n is a power of 2)
 * Round down to the nearest multiple of n
 * */
#define ROUNDDOWN(a, n) ({                                          \
            size_t __a = (size_t)(a);                               \
            (typeof(a))(__a - __a % (n));                           \
        })

/* Round up to the nearest multiple of n */
#define ROUNDUP(a, n) ({                                            \
            size_t __n = (size_t)(n);                               \
            (typeof(a))(ROUNDDOWN((size_t)(a) + __n - 1, __n));     \
        })

/* Round up the result of dividing of n */
#define ROUNDUP_DIV(a, n) ({                                        \
            uint32_t __n = (uint32_t)(n);                           \
            (typeof(a))(((a) + __n - 1) / __n);                     \
        })



#endif /* end of include guard: LIB_H_MUWN0Z7C */

