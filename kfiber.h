#ifndef KFIBER_H__
#define KFIBER_H__

#define __kfiber_SYSCALL_BASE 0x1000
#define __NR_kfiber_create    (__kfiber_SYSCALL_BASE + 1)
#define __NR_kfiber_join      (__kfiber_SYSCALL_BASE + 2)
#define __NR_kfiber_yield     (__kfiber_SYSCALL_BASE + 3)
#define __NR_kfiber_exit      (__kfiber_SYSCALL_BASE + 4)

#endif /* end of include guard: KFIBER_H__ */

