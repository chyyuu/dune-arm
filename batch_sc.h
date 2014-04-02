#ifndef BATCH_SC_ARM_H
#define BATCH_SC_ARM_H

struct bsc_request{
	unsigned int nr_syscall;
	unsigned int status;
	unsigned int args[7];
	unsigned int ret;
};

struct bsc_superblk{
	unsigned int len;
	unsigned long long id;
};

#define BSC_SLOT(nr) ((struct bsc_request*)((char*)__bsc_slots + 64 * ((nr)+1)))
#define BSC_SB() ((struct bsc_superblk*)__bsc_slots)
#define BSC_MAX_NR_CALL (4096 / 64 - 1)



#endif /* end of include guard: BATCH_SC_ARM_H */

