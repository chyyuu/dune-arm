CROSS_COMPILE:=arm-linux-gnueabihf-

CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
OBJCOPY=$(CROSS_COMPILE)objcopy
TARGET:=payload
OBJS:=entry.o init.o vectors.o trapentry.o syscall.o hw.o
#OBJS+= eabi_utils.o
#OBJS+= memcpy.o memset.o 
CFLAGS=-mcpu=cortex-a15 -I../linux_header_313_arm/include -O2 -marm
CFLAGS+=-I../arm-linux-uclibc/usr/include

CFLAGS+=-D__LINUX_ARM_ARCH__=7
LDFLAGS=-nostdlib -static
#LDFLAGS+=-L/usr/arm-linux-gnueabihf/lib -L/usr/lib/gcc/arm-linux-gnueabihf/4.6
LDFLAGS+=-L../arm-linux-uclibc/usr/lib -L/usr/lib/gcc/arm-linux-gnueabihf/4.6

HEADERS=$(shell echo *.h)

all: $(TARGET)

install: $(TARGET)
	cp $(TARGET) ../kvm_test
.PHONY: install

$(TARGET): $(OBJS)
	$(LD) $(LDFLAGS) -T payload.ld -o $@ $^ -lc -lgcc

syscall.o: syscall.S
	$(CC) -D__ASSEMBLY__ -c -o $@ $(CFLAGS) $<

%.o: %.S
	$(CC) -D__ASSEMBLY__ -c -o $@ $(CFLAGS) $<

%.o: %.c $(HEADERS)
	$(CC) -c -o $@ $(CFLAGS) $<

__linker.o: linker
	$(OBJCOPY) --input binary --output elf32-littlearm --binary-architecture arm linker $@

clean:
	rm -f *.o $(TARGET)
.PHONY: clean
