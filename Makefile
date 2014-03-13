CROSS_COMPILE:=arm-linux-gnueabihf-

CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
OBJCOPY=$(CROSS_COMPILE)objcopy
TARGET:=payload
OBJS:=entry.o memcpy.o memset.o init.o eabi_utils.o vectors.o trapentry.o
CFLAGS=-mcpu=cortex-a15 -I../linux_header_313_arm/include -O2

CFLAGS+=-D__LINUX_ARM_ARCH__=7
LDFLAGS=-nostdlib

HEADERS=$(shell echo *.h)

all: $(TARGET)

install: $(TARGET)
	cp $(TARGET) ../kvm_test
.PHONY: install

$(TARGET): $(OBJS)
	$(LD) $(LDFLAGS) -T payload.ld -o $@ $^

%.o: %.S
	$(CC) -D__ASSEMBLY__ -c -o $@ $(CFLAGS) $<

%.o: %.c $(HEADERS)
	$(CC) -c -o $@ $(CFLAGS) $<

clean:
	rm -f *.o $(TARGET)
.PHONY: clean
