obj-m		:= uart-omap_rtdm.o
KDIR		:= /root/embedded_linux/elabs_BBB/beagle-kernel/kernel
PWD		:= $(shell pwd)
EXTRA_CFLAGS    := -I/usr/xenomai/include -I/usr/include/

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean




#obj-m           := 101ass.o
#KDIR            := /root/embedded_linux/elabs_xeno_ws/linux-3.2.21
#PWD             := $(shell pwd)
#EXTRA_CFLAGS    := -I/usr/xenomai/include -I/usr/include/

#all:
#	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
#clean:
#	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

