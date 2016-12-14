obj-m	:= main.o	
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)
EXTRA_CFLAGS	:= -I/usr/xenomai/include -I/usr/include/ -I/opt/etherlab/include/ 

all:
	@$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules 
	@cp /usr/src/realtime/igh-ethercat/Module.symvers ./
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules 
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean
	
