obj-m := ec_xenomai_sample.o

#ec_xenomai_sample-objs := main.o libethercat_rtdm.a
ec_xenomai_sample-objs := main.o 

#EXTRA_CFLAGS := -I/include
EXTRA_CFLAGS	:= -I/usr/xenomai/include -I/usr/include/ -I/opt/etherlab/include/ 

KBUILD_EXTRA_SYMBOLS := \
	/usr/src/realtime/igh-ethercat/$(LINUX_SYMVERS) 
#	/usr/src/realtime/igh-ethercat/master/$(LINUX_SYMVERS)