
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)

#include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_vfs
ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_driver/include/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/core

obj-y += sunwave_fp.o
sunwave_fp-objs          := sf_ctl.o sf_spi.o

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else

#KBUILDDIR := $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ
KBUILDDIR ?= /opt/sunwave/longcheer/kernel-3.18/obj
MODULEPWD := $(shell pwd)

.PHONY: all install clean

all:
	$(MAKE) -C $(KBUILDDIR) ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- \
		M=$(MODULEPWD) CONFIG_SUNWAVE_FP=m modules

install:
	adb remount
	adb push sunwave_fp.ko /data/
	adb shell sync

clean:
	rm -rf *.o *.mod.c .*.cmd .*tmp_versions *.symvers *.order 

endif # End of KERNELRELEASE.

