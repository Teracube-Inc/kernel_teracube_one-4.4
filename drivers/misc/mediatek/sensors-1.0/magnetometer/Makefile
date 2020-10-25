# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/magnetometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  mag.o
obj-y   +=  mag_factory.o

ifeq ($(CONFIG_MTK_MAGHUB),y)
obj-y   +=  maghub/
else
# Stoneoim:yucheng on: Mon, 13 Mar 2017 16:38:03 +0800
project_name:=$(shell echo $(CONFIG_VANZO_INNER_PROJECT_NAME))
ifneq ($(project_name),)
custom_kernel_magnetometer:= $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_MAGNETOMETER_NAME)))
$(info "custom_kernel_magnetometer:$(custom_kernel_magnetometer)")
ifneq ($(custom_kernel_magnetometer),)
obj-y += $(custom_kernel_magnetometer)
endif
else
# End of Stoneoim: yucheng
obj-$(CONFIG_MTK_AKM09911)      +=  akm09911/
obj-$(CONFIG_MTK_ST480)         +=  st480/
obj-$(CONFIG_MTK_AKM09918)      +=  akm09918/
obj-$(CONFIG_MTK_MMC3680X)	+=  mmc3680x/
obj-$(CONFIG_MTK_MMC5603X)	+=  mmc5603x/
obj-$(CONFIG_MTK_QMCX983)	+=  qmcX983/
obj-$(CONFIG_MTK_AKM09911)      +=  akm09911/
obj-$(CONFIG_MTK_ST480)         +=  st480/
obj-$(CONFIG_MTK_AKM09918)      +=  akm09918/
obj-$(CONFIG_MTK_MMC3630X)      +=  mmc3630/
# Stoneoim:yucheng on: Mon, 13 Mar 2017 16:38:32 +0800
# TODO: replace this line with your comment
# added for new structure config alsps
endif
# End of Stoneoim: yucheng
endif
