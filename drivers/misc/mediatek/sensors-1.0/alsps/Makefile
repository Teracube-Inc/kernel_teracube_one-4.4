# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include

obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

ifeq ($(CONFIG_MTK_ALSPSHUB),y)
obj-y      +=  alspshub/
else
# Stoneoim:yucheng on: Sat, 15 Oct 2016 12:50:45 +0800
project_name:=$(shell echo $(CONFIG_VANZO_INNER_PROJECT_NAME))
ifneq ($(project_name),)
custom_kernel_alsps:= $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_ALSPS_NAME)))
$(info "custom_kernel_alsps:$(custom_kernel_alsps)")
ifneq ($(custom_kernel_alsps),)
obj-y += $(custom_kernel_alsps)
endif
else
# End of Stoneoim: yucheng
obj-$(CONFIG_MTK_CM36558)	+=  cm36558/
obj-$(CONFIG_MTK_LTR579_NEW)	+=  ltr579-new/
obj-$(CONFIG_MTK_LTR578)	+=  ltr578/
obj-$(CONFIG_MTK_EPL2182)	+=  epl2182/
obj-$(CONFIG_MTK_LTR303)	+=  ltr303/
# Stoneoim:yucheng on: Sat, 15 Oct 2016 12:51:46 +0800
# added for new structure config alsps
endif
# End of Stoneoim: yucheng
endif
