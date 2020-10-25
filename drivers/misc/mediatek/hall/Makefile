MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(strip $(project_name)),)
-include $(srctree)/../zprojects/$(project_name)/$(project_name).mk
VANZO_PROJECT_HEADERS:=$(srctree)/../zprojects/$(project_name)/kernel-headers
subdir-ccflags-y += -I$(VANZO_PROJECT_HEADERS)

custom_kernel_hall:= $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_HALL)))
$(info "custom_kernel_hall:$(custom_kernel_hall)")
ifneq ($(custom_kernel_hall),)
obj-y += $(custom_kernel_hall)
else
obj-y += dummy_hall/
endif
endif
