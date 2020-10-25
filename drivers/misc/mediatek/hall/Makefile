MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

project_name:=$(shell echo $(CONFIG_VANZO_INNER_PROJECT_NAME))
ifneq ($(strip $(project_name)),)

ifneq ($(strip $(CUSTOM_KERNEL_HALL)),)
CONFIG_CUSTOM_KERNEL_HALL:=$(CUSTOM_KERNEL_HALL)
endif

custom_kernel_hall:= $(addprefix hall/,$(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_HALL))))

$(info "custom_kernel_hall:$(custom_kernel_hall)")
ifneq ($(custom_kernel_hall),)
obj-y += $(custom_kernel_hall)
else
obj-y += dummy_hall/
endif
endif
