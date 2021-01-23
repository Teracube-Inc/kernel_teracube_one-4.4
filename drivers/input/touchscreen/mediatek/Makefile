#subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include

# Stoneoim:wangfei on: Wed, 12 Nov 2014 21:08:36 +0800
# added for aosp management to import our variable
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(strip $(project_name)),)
-include $(srctree)/../zprojects/$(project_name)/$(project_name).mk
VANZO_PROJECT_HEADERS:=$(srctree)/../zprojects/$(project_name)/kernel-headers
ccflags-y += -I$(VANZO_PROJECT_HEADERS)
endif
# End of Stoneoim:wangfei

# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

# Stoneoim:wangfei on: Wed, 12 Nov 2014 21:06:42 +0800
# for compatible the old style ,so use CUSTOM_KERNEL_IMGSENSOR
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(project_name),)

ifneq ($(strip $(BOOT_LOGO)),)
boot_logo := $(shell echo $(BOOT_LOGO) | tr '[a-z]' '[A-Z]')
$(info boot_logo = $(boot_logo))
subdir-ccflags-y += -D$(boot_logo)
endif

custom_kernel_touchpanel:= $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_TOUCHPANEL)))

$(info "custom_kernel_touchpanel:$(custom_kernel_touchpanel)")
obj-y += $(custom_kernel_touchpanel)
else
# End of Stoneoim:wangfei

obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT5688)	+=  GT5688/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_LM)	+=  GT9XX_LM/
obj-$(CONFIG_TOUCHSCREEN_MTK_GSLX680)	+=  gslX680/
obj-$(CONFIG_TOUCHSCREEN_FT5X26)	+=  FT5X26/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET)        +=  hxchipset/
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS)        +=  focaltech_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  NT36xxx/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  GSL1691/

# Stoneoim:wangfei on: Sat, 28 Feb 2015 16:22:43 +0800
# for use new style to config touchscreen
endif
# End of Stoneoim:wangfei

