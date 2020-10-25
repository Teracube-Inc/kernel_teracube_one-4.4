#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_IMGSENSOR_USE_MODULE_DRIVER),y)
$(info imx135_otp use module build)
obj-m += imx135_otp.o
else
obj-y += imx135_otp.o
endif

obj-y += imx135mipiraw_Sensor.o


