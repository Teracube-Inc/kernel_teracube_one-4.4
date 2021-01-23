#include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(strip $(MTK_K64_SUPPORT)),yes)
obj-y += s5k3l8mipiraw_Sensor.o
else
obj-y += s5k3l8mipiraw_Sensor_32.o
endif
obj-y += s5k3l8mipiraw_pdaf.o

