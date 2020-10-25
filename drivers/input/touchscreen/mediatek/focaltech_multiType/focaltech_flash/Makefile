#drivers/input/touchscreen/mediatek/focaltech_ts/focaltech_flash
#
#Makefile for test module
#
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base

ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT6336U)),yes)
    obj-y += focaltech_upgrade_ft6336gu.o
else 
    ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT6336G)),yes)
        obj-y += focaltech_upgrade_ft6336gu.o
    else 
        ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT5422U)),yes)
            obj-y += focaltech_upgrade_ft5422u.o
        else 
            ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT5822)),yes)
                obj-y += focaltech_upgrade_ft5822.o
            else 
                ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT5X46)),yes)
                    obj-y += focaltech_upgrade_ft5x46.o
                else 
                    ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT8006)),yes)
                        obj-y += focaltech_upgrade_ft8006.o
                    else 
                        ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT8006U)),yes)
                            obj-y += focaltech_upgrade_ft8006u.o
                        else 
                            ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT8606)),yes)
                                obj-y += focaltech_upgrade_ft8606.o
                            else 
                                ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT8607)),yes)
                                    obj-y += focaltech_upgrade_ft8607.o
                                else 
                                    ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT8716)),yes)
                                        obj-y += focaltech_upgrade_ft8716.o
                                    else 
                                        ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT8719)),yes)
                                            obj-y += focaltech_upgrade_ft8719.o
                                        else 
                                            ifeq ($(strip $(VANZO_FTS_CHIP_TYPE_FT8736)),yes)
                                                obj-y += focaltech_upgrade_ft8736.o
                                            else 
                                                obj-y += focaltech_upgrade_ft3427.o
                                            endif
                                        endif
                                    endif
                                endif
                            endif
                        endif
                    endif
                endif
            endif
       endif
    endif
endif
