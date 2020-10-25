# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  gyroscope.o gyro_factory.o

ifeq ($(CONFIG_MTK_GYROHUB),y)
obj-y      +=  gyrohub/
else
# Stoneoim:yucheng on: Wed, 12 Nov 2014 21:06:42 +0800
project_name:=$(shell echo $(CONFIG_VANZO_INNER_PROJECT_NAME))
ifneq ($(project_name),)
custom_kernel_gyroscope:= $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_GYROSCOPE)))
ifneq ($(custom_kernel_gyroscope),)
obj-y += $(custom_kernel_gyroscope)
endif
else
# End of Stoneoim:yucheng
obj-$(CONFIG_MTK_LSM6DS3HGY)   +=  lsm6ds3hgy/
obj-$(CONFIG_MTK_BMI160GY_I2C) +=  bmi160gy-i2c/
obj-$(CONFIG_MTK_ICM206XX_G) +=  icm206xx_g/
obj-$(CONFIG_MTK_ITG1010)   +=  ITG1010/
obj-$(CONFIG_MTK_MPU6050GY)   +=  mpu6050gy/
# Stoneoim:yucheng on: Tue, 06 Jan 2015 10:34:15 +0800
# added for new structure config alsps
endif
# End of Stoneoim:yucheng
endif
