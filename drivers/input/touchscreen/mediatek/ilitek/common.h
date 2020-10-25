/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

 #ifndef __COMMON_H
 #define __COMMON_H

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>

#include <linux/namei.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#else 
#include <linux/earlysuspend.h>
#endif

/*
 * Relative Driver with Touch IC
 */
/* Touch IC support */
#define CHIP_TYPE_ILI7807	0x7807
#define CHIP_TYPE_ILI9881	0x9881
#define TP_TOUCH_IC		CHIP_TYPE_ILI9881	

/* Platform support */
#define PT_RK	1 
#define PT_MTK	2
#define PT_SPRD	3
#define PT_QCOM	4
#define TP_PLATFORM PT_MTK

/* Driver version */
#define DRIVER_VERSION	"1.0.1.3_prepare_modify"

/* Protocol version */
#define PROTOCOL_MAJOR		0x5
#define PROTOCOL_MID		0x1
#define PROTOCOL_MINOR		0x0

/* Normal debug messages */
#define DBG_INFO(fmt, arg...) \
			pr_info("ILITEK: (%s, %d): " fmt , __func__, __LINE__, ##arg);

#define DBG_ERR(fmt, arg...) \
			pr_err("ILITEK: (%s, %d): " fmt , __func__, __LINE__, ##arg);

/* Detailed debug messages */
#ifdef BIT
#undef BIT
#endif
#define BIT(x)	(1 << (x))

enum {
	DEBUG_NONE = 0,
	DEBUG_IRQ = BIT(0),
	DEBUG_FINGER_REPORT = BIT(1),
	DEBUG_FIRMWARE = BIT(2),
	DEBUG_CONFIG = BIT(3),
	DEBUG_I2C = BIT(4),
	DEBUG_BATTERY = BIT(5),
	DEBUG_MP_TEST = BIT(6),
	DEBUG_IOCTL= BIT(7),
	DEBUG_NETLINK = BIT(8),
	DEBUG_PARSER = BIT(9),
	DEBUG_GESTURE = BIT(10),
	DEBUG_ALL = ~0,
};

#define DBG(level, fmt, arg...) \
			do { \
				if (level & ipio_debug_level) \
				pr_info( "ILITEK: (%s, %d): " fmt, __func__, __LINE__, ##arg); \
			} while (0)

/* Distributed to all core functions */
extern uint32_t ipio_debug_level;
extern uint32_t ipio_chip_list[2];

/* Macros */
#define CHECK_EQUAL(X,Y) ((X==Y) ? 0 : -1 )
#define ERR_ALLOC_MEM(X)	((IS_ERR(X) || X == NULL) ? 1 : 0)
#define USEC	1
#define MSEC	(USEC * 1000)

/* The size of firmware upgrade */
#define MAX_HEX_FILE_SIZE			(160*1024)
#define MAX_FLASH_FIRMWARE_SIZE		(256*1024)
#define MAX_IRAM_FIRMWARE_SIZE		(60*1024)

/* ILI7807 Series */
#define ILI7807_TYPE_F_AA		0x0000
#define ILI7807_TYPE_F_AB		0x0001
#define ILI7807_TYPE_H			0x1100

#define ILI7807_SLAVE_ADDR		0x41
#define ILI7807_ICE_MODE_ADDR	0x181062
#define ILI7807_PID_ADDR		0x4009C

/* ILI9881 Series */
#define ILI9881_SLAVE_ADDR		0x41
#define ILI9881_ICE_MODE_ADDR	0x181062
#define ILI9881_PID_ADDR		0x4009C

/*
 * Other settings
 */
#define CSV_PATH	"/sdcard"
#define INI_NAME_PATH	"/sdcard/mp.ini"

 /* define the width and heigth of a screen. */
#define TOUCH_SCREEN_X_MIN 0
#define TOUCH_SCREEN_Y_MIN 0
#define TOUCH_SCREEN_X_MAX 720
#define TOUCH_SCREEN_Y_MAX 1440

/* define the range on panel */
#define TPD_HEIGHT 2048
#define TPD_WIDTH 2048

/* define the size of window of phone cover */
#define UL_X_LOW	0
#define UL_X_HIGH	100
#define UL_Y_LOW	0
#define UL_Y_HIGH	100
#define BR_X_LOW	0
#define BR_X_HIGH	100
#define BR_Y_LOW	0
#define BR_Y_HIGH	100	
 
/* How many numbers of touch are supported by IC. */
#define MAX_TOUCH_NUM	10

/* Linux multiple touch protocol, either B type or A type. */
#define MT_B_TYPE

/* Enable the support of regulator power. */
#define REGULATOR_POWER_ON 

/* Either an interrupt event handled by kthread or work queue. */
#define USE_KTHREAD

/* Enable DMA with I2C. */
//#define I2C_DMA

/* Split the length written to or read from IC via I2C. */
//#define I2C_SEGMENT

/* Be able to upgrade fw at boot stage */
//#define BOOT_FW_UPGRADE

/* Check battery's status in order to avoid some effects from charge. */
//#define BATTERY_CHECK

#endif /* __COMMON_H */
