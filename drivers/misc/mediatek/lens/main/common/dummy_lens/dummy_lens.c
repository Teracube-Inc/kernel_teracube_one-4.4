/*
 * MD218A voice coil motor driver
 *
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

/* kernel standard for PMIC*/
#if !defined(CONFIG_MTK_LEGACY)
#include <linux/regulator/consumer.h>
#endif

/* OIS/EIS Timer & Workqueue */
#include <linux/init.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>


/* #define DUMMY_LENS_DEBUG */
#ifdef DUMMY_LENS_DEBUG
#define DUMMY_LENSDB pr_debug
#else
#define DUMMY_LENSDB(x, ...)
#endif

static int __init DUMMY_LENS_i2C_init(void)
{
	return 0;
}

static void __exit DUMMY_LENS_i2C_exit(void)
{

}
module_init(DUMMY_LENS_i2C_init);
module_exit(DUMMY_LENS_i2C_exit);
