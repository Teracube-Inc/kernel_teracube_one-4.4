/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <mtk_sleep.h>
#include <mtk_spm_idle.h>
#include <mt-plat/upmu_common.h>
#include "include/pmic_api_buck.h"
#include <mtk_spm_vcore_dvfs.h>
#include <mtk_spm_internal.h>
#ifdef CONFIG_MTK_DRAMC
#include <mtk_dramc.h>
#endif
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/irqchip/mtk-eic.h>
#include <linux/suspend.h>
#include <mt-plat/mtk_secure_api.h>
#ifdef CONFIG_MTK_WD_KICKER
#include <mach/wd_api.h>
#endif

#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <mtk_spm_misc.h>
#include <mtk_spm_resource_req_internal.h>

#include <trace/events/mtk_events.h>

__weak int get_spm_last_wakeup_src(struct seq_file *s, void *unused)
{
	return 0;
}
__weak int get_spm_sleep_count(struct seq_file *s, void *unused)
{
	return 0;
}

int spm_for_gps_flag;
static struct dentry *spm_dir;
static struct dentry *spm_file;

void __iomem *spm_base;
u32 spm_irq_0;

#define NF_EDGE_TRIG_IRQS	7
static u32 edge_trig_irqs[NF_EDGE_TRIG_IRQS];

/**************************************
 * Config and Parameter
 **************************************/

/**************************************
 * Define and Declare
 **************************************/
struct spm_irq_desc {
	unsigned int irq;
	irq_handler_t handler;
};

static twam_handler_t spm_twam_handler;

void __attribute__((weak)) spm_sodi3_init(void)
{
	pr_info("NO %s !!!\n", __func__);
}

void __attribute__((weak)) spm_sodi_init(void)
{
	pr_info("NO %s !!!\n", __func__);
}

void __attribute__((weak)) spm_deepidle_init(void)
{
	pr_info("NO %s !!!\n", __func__);
}

void __attribute__((weak)) spm_vcorefs_init(void)
{
	pr_info("NO %s !!!\n", __func__);
}

void __attribute__((weak)) mt_power_gs_t_dump_suspend(int count, ...)
{
	pr_info("NO %s !!!\n", __func__);
}

int __attribute__((weak)) spm_fs_init(void)
{
	pr_info("NO %s !!!\n", __func__);
	return 0;
}

/**************************************
 * Init and IRQ Function
 **************************************/
static irqreturn_t spm_irq0_handler(int irq, void *dev_id)
{
	u32 isr;
	unsigned long flags;
	struct twam_sig twamsig;

	spin_lock_irqsave(&__spm_lock, flags);
	/* get ISR status */
	isr = spm_read(SPM_IRQ_STA);
	if (isr & ISRS_TWAM) {
		twamsig.sig0 = spm_read(SPM_TWAM_LAST_STA0);
		twamsig.sig1 = spm_read(SPM_TWAM_LAST_STA1);
		twamsig.sig2 = spm_read(SPM_TWAM_LAST_STA2);
		twamsig.sig3 = spm_read(SPM_TWAM_LAST_STA3);
		udelay(40); /* delay 1T @ 32K */
	}

	/* clean ISR status */
	mt_secure_call(MTK_SIP_KERNEL_SPM_IRQ0_HANDLER, isr, 0, 0);
	spin_unlock_irqrestore(&__spm_lock, flags);

	if ((isr & ISRS_TWAM) && spm_twam_handler)
		spm_twam_handler(&twamsig);

	if (isr & (ISRS_SW_INT0 | ISRS_PCM_RETURN))
		spm_err("IRQ0 HANDLER SHOULD NOT BE EXECUTED (0x%x)\n", isr);

	return IRQ_HANDLED;
}

static int spm_irq_register(void)
{
	int i, err, r = 0;
	struct spm_irq_desc irqdesc[] = {
		{.irq = 0, .handler = spm_irq0_handler,}
	};
	irqdesc[0].irq = SPM_IRQ0_ID;
	for (i = 0; i < ARRAY_SIZE(irqdesc); i++) {
		if (cpu_present(i)) {
			err = request_irq(irqdesc[i].irq, irqdesc[i].handler,
					IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_PERCPU, "SPM", NULL);
			if (err) {
				spm_err("FAILED TO REQUEST IRQ%d (%d)\n", i, err);
				r = -EPERM;
			}
		}
	}
	return r;
}

static void spm_register_init(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,sleep");
	if (!node)
		spm_err("find SLEEP node failed\n");
	spm_base = of_iomap(node, 0);
	if (!spm_base)
		spm_err("base spm_base failed\n");

	spm_irq_0 = irq_of_parse_and_map(node, 0);
	if (!spm_irq_0)
		spm_err("get spm_irq_0 failed\n");

	spm_err("spm_base = %p, spm_irq_0 = %d\n", spm_base, spm_irq_0);

	/* kp_irq_b */
#if defined(CONFIG_MACH_MT6759) || defined(CONFIG_MACH_MT6758) || \
	defined(CONFIG_MACH_MT6775)
	node = of_find_compatible_node(NULL, NULL, "mediatek,kp");
	if (!node) {
		spm_err("find keypad node failed\n");
	} else {
		edge_trig_irqs[2] = irq_of_parse_and_map(node, 0);
		if (!edge_trig_irqs[2])
			spm_err("get keypad failed\n");
	}
#else
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6799-keypad");
	if (!node) {
		spm_err("find mt6799-keypad node failed\n");
	} else {
		edge_trig_irqs[2] = irq_of_parse_and_map(node, 0);
		if (!edge_trig_irqs[2])
			spm_err("get mt6799-keypad failed\n");
	}
#endif
	/* c2k_wdt_irq_b */
	node = of_find_compatible_node(NULL, NULL, "mediatek,ap2c2k_ccif");
	if (!node) {
		spm_err("find ap2c2k_ccif node failed\n");
	} else {
		edge_trig_irqs[4] = irq_of_parse_and_map(node, 1);
		if (!edge_trig_irqs[4])
			spm_err("get ap2c2k_ccif failed\n");
	}

	/* md_wdt_int_ao */
	node = of_find_compatible_node(NULL, NULL, "mediatek,mdcldma");
	if (!node) {
		spm_err("find mdcldma node failed\n");
	} else {
		edge_trig_irqs[5] = irq_of_parse_and_map(node, 2);
		if (!edge_trig_irqs[5])
			spm_err("get mdcldma failed\n");
	}
#if !defined(CONFIG_MACH_MT6759) && !defined(CONFIG_MACH_MT6758) && \
	!defined(CONFIG_MACH_MT6775)

	/* lowbattery_irq_b */
	node = of_find_compatible_node(NULL, NULL, "mediatek,auxadc");
	if (!node) {
		spm_err("find auxadc node failed\n");
	} else {
		edge_trig_irqs[6] = irq_of_parse_and_map(node, 0);
		if (!edge_trig_irqs[6])
			spm_err("get auxadc failed\n");
	}
#endif
	spm_err("edge trigger irqs: %d, %d, %d, %d, %d, %d, %d\n",
		 edge_trig_irqs[0],
		 edge_trig_irqs[1],
		 edge_trig_irqs[2],
		 edge_trig_irqs[3],
		 edge_trig_irqs[4],
		 edge_trig_irqs[5],
		 edge_trig_irqs[6]);
#if !defined(CONFIG_MACH_MT6759) && !defined(CONFIG_MACH_MT6758) && \
	!defined(CONFIG_MACH_MT6775)
	spm_set_dummy_read_addr(true);
#endif
}

static int local_spm_load_firmware_status = -1;
int spm_load_firmware_status(void)
{
	if (local_spm_load_firmware_status == -1)
		local_spm_load_firmware_status =
			mt_secure_call(MTK_SIP_KERNEL_SPM_FIRMWARE_STATUS, 0, 0, 0);

	return local_spm_load_firmware_status;
}

static int spm_sleep_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, get_spm_sleep_count, &inode->i_private);
}

static const struct file_operations spm_sleep_count_fops = {
	.open = spm_sleep_count_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int spm_last_wakeup_src_open(struct inode *inode, struct file *file)
{
	return single_open(file, get_spm_last_wakeup_src, &inode->i_private);
}

static const struct file_operations spm_last_wakeup_src_fops = {
	.open = spm_last_wakeup_src_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#if !defined(CONFIG_FPGA_EARLY_PORTING)
#ifdef CONFIG_PM
static int spm_pm_event(struct notifier_block *notifier, unsigned long pm_event,
			void *unused)
{
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	struct spm_data spm_d;
	int ret;
	unsigned long flags;
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:
		return NOTIFY_DONE;
	case PM_RESTORE_PREPARE:
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:
		return NOTIFY_DONE;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	case PM_SUSPEND_PREPARE:
		spm_d.u.notify.root_id = hps_get_root_id();
		spin_lock_irqsave(&__spm_lock, flags);
		ret = spm_to_sspm_command(SPM_SUSPEND_PREPARE, &spm_d);
		spin_unlock_irqrestore(&__spm_lock, flags);
		if (ret < 0) {
			pr_info("#@# %s(%d) PM_SUSPEND_PREPARE return %d!!!\n", __func__, __LINE__, ret);
			return NOTIFY_BAD;
		}
		return NOTIFY_DONE;
	case PM_POST_SUSPEND:
		spin_lock_irqsave(&__spm_lock, flags);
		ret = spm_to_sspm_command(SPM_POST_SUSPEND, &spm_d);
		spin_unlock_irqrestore(&__spm_lock, flags);
		if (ret < 0) {
			pr_info("#@# %s(%d) PM_POST_SUSPEND return %d!!!\n", __func__, __LINE__, ret);
			return NOTIFY_BAD;
		}
		return NOTIFY_DONE;
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	}
	return NOTIFY_OK;
}

static struct notifier_block spm_pm_notifier_func = {
	.notifier_call = spm_pm_event,
	.priority = 0,
};
#endif /* CONFIG_PM */
#endif /* CONFIG_FPGA_EARLY_PORTING */

static ssize_t show_debug_log(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *p = buf;

	p += sprintf(p, "for test\n");

	return p - buf;
}

static ssize_t store_debug_log(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t size)
{
	return size;
}

static DEVICE_ATTR(debug_log, 0664, show_debug_log, store_debug_log);	/*664*/

static int spm_probe(struct platform_device *pdev)
{
	int ret;

	ret = device_create_file(&(pdev->dev), &dev_attr_debug_log);

	return 0;
}

static int spm_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id spm_of_ids[] = {
	{.compatible = "mediatek,SLEEP",},
	{}
};

static struct platform_driver spm_dev_drv = {
	.probe = spm_probe,
	.remove = spm_remove,
	.driver = {
		   .name = "spm",
		   .owner = THIS_MODULE,
		   .of_match_table = spm_of_ids,
		   },
};

static struct platform_device *pspmdev;

int __init spm_module_init(void)
{
#if defined(CONFIG_MACH_MT6799)
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	struct spm_data spm_d;
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#endif
	int r = 0;
	int ret = -1;

	spm_register_init();
	if (spm_irq_register() != 0)
		r = -EPERM;
#if defined(CONFIG_PM)
	if (spm_fs_init() != 0)
		r = -EPERM;
#endif
	spm_sodi3_init();
	spm_sodi_init();
	spm_deepidle_init();
#if !defined(CONFIG_MACH_MT6759) && !defined(CONFIG_MACH_MT6758)
#if !defined(CONFIG_FPGA_EARLY_PORTING)
#ifdef CONFIG_MTK_DRAMC
	if (spm_golden_setting_cmp(1) != 0)
		aee_kernel_warning("SPM Warring", "dram golden setting mismach");
#endif /* CONFIG_MTK_DRAMC */
#endif /* CONFIG_FPGA_EARLY_PORTING */
#endif
	ret = platform_driver_register(&spm_dev_drv);
	if (ret) {
		pr_debug("fail to register platform driver\n");
		return ret;
	}

	pspmdev = platform_device_register_simple("spm", -1, NULL, 0);
	if (IS_ERR(pspmdev)) {
		pr_debug("Failed to register platform device.\n");
		return -EINVAL;
	}

	spm_dir = debugfs_create_dir("spm", NULL);
	if (spm_dir == NULL) {
		pr_debug("Failed to create spm dir in debugfs.\n");
		return -EINVAL;
	}

	spm_file = debugfs_create_file("spm_sleep_count", S_IRUGO, spm_dir, NULL, &spm_sleep_count_fops);
	spm_file = debugfs_create_file("spm_last_wakeup_src", S_IRUGO, spm_dir, NULL, &spm_last_wakeup_src_fops);
	spm_resource_req_debugfs_init(spm_dir);
	spm_suspend_debugfs_init(spm_dir);

#if !defined(CONFIG_FPGA_EARLY_PORTING)
#ifdef CONFIG_PM
	ret = register_pm_notifier(&spm_pm_notifier_func);
	if (ret) {
		pr_debug("Failed to register PM notifier.\n");
		return ret;
	}
#endif /* CONFIG_PM */
#endif /* CONFIG_FPGA_EARLY_PORTING */

#if defined(CONFIG_MACH_MT6799)
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	memset(&spm_d, 0, sizeof(struct spm_data));

	ret = spm_to_sspm_command(SPM_EXT_BUCK_STATUS, &spm_d);
	if (ret < 0)
		spm_crit2("ret %d", ret);

	mt_secure_call(MTK_SIP_KERNEL_SPM_ARGS, SPM_ARGS_SPMFW_IDX, ret, 0);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#endif

	spm_vcorefs_init();
	return 0;
}

/**************************************
 * TWAM Control API
 **************************************/
static unsigned int idle_sel;
void spm_twam_set_idle_select(unsigned int sel)
{
	idle_sel = sel & 0x3;
}
EXPORT_SYMBOL(spm_twam_set_idle_select);

static unsigned int window_len;
void spm_twam_set_window_length(unsigned int len)
{
	window_len = len;
}
EXPORT_SYMBOL(spm_twam_set_window_length);

static struct twam_sig mon_type;
void spm_twam_set_mon_type(struct twam_sig *mon)
{
	if (mon) {
		mon_type.sig0 = mon->sig0 & 0x3;
		mon_type.sig1 = mon->sig1 & 0x3;
		mon_type.sig2 = mon->sig2 & 0x3;
		mon_type.sig3 = mon->sig3 & 0x3;
	}
}
EXPORT_SYMBOL(spm_twam_set_mon_type);

void spm_twam_register_handler(twam_handler_t handler)
{
	spm_twam_handler = handler;
}
EXPORT_SYMBOL(spm_twam_register_handler);

void spm_twam_enable_monitor(const struct twam_sig *twamsig, bool speed_mode)
{
	u32 sig0 = 0, sig1 = 0, sig2 = 0, sig3 = 0;
	u32 mon0 = 0, mon1 = 0, mon2 = 0, mon3 = 0;
	unsigned int sel;
	unsigned int length;
	unsigned long flags;

	if (twamsig) {
		sig0 = twamsig->sig0 & 0x1f;
		sig1 = twamsig->sig1 & 0x1f;
		sig2 = twamsig->sig2 & 0x1f;
		sig3 = twamsig->sig3 & 0x1f;
	}

	/* Idle selection */
	sel = idle_sel;
	/* Window length */
	length = window_len;
	/* Monitor type */
	mon0 = mon_type.sig0 & 0x3;
	mon1 = mon_type.sig1 & 0x3;
	mon2 = mon_type.sig2 & 0x3;
	mon3 = mon_type.sig3 & 0x3;

	spin_lock_irqsave(&__spm_lock, flags);
	spm_write(SPM_IRQ_MASK, spm_read(SPM_IRQ_MASK) & ~ISRM_TWAM);
	/* Signal Select */
	spm_write(SPM_TWAM_IDLE_SEL, sel);
	/* Monitor Control */
	spm_write(SPM_TWAM_CON,
		  (sig3 << 27) |
		  (sig2 << 22) |
		  (sig1 << 17) |
		  (sig0 << 12) |
		  (mon3 << 10) |
		  (mon2 << 8) |
		  (mon1 << 6) |
		  (mon0 << 4) | (speed_mode ? REG_SPEED_MODE_EN_LSB : 0) | REG_TWAM_ENABLE_LSB);
	/* Window Length */
	/* 0x13DDF0 for 50ms, 0x65B8 for 1ms, 0x1458 for 200us, 0xA2C for 100us */
	/* in speed mode (26 MHz) */
	spm_write(SPM_TWAM_WINDOW_LEN, length);
	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_debug("enable TWAM for signal %u, %u, %u, %u (%u)\n",
		  sig0, sig1, sig2, sig3, speed_mode);
}
EXPORT_SYMBOL(spm_twam_enable_monitor);

void spm_twam_disable_monitor(void)
{
	unsigned long flags;

	spin_lock_irqsave(&__spm_lock, flags);
	spm_write(SPM_TWAM_CON, spm_read(SPM_TWAM_CON) & ~REG_TWAM_ENABLE_LSB);
	spm_write(SPM_IRQ_MASK, spm_read(SPM_IRQ_MASK) | ISRM_TWAM);
	spm_write(SPM_IRQ_STA, ISRC_TWAM);
	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_debug("disable TWAM\n");
}
EXPORT_SYMBOL(spm_twam_disable_monitor);

/**************************************
 * SPM Golden Seting API(MEMPLL Control, DRAMC)
 **************************************/
#if !defined(CONFIG_MACH_MT6759) && !defined(CONFIG_MACH_MT6758)
#ifdef CONFIG_MTK_DRAMC
struct ddrphy_golden_cfg {
	u32 base;
	u32 offset;
	u32 mask;
	u32 value;
};

#if defined(CONFIG_MACH_MT6775)
static struct ddrphy_golden_cfg ddrphy_setting[] = {
	{DRAMC_AO_CHA, 0x038, 0xc0000027, 0xc0000007},
	{DRAMC_AO_CHB, 0x038, 0xc0000027, 0xc0000007},
	{PHY_AO_CHA, 0x284, 0x001bff00, 0x00000100},
	{PHY_AO_CHB, 0x284, 0x001bff00, 0x00000100},
	{PHY_AO_CHA, 0x28c, 0xffffffff, 0x806003be},
	{PHY_AO_CHB, 0x28c, 0xffffffff, 0x806003be},
	{PHY_AO_CHA, 0x2a8, 0x0c000000, 0x00000000},
	{PHY_AO_CHB, 0x2a8, 0x0c000000, 0x00000000},
	{PHY_AO_CHA, 0xc20, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0xc20, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0x1120, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0x1120, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0x1620, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0x1620, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0x1b20, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0x1b20, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0xca0, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0xca0, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0x11a0, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0x11a0, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0x16a0, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0x16a0, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0x1ba0, 0xfff80000, 0x00200000},
	{PHY_AO_CHB, 0x1ba0, 0xfff80000, 0x00200000},
	{PHY_AO_CHA, 0xd20, 0xfff80000, 0x00000000},
	{PHY_AO_CHB, 0xd20, 0xfff80000, 0x00000000},
	{PHY_AO_CHA, 0x1220, 0xfff80000, 0x00000000},
	{PHY_AO_CHB, 0x1220, 0xfff80000, 0x00000000},
	{PHY_AO_CHA, 0x1720, 0xfff80000, 0x00000000},
	{PHY_AO_CHB, 0x1720, 0xfff80000, 0x00000000},
	{PHY_AO_CHA, 0x1c20, 0xfff80000, 0x00000000},
	{PHY_AO_CHB, 0x1c20, 0xfff80000, 0x00000000},
	{PHY_AO_CHA, 0x298, 0x00770000, 0x00770000},
	{PHY_AO_CHB, 0x298, 0x00770000, 0x00770000},
	{PHY_AO_CHA, 0x084, 0x00100000, 0x00000000},
	{PHY_AO_CHB, 0x084, 0x00100000, 0x00000000},
	{PHY_AO_CHA, 0x104, 0x00100000, 0x00000000},
	{PHY_AO_CHB, 0x104, 0x00100000, 0x00000000},
	{PHY_AO_CHA, 0x184, 0x00100000, 0x00000000},
	{PHY_AO_CHB, 0x184, 0x00100000, 0x00000000},
	{PHY_AO_CHA, 0xc34, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0xc34, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0xcb4, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0xcb4, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0xd34, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0xd34, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x1134, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x1134, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x11b4, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x11b4, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x1234, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x1234, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x1634, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x1634, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x16b4, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x16b4, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x1734, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x1734, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x1b34, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x1b34, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x1bb4, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x1bb4, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0x1c34, 0x00000001, 0x00000001},
	{PHY_AO_CHB, 0x1c34, 0x00000001, 0x00000001},
	{PHY_AO_CHA, 0xc1c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0xc1c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0xc9c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0xc9c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x111c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x111c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x119c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x119c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x161c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x161c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x169c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x169c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x1b1c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x1b1c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x1b9c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x1b9c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0xd1c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0xd1c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x121c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x121c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x171c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x171c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHA, 0x1c1c, 0x000e0000, 0x000e0000},
	{PHY_AO_CHB, 0x1c1c, 0x000e0000, 0x000e0000},
};
#else
static struct ddrphy_golden_cfg ddrphy_setting[] = {
	{DRAMC_AO_CHA, 0x038, 0xc0000027, 0xc0000007},
	{PHY_CHA, 0x284, 0x000bff00, 0x00000100},
	{PHY_CHA, 0x28c, 0xffffffff, 0x83e003be},
	{PHY_CHA, 0x088, 0xffffffff, 0x00000000},
	{PHY_CHA, 0x08c, 0xffffffff, 0x0002e800},
	{PHY_CHA, 0x108, 0xffffffff, 0x00000000},
	{PHY_CHA, 0x10c, 0xffffffff, 0x0002e800},
	{PHY_CHA, 0x188, 0xffffffff, 0x00000800},
	{PHY_CHA, 0x18c, 0xffffffff, 0x000ba000},
	{PHY_CHA, 0x274, 0xffffffff, 0xfffffe7f},
	{PHY_CHA, 0x27c, 0xffffffff, 0xffffffff},
};
#endif

int spm_golden_setting_cmp(bool en)
{
	int i, ddrphy_num, r = 0;

	if (!en)
		return r;

	/*Compare Dramc Goldeing Setting */
	ddrphy_num = ARRAY_SIZE(ddrphy_setting);
	for (i = 0; i < ddrphy_num; i++) {
		u32 value;

		value = lpDram_Register_Read(ddrphy_setting[i].base, ddrphy_setting[i].offset);
		if ((value & ddrphy_setting[i].mask) != ddrphy_setting[i].value) {
			spm_crit2("dramc mismatch addr: 0x%.2x, offset: 0x%.3x, mask: 0x%.8x, val: 0x%x, read: 0x%x\n",
				ddrphy_setting[i].base, ddrphy_setting[i].offset,
				ddrphy_setting[i].mask, ddrphy_setting[i].value, value);
			r = -EPERM;
		}
	}

	return r;

}
#endif /* CONFIG_MTK_DRAMC */
#endif

#if !defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT)
void spm_pmic_vcore_setting(int lp_mode)
{
	static int prev_mode = -1;

	if (lp_mode == prev_mode)
		return;

#if defined(CONFIG_MTK_PMIC_CHIP_MT6335)
	pmic_config_interface_nolock(
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_ADDR,
		lp_mode,
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_SHIFT);

	pmic_config_interface_nolock(
		PMIC_RG_VSRAM_VCORE_HW0_OP_EN_ADDR,
		lp_mode,
		PMIC_RG_VSRAM_VCORE_HW0_OP_EN_MASK,
		PMIC_RG_VSRAM_VCORE_HW0_OP_EN_SHIFT);
#elif defined(CONFIG_MTK_PMIC_CHIP_MT6355)
	pmic_buck_vcore_lp(SRCLKEN0, lp_mode, HW_LP);
	pmic_ldo_vsram_core_lp(SRCLKEN0, lp_mode, HW_LP);
#endif
	prev_mode = lp_mode;
}

void spm_pmic_vcore_setting_of_srclken2(int lp_mode)
{
	static int prev_mode = -1;

	if (lp_mode == prev_mode)
		return;

	pr_debug("set vcore low power setting of srclken2: %d -> %d\n", prev_mode, lp_mode);

#if defined(CONFIG_MTK_PMIC_CHIP_MT6335)
	pmic_config_interface_nolock(PMIC_RG_BUCK_VCORE_HW2_OP_EN_ADDR,
				     lp_mode,
				     PMIC_RG_BUCK_VCORE_HW2_OP_EN_MASK,
				     PMIC_RG_BUCK_VCORE_HW2_OP_EN_SHIFT);
#elif defined(CONFIG_MTK_PMIC_CHIP_MT6355)
	pmic_buck_vcore_lp(SRCLKEN2, lp_mode, HW_LP);
#endif

	prev_mode = lp_mode;
}

void spm_pmic_power_mode(int mode, int force, int lock)
{
	static int prev_mode = -1;

	if (mode < PMIC_PWR_NORMAL || mode >= PMIC_PWR_NUM) {
		pr_debug("wrong spm pmic power mode");
		return;
	}

	if (force == 0 && mode == prev_mode)
		return;

	switch (mode) {
	case PMIC_PWR_NORMAL:
		/* nothing */
		break;
	case PMIC_PWR_DEEPIDLE:
		/* nothing */
		break;
	case PMIC_PWR_SODI3:
#if defined(CONFIG_MTK_PMIC_CHIP_MT6335)
		pmic_ldo_vsram_vcore_lp(SRCLKEN0, 1, HW_LP);
		pmic_ldo_vsram_dvfs1_lp(SRCLKEN0, 1, HW_LP);
		pmic_ldo_va10_lp(SRCLKEN0, 1, HW_LP);
		pmic_ldo_vbif28_lp(SRCLKEN0, 1, HW_LP);
#elif defined(CONFIG_MTK_PMIC_CHIP_MT6355)
		pmic_ldo_vldo28_lp(SRCLKEN0, 0, HW_LP);
		pmic_ldo_vldo28_lp(SW, 1, SW_ON);
		pmic_ldo_vbif28_lp(SRCLKEN0, 1, HW_LP);
#endif
		break;
	case PMIC_PWR_SODI:
		/* nothing */
		break;
	case PMIC_PWR_SUSPEND:
#if defined(CONFIG_MTK_PMIC_CHIP_MT6335)
		pmic_ldo_vsram_vcore_lp(SRCLKEN0, 1, HW_LP);
		pmic_ldo_vsram_dvfs1_lp(SRCLKEN0, 0, HW_LP);
		pmic_ldo_vsram_dvfs1_lp(SPM, 1, SPM_OFF);
		pmic_ldo_va10_lp(SRCLKEN0, 1, HW_OFF);
		pmic_ldo_vbif28_lp(SRCLKEN0, 1, HW_OFF);
#elif defined(CONFIG_MTK_PMIC_CHIP_MT6355)
		pmic_ldo_vldo28_lp(SRCLKEN0, 1, HW_LP);
		pmic_ldo_vbif28_lp(SRCLKEN0, 1, HW_OFF);
#endif
		break;
	default:
		pr_debug("spm pmic power mode (%d) is not configured\n", mode);
	}

	prev_mode = mode;
}
#endif /* !defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) */

void *mt_spm_base_get(void)
{
	return spm_base;
}
EXPORT_SYMBOL(mt_spm_base_get);

void mt_spm_for_gps_only(int enable)
{
	spm_for_gps_flag = !!enable;
	/* pr_debug("#@# %s(%d) spm_for_gps_flag %d\n", __func__, __LINE__, spm_for_gps_flag); */
}
EXPORT_SYMBOL(mt_spm_for_gps_only);

void mt_spm_dcs_s1_setting(int enable, int flags)
{
	flags &= 0xf;

	mt_secure_call(MTK_SIP_KERNEL_SPM_DCS_S1, enable, flags, 0);
}
EXPORT_SYMBOL(mt_spm_dcs_s1_setting);

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT

#define SPM_D_LEN		(8) /* # of cmd + arg0 + arg1 + ... */
#define SPM_VCOREFS_D_LEN	(4) /* # of cmd + arg0 + arg1 + ... */

#include <sspm_ipi.h>

int spm_to_sspm_command_async(u32 cmd, struct spm_data *spm_d)
{
	unsigned int ret = 0;

	switch (cmd) {
	case SPM_DPIDLE_ENTER:
	case SPM_DPIDLE_LEAVE:
	case SPM_ENTER_SODI:
	case SPM_LEAVE_SODI:
	case SPM_ENTER_SODI3:
	case SPM_LEAVE_SODI3:
		spm_d->cmd = cmd;
		ret = sspm_ipi_send_async(IPI_ID_SPM_SUSPEND, IPI_OPT_DEFAUT, spm_d, SPM_D_LEN);
		if (ret != 0)
			pr_info("#@# %s(%d) sspm_ipi_send_async(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		break;
	default:
		pr_info("#@# %s(%d) cmd(%d) wrong!!!\n", __func__, __LINE__, cmd);
		break;
	}

	return ret;
}

int spm_to_sspm_command_async_wait(u32 cmd)
{
	unsigned int ret = 0;

	int ack_data;

	switch (cmd) {
	case SPM_DPIDLE_ENTER:
	case SPM_DPIDLE_LEAVE:
	case SPM_ENTER_SODI:
	case SPM_LEAVE_SODI:
	case SPM_ENTER_SODI3:
	case SPM_LEAVE_SODI3:
		ret = sspm_ipi_send_async_wait(IPI_ID_SPM_SUSPEND, IPI_OPT_DEFAUT, &ack_data);

		if (ret != 0) {
			pr_info("#@# %s(%d) sspm_ipi_send_async_wait(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_info("#@# %s(%d) cmd(%d) return %d\n", __func__, __LINE__, cmd, ret);
		}
		break;
	default:
		pr_info("#@# %s(%d) cmd(%d) wrong!!!\n", __func__, __LINE__, cmd);
		break;
	}

	return ret;
}

int spm_to_sspm_command(u32 cmd, struct spm_data *spm_d)
{
	unsigned int ret = 0;
	/* struct spm_data _spm_d; */

	int ack_data;

	switch (cmd) {
	case SPM_SUSPEND:
	case SPM_RESUME:
	case SPM_DPIDLE_ENTER:
	case SPM_DPIDLE_LEAVE:
	case SPM_ENTER_SODI:
	case SPM_ENTER_SODI3:
	case SPM_LEAVE_SODI:
	case SPM_LEAVE_SODI3:
		spm_d->cmd = cmd;
		ret = sspm_ipi_send_sync(IPI_ID_SPM_SUSPEND, IPI_OPT_POLLING, spm_d, SPM_D_LEN, &ack_data, 1);
		if (ret != 0) {
			pr_info("#@# %s(%d) sspm_ipi_send_sync(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_info("#@# %s(%d) cmd(%d) return %d\n", __func__, __LINE__, cmd, ret);
		}
		break;
	case SPM_VCORE_PWARP_CMD:
#if defined(CONFIG_MACH_MT6758) || defined(CONFIG_MACH_MT6775)
		spm_d->cmd = cmd;
		ret = sspm_ipi_send_sync(IPI_ID_SPM_SUSPEND, IPI_OPT_POLLING, spm_d, SPM_D_LEN, &ack_data, 1);
		if (ret != 0) {
			pr_info("#@# %s(%d) sspm_ipi_send_sync(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_info("#@# %s(%d) cmd(%d) return %d\n", __func__, __LINE__, cmd, ret);
		}
#endif
		break;
	case SPM_SUSPEND_PREPARE:
	case SPM_POST_SUSPEND:
		spm_d->cmd = cmd;
		ret = sspm_ipi_send_sync(IPI_ID_SPM_SUSPEND, IPI_OPT_POLLING, spm_d, SPM_D_LEN, &ack_data, 1);
		if (ret != 0) {
			pr_info("#@# %s(%d) sspm_ipi_send_sync(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_info("#@# %s(%d) cmd(%d) return %d\n", __func__, __LINE__, cmd, ret);
		}
		break;
	case SPM_DPIDLE_PREPARE:
	case SPM_POST_DPIDLE:
		spm_d->cmd = cmd;
		ret = sspm_ipi_send_sync(IPI_ID_SPM_SUSPEND, IPI_OPT_POLLING, spm_d, SPM_D_LEN, &ack_data, 1);
		if (ret != 0) {
			pr_info("#@# %s(%d) sspm_ipi_send_sync(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_info("#@# %s(%d) cmd(%d) return %d\n", __func__, __LINE__, cmd, ret);
		}
		break;
	case SPM_SODI_PREPARE:
	case SPM_POST_SODI:
		spm_d->cmd = cmd;
		ret = sspm_ipi_send_sync(IPI_ID_SPM_SUSPEND, IPI_OPT_POLLING, spm_d, SPM_D_LEN, &ack_data, 1);
		if (ret != 0) {
			pr_info("#@# %s(%d) sspm_ipi_send_sync(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_info("#@# %s(%d) cmd(%d) return %d\n", __func__, __LINE__, cmd, ret);
		}
		break;
	case SPM_EXT_BUCK_STATUS:
		spm_d->cmd = cmd;
		ret = sspm_ipi_send_sync(IPI_ID_SPM_SUSPEND, IPI_OPT_POLLING, spm_d, SPM_D_LEN, &ack_data, 1);
		if (ret != 0) {
			pr_info("#@# %s(%d) sspm_ipi_send_sync(cmd:0x%x) ret %d\n", __func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_info("#@# %s(%d) cmd(%d) return %d\n", __func__, __LINE__, cmd, ret);
		} else
			ret = ack_data;
		break;
	default:
		pr_info("#@# %s(%d) cmd(%d) wrong!!!\n", __func__, __LINE__, cmd);
		break;
	}
	return ret;
}
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

void unmask_edge_trig_irqs_for_cirq(void)
{
	int i;

	for (i = 0; i < NF_EDGE_TRIG_IRQS; i++) {
		if (edge_trig_irqs[i]) {
			/* unmask edge trigger irqs */
			mt_irq_unmask_for_sleep_ex(edge_trig_irqs[i]);
		}
	}
}

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
static atomic_t ipi_lock_cnt;

bool is_sspm_ipi_lock_spm(void)
{

	int lock_cnt = -1;
	bool ret = false;

	lock_cnt = atomic_read(&ipi_lock_cnt);

	ret = (lock_cnt == 0) ? false : true;

	return ret;
}

void sspm_ipi_lock_spm_scenario(int start, int id, int opt, const char *name)
{

	if (id == IPI_ID_SPM_SUSPEND)
		return;

	if (id < 0 || id >= IPI_ID_TOTAL)
		return;

	if (start)
		atomic_inc(&ipi_lock_cnt);
	else
		atomic_dec(&ipi_lock_cnt);

	/* FTRACE tag */
	trace_sspm_ipi(start, id, opt);

}
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

#if defined(CONFIG_MACH_MT6775)
#include <mtk_mcdi_governor.h>
#include <mtk_hps_internal.h>
bool is_big_buck_pdn_by_spm(void)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	/* If big buck off by mcdi or cpu hotplug
	 *  then return true
	 */
	#define BIG_BUCK_CLUSTER_ID     1
	return !(mcdi_is_buck_off(BIG_BUCK_CLUSTER_ID)
			|| cpuhp_is_buck_off(BIG_BUCK_CLUSTER_ID));
#else
	return false;
#endif
}
#endif

MODULE_DESCRIPTION("SPM Driver v0.1");
