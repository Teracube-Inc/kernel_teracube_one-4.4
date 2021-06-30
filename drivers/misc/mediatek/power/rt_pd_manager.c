/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * drivers/misc/mediatek/power/mt6757/rt_pd_manager.c
 *
 * Author: Sakya <jeff_chang@richtek.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include "tcpm.h"

#include <mt-plat/upmu_common.h>
#if CONFIG_MTK_GAUGE_VERSION == 30
#include <mt-plat/charger_class.h>
#include <mt-plat/mtk_battery.h>
#include <mt-plat/mtk_charger.h>
#include <charger/mtk_charger_intf.h>
#else
#include <mt-plat/battery_meter.h>
#include <mt-plat/charging.h>
#endif /* CONFIG_MTK_GAUGE_VERSION */

#include <mt-plat/mtk_boot.h>
#include "musb_core.h"
#define RT_PD_MANAGER_VERSION	"1.0.5_MTK"

#ifdef CONFIG_USB_C_SWITCH_U3_MUX
#include "usb_switch.h"
#include "typec.h"
#endif

static DEFINE_MUTEX(param_lock);

static struct tcpc_device *tcpc_dev;
static struct notifier_block pd_nb;
static int pd_sink_voltage_new;
static int pd_sink_voltage_old;
static int pd_sink_current_new;
static int pd_sink_current_old;
static unsigned char pd_sink_type;
static bool tcpc_kpoc;
static unsigned char bc12_chr_type;
#if 0 /* vconn is from vsys on mt6763 */
/* vconn boost gpio pin */
static int vconn_gpio;
static unsigned char vconn_on;
#endif

#if CONFIG_MTK_GAUGE_VERSION == 30
static struct charger_device *primary_charger;
static struct charger_consumer *chg_consumer;
#endif

static void tcpc_mt_power_off(void)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	kernel_power_off();
#endif /* CONFIG_MTK_KERNEL_POWER_OFF_CHARGING */
}

#if CONFIG_MTK_GAUGE_VERSION == 20
#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT

static DEFINE_MUTEX(pd_chr_mutex);
struct task_struct *pd_thread_handle;

static bool isCableIn;
static bool updatechrdet;

#if !defined CONFIG_HAS_WAKELOCKS
struct wakeup_source chrdet_Thread_lock;
#else
struct wake_lock chrdet_Thread_lock;
#endif

void pd_wake_lock(void)
{
#if !defined CONFIG_HAS_WAKELOCKS
	__pm_stay_awake(&chrdet_Thread_lock);
#else
	wake_lock(&chrdet_Thread_lock);
#endif
}

void pd_wake_unlock(void)
{
#if !defined CONFIG_HAS_WAKELOCKS
	__pm_relax(&chrdet_Thread_lock);
#else
	wake_unlock(&chrdet_Thread_lock);
#endif
}

void pd_chrdet_int_handler(void)
{
	pr_notice("[pd_chrdet_int_handler]CHRDET status = %d....\n",
		pmic_get_register_value(PMIC_RGS_CHRDET));

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (!upmu_get_rgs_chrdet()) {
		int boot_mode = 0;

		boot_mode = get_boot_mode();

		if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			|| boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			pr_notice("[pd_chrdet_int_handler] Unplug Charger/USB\n");
			kernel_power_off();
		}
	}
#endif

	pmic_set_register_value(PMIC_RG_USBDL_RST, 1);
	do_chrdet_int_task();
}

int chrdet_thread_kthread(void *x)
{
	struct sched_param param = {.sched_priority = 98 };

	sched_setscheduler(current, SCHED_FIFO, &param);
	set_current_state(TASK_INTERRUPTIBLE);

	pr_notice("[chrdet_thread_kthread] enter\n");
	pmic_enable_interrupt(CHRDET_INT_NO, 0, "pd_manager");

	/* Run on a process content */
	while (1) {
		mutex_lock(&pd_chr_mutex);
		if (updatechrdet == true) {
			pr_notice("chrdet_work_handler\n");
			pd_chrdet_int_handler();
		} else
			pr_err("chrdet_work_handler no update\n");
		mutex_unlock(&pd_chr_mutex);
		set_current_state(TASK_INTERRUPTIBLE);
		pd_wake_unlock();
		schedule();
	}

	return 0;
}

void wake_up_pd_chrdet(void)
{
	pr_notice("[wake_up_pd_chrdet]\r\n");
	pd_wake_lock();
	if (pd_thread_handle != NULL)
		wake_up_process(pd_thread_handle);
}
#endif /* CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT */
#endif /* This part is for GM20 */

enum {
	SINK_TYPE_REMOVE,
	SINK_TYPE_TYPEC,
	SINK_TYPE_PD_TRY,
	SINK_TYPE_PD_CONNECTED,
	SINK_TYPE_REQUEST,
};

bool mtk_is_pep30_en_unlock(void)
{
	return false;
}

static int pd_tcp_notifier_call(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	u32 vbus = 0;
	int ret = 0;
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	int boot_mode = 0;
#endif /* CONFIG_MTK_KERNEL_POWER_OFF_CHARGING */

	switch (event) {
	case TCP_NOTIFY_SOURCE_VCONN:
#if 0 /* vconn is from vsys on mt6763 */
		if (noti->swap_state.new_role) {
			if (!vconn_on) {
				pr_info("%s set vconn enable\n", __func__);
				gpio_set_value(vconn_gpio, 1);
				vconn_on = 1;
			}
		} else {
			if (vconn_on) {
				pr_info("%s set vconn disable\n", __func__);
				gpio_set_value(vconn_gpio, 0);
				vconn_on = 0;
			}
		}
#endif
		break;
	case TCP_NOTIFY_SINK_VBUS:
		mutex_lock(&param_lock);
		pd_sink_voltage_new = noti->vbus_state.mv;
		pd_sink_current_new = noti->vbus_state.ma;

		if (noti->vbus_state.type & TCP_VBUS_CTRL_PD_DETECT)
			pd_sink_type = SINK_TYPE_PD_CONNECTED;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_REMOVE)
			pd_sink_type = SINK_TYPE_REMOVE;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_TYPEC)
			pd_sink_type = SINK_TYPE_TYPEC;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_PD)
			pd_sink_type = SINK_TYPE_PD_TRY;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_REQUEST)
			pd_sink_type = SINK_TYPE_REQUEST;
		pr_info("%s sink vbus %dmv %dma type(%d)\n", __func__,
			pd_sink_voltage_new, pd_sink_current_new, pd_sink_type);
		mutex_unlock(&param_lock);

		if ((pd_sink_voltage_new != pd_sink_voltage_old) ||
		    (pd_sink_current_new != pd_sink_current_old)) {
			if (pd_sink_voltage_new) {
				/* enable charger */
#if CONFIG_MTK_GAUGE_VERSION == 30
				charger_manager_enable_power_path(chg_consumer, MAIN_CHARGER, true);
#else
				mtk_chr_pd_enable_power_path(1);
#endif
				pd_sink_voltage_old = pd_sink_voltage_new;
				pd_sink_current_old = pd_sink_current_new;
			} else if (pd_sink_type == SINK_TYPE_REMOVE) {
				if (tcpc_kpoc)
					break;
#if CONFIG_MTK_GAUGE_VERSION == 30
				charger_manager_enable_power_path(chg_consumer, MAIN_CHARGER, false);
#else
				mtk_chr_pd_enable_power_path(0);
#endif
				pd_sink_voltage_old = pd_sink_voltage_new;
				pd_sink_current_old = pd_sink_current_new;
			} else {
				bc12_chr_type = mt_get_charger_type();
				if (bc12_chr_type >= STANDARD_HOST &&
				    bc12_chr_type <= STANDARD_CHARGER)
					break;
				if (tcpc_kpoc)
					break;
				/* disable charge */
#if CONFIG_MTK_GAUGE_VERSION == 30
				charger_manager_enable_power_path(chg_consumer, MAIN_CHARGER, false);
#else
				mtk_chr_pd_enable_power_path(0);
#endif
				pd_sink_voltage_old = pd_sink_voltage_new;
				pd_sink_current_old = pd_sink_current_new;
			}
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			(noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
			noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
			noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC)) {
			charger_ignore_usb(false);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
#if CONFIG_MTK_GAUGE_VERSION == 30
			charger_dev_enable_chg_type_det(primary_charger, true);
#else
			mtk_chr_enable_chr_type_det(true);
#endif
#else
			mtk_pmic_enable_chr_type_det(true);
#endif
			pr_info("%s USB Plug in, pol = %d\n", __func__,
					noti->typec_state.polarity);
#ifdef CONFIG_USB_C_SWITCH_U3_MUX
			usb3_switch_dps_en(false);
			if (noti->typec_state.polarity == 0)
				usb3_switch_ctrl_sel(CC1_SIDE);
			else
				usb3_switch_ctrl_sel(CC2_SIDE);
#endif
#if CONFIG_MTK_GAUGE_VERSION == 20
#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
			mutex_lock(&pd_chr_mutex);
			isCableIn = true;
			updatechrdet = true;
			wake_up_pd_chrdet();
			mutex_unlock(&pd_chr_mutex);
			pr_notice("TCP_NOTIFY_SINK_VBUS=> plug in");
#endif
#endif
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
			noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
			noti->typec_state.old_state == TYPEC_ATTACHED_NORP_SRC)
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			if (tcpc_kpoc) {
				vbus = battery_meter_get_charger_voltage();
				pr_info("%s KPOC Plug out, vbus = %d\n",
					__func__, vbus);
				tcpc_mt_power_off();
				break;
			}
			pr_info("%s USB Plug out\n", __func__);
			charger_ignore_usb(false);
#if CONFIG_MTK_GAUGE_VERSION == 20
#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
			mutex_lock(&pd_chr_mutex);
			isCableIn = false;
			updatechrdet = true;
			wake_up_pd_chrdet();
			mutex_unlock(&pd_chr_mutex);
			pr_notice("TCP_NOTIFY_SINK_VBUS=> plug out");
#endif
#endif
#ifdef CONFIG_USB_C_SWITCH_U3_MUX
			usb3_switch_dps_en(true);
#endif
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
#if CONFIG_MTK_GAUGE_VERSION == 30
			ret = charger_dev_enable_chg_type_det(primary_charger, false);
#else
			ret = mtk_chr_enable_chr_type_det(false);
#endif
#else
			mtk_pmic_enable_chr_type_det(false);
#endif

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
			boot_mode = get_boot_mode();
			if (ret < 0) {
				if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
					|| boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
					pr_err("%s: notify chg detach fail, power off\n", __func__);
					kernel_power_off();
				}
			}
#endif /* CONFIG_MTK_KERNEL_POWER_OFF_CHARGING */
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SRC &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			/* source to sink */
			pr_info("%s: Source_to_Sink\n", __func__);
			charger_ignore_usb(true);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
			charger_dev_enable_chg_type_det(primary_charger, true);
#else
			mtk_pmic_enable_chr_type_det(true);
#endif
		}  else if (noti->typec_state.old_state == TYPEC_ATTACHED_SNK &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			/* sink to source */
			pr_info("%s: Sink_to_Source\n", __func__);
			charger_ignore_usb(true);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
			charger_dev_enable_chg_type_det(primary_charger, false);
#else
			mtk_pmic_enable_chr_type_det(false);
#endif
		}
		break;
	case TCP_NOTIFY_PD_STATE:
		pr_info("%s pd state = %d\n",
			__func__, noti->pd_state.connected);
		break;
#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK
	case TCP_NOTIFY_ATTACHWAIT_SNK:
		pr_info("%s attach wait sink\n", __func__);
		break;
#endif /* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK */
	case TCP_NOTIFY_EXT_DISCHARGE:
		pr_info("%s ext discharge = %d\n", __func__, noti->en_state.en);
#if CONFIG_MTK_GAUGE_VERSION == 30
		charger_dev_enable_discharge(primary_charger,
			noti->en_state.en);
#endif
		break;

	case TCP_NOTIFY_HARD_RESET_STATE:
		if (noti->hreset_state.state == TCP_HRESET_RESULT_DONE ||
			noti->hreset_state.state == TCP_HRESET_RESULT_FAIL)
			charger_manager_enable_kpoc_shutdown(chg_consumer, true);
		else if (noti->hreset_state.state == TCP_HRESET_SIGNAL_SEND ||
			noti->hreset_state.state == TCP_HRESET_SIGNAL_RECV)
			charger_manager_enable_kpoc_shutdown(chg_consumer, false);
		break;
	default:
		break;
	};
	return NOTIFY_OK;
}

static int rt_pd_manager_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *node = pdev->dev.of_node;

	pr_info("%s (%s)\n", __func__, RT_PD_MANAGER_VERSION);
	if (node == NULL) {
		pr_err("%s devicd of node not exist\n", __func__);
		return -ENODEV;
	}

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	ret = get_boot_mode();
	if (ret == KERNEL_POWER_OFF_CHARGING_BOOT ||
	    ret == LOW_POWER_OFF_CHARGING_BOOT)
		tcpc_kpoc = true;
#endif /* CONFIG_MTK_KERNEL_POWER_OFF_CHARGING */
	pr_info("%s KPOC(%d)\n", __func__, tcpc_kpoc);


	/* Get charger device */
#if CONFIG_MTK_GAUGE_VERSION == 30
	primary_charger = get_charger_by_name("primary_chg");
	if (!primary_charger) {
		pr_err("%s: get primary charger device failed\n", __func__);
		return -ENODEV;
	}
	chg_consumer = charger_manager_get_by_name(&pdev->dev, "charger_port1");
	if (!chg_consumer) {
		pr_err("%s: get charger consumer device failed\n", __func__);
		return -ENODEV;
	}
#endif /* CONFIG_MTK_GAUGE_VERSION == 30 */

#if 0 /* vconn is from vsys on mt6759 */
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(node, "rt,pd_vconn_gpio", 0);
	if (ret < 0) {
		pr_err("%s no pd_vconn_gpio info\n", __func__);
		return -EINVAL;
	}
	vconn_gpio = ret;
#else
	ret =  of_property_read_u32(node, "rt,pd_vconn_gpio_x", &vconn_gpio);
	if (ret < 0) {
		pr_err("%s no pd_vconn_gpio_x info\n", __func__);
		return -EINVAL;
	}
#endif
	pr_info("%s Vconn gpio = %d\n", __func__, vconn_gpio);
	ret = gpio_request_one(vconn_gpio, GPIOF_OUT_INIT_LOW, "pd,vconn_source");
	if (ret < 0) {
		pr_err("%s gpio request fail\n", __func__);
		return ret;
	}
#endif

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(tcpc_dev, &pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		return -EINVAL;
	}


#if CONFIG_MTK_GAUGE_VERSION == 20
#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
#if !defined CONFIG_HAS_WAKELOCKS
	wakeup_source_init(&chrdet_Thread_lock, "pd chrdet wakelock");
#else
	wake_lock_init(&chrdet_Thread_lock, WAKE_LOCK_SUSPEND, "pd chrdet wakelock");
#endif
	pd_thread_handle = kthread_create(chrdet_thread_kthread, (void *)NULL,
		"pd_chrdet_thread");
	if (IS_ERR(pd_thread_handle)) {
		pd_thread_handle = NULL;
		pr_err("[pd_thread_handle] creation fails\n");
	} else {
		pr_notice("[pd_thread_handle] kthread_create Done\n");
	}
#endif /* CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT */
#endif /* This part is for GM20 */

	pr_info("%s OK!!\n", __func__);
	return ret;
}

static int rt_pd_manager_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id rt_pd_manager_of_match[] = {
	{ .compatible = "mediatek,rt-pd-manager" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt_pd_manager_of_match);

static struct platform_driver rt_pd_manager_driver = {
	.driver = {
		.name = "rt-pd-manager",
		.of_match_table = of_match_ptr(rt_pd_manager_of_match),
	},
	.probe = rt_pd_manager_probe,
	.remove = rt_pd_manager_remove,
};

static int __init rt_pd_manager_init(void)
{
	return platform_driver_register(&rt_pd_manager_driver);
}

static void __exit rt_pd_manager_exit(void)
{
	platform_driver_unregister(&rt_pd_manager_driver);
}

late_initcall_sync(rt_pd_manager_init);
module_exit(rt_pd_manager_exit);

MODULE_AUTHOR("Jeff Chang");
MODULE_DESCRIPTION("Richtek pd manager driver");
MODULE_LICENSE("GPL");
