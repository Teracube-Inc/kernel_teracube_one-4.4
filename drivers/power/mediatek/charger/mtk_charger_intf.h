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
#include <mt-plat/charger_type.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/mtk_charger.h>

#ifndef __MTK_CHARGER_INTF_H__
#define __MTK_CHARGER_INTF_H__

#include <linux/ktime.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <mt-plat/mtk_battery.h>
#include <mtk_gauge_time_service.h>

/* PD */
#include <tcpm.h>


struct charger_manager;
#include "mtk_pe_intf.h"
#include "mtk_pe20_intf.h"
#include "mtk_pe30_intf.h"
#include "mtk_pe40_intf.h"
#include "mtk_pdc_intf.h"

#define CHARGING_INTERVAL 10
#define CHARGING_FULL_INTERVAL 20

#define MAX_CHARGING_TIME (12 * 60 * 60) /* 12 hours */

#define CHRLOG_ERROR_LEVEL   1
#define CHRLOG_DEBUG_LEVEL   2

extern int chr_get_debug_level(void);

#define chr_err(fmt, args...)   \
do {									\
	if (chr_get_debug_level() >= CHRLOG_ERROR_LEVEL) {			\
		pr_notice(fmt, ##args); \
	}								   \
} while (0)

#define chr_info(fmt, args...)   \
do {									\
	if (chr_get_debug_level() >= CHRLOG_ERROR_LEVEL) {		\
		pr_notice_ratelimited(fmt, ##args); \
	}								   \
} while (0)

#define chr_debug(fmt, args...)   \
do {									\
	if (chr_get_debug_level() >= CHRLOG_DEBUG_LEVEL) {		\
		pr_notice(fmt, ##args); \
	}								   \
} while (0)

#ifdef MTK_CHARGER_EXP
extern int charger_get_debug_level(void);
extern void charger_log(const char *fmt, ...);
extern void charger_log_flash(const char *fmt, ...);

#define chr_err(fmt, args...)   \
		do {									\
			if (charger_get_debug_level() >= CHRLOG_ERROR_LEVEL) {			\
				charger_log_flash(fmt, ##args); \
			}								   \
		} while (0)

#define chr_debug(fmt, args...)   \
		do {									\
			if (charger_get_debug_level() >= CHRLOG_DEBUG_LEVEL) {		\
				charger_log_flash(fmt, ##args); \
			}								   \
		} while (0)

#define chr_err_batch(fmt, args...)   \
		do {									\
			if (charger_get_debug_level() >= CHRLOG_ERROR_LEVEL) {			\
				charger_log(fmt, ##args); \
			}								   \
		} while (0)

#define chr_debug_batch(fmt, args...)   \
		do {									\
			if (charger_get_debug_level() >= CHRLOG_DEBUG_LEVEL) {		\
				charger_log(fmt, ##args); \
			}								   \
		} while (0)
#endif


#define CHR_CC		(0x0001)
#define CHR_TOPOFF  (0x0002)
#define CHR_TUNING	(0x0003)
#define CHR_POSTCC	(0x0004)
#define CHR_BATFULL	(0x0005)
#define CHR_ERROR	(0x0006)
#define	CHR_PE40_INIT	(0x0007)
#define	CHR_PE40_CC	(0x0008)
#define	CHR_PE40_TUNING (0x0009)
#define	CHR_PE40_POSTCC (0x000A)
#define CHR_PE30	(0x000B)


/* charger_algorithm notify charger_dev */
enum {
	EVENT_EOC,
	EVENT_RECHARGE,
};


/* charger_dev notify charger_manager */
enum {
	CHARGER_DEV_NOTIFY_VBUS_OVP,
	CHARGER_DEV_NOTIFY_BAT_OVP,
	CHARGER_DEV_NOTIFY_EOC,
	CHARGER_DEV_NOTIFY_RECHG,
	CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT,
};


/*
*Software Jeita
*T0:-10
*T1:0
*T2:10
*T3:45
*T4:50
*/
enum sw_jeita_state_enum {
	TEMP_BELOW_T0 = 0,
	TEMP_T0_TO_T1,
	TEMP_T1_TO_T2,
	TEMP_T2_TO_T3,
	TEMP_T3_TO_T4,
	TEMP_ABOVE_T4
};

struct sw_jeita_data {
	int sm;
	int pre_sm;
	int cv;
	bool charging;
	bool error_recovery_flag;
};

/* battery thermal protection */
enum bat_temp_state_enum {
	BAT_TEMP_LOW = 0,
	BAT_TEMP_NORMAL,
	BAT_TEMP_HIGH
};

struct battery_thermal_protection_data {
	int sm;
	bool enable_min_charge_temperature;
	int min_charge_temperature;
	int min_charge_temperature_plus_x_degree;
	int max_charge_temperature;
	int max_charge_temperature_minus_x_degree;
};

struct charger_custom_data {
	int battery_cv;	/* uv */
	int max_charger_voltage;
	int max_charger_voltage_setting;
	int min_charger_voltage;

	int usb_charger_current_suspend;
	int usb_charger_current_unconfigured;
	int usb_charger_current_configured;
	int usb_charger_current;
	int ac_charger_current;
	int ac_charger_input_current;
	int non_std_ac_charger_current;
	int charging_host_charger_current;
	int apple_1_0a_charger_current;
	int apple_2_1a_charger_current;
	int ta_ac_charger_current;
	int pd_charger_current;

	/* sw jeita */
	int jeita_temp_above_t4_cv_voltage;
	int jeita_temp_t3_to_t4_cv_voltage;
	int jeita_temp_t2_to_t3_cv_voltage;
	int jeita_temp_t1_to_t2_cv_voltage;
	int jeita_temp_t0_to_t1_cv_voltage;
	int jeita_temp_below_t0_cv_voltage;
	int temp_t4_threshold;
	int temp_t4_thres_minus_x_degree;
	int temp_t3_threshold;
	int temp_t3_thres_minus_x_degree;
	int temp_t2_threshold;
	int temp_t2_thres_plus_x_degree;
	int temp_t1_threshold;
	int temp_t1_thres_plus_x_degree;
	int temp_t0_threshold;
	int temp_t0_thres_plus_x_degree;
	int temp_neg_10_threshold;

	/* battery temperature protection */
	int mtk_temperature_recharge_support;
	int max_charge_temperature;
	int max_charge_temperature_minus_x_degree;
	int min_charge_temperature;
	int min_charge_temperature_plus_x_degree;

	/* pe */
	int pe_ichg_level_threshold;	/* ma */
	int ta_ac_12v_input_current;
	int ta_ac_9v_input_current;
	int ta_ac_7v_input_current;
	bool ta_12v_support;
	bool ta_9v_support;

	/* pe2.0 */
	int pe20_ichg_level_threshold;	/* ma */
	int ta_start_battery_soc;
	int ta_stop_battery_soc;

	/* pe4.0 */
	int pe40_single_charger_input_current;	/* ma */
	int pe40_single_charger_current;
	int pe40_dual_charger_input_current;
	int pe40_dual_charger_chg1_current;
	int pe40_dual_charger_chg2_current;
	int pe40_stop_battery_soc;

	/* pe3.0 */
	int cv_limit;	/* vbus upper bound (mv)*/
	int bat_upper_bound;	/* battery upper bound (mv)*/
	int bat_lower_bound;	/* battery low bound (mv)*/

	int cc_ss_init;	/*init charging current (ma)*/
	int cc_init;	/*max charging current (ma)*/
	int cc_init_bad_cable1;	/*charging current(ma) for bad cable1*/
	int cc_init_bad_cable2;	/*charging current(ma) for bad cable2*/
	int cc_init_r;	/*good cable max impedance*/
	int cc_init_bad_cable1_r;	/*bad cable1 max impedance*/
	int cc_init_bad_cable2_r;	/*bad cable2 max impedance*/

	int cc_normal;	/*normal charging ibus oc threshold (ua)*/
	int cc_max;	/*pe3.0 ibus oc threshold (ua)*/
	int cc_end;	/*pe3.0 min charging current (ma)*/
	int cc_step;	/*cc state charging current step (ma)*/
	int cc_ss_step;	/*soft start state charging current step (ma)*/
	int cc_ss_step2;	/*soft start state charging current step (ma),when battery voltage > cc_ss_step2*/
	int cc_ss_step3;	/*soft start state charging current step (ma),when battery voltage > cc_ss_step3*/
	int cv_ss_step2;	/*battery voltage threshold for cc_ss_step2*/
	int cv_ss_step3;	/*battery voltage threshold for cc_ss_step3*/
	int cc_ss_blanking;	/*polling duraction for init/soft start state(ms)*/
	int cc_blanking;	/*polling duraction for cc state(ms)*/
	int charger_temp_max;	/*max charger ic temperature*/
	int ta_temp_max;	/*max adapter temperature*/
	int vbus_ov_gap;
	int fod_current;	/*fod current threshold*/
	int r_vbat_min;	/*min r_vbat*/
	int r_sw_min;	/*min r_sw*/
	int pe30_max_charging_time;	/*pe3.0 max chargint time (sec)*/
	int battery_temp_min;
	int battery_temp_max;

	/* dual charger */
#ifdef CONFIG_MTK_DUAL_CHARGER_SUPPORT
	u32 chg1_ta_ac_charger_current;
	u32 chg2_ta_ac_charger_current;
#endif

	/* cable measurement impedance */
	int cable_imp_threshold;
	int vbat_cable_imp_threshold;

	/* bif */
	int bif_threshold1;	/* uv */
	int bif_threshold2;	/* uv */
	int bif_cv_under_threshold2;	/* uv */

	/* power path */
	bool power_path_support;

	int max_charging_time; /* second */
};

struct charger_data {
	int force_charging_current;
	int thermal_input_current_limit;
	int thermal_charging_current_limit;
	int input_current_limit;
	int charging_current_limit;
	int disable_charging_count;
	int input_current_limit_by_aicl;
	int junction_temp_min;
	int junction_temp_max;
};

struct charger_manager {
	bool init_done;
	const char *algorithm_name;
	struct platform_device *pdev;
	void	*algorithm_data;
	int usb_state;
	bool usb_unlimited;
	bool disable_charger;

	struct charger_device *chg1_dev;
	struct notifier_block chg1_nb;
	struct charger_data chg1_data;
	struct charger_consumer *chg1_consumer;

	struct charger_device *chg2_dev;
	struct notifier_block chg2_nb;
	struct charger_data chg2_data;

	enum charger_type chr_type;
	bool can_charging;

	int (*do_algorithm)(struct charger_manager *);
	int (*plug_in)(struct charger_manager *);
	int (*plug_out)(struct charger_manager *);
	int (*do_charging)(struct charger_manager *, bool en);
	int (*do_event)(struct notifier_block *nb, unsigned long event, void *v);
	int (*change_current_setting)(struct charger_manager *);

	/*notify charger user*/
	struct srcu_notifier_head evt_nh;
	/*receive from battery*/
	struct notifier_block psy_nb;

	/* common info */
	int battery_temperature;

	/* sw jeita */
	bool enable_sw_jeita;
	struct sw_jeita_data sw_jeita;

	/* dynamic_cv */
	bool enable_dynamic_cv;

	bool cmd_discharging;
	bool safety_timeout;
	bool vbusov_stat;

	/* battery warning */
	unsigned int notify_code;
	unsigned int notify_test_mode;

	/* battery thermal protection */
	struct battery_thermal_protection_data thermal;

	/* dtsi custom data */
	struct charger_custom_data data;

	bool enable_sw_safety_timer;

	/* High voltage charging */
	bool enable_hv_charging;

	/* pe */
	bool enable_pe_plus;
	struct mtk_pe pe;

	/* pe 2.0 */
	bool enable_pe_2;
	struct mtk_pe20 pe2;

	/* pe 3.0 */
	bool enable_pe_3;
	struct mtk_pe30 pe3;
	struct charger_device *dc_chg;

	/* pe 4.0 */
	bool enable_pe_4;
	struct mtk_pe40 pe4;

	/* type-C*/
	bool enable_type_c;

	/* pd */
	struct mtk_pdc pdc;

	int pd_type;
	struct tcpc_device *tcpc;
	struct notifier_block pd_nb;
	bool pd_reset;

	/* thread related */
	struct hrtimer charger_kthread_timer;
	struct gtimer charger_kthread_fgtimer;

	struct wake_lock charger_wakelock;
	struct mutex charger_lock;
	struct mutex charger_pd_lock;
	spinlock_t slock;
	unsigned int polling_interval;
	bool charger_thread_timeout;
	wait_queue_head_t  wait_que;
	bool charger_thread_polling;

	/* kpoc */
	atomic_t enable_kpoc_shdn;
};

/* charger related module interface */
extern int charger_manager_notifier(struct charger_manager *info, int event);
extern int mtk_switch_charging_init(struct charger_manager *);
extern int mtk_dual_switch_charging_init(struct charger_manager *);
extern int mtk_linear_charging_init(struct charger_manager *);
extern void _wake_up_charger(struct charger_manager *);
extern int mtk_get_dynamic_cv(struct charger_manager *info, unsigned int *cv);
extern bool is_dual_charger_supported(struct charger_manager *info);
extern int charger_enable_vbus_ovp(struct charger_manager *pinfo, bool enable);
extern bool is_typec_adapter(struct charger_manager *info);

/* pmic API */
extern unsigned int upmu_get_rgs_chrdet(void);
extern int pmic_get_vbus(void);
extern int pmic_get_charging_current(void);
extern int pmic_get_battery_voltage(void);
extern int pmic_get_bif_battery_voltage(int *vbat);
extern int pmic_is_bif_exist(void);
extern int pmic_enable_hw_vbus_ovp(bool enable);
extern bool pmic_is_battery_exist(void);

/* add legacy battery API */
extern unsigned int battery_get_bat_soc(void);
extern signed int battery_meter_get_battery_temperature(void);
extern bool battery_get_bat_current_sign(void);
extern signed int battery_get_bat_uisoc(void);
extern int get_ui_soc(void);

/* procfs */
#define PROC_FOPS_RW(name)							\
static int mtk_charger_##name##_open(struct inode *inode, struct file *file)	\
{										\
	return single_open(file, mtk_charger_##name##_show, PDE_DATA(inode));	\
}										\
static const struct file_operations mtk_charger_##name##_fops = {		\
	.owner = THIS_MODULE,							\
	.open = mtk_charger_##name##_open,					\
	.read = seq_read,							\
	.llseek = seq_lseek,							\
	.release = single_release,						\
	.write = mtk_charger_##name##_write,					\
}

#endif /* __MTK_CHARGER_INTF_H__ */

