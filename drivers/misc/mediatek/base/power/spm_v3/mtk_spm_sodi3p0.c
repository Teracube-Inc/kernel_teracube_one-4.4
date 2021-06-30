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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mt-plat/mtk_secure_api.h>

/* #include <mach/irqs.h> */
#if defined(CONFIG_WATCHDOG) && defined(CONFIG_MTK_WATCHDOG) && \
	defined(CONFIG_MTK_WD_KICKER)
#include <mach/wd_api.h>
#endif

#include <mt-plat/mtk_boot.h>
#if defined(CONFIG_MTK_SYS_CIRQ)
#include <mt-plat/mtk_cirq.h>
#endif
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_io.h>

#include <mtk_spm_sodi.h>
#include <mtk_spm_sodi3.h>
#include <mtk_spm_resource_req.h>
#include <mtk_spm_resource_req_internal.h>
#include <mtk_spm_pmic_wrap.h>
#if defined(CONFIG_MACH_MT6799)
#include <mt6337_api.h>
#endif
#include <mtk_power_gs_api.h>

#ifdef CONFIG_MTK_ICCS_SUPPORT
#include <mtk_hps_internal.h>
#endif

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
#include <sspm_define.h>
#include <sspm_timesync.h>
#endif

#if !defined(CONFIG_FPGA_EARLY_PORTING)
#include <trace/events/mtk_idle_event.h>
#endif
#include <mtk_idle_internal.h>
#include <mtk_idle_profile.h>

/**************************************
 * only for internal debug
 **************************************/
#define PCM_SEC_TO_TICK(sec)	(sec * 32768)
#if defined(CONFIG_MACH_MT6759) || defined(CONFIG_MACH_MT6758) || \
	defined(CONFIG_MACH_MT6775)
#define SPM_PCMWDT_EN		(0)
#else
#define SPM_PCMWDT_EN		(1)
#endif

static struct pwr_ctrl sodi3_ctrl;

struct spm_lp_scen __spm_sodi3 = {
	.pwrctrl = &sodi3_ctrl,
};

static bool gSpm_sodi3_en = true;

static void spm_sodi3_pre_process(struct pwr_ctrl *pwrctrl, u32 operation_cond)
{
#ifndef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
#ifdef CONFIG_MACH_MT6799
	unsigned int value = 0;

	/* Set PMIC wrap table for Vproc/Vsram voltage decreased */
	/* VSRAM_DVFS1 */
	pmic_read_interface_nolock(PMIC_RG_VSRAM_DVFS1_VOSEL_ADDR,
								&value,
								PMIC_RG_VSRAM_DVFS1_VOSEL_MASK,
								PMIC_RG_VSRAM_DVFS1_VOSEL_SHIFT);

	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_ALLINONE,
								IDX_ALL_1_VSRAM_NORMAL,
								value);

	/* VSRAM_DVFS2 */
	pmic_read_interface_nolock(PMIC_RG_VSRAM_DVFS2_VOSEL_ADDR,
								&value,
								PMIC_RG_VSRAM_DVFS2_VOSEL_MASK,
								PMIC_RG_VSRAM_DVFS2_VOSEL_SHIFT);

	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_ALLINONE,
								IDX_ALL_2_VSRAM_NORMAL,
								value);

	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_ALLINONE,
								IDX_ALL_VCORE_SUSPEND,
								pwrctrl->vcore_volt_pmic_val);
	wk_mt6337_set_lp_setting();
#endif
	mt_spm_pmic_wrap_set_phase(PMIC_WRAP_PHASE_ALLINONE);

	spm_pmic_power_mode(PMIC_PWR_SODI3, 0, 0);
	spm_pmic_vcore_setting((operation_cond & DEEPIDLE_OPT_VCORE_LP_MODE)?1:0);
#endif
	__spm_sync_pcm_flags(pwrctrl);
}

static void spm_sodi3_post_process(void)
{
#ifndef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	mt_spm_pmic_wrap_set_phase(PMIC_WRAP_PHASE_ALLINONE);
#ifdef CONFIG_MACH_MT6799
	wk_mt6337_restore_lp_setting();
#endif
#endif
}

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
static void spm_sodi3_notify_sspm_before_wfi(struct pwr_ctrl *pwrctrl, u32 operation_cond)
{
	int ret;
	struct spm_data spm_d;
	unsigned int spm_opt = 0;

	memset(&spm_d, 0, sizeof(struct spm_data));

#ifdef SSPM_TIMESYNC_SUPPORT
	sspm_timesync_ts_get(&spm_d.u.suspend.sys_timestamp_h, &spm_d.u.suspend.sys_timestamp_l);
	sspm_timesync_clk_get(&spm_d.u.suspend.sys_src_clk_h, &spm_d.u.suspend.sys_src_clk_l);
#endif

	spm_opt |= univpll_is_used() ? SPM_OPT_UNIVPLL_STAT : 0;
	spm_opt |= spm_for_gps_flag ?  SPM_OPT_GPS_STAT     : 0;
	spm_opt |= (operation_cond & DEEPIDLE_OPT_VCORE_LP_MODE) ?
		SPM_OPT_VCORE_LP_MODE : 0;
	spm_opt |= (operation_cond & DEEPIDLE_OPT_XO_UFS_ON_OFF) ?
		SPM_OPT_XO_UFS_OFF : 0;

	spm_d.u.suspend.spm_opt = spm_opt;
	spm_d.u.suspend.vcore_volt_pmic_val = pwrctrl->vcore_volt_pmic_val;

	ret = spm_to_sspm_command_async(SPM_ENTER_SODI3, &spm_d);
	if (ret < 0)
		spm_crit2("ret %d", ret);
}

static void spm_sodi3_notify_sspm_before_wfi_async_wait(void)
{
	int ret = 0;

	ret = spm_to_sspm_command_async_wait(SPM_ENTER_SODI3);
	if (ret < 0)
		spm_crit2("SPM_ENTER_SODI3 async wait: ret %d", ret);
}

static void spm_sodi3_notify_sspm_after_wfi(u32 operation_cond)
{
	int ret;
	struct spm_data spm_d;
	unsigned int spm_opt = 0;

	memset(&spm_d, 0, sizeof(struct spm_data));

#ifdef SSPM_TIMESYNC_SUPPORT
	sspm_timesync_ts_get(&spm_d.u.suspend.sys_timestamp_h, &spm_d.u.suspend.sys_timestamp_l);
	sspm_timesync_clk_get(&spm_d.u.suspend.sys_src_clk_h, &spm_d.u.suspend.sys_src_clk_l);
#endif

	spm_opt |= (operation_cond & DEEPIDLE_OPT_XO_UFS_ON_OFF) ?
		SPM_OPT_XO_UFS_OFF : 0;

	spm_d.u.suspend.spm_opt = spm_opt;

	ret = spm_to_sspm_command_async(SPM_LEAVE_SODI3, &spm_d);
	if (ret < 0)
		spm_crit2("ret %d", ret);
}

static void spm_sodi3_notify_sspm_after_wfi_async_wait(void)
{
	int ret = 0;

	ret = spm_to_sspm_command_async_wait(SPM_LEAVE_SODI3);
	if (ret < 0)
		spm_crit2("SPM_LEAVE_SODI3 async wait: ret %d", ret);
}

#else /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
static void spm_sodi3_notify_sspm_before_wfi(struct pwr_ctrl *pwrctrl, u32 operation_cond)
{
}

static void spm_sodi3_notify_sspm_before_wfi_async_wait(void)
{
}

static void spm_sodi3_notify_sspm_after_wfi(u32 operation_cond)
{
}

static void spm_sodi3_notify_sspm_after_wfi_async_wait(void)
{
}

#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

static void spm_sodi3_pcm_setup_before_wfi(
	u32 cpu, struct pcm_desc *pcmdesc, struct pwr_ctrl *pwrctrl, u32 operation_cond)
{
	unsigned int resource_usage;

	spm_sodi3_pre_process(pwrctrl, operation_cond);

	/* Get SPM resource request and update reg_spm_xxx_req */
	resource_usage = spm_get_resource_usage();
#if defined(CONFIG_MACH_MT6775)
	mt_secure_call(MTK_SIP_KERNEL_SPM_SODI_ARGS,
		pwrctrl->pcm_flags, resource_usage, pwrctrl->pcm_flags1);
#else
	mt_secure_call(MTK_SIP_KERNEL_SPM_SODI_ARGS,
		pwrctrl->pcm_flags, resource_usage, pwrctrl->timer_val);
#endif
	mt_secure_call(MTK_SIP_KERNEL_SPM_PWR_CTRL_ARGS,
		SPM_PWR_CTRL_SODI, PWR_OPP_LEVEL, pwrctrl->opp_level);
	mt_secure_call(MTK_SIP_KERNEL_SPM_PWR_CTRL_ARGS,
		SPM_PWR_CTRL_SODI, PWR_WDT_DISABLE, pwrctrl->wdt_disable);
}

static void spm_sodi3_pcm_setup_after_wfi(struct pwr_ctrl *pwrctrl, u32 operation_cond)
{
	spm_sodi3_post_process();
}

static void spm_sodi3_setup_wdt(struct pwr_ctrl *pwrctrl, void **api)
{
#if SPM_PCMWDT_EN && defined(CONFIG_WATCHDOG) && \
	defined(CONFIG_MTK_WATCHDOG) && defined(CONFIG_MTK_WD_KICKER)
	struct wd_api *wd_api = NULL;

	if (!get_wd_api(&wd_api)) {
		wd_api->wd_spmwdt_mode_config(WD_REQ_EN, WD_REQ_RST_MODE);
		wd_api->wd_suspend_notify();
		pwrctrl->wdt_disable = 0;
	} else {
		spm_crit2("FAILED TO GET WD API\n");
		api = NULL;
		pwrctrl->wdt_disable = 1;
	}
	*api = wd_api;
#else
	pwrctrl->wdt_disable = 1;
#endif
}

static void spm_sodi3_resume_wdt(struct pwr_ctrl *pwrctrl, void *api)
{
#if SPM_PCMWDT_EN && defined(CONFIG_WATCHDOG) && \
	defined(CONFIG_MTK_WATCHDOG) && defined(CONFIG_MTK_WD_KICKER)
	struct wd_api *wd_api = (struct wd_api *)api;

	if (!pwrctrl->wdt_disable && wd_api != NULL) {
		wd_api->wd_resume_notify();
		wd_api->wd_spmwdt_mode_config(WD_REQ_DIS, WD_REQ_RST_MODE);
	} else {
		spm_crit2("pwrctrl->wdt_disable %d\n", pwrctrl->wdt_disable);
	}
#endif
}

static void spm_sodi3_atf_time_sync(void)
{
	/* Get local_clock and sync to ATF */
	u64 time_to_sync = local_clock();

#ifdef CONFIG_ARM64
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC, time_to_sync, 0, 0);
#else
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC,
			(u32)time_to_sync, (u32)(time_to_sync >> 32), 0);
#endif
	sodi3_pr_debug("atf_time_sync\n");
}

unsigned int spm_go_to_sodi3(u32 spm_flags, u32 spm_data, u32 sodi3_flags, u32 operation_cond)
{
	void *api = NULL;
	struct wake_status wakesta;
	unsigned long flags;
#if defined(CONFIG_MTK_GIC_V3_EXT)
	struct mtk_irq_mask mask;
#endif
	unsigned int wr = WR_NONE;
	struct pcm_desc *pcmdesc = NULL;
	struct pwr_ctrl *pwrctrl = __spm_sodi3.pwrctrl;
	u32 cpu = smp_processor_id();
	int ch;

	spm_sodi3_footprint(SPM_SODI3_ENTER);

#ifdef SUPPORT_SW_SET_SPM_MEMEPLL_MODE
	if (spm_get_sodi_mempll() == 1)
		spm_flags |= SPM_FLAG_SODI_CG_MODE; /* CG mode */
	else
		spm_flags &= ~SPM_FLAG_SODI_CG_MODE; /* PDN mode */
#endif

	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);
#if defined(CONFIG_MACH_MT6775)
	if (is_big_buck_pdn_by_spm()) {
		spm_data |= (SPM_RSV_CON2_BIG_BUCK_ON_EN |
			     SPM_RSV_CON2_BIG_BUCK_OFF_EN);
	}

	set_pwrctrl_pcm_flags1(pwrctrl, spm_data);
#endif
	/* need be called after set_pwrctrl_pcm_flags1() */
	/* spm_set_dummy_read_addr(false); */

	/* for gps only case */
	if (spm_for_gps_flag) {
		/* sodi3_pr_debug("spm_for_gps_flag %d\n", spm_for_gps_flag); */
		pwrctrl->pcm_flags |= SPM_FLAG_DIS_ULPOSC_OFF;
	}

	pwrctrl->timer_val = PCM_SEC_TO_TICK(2);

	spm_sodi3_setup_wdt(pwrctrl, &api);
	soidle3_before_wfi(cpu);

	/* need be called before spin_lock_irqsave() */
	ch = get_channel_lock(0);
	pwrctrl->opp_level = __spm_check_opp_level(ch);
	pwrctrl->vcore_volt_pmic_val = __spm_get_vcore_volt_pmic_val(true, ch);
	wakesta.dcs_ch = (u32)ch;

	/* update pcm_flags with dcs flag */
	__spm_update_pcm_flags_dcs_workaround(pwrctrl, ch);

	spin_lock_irqsave(&__spm_lock, flags);

#ifdef CONFIG_MTK_ICCS_SUPPORT
	iccs_enter_low_power_state();
#endif

	profile_so3_start(PIDX_SSPM_BEFORE_WFI);
	spm_sodi3_notify_sspm_before_wfi(pwrctrl, operation_cond);
	profile_so3_end(PIDX_SSPM_BEFORE_WFI);

	profile_so3_start(PIDX_PRE_IRQ_PROCESS);
#if defined(CONFIG_MTK_GIC_V3_EXT)
	mt_irq_mask_all(&mask);
	mt_irq_unmask_for_sleep_ex(SPM_IRQ0_ID);
	unmask_edge_trig_irqs_for_cirq();
#endif

#if defined(CONFIG_MTK_SYS_CIRQ)
	mt_cirq_clone_gic();
	mt_cirq_enable();
#endif
	profile_so3_end(PIDX_PRE_IRQ_PROCESS);

	spm_sodi3_footprint(SPM_SODI3_ENTER_SPM_FLOW);

	profile_so3_start(PIDX_PCM_SETUP_BEFORE_WFI);
	spm_sodi3_pcm_setup_before_wfi(cpu, pcmdesc, pwrctrl, operation_cond);
	profile_so3_end(PIDX_PCM_SETUP_BEFORE_WFI);

	spm_sodi3_footprint(SPM_SODI3_ENTER_SSPM_ASYNC_IPI_BEFORE_WFI);

	profile_so3_start(PIDX_SSPM_BEFORE_WFI_ASYNC_WAIT);
	spm_sodi3_notify_sspm_before_wfi_async_wait();
	profile_so3_end(PIDX_SSPM_BEFORE_WFI_ASYNC_WAIT);

	spm_sodi3_footprint(SPM_SODI3_ENTER_UART_SLEEP);

#if defined(CONFIG_MTK_SERIAL)
	if (!(sodi3_flags & SODI_FLAG_DUMP_LP_GS)) {
		if (request_uart_to_sleep()) {
			wr = WR_UART_BUSY;
			goto RESTORE_IRQ;
		}
	}
#endif

	spm_sodi3_footprint_val((1 << SPM_SODI3_ENTER_WFI) |
		(1 << SPM_SODI3_B4) | (1 << SPM_SODI3_B5) | (1 << SPM_SODI3_B6));

	if (sodi3_flags & SODI_FLAG_DUMP_LP_GS)
		mt_power_gs_dump_sodi3();

#if !defined(CONFIG_FPGA_EARLY_PORTING)
	trace_sodi3_rcuidle(cpu, 1);
#endif

	profile_so3_end(PIDX_ENTER_TOTAL);

	spm_trigger_wfi_for_sodi(pwrctrl->pcm_flags);

	profile_so3_start(PIDX_LEAVE_TOTAL);

#if !defined(CONFIG_FPGA_EARLY_PORTING)
	trace_sodi3_rcuidle(cpu, 0);
#endif

	spm_sodi3_footprint(SPM_SODI3_LEAVE_WFI);

#if defined(CONFIG_MTK_SERIAL)
	if (!(sodi3_flags & SODI_FLAG_DUMP_LP_GS))
		request_uart_to_wakeup();
RESTORE_IRQ:
#endif

	profile_so3_start(PIDX_SSPM_AFTER_WFI);
	spm_sodi3_notify_sspm_after_wfi(operation_cond);
	profile_so3_end(PIDX_SSPM_AFTER_WFI);

	spm_sodi3_footprint(SPM_SODI3_LEAVE_SSPM_ASYNC_IPI_AFTER_WFI);

	__spm_get_wakeup_status(&wakesta);

	profile_so3_start(PIDX_PCM_SETUP_AFTER_WFI);
	spm_sodi3_pcm_setup_after_wfi(pwrctrl, operation_cond);
	profile_so3_end(PIDX_PCM_SETUP_AFTER_WFI);

	spm_sodi3_footprint(SPM_SODI3_LEAVE_SPM_FLOW);

	if (wr == WR_UART_BUSY)
		sodi3_pr_info("request uart sleep: fail\n");
	else
		wr = spm_sodi_output_log(&wakesta, pcmdesc, sodi3_flags|SODI_FLAG_3P0, operation_cond);

	spm_sodi3_footprint(SPM_SODI3_ENTER_UART_AWAKE);

	profile_so3_start(PIDX_POST_IRQ_PROCESS);
#if defined(CONFIG_MTK_SYS_CIRQ)
	mt_cirq_flush();
	mt_cirq_disable();
#endif

#if defined(CONFIG_MTK_GIC_V3_EXT)
	mt_irq_mask_restore(&mask);
#endif
	profile_so3_end(PIDX_POST_IRQ_PROCESS);

	spin_unlock_irqrestore(&__spm_lock, flags);

	/* need be called after spin_unlock_irqrestore() */
	get_channel_unlock();

	soidle3_after_wfi(cpu);

	profile_so3_start(PIDX_SSPM_AFTER_WFI_ASYNC_WAIT);
	spm_sodi3_notify_sspm_after_wfi_async_wait();
	profile_so3_end(PIDX_SSPM_AFTER_WFI_ASYNC_WAIT);

	spm_sodi3_resume_wdt(pwrctrl, api);

	spm_sodi3_atf_time_sync();

	spm_sodi3_reset_footprint();

	if (wr == WR_PCM_ASSERT)
		rekick_vcorefs_scenario();

	return wr;
}

void spm_enable_sodi3(bool en)
{
	gSpm_sodi3_en = en;
}

bool spm_get_sodi3_en(void)
{
	return gSpm_sodi3_en;
}

void spm_sodi3_init(void)
{
	sodi3_pr_debug("spm_sodi3_init\n");
	spm_sodi3_aee_init();
#if defined(CONFIG_MACH_MT6758) || defined(CONFIG_MACH_MT6799)
	sodi3_ctrl.wake_src = WAKE_SRC_FOR_SODI;
	mt_secure_call(MTK_SIP_KERNEL_SPM_PWR_CTRL_ARGS,
		SPM_PWR_CTRL_SODI, PWR_WAKE_SRC, sodi3_ctrl.wake_src);
#endif
}

MODULE_DESCRIPTION("SPM-SODI3 Driver v0.1");
