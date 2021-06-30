/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MT_GPUFREQ_H
#define _MT_GPUFREQ_H

#include <linux/module.h>
#include <linux/clk.h>

#define MAX_VCO_VALUE	3800000
#define MIN_VCO_VALUE	1500000
#define DIV4_MAX_FREQ	(MAX_VCO_VALUE / (POST_DIV4 << 2))
#define DIV4_MIN_FREQ	(MIN_VCO_VALUE / (POST_DIV4 << 2))

/* For MT6757, we can't power off GPU since HW limitation */
/* And this will cause power budge issue, so we add an extra para to support skipping PBM operation */
#define GPUFREQ_ENABLE_KICK_PBM

enum post_div_enum {
	POST_DIV2 = 0,
	POST_DIV4,
	POST_DIV8,
};

struct mt_gpufreq_table_info {
	unsigned int gpufreq_khz;
	unsigned int gpufreq_volt;
	unsigned int gpufreq_idx;
};

struct mt_gpufreq_power_table_info {
	unsigned int gpufreq_khz;
	unsigned int gpufreq_volt;
	unsigned int gpufreq_power;
};

struct mt_gpufreq_clk_t {
	struct clk *clk_mux;          /* main clock for mfg setting */
	struct clk *clk_main_parent;	 /* substitution clock for mfg transient mux setting */
	struct clk *clk_sub_parent;	 /* substitution clock for mfg transient parent setting */
};
struct mt_gpufreq_pmic_t {
	struct regulator *reg_vgpu;	/* vgpu regulator */
	struct regulator *reg_vsram;	/* vgpu sram regulator */
};
/*****************
 * extern function
 ******************/
extern int mt_gpufreq_state_set(int enabled);
extern void mt_gpufreq_thermal_protect(unsigned int limited_power);
extern unsigned int mt_gpufreq_get_cur_freq_index(void);
extern unsigned int mt_gpufreq_get_cur_freq(void);
extern unsigned int mt_gpufreq_get_cur_volt(void);
extern unsigned int mt_gpufreq_get_dvfs_table_num(void);
extern unsigned int mt_gpufreq_target(unsigned int idx);
/*Special API for Kibo+, found an issue that when GPU power off, driver will kick PBM to lower resource required*/
extern unsigned int mt_gpufreq_target_skip_kick_pbm(unsigned int idx);
extern unsigned int mt_gpufreq_voltage_enable_set(unsigned int enable);
extern unsigned int mt_gpufreq_update_volt(unsigned int pmic_volt[], unsigned int array_size);
extern unsigned int mt_gpufreq_get_freq_by_idx(unsigned int idx);
extern unsigned int mt_gpufreq_get_volt_by_idx(unsigned int idx);
extern void mt_gpufreq_thermal_protect(unsigned int limited_power);
extern void mt_gpufreq_restore_default_volt(void);
extern void mt_gpufreq_enable_by_ptpod(void);
extern void mt_gpufreq_disable_by_ptpod(void);
extern unsigned int mt_gpufreq_get_max_power(void);
extern unsigned int mt_gpufreq_get_min_power(void);
extern unsigned int mt_gpufreq_get_thermal_limit_index(void);
extern unsigned int mt_gpufreq_get_thermal_limit_freq(void);
extern void mt_gpufreq_set_power_limit_by_pbm(unsigned int limited_power);
extern unsigned int mt_gpufreq_get_leakage_mw(void);

extern unsigned int mt_get_mfgclk_freq(void);	/* Freq Meter API */
extern u32 get_devinfo_with_index(u32 index);
extern int mt_gpufreq_fan53555_init(void);
/* #ifdef MT_GPUFREQ_AEE_RR_REC */
extern void aee_rr_rec_gpu_dvfs_vgpu(u8 val);
extern void aee_rr_rec_gpu_dvfs_oppidx(u8 val);
extern void aee_rr_rec_gpu_dvfs_status(u8 val);
extern u8 aee_rr_curr_gpu_dvfs_status(void);
/* #endif */

/*****************
 * power limit notification
 ******************/
typedef void (*gpufreq_power_limit_notify)(unsigned int);
extern void mt_gpufreq_power_limit_notify_registerCB(gpufreq_power_limit_notify pCB);

/*****************
 * input boost notification
 ******************/
typedef void (*gpufreq_input_boost_notify)(unsigned int);
extern void mt_gpufreq_input_boost_notify_registerCB(gpufreq_input_boost_notify pCB);

/*****************
 * update voltage notification
 ******************/
typedef void (*gpufreq_ptpod_update_notify)(void);
extern void mt_gpufreq_update_volt_registerCB(gpufreq_ptpod_update_notify pCB);

/*****************
 * profiling purpose
 ******************/
typedef void (*sampler_func)(unsigned int);
extern void mt_gpufreq_setfreq_registerCB(sampler_func pCB);
extern void mt_gpufreq_setvolt_registerCB(sampler_func pCB);

#ifdef MTK_GPU_SPM
void mtk_gpu_spm_fix_by_idx(unsigned int idx);
void mtk_gpu_spm_reset_fix(void);
void mtk_gpu_spm_pause(void);
void mtk_gpu_spm_resume(void);
#endif

#endif
