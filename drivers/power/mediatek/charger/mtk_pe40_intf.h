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

#ifndef __MTK_PE_40_INTF_H
#define __MTK_PE_40_INTF_H

#define CONFIG_MTK_PUMP_EXPRESS_PLUS_40_SUPPORT
#include <mtk_direct_charge_vdm.h>

#define VBUS_IDX_MAX 5

struct pe40_power_cap {
	uint8_t selected_cap_idx;
	uint8_t nr;
	uint8_t pwr_limit[PDO_MAX_NR];
	int max_mv[PDO_MAX_NR];
	int min_mv[PDO_MAX_NR];
	int ma[PDO_MAX_NR];
	int maxwatt[PDO_MAX_NR];
	int minwatt[PDO_MAX_NR];
};

struct pe4_pps_status {
	int output_mv;	/* 0xffff means no support */
	int output_ma;	/* 0xff means no support */
	uint8_t real_time_flags;
};


struct mtk_pe40 {
	bool is_connect;
	bool is_enabled;
	bool can_query;
	struct pe40_power_cap cap;

	int avbus;
	int vbus;
	int ibus;
	int watt;

	int r_sw;
	int r_cable;
	int r_cable_1;
	int r_cable_2;

	int pmic_vbus;
	int TA_vbus;
	int vbus_cali;

	int max_charger_ibus;
	int max_vbus;
	int max_ibus;

	int pe4_input_current_limit;
	int pe4_input_current_limit_setting;
	int input_current_limit;

};

#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_40_SUPPORT
extern bool mtk_pe40_init(struct charger_manager *pinfo);
extern bool mtk_is_TA_support_pd_pps(struct charger_manager *info);
extern bool mtk_pe40_is_ready(struct charger_manager *pinfo);
extern bool mtk_pe40_get_is_connect(struct charger_manager *pinfo);
extern void mtk_pe40_set_is_enable(struct charger_manager *pinfo, bool enable);
extern bool mtk_pe40_get_is_enable(struct charger_manager *pinfo);
extern int mtk_pe40_init_state(struct charger_manager *pinfo);
extern int mtk_pe40_tune1_state(struct charger_manager *pinfo);
extern int mtk_pe40_tune2_state(struct charger_manager *pinfo);
extern int mtk_pe40_cc_state(struct charger_manager *pinfo);
extern void mtk_pe40_plugout_reset(struct charger_manager *pinfo);
extern void mtk_pe40_end(struct charger_manager *pinfo, int type, bool retry);
#else

#endif

#endif /* __MTK_PE_40_INTF_H */

