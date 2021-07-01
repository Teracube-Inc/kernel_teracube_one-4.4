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
#ifndef __SH_SVP_H__
#define __SH_SVP_H__

#if defined(CONFIG_TRUSTONIC_TRUSTED_UI) ||\
	defined(CONFIG_BLOWFISH_TUI_SUPPORT) ||\
	defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
#define SSVP_TUI_REGION_ENABLE
#else
#undef SSVP_TUI_REGION_ENABLE
#endif

#if defined(CONFIG_MTK_PROT_MEM_SUPPORT)
#define SSVP_PROT_SHAREDMEM_REGION_ENABLE
#else
#undef SSVP_PROT_SHAREDMEM_REGION_ENABLE
#endif

#include "memory_ssmr.h"

#define SVP_REGION_IOC_MAGIC		'S'

#define SVP_REGION_IOC_ONLINE		_IOR(SVP_REGION_IOC_MAGIC, 2, int)
#define SVP_REGION_IOC_OFFLINE		_IOR(SVP_REGION_IOC_MAGIC, 4, int)

#define SVP_REGION_ACQUIRE			_IOR(SVP_REGION_IOC_MAGIC, 6, int)
#define SVP_REGION_RELEASE			_IOR(SVP_REGION_IOC_MAGIC, 8, int)

#define UPPER_LIMIT32 (1ULL << 32)
#define UPPER_LIMIT64 (1ULL << 63)

#define NAME_LEN 32
#define CMD_LEN  64

enum region_type {
	SSVP_SVP,
#ifdef SSVP_TUI_REGION_ENABLE
	SSVP_TUI,
#endif
#ifdef SSVP_PROT_SHAREDMEM_REGION_ENABLE
	SSVP_PROT_SHAREDMEM,
#endif
	__MAX_NR_SSVPSUBS,
};

#define SSMR_INVALID_FEATURE(f) (f >= __MAX_NR_SSMR_FEATURES)
#define SSMR_INVALID_REGION(r) (r >= __MAX_NR_SSVPSUBS)

struct SSMR_Feature {
	char dt_prop_name[NAME_LEN];
	char feat_name[NAME_LEN];
	char cmd_online[CMD_LEN];
	char cmd_offline[CMD_LEN];
	u64 req_size;
	unsigned int region;
};

enum svp_state {
	SVP_STATE_DISABLED,
	SVP_STATE_ONING_WAIT,
	SVP_STATE_ONING,
	SVP_STATE_ON,
	SVP_STATE_OFFING,
	SVP_STATE_OFF,
	NR_STATES,
};

const char *const svp_state_text[NR_STATES] = {
	[SVP_STATE_DISABLED]   = "[DISABLED]",
	[SVP_STATE_ONING_WAIT] = "[ONING_WAIT]",
	[SVP_STATE_ONING]      = "[ONING]",
	[SVP_STATE_ON]         = "[ON]",
	[SVP_STATE_OFFING]     = "[OFFING]",
	[SVP_STATE_OFF]        = "[OFF]",
};

/*
 *  name :             region name
 *  proc_entry_fops :  file operation fun pointer
 *  state :            region online/offline state
 *  count :            region max alloc size by feature
 *  alloc_pages :      current feature offline alloc size
 *  is_unmapping :     unmapping state
 *  use_cache_memory : when use reserved memory it will be true
 *  page :             zmc alloc page
 *  cache_page :       alloc page by reserved memory
 *  usable_size :      cma usage size
 *  cur_feat :         current feature in use
 */
struct SSVP_Region {
	char name[NAME_LEN];
	const struct file_operations *proc_entry_fops;
	unsigned int state;
	unsigned long count;
	unsigned long alloc_pages;
	bool is_unmapping;
	bool use_cache_memory;
	struct page *page;
	struct page *cache_page;
	u64 usable_size;
	unsigned int cur_feat;
};

static struct SSMR_Feature _ssmr_feats[__MAX_NR_SSMR_FEATURES] = {
	[SSMR_FEAT_SVP] = {
		.dt_prop_name = "svp-size",
		.feat_name = "svp",
		.cmd_online = "svp=on",
		.cmd_offline = "svp=off",
		.req_size = (CONFIG_MTK_SVP_RAM_SIZE * SZ_1M),
		.region = SSVP_SVP
	},
#ifdef CONFIG_MTK_IRIS_SUPPORT
	[SSMR_FEAT_IRIS] = {
		.dt_prop_name = "iris-recognition-size",
		.feat_name = "iris",
		.cmd_online = "iris=on",
		.cmd_offline = "iris=off",
		.region = SSVP_SVP
	},
#endif
#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT
	[SSMR_FEAT_2D_FR] = {
		.dt_prop_name = "2d_fr-size",
		.feat_name = "2d_fr",
		.cmd_online = "2d_fr=on",
		.cmd_offline = "2d_fr=off",
		.region = SSVP_SVP
	},
#endif
#if defined(CONFIG_TRUSTONIC_TRUSTED_UI) ||\
	defined(CONFIG_BLOWFISH_TUI_SUPPORT)
	[SSMR_FEAT_TUI] = {
		.dt_prop_name = "tui-size",
		.feat_name = "tui",
		.cmd_online = "tui=on",
		.cmd_offline = "tui=off",
		.req_size = (CONFIG_MTK_TUI_RAM_SIZE * SZ_1M),
		.region = SSVP_TUI
	},
#endif
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	[SSMR_FEAT_WFD] = {
		.dt_prop_name = "wfd-size",
		.feat_name = "wfd",
		.cmd_online = "wfd=on",
		.cmd_offline = "wfd=off",
		.region = SSVP_TUI
	},
#endif
#ifdef CONFIG_MTK_PROT_MEM_SUPPORT
	[SSMR_FEAT_PROT_SHAREDMEM] = {
		.dt_prop_name = "prot-sharedmem-size",
		.feat_name = "prot-sharedmem",
		.cmd_online = "prot_sharedmem=on",
		.cmd_offline = "prot_sharedmem=off",
		.region = SSVP_PROT_SHAREDMEM
	}
#endif
};

static struct SSVP_Region _svpregs[__MAX_NR_SSVPSUBS] = {
	[SSVP_SVP] = {
		.name = "svp_region",
		.cur_feat = __MAX_NR_SSMR_FEATURES
	},
#ifdef SSVP_TUI_REGION_ENABLE
	[SSVP_TUI] = {
		.name = "tui_region",
		.cur_feat = __MAX_NR_SSMR_FEATURES
	},
#endif
#ifdef SSVP_PROT_SHAREDMEM_REGION_ENABLE
	[SSVP_PROT_SHAREDMEM] = {
		.name = "prot_sharedmem_region",
		.cur_feat = __MAX_NR_SSMR_FEATURES
	}
#endif
};

extern void show_pte(struct mm_struct *mm, unsigned long addr);

#ifdef CONFIG_MTK_ION
extern void ion_sec_heap_dump_info(void);
#else
static inline void ion_sec_heap_dump_info(void)
{
	pr_info("%s is not supported\n", __func__);
}
#endif

#endif
