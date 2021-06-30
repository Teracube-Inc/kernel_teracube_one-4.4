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

#ifndef _COMMON_MTK_UPOWER_H
#define _COMMON_MTK_UPOWER_H

#if defined(CONFIG_MACH_MT6759) || defined(CONFIG_MACH_MT6763) || defined(CONFIG_MACH_MT6758) \
	|| defined(CONFIG_MACH_MT6739) || defined(CONFIG_MACH_MT6771) || defined(CONFIG_MACH_MT6775)
#include "upower_v2/mtk_unified_power.h"
#endif

#if defined(CONFIG_MACH_MT6799)
#include "upower_v1/mtk_unified_power.h"
#endif

#endif /* _COMMON_MTK_UPOWER_H */

