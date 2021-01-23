/*
 * Copyright (C) 2006-2017 ILITEK TECHNOLOGY CORP.
 *
 * Description: ILITEK I2C touchscreen driver for linux platform.
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
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Author: Johnson Yeh
 * Maintain: Luca Hsu, Tigers Huang, Dicky Chiang
 */

/**
 *
 * @file    ilitek_drv_hotknot_queue.h
 *
 * @brief   This file defines the queue structure for hotknot
 *
 *
 */

#ifndef __ILITEK_DRV_HOTKNOT_QUEUE_H__
#define __ILITEK_DRV_HOTKNOT_QUEUE_H__


////////////////////////////////////////////////////////////
/// Included Files
////////////////////////////////////////////////////////////

#include "ilitek_drv_common.h"


#ifdef CONFIG_ENABLE_HOTKNOT
////////////////////////////////////////////////////////////
/// Constant
////////////////////////////////////////////////////////////
#define HOTKNOT_QUEUE_SIZE               1024


////////////////////////////////////////////////////////////
/// Variables
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
/// Function Prototype
////////////////////////////////////////////////////////////
extern void CreateQueue(void);
extern void ClearQueue(void);
extern int PushQueue(u8 * pBuf, u16 nLength);
extern int PopQueue(u8 * pBuf, u16 nLength);
extern int ShowQueue(u8 * pBuf, u16 nLength);    //just show data, not fetch data
extern void ShowAllQueue(u8 * pBuf, u16 * pFront, u16 * pRear);    //just show data, not fetch data
extern void DeleteQueue(void);


#endif //CONFIG_ENABLE_HOTKNOT
#endif // __ILITEK_DRV_HOTKNOT_QUEUE_H__
