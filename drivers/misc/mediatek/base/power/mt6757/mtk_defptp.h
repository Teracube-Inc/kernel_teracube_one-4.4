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

#ifndef _MT_DEFEEM_
#define _MT_DEFEEM_

#ifdef __KERNEL__
	#include <linux/kernel.h>
#endif

#define NUM_ELM_SRAM 8

#ifdef __KERNEL__
	#define EEMCONF_S       0x0011C010
	#define EEMCONF_E       0x0011C1C0
	#define EEMCONF_SIZE    (EEMCONF_E - EEMCONF_S)

	extern void __iomem *eem_base;
	#define EEM_BASEADDR eem_base

	#ifdef CONFIG_OF
	struct devinfo_ptp_tag {
		u32 size;
		u32 tag;
		u32 volt0;
		u32 volt1;
		u32 volt2;
		u32 have_550;
	};
	#endif
#endif

#define TEMPMONCTL0         (EEM_BASEADDR + 0x000)
#define TEMPMONCTL1         (EEM_BASEADDR + 0x004)
#define TEMPMONCTL2         (EEM_BASEADDR + 0x008)
#define TEMPMONINT          (EEM_BASEADDR + 0x00C)
#define TEMPMONINTSTS       (EEM_BASEADDR + 0x010)
#define TEMPMONIDET0        (EEM_BASEADDR + 0x014)
#define TEMPMONIDET1        (EEM_BASEADDR + 0x018)
#define TEMPMONIDET2        (EEM_BASEADDR + 0x01C)
#define TEMPH2NTHRE         (EEM_BASEADDR + 0x024)
#define TEMPHTHRE           (EEM_BASEADDR + 0x028)
#define TEMPCTHRE           (EEM_BASEADDR + 0x02C)
#define TEMPOFFSETH         (EEM_BASEADDR + 0x030)
#define TEMPOFFSETL         (EEM_BASEADDR + 0x034)
#define TEMPMSRCTL0         (EEM_BASEADDR + 0x038)
#define TEMPMSRCTL1         (EEM_BASEADDR + 0x03C)
#define TEMPAHBPOLL         (EEM_BASEADDR + 0x040)
#define TEMPAHBTO           (EEM_BASEADDR + 0x044)
#define TEMPADCPNP0         (EEM_BASEADDR + 0x048)
#define TEMPADCPNP1         (EEM_BASEADDR + 0x04C)
#define TEMPADCPNP2         (EEM_BASEADDR + 0x050)
#define TEMPADCMUX          (EEM_BASEADDR + 0x054)
#define TEMPADCEXT          (EEM_BASEADDR + 0x058)
#define TEMPADCEXT1         (EEM_BASEADDR + 0x05C)
#define TEMPADCEN           (EEM_BASEADDR + 0x060)
#define TEMPPNPMUXADDR      (EEM_BASEADDR + 0x064)
#define TEMPADCMUXADDR      (EEM_BASEADDR + 0x068)
#define TEMPADCEXTADDR      (EEM_BASEADDR + 0x06C)
#define TEMPADCEXT1ADDR     (EEM_BASEADDR + 0x070)
#define TEMPADCENADDR       (EEM_BASEADDR + 0x074)
#define TEMPADCVALIDADDR    (EEM_BASEADDR + 0x078)
#define TEMPADCVOLTADDR     (EEM_BASEADDR + 0x07C)
#define TEMPRDCTRL          (EEM_BASEADDR + 0x080)
#define TEMPADCVALIDMASK    (EEM_BASEADDR + 0x084)
#define TEMPADCVOLTAGESHIFT (EEM_BASEADDR + 0x088)
#define TEMPADCWRITECTRL    (EEM_BASEADDR + 0x08C)
#define TEMPMSR0            (EEM_BASEADDR + 0x090)
#define TEMPMSR1            (EEM_BASEADDR + 0x094)
#define TEMPMSR2            (EEM_BASEADDR + 0x098)
#define TEMPIMMD0           (EEM_BASEADDR + 0x0A0)
#define TEMPIMMD1           (EEM_BASEADDR + 0x0A4)
#define TEMPIMMD2           (EEM_BASEADDR + 0x0A8)
#define TEMPMONIDET3        (EEM_BASEADDR + 0x0B0)
#define TEMPADCPNP3         (EEM_BASEADDR + 0x0B4)
#define TEMPMSR3            (EEM_BASEADDR + 0x0B8)
#define TEMPIMMD3           (EEM_BASEADDR + 0x0BC)
#define TEMPPROTCTL         (EEM_BASEADDR + 0x0C0)
#define TEMPPROTTA          (EEM_BASEADDR + 0x0C4)
#define TEMPPROTTB          (EEM_BASEADDR + 0x0C8)
#define TEMPPROTTC          (EEM_BASEADDR + 0x0CC)
#define TEMPSPARE0          (EEM_BASEADDR + 0x0F0)
#define TEMPSPARE1          (EEM_BASEADDR + 0x0F4)
#define TEMPSPARE2          (EEM_BASEADDR + 0x0F8)
#define REVISIONID		(EEM_BASEADDR + 0x0FC)
#define DESCHAR			(EEM_BASEADDR + 0x200)
#define TEMPCHAR		(EEM_BASEADDR + 0x204)
#define DETCHAR			(EEM_BASEADDR + 0x208)
#define AGECHAR			(EEM_BASEADDR + 0x20C)
#define EEM_DCCONFIG		(EEM_BASEADDR + 0x210)
#define EEM_AGECONFIG		(EEM_BASEADDR + 0x214)
#define FREQPCT30		(EEM_BASEADDR + 0x218)
#define FREQPCT74		(EEM_BASEADDR + 0x21C)
#define LIMITVALS		(EEM_BASEADDR + 0x220)
#define EEM_VBOOT		(EEM_BASEADDR + 0x224)
#define EEM_DETWINDOW		(EEM_BASEADDR + 0x228)
#define EEMCONFIG		(EEM_BASEADDR + 0x22C)
#define TSCALCS			(EEM_BASEADDR + 0x230)
#define RUNCONFIG		(EEM_BASEADDR + 0x234)
#define EEMEN			(EEM_BASEADDR + 0x238)
#define INIT2VALS		(EEM_BASEADDR + 0x23C)
#define DCVALUES		(EEM_BASEADDR + 0x240)
#define AGEVALUES		(EEM_BASEADDR + 0x244)
#define VOP30			(EEM_BASEADDR + 0x248)
#define VOP74			(EEM_BASEADDR + 0x24C)
#define TEMP			(EEM_BASEADDR + 0x250)
#define EEMINTSTS		(EEM_BASEADDR + 0x254)
#define EEMINTSTSRAW		(EEM_BASEADDR + 0x258)
#define EEMINTEN		(EEM_BASEADDR + 0x25C)
#define VDESIGN30		(EEM_BASEADDR + 0x26C)
#define VDESIGN74		(EEM_BASEADDR + 0x270)
#define AGECOUNT		(EEM_BASEADDR + 0x27C)
#define SMSTATE0		(EEM_BASEADDR + 0x280)
#define SMSTATE1		(EEM_BASEADDR + 0x284)
#define EEMCORESEL		(EEM_BASEADDR + 0x400)
#define THERMINTST		(EEM_BASEADDR + 0x404)
#define EEMODINTST		(EEM_BASEADDR + 0x408)
#define THSTAGE0ST		(EEM_BASEADDR + 0x40C)
#define THSTAGE1ST		(EEM_BASEADDR + 0x410)
#define THSTAGE2ST		(EEM_BASEADDR + 0x414)
#define THAHBST0		(EEM_BASEADDR + 0x418)
#define THAHBST1		(EEM_BASEADDR + 0x41C)
#define EEMSPARE0		(EEM_BASEADDR + 0x420)
#define EEMSPARE1		(EEM_BASEADDR + 0x424)
#define EEMSPARE2		(EEM_BASEADDR + 0x428)
#define EEMSPARE3		(EEM_BASEADDR + 0x42C)
#define THSLPEVEB		(EEM_BASEADDR + 0x430)

#endif
