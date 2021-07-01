#include <linux/types.h>
#include <mach/upmu_sw.h>
#include "cust_alsps.h"
#include "alsps.h"
#include <mt-plat/upmu_common.h>


static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps = 0,		          /* not work, define in epl8882.c */
	.polling_mode_als = 1,		          /* not work, define in epl8882.c */
	.power_id   = MT65XX_POWER_NONE,      /* LDO is not used */
	.power_vol  = VOL_DEFAULT,            /* LDO is not used */
	.i2c_addr   = {0x92, 0x48, 0x78, 0x00},
	.als_level	= {20, 45, 70, 90, 150, 300, 500, 700, 1150, 2250, 4500, 8000, 15000, 30000, 50000},
	.als_value	= {10, 30, 60, 80, 100, 200, 400, 600, 800, 1500, 3000, 6000, 10000, 20000, 40000, 60000},
	.ps_threshold_low = 500,
	.ps_threshold_high = 800,
	.als_threshold_low = 1000,
	.als_threshold_high = 1500,
};
struct alsps_hw *epl259x_get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

