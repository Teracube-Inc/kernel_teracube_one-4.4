#ifndef _LINUX_FOCALTECH_IC_TYPE_H_
#define _LINUX_FOCALTECH_IC_TYPE_H_

/**************************************************/
/****** G: A, I: B, S: C, U: D  ******************/
/****** chip type defines, do not modify *********/
#define _FT8716             0x87160805
#define _FT8736             0x87360806
#define _FT8006M            0x80060807
#define _FT8201             0x82010807
#define _FT7250             0x72500807
#define _FT8606             0x86060808
#define _FT8607             0x86070809
#define _FTE716             0xE716080A
#define _FT8006U            0x8006D80B
#define _FT8613             0x8613080C
#define _FT8719             0x8719080D

#define _FT5416             0x54160402
#define _FT5426             0x54260402
#define _FT5435             0x54350402
#define _FT5436             0x54360402
#define _FT5526             0x55260402
#define _FT5526I            0x5526B402
#define _FT5446             0x54460402
#define _FT5346             0x53460402
#define _FT5446I            0x5446B402
#define _FT5346I            0x5346B402
#define _FT7661             0x76610402
#define _FT7511             0x75110402
#define _FT7421             0x74210402
#define _FT7681             0x76810402
#define _FT3C47U            0x3C47D402
#define _FT3417             0x34170402
#define _FT3517             0x35170402
#define _FT3327             0x33270402
#define _FT3427             0x34270402
#define _FT7311             0x73110402

#define _FT5626             0x56260401
#define _FT5726             0x57260401
#define _FT5826B            0x5826B401
#define _FT5826S            0x5826C401
#define _FT7811             0x78110401
#define _FT3D47             0x3D470401
#define _FT3617             0x36170401
#define _FT3717             0x37170401
#define _FT3817B            0x3817B401
#define _FT3517U            0x3517D401

#define _FT6236U            0x6236D003
#define _FT6336G            0x6336A003
#define _FT6336U            0x6336D003
#define _FT6436U            0x6436D003

#define _FT3267             0x32670004
#define _FT3367             0x33670004
#define _FT5422U            0x5422D482
#define _FT3327DQQ_001      0x3327D482

/*************************************************/

/*
 * choose your ic chip type of focaltech
 */
#if defined(CONFIG_FTS_CHIP_TYPE_FT8716)
#define FTS_CHIP_TYPE   _FT8716
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8736)
#define FTS_CHIP_TYPE   _FT8736
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8006M)
#define FTS_CHIP_TYPE   _FT8006M
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8201)
#define FTS_CHIP_TYPE   _FT8201
#elif defined(CONFIG_FTS_CHIP_TYPE_FT7250)
#define FTS_CHIP_TYPE   _FT7250
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8606)
#define FTS_CHIP_TYPE   _FT8606
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8607)
#define FTS_CHIP_TYPE   _FT8607
#elif defined(CONFIG_FTS_CHIP_TYPE_FTE716)
#define FTS_CHIP_TYPE   _FTE716
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8006U)
#define FTS_CHIP_TYPE   _FT8006U
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8613)
#define FTS_CHIP_TYPE   _FT8613
#elif defined(CONFIG_FTS_CHIP_TYPE_FT8719)
#define FTS_CHIP_TYPE   _FT8719
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5416)
#define FTS_CHIP_TYPE   _FT5416
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5426)
#define FTS_CHIP_TYPE   _FT5426
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5435)
#define FTS_CHIP_TYPE   _FT5435
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5436)
#define FTS_CHIP_TYPE   _FT5436
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5526)
#define FTS_CHIP_TYPE   _FT5526
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5526I)
#define FTS_CHIP_TYPE   _FT5526I
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5446)
#define FTS_CHIP_TYPE   _FT5446
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5346)
#define FTS_CHIP_TYPE   _FT5346
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5446I)
#define FTS_CHIP_TYPE   _FT5446I
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5346I)
#define FTS_CHIP_TYPE   _FT5346I
#elif defined(CONFIG_FTS_CHIP_TYPE_FT7661)
#define FTS_CHIP_TYPE   _FT7661
#elif defined(CONFIG_FTS_CHIP_TYPE_FT7511)
#define FTS_CHIP_TYPE   _FT7511
#elif defined(CONFIG_FTS_CHIP_TYPE_FT7421)
#define FTS_CHIP_TYPE   _FT7421
#elif defined(CONFIG_FTS_CHIP_TYPE_FT7681)
#define FTS_CHIP_TYPE   _FT7681
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3C47U)
#define FTS_CHIP_TYPE   _FT3C47U
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3417)
#define FTS_CHIP_TYPE   _FT3417
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3517)
#define FTS_CHIP_TYPE   _FT3517
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3327)
#define FTS_CHIP_TYPE   _FT3327
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3427)
#define FTS_CHIP_TYPE   _FT3427
#elif defined(CONFIG_FTS_CHIP_TYPE_FT7311)
#define FTS_CHIP_TYPE   _FT7311
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5626)
#define FTS_CHIP_TYPE   _FT5626
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5726)
#define FTS_CHIP_TYPE   _FT5726
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5826B)
#define FTS_CHIP_TYPE   _FT5826B
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5826S)
#define FTS_CHIP_TYPE   _FT5826S
#elif defined(CONFIG_FTS_CHIP_TYPE_FT7811)
#define FTS_CHIP_TYPE   _FT7811
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3D47)
#define FTS_CHIP_TYPE   _FT3D47
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3617)
#define FTS_CHIP_TYPE   _FT3617
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3717)
#define FTS_CHIP_TYPE   _FT3717
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3817B)
#define FTS_CHIP_TYPE   _FT3817B
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3517U)
#define FTS_CHIP_TYPE   _FT3517U
#elif defined(CONFIG_FTS_CHIP_TYPE_FT6236U)
#define FTS_CHIP_TYPE   _FT6236U
#elif defined(CONFIG_FTS_CHIP_TYPE_FT6336G)
#define FTS_CHIP_TYPE   _FT6336G
#elif defined(CONFIG_FTS_CHIP_TYPE_FT6336U)
#define FTS_CHIP_TYPE   _FT6336U
#elif defined(CONFIG_FTS_CHIP_TYPE_FT6436U)
#define FTS_CHIP_TYPE   _FT6436U
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3267)
#define FTS_CHIP_TYPE   _FT3267
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3367)
#define FTS_CHIP_TYPE   _FT3367
#elif defined(CONFIG_FTS_CHIP_TYPE_FT5422U)
#define FTS_CHIP_TYPE   _FT5422U
#elif defined(CONFIG_FTS_CHIP_TYPE_FT3327DQQ_001)
#define FTS_CHIP_TYPE   _FT3327DQQ_001
#else
#define FTS_CHIP_TYPE   _FT3427
#endif

#endif
