/* drivers/hwmon/mt6516/amit/epl259x.c - EPL259x ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <cust_alsps.h>
#include <linux/input/mt.h>

#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <alsps.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/sched.h>
#include "epl259x.h"
/******************************************************************************
 * driver info
*******************************************************************************/
#define EPL_DEV_NAME   		    "EPL259x"
#define DRIVER_VERSION          "3.0.8_ANDR_O"

#define ANDR_O  1
/******************************************************************************
 * ALS / PS sensor structure
*******************************************************************************/
#define COMPATIABLE_NAME "mediatek,epl259x"
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
static long long int_top_time;
/******************************************************************************
 *  ALS / PS define
 ******************************************************************************/
#define PS_DYN_K        1   // PS Auto K
#define PS_FIRST_REPORT 1   // PS First report ps status
#define ALS_DYN_INTT    0   // ALS Auto INTT
#define REG_MONITOR     1

#define ALSPS_DBG       1

#define FACTORY_PATCH   1

#define LUX_PER_COUNT	300  //ALS lux per count

static int als_rs_value[] = {1, 2, 4, 8, 16, 32, 64, 128};
int rs_num = sizeof(als_rs_value)/sizeof(int);
uint16_t als_intr_thd_percent;
int als_frame_time = 0;
int ps_frame_time = 0;
/******************************************************************************
 *  factory setting
 ******************************************************************************/
static const char ps_cal_file[]="/data/data/com.eminent.ps.calibration/ps.dat";  //ps calibration file path
static const char als_cal_file[]="/data/data/com.eminent.ps.calibration/als.dat";  //als calibration file path

static int PS_h_offset = 3000;  //factory high threshold offset
static int PS_l_offset = 2000;  //factory low threshold offset
static int PS_MAX_XTALK = 30000;  //factory max crosstalk, if real crosstalk > max crosstalk, return fail

/******************************************************************************
 *I2C function define
*******************************************************************************/
#define TXBYTES 				2
#define PACKAGE_SIZE 			48
#define I2C_RETRY_COUNT 		2
int i2c_max_count=8;
static const struct i2c_device_id epl_sensor_i2c_id[] = {{EPL_DEV_NAME,0},{}};
/******************************************************************************
 * extern functions
*******************************************************************************/
#if !ANDR_O
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
struct hwmsen_object *ps_hw, * als_hw;
static struct epl_sensor_priv *epl_sensor_obj = NULL;
static struct wake_lock ps_lock;
static struct mutex sensor_mutex;
static epl_optical_sensor epl_sensor;
static struct i2c_client *epl_sensor_i2c_client = NULL;

typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
} epl_raw_data;
static epl_raw_data	gRawData;

#if ALSPS_DBG
bool debug_flag = false;
#endif

#define APS_TAG                 	  	"[ALS/PS] "

#if ALSPS_DBG
#define APS_FUN(f)               	 if(debug_flag)\
                                        printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_LOG(fmt, args...)    	 if(debug_flag)\
                                        printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_ERR(fmt, args...)   	 printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define APS_FUN(f)              	  	printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk(KERN_INFO APS_TAG fmt, ##args)
#endif
#define APS_DBG(fmt, args...)    	    printk(KERN_INFO fmt, ##args)

static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl_sensor_i2c_remove(struct i2c_client *client);
#if ANDR_O
static int epl_sensor_i2c_suspend(struct device *dev);
static int epl_sensor_i2c_resume(struct device *dev);
#else
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl_sensor_i2c_resume(struct i2c_client *client);
#endif
static irqreturn_t epl_sensor_eint_func(int irq, void *desc);
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
void epl_sensor_update_mode(struct i2c_client *client);
void epl_sensor_fast_update(struct i2c_client *client);
int epl_sensor_read_ps(struct i2c_client *client);
static int ps_sensing_time(int intt, int osr, int cycle);
static int als_sensing_time(int intt, int osr, int cycle);
int factory_ps_data(void);
int factory_als_data(void);

#if PS_DYN_K
void epl_sensor_restart_dynk_polling(void);
#endif

typedef enum
{
    CMC_BIT_ALS   	= 1,
    CMC_BIT_PS     	= 2,
} CMC_BIT;

typedef enum
{
    CMC_BIT_RAW   			= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
    CMC_BIT_TABLE			= 0x3,
} CMC_ALS_REPORT_TYPE;

struct epl_sensor_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
};

struct epl_sensor_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
#if PS_DYN_K
    struct delayed_work  dynk_thd_polling_work;
#elif REG_MONITOR
    struct delayed_work  reg_monitor_polling_work;
#endif
    struct input_dev *gs_input_dev;
    /*i2c address group*/
    struct epl_sensor_i2c_addr  addr;
    /*misc*/
    atomic_t   	als_suspend;
    atomic_t    ps_suspend;
    /*data*/
    u16		    lux_per_count;
    ulong       enable;         	/*record HAL enalbe status*/
    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
    struct device_node *irq_node;
    int		irq;
#if FACTORY_PATCH
    int ps_cali;
#endif
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif
#if ANDR_O
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops epl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(epl_sensor_i2c_suspend, epl_sensor_i2c_resume)
};
#endif
#endif /*ANDR_O*/
static struct i2c_driver epl_sensor_i2c_driver =
{
    .probe     	= epl_sensor_i2c_probe,
    .remove     = epl_sensor_i2c_remove,
#if !ANDR_O
    .suspend    = epl_sensor_i2c_suspend,
    .resume     = epl_sensor_i2c_resume,
#endif
    .id_table   = epl_sensor_i2c_id,
    //.address_data = &epl_sensor_addr_data,
    .driver = {
        //.owner  = THIS_MODULE,
        .name   = EPL_DEV_NAME,
#if ANDR_O
#ifdef CONFIG_PM_SLEEP
		.pm   = &epl_pm_ops,
#endif
#endif /*ANDR_O*/
#ifdef CONFIG_OF
	    .of_match_table = alsps_of_match,
#endif
    },
};

static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail
static int alsps_local_init(void);
static int alsps_remove(void);
static struct alsps_init_info epl_sensor_init_info = {
		.name = EPL_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,
};

extern struct alsps_context *alsps_context_obj;

/******************************************************************************
 *  PS_DYN_K
 ******************************************************************************/
#if PS_DYN_K
static int dynk_polling_delay = 200;
int dynk_min_ps_raw_data = 0xffff;
int dynk_max_ir_data;
u32 dynk_thd_low = 0;
u32 dynk_thd_high = 0;
int dynk_low_offset;
int dynk_high_offset;
#endif  /*PS_DYN_K*/

/******************************************************************************
 *  PS First report
 ******************************************************************************/
#if PS_FIRST_REPORT
static bool ps_first_flag = true;
#define PS_MAX_CT  10000
#endif /*PS_FIRST_REPORT*/

/******************************************************************************
 *  ALS_DYN_INTT
 ******************************************************************************/
#if ALS_DYN_INTT
//Dynamic INTT
int dynamic_intt_idx;
int dynamic_intt_init_idx = 0;	//initial dynamic_intt_idx
int c_gain;
int dynamic_intt_lux = 0;
uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint32_t dynamic_intt_max_lux = 30000;
uint32_t dynamic_intt_min_lux = 0;
uint32_t dynamic_intt_min_unit = 1000;
static int als_dynamic_intt_intt[] = {EPL_ALS_INTT_1024, EPL_ALS_INTT_1024};
static int als_dynamic_intt_value[] = {1024, 1024}; //match als_dynamic_intt_intt table
static int als_dynamic_intt_gain[] = {EPL_GAIN_MID, EPL_GAIN_LOW};
static int als_dynamic_intt_high_thr[] = {60000, 65535};
static int als_dynamic_intt_low_thr[] = {0, 200};
static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_value)/sizeof(int);
static int als_dynamic_intt_rs[] = {EPL_RS_8, EPL_RS_0};
bool als_dyn_intt_over_flag = false;
#endif /*ALS_DYN_INTT*/
/******************************************************************************
 *  REG_MONITOR
 ******************************************************************************/
#if (REG_MONITOR && !PS_DYN_K)
static int reg_monitor_polling_delay = 200;
#endif/*REG_MONITOR*/

static int epl_sensor_I2C_Write_Block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];
    err =0;

    if (!client)
    {
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE)
    {
        APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }
    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }
    err = i2c_master_send(client, buf, num);
    if (err < 0)
    {
        APS_ERR("send command error!!\n");

        return -EFAULT;
    }

    return err;
}

static int epl_sensor_I2C_Write_Cmd(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = regaddr;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n",ret);
    }
    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}

static int epl_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = epl_sensor_I2C_Write_Cmd(client, regaddr, data, 0x02);
    return ret;
}

static int epl_sensor_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount)
{
    int ret = 0;
    int retry;
    int read_count=0, rx_count=0;
    while(bytecount>0)
    {
        epl_sensor_I2C_Write_Cmd(client, regaddr+read_count, 0x00, 0x01);
        for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
        {
            rx_count = bytecount>i2c_max_count?i2c_max_count:bytecount;
            ret = i2c_master_recv(client, &gRawData.raw_bytes[read_count], rx_count);

            if (ret == rx_count)
                break;

            APS_ERR("i2c read error %d\r\n",ret);
        }
        if(retry>=I2C_RETRY_COUNT)
        {
            APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
            return -EINVAL;
        }
        bytecount-=rx_count;
        read_count+=rx_count;
    }

    return ret;
}

static void set_als_ps_intr_type(struct i2c_client *client, bool ps_polling, bool als_polling)
{
    //set als / ps interrupt control mode and interrupt type
	switch((ps_polling << 1) | als_polling)
	{
		case 0: // ps and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;
		case 1: //ps interrupt and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;
		case 2: // ps polling and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;
		case 3: //ps and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;
	}
}

static void write_global_variable(struct i2c_client *client)
{
    u8 buf_block[7];

    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_ON | EPL_RESETN_RUN);
    epl_sensor_I2C_Write(client, DEVREG_PS_STATUS, (EPL_CMP_RESET | EPL_UN_LOCK));
    epl_sensor_I2C_Write(client, DEVREG_PS_STATUS, (EPL_CMP_RUN | EPL_UN_LOCK));
    epl_sensor_I2C_Write(client, DEVREG_ALS_STATUS, (EPL_CMP_RESET | EPL_UN_LOCK));
    epl_sensor_I2C_Write(client, DEVREG_ALS_STATUS, (EPL_CMP_RUN | EPL_UN_LOCK));
    epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_OFF | EPL_RESETN_RESET);

    /*chip refrash*/
    epl_sensor_I2C_Write(client, 0xfd, 0x8e);
    if(epl_sensor.revno == 0xa188)
    {
        epl_sensor_I2C_Write(client, 0xfe, 0xa2);
        epl_sensor_I2C_Write(client, 0xfe, 0x82);
    }
    else if(epl_sensor.revno == 0x0288)
    {
        epl_sensor_I2C_Write(client, 0xfb, 0x21);
        epl_sensor_I2C_Write(client, 0xfb, 0x20);
    }
    else
    {
        epl_sensor_I2C_Write(client, 0xfe, 0x22);
        epl_sensor_I2C_Write(client, 0xfe, 0x02);
    }
    epl_sensor_I2C_Write(client, 0xfd, 0x00);

    if(epl_sensor.revno == 0x0288)
        epl_sensor_I2C_Write(client, 0xfc, EPL_INTERNAL | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);
    else
        epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);

    mutex_unlock(&sensor_mutex);

    set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
    set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);

    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(client, DEVREG_PS_OFSL, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(client, DEVREG_PS_OFSH, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

    //set als / ps interrupt control mode and trigger type
    set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);

    if(epl_sensor.revno != 0x8188)
    {
        if(epl_sensor.revno == 0x0288)
        {

            buf_block[0] = epl_sensor.als.als_std | epl_sensor.als.integration_time | epl_sensor.als.gain;//REG0x01
            buf_block[1] = epl_sensor.als.als_rs | epl_sensor.als.osr | epl_sensor.als.cycle;
            buf_block[2] = epl_sensor.ps.ps_std | epl_sensor.ps.integration_time | epl_sensor.ps.gain;
            buf_block[3] = epl_sensor.ps.ps_rs | epl_sensor.ps.osr | epl_sensor.ps.cycle;
            buf_block[4] = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
            buf_block[5] = epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type;
            buf_block[6] = epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;
        }
        else
        {
            buf_block[0] = epl_sensor.als.als_intb_nonlos | epl_sensor.als.integration_time | epl_sensor.als.gain; //REG0x01
            buf_block[1] = epl_sensor.als.als_rs | epl_sensor.als.osr | epl_sensor.als.cycle;
            buf_block[2] = epl_sensor.ps.ps_intb_nonlos | epl_sensor.ps.integration_time | epl_sensor.ps.gain;
            buf_block[3] = epl_sensor.ps.ps_rs | epl_sensor.ps.osr | epl_sensor.ps.cycle;
            buf_block[4] = epl_sensor.ps.ps_std | epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
            buf_block[5] = epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type;
            buf_block[6] = epl_sensor.als.als_std | epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;
        }
    }
    else
    {
        buf_block[0] = epl_sensor.als.integration_time | epl_sensor.als.gain; //REG0x01
        buf_block[1] = epl_sensor.als.osr | epl_sensor.als.cycle;
        buf_block[2] = epl_sensor.ps.integration_time | epl_sensor.ps.gain;
        buf_block[3] = epl_sensor.ps.osr | epl_sensor.ps.cycle;
        buf_block[4] = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
        buf_block[5] = epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type;
        buf_block[6] = epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;

    }
    epl_sensor_I2C_Write_Block(client, DEVREG_ALS_CONFIG, buf_block, 7);
#if !ALS_DYN_INTT
    if(epl_sensor.revno == 0x0288 && epl_sensor.als.gain == EPL_AG_EN)
    {
        buf_block[0] = epl_sensor.als.als_ag_l | epl_sensor.als.als_aintt_l;//REG0x24
        buf_block[1] = epl_sensor.als.als_ag_h | epl_sensor.als.als_aintt_h;
        buf_block[2] = (u8)(epl_sensor.als.als_ag_thd & 0xff);
        buf_block[3] = (u8)((epl_sensor.als.als_ag_thd & 0xff00) >> 8);
        epl_sensor_I2C_Write_Block(client, 0x24, buf_block, 4);
    }
#endif


    //set mode and wait
    if(epl_sensor.revno == 0x0288)
        epl_sensor_I2C_Write(client, DEVREG_ENABLE, (epl_sensor.wait | epl_sensor.als_single_pixel | epl_sensor.mode));
    else
        epl_sensor_I2C_Write(client, DEVREG_ENABLE, (epl_sensor.wait | epl_sensor.mode));

    mutex_unlock(&sensor_mutex);
}

static int write_factory_calibration(struct epl_sensor_priv *epl_data, char* ps_data, int ps_cal_len)
{
    struct file *fp_cal;
	mm_segment_t fs;
	loff_t pos;

	APS_FUN();
    pos = 0;
	fp_cal = filp_open(ps_cal_file, O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
	if (IS_ERR(fp_cal))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}
    fs = get_fs();
	set_fs(KERNEL_DS);
	vfs_write(fp_cal, ps_data, ps_cal_len, &pos);
    filp_close(fp_cal, NULL);
	set_fs(fs);

	return 0;
}

static bool read_factory_calibration(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
    char buffer[100]= {0};

    if(epl_sensor.ps.factory.calibration_enable && !epl_sensor.ps.factory.calibrated)
    {
		fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.ps.factory.calibration_enable =  false;
        }
        else
        {
            int ps_cancelation = 0, ps_hthr = 0, ps_lthr = 0;
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%d,%d,%d", &ps_cancelation, &ps_hthr, &ps_lthr);
			epl_sensor.ps.factory.cancelation = ps_cancelation;
			epl_sensor.ps.factory.high_threshold = ps_hthr;
			epl_sensor.ps.factory.low_threshold = ps_lthr;
            set_fs(fs);

            epl_sensor.ps.high_threshold = epl_sensor.ps.factory.high_threshold;
            epl_sensor.ps.low_threshold = epl_sensor.ps.factory.low_threshold;
            epl_sensor.ps.cancelation = epl_sensor.ps.factory.cancelation;
        }
        mutex_lock(&sensor_mutex);
        epl_sensor_I2C_Write(obj->client,DEVREG_PS_OFSL, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(obj->client,DEVREG_PS_OFSH, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        mutex_unlock(&sensor_mutex);
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        epl_sensor.ps.factory.calibrated = true;
    }

    if(epl_sensor.als.factory.calibration_enable && !epl_sensor.als.factory.calibrated)
    {
        fp = filp_open(als_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.als.factory.calibration_enable =  false;
        }
        else
        {
            int als_lux_per_count = 0;
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);
            sscanf(buffer, "%d", &als_lux_per_count);
			epl_sensor.als.factory.lux_per_count = als_lux_per_count;
            set_fs(fs);
        }
        epl_sensor.als.factory.calibrated = true;
    }

    return true;
}

static int epl_run_ps_calibration(struct epl_sensor_priv *epl_data)
{
    struct epl_sensor_priv *epld = epl_data;
    u16 ch1=0;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0, ps_cal_len = 0;
    char ps_calibration[20];

    if(PS_MAX_XTALK < 0)
    {
        APS_ERR("[%s]:Failed: PS_MAX_XTALK < 0 \r\n", __func__);
        return -EINVAL;
    }
	switch(epl_sensor.mode)
	{
		case EPL_MODE_PS:
		case EPL_MODE_ALS_PS:
            ch1 = factory_ps_data();
        break;
	}
    if(ch1 > PS_MAX_XTALK)
    {
        APS_ERR("[%s]:Failed: ch1 > max_xtalk(%d) \r\n", __func__, ch1);
        return -EINVAL;
    }
    else if(ch1 < 0)
    {
        APS_ERR("[%s]:Failed: ch1 < 0\r\n", __func__);
        return -EINVAL;
    }
    ps_hthr = ch1 + PS_h_offset;
    ps_lthr = ch1 + PS_l_offset;
    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d", ps_cancelation, ps_hthr, ps_lthr);

    if(write_factory_calibration(epld, ps_calibration, ps_cal_len) < 0)
    {
        APS_ERR("[%s] create file error \n", __func__);
        return -EINVAL;
    }
    epl_sensor.ps.low_threshold = ps_lthr;
	epl_sensor.ps.high_threshold = ps_hthr;
	set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
	APS_LOG("[%s]: ch1 = %d\n", __func__, ch1);

	return ch1;
}

static void initial_global_variable(struct i2c_client *client, struct epl_sensor_priv *obj)
{
#if ALS_DYN_INTT
    int idx=0, i=0, gain_value=0, intt_value=0, total_value=0;
#endif
    /* read revno*/
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Read(client, DEVREG_REV_ID, 2);
    epl_sensor.revno = gRawData.raw_bytes[0] | gRawData.raw_bytes[1] << 8;
    mutex_unlock(&sensor_mutex);

    //general setting
    epl_sensor.power = EPL_POWER_ON;
    epl_sensor.reset = EPL_RESETN_RUN;
    epl_sensor.als_single_pixel = ALS_SIGLE_DIS;
    epl_sensor.mode = EPL_MODE_IDLE;
    epl_sensor.wait = EPL_WAIT_0_MS;
    epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;

    //als setting
    epl_sensor.als.polling_mode = obj->hw->polling_mode_als;
    epl_sensor.als.integration_time = EPL_ALS_INTT_1024;
    epl_sensor.als.gain = EPL_GAIN_LOW;
    epl_sensor.als.osr = EPL_PSALS_OSR_11;
    epl_sensor.als.cycle = EPL_CYCLE_16;
    epl_sensor.als.als_rs = EPL_RS_0;
    epl_sensor.als.als_intb_nonlos = EPL_T_DIS;
    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
    epl_sensor.als.persist = EPL_PERIST_1;
    if(epl_sensor.revno == 0x0288)
        epl_sensor.als.als_std = (EPL_ALS_PRE << 1);
    else
        epl_sensor.als.als_std = EPL_ALS_PRE;
    epl_sensor.als.compare_reset = EPL_CMP_RUN;
    epl_sensor.als.lock = EPL_UN_LOCK;
    epl_sensor.als.report_type = CMC_BIT_RAW; //CMC_BIT_RAW; //CMC_BIT_DYN_INT; //CMC_BIT_PRE_COUNT;
    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;
    //als factory
    epl_sensor.als.factory.calibration_enable =  false;
    epl_sensor.als.factory.calibrated = false;
    epl_sensor.als.factory.lux_per_count = LUX_PER_COUNT;
    als_intr_thd_percent = 20; // +/-20%
#if ALS_DYN_INTT
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        dynamic_intt_idx = dynamic_intt_init_idx;
        epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
        epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
    }
    c_gain = epl_sensor.als.factory.lux_per_count; // 300/1000=0.3 /*Lux per count*/

    if( epl_sensor.revno != 0x8188 && epl_sensor.als.report_type == CMC_BIT_DYN_INT )  //3638
    {
        for(i = 0; i < (als_dynamic_intt_intt_num-1); i++)
        {
            if( epl_sensor.revno == 0x0288 && (als_dynamic_intt_gain[i] == EPL_GAIN_HIGH) )
            {
                als_dynamic_intt_gain[i] = EPL_GAIN_MID;
                als_dynamic_intt_intt[i] = ((als_dynamic_intt_intt[i]>>2)+1) << 2;
                als_dynamic_intt_value[i] = als_intt_value[als_dynamic_intt_intt[i]>>2];

                APS_LOG("[%s]: this chip dont support EPL_GAIN_HIGH, force to change with GAIN and INTEG ..................... \r\n", __func__);
            }

            if(als_dynamic_intt_gain[i] == EPL_GAIN_HIGH)
                gain_value = 64;
            else if(als_dynamic_intt_gain[i] == EPL_GAIN_MID)
                gain_value = 8;
            else
                gain_value = 1;

            intt_value = als_dynamic_intt_value[i] / als_dynamic_intt_value[(als_dynamic_intt_intt_num-1)];
            total_value = gain_value * intt_value;

            if(total_value > als_rs_value[rs_num-1])
            {
                APS_LOG("[%s]: total_value=%d more than max RS \r\n", __func__, total_value);
                als_dyn_intt_over_flag = true;
                break;
            }
            else
            {
                for(idx = 0; idx < rs_num;  idx++)
            	{
            	    if(total_value < als_rs_value[idx])
            	    {
            	        break;
            	    }
            	}
            	APS_LOG("[%s]: idx=%d, als_rs_value=%d, total_value=%d\r\n", __func__, idx, als_rs_value[idx-1], total_value);

            	als_dynamic_intt_rs[i] = ((idx-1)<<5);
                als_dynamic_intt_high_thr[i] = als_dynamic_intt_high_thr[i]/total_value;
                als_dynamic_intt_low_thr[i] = als_dynamic_intt_low_thr[i]/total_value;
                APS_LOG("[%s]: als_dynamic_intt_low_thr[%d]=%d, als_dynamic_intt_high_thr[%d]=%d\r\n", __func__, i, als_dynamic_intt_low_thr[i], i, als_dynamic_intt_high_thr[i]);
            }

        }
    }
#else
    //als auto gain, dont support
    if(epl_sensor.als.gain == EPL_AG_EN)
    {
        epl_sensor.als.als_ag_l = EPL_AG_L;
        epl_sensor.als.als_ag_h = EPL_AG_M;
        epl_sensor.als.als_aintt_l = (EPL_ALS_INTT_1024>>2);
        epl_sensor.als.als_aintt_h = (EPL_ALS_INTT_1024>>2);
        epl_sensor.als.als_ag_thd = 7500;
    }
#endif
    //ps setting
    epl_sensor.ps.polling_mode = obj->hw->polling_mode_ps;
    epl_sensor.ps.integration_time = EPL_PS_INTT_272;
    epl_sensor.ps.gain = EPL_GAIN_LOW;
    epl_sensor.ps.osr = EPL_PSALS_OSR_11;
    epl_sensor.ps.cycle = EPL_CYCLE_16;
    epl_sensor.ps.ps_rs = EPL_RS_0;
    epl_sensor.ps.ps_intb_nonlos = EPL_T_DIS;
    epl_sensor.ps.persist = EPL_PERIST_1;
    epl_sensor.ps.ps_std = EPL_PS_PRE;
    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.ps.ir_drive = EPL_IR_DRIVE_100;
    epl_sensor.ps.compare_reset = EPL_CMP_RUN;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor.ps.high_threshold = obj->hw->ps_threshold_high;
    epl_sensor.ps.low_threshold = obj->hw->ps_threshold_low;
    //ps factory
    epl_sensor.ps.factory.calibration_enable =  false;
    epl_sensor.ps.factory.calibrated = false;
    epl_sensor.ps.factory.cancelation= 0;
#if PS_DYN_K
    dynk_max_ir_data = 50000; // ps max ch0
    dynk_low_offset = 500;
    dynk_high_offset = 800;
#endif /*PS_DYN_K*/

    //write setting to sensor
    write_global_variable(client);
}

#if ALS_DYN_INTT
long raw_convert_to_lux(u16 raw_data)
{
    long lux = 0;
    long dyn_intt_raw = 0;
    int gain_value = 0;

    if( (epl_sensor.revno != 0x8188) && (als_dyn_intt_over_flag == false) )
    {
        dyn_intt_raw = raw_data;
    }
    else
    {
        if(epl_sensor.als.gain == EPL_GAIN_HIGH)
        {
            gain_value = 64;
        }
        else if(epl_sensor.als.gain == EPL_GAIN_MID)
        {
            gain_value = 8;
        }
        else if (epl_sensor.als.gain == EPL_GAIN_LOW)
        {
            gain_value = 1;
        }
        dyn_intt_raw = (raw_data * 10) / (10 * gain_value * als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]); //float calculate
    }

    APS_LOG("[%s]: dyn_intt_raw=%ld \r\n", __func__, dyn_intt_raw);

    if(dyn_intt_raw > 0xffff)
        epl_sensor.als.dyn_intt_raw = 0xffff;
    else
        epl_sensor.als.dyn_intt_raw = dyn_intt_raw;

    lux = c_gain * epl_sensor.als.dyn_intt_raw;
    APS_LOG("[%s]:raw_data=%d, epl_sensor.als.dyn_intt_raw=%d, lux=%ld\r\n", __func__, raw_data, epl_sensor.als.dyn_intt_raw, lux);

    if(lux >= (dynamic_intt_max_lux*dynamic_intt_min_unit)){
        APS_LOG("[%s]:raw_convert_to_lux: change max lux\r\n", __func__);
        lux = dynamic_intt_max_lux * dynamic_intt_min_unit;
    }
    else if(lux <= (dynamic_intt_min_lux*dynamic_intt_min_unit)){
        APS_LOG("[%s]:raw_convert_to_lux: change min lux\r\n", __func__);
        lux = dynamic_intt_min_lux * dynamic_intt_min_unit;
    }

    return lux;
}
#endif

static int epl_sensor_get_als_value(struct epl_sensor_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
#if ALS_DYN_INTT
	long now_lux=0, lux_tmp=0;
    bool change_flag = false;
#endif

    switch(epl_sensor.als.report_type)
    {
        case CMC_BIT_RAW:
            return als;
        break;
        case CMC_BIT_PRE_COUNT:
            return (als * epl_sensor.als.factory.lux_per_count)/1000;
        break;
        case CMC_BIT_TABLE:
            for(idx = 0; idx < obj->als_level_num; idx++)
            {
                if(als < obj->hw->als_level[idx])
                {
                    break;
                }
            }

            if(idx >= obj->als_value_num)
            {
                APS_ERR("exceed range\n");
                idx = obj->als_value_num - 1;
            }

            if(!invalid)
            {
                APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
                return obj->hw->als_value[idx];
            }
            else
            {
                APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
                return als;
            }
        break;
#if ALS_DYN_INTT
		case CMC_BIT_DYN_INT:

            APS_LOG("[%s]: dynamic_intt_idx=%d, als_dynamic_intt_value=%d, dynamic_intt_gain=%d, als=%d \r\n",
                                    __func__, dynamic_intt_idx, als_dynamic_intt_value[dynamic_intt_idx], als_dynamic_intt_gain[dynamic_intt_idx], als);

            if( (als > dynamic_intt_high_thr) || (dynamic_intt_idx!=(als_dynamic_intt_intt_num-1) && epl_sensor.als.saturation==EPL_SATURATION) )
        	{
          		if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
                    //als = dynamic_intt_high_thr;
          		    lux_tmp = raw_convert_to_lux(als);
        	      	APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MAX_LUX\r\n");
          		}
                else{
                    change_flag = true;
        			als  = dynamic_intt_high_thr;
              		lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx++;
                    if(dynamic_intt_idx >= (als_dynamic_intt_intt_num - 1))
                        dynamic_intt_idx = (als_dynamic_intt_intt_num - 1);
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \r\n", dynamic_intt_idx, als);
                }
            }
            else if(als < dynamic_intt_low_thr)
            {
                if(dynamic_intt_idx == 0){
                    //als = dynamic_intt_low_thr;
                    lux_tmp = raw_convert_to_lux(als);
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MIN_LUX\r\n");
                }
                else{
                    change_flag = true;
        			als  = dynamic_intt_low_thr;
                	lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx--;
                    if(dynamic_intt_idx <= 0)
                        dynamic_intt_idx = 0;
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \r\n", dynamic_intt_idx, als);
                }
            }
            else
            {
            	lux_tmp = raw_convert_to_lux(als);
            }

            now_lux = lux_tmp;
            dynamic_intt_lux = now_lux/dynamic_intt_min_unit;

            if(change_flag == true)
            {
                APS_LOG("[%s]: ALS_DYN_INTT:Chang Setting \r\n", __func__);
                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                epl_sensor.als.als_rs = als_dynamic_intt_rs[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                epl_sensor_fast_update(obj->client);

                mutex_lock(&sensor_mutex);
                if(epl_sensor.revno == 0x0288)
                    epl_sensor_I2C_Write(obj->client, DEVREG_ENABLE, epl_sensor.wait | epl_sensor.als_single_pixel | epl_sensor.mode);
                else
                    epl_sensor_I2C_Write(obj->client, DEVREG_ENABLE, epl_sensor.wait | epl_sensor.mode);
                epl_sensor_I2C_Write(obj->client, DEVREG_RESET, EPL_POWER_ON | EPL_RESETN_RUN);
                mutex_unlock(&sensor_mutex);
            }

            return dynamic_intt_lux;
		break;
#endif
    }

    return 0;
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;
    u8 buf[4];

    buf[3] = high_msb = (uint8_t) (high_thd >> 8);
    buf[2] = high_lsb = (uint8_t) (high_thd & 0x00ff);
    buf[1] = low_msb  = (uint8_t) (low_thd >> 8);
    buf[0] = low_lsb  = (uint8_t) (low_thd & 0x00ff);
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write_Block(client, DEVREG_PS_ILTL, buf, 4);
    mutex_unlock(&sensor_mutex);
    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);
    return 0;
}

static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;
    u8 buf[4];

    buf[3] = high_msb = (uint8_t) (high_thd >> 8);
    buf[2] = high_lsb = (uint8_t) (high_thd & 0x00ff);
    buf[1] = low_msb  = (uint8_t) (low_thd >> 8);
    buf[0] = low_lsb  = (uint8_t) (low_thd & 0x00ff);
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write_Block(client, DEVREG_ALS_ILTL, buf, 4);
    mutex_unlock(&sensor_mutex);
    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    APS_LOG("chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    APS_LOG("chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    APS_LOG("chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    APS_LOG("chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    APS_LOG("chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    APS_LOG("chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    APS_LOG("chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    APS_LOG("chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    APS_LOG("chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    APS_LOG("chip id REG 0x20 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x21 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x21));
    APS_LOG("chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    APS_LOG("chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    APS_LOG("chip id REG 0xfb value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfb));
    APS_LOG("chip id REG 0xfc value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfc));
}

int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");
    epl_sensor_i2c_client = client;
    APS_LOG(" I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr,__LINE__);

    return 0;
}

int epl_sensor_get_addr(struct alsps_hw *hw, struct epl_sensor_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];

    return 0;
}
#if !ANDR_O
static void epl_sensor_power(struct alsps_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;
    //APS_LOG("power %s\n", on ? "on" : "off");
    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, EPL_DEV_NAME))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, EPL_DEV_NAME))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
}
#endif
static void epl_sensor_report_ps_status(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int report_value;
    int err;

    if( (epl_sensor.ps.compare_low >> 3) == 0)
        report_value = PS_NEAR;
    else
        report_value = PS_FAR;

    APS_LOG("[%s]: epl_sensor.ps.data.data=%d, report_value=%d \r\n", __func__, epl_sensor.ps.data.data, report_value);

    err = ps_report_interrupt_data(report_value);
    if(err != 0)  //if report status is fail, write unlock again.
    {
        APS_ERR("epl_sensor_eint_work err: %d\n", err);
    	epl_sensor_I2C_Write(epld->client, DEVREG_PS_STATUS, EPL_CMP_RESET | EPL_UN_LOCK);
    }
}
static void epl_sensor_report_lux(int report_lux)
{
    int err = 0;

#if ALS_DYN_INTT
    APS_LOG("-------------------  ALS raw = %d, lux = %d\n\n", epl_sensor.als.dyn_intt_raw,  report_lux);
#else
    APS_LOG("-------------------  ALS raw = %d, lux = %d\n\n", epl_sensor.als.data.channels[1],  report_lux);
#endif

#if ANDR_O
    if((err = als_data_report(report_lux, SENSOR_STATUS_ACCURACY_MEDIUM)))
#else
    if((err = als_data_report(alsps_context_obj->idev, report_lux, SENSOR_STATUS_ACCURACY_MEDIUM)))
#endif
    {
       APS_ERR("epl_sensor call als_data_report fail = %d\n", err);
    }
}
/*----------------------------------------------------------------------------*/

int epl_sensor_read_als(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    u8 buf[5];

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Read(obj->client, DEVREG_ALS_STATUS, 5);
    buf[0] = gRawData.raw_bytes[0];
    buf[1] = gRawData.raw_bytes[1];
    buf[2] = gRawData.raw_bytes[2];
    buf[3] = gRawData.raw_bytes[3];
    buf[4] = gRawData.raw_bytes[4];
    mutex_unlock(&sensor_mutex);

    epl_sensor.als.saturation = (buf[0] & 0x20);
    epl_sensor.als.compare_high = (buf[0] & 0x10);
    epl_sensor.als.compare_low = (buf[0] & 0x08);
    epl_sensor.als.interrupt_flag = (buf[0] & 0x04);
    epl_sensor.als.compare_reset = (buf[0] & 0x02);
    epl_sensor.als.lock = (buf[0] & 0x01);
    epl_sensor.als.data.channels[0] = (buf[2]<<8) | buf[1];
    epl_sensor.als.data.channels[1] = (buf[4]<<8) | buf[3];

	APS_LOG("als: ~~~~ ALS ~~~~~ \n");
	APS_LOG("als: buf = 0x%x\n", buf[0]);
	APS_LOG("als: sat = 0x%x\n", epl_sensor.als.saturation);
	APS_LOG("als: cmp h = 0x%x, l = %d\n", epl_sensor.als.compare_high, epl_sensor.als.compare_low);
	APS_LOG("als: int_flag = 0x%x\n",epl_sensor.als.interrupt_flag);
	APS_LOG("als: cmp_rstn = 0x%x, lock = 0x%0x\n", epl_sensor.als.compare_reset, epl_sensor.als.lock);
    APS_LOG("read als channel 0 = %d\n", epl_sensor.als.data.channels[0]);
    APS_LOG("read als channel 1 = %d\n", epl_sensor.als.data.channels[1]);

    if(epl_sensor.revno == 0x0288 && epl_sensor.als.gain == EPL_AG_EN)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_I2C_Read(obj->client, 0x28, 3);
        buf[0] = gRawData.raw_bytes[0];
        buf[1] = gRawData.raw_bytes[1];
        buf[2] = gRawData.raw_bytes[2];
        mutex_unlock(&sensor_mutex);
        epl_sensor.als.als_ag_l_value = buf[0];
        epl_sensor.als.als_ag_h_value = buf[1];
        epl_sensor.als.als_ag_flag = (buf[2] & 0x10);
        epl_sensor.als.als_ag_rs_weight = (buf[2] & 0x0f);
        APS_LOG("read als_ag low value = %d\n", epl_sensor.als.als_ag_l_value);
        APS_LOG("read als_ag high value = %d\n", epl_sensor.als.als_ag_h_value);
        APS_LOG("read als_ag flag = 0x%x\n", epl_sensor.als.als_ag_flag);
        APS_LOG("read als_ag rs wegiht = %d\n", epl_sensor.als.als_ag_rs_weight);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_read_ps(struct i2c_client *client)
{
    u8 buf[5];

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Read(client, DEVREG_PS_STATUS, 5);
    buf[0] = gRawData.raw_bytes[0];
    buf[1] = gRawData.raw_bytes[1];
    buf[2] = gRawData.raw_bytes[2];
    buf[3] = gRawData.raw_bytes[3];
    buf[4] = gRawData.raw_bytes[4];
    mutex_unlock(&sensor_mutex);

    if(epl_sensor.revno != 0x8188)
        epl_sensor.ps.saturation_1 = (buf[0] & 0x40);
    epl_sensor.ps.saturation = (buf[0] & 0x20);
    epl_sensor.ps.compare_high = (buf[0] & 0x10);
    epl_sensor.ps.compare_low = (buf[0] & 0x08);
    epl_sensor.ps.interrupt_flag = (buf[0] & 0x04);
    epl_sensor.ps.compare_reset = (buf[0] & 0x02);
    epl_sensor.ps.lock= (buf[0] & 0x01);
    epl_sensor.ps.data.ir_data = (buf[2]<<8) | buf[1];
    epl_sensor.ps.data.data = (buf[4]<<8) | buf[3];

	APS_LOG("ps: ~~~~ PS ~~~~~ \n");
	APS_LOG("ps: buf = 0x%x\n", buf[0]);
	if(epl_sensor.revno != 0x8188)
	    APS_LOG("ps: sat_1 = 0x%x\n", epl_sensor.ps.saturation_1);
	APS_LOG("ps: sat = 0x%x\n", epl_sensor.ps.saturation);
	APS_LOG("ps: cmp h = 0x%x, l = 0x%x\n", epl_sensor.ps.compare_high, epl_sensor.ps.compare_low);
	APS_LOG("ps: int_flag = 0x%x\n",epl_sensor.ps.interrupt_flag);
	APS_LOG("ps: cmp_rstn = 0x%x, lock = %x\n", epl_sensor.ps.compare_reset, epl_sensor.ps.lock);
	APS_LOG("[%s]: data = %d\n", __func__, epl_sensor.ps.data.data);
	APS_LOG("[%s]: ir data = %d\n", __func__, epl_sensor.ps.data.ir_data);
    return 0;
}

int factory_ps_data(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;

#if !PS_DYN_K
    if(enable_ps == 1)
    {
        epl_sensor_read_ps(epld->client);
    }
    else
    {
        APS_LOG("[%s]: ps is disabled \r\n", __func__);
    }
#else
    if(enable_ps == 0)
    {
        APS_LOG("[%s]: ps is disabled \r\n", __func__);
    }
#endif
    APS_LOG("[%s]: enable_ps=%d, ps_raw=%d \r\n", __func__, enable_ps, epl_sensor.ps.data.data);

    return epl_sensor.ps.data.data;
}

int factory_als_data(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    u16 als_raw = 0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;

    if(enable_als == 1)
    {
        epl_sensor_read_als(epld->client);

        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            int als_lux=0;
            als_lux = epl_sensor_get_als_value(epld, epl_sensor.als.data.channels[1]);
        }
    }
    else
    {
        APS_LOG("[%s]: als is disabled \r\n", __func__);
    }

    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        als_raw = epl_sensor.als.dyn_intt_raw;
        APS_LOG("[%s]: ALS_DYN_INTT: als_raw=%d \r\n", __func__, als_raw);
    }
    else
    {
        als_raw = epl_sensor.als.data.channels[1];
        APS_LOG("[%s]: als_raw=%d \r\n", __func__, als_raw);
    }

    return als_raw;
}

void epl_sensor_enable_ps(int enable)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable);

    APS_LOG("[%s]: ps enable = %d\r\n", __func__, enable);

    if(enable_ps != enable)
    {
        if(enable)
        {
            //wake_lock(&ps_lock);
            set_bit(CMC_BIT_PS, &epld->enable);
#if REG_MONITOR
            write_global_variable(epld->client);
#endif
#if PS_FIRST_REPORT
            ps_first_flag = true;
            set_psensor_intr_threshold(PS_MAX_CT, PS_MAX_CT+1); //set the same threshold
#endif /*PS_FIRST_REPORT*/
#if PS_DYN_K
#if !PS_FIRST_REPORT
            set_psensor_intr_threshold(65534, 65535);   //dont use first ps status
#endif /*PS_FIRST_REPORT*/
            dynk_min_ps_raw_data = 0xffff;
            epl_sensor.ps.compare_low = 0x08; //reg0x1b=0x08, FAR
#endif
        }
        else
        {
            clear_bit(CMC_BIT_PS, &epld->enable);
#if PS_DYN_K
            cancel_delayed_work(&epld->dynk_thd_polling_work);
#elif REG_MONITOR
            cancel_delayed_work(&epld->reg_monitor_polling_work);
#endif
            //wake_unlock(&ps_lock);
        }
        epl_sensor_fast_update(epld->client);
        epl_sensor_update_mode(epld->client);
    }

}

void epl_sensor_enable_als(int enable)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable);

    APS_LOG("[%s]: als enable = %d\r\n", __func__, enable);
    if(enable_als != enable)
    {
        if(enable)
        {
            set_bit(CMC_BIT_ALS, &epld->enable);
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                dynamic_intt_idx = dynamic_intt_init_idx;
                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                epl_sensor.als.als_rs = als_dynamic_intt_rs[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
            }
#endif
            epl_sensor_fast_update(epld->client);
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &epld->enable);
        }
        epl_sensor_update_mode(epld->client);
    }
}

#if REG_MONITOR
int epl_sensor_check_reg(struct i2c_client *client)
{
    u8 buf[2];
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Read(client, DEVREG_LED_CONFIG, 2);
    buf[0] = gRawData.raw_bytes[0];
    buf[1] = gRawData.raw_bytes[1];
    mutex_unlock(&sensor_mutex);
    APS_LOG("[%s]: REG0x05=0x%x, REG0x06 = 0x%x \r\n", __func__, buf[0], buf[1]);

    if(buf[0]!= (epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive)
        || buf[1] != (epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type))
    {
        APS_LOG("[%s]: REG different..... REG0x05=0x%x, REG0x06 = 0x%x \r\n", __func__, buf[0], buf[1]);
        APS_LOG("[%s]: reset all setting \r\n", __func__);
#if PS_DYN_K
        epl_sensor.ps.low_threshold = dynk_thd_low;
        epl_sensor.ps.high_threshold = dynk_thd_high;
#endif
        write_global_variable(client);
        mutex_lock(&sensor_mutex);
        epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_ON | EPL_RESETN_RUN);
        APS_LOG("[%s]: PS CMP RESET/RUN \r\n", __func__);
        epl_sensor_I2C_Write(client, DEVREG_PS_STATUS, EPL_CMP_RESET | EPL_UN_LOCK);
        epl_sensor_I2C_Write(client, DEVREG_PS_STATUS, EPL_CMP_RUN | EPL_UN_LOCK);
        mutex_unlock(&sensor_mutex);
        return -1;
    }
    return 0;
}
#endif /*REG_MONITOR*/

#if (REG_MONITOR && !PS_DYN_K)
void epl_sensor_restart_reg_monitor_polling(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    cancel_delayed_work(&epld->reg_monitor_polling_work);
    schedule_delayed_work(&epld->reg_monitor_polling_work, msecs_to_jiffies(reg_monitor_polling_delay));
}

void epl_sensor_reg_monitor_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    epl_sensor_check_reg(epld->client);

    schedule_delayed_work(&epld->reg_monitor_polling_work, msecs_to_jiffies(reg_monitor_polling_delay));
}
#endif /*(REG_MONITOR && !PS_DYN_K)*/

#if PS_DYN_K
void epl_sensor_restart_dynk_polling(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    cancel_delayed_work(&obj->dynk_thd_polling_work);
    schedule_delayed_work(&obj->dynk_thd_polling_work, msecs_to_jiffies(2*dynk_polling_delay));
}

void epl_sensor_dynk_thd_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    APS_LOG("[%s]:als / ps enable: %d / %d\n", __func__,enable_als, enable_ps);

#if REG_MONITOR
    if(epl_sensor_check_reg(obj->client) != 0)
    {
        schedule_delayed_work(&obj->dynk_thd_polling_work, msecs_to_jiffies(dynk_polling_delay));
        return;
    }
#endif/*REG_MONITOR*/

    if(enable_ps == true)
    {
        epl_sensor_read_ps(obj->client);
        if( (dynk_min_ps_raw_data > epl_sensor.ps.data.data)
            && (epl_sensor.ps.saturation == 0)
#if PS_FIRST_REPORT
            && (epl_sensor.ps.data.data < PS_MAX_CT)
#endif /*PS_FIRST_REPORT*/
            && (epl_sensor.ps.data.ir_data < dynk_max_ir_data) )
        {
            dynk_min_ps_raw_data = epl_sensor.ps.data.data;
            if(enable_ps == true)
            {
                dynk_thd_low = dynk_min_ps_raw_data + dynk_low_offset;
    		    dynk_thd_high = dynk_min_ps_raw_data + dynk_high_offset;
    		    if(dynk_thd_low>65534)
                    dynk_thd_low = 65534;
                if(dynk_thd_high>65535)
                    dynk_thd_high = 65535;
		    }
		    set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
		    APS_LOG("[%s]:dyn ps raw = %d, min = %d, ir_data = %d\n", __func__, epl_sensor.ps.data.data, dynk_min_ps_raw_data, epl_sensor.ps.data.ir_data);
		    APS_LOG("[%s]:dyn k thre_l = %d, thre_h = %d\n", __func__, (u16)dynk_thd_low, (u16)dynk_thd_high);
        }
#if PS_FIRST_REPORT
        else if(epl_sensor.ps.data.data >= PS_MAX_CT && (dynk_min_ps_raw_data > epl_sensor.ps.data.data))
        {
            APS_LOG("[%s]:recover default thesdhold(%d/%d)\n", __func__, epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
        }
#endif /*PS_FIRST_REPORT*/
        schedule_delayed_work(&obj->dynk_thd_polling_work, msecs_to_jiffies(dynk_polling_delay));
    }
}
#endif /*PS_DYN_K*/

//for 3637
static int als_sensing_time(int intt, int osr, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_osr, als_cycle;

    als_intt = als_intt_value[intt>>2];
    als_osr = osr_value[osr>>3];
    als_cycle = cycle_value[cycle];
    APS_LOG("ALS: INTT=%d, OSR=%d, Cycle=%d \r\n", als_intt, als_osr, als_cycle);

    sensing_us_time = (als_intt + als_osr*2*2) * 2 * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;
    APS_LOG("[%s]: sensing=%d ms \r\n", __func__, sensing_ms_time);
    return (sensing_ms_time + 5);
}

static int ps_sensing_time(int intt, int osr, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int ps_intt, ps_osr, ps_cycle;

    ps_intt = ps_intt_value[intt>>2];
    ps_osr = osr_value[osr>>3];
    ps_cycle = cycle_value[cycle];
    APS_LOG("PS: INTT=%d, OSR=%d, Cycle=%d \r\n", ps_intt, ps_osr, ps_cycle);

    sensing_us_time = (ps_intt*3 + ps_osr*2*3) * ps_cycle;
    sensing_ms_time = sensing_us_time / 1000;
    APS_LOG("[%s]: sensing=%d ms\r\n", __func__, sensing_ms_time);
    return (sensing_ms_time + 5);
}

void epl_sensor_fast_update(struct i2c_client *client)
{
    int als_fast_time = 0;

    APS_FUN();
    mutex_lock(&sensor_mutex);
    als_fast_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.osr, EPL_CYCLE_1);

    epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_OFF | EPL_RESETN_RESET);
    if(epl_sensor.als.polling_mode == 0)
    {
        if(epl_sensor.revno == 0x0288)
            epl_sensor_I2C_Write(client, DEVREG_ALS_INT, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | EPL_INTTY_DISABLE);
        else
            epl_sensor_I2C_Write(client, DEVREG_ALS_INT, epl_sensor.als.als_std | epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | EPL_INTTY_DISABLE);
    }
#if ALS_DYN_INTT
    if( (epl_sensor.revno != 0x8188) && (als_dyn_intt_over_flag == false) )
    {
        epl_sensor_I2C_Write(client, DEVREG_ALS_FILT, epl_sensor.als.als_rs | epl_sensor.als.osr | EPL_CYCLE_1);
    }
    else
#endif
    {
        epl_sensor_I2C_Write(client, DEVREG_ALS_FILT, epl_sensor.als.osr | EPL_CYCLE_1);
    }
    if(epl_sensor.revno == 0x0288)
    {
        epl_sensor_I2C_Write(client, DEVREG_ALS_CONFIG, epl_sensor.als.als_std | epl_sensor.als.integration_time | epl_sensor.als.gain);
        epl_sensor_I2C_Write(client, DEVREG_ENABLE, epl_sensor.wait | epl_sensor.als_single_pixel | EPL_MODE_ALS);
    }
    else
    {
        epl_sensor_I2C_Write(client, DEVREG_ALS_CONFIG, epl_sensor.als.als_intb_nonlos | epl_sensor.als.integration_time | epl_sensor.als.gain);
        epl_sensor_I2C_Write(client, DEVREG_ENABLE, epl_sensor.wait | EPL_MODE_ALS);
    }
    epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);

    msleep(als_fast_time);
    APS_LOG("[%s]: msleep(%d)\r\n", __func__, als_fast_time);

    mutex_lock(&sensor_mutex);
    if(epl_sensor.als.polling_mode == 0)
    {
        //fast_mode is already ran one frame, so must to reset CMP bit for als intr mode
        //IDLE mode and CMP reset
        epl_sensor_I2C_Write(client, DEVREG_ENABLE, epl_sensor.wait | EPL_MODE_IDLE);
        epl_sensor_I2C_Write(client, DEVREG_ALS_STATUS, EPL_CMP_RESET | EPL_UN_LOCK);
        epl_sensor_I2C_Write(client, DEVREG_ALS_STATUS, EPL_CMP_RUN | EPL_UN_LOCK);
        if(epl_sensor.revno == 0x0288)
            epl_sensor_I2C_Write(client, DEVREG_ALS_INT, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
        else
            epl_sensor_I2C_Write(client, DEVREG_ALS_INT, epl_sensor.als.als_std | epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    }

    epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_OFF | EPL_RESETN_RESET);
#if ALS_DYN_INTT
    if( (epl_sensor.revno != 0x8188) && (als_dyn_intt_over_flag == false) )
    {
        epl_sensor_I2C_Write(client, DEVREG_ALS_FILT, epl_sensor.als.als_rs | epl_sensor.als.osr | epl_sensor.als.cycle);
    }
    else
#endif
    {
        epl_sensor_I2C_Write(client, DEVREG_ALS_FILT, epl_sensor.als.osr | epl_sensor.als.cycle);
    }
    //epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);
}

void epl_sensor_update_mode(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int als_time = 0, ps_time = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    als_frame_time = als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.osr, epl_sensor.als.cycle);
    ps_frame_time = ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.osr, epl_sensor.ps.cycle);

	APS_LOG("mode selection =0x%x\n", enable_ps | (enable_als << 1));
    //**** mode selection ****
    switch((enable_als << 1) | enable_ps)
    {
        case 0: //disable all
            epl_sensor.mode = EPL_MODE_IDLE;
            break;
        case 1: //als = 0, ps = 1
            epl_sensor.mode = EPL_MODE_PS;
         break;
        case 2: //als = 1, ps = 0
            epl_sensor.mode = EPL_MODE_ALS;
            break;
        case 3: //als = 1, ps = 1
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

    // initial factory calibration variable
    if(epl_sensor.mode != EPL_MODE_IDLE)
        read_factory_calibration();

    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_OFF | EPL_RESETN_RESET);
    if(epl_sensor.revno == 0x0288)
        epl_sensor_I2C_Write(obj->client, DEVREG_ENABLE, epl_sensor.wait | epl_sensor.als_single_pixel | epl_sensor.mode);
    else
        epl_sensor_I2C_Write(obj->client, DEVREG_ENABLE, epl_sensor.wait | epl_sensor.mode);

    if(epl_sensor.mode != EPL_MODE_IDLE)    // if mode isnt IDLE, PWR_ON and RUN
        epl_sensor_I2C_Write(client, DEVREG_RESET, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);

    //**** check setting ****
    if(enable_ps == 1)
    {
        APS_LOG("[%s] PS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
    }
    if(enable_als == 1 && epl_sensor.als.polling_mode == 0)
    {
        APS_LOG("[%s] ALS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    }

    if(epl_sensor.revno == 0x0288)
    {
        APS_LOG("[%s] reg0x00= 0x%x\n", __func__, epl_sensor.wait | epl_sensor.als_single_pixel | epl_sensor.mode);
        APS_LOG("[%s] reg0x07= 0x%x\n", __func__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    }
    else
    {
	    APS_LOG("[%s] reg0x00= 0x%x\n", __func__,  epl_sensor.wait | epl_sensor.mode);
	    APS_LOG("[%s] reg0x07= 0x%x\n", __func__, epl_sensor.als.als_std | epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    }
	APS_LOG("[%s] reg0x06= 0x%x\n", __func__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
	APS_LOG("[%s] reg0x11= 0x%x\n", __func__, epl_sensor.power | epl_sensor.reset);
	APS_LOG("[%s] reg0x12= 0x%x\n", __func__, epl_sensor.als.compare_reset | epl_sensor.als.lock);
	APS_LOG("[%s] reg0x1b= 0x%x\n", __func__, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);


#if PS_FIRST_REPORT
    if(ps_first_flag == true)
    {
#if PS_DYN_K
        ps_first_flag = false;
#endif
        APS_LOG("[%s]: PS CMP RESET/RUN \r\n", __func__);
        mutex_lock(&sensor_mutex);
        epl_sensor_I2C_Write(obj->client, DEVREG_PS_STATUS, EPL_CMP_RESET | EPL_UN_LOCK);
        epl_sensor_I2C_Write(obj->client, DEVREG_PS_STATUS, EPL_CMP_RUN | EPL_UN_LOCK);
        mutex_unlock(&sensor_mutex);
        msleep(ps_time);
        APS_LOG("[%s] PS msleep(%dms)\r\n", __func__, ps_time);
    }
#endif /*PS_FIRST_REPORT*/

    if(enable_ps == 1)
#if PS_DYN_K
        epl_sensor_restart_dynk_polling();
#elif REG_MONITOR
        epl_sensor_restart_reg_monitor_polling();
#endif

}

/*----------------------------------------------------------------------------*/
#if ALS_DYN_INTT
static void epl_sensor_intr_als_set_thd(struct epl_sensor_priv *epld, uint16_t raw, bool chang_setting)
#else
static void epl_sensor_intr_als_set_thd(struct epl_sensor_priv *epld, uint16_t raw)
#endif
{
    uint16_t thd_offset = 0;
    APS_LOG("[%s]: raw=%d, als_intr_thd_percent=%d \r\n", __func__, raw, als_intr_thd_percent);

    thd_offset = raw * als_intr_thd_percent / 100;
    if(thd_offset <= 0)
        thd_offset = 1;
    APS_LOG("[%s]: thd_offset=%d \r\n", __func__, thd_offset);

#if ALS_DYN_INTT
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        //set low threshold
        if(raw <= thd_offset)    //overflow
        {
            if(dynamic_intt_idx == 0)
            {
                if(chang_setting == false)
                    epl_sensor.als.low_threshold = 0;
                else
                    epl_sensor.als.low_threshold = als_dynamic_intt_low_thr[dynamic_intt_idx+1];
            }
            else
                epl_sensor.als.low_threshold = als_dynamic_intt_low_thr[dynamic_intt_idx];
        }
        else
            epl_sensor.als.low_threshold = raw - thd_offset;

        //set low threshold
        if(raw >= (65535-thd_offset)) //overflow
        {
            if( dynamic_intt_idx == (als_dynamic_intt_intt_num-1) )
            {
                if (chang_setting == false)
                    epl_sensor.als.high_threshold = 65535;
                else
                    epl_sensor.als.high_threshold = als_dynamic_intt_high_thr[dynamic_intt_idx-1];
            }
            else
                epl_sensor.als.high_threshold = als_dynamic_intt_high_thr[dynamic_intt_idx];
        }
        else
            epl_sensor.als.high_threshold = raw + thd_offset;

    }
    else
#endif
    {
        //set low threshold
        if(epl_sensor.als.data.channels[1] < thd_offset)    //overflow
            epl_sensor.als.low_threshold = 0;
        else
            epl_sensor.als.low_threshold = raw - thd_offset;

        //set high threshold
        if(epl_sensor.als.data.channels[1] > (0xffff-thd_offset)) //overflow
            epl_sensor.als.high_threshold = 65535;
        else
            epl_sensor.als.high_threshold = raw + thd_offset;
    }
    //set new threshold
    set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
}
/*----------------------------------------------------------------------------*/
static void epl_sensor_intr_als_report_lux(struct epl_sensor_priv *epld)
{
    uint16_t lux;
#if ALS_DYN_INTT
    int last_dyn_intt_idx;
    bool change_flag = false;
    last_dyn_intt_idx = dynamic_intt_idx;
#endif
    lux = epl_sensor_get_als_value(epld, epl_sensor.als.data.channels[1]);
    epl_sensor_report_lux(lux); //report Lux

    APS_LOG("[%s]: report lux = %d \r\n", __func__, lux);

#if ALS_DYN_INTT
    if(last_dyn_intt_idx != dynamic_intt_idx)
        change_flag = true;
    if( (epl_sensor.revno == 0x8188) || (als_dyn_intt_over_flag == true) )
        epl_sensor_intr_als_set_thd(epld, epl_sensor.als.data.channels[1], change_flag);
    else
        epl_sensor_intr_als_set_thd(epld, epl_sensor.als.dyn_intt_raw, change_flag);
#else
    epl_sensor_intr_als_set_thd(epld, epl_sensor.als.data.channels[1]);
#endif
}
/*----------------------------------------------------------------------------*/
static irqreturn_t epl_sensor_eint_func(int irq, void *desc)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    int_top_time = sched_clock();

    if(!obj)
    {
        return IRQ_HANDLED;
    }

    disable_irq_nosync(obj->irq);

    schedule_delayed_work(&obj->eint_work, 0);

    return IRQ_HANDLED;
}
/*----------------------------------------------------------------------------*/
static void epl_sensor_eint_work(struct work_struct *work)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_LOG("xxxxxxxxxxx\n\n");

    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(obj->client, DEVREG_ENABLE, epl_sensor.wait | EPL_MODE_IDLE);
    mutex_unlock(&sensor_mutex);

    epl_sensor_read_ps(obj->client);
    epl_sensor_read_als(obj->client);
    if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER)
    {
#if PS_FIRST_REPORT && !PS_DYN_K
        if(ps_first_flag == true && (epl_sensor.ps.compare_low >> 3) == 1)
        {
            ps_first_flag = false;
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
        }
#endif
        if(enable_ps)
        {
            wake_lock_timeout(&ps_lock, 2*HZ);
            epl_sensor_report_ps_status();
        }
        //PS unlock interrupt pin and restart chip
		mutex_lock(&sensor_mutex);
		epl_sensor_I2C_Write(obj->client, DEVREG_PS_STATUS, EPL_CMP_RUN | EPL_UN_LOCK);
		mutex_unlock(&sensor_mutex);
    }

    if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER)
    {
        epl_sensor_intr_als_report_lux(obj);
        //ALS unlock interrupt pin and restart chip
    	mutex_lock(&sensor_mutex);
    	epl_sensor_I2C_Write(obj->client, DEVREG_ALS_STATUS, EPL_CMP_RESET | EPL_UN_LOCK);
    	epl_sensor_I2C_Write(obj->client, DEVREG_ALS_STATUS, EPL_CMP_RUN | EPL_UN_LOCK);
    	mutex_unlock(&sensor_mutex);
    }
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(obj->client, DEVREG_RESET, EPL_POWER_OFF | EPL_RESETN_RESET);
    if(epl_sensor.revno == 0x0288)
        epl_sensor_I2C_Write(obj->client, DEVREG_ENABLE, epl_sensor.wait | epl_sensor.als_single_pixel | epl_sensor.mode);
    else
        epl_sensor_I2C_Write(obj->client, DEVREG_ENABLE, epl_sensor.wait | epl_sensor.mode);
    epl_sensor_I2C_Write(obj->client, DEVREG_RESET, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);

    enable_irq(obj->irq);
}

int epl_sensor_setup_eint(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
	int ret;
	u32 ints[2] = { 0, 0 };
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
#if !ANDR_O
	struct platform_device *alsps_pdev = get_alsps_platformdev();
#endif
	APS_LOG("epl_sensor_setup_eint\n");
	/*configure to GPIO function, external interrupt */
#ifndef FPGA_EARLY_PORTING
    /* gpio setting */
#if ANDR_O
    pinctrl = devm_pinctrl_get(&client->dev);
#else
	pinctrl = devm_pinctrl_get(&alsps_pdev->dev);
#endif
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");
	}
	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
    pinctrl_select_state(pinctrl, pins_cfg);

	if (obj->irq_node)
	{
#ifndef CONFIG_MTK_EIC
		gpio_request(ints[0], "p-sensor");
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
#else

		ret = of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        if (ret) {
			APS_LOG("of_property_read_u32_array fail, ret = %d\n", ret);
			return ret;
		}
#if !ANDR_O
		gpio_request(ints[0], "p-sensor");
#endif

		gpio_set_debounce(ints[0], ints[1]);
		APS_LOG("EIC:ints[0]=%d,ints[1]=%d!!\n", ints[0], ints[1]);
#endif
		obj->irq = irq_of_parse_and_map(obj->irq_node, 0);
		APS_LOG("obj->irq = %d\n", obj->irq);

		if (!obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(obj->irq, epl_sensor_eint_func, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
	}
	else
	{
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
#if ANDR_O
    enable_irq_wake(obj->irq);
#endif
	enable_irq(obj->irq);
#endif				/* #ifndef FPGA_EARLY_PORTING */
    return 0;
}

static int epl_sensor_init_client(struct i2c_client *client)
{
    int err=0;
    /*  interrupt mode */
    APS_LOG("I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr, __LINE__);
    if((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }

    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_reg(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct i2c_client *client = epl_sensor_obj->client;
    mutex_lock(&sensor_mutex);
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    if(epl_sensor.als.polling_mode == 0)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x08 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x08));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x09));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0A));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0B));
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x13));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x14 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x14));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x15 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x15));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x16 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x16));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x22 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x22));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x23 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x23));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x26 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x26));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x27 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x27));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x28 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x28));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x29 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x29));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x2A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x2A));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFC value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFC));
    mutex_unlock(&sensor_mutex);
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;

    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip is %s, ver is %s \n", EPL_DEV_NAME, DRIVER_VERSION);
    len += snprintf(buf+len, PAGE_SIZE-len, "RENVO = 0x%x\n", epl_sensor.revno);
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps polling is %d-%d\n", epl_sensor.als.polling_mode, epl_sensor.ps.polling_mode);
    if(epl_sensor.revno == 0x0288)
       len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, als_single_pixel = %d, mode = %d\n",epl_sensor.wait >> 4, epl_sensor.als_single_pixel, epl_sensor.mode);
    else
       len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, mode = %d\n",epl_sensor.wait >> 4, epl_sensor.mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "frame time ps=%dms, als=%dms\n", ps_frame_time, als_frame_time);
    if(enable_ps)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "PS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.ps.integration_time >> 2, epl_sensor.ps.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "OSR = %d, cycle = %d, ir drive = %d\n", epl_sensor.ps.osr >> 3, epl_sensor.ps.cycle, epl_sensor.ps.ir_drive);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d, int flag = %d\n", epl_sensor.ps.saturation >> 5, epl_sensor.ps.interrupt_flag >> 2);
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
#if PS_DYN_K
        len += snprintf(buf+len, PAGE_SIZE-len, "Dyn thr(L/H) = (%d/%d)\n", (u16)dynk_thd_low, (u16)dynk_thd_high);
#endif
        len += snprintf(buf+len, PAGE_SIZE-len, "pals data = %d, data = %d\n", epl_sensor.ps.data.ir_data, epl_sensor.ps.data.data);
    }
    if(enable_als)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "ALS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.als.integration_time >> 2, epl_sensor.als.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "OSR = %d, cycle = %d\n", epl_sensor.als.osr >> 3, epl_sensor.als.cycle);
#if ALS_DYN_INTT
        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "c_gain = %d\n", c_gain);
            len += snprintf(buf+len, PAGE_SIZE-len, "dyn_intt_raw=%d, dynamic_intt_lux=%d\n", epl_sensor.als.dyn_intt_raw, dynamic_intt_lux);
        }
#endif
    if(epl_sensor.als.polling_mode == 0)
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch0 = %d, ch1 = %d\n", epl_sensor.als.data.channels[0], epl_sensor.als.data.channels[1]);
    }

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;

    APS_FUN();
    sscanf(buf, "%hu", &mode);
    APS_LOG("[%s]: als enable=%d \r\n", __func__, mode);
    epl_sensor_enable_als(mode);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    APS_FUN();

    sscanf(buf, "%hu", &mode);

    APS_LOG("[%s]: ps enable=%d \r\n", __func__, mode);
    epl_sensor_enable_ps(mode);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_cal_raw(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    u16 ch1=0;
    size_t len = 0;
    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            ch1 = factory_ps_data();
        break;

        case EPL_MODE_ALS:
            ch1 = factory_als_data();
            break;
    }
    APS_LOG("cal_raw = %d \r\n" , ch1);
    len += snprintf(buf + len, PAGE_SIZE - len, "%d \r\n", ch1);

    return  len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_threshold(struct device_driver *ddri,const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int low, high;

    APS_FUN();
    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d", &low, &high);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            obj->hw->ps_threshold_low = low;
            obj->hw->ps_threshold_high = high;
            epl_sensor.ps.low_threshold = low;
            epl_sensor.ps.high_threshold = high;
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
            break;

        case EPL_MODE_ALS:
            obj->hw->als_threshold_low = low;
            obj->hw->als_threshold_high = high;
            epl_sensor.als.low_threshold = low;
            epl_sensor.als.high_threshold = high;
            set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
            break;

    }
    return  count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_threshold(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    ssize_t len = 0;

    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_low=%d \r\n", obj->hw->ps_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_high=%d \r\n", obj->hw->ps_threshold_high);
            break;

        case EPL_MODE_ALS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_low=%d \r\n", obj->hw->als_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_high=%d \r\n", obj->hw->als_threshold_high);
            break;

    }
    return  len;

}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_integration(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
#if ALS_DYN_INTT
    int value1=0, value2=0;
#endif
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();
    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.integration_time = (value & 0xf) << 2;
            if(epl_sensor.revno == 0x0288)
                epl_sensor_I2C_Write(obj->client, DEVREG_PS_CONFIG, epl_sensor.ps.ps_std | epl_sensor.ps.integration_time | epl_sensor.ps.gain);
            else
                epl_sensor_I2C_Write(obj->client, DEVREG_PS_CONFIG, epl_sensor.ps.ps_intb_nonlos | epl_sensor.ps.integration_time | epl_sensor.ps.gain);
            epl_sensor_I2C_Read(obj->client, DEVREG_PS_CONFIG, 1);
            APS_LOG("%s: 0x03 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.ps_intb_nonlos | epl_sensor.ps.integration_time | epl_sensor.ps.gain, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                sscanf(buf, "%d,%d,%d",&value, &value1, &value2);

                als_dynamic_intt_intt[0] = (value & 0xf) << 2;
                als_dynamic_intt_value[0] = als_intt_value[value];

                als_dynamic_intt_intt[1] = (value1 & 0xf) << 2;
                als_dynamic_intt_value[1] = als_intt_value[value1];

                als_dynamic_intt_intt[2] = (value2 & 0xf) << 2;
                als_dynamic_intt_value[2] = als_intt_value[value2];

                APS_LOG("[%s]: als_dynamic_intt_value=%d,%d,%d \r\n", __func__, als_dynamic_intt_value[0], als_dynamic_intt_value[1], als_dynamic_intt_value[2]);

                if( epl_sensor.revno != 0x8188 && epl_sensor.als.report_type == CMC_BIT_DYN_INT )
                {
                    int idx=0, gain_value=0, intt_value=0, total_value=0, i=0;

                    for(i = 0; i < (als_dynamic_intt_intt_num-1); i++)
                    {
                        if(als_dynamic_intt_gain[i] == EPL_GAIN_HIGH)
                            gain_value = 64;
                        else if(als_dynamic_intt_gain[i] == EPL_GAIN_MID)
                            gain_value = 8;
                        else
                            gain_value = 1;

                        intt_value = als_dynamic_intt_value[i] / als_dynamic_intt_value[(als_dynamic_intt_intt_num-1)];
                        total_value = gain_value * intt_value;

                        if(total_value > als_rs_value[rs_num-1])
                        {
                            APS_LOG("[%s]: total_value=%d more than max RS \r\n", __func__, total_value);
                            als_dyn_intt_over_flag = true;
                            break;
                        }
                        else
                        {
                            for(idx = 0; idx < rs_num;  idx++)
                        	{
                        	    if(total_value < als_rs_value[idx])
                        	    {
                        	        break;
                        	    }
                        	}
                        	APS_LOG("[%s]: idx=%d, als_rs_value=%d, total_value=%d\r\n", __func__, idx, als_rs_value[idx-1], total_value);

                        	als_dynamic_intt_rs[i] = ((idx-1)<<5);
                            als_dynamic_intt_high_thr[i] = als_dynamic_intt_high_thr[i]/total_value;
                            als_dynamic_intt_low_thr[i] = als_dynamic_intt_low_thr[i]/total_value;
                            APS_LOG("[%s]: als_dynamic_intt_low_thr[%d]=%d, als_dynamic_intt_high_thr[%d]=%d\r\n", __func__, i, als_dynamic_intt_high_thr[i], i, als_dynamic_intt_high_thr[i]);
                        }

                    }
                }
                dynamic_intt_idx = dynamic_intt_init_idx;
                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                epl_sensor.als.als_rs = als_dynamic_intt_rs[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
            }
            else
#endif
            {
                epl_sensor.als.integration_time = (value & 0xf) << 2;
                if(epl_sensor.revno == 0x0288)
                    epl_sensor_I2C_Write(obj->client, DEVREG_ALS_CONFIG, epl_sensor.als.als_std | epl_sensor.als.integration_time | epl_sensor.als.gain);
                else
                    epl_sensor_I2C_Write(obj->client, DEVREG_ALS_CONFIG, epl_sensor.als.als_intb_nonlos | epl_sensor.als.integration_time | epl_sensor.als.gain);
                epl_sensor_I2C_Read(obj->client, DEVREG_ALS_CONFIG, 1);
                APS_LOG("%s: 0x01 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.als_intb_nonlos | epl_sensor.als.integration_time | epl_sensor.als.gain, gRawData.raw_bytes[0]);
            }
            break;

    }
    epl_sensor_update_mode(obj->client);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;

    APS_FUN();
    sscanf(buf, "%d", &value);
    value = value & 0x03;

	switch (epl_sensor.mode)
	{
        case EPL_MODE_PS:
            epl_sensor.ps.gain = value;
            if(epl_sensor.revno == 0x0288)
                epl_sensor_I2C_Write(epld->client, DEVREG_PS_CONFIG, epl_sensor.ps.ps_std | epl_sensor.ps.integration_time | epl_sensor.ps.gain);
            else
	            epl_sensor_I2C_Write(epld->client, DEVREG_PS_CONFIG, epl_sensor.ps.ps_intb_nonlos | epl_sensor.ps.integration_time | epl_sensor.ps.gain);
		break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.gain = value;
            if(epl_sensor.revno == 0x0288)
                epl_sensor_I2C_Write(epld->client, DEVREG_ALS_CONFIG, epl_sensor.als.als_std | epl_sensor.als.integration_time | epl_sensor.als.gain);
            else
	            epl_sensor_I2C_Write(epld->client, DEVREG_ALS_CONFIG, epl_sensor.als.als_intb_nonlos | epl_sensor.als.integration_time | epl_sensor.als.gain);
		break;
    }
	epl_sensor_update_mode(epld->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_cycle(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();
    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, DEVREG_PS_FILT, epl_sensor.ps.ps_rs | epl_sensor.ps.osr | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, DEVREG_PS_FILT, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.ps_rs | epl_sensor.ps.osr | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
        break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, DEVREG_ALS_FILT, epl_sensor.als.als_rs | epl_sensor.als.osr | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, DEVREG_ALS_FILT, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.als_rs | epl_sensor.als.osr | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
        break;
    }
    epl_sensor_update_mode(obj->client);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_report_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;

    APS_FUN();
    sscanf(buf, "%d", &value);
    epl_sensor.als.report_type = value & 0xf;

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    sscanf(buf, "%d", &epld->hw->polling_mode_ps);
    APS_LOG("epld->hw->polling_mode_ps=%d \r\n", epld->hw->polling_mode_ps);
    epl_sensor.ps.polling_mode = epld->hw->polling_mode_ps;

    alsps_context_obj->ps_ctl.is_polling_mode = epl_sensor.ps.polling_mode==1? true:false;
    alsps_context_obj->ps_ctl.is_report_input_direct = epl_sensor.ps.polling_mode==0? true:false;

    set_als_ps_intr_type(epld->client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    epl_sensor_I2C_Write(epld->client, DEVREG_PS_INT, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
    if(epl_sensor.revno == 0x0288)
        epl_sensor_I2C_Write(epld->client, DEVREG_ALS_INT, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    else
        epl_sensor_I2C_Write(epld->client, DEVREG_ALS_INT, epl_sensor.als.als_std | epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_fast_update(epld->client);
    epl_sensor_update_mode(epld->client);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    sscanf(buf, "%d",&epld->hw->polling_mode_als);
    APS_LOG("epld->hw->polling_mode_als=%d \r\n", epld->hw->polling_mode_als);
    epl_sensor.als.polling_mode = epld->hw->polling_mode_als;

    alsps_context_obj->als_ctl.is_polling_mode = epl_sensor.als.polling_mode==1? true:false;
    alsps_context_obj->als_ctl.is_report_input_direct = epl_sensor.als.polling_mode==0? true:false;

    set_als_ps_intr_type(epld->client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    epl_sensor_I2C_Write(epld->client, DEVREG_PS_INT, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
    if(epl_sensor.revno == 0x0288)
        epl_sensor_I2C_Write(epld->client, DEVREG_ALS_INT, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    else
        epl_sensor_I2C_Write(epld->client, DEVREG_ALS_INT, epl_sensor.als.als_std | epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_fast_update(epld->client);
    epl_sensor_update_mode(epld->client);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_w_calfile(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0;
    int ps_cal_len = 0;
    char ps_calibration[20];

	APS_FUN();
	if(!epl_sensor_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d,%d",&ps_cancelation, &ps_hthr, &ps_lthr);

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d",  ps_cancelation, ps_hthr, ps_lthr);

    write_factory_calibration(epld, ps_calibration, ps_cal_len);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_lux_per_count(struct device_driver *ddri, const char *buf, size_t count)
{
    int lux_per_count = 0;

    sscanf(buf, "%d",&lux_per_count);
    epl_sensor.als.factory.lux_per_count = lux_per_count;

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_ps_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;

    tmp[0]= epl_sensor.ps.polling_mode;

    return 2;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_als_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;

    tmp[0]= epl_sensor.als.polling_mode;

    return 2;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_ps_run_cali(struct device_driver *ddri, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	ssize_t len = 0;
    int ret;

    APS_FUN();
    ret = epl_run_ps_calibration(epld);
    len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\r\n", ret);

	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_pdata(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int ps_raw;

    ps_raw = factory_ps_data();
    APS_LOG("[%s]: ps_raw=%d \r\n", __func__, ps_raw);
    len += snprintf(buf + len, PAGE_SIZE - len, "%d", ps_raw);

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_als_data(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    u16 als_raw = 0;

    als_raw = factory_als_data();
    APS_LOG("[%s]: als_raw=%d \r\n", __func__, als_raw);
    len += snprintf(buf + len, PAGE_SIZE - len, "%d", als_raw);

    return len;
}
/*----------------------------------------------------------------------------*/
#if PS_DYN_K
static ssize_t epl_sensor_store_dyn_offset(struct device_driver *ddri, const char *buf, size_t count)
{
    int dyn_h,dyn_l;

    APS_FUN();
    sscanf(buf, "%d,%d",&dyn_l, &dyn_h);
    dynk_low_offset = dyn_l;
    dynk_high_offset = dyn_h;

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_dyn_max_ir_data(struct device_driver *ddri, const char *buf, size_t count)
{
    int max_ir_data;

    APS_FUN();
    sscanf(buf, "%d",&max_ir_data);
    dynk_max_ir_data = max_ir_data;

    return count;
}
#endif
/*----------------------------------------------------------------------------*/
#if ALS_DYN_INTT
static ssize_t epl_sensor_store_c_gain(struct device_driver *ddri, const char *buf, size_t count)
{
    int als_c_gain;

    APS_FUN();
    sscanf(buf, "%d",&als_c_gain);
    c_gain = als_c_gain;
    APS_LOG("c_gain = %d \r\n", c_gain);

	return count;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_reg_write(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int reg;
    int data;

    APS_FUN();
    sscanf(buf, "%x,%x",&reg, &data);
    APS_LOG("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);

    if(reg == 0x00 && ((data & 0x0f) == EPL_MODE_PS || (data & 0x0f) == EPL_MODE_ALS_PS))
    {
        set_bit(CMC_BIT_PS, &epld->enable);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &epld->enable);
    }
    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(epld->client, reg, data);
    mutex_unlock(&sensor_mutex);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_renvo(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    APS_FUN();
    APS_LOG("gRawData.renvo=0x%x \r\n", epl_sensor.revno);
    len += snprintf(buf+len, PAGE_SIZE-len, "%x", epl_sensor.revno);

    return len;
}
/*----------------------------------------------------------------------------*/
#if ALSPS_DBG
static ssize_t epl_sensor_store_debug_flag(struct device_driver *ddri, const char *buf, size_t count)
{
    int dub_falg=0;

    APS_FUN();
    sscanf(buf, "%d",&dub_falg);
    debug_flag = dub_falg;

    return count;
}
#endif

/*CTS --> S_IWUSR | S_IRUGO*/
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,					S_IWUSR | S_IRUGO, epl_sensor_show_status,  	  		NULL										);
static DRIVER_ATTR(elan_reg,    				S_IWUSR | S_IRUGO, epl_sensor_show_reg,   				NULL										);
static DRIVER_ATTR(als_enable,					S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_als_enable					    );
static DRIVER_ATTR(als_report_type,				S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_als_report_type			);
static DRIVER_ATTR(als_polling_mode,			S_IWUSR | S_IRUGO, epl_sensor_show_als_polling,   		epl_sensor_store_als_polling_mode			);
static DRIVER_ATTR(als_lux_per_count,			S_IWUSR | S_IRUGO, NULL,   					 		epl_sensor_store_als_lux_per_count			    );
static DRIVER_ATTR(ps_enable,					S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_ps_enable					    );
static DRIVER_ATTR(ps_polling_mode,			    S_IWUSR | S_IRUGO, epl_sensor_show_ps_polling,   		epl_sensor_store_ps_polling_mode			);
static DRIVER_ATTR(set_threshold,     			S_IWUSR | S_IRUGO, epl_sensor_show_threshold,          epl_sensor_store_threshold			        );
static DRIVER_ATTR(integration,					S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_integration				);
static DRIVER_ATTR(cal_raw, 					S_IWUSR | S_IRUGO, epl_sensor_show_cal_raw, 	  		NULL										);
static DRIVER_ATTR(run_ps_cali, 				S_IWUSR | S_IRUGO, epl_sensor_show_ps_run_cali, 	  	NULL								    	);
static DRIVER_ATTR(pdata,                       S_IWUSR | S_IRUGO, epl_sensor_show_pdata,              NULL                                         );
static DRIVER_ATTR(als_data,                    S_IWUSR | S_IRUGO, epl_sensor_show_als_data,           NULL                                         );
static DRIVER_ATTR(ps_w_calfile,				S_IWUSR | S_IRUGO, NULL,			                    epl_sensor_store_ps_w_calfile				);
static DRIVER_ATTR(i2c_w,                       S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_reg_write                   );
static DRIVER_ATTR(elan_renvo,                  S_IWUSR | S_IRUGO, epl_sensor_show_renvo,              NULL                                         );
static DRIVER_ATTR(gain,					    S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_gain					    );
static DRIVER_ATTR(cycle,						S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_cycle						);
#if PS_DYN_K
static DRIVER_ATTR(dyn_offset,                  S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_dyn_offset                  );
static DRIVER_ATTR(dyn_max_ir_data,             S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_dyn_max_ir_data             );
#endif
#if ALS_DYN_INTT
static DRIVER_ATTR(als_dyn_c_gain,              S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_c_gain                      );
#endif
#if ALSPS_DBG
static DRIVER_ATTR(dbg_flag,			        S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_debug_flag                  );
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl_sensor_attr_list[] =
{
    &driver_attr_elan_status,
    &driver_attr_elan_reg,
    &driver_attr_als_enable,
    &driver_attr_als_report_type,
    &driver_attr_als_polling_mode,
    &driver_attr_als_lux_per_count,
    &driver_attr_ps_enable,
    &driver_attr_ps_polling_mode,
    &driver_attr_elan_renvo,
    &driver_attr_i2c_w,
    &driver_attr_set_threshold,
    &driver_attr_integration,
    &driver_attr_cal_raw,
    &driver_attr_run_ps_cali,
    &driver_attr_pdata,
    &driver_attr_als_data,
    &driver_attr_ps_w_calfile,
    &driver_attr_gain,
    &driver_attr_cycle,
#if PS_DYN_K
    &driver_attr_dyn_offset,
    &driver_attr_dyn_max_ir_data,
#endif
#if ALS_DYN_INTT
    &driver_attr_als_dyn_c_gain,
#endif
#if ALSPS_DBG
    &driver_attr_dbg_flag,
#endif
};

/*----------------------------------------------------------------------------*/
static int epl_sensor_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }
    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, epl_sensor_attr_list[idx])))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl_sensor_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl_sensor_attr_list[idx]);
    }

    return err;
}

#if !ANDR_O
/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl_sensor_open(struct inode *inode, struct file *file)
{
    file->private_data = epl_sensor_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl_sensor_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long epl_sensor_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err=0, ps_result=0, threshold[2], als_data=0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
#if 0
    int ps_cali;
#endif
    APS_LOG("%s cmd = 0x%04x", __FUNCTION__, cmd);
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            APS_LOG("[%s]: ps enable=%d \r\n", __func__, enable);
            epl_sensor_enable_ps(enable);
        break;

        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_PS_DATA:

            factory_ps_data();
            dat = epl_sensor.ps.compare_low >> 3;

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_PS_RAW_DATA:

            dat = factory_ps_data();

            APS_LOG("ioctl ps raw value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            APS_LOG("[%s]: als enable=%d \r\n", __func__, enable);
            epl_sensor_enable_als(enable);
        break;

        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_ALS_DATA:
            als_data = factory_als_data();
            dat = epl_sensor_get_als_value(obj, als_data);

            APS_LOG("ioctl get als data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_ALS_RAW_DATA:
            dat = factory_als_data();
            APS_LOG("ioctl get als raw data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;
/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:

            dat = factory_ps_data();
            if(dat > obj->hw->ps_threshold_high)
			{
			    ps_result = 0;
			}
			else
			    ps_result = 1;

			APS_LOG("[%s] ps_result = %d \r\n", __func__, ps_result);

			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}
		break;
#if 0 //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

		case ALSPS_IOCTL_CLR_CALI:
#if 0
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
#else

            APS_LOG("[%s]: ALSPS_IOCTL_CLR_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_GET_CALI:
#if 0
			ps_cali = obj->ps_cali ;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
#else
            APS_LOG("[%s]: ALSPS_IOCTL_GET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_SET_CALI:
#if 0
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
#else
            APS_LOG("[%s]: ALSPS_IOCTL_SET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
#if 0
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm
			set_psensor_threshold(obj->client);
#else
            APS_LOG("[%s] set threshold high: %d, low: %d\n", __func__, threshold[0],threshold[1]);
            obj->hw->ps_threshold_high = threshold[0];
            obj->hw->ps_threshold_low = threshold[1];
            set_psensor_intr_threshold(obj->hw->ps_threshold_low, obj->hw->ps_threshold_high);
#endif
			break;
#endif //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		case ALSPS_GET_PS_THRESHOLD_HIGH:
#if 0
			APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_high));
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_high;
            APS_LOG("[%s] get threshold high: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
		break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
#if 0
			APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_low));
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_low;
            APS_LOG("[%s] get threshold low: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
		break;
		/*------------------------------------------------------------------------------------------*/
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
        break;
    }

err_out:
    return err;
}
/*----------------------------------------------------------------------------*/
static struct file_operations epl_sensor_fops =
{
    .owner = THIS_MODULE,
    .open = epl_sensor_open,
    .release = epl_sensor_release,
    .unlocked_ioctl = epl_sensor_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice epl_sensor_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl_sensor_fops,
};
#endif /*ANDR_O*/
/*----------------------------------------------------------------------------*/
#if ANDR_O
static int epl_sensor_i2c_suspend(struct device *dev)
#else
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg)
#endif
{
#if ANDR_O
    struct i2c_client *client = to_i2c_client(dev);
#endif
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);

    APS_FUN();
    if(!obj)
    {
        APS_ERR("[%s]: null pointer!!\n", __func__);
        return -EINVAL;
    }

    return 0;

}
/*----------------------------------------------------------------------------*/
#if ANDR_O
static int epl_sensor_i2c_resume(struct device *dev)
#else
static int epl_sensor_i2c_resume(struct i2c_client *client)
#endif
{
#if ANDR_O
    struct i2c_client *client = to_i2c_client(dev);
#endif
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);

    APS_FUN();
    if(!obj)
    {
        APS_ERR("[%s]: null pointer!!\n", __func__);
        return -EINVAL;
    }

    return 0;
}
/*----------------------------------------------------------------------------*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void epl_sensor_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);

    APS_FUN();
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);

    APS_LOG("[%s]: enable_ps(%d), enable_als(%d) \r\n", __func__, enable_ps, enable_als);

    if(enable_als == 1 && epl_sensor.als.polling_mode == 0)
    {
        APS_LOG("[%s]: check ALS interrupt_flag............ \r\n", __func__);
        epl_sensor_read_als(obj->client);
        if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER)
        {
            APS_LOG("[%s]: epl_sensor.als.interrupt_flag = %d \r\n", __func__, epl_sensor.als.interrupt_flag);
            //ALS unlock interrupt pin and restart chip
            mutex_lock(&sensor_mutex);
        	epl_sensor_I2C_Write(obj->client, DEVREG_ALS_STATUS, EPL_CMP_RESET | EPL_UN_LOCK);
        	epl_sensor_I2C_Write(obj->client, DEVREG_ALS_STATUS, EPL_CMP_RUN | EPL_UN_LOCK);
            mutex_unlock(&sensor_mutex);
        }
    }
    epl_sensor_update_mode(obj->client);

}
/*----------------------------------------------------------------------------*/
static void epl_sensor_late_resume(struct early_suspend *h)
{
    /*late_resume is only applied for ALS*/
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);

    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);

    APS_LOG("[%s]: enable_ps(%d), enable_als(%d) \r\n", __func__, enable_ps, enable_als);
    if(enable_als == 1)
        epl_sensor_fast_update(obj->client);
    epl_sensor_update_mode(obj->client);

}
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

	if(!epld)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

	APS_LOG("[%s]: als enable=%d \r\n", __func__, en);
    epl_sensor_enable_als(en);

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
#if ANDR_O
static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}
static int als_flush(void)
{
	return als_flush_report();
}
#endif
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;
	u16 report_lux = 0;
	struct epl_sensor_priv *obj = epl_sensor_obj;
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

    epl_sensor_read_als(obj->client);

    report_lux = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);

    *value = report_lux;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    if(!epld)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

    APS_LOG("[%s]: ps enable=%d \r\n", __func__, en);
    epl_sensor_enable_ps(en);

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
#if ANDR_O
static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}
static int ps_flush(void)
{
	return ps_flush_report();
}
#endif
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{

    int err = 0;
#if !PS_DYN_K
    struct epl_sensor_priv *obj = epl_sensor_obj;
#endif

    APS_LOG("---SENSOR_GET_DATA---\n\n");
#if !PS_DYN_K
    epl_sensor_read_ps(obj->client);
#if PS_FIRST_REPORT
    if(ps_first_flag == true && (epl_sensor.ps.compare_low >> 3) == 1)
    {
        ps_first_flag = false;
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
    }
#endif
#endif

    *value = epl_sensor.ps.compare_low >> 3;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
#if FACTORY_PATCH
static int epl_sensor_als_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
    int err = 0;

	err = als_enable_nodata(enable_disable ? 1 : 0);

	if (err) {
		APS_ERR("%s:%s failed\n", __func__, enable_disable ? "enable" : "disable");
		return -1;
	}
	err = als_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		APS_ERR("%s set_batch failed\n", __func__);
		return -1;
	}
	return 0;
}

static int epl_sensor_als_factory_get_data(int32_t *data)
{
	int status;

	return als_get_data(data, &status);
}

static int epl_sensor_als_factory_get_raw_data(int32_t *data)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;

	if (!obj) {
		APS_ERR("obj is null!!\n");
		return -1;
	}

	*data = factory_als_data();

	return 0;
}
static int epl_sensor_als_factory_enable_calibration(void)
{
	return 0;
}
static int epl_sensor_als_factory_clear_cali(void)
{
	return 0;
}
static int epl_sensor_als_factory_set_cali(int32_t offset)
{
	return 0;
}
static int epl_sensor_als_factory_get_cali(int32_t *offset)
{
	return 0;
}


static int epl_sensor_ps_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
	int err = 0;

	err = ps_enable_nodata(enable_disable ? 1 : 0);
	if (err) {
		APS_ERR("%s:%s failed\n", __func__, enable_disable ? "enable" : "disable");
		return -1;
	}
	err = ps_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		APS_ERR("%s set_batch failed\n", __func__);
		return -1;
	}
	return err;
}
static int epl_sensor_ps_factory_get_data(int32_t *data)
{
	int err = 0, status = 0;

	err = ps_get_data(data, &status);
	if (err < 0)
		return -1;
	return 0;
}

static int epl_sensor_ps_factory_get_raw_data(int32_t *data)
{
	*data = factory_ps_data();
	return 0;
}
static int epl_sensor_ps_factory_enable_calibration(void)
{
	return 0;
}
static int epl_sensor_ps_factory_clear_cali(void)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;

	obj->ps_cali = 0;

	return 0;
}
static int epl_sensor_ps_factory_set_cali(int32_t offset)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;

	obj->ps_cali = offset;
	return 0;
}
static int epl_sensor_ps_factory_get_cali(int32_t *offset)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;

	*offset = obj->ps_cali;

	return 0;
}
#if 0 //emi
static int epl_sensor_ps_factory_set_threashold(int32_t threshold[2])
{
	int err = 0;
	struct epl_sensor_priv *obj = epl_sensor_obj;

	APS_LOG("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0], threshold[1]);

	epl_sensor.ps.high_threshold = (threshold[0] + obj->ps_cali);
    epl_sensor.ps.low_threshold = (threshold[1] + obj->ps_cali);

	err = set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

	if (err < 0) {
		APS_ERR("set_psensor_threshold fail\n");
		return -1;
	}

	return 0;
}
static int epl_sensor_ps_factory_get_threashold(int32_t threshold[2])
{
	struct epl_sensor_priv *obj = epl_sensor_obj;

	threshold[0] = epl_sensor.ps.high_threshold - obj->ps_cali;
	threshold[1] = epl_sensor.ps.low_threshold - obj->ps_cali;

	return 0;
}
#endif
static struct alsps_factory_fops epl_sensor_factory_fops = {
	.als_enable_sensor = epl_sensor_als_factory_enable_sensor,
	.als_get_data = epl_sensor_als_factory_get_data,
	.als_get_raw_data = epl_sensor_als_factory_get_raw_data,
	.als_enable_calibration = epl_sensor_als_factory_enable_calibration,
	.als_clear_cali = epl_sensor_als_factory_clear_cali,
	.als_set_cali = epl_sensor_als_factory_set_cali,
	.als_get_cali = epl_sensor_als_factory_get_cali,

	.ps_enable_sensor = epl_sensor_ps_factory_enable_sensor,
	.ps_get_data = epl_sensor_ps_factory_get_data,
	.ps_get_raw_data = epl_sensor_ps_factory_get_raw_data,
	.ps_enable_calibration = epl_sensor_ps_factory_enable_calibration,
	.ps_clear_cali = epl_sensor_ps_factory_clear_cali,
	.ps_set_cali = epl_sensor_ps_factory_set_cali,
	.ps_get_cali = epl_sensor_ps_factory_get_cali,
#if 0 //emi
	.ps_set_threashold = epl_sensor_ps_factory_set_threashold,
	.ps_get_threashold = epl_sensor_ps_factory_get_threashold,
#endif
};

static struct alsps_factory_public epl_sensor_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &epl_sensor_factory_fops,
};
#endif /*FACTORY_PATCH*/
/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl_sensor_priv *obj;
    int renvo = 0;
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
    int err = 0;

    APS_FUN();
    epl_sensor_dumpReg(client);

    renvo = i2c_smbus_read_byte_data(client, 0x21);
	if(renvo != 0x81 && renvo != 0x91 && renvo != 0xa1 && renvo != 0x02){
        APS_LOG("elan ALS/PS sensor is failed(0x%x). \n", renvo);
        err = -1;
        goto exit;
    }

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
#if ANDR_O
	err = get_alsps_dts_func(client->dev.of_node, hw);
    if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		goto exit_init_failed;
	}
#endif

    memset(obj, 0, sizeof(*obj));

    epl_sensor_obj = obj;

    obj->hw = hw;

    epl_sensor_get_addr(obj->hw, &obj->addr);

    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));

    INIT_DELAYED_WORK(&obj->eint_work, epl_sensor_eint_work);
#if PS_DYN_K
    INIT_DELAYED_WORK(&obj->dynk_thd_polling_work, epl_sensor_dynk_thd_polling_work);
#elif REG_MONITOR
    INIT_DELAYED_WORK(&obj->reg_monitor_polling_work, epl_sensor_reg_monitor_polling_work);
#endif

    obj->client = client;

    mutex_init(&sensor_mutex);
    wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);
#if ANDR_O
    obj->irq_node = client->dev.of_node;
#else
    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
#endif
    obj->enable = 0;

    epl_sensor_i2c_client = client;

    //initial global variable and write to senosr
    initial_global_variable(client, obj);

    if((err = epl_sensor_init_client(client)))
    {
        goto exit_init_failed;
    }

#if !ANDR_O
    if((err = misc_register(&epl_sensor_device)))
    {
        APS_ERR("epl_sensor_device register failed\n");
        goto exit_misc_device_register_failed;
    }
#endif

#if FACTORY_PATCH
    obj->ps_cali = 0;
    err = alsps_factory_device_register(&epl_sensor_factory_device);
	if (err) {
		APS_ERR("epl_sensor_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;
	APS_LOG("epl_sensor_device misc_register OK!\n");
#endif /*FACTORY_PATCH*/

    if((err = epl_sensor_create_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
#if ANDR_O
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
#endif
	als_ctl.is_report_input_direct = epl_sensor.als.polling_mode==0? true:false;
	als_ctl.is_polling_mode = epl_sensor.als.polling_mode==1? true:false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
    als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
#if ANDR_O
    ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
#endif
	ps_ctl.is_report_input_direct = epl_sensor.ps.polling_mode==0? true:false;
	ps_ctl.is_polling_mode = epl_sensor.ps.polling_mode==1? true:false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
    ps_ctl.is_support_batch = false;
#endif

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    obj->early_drv.suspend  = epl_sensor_early_suspend,
    obj->early_drv.resume   = epl_sensor_late_resume,
    register_early_suspend(&obj->early_drv);
#endif

    if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
        epl_sensor_setup_eint(client);
    alsps_init_flag = 0;

    APS_LOG("%s: OK\n", __FUNCTION__);
    return 0;

exit_create_attr_failed:
#if !ANDR_O
    misc_deregister(&epl_sensor_device);
#endif
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
//	exit_kfree:
    kfree(obj);
exit:
    epl_sensor_i2c_client = NULL;
    alsps_init_flag = -1;
    APS_ERR("%s: err = %d\n", __FUNCTION__, err);
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_remove(struct i2c_client *client)
{
    int err;
    if((err = epl_sensor_delete_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }

#if FACTORY_PATCH
    alsps_factory_device_deregister(&epl_sensor_factory_device);
#endif /*FACTORY_PATCH*/
#if !ANDR_O
    if((err = misc_deregister(&epl_sensor_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }
#endif /*ANDR_O*/
    epl_sensor_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
	//printk("fwq loccal init+++\n");

#if !ANDR_O
	epl_sensor_power(hw, 1);
#endif
	if(i2c_add_driver(&epl_sensor_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(-1 == alsps_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
    APS_FUN();
#if !ANDR_O
    epl_sensor_power(hw, 0);
#endif
    APS_ERR("epl_sensor remove \n");

    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int __init epl_sensor_init(void)
{
#if !ANDR_O
    const char *name = COMPATIABLE_NAME;
#endif
    APS_FUN();
#if !ANDR_O
    hw = get_alsps_dts_func(name, hw);

    if (!hw)
    {
    	APS_ERR("get dts info fail\n");
    }
#endif
    alsps_driver_add(&epl_sensor_init_info);
    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit epl_sensor_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(epl_sensor_init);
module_exit(epl_sensor_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL259x ALPsr driver");
MODULE_LICENSE("GPL");

