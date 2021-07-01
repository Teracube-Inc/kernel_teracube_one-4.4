/* 
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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "ltr579.h"
#include "alsps.h"

#define GN_MTK_BSP_PS_DYNAMIC_CALI
//#define DEMO_BOARD

/******************************************************************************
 * configuration
 *******************************************************************************/
/*----------------------------------------------------------------------------*/
#define LTR579_DEV_NAME			"ltr579"
#define APS_TAG					"[ltr579] "

//#define LTR579_DEBUG

#ifdef LTR579_DEBUG
/*----------------------------------------------------------------------------*/
#define APS_FUN(f)
#define APS_ERR(fmt, args...)   pr_err(APS_TAG fmt, ##args)
#define APS_LOG(fmt, args...)   pr_err(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   pr_err(APS_TAG fmt, ##args)
#else
#define APS_FUN(f)
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_client *ltr579_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr579_i2c_id[] = {{LTR579_DEV_NAME,0},{}};
static unsigned long long int_top_time;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

/*----------------------------------------------------------------------------*/
static int ltr579_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr579_i2c_remove(struct i2c_client *client);
static int ltr579_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ltr579_i2c_suspend(struct device *dev);
static int ltr579_i2c_resume(struct device *dev);
static int ltr579_just_for_reset(void);
static int ltr579_just_for_init(void);


//static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;
static int PS_enable_flag=0;

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr579_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct	eint_work;

    /*misc*/
    u16 		als_modulus;
    atomic_t	i2c_retry;
    atomic_t	als_suspend;
    atomic_t	als_debounce;	/*debounce time after enabling als*/
    atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
    atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
    atomic_t	ps_mask;		/*mask ps: always return far away*/
    atomic_t	ps_debounce;	/*debounce time after enabling ps*/
    atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
    atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
    atomic_t	ps_suspend;
    atomic_t 	trace;

#ifdef CONFIG_OF
    struct device_node *irq_node;
    int		irq;
#endif

    /*data*/
    u16			als;
    u16 		ps;
    u8			_align;
    u16			als_level_num;
    u16			als_value_num;
    u32			als_level[C_CUST_ALS_LEVEL-1];
    u32			als_value[C_CUST_ALS_LEVEL];
    int			ps_cali;

    atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
    atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
    atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
    atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
    atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
    atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
    atomic_t	ps_thd_val;
    ulong		enable; 		/*enable mask*/
    ulong		pending_intr;	/*pending interrupt*/
};

struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;

static struct ltr579_priv *ltr579_obj = NULL;

static DEFINE_MUTEX(ltr579_mutex);

static int ltr579_local_init(void);
static int ltr579_remove(void);
static int ltr579_init_flag =-1; // 0<==>OK -1 <==> fail

static struct alsps_init_info ltr579_init_info = {
    .name = "ltr579",
    .init = ltr579_local_init,
    .uninit = ltr579_remove,	
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
    {.compatible = "mediatek,alsps"},
    {},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltr579_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(ltr579_i2c_suspend, ltr579_i2c_resume)
};
#endif

static struct i2c_driver ltr579_i2c_driver = {	
    .probe      = ltr579_i2c_probe,
    .remove     = ltr579_i2c_remove,
    .detect     = ltr579_i2c_detect,
    .id_table   = ltr579_i2c_id,
    .driver = {
        .name           = LTR579_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
        .pm   = &ltr579_pm_ops,
#endif
#ifdef CONFIG_OF
        .of_match_table = alsps_of_match,
#endif
    },
};

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr579_dynamic_calibrate(void);
static int dynamic_calibrate = 2047;
#endif
/*-----------------------------------------------------------------------------*/

/* 
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ltr579_i2c_read_reg(u8 regnum)
{
    u8 buffer[1],reg_value[1];
    int res = 0;

    buffer[0]= regnum;
    res = i2c_master_send(ltr579_obj->client, buffer, 0x1);
    if(res <= 0)
    {	   
        APS_ERR("read reg send res = %d\n",res);
        return res;
    }
    res = i2c_master_recv(ltr579_obj->client, reg_value, 0x1);
    if(res <= 0)
    {
        APS_ERR("read reg recv res = %d\n",res);
        return res;
    }
    return reg_value[0];
}

// I2C Write
static int ltr579_i2c_write_reg(u8 regnum, u8 value)
{
    u8 databuf[2];    
    int res = 0;

    databuf[0] = regnum;   
    databuf[1] = value;
    res = i2c_master_send(ltr579_obj->client, databuf, 0x2);
    if (res < 0)
    {
        APS_ERR("wirte reg send res = %d\n",res);
        return res;
    }		
    else
        return 0;
}

/*----------------------------------------------------------------------------*/
static void ltr579_power(struct alsps_hw *hw, unsigned int on) 
{
}
/********************************************************************/
/* 
 * ###############
 * ## PS CONFIG ##
 * ###############

 */
static int ltr579_ps_set_thres(void)
{

    int res;
    u8 databuf[2];

    struct i2c_client *client = ltr579_obj->client;
    struct ltr579_priv *obj = ltr579_obj;
    APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);

    if(1 == ps_cali.valid)
    {
        databuf[0] = LTR579_PS_THRES_LOW_0; 
        databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {			
            goto EXIT_ERR;
        }
        databuf[0] = LTR579_PS_THRES_LOW_1; 
        databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
        databuf[0] = LTR579_PS_THRES_UP_0;	
        databuf[1] = (u8)(ps_cali.close & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
        databuf[0] = LTR579_PS_THRES_UP_1;	
        databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
    }
    else
    {
        databuf[0] = LTR579_PS_THRES_LOW_0; 
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
        databuf[0] = LTR579_PS_THRES_LOW_1; 
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);

        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
        databuf[0] = LTR579_PS_THRES_UP_0;	
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
        databuf[0] = LTR579_PS_THRES_UP_1;	
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }	
    }

    res = 0;
    return res;

EXIT_ERR:
    APS_ERR("set thres: %d\n", res);
    res = LTR579_ERR_I2C;
    return res;
}

static int ltr579_ps_enable(struct i2c_client *client, int enable)
{
    u8 regdata;
    int err;
    int pulse_test;
    int interrupt_test;

    APS_LOG("ltr579_ps_enable() ...start!\n");

    /*added by fully for test whether the sensor is reset 20160802*/
    pulse_test = ltr579_i2c_read_reg(LTR579_PS_PULSES);
    interrupt_test = ltr579_i2c_read_reg(LTR579_INT_CFG);
    if((pulse_test != 32) || (interrupt_test != 1)){
        ltr579_just_for_init();
        APS_ERR("added by fully. \n");
    }

    regdata = ltr579_i2c_read_reg(LTR579_MAIN_CTRL);
    if (enable != 0) {
        APS_LOG("PS: enable ps only \n");
        regdata |= 0x01;
        PS_enable_flag=1;
    }
    else {
        APS_LOG("PS: disable ps only \n");
        regdata &= 0xFE;
        PS_enable_flag=0;
    }

    err = ltr579_i2c_write_reg(LTR579_MAIN_CTRL, regdata);
    if (err < 0)
    {
        APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
        return err;
    }
    mdelay(WAKEUP_DELAY);
    ltr579_i2c_read_reg(LTR579_MAIN_CTRL);

    if (0 == ltr579_obj->hw->polling_mode_ps && enable != 0)
    {
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
        err = ltr579_dynamic_calibrate();
        if (err < 0)
        {
            APS_LOG("ltr579_dynamic_calibrate() failed\n");
        }
#endif
        ltr579_ps_set_thres();
    }
    else if (0 == ltr579_obj->hw->polling_mode_ps && enable == 0)
    {
        cancel_work_sync(&ltr579_obj->eint_work);		
    }

    return err;
}

/********************************************************************/
static int ltr579_ps_read(struct i2c_client *client, u16 *data)
{
    int psval_lo, psval_hi, psdata;

    psval_lo = ltr579_i2c_read_reg(LTR579_PS_DATA_0);
    APS_DBG("ps_rawdata_psval_lo = %d\n", psval_lo);
    if (psval_lo < 0){	    
        APS_DBG("psval_lo error\n");
        psdata = psval_lo;
        goto out;
    }

    psval_hi = ltr579_i2c_read_reg(LTR579_PS_DATA_1);
    APS_DBG("ps_rawdata_psval_hi = %d\n", psval_hi);
    if (psval_hi < 0){
        APS_DBG("psval_hi error\n");
        psdata = psval_hi;
        goto out;
    }

    psdata = ((psval_hi & 7)* 256) + psval_lo;
    *data = psdata;
    APS_DBG("ltr579_ps_read: ps_rawdata = %d\n", psdata);

out:
    final_prox_val = psdata;	
    return psdata;
}

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr579_dynamic_calibrate(void)
{
    int i = 0;
    int data;
    int data_total = 0;
    int noise = 0;
    int count = 5;
    int ps_thd_val_low, ps_thd_val_high;
    struct ltr579_priv *obj = ltr579_obj;

    if (!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return -1;
    }	

    for (i = 0; i < count; i++) {
        // wait for ps value be stable
        msleep(15);

        data = ltr579_ps_read(ltr579_obj->client, &ltr579_obj->ps);
        if (data < 0) {
            i--;
            continue;
        }

        if (data & 0x0800) {
            break;
        }		

        data_total += data;
    }

    noise = data_total / count;
    /*added by fully for fisrt enable20160803*/
    if(noise > dynamic_calibrate + 100){
        noise = dynamic_calibrate + 100;
    }else{
        dynamic_calibrate = noise;
    }
    /*the end added by fully 20160803*/

    if (noise < 100) {
        ps_thd_val_high = noise + 50;
        ps_thd_val_low  = noise + 30;
    }
    else if (noise < 200) {
        ps_thd_val_high = noise + 60;
        ps_thd_val_low  = noise + 40;
    }
    else if (noise < 300) {
        ps_thd_val_high = noise + 70;
        ps_thd_val_low  = noise + 50;
    }
    else if (noise < 400) {
        ps_thd_val_high = noise + 100;
        ps_thd_val_low  = noise + 60;
    }
    else if (noise < 600) {
        ps_thd_val_high = noise + 180;
        ps_thd_val_low  = noise + 90;
    }
    else if (noise < 1000) {
        ps_thd_val_high = noise + 300;
        ps_thd_val_low  = noise + 180;
    }
    else if (noise < 1250) {
        ps_thd_val_high = noise + 400;
        ps_thd_val_low  = noise + 300;
    }
    else {
        ps_thd_val_high = (noise + 1600) > 2047 ? 2047 : noise + 1600;
        ps_thd_val_low  = (noise + 1000) > 2047 ? 2047 : noise + 1000;
        APS_ERR("dynamic calibrate fails!!\n");
    }

    atomic_set(&obj->ps_thd_val_high, ps_thd_val_high);
    atomic_set(&obj->ps_thd_val_low, ps_thd_val_low);

    APS_LOG("%s:noise = %d\n", __func__, noise);
    APS_LOG("%s:obj->ps_thd_val_high = %d\n", __func__, ps_thd_val_high);
    APS_LOG("%s:obj->ps_thd_val_low = %d\n", __func__, ps_thd_val_low);

    return 0;
}
#endif
/********************************************************************/
/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr579_als_enable(struct i2c_client *client, int enable)
{
    int err = 0;
    u8 regdata = 0;

    regdata = ltr579_i2c_read_reg(LTR579_MAIN_CTRL);
    if (enable != 0) {
        APS_LOG("ALS(1): enable als only \n");
        regdata |= 0x02;
    }
    else {
        APS_LOG("ALS(1): disable als only \n");
        regdata &= 0xFD;
    }

    err = ltr579_i2c_write_reg(LTR579_MAIN_CTRL, regdata);
    if (err < 0)
    {
        APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
        return err;
    }

    mdelay(WAKEUP_DELAY);

    return 0;
}

static int ltr579_als_read(struct i2c_client *client, u16* data)
{
    int alsval_0, alsval_1, alsval_2, alsval;
    int luxdata_int;

    alsval_0 = ltr579_i2c_read_reg(LTR579_ALS_DATA_0);
    alsval_1 = ltr579_i2c_read_reg(LTR579_ALS_DATA_1);
    alsval_2 = ltr579_i2c_read_reg(LTR579_ALS_DATA_2);
    alsval = (alsval_2 * 256 * 256) + (alsval_1 * 256) + alsval_0;
    APS_DBG("alsval_0 = %d,alsval_1=%d,alsval_2=%d,alsval=%d\n", alsval_0, alsval_1, alsval_2, alsval);

    if (alsval == 0)
    {
        luxdata_int = 0;
        goto out;
    }
#if 0
    luxdata_int = alsval * 8 / als_gainrange / 10;//formula: ALS counts * 0.8/gain/int , int=1	
#else	
    luxdata_int = alsval * 8 / 10;
#endif
    APS_DBG("ltr579_als_read: als_value_lux = %d\n", luxdata_int);
out:
    *data = luxdata_int;
    final_lux_val = luxdata_int;
    return luxdata_int;
}
/********************************************************************/
static int ltr579_get_ps_value(struct ltr579_priv *obj, u16 ps)
{
    int val = 1;
    //int invalid = 0;
#if 1
    int buffer = 0;
    int ps_flag;

    buffer = ltr579_i2c_read_reg(LTR579_MAIN_STATUS);
    if (buffer < 0) {
        return -1;
    }

    ps_flag = buffer & 0x04;
    ps_flag = ps_flag >> 2;
    if (ps_flag == 1) //Near
    {
        intr_flag_value = 1;
        val = 0;
    }
    else if (ps_flag == 0) //Far
    {
        intr_flag_value = 0;
        val = 1;
    }

    return val;
#else
    if((ps > atomic_read(&obj->ps_thd_val_high)))
    {
        val = 0;  /*close*/
        intr_flag_value = 1;
    }
    else if((ps < atomic_read(&obj->ps_thd_val_low)))
    {
        val = 1;  /*far away*/
        intr_flag_value = 0;
    }

    if(atomic_read(&obj->ps_suspend))
    {
        invalid = 1;
    }
    else if(1 == atomic_read(&obj->ps_deb_on))
    {
        unsigned long endt = atomic_read(&obj->ps_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->ps_deb_on, 0);
        }

        if (1 == atomic_read(&obj->ps_deb_on))
        {
            invalid = 1;
        }
    }
    else if (obj->als > 50000)
    {
        //invalid = 1;
        APS_DBG("ligh too high will result to failt proximiy\n");
        return 1;  /*far away*/
    }

    if(!invalid)
    {
        APS_DBG("PS:  %05d => %05d\n", ps, val);
        return val;
    }	
    else
    {
        return -1;
    }
#endif
}
/********************************************************************/
static int ltr579_get_als_value(struct ltr579_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
    APS_DBG("als  = %d\n",als); 
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

    if(1 == atomic_read(&obj->als_deb_on))
    {
        unsigned long endt = atomic_read(&obj->als_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->als_deb_on, 0);
        }

        if(1 == atomic_read(&obj->als_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
        return obj->hw->als_value[idx];
    }
    else
    {
        APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
        return -1;
    }
}
/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
 *******************************************************************************/
static ssize_t ltr579_show_config(struct device_driver *ddri, char *buf)
{
    ssize_t res;

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
            atomic_read(&ltr579_obj->i2c_retry), atomic_read(&ltr579_obj->als_debounce), 
            atomic_read(&ltr579_obj->ps_mask), atomic_read(&ltr579_obj->ps_thd_val), atomic_read(&ltr579_obj->ps_debounce));     
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
    int retry, als_deb, ps_deb, mask, thres;
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }

    if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
    { 
        atomic_set(&ltr579_obj->i2c_retry, retry);
        atomic_set(&ltr579_obj->als_debounce, als_deb);
        atomic_set(&ltr579_obj->ps_mask, mask);
        atomic_set(&ltr579_obj->ps_thd_val, thres);        
        atomic_set(&ltr579_obj->ps_debounce, ps_deb);
    }
    else
    {
        APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
    }
    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_trace(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr579_obj->trace));     
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }

    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&ltr579_obj->trace, trace);
    }
    else 
    {
        APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
    }
    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_als(struct device_driver *ddri, char *buf)
{
    int res;

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }
    res = ltr579_als_read(ltr579_obj->client, &ltr579_obj->als);
    return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);

}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_ps(struct device_driver *ddri, char *buf)
{
    int  res;
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }
    res = ltr579_ps_read(ltr579_obj->client, &ltr579_obj->ps);
    return snprintf(buf, PAGE_SIZE, "0x%04X\n", ltr579_obj->ps);

}

/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_reg(struct device_driver *ddri, char *buf)
{
    int i,len=0;
    int reg[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0d,0x0e,0x0f,
        0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26};
    for(i=0;i<27;i++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%04X value: 0x%04X\n", reg[i],ltr579_i2c_read_reg(reg[i]));	
    }
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
    int addr, cmd;
    u8 dat;

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }
    else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
    {
        APS_ERR("invalid format: '%s'\n", buf);
        return 0;
    }

    dat = (u8)cmd;
    //****************************
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
    int addr;

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }
    else if(1 != sscanf(buf, "%x", &addr))
    {
        APS_ERR("invalid format: '%s'\n", buf);
        return 0;
    }

    //****************************
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }

    if(ltr579_obj->hw)
    {	
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
                ltr579_obj->hw->i2c_num, ltr579_obj->hw->power_id, ltr579_obj->hw->power_vol);		
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }


    len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr579_obj->als_suspend), atomic_read(&ltr579_obj->ps_suspend));

    return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr579_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
    int idx = 0;
    char *cur = (char*)buf, *end = (char*)(buf+count);

    while(idx < len)
    {
        while((cur < end) && IS_SPACE(*cur))
        {
            cur++;        
        }

        if(1 != sscanf(cur, "%d", &data[idx]))
        {
            break;
        }

        idx++; 
        while((cur < end) && !IS_SPACE(*cur))
        {
            cur++;
        }
    }
    return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_alslv(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr579_obj->als_level_num; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr579_obj->hw->als_level[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }
    else if(!strcmp(buf, "def"))
    {
        memcpy(ltr579_obj->als_level, ltr579_obj->hw->als_level, sizeof(ltr579_obj->als_level));
    }
    else if(ltr579_obj->als_level_num != read_int_from_buf(ltr579_obj, buf, count, 
                ltr579_obj->hw->als_level, ltr579_obj->als_level_num))
    {
        APS_ERR("invalid format: '%s'\n", buf);
    }    
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_alsval(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr579_obj->als_value_num; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr579_obj->hw->als_value[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return 0;
    }
    else if(!strcmp(buf, "def"))
    {
        memcpy(ltr579_obj->als_value, ltr579_obj->hw->als_value, sizeof(ltr579_obj->als_value));
    }
    else if(ltr579_obj->als_value_num != read_int_from_buf(ltr579_obj, buf, count, 
                ltr579_obj->hw->als_value, ltr579_obj->als_value_num))
    {
        APS_ERR("invalid format: '%s'\n", buf);
    }    
    return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,    S_IRUGO, ltr579_show_als,		NULL);
static DRIVER_ATTR(ps,      S_IRUGO, ltr579_show_ps,		NULL);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, ltr579_show_config,	ltr579_store_config);
static DRIVER_ATTR(alslv,  S_IWUSR | S_IRUGO, ltr579_show_alslv,	ltr579_store_alslv);
static DRIVER_ATTR(alsval, S_IWUSR | S_IRUGO, ltr579_show_alsval,	ltr579_store_alsval);
static DRIVER_ATTR(trace,  S_IWUSR | S_IRUGO, ltr579_show_trace,	ltr579_store_trace);
static DRIVER_ATTR(status, S_IRUGO, ltr579_show_status,	NULL);
static DRIVER_ATTR(send,   S_IWUSR | S_IRUGO, ltr579_show_send,	ltr579_store_send);
static DRIVER_ATTR(recv,   S_IWUSR | S_IRUGO, ltr579_show_recv,	ltr579_store_recv);
static DRIVER_ATTR(reg,    S_IRUGO, ltr579_show_reg,		NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr579_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int ltr579_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(ltr579_attr_list)/sizeof(ltr579_attr_list[0]));

    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, ltr579_attr_list[idx])))
        {            
            APS_ERR("driver_create_file (%s) = %d\n", ltr579_attr_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr579_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(ltr579_attr_list)/sizeof(ltr579_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) 
    {
        driver_remove_file(driver, ltr579_attr_list[idx]);
    }

    return err;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------interrupt functions--------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
#if 0
static int ltr579_check_and_clear_intr(struct i2c_client *client) 
{

    int res,intp,intl;
    u8 buffer[2];	

    APS_FUN();
    //if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/	
    //	  return 0;

    buffer[0] = LTR579_MAIN_STATUS;
    res = i2c_master_send(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    res = i2c_master_recv(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }

    res = 1;
    intp = 0;
    intl = 0;
    if(0 != (buffer[0] & 0x02))
    {
        res = 0;
        intp = 1;
    }
    if(0 != (buffer[0] & 0x10))
    {
        res = 0;
        intl = 1;		
    }

    if(0 == res)
    {
        if((1 == intp) && (0 == intl))
        {
            buffer[1] = buffer[0] & 0xFD;			
        }
        else if((0 == intp) && (1 == intl))
        {
            buffer[1] = buffer[0] & 0xEF;
        }
        else
        {
            buffer[1] = buffer[0] & 0xED;
        }
        buffer[0] = LTR579_MAIN_STATUS;
        res = i2c_master_send(client, buffer, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
        else
        {
            res = 0;
        }
    }
    else
        return 0;

EXIT_ERR:
    APS_ERR("ltr579_check_and_clear_intr fail\n");
    return 1;
}
static int ltr579_check_intr(struct i2c_client *client) 
{

    int res,intp,intl;
    u8 buffer[2];

    //if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
    //    return 0;

    APS_FUN();
    buffer[0] = LTR579_MAIN_STATUS;
    res = i2c_master_send(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    res = i2c_master_recv(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }

    res = 0;
    intp = 0;
    intl = 0;
    if (0 != (buffer[0] & 0x02))
    {
        res = 1;	//PS int
        intp = 1;
    }
    if (0 != (buffer[0] & 0x10))
    {
        res = 2;	//ALS int
        intl = 1;		
    }
    if ((intp == 1) && (intl == 1))
    {
        res = 4;	//ALS & PS int		
    }

    return res;

EXIT_ERR:
    APS_ERR("ltr579_check_intr fail\n");
    return 0;
}

static int ltr579_clear_intr(struct i2c_client *client) 
{
    int res;
    u8 buffer[2];

    APS_FUN();

    buffer[0] = LTR579_MAIN_STATUS;
    res = i2c_master_send(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    res = i2c_master_recv(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    APS_DBG("buffer[0] = %d \n",buffer[0]);
    buffer[1] = buffer[0] & 0xED;
    buffer[0] = LTR579_MAIN_STATUS;

    res = i2c_master_send(client, buffer, 0x2);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    else
    {
        res = 0;
    }

    return res;

EXIT_ERR:
    APS_ERR("ltr579_clear_intr fail\n");
    return 1;
}
#endif
#endif //#ifndef CUSTOM_KERNEL_SENSORHUB
/*----------------------------------------------------------------------------*/
static void ltr579_eint_work(struct work_struct *work)
{
    struct ltr579_priv *obj = (struct ltr579_priv *)container_of(work, struct ltr579_priv, eint_work);
    //u8 databuf[2];
    int res = 0;
    int err;
    int value = 1;

    //err = ltr579_check_intr(obj->client);
    //if (err < 0) {
    //	goto EXIT_INTR;
    //}
    //else
    {
        //get raw data
        obj->ps = ltr579_ps_read(obj->client, &obj->ps);
        if (obj->ps < 0)
        {
            err = -1;
            goto EXIT_INTR;
        }

        APS_DBG("ltr579_eint_work: rawdata ps=%d!\n",obj->ps);
        value = ltr579_get_ps_value(obj, obj->ps);
        APS_DBG("intr_flag_value=%d\n",intr_flag_value);
        if(intr_flag_value){
            /*
               databuf[0] = LTR579_PS_THRES_LOW_0;	
               databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
               res = i2c_master_send(obj->client, databuf, 0x2);
               if(res <= 0)
               {
               goto EXIT_INTR;
               }
               databuf[0] = LTR579_PS_THRES_LOW_1;	
               databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
               res = i2c_master_send(obj->client, databuf, 0x2);
               if(res <= 0)
               {
               goto EXIT_INTR;
               }
               databuf[0] = LTR579_PS_THRES_UP_0;	
               databuf[1] = (u8)(0x00FF);
               res = i2c_master_send(obj->client, databuf, 0x2);
               if(res <= 0)
               {
               goto EXIT_INTR;
               }
               databuf[0] = LTR579_PS_THRES_UP_1; 
               databuf[1] = (u8)((0xFF00) >> 8);;
               res = i2c_master_send(obj->client, databuf, 0x2);
               if(res <= 0)
               {
               goto EXIT_INTR;
               }
             */
        }
        else{	
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
            if(obj->ps > 20 && obj->ps < (dynamic_calibrate - 50)){ 
                if(obj->ps < 100){			
                    atomic_set(&obj->ps_thd_val_high,  obj->ps+100);
                    atomic_set(&obj->ps_thd_val_low, obj->ps+50);
                }else if(obj->ps < 200){
                    atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
                    atomic_set(&obj->ps_thd_val_low, obj->ps+60);
                }else if(obj->ps < 300){
                    atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
                    atomic_set(&obj->ps_thd_val_low, obj->ps+60);
                }else if(obj->ps < 400){
                    atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
                    atomic_set(&obj->ps_thd_val_low, obj->ps+60);
                }else if(obj->ps < 600){
                    atomic_set(&obj->ps_thd_val_high,  obj->ps+180);
                    atomic_set(&obj->ps_thd_val_low, obj->ps+90);
                }else if(obj->ps < 1000){
                    atomic_set(&obj->ps_thd_val_high,  obj->ps+300);
                    atomic_set(&obj->ps_thd_val_low, obj->ps+180);	
                }else if(obj->ps < 1250){
                    atomic_set(&obj->ps_thd_val_high,  obj->ps+400);
                    atomic_set(&obj->ps_thd_val_low, obj->ps+300);
                }
                else{
                    atomic_set(&obj->ps_thd_val_high,  1400);
                    atomic_set(&obj->ps_thd_val_low, 1000);        			
                }

                dynamic_calibrate = obj->ps;
            }	        
#endif      
            /*  	
                    databuf[0] = LTR579_PS_THRES_LOW_0;	
                    databuf[1] = (u8)(0 & 0x00FF);
                    res = i2c_master_send(obj->client, databuf, 0x2);
                    if(res <= 0)
                    {
                    goto EXIT_INTR;
                    }
                    databuf[0] = LTR579_PS_THRES_LOW_1;	
                    databuf[1] = (u8)((0 & 0xFF00) >> 8);
                    res = i2c_master_send(obj->client, databuf, 0x2);
                    if(res <= 0)
                    {
                    goto EXIT_INTR;
                    }
                    databuf[0] = LTR579_PS_THRES_UP_0;	
                    databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
                    res = i2c_master_send(obj->client, databuf, 0x2);
                    if(res <= 0)
                    {
                    goto EXIT_INTR;
                    }
                    databuf[0] = LTR579_PS_THRES_UP_1; 
                    databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
                    res = i2c_master_send(obj->client, databuf, 0x2);
                    if(res <= 0)
                    {
                    goto EXIT_INTR;
                    }
             */
        }
        //let up layer to know
        res = ps_report_interrupt_data(value);
    }

EXIT_INTR:
    //ltr579_clear_intr(obj->client);
#ifdef CONFIG_OF
    enable_irq(obj->irq);
#endif
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void ltr579_eint_func(void)
{
    struct ltr579_priv *obj = ltr579_obj;
    if(!obj)
    {
        return;
    }	
    int_top_time = sched_clock();
    schedule_work(&obj->eint_work);
}
#ifdef CONFIG_OF
static irqreturn_t ltr579_eint_handler(int irq, void *desc)
{
    ltr579_eint_func();
    disable_irq_nosync(ltr579_obj->irq);

    return IRQ_HANDLED;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int ltr579_setup_eint(struct i2c_client *client)
{

    int ret;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_cfg;
    u32 ints[2] = { 0, 0 };	

    APS_FUN();

    /* gpio setting */
    pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);
        APS_ERR("Cannot find alsps pinctrl!\n");
    }
    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        ret = PTR_ERR(pins_cfg);
        APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
    }
    pinctrl_select_state(pinctrl, pins_cfg);

    /* eint request */
    if (ltr579_obj->irq_node) {
        of_property_read_u32_array(ltr579_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);
        APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        ltr579_obj->irq = irq_of_parse_and_map(ltr579_obj->irq_node, 0);
        APS_LOG("ltr579_obj->irq = %d\n", ltr579_obj->irq);
        if (!ltr579_obj->irq) {
            APS_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        if (request_irq(ltr579_obj->irq, ltr579_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
            APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
        enable_irq(ltr579_obj->irq);
        enable_irq_wake(ltr579_obj->irq);
    }
    else {
        APS_ERR("null irq node!!\n");
        return -EINVAL;
    }

    return 0;
}
/**********************************************************************************************/

/*-------------------------------MISC device related------------------------------------------*/
static int ltr579_open(struct inode *inode, struct file *file)
{
    file->private_data = ltr579_i2c_client;

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}
/************************************************************/
static int ltr579_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/************************************************************/
static long ltr579_unlocked_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)       
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct ltr579_priv *obj = i2c_get_clientdata(client);  
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    int ps_cali;
    int threshold[2];
    APS_DBG("cmd= %d\n", cmd); 
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            err = ltr579_ps_enable(obj->client, enable);
            if (err < 0)
            {
                APS_ERR("enable ps fail: %d en: %d\n", err, enable);
                goto err_out;
            }
            if (enable)
                set_bit(CMC_BIT_PS, &obj->enable);
            else
                clear_bit(CMC_BIT_PS, &obj->enable);				
            break;

        case ALSPS_GET_PS_MODE:
            enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA:
            APS_DBG("ALSPS_GET_PS_DATA\n"); 
            obj->ps = ltr579_ps_read(obj->client, &obj->ps);
            if (obj->ps < 0)
            {
                goto err_out;
            }

            dat = ltr579_get_ps_value(obj, obj->ps);
            if (copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:    
            obj->ps = ltr579_ps_read(obj->client, &obj->ps);
            if (obj->ps < 0)
            {
                goto err_out;
            }
            dat = obj->ps;
            if (copy_to_user(ptr, &dat, sizeof(dat)))
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
            err = ltr579_als_enable(obj->client, enable);
            if (err < 0)
            {
                APS_ERR("enable als fail: %d en: %d\n", err, enable);
                goto err_out;
            }
            if (enable)
                set_bit(CMC_BIT_ALS, &obj->enable);
            else
                clear_bit(CMC_BIT_ALS, &obj->enable);
            break;

        case ALSPS_GET_ALS_MODE:
            enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_DATA: 
            obj->als = ltr579_als_read(obj->client, &obj->als);
            if (obj->als < 0)
            {
                goto err_out;
            }

            dat = ltr579_get_als_value(obj, obj->als);
            if (copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_RAW_DATA:    
            obj->als = ltr579_als_read(obj->client, &obj->als);
            if (obj->als < 0)
            {
                goto err_out;
            }

            dat = obj->als;
            if (copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

            /*----------------------------------for factory mode test---------------------------------------*/
        case ALSPS_GET_PS_TEST_RESULT:
            obj->ps = ltr579_ps_read(obj->client, &obj->ps);
            if (obj->ps < 0)
            {
                goto err_out;
            }
            if(obj->ps > atomic_read(&obj->ps_thd_val_low))
                dat = 1;
            else	
                dat = 0;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }				   
            break;

        case ALSPS_IOCTL_CLR_CALI:
            if(copy_from_user(&dat, ptr, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(dat == 0)
                obj->ps_cali = 0;
            break;

        case ALSPS_IOCTL_GET_CALI:
            ps_cali = obj->ps_cali ;
            if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_IOCTL_SET_CALI:
            if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
            {
                err = -EFAULT;
                goto err_out;
            }
            obj->ps_cali = ps_cali;
            break;

            //case ALSPS_CALIBRATE_PS:    //ps D!j!A?
            //ps_calibration();
            //break;

            //case ALSPS_WRITE_CALIBRATE:     //??((? psD!j!A?(:y?Y
            //get_ calibration_data();
            //set_ps_pulse();
            //set_ps_thres();
            //break;

        case ALSPS_SET_PS_THRESHOLD:
            if(copy_from_user(threshold, ptr, sizeof(threshold)))
            {
                err = -EFAULT;
                goto err_out;
            }
            atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
            atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

            ltr579_ps_set_thres();
            break;

        case ALSPS_GET_PS_THRESHOLD_HIGH:
            threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
            if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_THRESHOLD_LOW:
            threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
            if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
            /*------------------------------------------------------------------------------------------*/

        default:
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;    
}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations ltr579_fops = {
    .owner = THIS_MODULE,
    .open = ltr579_open,
    .release = ltr579_release,
    .unlocked_ioctl = ltr579_unlocked_ioctl,
};

static struct miscdevice ltr579_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &ltr579_fops,
};


/*--------------------------------------------------------------------------------*/
static int ltr579_just_for_reset(void)
{


    //struct i2c_client *client = ltr579_obj->client;

    //struct ltr579_priv *obj = ltr579_obj;

    int pulse_test;
    int interrupt_test;
    int ps_enable_data;

    APS_ERR("added by fully.\n");

    pulse_test = ltr579_i2c_read_reg(LTR579_PS_PULSES);
    interrupt_test = ltr579_i2c_read_reg(LTR579_INT_CFG);

    APS_ERR("added by fully ps pulse: %d, interrupt_test: %d .\n", pulse_test, interrupt_test);

    if((pulse_test != 32) || (interrupt_test != 1)){
        mdelay(50);
        ltr579_just_for_init();

        ps_enable_data = ltr579_i2c_read_reg(LTR579_MAIN_CTRL);

        interrupt_test = ltr579_i2c_read_reg(LTR579_INT_CFG);
        APS_ERR("added by fully  interrupt_test: %d, ps_enable:%d .\n", interrupt_test, ps_enable_data);
        if(interrupt_test != 0x01){
            ltr579_i2c_write_reg(LTR579_MAIN_CTRL, 0x00); 
            ltr579_i2c_write_reg(LTR579_INT_CFG, 0x01); 
            ltr579_i2c_write_reg(LTR579_MAIN_CTRL, ps_enable_data); 
        }

        if(test_bit(CMC_BIT_PS,  &ltr579_obj->enable)){
            if((ps_enable_data & 0x01) != 0x01){
                ps_enable_data |= 0x01;
                ltr579_i2c_write_reg(LTR579_MAIN_CTRL, ps_enable_data);
            }
        }

    }	


    return 0;

}
/*--------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------*/
static int ltr579_just_for_init(void)
{
    int res;
    int init_als_gain;
    u8 databuf[2];
    struct i2c_client *client = ltr579_obj->client;

    struct ltr579_priv *obj = ltr579_obj;

    APS_ERR("added by fully.\n");
    res = ltr579_i2c_write_reg(LTR579_PS_PULSES, 32); //32pulses 
    if (res<0)
    {
        APS_LOG("ltr579_init_client() PS Pulses error...\n");
        goto EXIT_ERR;
    }
    res = ltr579_i2c_write_reg(LTR579_PS_LED, 0x36); // 60khz & 100mA 
    if (res<0)
    {
        APS_LOG("ltr579_init_client() PS LED error...\n");
        goto EXIT_ERR;
    }
    res = ltr579_i2c_write_reg(LTR579_PS_MEAS_RATE, 0x5C); // 11bits & 50ms time 
    if (res<0)
    {
        APS_LOG("ltr579_init_client() PS time error...\n");
        goto EXIT_ERR;
    }

    /*for interrup work mode support */
    if (0 == obj->hw->polling_mode_ps)
    {
        ltr579_ps_set_thres();

        databuf[0] = LTR579_INT_CFG;
        databuf[1] = 0x01;
        res = i2c_master_send(client, databuf, 0x2);
        if (res <= 0)
        {
            goto EXIT_ERR;			
        }

        databuf[0] = LTR579_INT_PST;
        databuf[1] = 0x02;
        res = i2c_master_send(client, databuf, 0x2);
        if (res <= 0)
        {
            goto EXIT_ERR;
        }
    }


    // Enable ALS to Full Range at startup
    init_als_gain = ALS_RANGE_3;
    als_gainrange = init_als_gain;//Set global variable
    APS_ERR("ALS sensor gainrange %d!\n", init_als_gain);

    switch (als_gainrange)
    {
        case ALS_RANGE_1:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range1);
            break;

        case ALS_RANGE_3:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range3);
            break;

        case ALS_RANGE_6:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range6);
            break;

        case ALS_RANGE_9:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range9);
            break;

        case ALS_RANGE_18:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range18);
            break;

        default:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range3);
            break;
    }


    return 0;

EXIT_ERR:
    APS_ERR("init dev: %d\n", res);
    return 1;
}
/*--------------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------------*/
static int ltr579_init_client(void)
{
    int res;
    int init_als_gain;
    u8 databuf[2];

    struct i2c_client *client = ltr579_obj->client;

    struct ltr579_priv *obj = ltr579_obj;

    //mdelay(PON_DELAY);

    /* ===============
     * ** IMPORTANT **
     * ===============
     * Other settings like timing and threshold to be set here, if required.
     * Not set and kept as device default for now.
     */
    res = ltr579_i2c_write_reg(LTR579_PS_PULSES, 32); //32pulses 
    if (res<0)
    {
        APS_LOG("ltr579_init_client() PS Pulses error...\n");
        goto EXIT_ERR;
    }
    res = ltr579_i2c_write_reg(LTR579_PS_LED, 0x36); // 60khz & 100mA 
    if (res<0)
    {
        APS_LOG("ltr579_init_client() PS LED error...\n");
        goto EXIT_ERR;
    }
    res = ltr579_i2c_write_reg(LTR579_PS_MEAS_RATE, 0x5C); // 11bits & 50ms time 
    if (res<0)
    {
        APS_LOG("ltr579_init_client() PS time error...\n");
        goto EXIT_ERR;
    }

    /*for interrup work mode support */
    if (0 == obj->hw->polling_mode_ps)
    {
        ltr579_ps_set_thres();

        databuf[0] = LTR579_INT_CFG;
        databuf[1] = 0x01;
        res = i2c_master_send(client, databuf, 0x2);
        if (res <= 0)
        {
            goto EXIT_ERR;			
        }

        databuf[0] = LTR579_INT_PST;
        databuf[1] = 0x02;
        res = i2c_master_send(client, databuf, 0x2);
        if (res <= 0)
        {
            goto EXIT_ERR;
        }
    }

#if 0
    res = ltr579_ps_enable(client, 1);
    if (res < 0)
    {
        APS_ERR("enable ps fail: %d\n", res);
        goto EXIT_ERR;
    }
#endif	

    // Enable ALS to Full Range at startup
    init_als_gain = ALS_RANGE_3;
    als_gainrange = init_als_gain;//Set global variable
    APS_ERR("ALS sensor gainrange %d!\n", init_als_gain);

    switch (als_gainrange)
    {
        case ALS_RANGE_1:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range1);
            break;

        case ALS_RANGE_3:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range3);
            break;

        case ALS_RANGE_6:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range6);
            break;

        case ALS_RANGE_9:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range9);
            break;

        case ALS_RANGE_18:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range18);
            break;

        default:
            res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range3);
            break;
    }

    res = ltr579_i2c_write_reg(LTR579_ALS_MEAS_RATE, ALS_RESO_MEAS);// 18 bit & 100ms measurement rate		
    APS_ERR("ALS sensor resolution & measurement rate: %d!\n", ALS_RESO_MEAS);

#if 0
    res = ltr579_als_enable(client, 1);
    if (res < 0)
    {
        APS_ERR("enable als fail: %d\n", res);
        goto EXIT_ERR;
    }
#endif	

    if ((res = ltr579_setup_eint(client)) != 0)
    {
        APS_ERR("setup eint: %d\n", res);
        goto EXIT_ERR;
    }
#if 0
    if ((res = ltr579_check_and_clear_intr(client)))
    {
        APS_ERR("check/clear intr: %d\n", res);
        goto EXIT_ERR;
    }
#endif
    return 0;

EXIT_ERR:
    APS_ERR("init dev: %d\n", res);
    return 1;
}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    int res = 0;
    APS_LOG("ltr579_obj als enable value = %d\n", en);

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return -1;
    }

    mutex_lock(&ltr579_mutex);
    if (en)
        set_bit(CMC_BIT_ALS, &ltr579_obj->enable);
    else
        clear_bit(CMC_BIT_ALS, &ltr579_obj->enable);
    mutex_unlock(&ltr579_mutex);

    if(PS_enable_flag==0)
    {
        res = ltr579_als_enable(ltr579_obj->client, en);
        if (res) {
            APS_ERR("als_enable_nodata is failed!!\n");
            return -1;
        }
    }
    return 0;
}

static int als_set_delay(u64 ns)
{
    // Do nothing
    return 0;
}

static int als_get_data(int* value, int* status)
{
    int err = 0;

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return -1;
    }

    ltr579_obj->als = ltr579_als_read(ltr579_obj->client, &ltr579_obj->als);
    if (ltr579_obj->als < 0)
        err = -1;
    else {
        *value = ltr579_get_als_value(ltr579_obj, ltr579_obj->als);
        if (*value < 0)
            err = -1;
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    return err;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
    int res = 0;
    APS_LOG("ltr579_obj ps enable value = %d\n", en);

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return -1;
    }

    mutex_lock(&ltr579_mutex);
    if (en)
        set_bit(CMC_BIT_PS, &ltr579_obj->enable);
    else
        clear_bit(CMC_BIT_PS, &ltr579_obj->enable);
    mutex_unlock(&ltr579_mutex);

    res = ltr579_ps_enable(ltr579_obj->client, en);
    if (res < 0) {
        APS_ERR("als_enable_nodata is failed!!\n");
        return -1;
    }
    return 0;
}

static int ps_set_delay(u64 ns)
{
    // Do nothing
    return 0;
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;

    if(!ltr579_obj)
    {
        APS_ERR("ltr579_obj is null!!\n");
        return -1;
    }

    ltr579_obj->ps = ltr579_ps_read(ltr579_obj->client, &ltr579_obj->ps);
    if (ltr579_obj->ps < 0)
        err = -1;
    else {
        *value = ltr579_get_ps_value(ltr579_obj, ltr579_obj->ps);
        if (*value < 0)
            err = -1;
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    return err;
}
/*-----------------------------------------------------------------------------------*/

/*-----------------------------------i2c operations----------------------------------*/
static int ltr579_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ltr579_priv *obj = NULL;
    struct als_control_path als_ctl={0};
    struct als_data_path als_data={0};
    struct ps_control_path ps_ctl={0};
    struct ps_data_path ps_data={0};
    int err = 0;

    APS_FUN();

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
    memset(obj, 0, sizeof(*obj));
    ltr579_obj = obj;

    obj->hw = hw;
    INIT_WORK(&obj->eint_work, ltr579_eint_work);	
    obj->client = client;
    i2c_set_clientdata(client, obj);	

    /*-----------------------------value need to be confirmed-----------------------------------------*/
    atomic_set(&obj->als_debounce, 300);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 300);
    atomic_set(&obj->ps_deb_on, 0);
    atomic_set(&obj->ps_deb_end, 0);
    atomic_set(&obj->ps_mask, 0);
    atomic_set(&obj->als_suspend, 0);
    //atomic_set(&obj->als_cmd_val, 0xDF);
    //atomic_set(&obj->ps_cmd_val,  0xC1);
    atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
    atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
    atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
    atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
    atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);

    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek,als_ps");

    obj->enable = 0;
    obj->pending_intr = 0;
    obj->ps_cali = 0;
    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
    //(400)/16*2.72 here is amplify *100	
    /*-----------------------------value need to be confirmed-----------------------------------------*/

    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
    atomic_set(&obj->i2c_retry, 3);
    //set_bit(CMC_BIT_ALS, &obj->enable);
    //set_bit(CMC_BIT_PS, &obj->enable);

    APS_LOG("ltr579_init_client() start...!\n");
    ltr579_i2c_client = client;
    err = ltr579_init_client();
    if(err)
    {
        goto exit_init_failed;
    }
    APS_LOG("ltr579_init_client() OK!\n");

    err = misc_register(&ltr579_device);
    if(err)
    {
        APS_ERR("ltr579_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    als_ctl.is_use_common_factory =false;
    ps_ctl.is_use_common_factory = false;

    /*------------------------ltr579 attribute file for debug--------------------------------------*/
    err = ltr579_create_attr(&(ltr579_init_info.platform_diver_addr->driver));
    //err = ltr579_create_attr(&(ltr579_i2c_driver.driver));
    if(err)
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    /*------------------------ltr579 attribute file for debug--------------------------------------*/

    als_ctl.open_report_data= als_open_report_data;
    als_ctl.enable_nodata = als_enable_nodata;
    als_ctl.set_delay  = als_set_delay;
    als_ctl.is_report_input_direct = false;
    als_ctl.is_support_batch = false;

    err = als_register_control_path(&als_ctl);
    if(err)
    {
        APS_ERR("register fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }

    als_data.get_data = als_get_data;
    als_data.vender_div = 100;
    err = als_register_data_path(&als_data);	
    if(err)
    {
        APS_ERR("register fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }

    ps_ctl.open_report_data= ps_open_report_data;
    ps_ctl.enable_nodata = ps_enable_nodata;
    ps_ctl.set_delay  = ps_set_delay;
    ps_ctl.is_report_input_direct = false;
    ps_ctl.is_support_batch = false;

    err = ps_register_control_path(&ps_ctl);
    if(err)
    {
        APS_ERR("register fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }

    ps_data.get_data = ps_get_data;
    ps_data.vender_div = 100;
    err = ps_register_data_path(&ps_data);	
    if(err)
    {
        APS_ERR("tregister fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }
/*
    err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 1, 0);
    if(err)
    {
        APS_ERR("register light batch support err = %d\n", err);
    }

    err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 1, 0);
    if(err)
    {
        APS_ERR("register proximity batch support err = %d\n", err);
    }
*/
    ltr579_init_flag =0;
    APS_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
    misc_deregister(&ltr579_device);
exit_init_failed:
    kfree(obj);
exit:
    ltr579_i2c_client = NULL;           
    APS_ERR("%s: err = %d\n", __func__, err);
    ltr579_init_flag =-1;
    return err;
}

static int ltr579_i2c_remove(struct i2c_client *client)
{
    int err;

    err = ltr579_delete_attr(&(ltr579_init_info.platform_diver_addr->driver));
    if(err)
    {
        APS_ERR("ltr579_delete_attr fail: %d\n", err);
    }

    misc_deregister(&ltr579_device);

    ltr579_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}

static int ltr579_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, LTR579_DEV_NAME);
    return 0;
}

static int ltr579_i2c_suspend(struct device *dev) 
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ltr579_priv *obj = i2c_get_clientdata(client);    
    int err;
    APS_FUN();    

    ltr579_just_for_reset();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    /*added by fully  for test whether ps is enable or not 20160720*/
    if(test_bit(CMC_BIT_PS, &obj->enable)){
        return 0;
    }	

    atomic_set(&obj->als_suspend, 1);
    err = ltr579_als_enable(obj->client, 0);
    if(err < 0)
    {
        APS_ERR("disable als: %d\n", err);
        return err;
    }

#if 0
    atomic_set(&obj->ps_suspend, 1);
    err = ltr579_ps_enable(obj->client, 0);
    if(err < 0)
    {
        APS_ERR("disable ps:  %d\n", err);
        return err;
    }

    ltr579_power(obj->hw, 0);
#endif

    return 0;
}

static int ltr579_i2c_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ltr579_priv *obj = i2c_get_clientdata(client);        
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    ltr579_just_for_reset();

    /*added by fully  for test whether ps is enable or not 20160720*/
    if(test_bit(CMC_BIT_PS, &obj->enable)){
        return 0;
    }	

    ltr579_power(obj->hw, 1);

    atomic_set(&obj->als_suspend, 0);
    if(test_bit(CMC_BIT_ALS, &obj->enable))
    {
        err = ltr579_als_enable(obj->client, 1);
        if (err < 0)
        {
            APS_ERR("enable als fail: %d\n", err);        
        }
    }

#if 0
    atomic_set(&obj->ps_suspend, 0);
    if(test_bit(CMC_BIT_PS,  &obj->enable))
    {
        err = ltr579_ps_enable(obj->client, 1);
        if (err < 0)
        {
            APS_ERR("enable ps fail: %d\n", err);                
        }
    }
#endif	

    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int ltr579_remove(void)
{
    APS_FUN();

    ltr579_power(hw, 0);	
    i2c_del_driver(&ltr579_i2c_driver);

    return 0;
}
/*----------------------------------------------------------------------------*/
static int  ltr579_local_init(void)
{
    APS_FUN();

    ltr579_power(hw, 1);

    if(i2c_add_driver(&ltr579_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }

    if(-1 == ltr579_init_flag)
    {
        return -1;
    }

    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init ltr579_init(void)
{

    //const char *name = "mediatek,ltr579";
    APS_FUN();
   /* hw = get_alsps_dts_func(name, hw);
    if (!hw)
        APS_ERR("get dts info fail\n");
*/
    alsps_driver_add(&ltr579_init_info);
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr579_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(ltr579_init);
module_exit(ltr579_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liteon");
MODULE_DESCRIPTION("LTR-579ALSPS Driver");
MODULE_LICENSE("GPL");

