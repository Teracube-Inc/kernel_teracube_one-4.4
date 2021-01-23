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
#include "ltr578.h"
#include "alsps.h"

#define GN_MTK_BSP_PS_DYNAMIC_CALI
//#define DELAYED_PS_CALI
//#define DEMO_BOARD
//#define LTR578_DEBUG
//#define SENSOR_DEFAULT_ENABLED
#define UPDATE_PS_THRESHOLD
#define CHECK_FAIL_WORK
#define LTR577_NEARSTATE


/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define LTR578_DEV_NAME			"ltr578"

/*----------------------------------------------------------------------------*/
#define APS_TAG					"[ALS/PS] "
#define APS_FUN(f)              printk(KERN_INFO 	APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)   printk(KERN_ERR  	APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)   printk(KERN_NOTICE	APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   printk(KERN_ERR 	APS_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr578_i2c_id[] = {{LTR578_DEV_NAME,0},{}};
static unsigned long long int_top_time;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

/*----------------------------------------------------------------------------*/
static int ltr578_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr578_i2c_remove(struct i2c_client *client);
static int ltr578_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ltr578_i2c_suspend(struct device *dev);
static int ltr578_i2c_resume(struct device *dev);

#ifdef LTR578_DEBUG
static int ltr578_dump_reg(void);
#endif

//static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr578_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;
	struct delayed_work check_ps_work;
	struct delayed_work cali_ps_work;

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
static int sensor_part_id = 0;

static struct ltr578_priv *ltr578_obj = NULL;
static struct i2c_client *ltr578_i2c_client = NULL;
static int ltr578_get_ps_value(struct ltr578_priv *obj, u16 ps);
static int ltr578_ps_read(struct i2c_client *client, u16 *data);

static DEFINE_MUTEX(ltr578_mutex);
static DEFINE_MUTEX(ltr578_i2c_mutex);

static int ltr578_local_init(void);
static int ltr578_remove(void);
static int ltr578_init_flag =-1; // 0<==>OK -1 <==> fail

static int ps_enabled = 0;
static int als_enabled = 0;

//static int irq_enabled = 0;

static struct alsps_init_info ltr578_init_info = {
		.name = "ltr578",
		.init = ltr578_local_init,
		.uninit = ltr578_remove,	
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltr578_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr578_i2c_suspend, ltr578_i2c_resume)
};
#endif

static struct i2c_driver ltr578_i2c_driver = {	
	.probe      = ltr578_i2c_probe,
	.remove     = ltr578_i2c_remove,
	.detect     = ltr578_i2c_detect,
	.id_table   = ltr578_i2c_id,
	.driver = {
		.name           = LTR578_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif

#ifdef CONFIG_PM_SLEEP
		.pm = &ltr578_pm_ops,
#endif
	},
};

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr578_dynamic_calibrate(void);
static int dynamic_calibrate = 2047;
#endif
/*-----------------------------------------------------------------------------*/

/* 
 * #########
 * ## I2C ##
 * #########
 */
static int ltr578_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err;
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &beg, },
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data, }
	};

	mutex_lock(&ltr578_i2c_mutex);
	if (!client) {
		mutex_unlock(&ltr578_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&ltr578_i2c_mutex);
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	mutex_unlock(&ltr578_i2c_mutex);
	if (err != 2) {
		APS_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else {
		err = 0;	/*no error */
	}
	return err;
}

static int ltr578_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&ltr578_i2c_mutex);
	if (!client) {
		mutex_unlock(&ltr578_i2c_mutex);
		return -EINVAL;
	}
	else if (len >= C_I2C_FIFO_SIZE) {
		APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&ltr578_i2c_mutex);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++) {
		buf[num++] = data[idx];
	}

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		APS_ERR("send command error!!\n");
		mutex_unlock(&ltr578_i2c_mutex);
		return -EFAULT;
	}
	mutex_unlock(&ltr578_i2c_mutex);
	return err;
}

/*----------------------------------------------------------------------------*/
static int ltr578_master_recv(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltr578_obj->trace);
	int max_try = atomic_read(&ltr578_obj->i2c_retry);

	while (retry++ < max_try) {
		ret = ltr578_i2c_read_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000)) {
			APS_LOG("(recv) %d/%d\n", retry - 1, max_try);
		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/*----------------------------------------------------------------------------*/
static int ltr578_master_send(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltr578_obj->trace);
	int max_try = atomic_read(&ltr578_obj->i2c_retry);

	while (retry++ < max_try) {
		ret = ltr578_i2c_write_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000)) {
			APS_LOG("(send) %d/%d\n", retry - 1, max_try);
		}
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/*----------------------------------------------------------------------------*/
static void ltr578_power(struct alsps_hw *hw, unsigned int on) 
{
#ifdef DEMO_BOARD
	static unsigned int power_on = 0;	

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "ltr578")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "ltr578")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
#endif
}
/********************************************************************/
/* 
 * ###############
 * ## PS CONFIG ##
 * ###############

 */
static int ltr578_ps_set_thres(void)
{
	int res;
	u8 databuf[2];
	
	struct i2c_client *client = ltr578_obj->client;
	struct ltr578_priv *obj = ltr578_obj;

	APS_FUN();

	APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);

	if(1 == ps_cali.valid)
	{
		databuf[0] = LTR578_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{			
			goto EXIT_ERR;
		}
		databuf[0] = LTR578_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR578_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR578_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
        ps_cali.valid = 0;
	}
	else
	{
		databuf[0] = LTR578_PS_THRES_LOW_0; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR578_PS_THRES_LOW_1; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR578_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR578_PS_THRES_UP_1;	
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
	res = LTR578_ERR_I2C;
	return res;
}

#ifdef CHECK_FAIL_WORK

/* I2C Read */
static int ltr578_i2c_read_reg(u8 regnum)
{
	u8 buffer[1], reg_value[1];
	int res = 0;

	mutex_lock(&ltr578_i2c_mutex);						   
	buffer[0] = regnum;
	res = i2c_master_send(ltr578_obj->client, buffer, 0x1);
	if (res <= 0)	{
	APS_ERR("read reg send res = %d\n", res);
	mutex_unlock(&ltr578_i2c_mutex);							 
	return res;
	}
	res = i2c_master_recv(ltr578_obj->client, reg_value, 0x1);
	if (res <= 0) {
	APS_ERR("read reg recv res = %d\n", res);
	mutex_unlock(&ltr578_i2c_mutex);						 
	return res;
	}
	mutex_unlock(&ltr578_i2c_mutex);							 
	return reg_value[0];
}

/* I2C Write */
static int ltr578_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];
	int res = 0;

	mutex_lock(&ltr578_i2c_mutex);						   
	databuf[0] = regnum;
	databuf[1] = value;
	res = i2c_master_send(ltr578_obj->client, databuf, 0x2);
	mutex_unlock(&ltr578_i2c_mutex);
	if (res < 0) {
	APS_ERR("wirte reg send res = %d\n", res);
	return res;
	}

	else
	return 0;
}


static void ltr578_init_reg(void)
{
	ltr578_i2c_write_reg(LTR578_MAIN_CTRL, 0x00);
	if(sensor_part_id){
		ltr578_i2c_write_reg(LTR578_PS_MEAS_RATE, 0x1C); /* ltr577 11bits & 50ms time */
	}else{
		ltr578_i2c_write_reg(LTR578_PS_MEAS_RATE, 0x5C); /* ltr578 11bits & 50ms time */
	}
	ltr578_i2c_write_reg(LTR578_INT_CFG, 0x01);
	ltr578_i2c_write_reg(LTR578_INT_PST, 0x02);
	ltr578_i2c_write_reg(LTR578_PS_PULSES, 0x0E);
	ltr578_i2c_write_reg(LTR578_PS_LED, 0x36);

	mdelay(WAKEUP_DELAY);
}

struct delayed_work dwork;// add delaywork for check 0x00 status
static u16 g_ltr578_error_check_delay=500;
static u32 g_ltr578_error_cnt=0;
static void ltr578_delay_work(struct work_struct *work)
{
    int status1 = 0;
	int status2 = 0;
	int status3 = 0;
	int err;

#ifdef LTR577_NEARSTATE
	struct hwm_sensor_data sensor_data;
	struct ltr578_priv *obj = ltr578_obj;
	int value = 1;
#endif
	
	APS_FUN();
	status1 = ltr578_i2c_read_reg(LTR578_MAIN_CTRL);
	status2 = ltr578_i2c_read_reg(LTR578_INT_CFG);	
	status3 = ltr578_i2c_read_reg(LTR578_PS_LED);	

	APS_LOG("ltr578_delay_work LTR578_MAIN_CTRL = 0x%x LTR578_INT_CFG = 0x%x LTR578_PS_LED = 0x%x \n", status1,status2,status3);

	if(((status1 & 0x01) == 0x00) || ((status2 & 0x01) == 0x00) || ((status3 != 0x36)))
	{
		g_ltr578_error_cnt++; // calc the fail times

		ltr578_init_reg();

		ltr578_i2c_write_reg(LTR578_MAIN_CTRL,((status1 | 0x03) & 0X03) );	// reset the main ctrl
		
		err = ltr578_dynamic_calibrate();
		if (err < 0)
		{
			APS_LOG("ltr578_dynamic_calibrate() failed\n");
		}
		ltr578_ps_set_thres();
		APS_LOG("ltr578_delay_work g_ltr578_error_cnt = %d \n", g_ltr578_error_cnt);


		 mdelay(WAKEUP_DELAY);

		if(g_ltr578_error_cnt>1000)
			g_ltr578_error_cnt=0;
	}

#ifdef LTR577_NEARSTATE
	/*added by fully 20180413*/

	if(intr_flag_value == 1){   //for the near state 20180413
		APS_DBG("added by fully : rawdata intr_flag_value=%d!\n",intr_flag_value);

		obj->ps = ltr578_ps_read(obj->client, &obj->ps);
		if (obj->ps < 0)
		{
			return ;
		}
				
		APS_DBG(" added by fully ltr578_eint_work: rawdata ps=%d!\n",obj->ps);
		
		value = ltr578_get_ps_value(obj, obj->ps);
		if(value == 1){
			sensor_data.values[0] =value;
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

			if (ps_report_interrupt_data(sensor_data.values[0]))
			{
				APS_ERR("call ps_report_interrupt_data fail \n");
			}
			
			APS_DBG(" added by fully ltr578_eint_work: value=%d!\n",value);
		}
	}
#endif	

	schedule_delayed_work(&dwork,msecs_to_jiffies(g_ltr578_error_check_delay));
	
}

static void ltr578_enable_delay_work(u8 enable)
{
	if (enable) 
	{
		schedule_delayed_work(&dwork,msecs_to_jiffies(g_ltr578_error_check_delay));
	} 
	else 
	{
		cancel_delayed_work_sync(&dwork);
	}
}

#endif



static int ltr578_ps_enable(struct i2c_client *client, int enable)
{
	u8 regdata;
	int err;

	APS_LOG("ltr578_ps_enable() ...start!\n");

	if (enable != 0 && ps_enabled == 1)
	{
		APS_LOG("PS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && ps_enabled == 0)
	{
		APS_LOG("PS: Already disabled \n");
		return 0;
	}

	err = ltr578_master_recv(client, LTR578_MAIN_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}

	regdata &= 0x0F;	// Clear reset bit
	
	if (enable != 0) {
		APS_LOG("PS: enable ps only \n");
		regdata = 0x03;
	}
	else {
		APS_LOG("PS: disable ps only \n");
		regdata &= 0x0E;
	}

	err = ltr578_master_send(client, LTR578_MAIN_CTRL, (char *)&regdata, 1);
	if (err < 0)
	{
		APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
		return err;
	}
	mdelay(WAKEUP_DELAY);
	err = ltr578_master_recv(client, LTR578_MAIN_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}
	
	if (0 == ltr578_obj->hw->polling_mode_ps && enable != 0)
	{
#ifndef DELAYED_PS_CALI
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
		err = ltr578_dynamic_calibrate();
		if (err < 0)
		{
			APS_LOG("ltr578_dynamic_calibrate() failed\n");
		}
#endif
		ltr578_ps_set_thres();
#else
		cancel_delayed_work(&ltr578_obj->cali_ps_work);
		schedule_delayed_work(&ltr578_obj->cali_ps_work, msecs_to_jiffies(200));
#endif
	}
	else if (0 == ltr578_obj->hw->polling_mode_ps && enable == 0)
	{
		//cancel_work_sync(&ltr578_obj->eint_work);		
	}

	if (enable != 0)
		ps_enabled = 1;
	else
		ps_enabled = 0;

//	if ((irq_enabled == 1) && (enable != 0))
	{
	//	irq_enabled = 2;
	}


	
#ifdef CHECK_FAIL_WORK

if(enable != 0)
	
		ltr578_enable_delay_work(1);
else
		ltr578_enable_delay_work(0);

	
#endif

	return err;
}

/********************************************************************/
static int ltr578_ps_read(struct i2c_client *client, u16 *data)
{
	int psdata, ret = 0;
	u8 buf[2];

	ret = ltr578_master_recv(client, LTR578_PS_DATA_0, buf, 0x02);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
		return ret;
	}

	APS_DBG("ps_rawdata_lo = %d\n", buf[0]);	
    APS_DBG("ps_rawdata_hi = %d\n", buf[1]);	
	
	psdata = ((buf[1] & 0x07) << 8) | (buf[0]);
	*data = psdata;
    APS_DBG("ltr578_ps_read: ps_rawdata = %d\n", psdata);
	
	final_prox_val = psdata;	
	return psdata;
}

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr578_dynamic_calibrate(void)
{
	int i = 0;
	int data;
	int data_total = 0;
	int noise = 0;
	int count = 5;
	int ps_thd_val_low, ps_thd_val_high;
	struct ltr578_priv *obj = ltr578_obj;

	if (!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return -1;
	}	

	for (i = 0; i < count; i++) {
		// wait for ps value be stable
		msleep(51);

		data = ltr578_ps_read(ltr578_obj->client, &ltr578_obj->ps);
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

	
	APS_LOG("%s:noise001 = %d\n", __func__, noise);

	if(noise < dynamic_calibrate + 600){
			dynamic_calibrate = noise;
			if(sensor_part_id)
			{
				if (noise < 100) {
					ps_thd_val_high = noise + 70;
					ps_thd_val_low  = noise + 40;
				}else if (noise < 200) {
					ps_thd_val_high = noise + 150;
					ps_thd_val_low  = noise + 60;
				}else if (noise < 300) {
					ps_thd_val_high = noise + 150;
					ps_thd_val_low  = noise + 60;
				}else if (noise < 400) {
					ps_thd_val_high = noise + 150;
					ps_thd_val_low  = noise + 60;
				}else if (noise < 1000) {
					ps_thd_val_high = noise + 180;
					ps_thd_val_low  = noise + 90;
				}else if (noise < 1600) {
					ps_thd_val_high = noise + 300;
					ps_thd_val_low  = noise + 180;
				}else {
					ps_thd_val_high = 2046;
					ps_thd_val_low  = 1800;
				}
			}
			else
			{
				if (noise < 100) {
					ps_thd_val_high = noise + 70;
					ps_thd_val_low	= noise + 40;
				}else if (noise < 200) {
					ps_thd_val_high = noise + 150;
					ps_thd_val_low	= noise + 60;
				}else if (noise < 300) {
					ps_thd_val_high = noise + 150;
					ps_thd_val_low	= noise + 60;
				}else if (noise < 400) {
					ps_thd_val_high = noise + 150;
					ps_thd_val_low	= noise + 60;
				}else if (noise < 1000) {
					ps_thd_val_high = noise + 180;
					ps_thd_val_low	= noise + 90;
				}else if (noise < 1600) {
					ps_thd_val_high = noise + 300;
					ps_thd_val_low	= noise + 180;
				}else {
					ps_thd_val_high = 2046;
					ps_thd_val_low	= 1800;
				}
			}
			atomic_set(&obj->ps_thd_val_high, ps_thd_val_high);
			atomic_set(&obj->ps_thd_val_low, ps_thd_val_low);
    ps_cali.valid = 1;
    ps_cali.far_away = ps_thd_val_low;
    ps_cali.close = ps_thd_val_high;
	
	
	APS_LOG("%s:noise002 = %d\n", __func__, noise);
	APS_LOG("%s:obj->ps_thd_val_high = %d\n", __func__, ps_thd_val_high);
	APS_LOG("%s:obj->ps_thd_val_low = %d\n", __func__, ps_thd_val_low);
	
	}

	return 0;
}
#endif
/********************************************************************/
/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr578_als_enable(struct i2c_client *client, int enable)
{
	int err = 0;
	u8 regdata = 0;

	if (enable != 0 && als_enabled == 1)
	{
		APS_LOG("ALS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && als_enabled == 0)
	{
		APS_LOG("ALS: Already disabled \n");
		return 0;
	}
	if ((ps_enabled == 1) && (als_enabled == 0))
	{
		APS_LOG("ALS: PS enabled, do nothing \n");
		return 0;
	}
	err = ltr578_master_recv(client, LTR578_MAIN_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}

	regdata &= 0x0F;	// Clear reset bit
	
	if (enable != 0) {
		APS_LOG("ALS(1): enable als only \n");
		regdata |= 0x02;
	}else{
		APS_LOG("ALS(1): disable als only \n");
		regdata &= 0xFD;
	}

	err = ltr578_master_send(client, LTR578_MAIN_CTRL, (char *)&regdata, 1);
	if (err < 0)
	{
		APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
		return err;
	}

	mdelay(WAKEUP_DELAY);

	if (enable != 0)
		als_enabled = 1;
	else
		als_enabled = 0;

	return 0;
}

static int ltr578_als_read(struct i2c_client *client, u16* data)
{
	int alsval = 0, clearval = 0;
	int luxdata_int;	
	u8 buf[3];
	int ret;

	ret = ltr578_master_recv(client, LTR578_ALS_DATA_0, buf, 0x03);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	alsval = (buf[2] * 256 * 256) + (buf[1] * 256) + buf[0];
	APS_DBG("alsval_0 = %d,alsval_1=%d,alsval_2=%d,alsval=%d\n", buf[0], buf[1], buf[2], alsval);	

	ret = ltr578_master_recv(client, LTR578_CLEAR_DATA_0, buf, 0x03);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	clearval = (buf[2] * 256 * 256) + (buf[1] * 256) + buf[0];
	APS_DBG("clearval_0 = %d,clearval_1=%d,clearval_2=%d,clearval=%d\n", buf[0], buf[1], buf[2], clearval);

	if (alsval == 0)
	{
		luxdata_int = 0;
		goto out;
	}

	if (ALS_USE_CLEAR_DATA == 1)
	{
		// ALS_Lux = ALS_DATA * 0.8 * WINFAC1 * (1 – WINFAC2 * CLEAR_DATA / ALS_DATA) / ALS_GAIN / ALS_INT
#if 0
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR * (1 - ALS_WIN_FACTOR2 * clearval / alsval / 1000) / als_gainrange / 10;
#else	
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR * (1 - ALS_WIN_FACTOR2 * clearval / alsval / 1000) / 20;
#endif
	}
	else
	{
#if 0
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR / als_gainrange / 10;//formula: ALS counts * 0.8/gain/int , int=1	
#else	
	if(sensor_part_id)
	{
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR / 20;
		APS_DBG("LTR_577_luxdata_int=%d\n", luxdata_int);
	}
	else
	{
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR / 20;
		APS_DBG("LTR_578_luxdata_int=%d\n", luxdata_int);
	}

#endif
	}	
	APS_DBG("ltr578_als_read: als_value_lux = %d\n", luxdata_int);
out:
	*data = luxdata_int;
	final_lux_val = luxdata_int;
	return luxdata_int;
}
/********************************************************************/
static int ltr578_get_ps_value(struct ltr578_priv *obj, u16 ps)
{
	int val = 1;
	u8 buffer = 0;
	int ps_flag;
	int ret;
	static int val_temp = 1;

#if 1	
	if(sensor_part_id)
	{

		if((ps > atomic_read(&obj->ps_thd_val_high)))
		{
			val = 0;  /*close*/
			val_temp = 0;
			intr_flag_value = 1;
		}
		else if((ps < atomic_read(&obj->ps_thd_val_low)))
		{
			val = 1;  /*far away*/
			val_temp = 1;
			intr_flag_value = 0;
		}
		else
			val = val_temp;
			APS_DBG("ltr577_get_ps_val=%d\n", val);
	}
	else
#endif

	{
		ret = ltr578_master_recv(ltr578_obj->client, LTR578_MAIN_STATUS, &buffer, 0x01);
		if (ret < 0) 
		{
			APS_DBG("i2c error: %d\n", ret);
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
		APS_DBG("ltr578_get_ps_val=%d\n", val);
	}

	return val;
}
/********************************************************************/
#if 0
static int ltr578_get_als_value(struct ltr578_priv *obj, u16 als)
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
		//return als;
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
#endif
/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ltr578_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&ltr578_obj->i2c_retry), atomic_read(&ltr578_obj->als_debounce), 
		atomic_read(&ltr578_obj->ps_mask), atomic_read(&ltr578_obj->ps_thd_val), atomic_read(&ltr578_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&ltr578_obj->i2c_retry, retry);
		atomic_set(&ltr578_obj->als_debounce, als_deb);
		atomic_set(&ltr578_obj->ps_mask, mask);
		atomic_set(&ltr578_obj->ps_thd_val, thres);        
		atomic_set(&ltr578_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr578_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&ltr578_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_show_als(struct device_driver *ddri, char *buf)
{
	int res;
		
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	res = ltr578_als_read(ltr578_obj->client, &ltr578_obj->als);
	return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);
	
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	res = ltr578_ps_read(ltr578_obj->client, &ltr578_obj->ps);
    return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr578_show_reg(struct device_driver *ddri, char *buf)
{
	int i,len=0;
	int reg[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0d,0x0e,0x0f,
			   0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26};
	int ret;
	u8 buffer;

	for(i=0;i<27;i++)
	{
		ret = ltr578_master_recv(ltr578_obj->client, reg[i], &buffer, 0x01);
		if (ret < 0) {
			APS_DBG("i2c error: %d\n", ret);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return len;
}

#ifdef LTR578_DEBUG
static int ltr578_dump_reg(void)
{
	int i=0;
	int reg[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0d,0x0e,0x0f,
		       0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26};
	int ret;
	u8 buffer;

	for(i=0;i<27;i++)
	{
		ret = ltr578_master_recv(ltr578_obj->client, reg[i], &buffer, 0x01);
		if (ret < 0) {
			APS_DBG("i2c error: %d\n", ret);
		}

		APS_DBG("reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t ltr578_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
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
static ssize_t ltr578_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
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
static ssize_t ltr578_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	
	if(ltr578_obj->hw)
	{	
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr578_obj->hw->i2c_num, ltr578_obj->hw->power_id, ltr578_obj->hw->power_vol);		
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr578_obj->als_suspend), atomic_read(&ltr578_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr578_priv *obj, const char* buf, size_t count, u32 data[], int len)
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
static ssize_t ltr578_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < ltr578_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr578_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr578_obj->als_level, ltr578_obj->hw->als_level, sizeof(ltr578_obj->als_level));
	}
	else if(ltr578_obj->als_level_num != read_int_from_buf(ltr578_obj, buf, count, 
			ltr578_obj->hw->als_level, ltr578_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < ltr578_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr578_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr578_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr578_obj->als_value, ltr578_obj->hw->als_value, sizeof(ltr578_obj->als_value));
	}
	else if(ltr578_obj->als_value_num != read_int_from_buf(ltr578_obj, buf, count, 
			ltr578_obj->hw->als_value, ltr578_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, ltr578_show_als,		NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, ltr578_show_ps,		NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, ltr578_show_config,	ltr578_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, ltr578_show_alslv,	ltr578_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, ltr578_show_alsval,	ltr578_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, ltr578_show_trace,	ltr578_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, ltr578_show_status,	NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, ltr578_show_send,	ltr578_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, ltr578_show_recv,	ltr578_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, ltr578_show_reg,		NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr578_attr_list[] = {
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
static int ltr578_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(ltr578_attr_list));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ltr578_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr578_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr578_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(ARRAY_SIZE(ltr578_attr_list));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ltr578_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/
static void ltr578_check_ps_work(struct work_struct *work)
{
	struct ltr578_priv *obj = ltr578_obj;	
	struct hwm_sensor_data sensor_data;
	APS_FUN();

	if (test_bit(CMC_BIT_PS, &obj->enable)) {
		ltr578_ps_read(obj->client, &obj->ps);
		APS_LOG("ltr578_check_ps_work rawdata ps=%d high=%d low=%d\n", obj->ps, atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));

		sensor_data.values[0] = ltr578_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

		if (ps_report_interrupt_data(sensor_data.values[0]))
		{
			APS_ERR("call ps_report_interrupt_data fail \n");
		}
	}

	return;
}

static void ltr578_cali_ps_work(struct work_struct *work)
{
	int err = 0;

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
	err = ltr578_dynamic_calibrate();
	if (err < 0)
	{
		APS_LOG("ltr578_dynamic_calibrate() failed\n");
	}
#endif		
	ltr578_ps_set_thres();
}
/*----------------------------------------------------------------------------*/
static void ltr578_eint_work(struct work_struct *work)
{
	struct ltr578_priv *obj = (struct ltr578_priv *)container_of(work, struct ltr578_priv, eint_work);
	int res = 0;
	int value = 1;
#ifdef UPDATE_PS_THRESHOLD
	u8 databuf[2];
#endif
			   
	//get raw data
	obj->ps = ltr578_ps_read(obj->client, &obj->ps);
	if (obj->ps < 0)
	{						  
		goto EXIT_INTR;
	}
				
	APS_DBG("ltr578_eint_work: rawdata ps=%d!\n",obj->ps);
	value = ltr578_get_ps_value(obj, obj->ps);
	APS_DBG("intr_flag_value=%d\n",intr_flag_value);
	if(intr_flag_value){
#ifdef UPDATE_PS_THRESHOLD
		databuf[0] = LTR578_PS_THRES_LOW_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{				   
			goto EXIT_INTR;
		}
		databuf[0] = LTR578_PS_THRES_LOW_1;	
		databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{			   
			goto EXIT_INTR;
		}
		databuf[0] = LTR578_PS_THRES_UP_0;	
		databuf[1] = (u8)(0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{					   
			goto EXIT_INTR;
		}
		databuf[0] = LTR578_PS_THRES_UP_1; 
		databuf[1] = (u8)((0xFF00) >> 8);;
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{					   
			goto EXIT_INTR;
		}
#endif
	}
	else{	
#ifndef DELAYED_PS_CALI
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
		if(obj->ps > 20 && obj->ps < (dynamic_calibrate - 50)){ 
                    if(sensor_part_id)
                    {
                        if(obj->ps < 100){
                            atomic_set(&obj->ps_thd_val_high,  obj->ps+70);
                            atomic_set(&obj->ps_thd_val_low, obj->ps+40);
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
                    }
                    else
                    {
                        if(obj->ps < 100){
                            atomic_set(&obj->ps_thd_val_high,  obj->ps+70);
                            atomic_set(&obj->ps_thd_val_low, obj->ps+40);
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
                    }
        	dynamic_calibrate = obj->ps;
        }	        
#endif
#ifdef UPDATE_PS_THRESHOLD
		databuf[0] = LTR578_PS_THRES_LOW_0;					  
		databuf[1] = (u8)(0 & 0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
		databuf[0] = LTR578_PS_THRES_LOW_1;						  
		databuf[1] = (u8)((0 & 0xFF00) >> 8);												   
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{					   
			goto EXIT_INTR;
		}
		databuf[0] = LTR578_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{		   
			goto EXIT_INTR;
		}
		databuf[0] = LTR578_PS_THRES_UP_1; 
		databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{		   
			goto EXIT_INTR;
		}
#endif
#else
		cancel_delayed_work(&ltr578_obj->cali_ps_work);
		schedule_delayed_work(&ltr578_obj->cali_ps_work, msecs_to_jiffies(2000));
#endif
	}
	//let up layer to know				  
	res = ps_report_interrupt_data(value);

EXIT_INTR:	
#ifdef CONFIG_OF
	enable_irq(obj->irq);
#endif
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void ltr578_eint_func(void)
{
	struct ltr578_priv *obj = ltr578_obj;
	if(!obj)
	{
		return;
	}	
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}
#ifdef CONFIG_OF
static irqreturn_t ltr578_eint_handler(int irq, void *desc)
{	
	//if (irq_enabled == 2)
	{
		disable_irq_nosync(ltr578_obj->irq);
		ltr578_eint_func();
	}

	return IRQ_HANDLED;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int ltr578_setup_eint(struct i2c_client *client)
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

	/* eint request */
	if (ltr578_obj->irq_node) {
		of_property_read_u32_array(ltr578_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		ltr578_obj->irq = irq_of_parse_and_map(ltr578_obj->irq_node, 0);
		APS_LOG("ltr578_obj->irq = %d\n", ltr578_obj->irq);
		if (!ltr578_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		
		if (request_irq(ltr578_obj->irq, ltr578_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq_wake(ltr578_obj->irq);
		//enable_irq(ltr578_obj->irq);
		//irq_enabled = 1;
	}
	else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return 0;
}
/**********************************************************************************************/

/*--------------------------------------------------------------------------------*/
static int ltr578_init_client(void)
{
	int res;
	int init_als_gain;
	u8 buf;

	struct i2c_client *client = ltr578_obj->client;

	struct ltr578_priv *obj = ltr578_obj;

	mdelay(PON_DELAY);

	/* ===============
	* ** IMPORTANT **
	* ===============
	* Other settings like timing and threshold to be set here, if required.
	* Not set and kept as device default for now.
	*/
	buf = 14; // 16 pulses	
	res = ltr578_master_send(client, LTR578_PS_PULSES, (char *)&buf, 1);	
	if (res<0)
	{
		APS_LOG("ltr578_init_client() PS Pulses error...\n");
		goto EXIT_ERR;
	}

	buf = 0x36;	// 60khz & 100mA 
	res = ltr578_master_send(client, LTR578_PS_LED, (char *)&buf, 1);	
	if (res<0)
	{
		APS_LOG("ltr578_init_client() PS LED error...\n");
		goto EXIT_ERR;
	}

	if(sensor_part_id){
		buf = 0x1C;	// 11bits & 50ms time
	}
	else{
		buf = 0x5C;	// 11bits & 50ms time
	}
	res = ltr578_master_send(client, LTR578_PS_MEAS_RATE, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr578_init_client() PS time error...\n");
		goto EXIT_ERR;
	}

	/*for interrup work mode support */
	if (0 == obj->hw->polling_mode_ps)
	{
		ltr578_ps_set_thres();

		buf = 0x01;
		res = ltr578_master_send(client, LTR578_INT_CFG, (char *)&buf, 1);		
		if (res < 0)
		{
			goto EXIT_ERR;			
		}

		buf = 0x02;
		res = ltr578_master_send(client, LTR578_INT_PST, (char *)&buf, 1);		
		if (res < 0)
		{
			goto EXIT_ERR;
		}
	}
	// Enable ALS to Full Range at startup
	init_als_gain = ALS_RANGE_18;
	als_gainrange = init_als_gain;//Set global variable
	APS_ERR("ALS sensor gainrange %d!\n", init_als_gain);

	switch (als_gainrange)
	{
	case ALS_RANGE_1:
		buf = MODE_ALS_Range1;
		res = ltr578_master_send(client, LTR578_ALS_GAIN, (char *)&buf, 1);		
		break;

	case ALS_RANGE_3:
		buf = MODE_ALS_Range3;
		res = ltr578_master_send(client, LTR578_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_6:
		buf = MODE_ALS_Range6;
		res = ltr578_master_send(client, LTR578_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_9:
		buf = MODE_ALS_Range9;
		res = ltr578_master_send(client, LTR578_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_18:
		buf = MODE_ALS_Range18;
		res = ltr578_master_send(client, LTR578_ALS_GAIN, (char *)&buf, 1);
		break;

	default:
		buf = MODE_ALS_Range3;
		res = ltr578_master_send(client, LTR578_ALS_GAIN, (char *)&buf, 1);
		break;
	}

	buf = ALS_RESO_MEAS;	// 18 bit & 100ms measurement rate
	res = ltr578_master_send(client, LTR578_ALS_MEAS_RATE, (char *)&buf, 1);
	APS_ERR("ALS sensor resolution & measurement rate: %d!\n", ALS_RESO_MEAS);
	if ((res = ltr578_setup_eint(client)) != 0)
	{
		APS_ERR("setup eint: %d\n", res);
		goto EXIT_ERR;
	}

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
	APS_LOG("ltr578_obj als enable value = %d\n", en);

	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return -1;
	}	

	res = ltr578_als_enable(ltr578_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr578_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr578_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr578_obj->enable);
	mutex_unlock(&ltr578_mutex);	

	//cancel_delayed_work(&ltr578_obj->check_ps_work);
	//schedule_delayed_work(&ltr578_obj->check_ps_work, msecs_to_jiffies(300));
	
	return 0;
}

static int als_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int als_get_data(int* value, int* status)
{
	int err = 0;
	
	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return -1;
	}

	ltr578_obj->als = ltr578_als_read(ltr578_obj->client, &ltr578_obj->als);
	if (ltr578_obj->als < 0)
		err = -1;
	else {
        *value =ltr578_obj->als; //ltr578_get_als_value(ltr578_obj, ltr578_obj->als);
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
	APS_LOG("ltr578_obj ps enable value = %d\n", en);

	if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return -1;
	}
	
	res = ltr578_ps_enable(ltr578_obj->client, en);
	if (res < 0) {
		APS_ERR("ps_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr578_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr578_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &ltr578_obj->enable);
	mutex_unlock(&ltr578_mutex);
	
	return 0;
}

static int ps_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;

    if(!ltr578_obj)
	{
		APS_ERR("ltr578_obj is null!!\n");
		return -1;
	}
    
	ltr578_obj->ps = ltr578_ps_read(ltr578_obj->client, &ltr578_obj->ps);
	if (ltr578_obj->ps < 0)
		err = -1;
	else {
		*value = ltr578_get_ps_value(ltr578_obj, ltr578_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
    
	return err;
}
/*-----------------------------------------------------------------------------------*/
static int ltr578_als_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
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
static int ltr578_als_factory_get_data(int32_t *data)
{
	int status;

	return als_get_data(data, &status);
}
static int ltr578_als_factory_get_raw_data(int32_t *data)
{
	int status;

	return als_get_data(data, &status);
}
static int ltr578_als_factory_enable_calibration(void)
{
	return 0;
}
static int ltr578_als_factory_clear_cali(void)
{
	return 0;
}
static int ltr578_als_factory_set_cali(int32_t offset)
{
	return 0;
}
static int ltr578_als_factory_get_cali(int32_t *offset)
{
	return 0;
}
static int ltr578_ps_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
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
static int ltr578_ps_factory_get_data(int32_t *data)
{
	int err = 0, status = 0;

	err = ps_get_data(data, &status);
	if (err < 0)
		return -1;
	return 0;
}
static int ltr578_ps_factory_get_raw_data(int32_t *data)
{
	int err = 0;
	struct ltr578_priv *obj = ltr578_obj;

	err = ltr578_ps_read(obj->client, &obj->ps);
	if (err < 0) {
		APS_ERR("%s failed\n", __func__);
		return -1;
	}
	*data = ltr578_obj->ps;
	return 0;
}
static int ltr578_ps_factory_enable_calibration(void)
{
	return 0;
}
static int ltr578_ps_factory_clear_cali(void)
{
	struct ltr578_priv *obj = ltr578_obj;

	obj->ps_cali = 0;
	return 0;
}
static int ltr578_ps_factory_set_cali(int32_t offset)
{
	struct ltr578_priv *obj = ltr578_obj;

	obj->ps_cali = offset;
	return 0;
}
static int ltr578_ps_factory_get_cali(int32_t *offset)
{
	struct ltr578_priv *obj = ltr578_obj;

	*offset = obj->ps_cali;
	return 0;
}
#if 0
static int ltr578_ps_factory_set_threashold(int32_t threshold[2])
{
	int err = 0;
	struct ltr578_priv *obj = ltr578_obj;

	APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0], threshold[1]);
	atomic_set(&obj->ps_thd_val_high, (threshold[0] + obj->ps_cali));
	atomic_set(&obj->ps_thd_val_low, (threshold[1] + obj->ps_cali));
	err = ltr578_ps_set_thres();

	if (err < 0) {
		APS_ERR("set_psensor_threshold fail\n");
		return -1;
	}
	return 0;
}
static int ltr578_ps_factory_get_threashold(int32_t threshold[2])
{
	struct ltr578_priv *obj = ltr578_obj;

	threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
	threshold[1] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
	return 0;
}
#endif
static struct alsps_factory_fops ltr578_factory_fops = {
	.als_enable_sensor = ltr578_als_factory_enable_sensor,
	.als_get_data = ltr578_als_factory_get_data,
	.als_get_raw_data = ltr578_als_factory_get_raw_data,
	.als_enable_calibration = ltr578_als_factory_enable_calibration,
	.als_clear_cali = ltr578_als_factory_clear_cali,
	.als_set_cali = ltr578_als_factory_set_cali,
	.als_get_cali = ltr578_als_factory_get_cali,

	.ps_enable_sensor = ltr578_ps_factory_enable_sensor,
	.ps_get_data = ltr578_ps_factory_get_data,
	.ps_get_raw_data = ltr578_ps_factory_get_raw_data,
	.ps_enable_calibration = ltr578_ps_factory_enable_calibration,
	.ps_clear_cali = ltr578_ps_factory_clear_cali,
	.ps_set_cali = ltr578_ps_factory_set_cali,
	.ps_get_cali = ltr578_ps_factory_get_cali,
	//.ps_set_threashold = ltr578_ps_factory_set_threashold,
	//.ps_get_threashold = ltr578_ps_factory_get_threashold,
};

static struct alsps_factory_public ltr578_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &ltr578_factory_fops,
};
/*-----------------------------------i2c operations----------------------------------*/
static int ltr578_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr578_priv *obj = NULL;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	int err = 0;
	u8 chip_id = 0;


	APS_FUN();
	/* get customization and power on */
	err = get_alsps_dts_func(client->dev.of_node, hw);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		return -EFAULT;
	}
	ltr578_power(hw, 1);
		
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr578_obj = obj;
	
	obj->hw = hw;
	INIT_WORK(&obj->eint_work, ltr578_eint_work);	
	INIT_DELAYED_WORK(&obj->check_ps_work, ltr578_check_ps_work);
	INIT_DELAYED_WORK(&obj->cali_ps_work, ltr578_cali_ps_work);

	
#ifdef CHECK_FAIL_WORK
	
		INIT_DELAYED_WORK(&dwork, ltr578_delay_work);//add delaywork
		
#endif

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
	obj->irq_node = client->dev.of_node;

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = ARRAY_SIZE(obj->hw->als_level);
	obj->als_value_num = ARRAY_SIZE(obj->hw->als_value);
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100	
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);

#ifdef SENSOR_DEFAULT_ENABLED
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
#else
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
#endif

	APS_LOG("ltr578_init_client() start...!\n");
	ltr578_i2c_client = client;
	err = ltr578_init_client();
	if(err)
	{
		goto exit_init_failed;
	}
	APS_LOG("ltr578_init_client() OK!\n");
	err = ltr578_master_recv(client, LTR578_PART_ID, &chip_id, 0x01);
	if (err < 0)
	{
		return err;
	}

	if(chip_id == LTR577_PART_ID)
	{
		sensor_part_id= 1;
		APS_LOG("the chip is 577 \n");
	}else
	{
		sensor_part_id= 0;
		APS_LOG("the chip is 578 \n");
	}
	err = alsps_factory_device_register(&ltr578_factory_device);
	if(err)
	{
		APS_ERR("ltr578_device register failed\n");
		goto exit_misc_device_register_failed;
	}

    als_ctl.is_use_common_factory =false;
	ps_ctl.is_use_common_factory = false;
	
	/*------------------------ltr578 attribute file for debug--------------------------------------*/
	//err = ltr578_create_attr(&(ltr578_init_info.platform_diver_addr->driver));
	err = ltr578_create_attr(&(ltr578_i2c_driver.driver));
	if(err)
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------ltr578 attribute file for debug--------------------------------------*/
	
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
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
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = true;
	ps_ctl.is_support_batch = false;
	ps_ctl.is_polling_mode = obj->hw->polling_mode_ps;

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

	ltr578_init_flag =0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:	
exit_init_failed:
	kfree(obj);
exit:
	ltr578_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr578_init_flag =-1;
	return err;
}

static int ltr578_i2c_remove(struct i2c_client *client)
{
	int err;

	ltr578_power(hw, 0);

	//err = ltr578_delete_attr(&(ltr578_init_info.platform_diver_addr->driver));
	err = ltr578_delete_attr(&(ltr578_i2c_driver.driver));
	if(err)
	{
		APS_ERR("ltr578_delete_attr fail: %d\n", err);
	}

	alsps_factory_device_deregister(&ltr578_factory_device);	
		
	ltr578_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int ltr578_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strncpy(info->type, LTR578_DEV_NAME, sizeof(info->type));
	return 0;
}

static int ltr578_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr578_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	if(test_bit(CMC_BIT_PS, &obj->enable))
	{
		APS_LOG("PS is in enable, no need to do anything to ALS!!\n");
		return 0;
	}
		
	atomic_set(&obj->als_suspend, 1);
	err = ltr578_als_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable als: %d\n", err);
		return err;
	}
#if 0
	atomic_set(&obj->ps_suspend, 1);
	err = ltr578_ps_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable ps:  %d\n", err);
		return err;
	}	
#endif	
	return 0;
}

static int ltr578_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr578_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}	

	if(test_bit(CMC_BIT_PS, &obj->enable))
	{
		APS_LOG("PS is in enable, no need to do anything to ALS!!\n");
		return 0;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		err = ltr578_als_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
#if 0
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = ltr578_ps_enable(obj->client, 1);
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
static int ltr578_remove(void)
{
	APS_FUN();

	i2c_del_driver(&ltr578_i2c_driver);
	ltr578_init_flag = -1;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int  ltr578_local_init(void)
{
	APS_FUN();	
	
	if(i2c_add_driver(&ltr578_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(-1 == ltr578_init_flag)
	{
	   return -1;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init ltr578_init(void)
{
	alsps_driver_add(&ltr578_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr578_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(ltr578_init);
module_exit(ltr578_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liteon");
MODULE_DESCRIPTION("LTR-578ALSPS Driver");
MODULE_LICENSE("GPL");

