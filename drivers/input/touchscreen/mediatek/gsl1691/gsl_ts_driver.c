
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
//#include <linux/rtpm_prio.h>
#include "tpd.h"

#include <linux/dma-mapping.h>

#include <linux/device.h> 
#include <linux/cdev.h> 
#include <linux/fs.h> 
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
//#include <linux/wakelock.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include "gsl_ts_driver.h"
//#include <mt_boot_common.h>



//#define TPD_PROXIMITY


#define GSL_DEBUG
//#define GSL_MONITOR
#define GSL_NOID_VERSION
#define GSLX680_NAME	"gslX680"
#define GSLX680_ADDR	0x40
#define MAX_FINGERS	  	10
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
#define SMBUS_TRANS_LEN	0x01
#define GSL_PAGE_REG		0xf0
//#define ADD_I2C_DEVICE_ANDROID_4_0
//#define HIGH_SPEED_I2C
//#define FILTER_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	9
#endif

#define GPIO_CTP_RST_PIN 0
#define GPIO_CTP_EINT_PIN 1
static unsigned int touch_irq;




#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>  //chenjiaxi
//static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif
#ifdef TPD_PROXIMITY
//#include <sensors_io.h>
//#include <hwmsensor.h>
//#include <hwmsen_dev.h>
#include "inc/aal_control.h"
#include "hwmsensor.h"
#include "sensors_io.h"
#include "hwmsen_helper.h"
#include <linux/io.h>
#include <linux/wakelock.h>
#include <alsps.h>
#include <linux/types.h>
#include <linux/ioctl.h>
static u8 tpd_proximity_flag = 0; //flag whether start alps
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
//static struct wake_lock ps_lock;
//static u8 gsl_psensor_data[8]={0};
//static u8 tdp_proximity_enabled = 1;
extern int ps_flush_report(void);
static int ps_flush(void);
struct gsl_priv {
	struct work_struct eint_work;
	ulong enable;		/*enable mask */
	ulong pending_intr;	/*pending interrupt */

	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
	bool ps_flush;

};
	static struct gsl_priv *gsl_obj;
//enum {
//	CMC_BIT_ALS = 1,
//	CMC_BIT_PS = 2,
//} CMC_BIT;
#endif

//static char GSL_TP_ID_USED = 0;							     
static int tpd_flag = 0;
static int tpd_halt=0;
static char eint_flag = 0;
extern struct tpd_device *tpd;
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;
#ifdef GSL_MONITOR
static struct delayed_work gsl_monitor_work;
static struct workqueue_struct *gsl_monitor_workqueue = NULL;
static u8 int_1st[4] = {0};
static u8 int_2nd[4] = {0};
static char bc_counter = 0;
static char b0_counter = 0;
static char i2c_lock_flag = 0;
#endif

static u32 id_sign[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;
//struct i2c_client *i2c_client_point = NULL;
//static atomic_t tpd_reg_en;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
//extern void mt_eint_unmask(unsigned int line);
//extern void mt_eint_mask(unsigned int line);

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)
#endif


#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif


static void startup_chip(struct i2c_client *client)
{
	char write_buf = 0x00;

	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf); 	
#ifdef GSL_NOID_VERSION
	gsl_DataInit(gsl_config_data_id);

#endif
	msleep(10);		
}


static void reset_chip(struct i2c_client *client)
{
	u8 write_buf[4]	= {0};
	u8 buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);

	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf[0]); 	
	msleep(10);

	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;
	write_buf[3] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xbc, 4, write_buf); 	
	msleep(10);



    //set vddio 1.8
	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0xfe;
	write_buf[3] = 0x01;
	i2c_smbus_write_i2c_block_data(client,0xf0,4,write_buf);
	buf[0] = 0x05; 
	buf[1] = 0x00; 
	buf[2] = 0x00; 
	buf[3] = 0x80; 
	i2c_smbus_write_i2c_block_data(client,0x78,4,buf);
	msleep(5);	
}

static void clr_reg(struct i2c_client *client)
{
	char write_buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);

	write_buf[0] = 0x03;
	i2c_smbus_write_i2c_block_data(client, 0x80, 1, &write_buf[0]); 	
	msleep(5);
	
	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf[0]); 	
	msleep(5);

	write_buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);
}


#ifdef HIGH_SPEED_I2C
static u32 gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{

}

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *pbt_buf, u32 dw_len)
{
	int i = 0;
	CTPI2CDMABuf_va[i] = reg;
	printk("TTTT %s  lens=%d  CTPI2CDMABuf_pa=0x%p\n",__func__,dw_len,CTPI2CDMABuf_pa);
	for(i = 0 ; i < dw_len; i++)
	{
		CTPI2CDMABuf_va[i+1] = pbt_buf[i];
	}
	if(dw_len <= 8)
	{
		return i2c_master_send(i2c_client, pbt_buf, dw_len);
	}
	else
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		return i2c_master_send(i2c_client, CTPI2CDMABuf_pa, dw_len + 1);
	}
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len;
	struct fw_data *ptr_fw;

	printk("=============gsl_load_fw start==============\n");
   
	ptr_fw = GSLX680_FW;
	source_len = ARRAY_SIZE(GSLX680_FW);
	
	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == ptr_fw[source_line].offset)
		{
			fw2buf(cur, &ptr_fw[source_line].val);
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			buf[0] = (u8)ptr_fw[source_line].offset;

			fw2buf(cur, &ptr_fw[source_line].val);
			cur += 4;

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
	    			gsl_write_interface(client, buf[0], buf, cur - buf - 1);
	    			cur = buf + 1;
			}

			send_flag++;
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#else


static void gsl_load_fw(struct i2c_client *client)
{
	char buf[SMBUS_TRANS_LEN*4] = {0};
	char reg = 0, send_flag = 1, cur = 0;
	
	unsigned int source_line = 0;
	unsigned int source_len = ARRAY_SIZE(GSLX680_FW);

	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		if(1 == SMBUS_TRANS_LEN)
		{
			reg = GSLX680_FW[source_line].offset;

			buf[0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
			buf[1] = (char)((GSLX680_FW[source_line].val & 0x0000ff00) >> 8);
			buf[2] = (char)((GSLX680_FW[source_line].val & 0x00ff0000) >> 16);
			buf[3] = (char)((GSLX680_FW[source_line].val & 0xff000000) >> 24);

			i2c_smbus_write_i2c_block_data(client, reg, 4, buf); 	
		}
		else
		{
			/* init page trans, set the page val */
			if (GSL_PAGE_REG == GSLX680_FW[source_line].offset)
			{
				buf[0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
				i2c_smbus_write_i2c_block_data(client, GSL_PAGE_REG, 1, &buf[0]); 	
				send_flag = 1;
			}
			else 
			{
				if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
					reg = GSLX680_FW[source_line].offset;

				buf[cur + 0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
				buf[cur + 1] = (char)((GSLX680_FW[source_line].val & 0x0000ff00) >> 8);
				buf[cur + 2] = (char)((GSLX680_FW[source_line].val & 0x00ff0000) >> 16);
				buf[cur + 3] = (char)((GSLX680_FW[source_line].val & 0xff000000) >> 24);
				cur += 4;

				if (0 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08)) 
				{
					i2c_smbus_write_i2c_block_data(client, reg, SMBUS_TRANS_LEN*4, buf); 	
					cur = 0;
				}

				send_flag++;

			}
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#endif


static int test_i2c(struct i2c_client *client)
{
	char read_buf = 0;
	char write_buf = 0x12;
	int ret, rc = 1;
	
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if  (ret  < 0)  {
		    printk("NOT OK I read reg 0xf0 is %x\n", read_buf);
    		rc --;
    	}
	else
		printk("I read reg 0xf0 is %x\n", read_buf);

	msleep(2);
	ret = i2c_smbus_write_i2c_block_data( client, 0xf0, 1, &write_buf );
	if(ret  >=  0 )
		printk("I write reg 0xf0 0x12\n");
	
	msleep(2);
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if(ret <  0 )
		rc --;
	else
		printk("I read reg 0xf0 is 0x%x\n", read_buf);

	return rc;
}


static void init_chip(struct i2c_client *client)
{
	int rc;
	
	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	//msleep(20); 	
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	//msleep(20); 		
	tpd_gpio_output(GPIO_CTP_RST_PIN, 0);
	msleep(20);
	tpd_gpio_output(GPIO_CTP_RST_PIN, 1);
	msleep(20);

	rc = test_i2c(client);
	if(rc < 0)
	{
		printk("------gslX680 test_i2c error------\n");	
		return;
	}	
	clr_reg(client);
	reset_chip(client);
	gsl_load_fw(client);			
	startup_chip(client);
	reset_chip(client);
	startup_chip(client);		
}


#ifdef TPD_PROXIMITY
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;//send to OS to controll backlight on/off
}

static int tpd_enable_ps(int enable)
{
	u8 buf[4];
	if (enable) {
		//wake_lock(&ps_lock);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x2;
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);
		tpd_proximity_flag = 1;
		//add alps of function
		printk("tpd-ps function is on\n");
	}
	else 
	{
		tpd_proximity_flag = 0;
		//wake_unlock(&ps_lock);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;

		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] =0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x0;
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);	
		printk("tpd-ps function is off\n");
	}
	return 0;
}
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int ps_enable_nodata(int en)
{
	printk("gsl_obj ps enable value = %d\n", en);
	if (en) {
		tpd_enable_ps(1);
		set_bit(2/*CMC_BIT_PS*/, &gsl_obj->enable);
	} else {
		tpd_enable_ps(0);
		clear_bit(2/*CMC_BIT_PS*/, &gsl_obj->enable);
	}
	return 0;
}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{
	*value = tpd_get_ps_value();
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;
	/*FIX  ME */

	printk("ltr559 ps set delay = (%d) ok.\n", value);
	return 0;
}

static int ps_flush(void)
{
	int err = 0;
	if (!test_bit(2, &gsl_obj->enable)) {
		gsl_obj->ps_flush = true;
		return 0;
	}
  err = ps_flush_report();
  if (err >= 0)
  			gsl_obj->ps_flush = false;
	return err;
}


#endif

static void check_mem_data(struct i2c_client *client)
{
	char read_buf[4]  = {0};
	
	msleep(30);
	i2c_smbus_read_i2c_block_data(client,0xb0, sizeof(read_buf), read_buf);
	
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		printk("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip(client);
	}
}

#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
    if(ch>='0' && ch<='9')
        return (ch-'0');
    else
        return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	//char *ptr = page;
	char temp_data[5] = {0};
	unsigned int tmp=0;
	
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_NOID_VERSION
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		//ptr += sprintf(ptr,"version:%x\n",tmp);
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_NOID_VERSION 
	        
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			//ptr +=sprintf(ptr,"gsl_config_data_id[%d] = ",tmp);
			//if(tmp>=0&&tmp<512)
			//	ptr +=sprintf(ptr,"%d\n",gsl_config_data_id[tmp]); 
#endif
		}
		else 
		{
			i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,&gsl_data_proc[4]);
			if(gsl_data_proc[0] < 0x80)
				i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);

			//ptr +=sprintf(ptr,"offset : {0x%02x,0x",gsl_data_proc[0]);
			//ptr +=sprintf(ptr,"%02x",temp_data[3]);
			//ptr +=sprintf(ptr,"%02x",temp_data[2]);
			//ptr +=sprintf(ptr,"%02x",temp_data[1]);
			//ptr +=sprintf(ptr,"%02x};\n",temp_data[0]);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
	//*eof = 1;
	//return (ptr - page);
	return 0;
}
static ssize_t gsl_config_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%d]\n", CONFIG_LEN, count);
        return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}	
	//if(copy_from_user(path_buf, buffer, (count<CONFIG_LEN?count:CONFIG_LEN)))
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
	
#ifdef GSL_MONITOR
		//printk( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
		cancel_delayed_work_sync(&gsl_monitor_work);
#endif
		gsl_proc_flag = 1;
		reset_chip(i2c_client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		reset_chip(i2c_client);
		startup_chip(i2c_client);
		
#ifdef GSL_MONITOR
			//printk( "gsl_ts_resume () : queue gsl_monitor_work\n");
			queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
#endif
	
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		i2c_smbus_write_i2c_block_data(i2c_client,buf[4],4,buf);
	}
#ifdef GSL_NOID_VERSION
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<512)
		{
			gsl_config_data_id[tmp1] = tmp;
		}	

		
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}

static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif


#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;
	u16 filter_step_x = 0, filter_step_y = 0;
	
	id_sign[id] = id_sign[id] + 1;
	if(id_sign[id] == 1)
	{
		x_old[id] = x;
		y_old[id] = y;
	}
	
	x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
	y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);

	if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
	{
		filter_step_x = x_err;
		filter_step_y = y_err;
	}
	else
	{
		if(x_err > FILTER_MAX)
			filter_step_x = x_err; 
		if(y_err> FILTER_MAX)
			filter_step_y = y_err;
	}

	if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
	{
		filter_step_x >>= 2; 
		filter_step_y >>= 2;
	}
	else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
	{
		filter_step_x >>= 1; 
		filter_step_y >>= 1;
	}	
	else if(x_err <= 4*FILTER_MAX && y_err <= 4*FILTER_MAX)
	{
		filter_step_x = filter_step_x*3/4; 
		filter_step_y = filter_step_y*3/4;
	}	
	
	x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
	y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);

	x_old[id] = x_new;
	y_old[id] = y_new;
}
#else

static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;

	id_sign[id]=id_sign[id]+1;
	
	if(id_sign[id]==1){
		x_old[id]=x;
		y_old[id]=y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;
		
	if(x>x_old[id]){
		x_err=x -x_old[id];
	}
	else{
		x_err=x_old[id]-x;
	}

	if(y>y_old[id]){
		y_err=y -y_old[id];
	}
	else{
		y_err=y_old[id]-y;
	}

	if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
		x_new = x;     x_old[id] = x;
		y_new = y;     y_old[id] = y;
	}
	else{
		if(x_err > 3){
			x_new = x;     x_old[id] = x;
		}
		else
			x_new = x_old[id];
		if(y_err> 3){
			y_new = y;     y_old[id] = y;
		}
		else
			y_new = y_old[id];
	}

	if(id_sign[id]==1){
		x_new= x_old[id];
		y_new= y_old[id];
	}
	
}
#endif

void tpd_down( int id, int x, int y, int p) 
{

	print_info("------11111tpd_down id: %d, x:%d, y:%d------ \n", id, x, y);

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id); 	
	input_mt_sync(tpd->dev);

	//if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	//{   
	//#ifdef TPD_HAVE_BUTTON 
	//	tpd_button(x, y, 1);  
	//#endif
	//}
}

void tpd_up(void) 
{
	print_info("------tpd_up------ \n");

	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	
	//if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	//{   
	//#ifdef TPD_HAVE_BUTTON 
	//    tpd_button(0, 0, 0);  
	//#endif
	//}
}

static void report_data_handle(void)
{

	char touch_data[MAX_FINGERS * 4 + 4] = {0};
	char buf[4] = {0};
	unsigned char id, point_num = 0;
	unsigned int x, y, temp_a, temp_b, i,tmp1;
#ifdef GSL_NOID_VERSION
	struct gsl_touch_info cinfo={{0},{0},{0},0};
#endif
#ifdef TPD_PROXIMITY
		//int err;
		struct hwm_sensor_data sensor_data;
	
	if (tpd_proximity_flag == 1)
        {
                i2c_smbus_read_i2c_block_data(i2c_client,0xac,4,buf);
                print_info("gslX680   buf[0] = %d buf[1] = %d,  buf[2] = %d  buf[3] = %d \n",buf[0],buf[1],buf[2],buf[3]);

                if (buf[0] == 1 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
                {
                    tpd_proximity_detect = 0;
                }
                else
                {
                    tpd_proximity_detect = 1;
                }
                print_info("gslX680    ps change   tpd_proximity_detect = %d  \n",tpd_proximity_detect);
                //map and store data to hwm_sensor_data
                sensor_data.values[0] = tpd_get_ps_value();
                sensor_data.value_divide = 1;
                sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
                //let up layer to know
                if(ps_report_interrupt_data(tpd_get_ps_value()))
                {
                    print_info("call hwmsen_get_interrupt_data fail \n");
                }
            
        }
#endif
#ifdef GSL_MONITOR
	if(i2c_lock_flag != 0)
		return;
	else
		i2c_lock_flag = 1;
#endif



#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
        return;
#endif

	i2c_smbus_read_i2c_block_data(i2c_client, 0x80, 4, &touch_data[0]);
	point_num = touch_data[0];
	if(point_num > 0)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x84, 8, &touch_data[4]);
	if(point_num > 2)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x8c, 8, &touch_data[12]);
	if(point_num > 4)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x94, 8, &touch_data[20]);
	if(point_num > 6)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x9c, 8, &touch_data[28]);
	if(point_num > 8)
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa4, 8, &touch_data[36]);
	
#ifdef GSL_NOID_VERSION
	cinfo.finger_num = point_num;
	print_info("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(i = 0; i < (point_num < MAX_CONTACTS ? point_num : MAX_CONTACTS); i ++)
	{
		temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
		temp_b = touch_data[(i + 1) * 4 + 2];
		cinfo.x[i] = temp_a << 8 |temp_b;
		temp_a = touch_data[(i + 1) * 4 + 1];
		temp_b = touch_data[(i + 1) * 4 + 0];
		cinfo.y[i] = temp_a << 8 |temp_b;		
		cinfo.id[i] = ((touch_data[(i + 1) * 4 + 3] & 0xf0)>>4);
		print_info("tp-gsl  before: x[%d] = %d, y[%d] = %d, id[%d] = %d \n",i,cinfo.x[i],i,cinfo.y[i],i,cinfo.id[i]);
	}
	cinfo.finger_num = (touch_data[3]<<24)|(touch_data[2]<<16)|
		(touch_data[1]<<8)|touch_data[0];
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
	}
	point_num = cinfo.finger_num;
#endif


	for(i = 1 ;i <= MAX_CONTACTS; i ++)
	{
		if(point_num == 0)
			id_sign[i] = 0;	
		id_state_flag[i] = 0;
	}
	for(i = 0; i < (point_num < MAX_FINGERS ? point_num : MAX_FINGERS); i ++)
	{
	#ifdef GSL_NOID_VERSION
		id = cinfo.id[i];
		x =  cinfo.x[i];
		y =  cinfo.y[i];
	#else
		id = touch_data[(i + 1) * 4 + 3] >> 4;
		temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
		temp_b = touch_data[(i + 1) * 4 + 2];
		x = temp_a << 8 |temp_b;
		temp_a = touch_data[(i + 1) * 4 + 1];
		temp_b = touch_data[(i + 1) * 4 + 0];
		y = temp_a << 8 |temp_b;	
	#endif
	
		if(1 <= id && id <= MAX_CONTACTS)
		{
		#ifdef FILTER_POINT
			filter_point(x, y ,id);
		#else
			record_point(x, y , id);
		#endif
			tpd_down(id, x_new, y_new, 10);
			id_state_flag[id] = 1;
		}
	}
	for(i = 1; i <= MAX_CONTACTS; i ++)
	{	
		if( (0 == point_num) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
		{
			id_sign[i]=0;
		}
		id_state_old_flag[i] = id_state_flag[i];
	}			
	if(0 == point_num)
	{
		tpd_up();
	}
	input_sync(tpd->dev);
#ifdef GSL_MONITOR
	i2c_lock_flag = 0;
#endif
}

#ifdef GSL_MONITOR
static void gsl_monitor_worker(struct work_struct *work)
{
	//u8 write_buf[4] = {0};
	u8 read_buf[4]  = {0};
	u8 init_chip_flag = 0;
	
	print_info("----------------gsl_monitor_worker-----------------\n");	

	if(i2c_lock_flag != 0)
		goto queue_monitor_work;
	else
		i2c_lock_flag = 1;
	
	i2c_smbus_read_i2c_block_data(i2c_client, 0xb0, 4, read_buf);
	if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter ++;
	else
		b0_counter = 0;

	if(b0_counter > 1)
	{
		printk("======read 0xb0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		b0_counter = 0;
		goto queue_monitor_init_chip;
	}

	i2c_smbus_read_i2c_block_data(i2c_client, 0xb4, 4, read_buf);	
	
	int_2nd[3] = int_1st[3];
	int_2nd[2] = int_1st[2];
	int_2nd[1] = int_1st[1];
	int_2nd[0] = int_1st[0];
	int_1st[3] = read_buf[3];
	int_1st[2] = read_buf[2];
	int_1st[1] = read_buf[1];
	int_1st[0] = read_buf[0];

	if (int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0]) 
	{
		printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",int_1st[3], int_1st[2], int_1st[1], int_1st[0], int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
		init_chip_flag = 1;
		goto queue_monitor_init_chip;
	}
#if 1 //version 1.4.0 or later than 1.4.0 read 0xbc for esd checking
	i2c_smbus_read_i2c_block_data(i2c_client, 0xbc, 4, read_buf);
	if(read_buf[3] != 0 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
		bc_counter ++;
	else
		bc_counter = 0;

	if(bc_counter > 1)
	{
		printk("======read 0xbc: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		bc_counter = 0;
	}
#else //chenjiaxi
	write_buf[3] = 0x01;
	write_buf[2] = 0xfe;
	write_buf[1] = 0x10;
	write_buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, write_buf);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 4, read_buf);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 4, read_buf);
	
	if(read_buf[3] < 10 && read_buf[2] < 10 && read_buf[1] < 10 && read_buf[0] < 10)
		dac_counter ++;
	else
		dac_counter = 0;

	if(dac_counter > 1) 
	{
		printk("======read DAC1_0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		dac_counter = 0;
	}
#endif
queue_monitor_init_chip:
	if(init_chip_flag)
		init_chip(i2c_client);
	
	i2c_lock_flag = 0;
	
queue_monitor_work:	
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 100);
}
#endif

static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);
	
	do
	{
//		enable_irq(touch_irq);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		print_info("===touch_event_handler, task running===\n");

		eint_flag = 0;
		report_data_handle();
		enable_irq(touch_irq);
	} while (!kthread_should_stop());
	
	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(void)
{
	print_info("===tpd irq interrupt===\n");

	eint_flag = 1;
	tpd_flag=1; 
	disable_irq_nosync(touch_irq);
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	//node = of_find_compatible_node(NULL, NULL, "mediatek,cap_touch");
	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		TPD_DMESG("wqcat tpd_irq_registration node is found!\n");
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		
		touch_irq = irq_of_parse_and_map(node, 0);
		printk("gsl touch_irq = %d\n",touch_irq);
		ret = request_irq(touch_irq, (irq_handler_t) tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "TOUCH-eint", NULL); //
		printk("gsl ret = %d\n",ret);
		if (ret > 0)
			printk("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		printk("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}

	
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int err = 0;
//	char buffer[5];
//	int status=0;
	int ret=0;
#ifdef TPD_PROXIMITY
	struct gsl_priv *obj;
	//struct hwmsen_object obj_ps;
	//struct als_control_path als_ctl = { 0 };
	//struct als_data_path als_data = { 0 };
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };
		obj = kzalloc(sizeof(*obj), GFP_KERNEL);
		gsl_obj = obj;

#endif
	printk("==tpd_i2c_probe==\n");
	tpd_gpio_output(GPIO_CTP_RST_PIN,0);
	msleep(100);
  tpd->reg=regulator_get(tpd->tpd_dev,"vtouch");
  if(IS_ERR(tpd->reg)){
  	printk("fy cannot find vtouch\n");
  }else{
  	ret=regulator_set_voltage(tpd->reg,2800000,2800000);
  	if(ret!=0){
  		printk("regulator set vtouch failed\n");
  	}else{
  		if(0!=regulator_enable(tpd->reg)){
  			printk("fail to enable vtouch\n");
  		}
  	}
  }
		//power on, need confirm with SA
//	hwPowerOn(TP_POWER, VOL_2800, "TP");
//	pmic_set_register_value(PMIC_RG_VGP1_VOSEL,5);
//	pmic_set_register_value(PMIC_RG_VGP1_EN,1);
	msleep(100);
	tpd_gpio_output(GPIO_CTP_RST_PIN,1);
  tpd_gpio_as_int(GPIO_CTP_EINT_PIN);
	msleep(50);
	

	i2c_client = client;
	//i2c_client->timing = 400;	//Paul@ added for i2c speed 
	i2c_client->addr=GSLX680_ADDR;

	init_chip(i2c_client);
	check_mem_data(i2c_client);
  	tpd_irq_registration();
 	 disable_irq(touch_irq);
	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}

#ifdef GSL_MONITOR
	printk( "tpd_i2c_probe () : queue gsl_monitor_workqueue\n");
	INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
	gsl_monitor_workqueue = create_singlethread_workqueue("gsl_monitor_workqueue");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1000);
#endif


#ifdef TPD_PROC_DEBUG
#if 0
    gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
    //printk("[tp-gsl] [%s] gsl_config_proc = %x \n",__func__,gsl_config_proc);
	if (gsl_config_proc == NULL)
	{
		print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	else
	{
		gsl_config_proc->read_proc = gsl_config_read_proc;
		gsl_config_proc->write_proc = gsl_config_write_proc;
	}
#else
    proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
#endif
    gsl_proc_flag = 0;
#endif
#ifdef TPD_PROXIMITY

       gsl_obj = kzalloc(sizeof(*gsl_obj), GFP_KERNEL);

	//wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");  
	ps_ctl.is_use_common_factory = false;
	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		printk("register fail = %d\n", err);
		//goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		printk("tregister fail = %d\n", err);
		//goto exit_sensor_obj_attach_fail;
	}
	//tpd_enable_ps(1);
#endif

	printk("==tpd_i2c_probe end==\n");
		
  enable_irq(touch_irq);
	return 0;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	printk("==tpd_i2c_remove==\n");
	
	return 0;
}


static const struct i2c_device_id tpd_i2c_id[] = {{TPD_DEVICE,0},{}};
#ifdef ADD_I2C_DEVICE_ANDROID_4_0
static struct i2c_board_info __initdata gslX680_i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (GSLX680_ADDR))};
#else
static const struct of_device_id gsl680_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
#endif

struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = TPD_DEVICE,
	#ifndef ADD_I2C_DEVICE_ANDROID_4_0	 
		.owner = THIS_MODULE,
	#endif
	  .of_match_table=of_match_ptr(gsl680_dt_match)
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = tpd_i2c_id,
//	.detect = tpd_i2c_detect,
};

int tpd_local_init(void)
{
	printk("==tpd_local_init==\n");

	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	/*
	if(tpd_load_status == 0)
	{
		TPD_DMESG("add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	*/
#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data 在tpd_button.c中 就是讲dts中的按键有关的值拷贝到全局变量中
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);
#endif
	tpd_type_cap = 1;

	printk("==tpd_local_init end==\n");
	return 0;
}

/* Function to manage low power suspend */
void tpd_suspend(struct device *h)
{
	printk("==tpd_suspend_start==\n");
#ifdef TPD_PROXIMITY
    if (tpd_proximity_flag == 1)
    {
        return ;
    }
#endif
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
	tpd_halt = 1;
#ifdef GSL_MONITOR
	printk( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
	cancel_delayed_work_sync(&gsl_monitor_work);
#endif

  		disable_irq(touch_irq);
	    tpd_gpio_output(GPIO_CTP_RST_PIN, 0);
	    tpd_gpio_output(GPIO_CTP_EINT_PIN,0);
	printk("==tpd_suspend_end==\n");
//#endif	
}

/* Function to manage power-on resume */
void tpd_resume(struct device *h)
{
	printk("==tpd_resume_start==\n");
#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1){
        return;
    }
#endif	
	//printk("==tpd_resume_hwPowerOn_start==\n");
	//hwPowerOn(TP_POWER, VOL_2800, "TP");	
	//msleep(100);start
	//printk("==tpd_resume_hwPowerOn_end==\n");
  	tpd_gpio_output(GPIO_CTP_RST_PIN, 1);
  	tpd_gpio_as_int(GPIO_CTP_EINT_PIN);
	msleep(30);	
	enable_irq(touch_irq);
//	init_chip_without_rst(i2c_client);
	reset_chip(i2c_client);
	startup_chip(i2c_client);
	check_mem_data(i2c_client);	
//#endif //Paul@ added 20140116 end
#ifdef GSL_MONITOR
	printk( "gsl_ts_resume () : queue gsl_monitor_work\n");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
#endif
#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			tpd_enable_ps(1);
		}
#endif
	tpd_halt = 0;
	printk("==tpd_resume_end==\n");
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = GSLX680_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON //有定义
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
	
};

//start by lai
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("Sileadinc gslX680 touch panel driver init\n");
#ifdef ADD_I2C_DEVICE_ANDROID_4_0 //没有定义
	i2c_register_board_info(1, &gslX680_i2c_tpd, 1);	
#endif
  tpd_get_dts_info(); //在mtk_tpd.c中 获取dts中的信息
	if(tpd_driver_add(&tpd_device_driver) < 0) //加到tpd_driver_list[i]列表中
		printk("add gslX680 driver failed\n");
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
	printk("Sileadinc gslX680 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);



