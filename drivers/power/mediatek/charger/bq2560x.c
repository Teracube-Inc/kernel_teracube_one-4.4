#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include "mtk_charger_intf.h"
#include "bq2560x.h"

#define STATUS_OK   0
#define STATUS_UNSUPPORTED  -1
#define HIGH_BATTERY_VOLTAGE_SUPPORT
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

const unsigned int VBAT_CV_VTH[] = {
    3856000, 3888000, 3920000, 3952000, 
    3984000, 4016000, 4048000, 4080000, 
    4112000, 4144000, 4176000, 4208000, 
    4240000, 4272000, 4304000, 4336000, 
    4368000, 4400000, 4432000, 4464000,
    4496000, 4528000, 4560000, 4592000,
    4624000,
};

const unsigned int CS_VTH[] = {
    50000, 60000, 120000, 180000,
    240000, 300000, 360000, 420000,
    480000, 540000, 600000, 660000,
    720000, 780000, 840000, 900000,
    960000, 1020000, 1080000, 1140000,
    1200000, 1260000, 1320000, 1380000,
    1440000, 1500000, 1560000, 1620000,
    1680000, 1740000, 1800000, 1860000,
    1920000, 1980000, 2040000, 2100000
};

const unsigned int INPUT_CS_VTH[] = {
    80000, 100000, 200000, 300000,
    400000, 500000, 600000, 700000,
    800000, 900000, 1000000, 1100000,
    1200000, 1300000, 1400000, 1500000,
    1600000, 1700000, 1800000, 1900000,
    2000000, 2100000, 2200000, 2300000,
    2400000, 2500000, 2600000, 2700000,
    2800000, 2900000, 3000000, 3200000
};

const unsigned int VCDT_HV_VTH[] = {
    4200000, 4250000, 4300000, 4350000,
    4400000, 4450000, 4500000, 4550000,
    4600000, 6000000, 6500000, 7000000,
    7500000, 8500000, 9500000, 10500000
};

struct bq2560x_info {
    struct device *dev;
    struct i2c_client *i2c;
    struct charger_device *chg_dev;
    const char *chg_dev_name;
};

/* ============================================================ // */
unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
                       const unsigned int val)
{
    if (val < array_size) {
        return parameter[val];
    } else {
        pr_debug("Can't find the parameter \r\n");
        return parameter[0];
    }
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
                       const unsigned int val)
{
    unsigned int i;

    pr_debug("array_size = %d \r\n", array_size);

    for (i = 0; i < array_size; i++) {
        if (val == *(parameter + i))
            return i;

    }

    pr_debug("NO register value match. val=%d\r\n", val);
    /* TODO: ASSERT(0);      // not find the value */
    return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
                     unsigned int level)
{
    unsigned int i;
    unsigned int max_value_in_last_element;

    if (pList[0] < pList[1])
        max_value_in_last_element = true;
    else
        max_value_in_last_element = false;

    if (max_value_in_last_element == true) {
        for (i = (number - 1); i != 0; i--) {/* max value in the last element */

            if (pList[i] <= level)
                return pList[i];

        }

        pr_debug("Can't find closest level, small value first \r\n");
        return pList[0];
        /* return CHARGE_CURRENT_0_00_MA; */
    } else {
        for (i = 0; i < number; i++) {/* max value in the first element */

            if (pList[i] <= level)
                return pList[i];

        }

        pr_debug("Can't find closest level, large value first \r\n");
        return pList[number - 1];
        /* return CHARGE_CURRENT_0_00_MA; */
    }
}

/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define BQ2560x_SLAVE_ADDR_WRITE   0xD6
#define BQ2560x_SLAVE_ADDR_READ    0xD7


static struct i2c_client *new_client;
static const struct i2c_device_id bq2560x_i2c_id[] = { {"bq2560x", 0}, {} };

kal_bool chargin_hw_init_done = false;
static int bq2560x_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id bq2560x_of_match[] = {
	{.compatible = "mediatek,sw_charger",},
	{},
};

MODULE_DEVICE_TABLE(of, bq2560x_of_match);
#endif

static struct i2c_driver bq2560x_driver = {
	.driver = {
		   .name = "bq2560x",
#ifdef CONFIG_OF
		   .of_match_table = bq2560x_of_match,
#endif
		   },
	.probe = bq2560x_driver_probe,
	.id_table = bq2560x_i2c_id,
	//.shutdown = bq2560x_shutdown,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq2560x_reg[bq2560x_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq2560x_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write bq2560x]
  *
  *********************************************************/
int bq2560x_read_byte(unsigned char cmd, unsigned char *returnData)
{
    char     readData = 0;
    int      ret = 0;
    struct i2c_msg msg[2];
    struct i2c_adapter *adap = new_client->adapter;

    mutex_lock(&bq2560x_i2c_access);
    msg[0].addr = new_client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &cmd;

    msg[1].addr = new_client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &readData;

    ret = i2c_transfer(adap, msg, 2);
    if (ret < 0) {
        mutex_unlock(&bq2560x_i2c_access);
        return 0;
    }
    *returnData = readData;

    mutex_unlock(&bq2560x_i2c_access);
    return 1;
}

int bq2560x_write_byte(unsigned char cmd, unsigned char writeData)
{
    char write_data[2] = { 0 };
    int ret = 0;
    struct i2c_msg msg;
    struct i2c_adapter *adap = new_client->adapter;

    mutex_lock(&bq2560x_i2c_access);
    write_data[0] = cmd;
    write_data[1] = writeData;
    msg.addr = new_client->addr;
    msg.flags = 0;
    msg.len = 2;
    msg.buf = (char *)write_data;

    ret = i2c_transfer(adap, &msg, 1);
    if (ret < 0) {
        mutex_unlock(&bq2560x_i2c_access);
        return 0;
    }

    mutex_unlock(&bq2560x_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq2560x_read_interface(unsigned char RegNum, unsigned char*val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq2560x_reg = 0;
	int ret = 0;


	ret = bq2560x_read_byte(RegNum, &bq2560x_reg);
	//pr_debug("[bq2560x_read_interface] Reg[%x]=0x%x\n", RegNum, bq2560x_reg);

	bq2560x_reg &= (MASK << SHIFT);
	*val = (bq2560x_reg >> SHIFT);
	//pr_debug("[bq2560x_read_interface] Val=0x%x\n", *val);

	return ret;
}

unsigned int bq2560x_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq2560x_reg = 0;
	int ret = 0;

	//pr_debug("--------------------------------------------------\n");

	ret = bq2560x_read_byte(RegNum, &bq2560x_reg);
	//pr_debug("[bq2560x_config_interface] Reg[%x]=0x%x\n", RegNum, bq2560x_reg);

	bq2560x_reg &= ~(MASK << SHIFT);
	bq2560x_reg |= (val << SHIFT);

	ret = bq2560x_write_byte(RegNum, bq2560x_reg);
	//pr_debug("[bq2560x_config_interface] Write Reg[%x]=0x%x\n", RegNum, bq2560x_reg);

	/* Check */
	 bq2560x_read_byte(RegNum, &bq2560x_reg); 
	 //pr_debug("[bq2560x_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq2560x_reg); 

	return ret;
}

/* write one register directly */
unsigned int bq2560x_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned char ret = 0;

	ret = bq2560x_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */

void bq2560x_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}

void bq2560x_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
	    );
}

void bq2560x_set_stat_ctrl(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_STAT_IMON_CTRL_MASK),
				       (unsigned char) (CON0_STAT_IMON_CTRL_SHIFT)
	    );
}

/* CON1---------------------------------------------------- */

void bq2560x_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON11),
				       (unsigned char) (val),
				       (unsigned char) (CON11_REG_RST_MASK),
				       (unsigned char) (CON11_REG_RST_SHIFT)
	    );
}

void bq2560x_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK),
				       (unsigned char) (CON1_WDT_RST_SHIFT)
	    );
}

void bq2560x_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK),
				       (unsigned char) (CON1_OTG_CONFIG_SHIFT)
	    );
}


void bq2560x_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT)
	    );
}


void bq2560x_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK),
				       (unsigned char) (CON1_SYS_MIN_SHIFT)
	    );
}

void bq2560x_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_MIN_VBAT_SEL_MASK),
				       (unsigned char) (CON1_MIN_VBAT_SEL_SHIFT)
	    );
}



/* CON2---------------------------------------------------- */

void bq2560x_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BOOST_LIM_MASK),
				       (unsigned char) (CON2_BOOST_LIM_SHIFT)
	    );
}

void bq2560x_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICHG_MASK), (unsigned char) (CON2_ICHG_SHIFT)
	    );
}

#if 0 //this function does not exist on bq2560x
void bq2560x_set_force_20pct(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_20PCT_MASK),
				       (unsigned char) (CON2_FORCE_20PCT_SHIFT)
	    );
}
#endif
/* CON3---------------------------------------------------- */

void bq2560x_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK),
				       (unsigned char) (CON3_IPRECHG_SHIFT)
	    );
}

void bq2560x_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_ITERM_MASK), (unsigned char) (CON3_ITERM_SHIFT)
	    );
}

/* CON4---------------------------------------------------- */

void bq2560x_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VREG_MASK), (unsigned char) (CON4_VREG_SHIFT)
	    );
}

void bq2560x_set_topoff_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_TOPOFF_TIMER_MASK), (unsigned char) (CON4_TOPOFF_TIMER_SHIFT)
	    );

}


void bq2560x_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VRECHG_MASK),
				       (unsigned char) (CON4_VRECHG_SHIFT)
	    );
}

/* CON5---------------------------------------------------- */

void bq2560x_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK),
				       (unsigned char) (CON5_EN_TERM_SHIFT)
	    );
}



void bq2560x_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK),
				       (unsigned char) (CON5_WATCHDOG_SHIFT)
	    );
}

void bq2560x_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK),
				       (unsigned char) (CON5_EN_TIMER_SHIFT)
	    );
}

void bq2560x_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK),
				       (unsigned char) (CON5_CHG_TIMER_SHIFT)
	    );
}

void bq2560x_set_treg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_TREG_MASK), (unsigned char) (CON5_TREG_SHIFT)
	    );
}

/* CON6---------------------------------------------------- */

void bq2560x_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VINDPM_MASK),
				       (unsigned char) (CON6_VINDPM_SHIFT)
	    );
}


void bq2560x_set_ovp(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_OVP_MASK),
				       (unsigned char) (CON6_OVP_SHIFT)
	    );

}

void bq2560x_set_boostv(unsigned int val)
{

	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
	    );



}



/* CON7---------------------------------------------------- */

void bq2560x_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_TMR2X_EN_MASK),
				       (unsigned char) (CON7_TMR2X_EN_SHIFT)
	    );
}

void bq2560x_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_Disable_MASK),
				       (unsigned char) (CON7_BATFET_Disable_SHIFT)
	    );
}


void bq2560x_set_batfet_delay(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_DLY_MASK),
				       (unsigned char) (CON7_BATFET_DLY_SHIFT)
	    );
}

void bq2560x_set_batfet_reset_enable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_RST_EN_MASK),
				       (unsigned char) (CON7_BATFET_RST_EN_SHIFT)
	    );
}


/* CON8---------------------------------------------------- */

unsigned int bq2560x_get_system_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val), (unsigned char) (0xFF), (unsigned char) (0x0)
	    );
	return val;
}

unsigned int bq2560x_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_VBUS_STAT_MASK),
				     (unsigned char) (CON8_VBUS_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq2560x_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_CHRG_STAT_MASK),
				     (unsigned char) (CON8_CHRG_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq2560x_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_VSYS_STAT_MASK),
				     (unsigned char) (CON8_VSYS_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq2560x_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_PG_STAT_MASK),
				     (unsigned char) (CON8_PG_STAT_SHIFT)
	    );
	return val;
}


/*CON10----------------------------------------------------------*/

void bq2560x_set_int_mask(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_INT_MASK_MASK),
				       (unsigned char) (CON10_INT_MASK_SHIFT)
	    );
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
static int  bq2560x_dump_register(struct charger_device *chg_dev)
{
	unsigned char i = 0;
	//printk("[bq2560x] ");
	for (i = 0; i < bq2560x_REG_NUM; i++) {
		bq2560x_read_byte(i, &bq2560x_reg[i]);
		//printk("[0x%x]=0x%x ", i, bq2560x_reg[i]);
	}
	//pr_debug("\n");

    return 0;
}


static unsigned int bq2560x_charging_hw_init(struct charger_device *chg_dev)
{
	unsigned int status = 0;

	bq2560x_set_en_hiz(0x0);
	bq2560x_set_vindpm(0x7);	/* VIN DPM check 4.6V */
	bq2560x_set_reg_rst(0x0);
	bq2560x_set_wdt_rst(0x1);	/* Kick watchdog */
	bq2560x_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	bq2560x_set_iprechg(0x8);	/* Precharge current 540mA */
	bq2560x_set_iterm(0x1);	/* Termination current 120mA */

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	bq2560x_set_vreg(0x11);	/* VREG 4.400V */
#else
	bq2560x_set_vreg(0x0B);	/* VREG 4.208V */
#endif

	bq2560x_set_batlowv(0x1);	/* BATLOWV 3.0V */
	bq2560x_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	bq2560x_set_en_term(0x1);	/* Enable termination */
//	bq2560x_set_stat_ctrl(0x0);	/* Enable STAT pin function */
	bq2560x_set_watchdog(0x0);	/* set 0x0 disable timer ,set 0x1 WDT 40s */
	bq2560x_set_en_timer(0x0);	/* Enable charge timer */
	//bq2560x_set_chg_timer(0x02);	/*set charge time 12h*/
	bq2560x_set_int_mask(0x0);	/* Disable fault interrupt */
	bq2560x_set_batfet_reset_enable(0);
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	if (wireless_charger_gpio_number != 0) {
		mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
		mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
	}
#endif

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
	mt_set_gpio_mode(vin_sel_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(vin_sel_gpio_number, 0);	/* 0: input, 1: output */
#endif
	return status;
}

static int bq2560x_charging_enable(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;

	if (en) {
		bq2560x_set_en_hiz(0x0);
		bq2560x_set_chg_config(0x1);	/* charger enable */
	} else {
#if defined(CONFIG_USB_MTK_HDRC_HCD)
        if (mt_usb_is_device())
#endif
		{
			bq2560x_set_chg_config(0x0);
			bq2560x_set_en_hiz(0x1);
		}
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		bq2560x_set_chg_config(0x0);
		bq2560x_set_en_hiz(0x1);	/* disable power path */
#endif
	}

	return status;
}

static int bq2560x_charging_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	unsigned int status = 0;
	/* unsigned int array_size; */
	/* unsigned char reg_value; */
	unsigned char ret_val = 0;

	/* Get current level */
	bq2560x_read_interface(bq2560x_CON2, &ret_val, CON2_ICHG_MASK, CON2_ICHG_SHIFT);

	/* Parsing */
	ret_val = (ret_val * 60) ;

		/* Get current level */
		/* array_size = GETARRAYNUM(CS_VTH); */
		/* *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value); */
	*ichg = ret_val;

	return status;
}

static int bq2560x_charging_set_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size, set_chr_current);
	bq2560x_set_ichg(register_value);
	//charging_dump_register(1);

	return status;
}

static int bq2560x_charging_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	
	//pr_debug("bq2560x_charging_set_cc_input_current set_cc_current=%d\n", current_value);

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size, set_chr_current);
	bq2560x_set_iinlim(register_value);

	return status;
}

static int bq2560x_charging_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	unsigned int status = 0;
	unsigned int array_size;
	unsigned int set_cv_voltage;
	unsigned short register_value;
	unsigned int cv_value = cv;
	static short pre_register_value = -1;

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	/* highest of voltage will be 4.3V, because powerpath limitation */
	if (cv_value >= 4300000)
		cv_value = 4400000;
#endif

	/* use nearest value */
	if (4200000 == cv_value)
		cv_value = 4208000;

	array_size = GETARRAYNUM(VBAT_CV_VTH);
	//pr_debug("bq2560x_charging_set_cv_voltage set_cv_voltage=%d\n", cv);
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv_value);
	//pr_debug("bq2560x_charging_set_cv_voltage set_cv_voltage=%d\n", set_cv_voltage);
	//battery_set_cv_voltage(set_cv_voltage);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size, set_cv_voltage);
	//pr_debug("bq2560x_charging_set_cv_voltage register_value=0x%x\n", register_value);
	bq2560x_set_vreg(register_value);
	if (pre_register_value != register_value)
		bq2560x_set_chg_config(1);
	pre_register_value = register_value;
	return status;
}

static int bq2560x_charging_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	unsigned int status = 0;

	pr_debug("bq2560x_charging_reset_watch_dog_timer\r\n");

	bq2560x_set_wdt_rst(0x1);	/* Kick watchdog */

	return status;
}

static int bq2560x_charging_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	unsigned int status = 0;
	unsigned int ret_val;

	ret_val = bq2560x_get_chrg_stat();

	if (ret_val == 0x3)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int bq2560x_enable_otg(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;

	pr_debug("bq2560x_enable_otg\r\n");

	bq2560x_set_otg_config(en);

	return status;
}

#if 0
static unsigned int bq2560x_set_boost_current_limit(struct charger_device *chg_dev, u32 current_limit)
{
	unsigned int status = 0;

	pr_debug("bq2560x_set_boost_current_limit\r\n");

	bq2560x_set_boost_lim(current_limit);

	return status;
}
#endif

static int bq2560x_set_pep_current_pattern(struct charger_device *chg_dev, bool is_increase)
{
	bool charging_status = false;

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    unsigned int cv_voltage = 4400000;
#else
	unsigned int cv_voltage = 4200000;
#endif

	bq2560x_charging_get_charging_status(chg_dev, &charging_status);
	if (false == charging_status) {
		bq2560x_charging_set_cv_voltage(chg_dev, cv_voltage);	/* Set CV */
		bq2560x_set_ichg(0x0);	/* Set charging current 500ma */
		bq2560x_set_chg_config(0x1);	/* Enable Charging */
	}

	if (is_increase) {
		bq2560x_set_iinlim(0x0);	/* 100mA */
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 1");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 1");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 2");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 2");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 3");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 3");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 4");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 4");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 5");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 5");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 6");
		msleep(485);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 6");
		msleep(50);

		pr_debug("mtk_ta_increase() end\n");

		bq2560x_set_iinlim(0x2);	/* 500mA */
		msleep(200);
	} else {
		bq2560x_set_iinlim(0x0);	/* 100mA */
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 1");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 1");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 2");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 2");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 3");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 3");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 4");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 4");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 5");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 5");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 6");
		msleep(485);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 6");
		msleep(50);

		pr_debug("mtk_ta_decrease() end\n");

		bq2560x_set_iinlim(0x2);	/* 500mA */
	}

	return 0;
}

static int bq2560x_do_event(struct charger_device *chg_dev, unsigned int event, unsigned int args)
{
    if (chg_dev == NULL)
        return -EINVAL;

    pr_info("%s: event = %d\n", __func__, event);

    switch (event) {
    case EVENT_EOC:
        charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
        break;
    case EVENT_RECHARGE:
        charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
        break;
    default:
        break;
    }

    return 0;
}

static struct charger_ops bq2560x_chg_ops = {
    /* Normal charging */
    .dump_registers = bq2560x_dump_register,
    .enable = bq2560x_charging_enable,
    .get_charging_current = bq2560x_charging_get_current,
    .set_charging_current = bq2560x_charging_set_current,
    .set_input_current = bq2560x_charging_set_input_current,
    //.get_constant_voltage = bq2560x_get_cv,
    .set_constant_voltage = bq2560x_charging_set_cv_voltage,
    .kick_wdt = bq2560x_charging_reset_watch_dog_timer,
    //.set_mivr = bq2560x_set_mivr,
    .is_charging_done = bq2560x_charging_get_charging_status,
    //.get_min_charging_current = bq2560x_get_min_ichg,
    //.set_eoc_current = bq2560x_set_ieoc,
    //.enable_termination = bq2560x_enable_te,
    //.run_aicl = bq2560x_run_aicl,
    //.reset_eoc_state = bq2560x_reset_eoc_state,

    /* Safety timer */
    //.enable_safety_timer = bq2560x_enable_safety_timer,
    //.is_safety_timer_enabled = bq2560x_is_safety_timer_enable,

    /* Power path */
    //.enable_powerpath = bq2560x_enable_power_path,
    //.is_powerpath_enabled = bq2560x_is_power_path_enable,

    /* OTG */
    .enable_otg = bq2560x_enable_otg,
    //.set_boost_current_limit = bq2560x_set_boost_current_limit,
    //.enable_discharge = bq2560x_enable_discharge,

    /* PE+/PE+20 */
    .send_ta_current_pattern = bq2560x_set_pep_current_pattern,
    //.set_pe20_efficiency_table = bq2560x_set_pep20_efficiency_table,
    //.send_ta20_current_pattern = bq2560x_set_pep20_current_pattern,
    //.set_ta20_reset = bq2560x_set_pep20_reset,
    //.enable_cable_drop_comp = bq2560x_enable_cable_drop_comp,

    /* ADC */
    //.get_tchg_adc = bq2560x_get_tchg,

    /* Event */
    .event = bq2560x_do_event,
};

static const struct charger_properties bq2560x_chg_props = {
    .alias_name = "bq2560x",
};

static int bq2560x_parse_dt(struct bq2560x_info *info, struct device *dev)
{
    struct device_node *np = dev->of_node;

    pr_info("%s\n", __func__);

    if (!np) {
        pr_err("%s: no of node\n", __func__);
        return -ENODEV;
    }

    if (of_property_read_string(np, "charger_name", &info->chg_dev_name) < 0) {
        info->chg_dev_name = "primary_chg";
        pr_warn("%s: no charger name\n", __func__);
    }

    return 0;
}


static int bq2560x_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
    struct bq2560x_info *info = NULL;

	//pr_debug("[bq2560x_driver_probe]\n");
    info = devm_kzalloc(&client->dev, sizeof(struct bq2560x_info), GFP_KERNEL);
    if (!info)
        return -ENOMEM;
    new_client = client;
    info->dev = &client->dev;
    ret = bq2560x_parse_dt(info, &client->dev);
    if (ret < 0)
        return ret;

    /* Register charger device */
    info->chg_dev = charger_device_register(info->chg_dev_name,
        &client->dev, info, &bq2560x_chg_ops, &bq2560x_chg_props);

    if (IS_ERR_OR_NULL(info->chg_dev)) {
        pr_err("%s: register charger device failed\n", __func__);
        ret = PTR_ERR(info->chg_dev);
        return ret;
    }

    bq2560x_charging_hw_init(info->chg_dev);

	bq2560x_dump_register(info->chg_dev);
	chargin_hw_init_done = true;

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq2560x = 0;
static ssize_t show_bq2560x_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_debug("[show_bq2560x_access] 0x%x\n", g_reg_value_bq2560x);
	return sprintf(buf, "%u\n", g_reg_value_bq2560x);
}

static ssize_t store_bq2560x_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_debug("[store_bq2560x_access]\n");

	if (buf != NULL && size != 0) {
		pr_debug("[store_bq2560x_access] buf is %s and size is %zu\n", buf, size);
		reg_address = simple_strtoul(buf, &pvalue, 16);

		if (size > 3) {
			reg_value = simple_strtoul((pvalue + 1), NULL, 16);
			pr_debug
			    ("[store_bq2560x_access] write bq2560x reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = bq2560x_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq2560x_read_interface(reg_address, &g_reg_value_bq2560x, 0xFF, 0x0);
			pr_debug("[store_bq2560x_access] read bq2560x reg 0x%x with value 0x%x !\n",
				 reg_address, g_reg_value_bq2560x);
			pr_debug
			    ("[store_bq2560x_access] Please use \"cat bq2560x_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(bq2560x_access, 0664, show_bq2560x_access, store_bq2560x_access);	/* 664 */

static int bq2560x_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	pr_debug("******** bq2560x_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq2560x_access);

	return 0;
}

struct platform_device bq2560x_user_space_device = {
	.name = "bq2560x-user",
	.id = -1,
};

static struct platform_driver bq2560x_user_space_driver = {
	.probe = bq2560x_user_space_probe,
	.driver = {
		   .name = "bq2560x-user",
		   },
};


static int __init bq2560x_subsys_init(void)
{
	int ret = 0;



	if (i2c_add_driver(&bq2560x_driver) != 0)
		pr_debug("[bq2560x_init] failed to register bq2560x i2c driver.\n");
	else
		pr_debug("[bq2560x_init] Success to register bq2560x i2c driver.\n");


	/* bq2560x user space access interface */
	ret = platform_device_register(&bq2560x_user_space_device);
	if (ret) {
		pr_debug("****[bq2560x_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&bq2560x_user_space_driver);
	if (ret) {
		pr_debug("****[bq2560x_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit bq2560x_exit(void)
{
	i2c_del_driver(&bq2560x_driver);
}

/* module_init(bq2560x_init); */
/* module_exit(bq2560x_exit); */
subsys_initcall(bq2560x_subsys_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq2560x Driver");
MODULE_AUTHOR("YT Lee<yt.lee@mediatek.com>");
