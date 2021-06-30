/*
 * Copyright (C) 2016 MediaTek Inc.
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

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sched.h>

#include <linux/printk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/i2c.h>

#include "i2c-mtk.h"
#include <m4u.h>

#include "ccu_cmn.h"
#include "ccu_i2c.h"
#include "ccu_i2c_hw.h"

/*******************************************************************************
*
********************************************************************************/
#define MAX_I2C_CMD_LEN 255
#define CCU_I2C_APDMA_TXLEN 128
#define CCU_I2C_MAIN_HW_DRVNAME  "ccu_i2c_main_hwtrg"
#define CCU_I2C_SUB_HW_DRVNAME  "ccu_i2c_sub_hwtrg"

/*--todo: check if need id-table & name, id of_match_table is given*/
static int ccu_i2c_probe_main(struct i2c_client *client, const struct i2c_device_id *id);
static int ccu_i2c_probe_sub(struct i2c_client *client, const struct i2c_device_id *id);
static int ccu_i2c_remove(struct i2c_client *client);
static struct i2c_client *getCcuI2cClient(void);
static inline u32 i2c_readl_dma(struct mt_i2c *i2c, u8 offset);
static inline void i2c_writel_dma(u32 value, struct mt_i2c *i2c, u8 offset);
static inline u16 i2c_readw(struct mt_i2c *i2c, u8 offset);
static inline void i2c_writew(u16 value, struct mt_i2c *i2c, u8 offset);
static void ccu_record_i2c_dma_info(struct mt_i2c *i2c);
static void ccu_i2c_dump_info(struct mt_i2c *i2c);
static int ccu_reset_i2c_apdma(struct mt_i2c *i2c);

static enum CCU_I2C_CHANNEL g_ccuI2cChannel = CCU_I2C_CHANNEL_UNDEF;
static struct i2c_client *g_ccuI2cClientMain;
static struct i2c_client *g_ccuI2cClientSub;
static struct i2c_dma_info g_dma_reg;
static struct i2c_msg ccu_i2c_msg[MAX_I2C_CMD_LEN];
static MBOOL ccu_i2c_enabled = MFALSE;
static MBOOL ccu_i2c_locked  = MFALSE;

static const struct i2c_device_id ccu_i2c_main_ids[] = { {CCU_I2C_MAIN_HW_DRVNAME, 0}, {} };
static const struct i2c_device_id ccu_i2c_sub_ids[] = { {CCU_I2C_SUB_HW_DRVNAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id ccu_i2c_main_driver_of_ids[] = {
	{.compatible = "mediatek,ccu_sensor_i2c_main_hw",},
	{}
};

static const struct of_device_id ccu_i2c_sub_driver_of_ids[] = {
	{.compatible = "mediatek,ccu_sensor_i2c_sub_hw",},
	{}
};
#endif

struct i2c_driver ccu_i2c_main_driver = {
	.probe = ccu_i2c_probe_main,
	.remove = ccu_i2c_remove,
	.driver = {
		   .name = CCU_I2C_MAIN_HW_DRVNAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = ccu_i2c_main_driver_of_ids,
#endif
		   },
	.id_table = ccu_i2c_main_ids,
};

struct i2c_driver ccu_i2c_sub_driver = {
	.probe = ccu_i2c_probe_sub,
	.remove = ccu_i2c_remove,
	.driver = {
		   .name = CCU_I2C_SUB_HW_DRVNAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = ccu_i2c_sub_driver_of_ids,
#endif
		   },
	.id_table = ccu_i2c_sub_ids,
};

/*---------------------------------------------------------------------------*/
/* CCU Driver: i2c driver funcs                                              */
/*---------------------------------------------------------------------------*/
static int ccu_i2c_probe_main(struct i2c_client *client, const struct i2c_device_id *id)
{
	/*int i4RetValue = 0;*/
	LOG_DBG("[ccu_i2c_probe] Attach I2C for HW trriger g_ccuI2cClientMain %p\n", client);

	/* get sensor i2c client */
	/*--todo: add subcam implementation*/
	g_ccuI2cClientMain = client;

	/* set I2C clock rate */
	/*#ifdef CONFIG_MTK_I2C_EXTENSION*/
	/*g_pstI2Cclient3->timing = 100;*/ /* 100k */
	/*g_pstI2Cclient3->ext_flag &= ~I2C_POLLING_FLAG;*/ /* No I2C polling busy waiting */
	/*#endif*/

	LOG_DBG("[ccu_i2c_probe] Attached!!\n");
	return 0;
}

static int ccu_i2c_probe_sub(struct i2c_client *client, const struct i2c_device_id *id)
{
	/*int i4RetValue = 0;*/
	LOG_DBG("[ccu_i2c_probe] Attach I2C for HW trriger g_ccuI2cClientSub %p\n", client);

	g_ccuI2cClientSub = client;

	LOG_DBG("[ccu_i2c_probe] Attached!!\n");
	return 0;
}


static int ccu_i2c_remove(struct i2c_client *client)
{
	return 0;
}

/*---------------------------------------------------------------------------*/
/* CCU i2c public funcs                                                      */
/*---------------------------------------------------------------------------*/
int ccu_i2c_register_driver(void)
{
	int i2c_ret = 0;

	LOG_DBG("i2c_add_driver(&ccu_i2c_main_driver)++\n");
	i2c_ret = i2c_add_driver(&ccu_i2c_main_driver);
	LOG_DBG("i2c_add_driver(&ccu_i2c_main_driver), ret: %d--\n", i2c_ret);
	LOG_DBG("i2c_add_driver(&ccu_i2c_sub_driver)++\n");
	i2c_ret = i2c_add_driver(&ccu_i2c_sub_driver);
	LOG_DBG("i2c_add_driver(&ccu_i2c_sub_driver), ret: %d--\n", i2c_ret);

	return 0;
}

int ccu_i2c_delete_driver(void)
{
	i2c_del_driver(&ccu_i2c_main_driver);
	i2c_del_driver(&ccu_i2c_sub_driver);

	return 0;
}

int ccu_i2c_set_channel(enum CCU_I2C_CHANNEL channel)
{
	if ((channel == CCU_I2C_CHANNEL_MAINCAM) || (channel == CCU_I2C_CHANNEL_SUBCAM)) {
		g_ccuI2cChannel = channel;
		return 0;
	} else
		return -EFAULT;
}

int ccu_i2c_frame_reset(void)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;

	pClient = getCcuI2cClient();
	i2c = i2c_get_adapdata(pClient->adapter);

	ccu_reset_i2c_apdma(i2c);

	/*--todo:remove dump log on production*/
	/*ccu_record_i2c_dma_info(i2c);*/

	i2c_writew(I2C_FIFO_ADDR_CLR, i2c, OFFSET_FIFO_ADDR_CLR);
	i2c_writew(I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP, i2c, OFFSET_INTR_MASK);

	/**/
	mb();
	/*--todo:remove dump log on production*/
	/*ccu_i2c_dump_info(i2c);*/

	return 0;
}


int ccu_trigger_i2c(int transac_len, MBOOL do_dma_en)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;

	u8 *dmaBufVa;

	pClient = getCcuI2cClient();
	i2c = i2c_get_adapdata(pClient->adapter);

	dmaBufVa = i2c->dma_buf.vaddr;

	/*LOG_DBG("i2c_dma_buf_content: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",*/
	/*  dmaBufVa[0],dmaBufVa[1],dmaBufVa[2],dmaBufVa[3],dmaBufVa[4],dmaBufVa[5],*/
	/*dmaBufVa[18],dmaBufVa[19],dmaBufVa[20],dmaBufVa[21],dmaBufVa[22],dmaBufVa[23]);*/

	/*set i2c transaction length & enable apdma*/
	i2c_writew(transac_len, i2c, OFFSET_TRANSAC_LEN);

	/*ccu_record_i2c_dma_info(i2c);*/

	i2c_writel_dma(I2C_DMA_START_EN, i2c, OFFSET_EN);

	/*ccu_i2c_dump_info(i2c);*/

	/*trigger i2c start from n3d_a*/
	ccu_trigger_i2c_hw(g_ccuI2cChannel, transac_len, do_dma_en);

	/*ccu_i2c_dump_info(i2c);*/

	return 0;
}


int ccu_config_i2c_buf_mode(int transfer_len)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;

	pClient = getCcuI2cClient();
	i2c = i2c_get_adapdata(pClient->adapter);

	/*write i2c controller tx len*/
	i2c->total_len = transfer_len;
	i2c->msg_len = transfer_len;
	i2c_writew(transfer_len, i2c, OFFSET_TRANSFER_LEN);

	/*ccu_reset_i2c_apdma(i2c);*/

	ccu_record_i2c_dma_info(i2c);

	/*flush before sending DMA start*/
	/*mb();*/
	/*i2c_writel_dma(I2C_DMA_START_EN, i2c, OFFSET_EN);*/

	ccu_i2c_frame_reset();

	ccu_i2c_dump_info(i2c);

	return 0;
}


/*--todo: add subcam implementation*/

int ccu_init_i2c_buf_mode(u16 i2cId)
{
	int ret = 0;
	unsigned char dummy_data[] = {
		0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18
	};

	struct i2c_client *pClient = NULL;
	int trans_num = 1;

	memset(ccu_i2c_msg, 0, MAX_I2C_CMD_LEN * sizeof(struct i2c_msg));

	pClient = getCcuI2cClient();

	/*for(i = 0 ; i<trans_num ; i++){*/
	ccu_i2c_msg[0].addr = i2cId >> 1;
	ccu_i2c_msg[0].flags = 0;
	ccu_i2c_msg[0].len = 16;
	ccu_i2c_msg[0].buf = dummy_data;
	/*}*/

	ret = hw_trig_i2c_transfer(pClient->adapter, ccu_i2c_msg, trans_num);
	return ret;
}


static int ccu_i2c_seize_controller(void)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c = NULL;

	pClient = getCcuI2cClient();

	LOG_DBG("pClient: %p\n", pClient);

	if (pClient == NULL) {
		LOG_ERR("i2c client is null");
		return -1;
	}

	i2c = i2c_get_adapdata(pClient->adapter);
	/*mutex_lock(&i2c->i2c_mutex);*/
	disable_irq(i2c->irqnr);
	ccu_i2c_locked = MTRUE;

	/*LOG_INF_MUST("ccu i2c controller locked");*/
	LOG_INF_MUST("i2c controller irq disabled by ccu");
	return 0;
}

static int ccu_i2c_release_controller(void)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c = NULL;

	if (ccu_i2c_locked == MTRUE) {
		pClient = getCcuI2cClient();

		LOG_DBG("pClient: %p\n", pClient);

		if (pClient == NULL) {
			LOG_ERR("i2c client is null\n");
			return -1;
		}

		i2c = i2c_get_adapdata(pClient->adapter);
		enable_irq(i2c->irqnr);
		/*mutex_unlock(&i2c->i2c_mutex);*/
		ccu_i2c_locked = MFALSE;
		/*LOG_INF_MUST("ccu i2c controller unlocked\n");*/
		LOG_INF_MUST("i2c controller irq enabled by ccu");
	} else {
		/*LOG_INF_MUST("ccu i2c controller not in locked status, skip unlock\n");*/
		LOG_INF_MUST("i2c controller irq is not in disable status, skip enable\n");
	}

	return 0;
}

int ccu_i2c_buf_mode_init(unsigned char i2c_write_id, int transfer_len)
{
	if (ccu_i2c_buf_mode_en(1) == -1) {
		LOG_DBG("i2c_buf_mode_en fail\n");
		return -1;
	}

	/*to set i2c buffer mode configuration*/
	/*Transfer a dummy data, must>8 to let i2c drv go to dma mode*/
	ccu_init_i2c_buf_mode(i2c_write_id);
	ccu_config_i2c_buf_mode(transfer_len);
	ccu_i2c_seize_controller();
	LOG_DBG_MUST("ccu_i2c_buf_mode_init done.\n");

	return 0;
}

int ccu_i2c_buf_mode_en(int enable)
{
	int ret = 0;
	struct i2c_client *pClient = NULL;

	LOG_DBG_MUST("i2c_buf_mode_en %d\n", enable);

	pClient = getCcuI2cClient();

	LOG_DBG("i2c_buf_mode_en, pClient: %p\n", pClient);

	if (pClient == NULL) {
		LOG_ERR("i2c_client is null\n");
		return -1;
	}

	LOG_DBG_MUST("ccu_i2c_enabled %d\n", ccu_i2c_enabled);

	if (enable) {
		if (ccu_i2c_enabled == MFALSE) {
			ret = hw_trig_i2c_enable(pClient->adapter);
			ccu_i2c_enabled = MTRUE;
			LOG_DBG_MUST("hw_trig_i2c_enable done.\n");
		}
	} else {
		if (ccu_i2c_enabled == MTRUE) {
			ccu_i2c_release_controller();
			ret = hw_trig_i2c_disable(pClient->adapter);
			ccu_i2c_enabled = MFALSE;

			LOG_DBG_MUST("hw_trig_i2c_disable done.\n");
		}
	}
	return ret;
}

int i2c_get_dma_buffer_addr(void **va, uint32_t *pa_h, uint32_t *pa_l, uint32_t *i2c_id)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;

	pClient = getCcuI2cClient();

	if (pClient == MNULL) {
		LOG_ERR("ccu client is NULL");
		return -EFAULT;
	}

	i2c = i2c_get_adapdata(pClient->adapter);

	/*i2c_get_dma_buffer_addr_imp(pClient->adapter ,va);*/
	*va = i2c->dma_buf.vaddr;
	*pa_l = i2c->dma_buf.paddr;
	*pa_h = (i2c->dma_buf.paddr >> 32);
	*i2c_id = i2c->id;
	LOG_DBG_MUST("got i2c buf: %p %d %d %d\n", *va, *pa_l, *pa_h, *i2c_id);

	return 0;
}

/*---------------------------------------------------------------------------*/
/* CCU i2c static funcs                                              */
/*---------------------------------------------------------------------------*/
static struct i2c_client *getCcuI2cClient(void)
{
	switch (g_ccuI2cChannel) {
	case CCU_I2C_CHANNEL_MAINCAM:
		{
			return g_ccuI2cClientMain;
		}
	case CCU_I2C_CHANNEL_SUBCAM:
		{
			return g_ccuI2cClientSub;
		}
	default:
		{
			return MNULL;
		}
	}
}

static inline u32 i2c_readl_dma(struct mt_i2c *i2c, u8 offset)
{
	return readl(i2c->pdmabase + offset);
}

static inline void i2c_writel_dma(u32 value, struct mt_i2c *i2c, u8 offset)
{
	writel(value, i2c->pdmabase + offset);
}

static inline u16 i2c_readw(struct mt_i2c *i2c, u8 offset)
{
	return readw(i2c->base + offset);
}

static inline void i2c_writew(u16 value, struct mt_i2c *i2c, u8 offset)
{
	writew(value, i2c->base + offset);
}

static void ccu_record_i2c_dma_info(struct mt_i2c *i2c)
{
	g_dma_reg.base = (unsigned long)i2c->pdmabase;
	g_dma_reg.int_flag = i2c_readl_dma(i2c, OFFSET_INT_FLAG);
	g_dma_reg.int_en = i2c_readl_dma(i2c, OFFSET_INT_EN);
	g_dma_reg.en = i2c_readl_dma(i2c, OFFSET_EN);
	g_dma_reg.rst = i2c_readl_dma(i2c, OFFSET_RST);
	g_dma_reg.stop = i2c_readl_dma(i2c, OFFSET_STOP);
	g_dma_reg.flush = i2c_readl_dma(i2c, OFFSET_FLUSH);
	g_dma_reg.con = i2c_readl_dma(i2c, OFFSET_CON);
	g_dma_reg.tx_mem_addr = i2c_readl_dma(i2c, OFFSET_TX_MEM_ADDR);
	g_dma_reg.rx_mem_addr = i2c_readl_dma(i2c, OFFSET_RX_MEM_ADDR);
	g_dma_reg.tx_len = i2c_readl_dma(i2c, OFFSET_TX_LEN);
	g_dma_reg.rx_len = i2c_readl_dma(i2c, OFFSET_RX_LEN);
	g_dma_reg.int_buf_size = i2c_readl_dma(i2c, OFFSET_INT_BUF_SIZE);
	g_dma_reg.debug_sta = i2c_readl_dma(i2c, OFFSET_DEBUG_STA);
	g_dma_reg.tx_mem_addr2 = i2c_readl_dma(i2c, OFFSET_TX_MEM_ADDR2);
	g_dma_reg.rx_mem_addr2 = i2c_readl_dma(i2c, OFFSET_RX_MEM_ADDR2);
}

void ccu_i2c_dump_errr(void)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;

	LOG_DBG_MUST("CCU Dump I2C reg\n");

	pClient = getCcuI2cClient();
	i2c = i2c_get_adapdata(pClient->adapter);
	ccu_i2c_dump_info(i2c);
}

static void ccu_i2c_dump_info(struct mt_i2c *i2c)
{
	/* I2CFUC(); */
	/* int val=0; */
	pr_info("i2c_dump_info ++++++++++++++++++++++++++++++++++++++++++\n");
	pr_info("I2C structure:\n"
	       I2CTAG "Clk=%d,Id=%d,Op=%x,Irq_stat=%x,Total_len=%x\n"
	       I2CTAG "Trans_len=%x,Trans_num=%x,Trans_auxlen=%x,speed=%d\n"
	       I2CTAG "Trans_stop=%u\n",
	       15600, i2c->id, i2c->op, i2c->irq_stat, i2c->total_len,
	       i2c->msg_len, 1, i2c->msg_aux_len, i2c->speed_hz, i2c->trans_stop);

	pr_info("base address 0x%p\n", i2c->base);
	pr_info("I2C register:\n"
	       I2CTAG "SLAVE_ADDR=%x,INTR_MASK=%x,INTR_STAT=%x,CONTROL=%x,TRANSFER_LEN=%x\n"
	       I2CTAG "TRANSAC_LEN=%x,DELAY_LEN=%x,TIMING=%x,START=%x,FIFO_STAT=%x\n"
	       I2CTAG "IO_CONFIG=%x,HS=%x,DCM_EN=%x,DEBUGSTAT=%x,EXT_CONF=%x,TRANSFER_LEN_AUX=%x\n",
	       (i2c_readw(i2c, OFFSET_SLAVE_ADDR)),
	       (i2c_readw(i2c, OFFSET_INTR_MASK)),
	       (i2c_readw(i2c, OFFSET_INTR_STAT)),
	       (i2c_readw(i2c, OFFSET_CONTROL)),
	       (i2c_readw(i2c, OFFSET_TRANSFER_LEN)),
	       (i2c_readw(i2c, OFFSET_TRANSAC_LEN)),
	       (i2c_readw(i2c, OFFSET_DELAY_LEN)),
	       (i2c_readw(i2c, OFFSET_TIMING)),
	       (i2c_readw(i2c, OFFSET_START)),
	       (i2c_readw(i2c, OFFSET_FIFO_STAT)),
	       (i2c_readw(i2c, OFFSET_IO_CONFIG)),
	       (i2c_readw(i2c, OFFSET_HS)),
	       (i2c_readw(i2c, OFFSET_DCM_EN)),
	       (i2c_readw(i2c, OFFSET_DEBUGSTAT)),
	       (i2c_readw(i2c, OFFSET_EXT_CONF)), (i2c_readw(i2c, OFFSET_TRANSFER_LEN_AUX)));

	pr_info("before enable DMA register(0x%lx):\n"
	       I2CTAG "INT_FLAG=%x,INT_EN=%x,EN=%x,RST=%x,\n"
	       I2CTAG "STOP=%x,FLUSH=%x,CON=%x,TX_MEM_ADDR=%x, RX_MEM_ADDR=%x\n"
	       I2CTAG "TX_LEN=%x,RX_LEN=%x,INT_BUF_SIZE=%x,DEBUG_STATUS=%x\n"
	       I2CTAG "TX_MEM_ADDR2=%x, RX_MEM_ADDR2=%x\n",
	       g_dma_reg.base,
	       g_dma_reg.int_flag,
	       g_dma_reg.int_en,
	       g_dma_reg.en,
	       g_dma_reg.rst,
	       g_dma_reg.stop,
	       g_dma_reg.flush,
	       g_dma_reg.con,
	       g_dma_reg.tx_mem_addr,
	       g_dma_reg.tx_mem_addr,
	       g_dma_reg.tx_len,
	       g_dma_reg.rx_len,
	       g_dma_reg.int_buf_size, g_dma_reg.debug_sta,
	       g_dma_reg.tx_mem_addr2, g_dma_reg.tx_mem_addr2);
	pr_info("DMA register(0x%p):\n"
	       I2CTAG "INT_FLAG=%x,INT_EN=%x,EN=%x,RST=%x,\n"
	       I2CTAG "STOP=%x,FLUSH=%x,CON=%x,TX_MEM_ADDR=%x, RX_MEM_ADDR=%x\n"
	       I2CTAG "TX_LEN=%x,RX_LEN=%x,INT_BUF_SIZE=%x,DEBUG_STATUS=%x\n"
	       I2CTAG "TX_MEM_ADDR2=%x, RX_MEM_ADDR2=%x\n",
	       i2c->pdmabase,
	       (i2c_readl_dma(i2c, OFFSET_INT_FLAG)),
	       (i2c_readl_dma(i2c, OFFSET_INT_EN)),
	       (i2c_readl_dma(i2c, OFFSET_EN)),
	       (i2c_readl_dma(i2c, OFFSET_RST)),
	       (i2c_readl_dma(i2c, OFFSET_STOP)),
	       (i2c_readl_dma(i2c, OFFSET_FLUSH)),
	       (i2c_readl_dma(i2c, OFFSET_CON)),
	       (i2c_readl_dma(i2c, OFFSET_TX_MEM_ADDR)),
	       (i2c_readl_dma(i2c, OFFSET_RX_MEM_ADDR)),
	       (i2c_readl_dma(i2c, OFFSET_TX_LEN)),
	       (i2c_readl_dma(i2c, OFFSET_RX_LEN)),
	       (i2c_readl_dma(i2c, OFFSET_INT_BUF_SIZE)),
	       (i2c_readl_dma(i2c, OFFSET_DEBUG_STA)),
	       (i2c_readl_dma(i2c, OFFSET_TX_MEM_ADDR2)),
	       (i2c_readl_dma(i2c, OFFSET_RX_MEM_ADDR2)));
	pr_info("i2c_dump_info ------------------------------------------\n");

}

/*do i2c apdma warm reset & re-write dma buf addr, txlen*/
static int ccu_reset_i2c_apdma(struct mt_i2c *i2c)
{
	i2c_writel_dma(I2C_DMA_WARM_RST, i2c, OFFSET_RST);

#ifdef CONFIG_MTK_LM_MODE
	if ((i2c->dev_comp->dma_support == 1) && (enable_4G())) {
		i2c_writel_dma(0x1, i2c, OFFSET_TX_MEM_ADDR2);
		i2c_writel_dma(0x1, i2c, OFFSET_RX_MEM_ADDR2);
	}
#endif

	i2c_writel_dma(I2C_DMA_INT_FLAG_NONE, i2c, OFFSET_INT_FLAG);
	i2c_writel_dma(I2C_DMA_CON_TX, i2c, OFFSET_CON);
	i2c_writel_dma((u32) i2c->dma_buf.paddr, i2c, OFFSET_TX_MEM_ADDR);
	if ((i2c->dev_comp->dma_support >= 2))
		i2c_writel_dma(i2c->dma_buf.paddr >> 32, i2c, OFFSET_TX_MEM_ADDR2);

	/*write ap mda tx len = 128(must > totoal tx len within a frame)*/
	i2c_writel_dma(CCU_I2C_APDMA_TXLEN, i2c, OFFSET_TX_LEN);

	return 0;
}
