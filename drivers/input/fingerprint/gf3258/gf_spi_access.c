/* Goodix's GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206
 *  fingerprint sensor linux driver for factory test
 *
 * 2010 - 2015 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

/* MTK header */
//#include "mt_spi.h"
//#include "mt_spi_hal.h"
#include "mtk_gpio.h"
//#include "mach/gpio_const.h"

#include "gf_common.h"

#define SPI_CLK_TOTAL_TIME  107

extern u8 g_debug_level;

#if GF358
#define GF_W          	0xF0
#define GF_R          	0xF1
#define GF_WDATA_OFFSET	(0x3)
#define GF_RDATA_OFFSET	(0x5)

#define MTK_SPI_ALIGN_MASK_NUM  10
#define MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_MASK_NUM) - 1)

#endif 
/*
int  gf_ioctl_spi_init_cfg_cmd(struct mt_chip_conf *mcc, unsigned long arg){
    int retval = 0;
    return retval;
}
*/
/* gf_spi_setup_conf_factory, configure spi speed and transfer mode in REE mode
  *
  * speed: 1, 4, 6, 8 unit:MHz
  * mode: DMA mode or FIFO mode
  */
#if 0
void gf_spi_setup_conf_factory(struct gf_device *gf_dev, u32 speed, enum spi_transfer_mode mode)
{
    struct mt_chip_conf *mcc = &gf_dev->spi_mcc;

    switch(speed) {
    case 1:
        /* set to 1MHz clock */
        mcc->high_time = 50;
        mcc->low_time = 50;
        break;
    case 4:
        /* set to 4MHz clock */
        mcc->high_time = 15;
        mcc->low_time = 15;
        break;
    case 6:
        /* set to 6MHz clock */
        mcc->high_time = 10;
        mcc->low_time = 10;
        break;
    case 8:
	case 9:
        /* set to 8MHz clock */
        mcc->high_time = 8;
        mcc->low_time = 8;
        break;
    default:
        /* default set to 1MHz clock */
        mcc->high_time = 50;
        mcc->low_time = 50;
    }

    if ((mode == DMA_TRANSFER) || (mode == FIFO_TRANSFER)) {
        mcc->com_mod = mode;
    } else {
        /* default set to FIFO mode */
        mcc->com_mod = FIFO_TRANSFER;
    }

    if (spi_setup(gf_dev->spi))
        gf_debug(ERR_LOG, "%s, failed to setup spi conf\n", __func__);

}
#endif
static int gf_spi_transfer_raw_ree(struct gf_device *gf_dev, u8 *tx_buf, u8 *rx_buf, u32 len)
{
    struct spi_message msg;
    struct spi_transfer xfer;

    spi_message_init(&msg);
    memset(&xfer, 0, sizeof(struct spi_transfer));

    xfer.tx_buf = tx_buf;
    xfer.rx_buf = rx_buf;
    xfer.len = len;
    spi_message_add_tail(&xfer, &msg);
    spi_sync(gf_dev->spi, &msg);

    return 0;
}

int gf_ioctl_transfer_raw_cmd(struct gf_device *gf_dev, unsigned long arg,unsigned int bufsiz){
    struct gf_ioc_transfer_raw ioc_xraw;
    int retval = 0;

    do {
        u8 *tx_buf;
        u8 *rx_buf;
        uint32_t len;

        //gf_debug(ERR_LOG, "%s:enter\n", __func__);

        if (copy_from_user(&ioc_xraw, (struct gf_ioc_transfer_raw *)arg, sizeof(struct gf_ioc_transfer_raw))) {
            gf_debug(ERR_LOG, "%s: Failed to copy gf_ioc_transfer_raw from user to kernel\n", __func__);
            retval = -EFAULT;
            break;
        }

        //gf_debug(ERR_LOG, "%s:len:%d,read_buf:0x%p,write_buf:%p,high_time:%d,low_time:%d\n", __func__,ioc_xraw.len,ioc_xraw.read_buf,ioc_xraw.write_buf,ioc_xraw.high_time,ioc_xraw.low_time);
        //if ((ioc_xraw.len > bufsiz) || (ioc_xraw.len == 0)) {
        if (ioc_xraw.len == 0) {
            gf_debug(ERR_LOG, "%s: request transfer length larger than maximum buffer\n", __func__);
            retval = -EINVAL;
            break;
        }

        if (ioc_xraw.read_buf == NULL || ioc_xraw.write_buf == NULL) {
            gf_debug(ERR_LOG, "%s: read buf and write buf can not equal to NULL simultaneously.\n", __func__);
            retval = -EINVAL;
            break;
        }

        /* change speed and set transfer mode */
#if 0
        if (ioc_xraw.len > 32) {
            gf_spi_setup_conf_factory(gf_dev, ioc_xraw.high_time, DMA_TRANSFER);
        } else {
            gf_spi_setup_conf_factory(gf_dev, ioc_xraw.high_time, FIFO_TRANSFER);
        }
//#else
        if (ioc_xraw.len > 32) {
            gf_spi_setup_conf_factory(gf_dev, 8,DMA_TRANSFER);
        } else {
            gf_spi_setup_conf_factory(gf_dev, ioc_xraw.high_time,FIFO_TRANSFER);
        }
#endif

        len = ioc_xraw.len;
        if (len % 1024 != 0 && len > 1024){
            len = ((ioc_xraw.len /1024) + 1) * 1024;
        }

        tx_buf = kzalloc(len, GFP_KERNEL);
        if (NULL == tx_buf) {
            gf_debug(ERR_LOG, "%s: failed to allocate raw tx buffer\n", __func__);
            retval = -EMSGSIZE;
            break;
        }

        rx_buf = kzalloc(len, GFP_KERNEL);
        if (NULL == rx_buf) {
            kfree(tx_buf);
            gf_debug(ERR_LOG, "%s: failed to allocate raw rx buffer\n", __func__);
            retval = -EMSGSIZE;
            break;
        }

        if (copy_from_user(tx_buf, ioc_xraw.write_buf, ioc_xraw.len)) {
            kfree(tx_buf);
            kfree(rx_buf);
            gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer from user to kernel\n");
            //gf_debug(ERR_LOG, "%s:Failed to copy gf_ioc_transfer from user to kernel:tx_buf:0x%p,write_buf:0x%p,len:%d\n",__func__,tx_buf, ioc_xraw.write_buf, ioc_xraw.len);
            retval = -EFAULT;
            break;
        }

        gf_spi_transfer_raw_ree(gf_dev, tx_buf, rx_buf, len);

        if (copy_to_user(ioc_xraw.read_buf, rx_buf, ioc_xraw.len)) {
            gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer_raw from kernel to user\n");
            //gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer_raw from kernel to user:0x%p\n",ioc_xraw.read_buf);
            retval = -EFAULT;
        }

        kfree(tx_buf);
        kfree(rx_buf);
    } while(0);

    //gf_debug(ERR_LOG, "%s:exit:%d\n", __func__,retval);
    return retval;
 }


 #if 1 //add for test SPI communication in kernel level start
 int gf_spi_read_bytes(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *rx_buf)
 {
	 struct spi_message msg;
	 struct spi_transfer *xfer = NULL;
	 u8 *tmp_buf = NULL;
	 u32 package, reminder, retry;
 
	 package = (data_len + 2) / 1024;
	 reminder = (data_len + 2) % 1024;
 
	 if ((package > 0) && (reminder != 0)) {
		 xfer = kzalloc(sizeof(*xfer) * 4, GFP_KERNEL);
		 retry = 1;
	 } else {
		 xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
		 retry = 0;
	 }
	 if (xfer == NULL) {
		 gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
		 return -ENOMEM;
	 }
 
	 tmp_buf = gf_dev->spi_buffer;
 
#ifndef CONFIG_SPI_MT65XX
	 /* switch to DMA mode if transfer length larger than 32 bytes */
	 if ((data_len + 1) > 32) {
		 gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
		 spi_setup(gf_dev->spi);
	 }
#endif
	 spi_message_init(&msg);
	 *tmp_buf = 0xF0;
	 *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	 *(tmp_buf + 2) = (u8)(addr & 0xFF);
	 xfer[0].tx_buf = tmp_buf;
	 xfer[0].len = 3;
	 xfer[0].delay_usecs = 5;
	 spi_message_add_tail(&xfer[0], &msg);
	 spi_sync(gf_dev->spi, &msg);
 
	 spi_message_init(&msg);
	 /* memset((tmp_buf + 4), 0x00, data_len + 1); */
	 /* 4 bytes align */
	 *(tmp_buf + 4) = 0xF1;
	 xfer[1].tx_buf = tmp_buf + 4;
	 xfer[1].rx_buf = tmp_buf + 4;
 
	 if (retry)
		 xfer[1].len = package * 1024;
	 else
		 xfer[1].len = data_len + 1;
 
	 xfer[1].delay_usecs = 5;
	 spi_message_add_tail(&xfer[1], &msg);
	 spi_sync(gf_dev->spi, &msg);
 
	 /* copy received data */
	 if (retry)
		 memcpy(rx_buf, (tmp_buf + 5), (package * 1024 - 1));
	 else
		 memcpy(rx_buf, (tmp_buf + 5), data_len);
 
	 /* send reminder SPI data */
	 if (retry) {
		 addr = addr + package * 1024 - 2;
		 spi_message_init(&msg);
 
		 *tmp_buf = 0xF0;
		 *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
		 *(tmp_buf + 2) = (u8)(addr & 0xFF);
		 xfer[2].tx_buf = tmp_buf;
		 xfer[2].len = 3;
		 xfer[2].delay_usecs = 5;
		 spi_message_add_tail(&xfer[2], &msg);
		 spi_sync(gf_dev->spi, &msg);
 
		 spi_message_init(&msg);
		 *(tmp_buf + 4) = 0xF1;
		 xfer[3].tx_buf = tmp_buf + 4;
		 xfer[3].rx_buf = tmp_buf + 4;
		 xfer[3].len = reminder + 1;
		 xfer[3].delay_usecs = 5;
		 spi_message_add_tail(&xfer[3], &msg);
		 spi_sync(gf_dev->spi, &msg);
 
		 memcpy((rx_buf + package * 1024 - 1), (tmp_buf + 6), (reminder - 1));
	 }
 
	 /* restore to FIFO mode if has used DMA */
         
#ifndef CONFIG_SPI_MT65XX
	 if ((data_len + 1) > 32) {
		 gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
		 spi_setup(gf_dev->spi);
	 }
#endif
	 kfree(xfer);
	 if (xfer != NULL)
		 xfer = NULL;
 
	 return 0;
 }

 int gf_spi_write_bytes(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *tx_buf)
 {
	 struct spi_message msg;
	 struct spi_transfer *xfer = NULL;
	 u8 *tmp_buf = NULL;
	 u32 package, reminder, retry;
 
	 package = (data_len + 3) / 1024;
	 reminder = (data_len + 3) % 1024;
 
	 if ((package > 0) && (reminder != 0)) {
		 xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
		 retry = 1;
	 } else {
		 xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
		 retry = 0;
	 }
	 if (xfer == NULL) {
		 gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
		 return -ENOMEM;
	 }
	 tmp_buf = gf_dev->spi_buffer;
 
#ifndef CONFIG_SPI_MT65XX
	 /* switch to DMA mode if transfer length larger than 32 bytes */
	 if ((data_len + 3) > 32) {
		 gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
		 spi_setup(gf_dev->spi);
	 }
#endif
	 spi_message_init(&msg);
	 *tmp_buf = 0xF0;
	 *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	 *(tmp_buf + 2) = (u8)(addr & 0xFF);
	 if (retry) {
		 memcpy(tmp_buf + 3, tx_buf, (package * 1024 - 3));
		 xfer[0].len = package * 1024;
	 } else {
		 memcpy(tmp_buf + 3, tx_buf, data_len);
		 xfer[0].len = data_len + 3;
	 }
	 xfer[0].tx_buf = tmp_buf;
	 xfer[0].delay_usecs = 5;
	 spi_message_add_tail(&xfer[0], &msg);
	 spi_sync(gf_dev->spi, &msg);
 
	 if (retry) {
		 addr = addr + package * 1024 - 3;
		 spi_message_init(&msg);
		 *tmp_buf = 0xF0;
		 *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
		 *(tmp_buf + 2) = (u8)(addr & 0xFF);
		 memcpy(tmp_buf + 3, (tx_buf + package * 1024 - 3), reminder);
		 xfer[1].tx_buf = tmp_buf;
		 xfer[1].len = reminder + 3;
		 xfer[1].delay_usecs = 5;
		 spi_message_add_tail(&xfer[1], &msg);
		 spi_sync(gf_dev->spi, &msg);
	 }
 
#ifndef CONFIG_SPI_MT65XX
	 /* restore to FIFO mode if has used DMA */
	 if ((data_len + 3) > 32) {
		 gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
		 spi_setup(gf_dev->spi);
	 }
#endif
	 kfree(xfer);
	 if (xfer != NULL)
		 xfer = NULL;
 
	 return 0;
 }
#endif //add for test SPI communication in kernel level end

#if GF358

/*
void gf_release_pinctrl(struct gf_dev* gf_dev)
{
	if(gf_dev != NULL)
	{
		devm_pinctrl_put(gf_dev->pinctrl_gpios);
		pr_info("%s, release pinctrl success!\n", __func__);
	}
}

*/
int gf_spi_read_bytes_gf358(struct gf_dev* gf_dev, unsigned short addr, unsigned short data_len,
                      unsigned char* rx_buf)

{

    struct spi_message msg;
    struct spi_transfer *xfer;
    u32 package_num = (data_len + 1 + 1)>>MTK_SPI_ALIGN_MASK_NUM;
    u32 reminder = (data_len + 1 + 1) & MTK_SPI_ALIGN_MASK;
    u8 *one_more_buff = NULL;
    u8 twice = 0;
    int ret = 0;

    one_more_buff = kzalloc(30720, GFP_KERNEL);
    if(one_more_buff == NULL ) {
        pr_info("No memory for one_more_buff.");
        return -ENOMEM;
    }
    xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
    if((package_num > 0) && (reminder != 0)) {
        twice = 1;
        printk("stone package_num is %d reminder is %d\n",package_num,reminder);
    } else {
        twice = 0;
    }
    if( xfer == NULL) {
        pr_info("No memory for command.");
        if(one_more_buff != NULL)
            kfree(one_more_buff);
        return -ENOMEM;
    }

#ifndef CONFIG_SPI_MT65XX
    /*set spi mode.*/
    /* switch to DMA mode if transfer length larger than 32 bytes */
    if ((data_len + 1) > 32)
	gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
    else
	gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
#endif
    spi_setup(gf_dev->spi);
    spi_message_init(&msg);
    /*send GF command to device.*/
    rx_buf[0] = GF_W;
    rx_buf[1] = (u8)((addr >> 8)&0xFF);
    rx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = rx_buf;
    xfer[0].len = 3;
    spi_message_add_tail(&xfer[0], &msg);
    spi_sync(gf_dev->spi, &msg);

    spi_message_init(&msg);
    /*if wanted to read data from GF.
     *Should write Read command to device
     *before read any data from device.
     */
    //memset(rx_buf, 0xff, data_len);
    one_more_buff[0] = GF_R;
    xfer[1].tx_buf = &one_more_buff[0];
    xfer[1].rx_buf = &one_more_buff[0];
    //read 1 additional package to ensure no even data read
    if(twice == 1)
        xfer[1].len = ((package_num+1) << MTK_SPI_ALIGN_MASK_NUM);
    else
        xfer[1].len = data_len + 1;
    spi_message_add_tail(&xfer[1], &msg);
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0) {
        memcpy(rx_buf + GF_RDATA_OFFSET,one_more_buff+1,data_len);
        ret = data_len;
    } else {
        pr_info("gf: read failed. ret = %d", ret);
    }
    if(xfer != NULL) {
        kfree(xfer);
        xfer = NULL;
    }
    if(one_more_buff != NULL) {
        kfree(one_more_buff);
        one_more_buff = NULL;
    }
    return 0;
}



int gf_spi_read_word(struct gf_dev* gf_dev, unsigned short addr, unsigned short* value)
{
    int status = 0;
    u8* buf = NULL;
    mutex_lock(&gf_dev->buf_lock);
    status = gf_spi_read_bytes_gf358(gf_dev, addr, 2, gf_dev->gBuffer);
    buf = gf_dev->gBuffer + GF_RDATA_OFFSET;
    *value = ((unsigned short)(buf[0]<<8)) | buf[1];
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}


#endif //
