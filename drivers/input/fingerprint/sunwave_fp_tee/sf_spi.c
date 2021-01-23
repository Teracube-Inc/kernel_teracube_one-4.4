/**
 * The platform spi device for sunwave's fingerprint sensor.
 *
 * Copyright (C) 2016 Sunwave Corporation. <http://www.sunwavecorp.com>
 * Copyright (C) 2016 Langson Leung <mailto:liangzh@sunwavecorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>

#include <linux/clk.h>
#include <linux/spi/spi.h>

#ifndef CONFIG_SPI_MT65XX
#include "mt_spi.h"
#include "mt_spi_hal.h"
#endif
#include "../../../spi/mediatek/mt6763/mtk_spi.h"

#define SF_SPI_BUS_NUM 1 // TODO:
#define SF_SPI_CS__NUM 0 // TODO:

#define MODULE_NAME "sf_spi"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME": "fmt, ##args)

// TODO: register REE spi device spec.
/*
static struct mt_chip_conf mt_spi_chip_conf = {
    .setuptime    = 20,
    .holdtime     = 20,
    .high_time    = 50,
    .low_time     = 50,
    .cs_idletime  = 10,
    .ulthgh_thrsh =  0,
    .sample_sel   = POSEDGE,
    .cs_pol       = ACTIVE_LOW,
    .cpol         = SPI_CPOL_0,
    .cpha         = SPI_CPHA_0,
    .tx_mlsb      = SPI_MSB,
    .rx_mlsb      = SPI_MSB,
    .tx_endian    = SPI_LENDIAN,
    .rx_endian    = SPI_LENDIAN,
    .com_mod      = DMA_TRANSFER, // FIFO_TRANSFER/DMA_TRANSFER
    .pause        = PAUSE_MODE_ENABLE,
    .finish_intr  = FINISH_INTR_EN,
    .deassert     = DEASSERT_DISABLE,
    .ulthigh      = ULTRA_HIGH_DISABLE,
    .tckdly       = TICK_DLY0,
};
*/
/*
 * REE using the standard 'spidev' driver.
 * CONFIG_SPI_SPIDEV must be set in REE.
 */
/*
static struct spi_board_info spi_desc __initdata = {
    .modalias        = "spidev",
    .bus_num         = SF_SPI_BUS_NUM,
    .chip_select     = SF_SPI_CS__NUM,
    .mode            = SPI_MODE_0,
    .controller_data = &mt_spi_chip_conf,
};
*/
//static struct spi_master *spi_ctl = NULL;
//static struct spi_device *spi_dev = NULL;

////////////////////////////////////////////////////////////////////////////////
#if 0
int sf_spi_clock_enable(bool on)
{
    int err = 0;
    struct mt_spi_t *mt_spi = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    mt_spi = spi_master_get_devdata(spi_ctl);

    if (!mt_spi) {
        xprintk(KERN_ERR, "fail to get mediatek spi device.\n");
        dump_stack();
        return (-ENODEV);
    }

    if (on) {
        clk_enable(mt_spi->clk_main);
    }
    else {
        clk_disable(mt_spi->clk_main);
    }

    return err;
}

int __init sf_spi_platform_init(void)
{
    int err = 0;
    spi_ctl = spi_busnum_to_master(SF_SPI_BUS_NUM);

    if (!spi_ctl) {
        xprintk(KERN_ERR, "there is no spi master for bus %d.\n", SF_SPI_BUS_NUM);
        return (-ENODEV);
    }

    sf_spi_clock_enable(true);
    //    spi_dev = spi_new_device(spi_ctl, &spi_desc);
    //    if (!spi_dev) {
    //      xprintk(KERN_ERR, "failed to register spi board info.\n");
    //      spi_master_put(spi_ctl);
    //      return (-EBUSY);
    //  }
    xprintk(KERN_INFO, "sunwave spi board info has been registered.\n");
    return err;
}

void __exit sf_spi_platform_exit(void)
{
    sf_spi_clock_enable(false);

    if (spi_dev) {
        spi_master_put(spi_dev->master);
        spi_unregister_device(spi_dev);
        /* CHECKME: will spi_unregister_device(..) free the memory of 'spi_dev'? */
        /* kfree(spi_dev); */
        spi_dev = NULL;
    }

    if (spi_ctl) {
        spi_master_put(spi_ctl);
        spi_ctl = NULL;
    }

    xprintk(KERN_INFO, "sunwave spi board info released.\n");
}
#endif
