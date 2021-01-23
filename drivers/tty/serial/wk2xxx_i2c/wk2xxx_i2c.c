/*
 *	WKIC Ltd.
 *	  By  Xu XunWei Tech  
 *	DEMO Version :1.01 Data:2016-7-1
 *
 *
 *
 * 	1. compiler warnings all changes
 */												   




#include<linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include<linux/timer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "wk2xxx.h"

#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/of_gpio.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>




MODULE_LICENSE("Dual FE/GPL");

#define _DEBUG_WK2XXX
//#define _DEBUG_WK2XXX1
//#define _DEBUG_WK2XXX2
//#define _DEBUG_WK2XXX3
//#define _DEBUG_WK2XXX4
#define WK2XXX_PAGE1        1
#define WK2XXX_PAGE0        0


#define WK2XXX_STATUS_PE    1
#define WK2XXX_STATUS_FE    2
#define WK2XXX_STATUS_BRK   4
#define WK2XXX_STATUS_OE    8



static DEFINE_MUTEX(wk2xxxs_lock);                /* race on probe */  
static DEFINE_MUTEX(wk2xxxs_reg);
static DEFINE_MUTEX(wk2xxs_work_lock);                /* work on probe */
static struct i2c_client  *wk2xxx_i2c_client;/*定义一个全局变量，用于获得i2c_client*/
static u32 wk2xxx_irq;

struct wk2xxx_i2c_pdata{
  /* wk gpio*/
    unsigned int reset_gpio;
    unsigned int irq_gpio;
    unsigned int power_gpio;
    unsigned int ls_gpio;
};
struct wk2xxx_port 
{
  //struct timer_list mytimer;	

  struct uart_port port;//[NR_PORTS];
  struct i2c_client  *client;
  spinlock_t conf_lock;	/* shared data */
  struct workqueue_struct *workqueue;
  struct work_struct work;
  int suspending;
  void (*wk2xxx_hw_suspend) (int suspend);
  int tx_done;

  int force_end_work;
  int irq;
  int minor;		/* minor number */
  int tx_empty; 
  int tx_empty_flag;

  //int start_tx;
  int start_tx_flag;
  int stop_tx_flag;
  int stop_rx_flag; 
  int irq_flag;
  int conf_flag;

  int tx_empty_fail;
  int start_tx_fail;
  int stop_tx_fail;
  int stop_rx_fail;
  int irq_fail;
  int conf_fail;

  uint8_t new_lcr;
  uint8_t new_scr; 
  /*set baud 0f register*/
  uint8_t new_baud1;
  uint8_t new_baud0;
  uint8_t new_pres;
  struct wk2xxx_i2c_pdata pdata;
};

static struct wk2xxx_port wk2xxxs[NR_PORTS]; /* the chips */

static int wk2xxx_read_reg(struct i2c_client  *client,uint8_t port,uint8_t reg,uint8_t *dat)
{       

  //struct i2c_adapter *adap=client->adapter;
  struct i2c_msg msg[2];
  uint8_t wk_addr=(0x20|((port-1)<<2))>>1;
  uint8_t ret,count,status=0;
  uint8_t wk_buf[2];
  mutex_lock(&wk2xxxs_reg);
#ifdef _DEBUG_WK2XXX3
  pr_info("--wk2xxx_read_reg---in---\n");
#endif  
  count=0;
err_read:
  wk_buf[0]=reg;
  msg[0].addr=wk_addr;
  msg[0].flags=0;
  msg[0].len=1;
  msg[0].buf=&wk_buf[0];
  if(i2c_transfer(client->adapter,&msg[0],1)<0)
  {      
    if(count<=2)
    {
      count++;
      goto err_read;
    }
    else
    {
      pr_info("wk2xxx_read_reg w_error!\n");
      status=1;
    }	
  }
#ifdef _DEBUG_WK2XXX3
  pr_info("--wk2xxx_read_reg---in1---\n");
#endif

  msg[1].addr=wk_addr;
  msg[1].flags=I2C_M_RD;
  msg[1].len=1;
  msg[1].buf=&wk_buf[1];
  ret=i2c_transfer(client->adapter,&msg[1],1);
  if(ret==1)
  {
    *dat=wk_buf[1];
  }
  else
  {	
    if(count<=2)
    {
      count++;
      goto err_read;
    }
    else
    {
      pr_info("wk2xxx_read_reg r_error!\n");
      *dat=0x0;
      status=1;
    }
  }
#ifdef _DEBUG_WK2XXX3
  pr_info("--wk2xxx_read_reg---out---\n");
#endif
  udelay(5);
  mutex_unlock(&wk2xxxs_reg);

  return status;

}


static int wk2xxx_write_reg(struct i2c_client  *client,uint8_t port,uint8_t reg,uint8_t dat)
{
  //struct i2c_adapter *adap=client->adapter;
  struct i2c_msg msg;
  uint8_t wk_addr=(0x20|((port-1)<<2))>>1;//P1P0=00
  uint8_t wk_buf[2];
  uint8_t count1,status=0;
#ifdef _DEBUG_WK2XXX3
  pr_info("--wk2xxx_write_reg---in---\n");
#endif
  count1=0;
  mutex_lock(&wk2xxxs_reg);
err_write:

  wk_buf[0]=reg;
  wk_buf[1]=dat;
  msg.addr=wk_addr;
  msg.flags=0;
  msg.len=2;
  msg.buf=wk_buf;
  if(i2c_transfer(client->adapter,&msg,1)<0)
  {
    if(count1==0)
    {
      count1++;
      goto err_write;
    }
    else
    {
      pr_info("wk2xxx_write_reg error!\n");
      status=1;
    }

  }
#ifdef _DEBUG_WK2XXX3
  pr_info("--wk2xxx_write_reg---out---\n");
#endif
  udelay(5);
  mutex_unlock(&wk2xxxs_reg);
  return status;

}

#define MAX_RFCOUNT_SIZE 256
static int wk2xxx_read_fifo(struct i2c_client  *client,uint8_t port,uint8_t fifolen,uint8_t *dat)
{
  //struct i2c_adapter *adap=client->adapter;
  struct i2c_msg msg;
  uint8_t wk_addr=(0x20|((port-1)<<2)|0x03)>>1;//P1P0=00
  int i,status=0;
  uint8_t fifo_data[256]={0};


  msg.addr=wk_addr;
  msg.flags=I2C_M_RD;
  msg.len=fifolen;
  msg.buf=fifo_data;
  if(i2c_transfer(client->adapter,&msg,1)<0)
  {
    pr_info("wk2xxx_write_reg error!\n");
    status=1;
  }
  else
  {
    for(i=0;i<fifolen;i++)
    {
      *(dat+i)=fifo_data[i];
    }
  }
  return status;
}
static int wk2xxx_write_fifo(struct i2c_client  *client,uint8_t port,uint8_t fifolen,uint8_t *dat)
{
  // struct i2c_adapter *adap=client->adapter;
  struct i2c_msg msg;
  uint8_t wk_addr=(0x20|((port-1)<<2)|0x02)>>1;//P1P0=00
  int i,status=0;
  uint8_t fifo_data[256]={0};

  for(i=0;i<fifolen;i++)
  {
    fifo_data[i+1]=*(dat+i);
  }

  msg.addr=wk_addr;
  msg.flags=0;
  msg.len=fifolen;
  msg.buf=dat;
  if(i2c_transfer(client->adapter,&msg,1)<0)
  {
    pr_info("wk2xxx_write_reg error!\n");
    status=1;
  }
  return status;
}
static void wk2xxxirq_app(struct uart_port *port);//
static void conf_wk2xxx_subport(struct uart_port *port);//
static void wk2xxx_work(struct work_struct *w);
static void wk2xxx_stop_tx(struct uart_port *port);
static u_int wk2xxx_tx_empty(struct uart_port *port);// or query the tx fifo is not empty?

static int wk2xxx_dowork(struct wk2xxx_port *s)
{    
#ifdef _DEBUG_WK2XXX
  pr_info("--wk2xxx_dowork---in---\n");
#endif

  if (!s->force_end_work && !work_pending(&s->work) && !freezing(current) && !s->suspending)
  {
    queue_work(s->workqueue, &s->work);//
#ifdef _DEBUG_WK2XXX
    pr_info("--queue_work---ok---\n");
#endif
    return 1;	
  }
  else
  {
#ifdef _DEBUG_WK2XXX
    pr_info("--queue_work---error---\n");
#endif
    return 0;
  }

}

static void wk2xxx_work(struct work_struct *w)
{  


  struct wk2xxx_port *s = container_of(w,struct wk2xxx_port, work);
  uint8_t rx;
#ifdef _DEBUG_WK2XXX
  pr_info("--wk2xxx_work---in---\n");
#endif    

  int work_start_tx_flag; 
  int work_stop_rx_flag;

  int work_irq_flag;
  int work_conf_flag;
  do {

    mutex_lock(&wk2xxs_work_lock);
    /*	work_tx_empty_flag = s->tx_empty_flag;
        if(work_tx_empty_flag)
        s->tx_empty_flag = 0;*/
    work_start_tx_flag = s->start_tx_flag;
    if(work_start_tx_flag)
      s->start_tx_flag = 0;
    /*work_stop_tx_flag = s->stop_tx_flag;
      if(work_stop_tx_flag)
      s->stop_tx_flag = 0;*/
    work_stop_rx_flag = s->stop_rx_flag;
    if(work_stop_rx_flag)
      s->stop_rx_flag = 0;
    work_conf_flag = s->conf_flag;
    /*if(work_conf_flag)
      s->conf_flag = 0;*/



    work_irq_flag = s->irq_flag;
    if(work_irq_flag)
      s->irq_flag = 0;

    //work_irq_fail = s->irq_fail;
    //if(work_irq_fail)
    //s->irq_fail = 0;


    mutex_unlock(&wk2xxs_work_lock);

    /*	if(work_conf_flag)
        {
        conf_wk2xxx_subport(&s->port);
        }*/
    /*if(work_tx_empty_flag)
      {
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,&rx);
      s->tx_empty = (rx & WK2XXX_TDAT)<=0;
      }*/
    if(work_start_tx_flag)
    {
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,&rx);
      rx |= WK2XXX_TFTRIG_IEN; 
      wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,rx);
#ifdef _DEBUG_WK2XXX1 
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,&rx);
      pr_info("start_tx()----port:%lx--SIER:0x%x---\n",s->port.iobase,rx);
#endif

    }
    /* if(work_stop_tx_flag)
       {
       wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,&rx);
       rx &=~WK2XXX_TFTRIG_IEN;
       wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,rx);
       wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,&rx);
       rx &= ~WK2XXX_TFTRIG_INT;
       wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,rx);
       }*/
    if(work_stop_rx_flag)
    {
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,&rx);
      rx &=~WK2XXX_RFTRIG_IEN;
      rx &=~WK2XXX_RXOUT_IEN;
      wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,rx);
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,&rx);
      rx &= ~WK2XXX_RXOVT_INT;
      rx &= ~WK2XXX_RFTRIG_INT;
      wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,rx);
    }

    if(work_irq_flag)
    {
      wk2xxxirq_app(&s->port);
      s->irq_fail = 1;
    }



  }while (!s->force_end_work && !freezing(current) && \
      (work_irq_flag || work_stop_rx_flag ));
  /*work_stop_tx_flag || work_tx_empty_flag || work_conf_flag*/
  /*
     }while (!s->force_end_work && !freezing(current) && \
     ((s->work_irq_flag != s->irq_flag) ||\
     (s->work_stop_rx_flag != s->stop_rx_flag) ||\
     (s->work_stop_tx_flag != s->stop_tx_flag) ||\
     (s->work_tx_empty_flag != s->tx_empty_flag) ||\
     (s->work_conf_flag != s->conf_flag)));

   */

#ifdef _DEBUG_WK2XXX
pr_info("-----exit------- work ------\n");
#endif

/* if(s->conf_fail)
   {
   conf_wk2xxx_subport(&s->port);
   s->conf_fail =0;
   }*/

/*	if(s->tx_empty_fail)
    {
    wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,&rx);
    s->tx_empty = (rx & WK2XXX_TDAT) > 0;
    s->tx_empty_fail =0;
    }*/

if(s->start_tx_fail)
{
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,&rx);
  rx |= WK2XXX_TFTRIG_IEN;
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,rx);
  s->start_tx_fail =0;
}

/* if(s->stop_tx_fail)
   {

   wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,&rx);
   rx &=~WK2XXX_TFTRIG_IEN;
   wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,rx);
   wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,&rx);
   rx &= ~WK2XXX_TFTRIG_INT;
   wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,rx);
   s->stop_tx_fail =0;

   }*/

if(s->stop_rx_fail)
{
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,&rx);
  rx &=~WK2XXX_RFTRIG_IEN;
  rx &=~WK2XXX_RXOUT_IEN;
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,rx);

  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,&rx);
  rx &= ~WK2XXX_RFTRIG_INT;
  rx &= ~WK2XXX_RXOVT_INT;
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,rx);
  s->stop_rx_fail =0;
}
if(s->irq_fail)
{
  s->irq_fail = 0;
  enable_irq(s->port.irq);
}

#ifdef _DEBUG_WK2XXX
pr_info("--wk2xxx_work---exit---\n");
#endif

}


static void wk2xxx_rx_chars(struct uart_port *port)//vk32xx_port *port)
{


  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
  uint8_t fsr,lsr,dat[1],rx_dat[256]={0};
  unsigned int ch, flg,sifr, ignored=0,status = 0,rx_count=0;
  int rfcnt=0,rx_num=0;
#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_rx_chars()---------in---\n");
#endif

  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,dat);
  fsr = dat[0];
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_LSR,dat);
  lsr = dat[0];
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,dat);
  sifr=dat[0];
#ifdef _DEBUG_WK2XXX
  pr_info("rx_chars()-port:0x%lx--fsr:0x%x--lsr:0x%x--\n",s->port.iobase,fsr,lsr);
#endif
  if(!(sifr&0x80))//no error
  { 
    flg = TTY_NORMAL;
    if (fsr& WK2XXX_RDAT)
    {
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_RFCNT,dat);
      rfcnt=dat[0];
      if(rfcnt==0)
      {
        rfcnt=255; 
      }
#ifdef _DEBUG_WK2XXX

      pr_info("1wk2xxx_rx_chars()----port:%lx--RFCNT:0x%x----\n",s->port.iobase,rfcnt);
#endif


      wk2xxx_read_fifo(wk2xxx_i2c_client,s->port.iobase, rfcnt,rx_dat);

      //pr_info("rx_chars_wk2xxx_read_fifo!!!!!!!!!!!\n");
      s->port.icount.rx+=rfcnt;
      for(rx_num=0;rx_num<rfcnt;rx_num++)
      {
        if (uart_handle_sysrq_char(&s->port,rx_dat[rx_num]))//.state, ch))
          break;//

#ifdef _DEBUG_WK2XXX5 

        pr_info("rx_chars:0x%x----\n",rx_dat[rx_num]);
#endif
        uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, rx_dat[rx_num], flg);
        rx_count++;

        if ((rx_count >= 64 ) && (s->port.state->port.tty != NULL))
        {
          tty_flip_buffer_push(&(s->port.state->port));
          rx_count = 0;
        }

      }//for
      if((rx_count > 0)&&(s->port.state->port.tty != NULL))
      {
#ifdef _DEBUG_WK2XXX
        pr_info("push buffer tty flip port = :0x%lx count = :0x%x\n",s->port.iobase,rx_count);
#endif
        tty_flip_buffer_push(&(s->port.state->port));
        rx_count = 0;
      }

    }
  }//ifm
  else//error
  {
    while (fsr& WK2XXX_RDAT)/**/
    {
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FDAT,dat);
      ch = (int)dat[0];
#ifdef _DEBUG_WK2XXX

      pr_info("wk2xxx_rx_chars()----port:%lx--RXDAT:0x%x----\n",s->port.iobase,ch);
#endif

      s->port.icount.rx++;
      //rx_count++;
#ifdef _DEBUG_WK2XXX1
      pr_info("wk2xxx_rx_chars()----port:%lx error\n",s->port.iobase);
#endif
      flg = TTY_NORMAL;
      if (lsr&(WK2XXX_OE |WK2XXX_FE|WK2XXX_PE|WK2XXX_BI))
      {
        pr_info("wk2xxx_rx_chars()----port:0x%lx error,lsr:0x%x!!!!!!!!!!!!!!!!!\n",s->port.iobase,lsr);
        //goto handle_error;
        if (lsr & WK2XXX_PE)
        {
          s->port.icount.parity++;
          status |= WK2XXX_STATUS_PE;
          flg = TTY_PARITY;
        }
        if (lsr & WK2XXX_FE)
        {
          s->port.icount.frame++;
          status |= WK2XXX_STATUS_FE;
          flg = TTY_FRAME;
        }
        if (lsr & WK2XXX_OE)
        {
          s->port.icount.overrun++;
          status |= WK2XXX_STATUS_OE;
          flg = TTY_OVERRUN;
        }
        if(lsr&fsr & WK2XXX_BI)
        {
          s->port.icount.brk++;
          status |= WK2XXX_STATUS_BRK;
          flg = TTY_BREAK;
        }

        if (++ignored > 100) 
          goto out;

        goto ignore_char;       
      }

error_return:
      if (uart_handle_sysrq_char(&s->port,ch))//.state, ch))
        goto ignore_char;

      uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, ch, flg);
      rx_count++;

      if ((rx_count >= 64 ) && (s->port.state->port.tty != NULL)) 
      {
        tty_flip_buffer_push(&(s->port.state->port));
        rx_count = 0;
      } 
#ifdef _DEBUG_WK2XXX1
      pr_info(" s->port.icount.rx = 0x%X ,char = 0x%X ,flg = 0x%X ,port = %lx, rx_count = %d\n",s->port.icount.rx,ch,flg,s->port.iobase,rx_count);
#endif
ignore_char:

      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,dat);
      fsr = dat[0];
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_LSR,dat);
      lsr = dat[0];
    }
out:
    if((rx_count > 0)&&(s->port.state->port.tty != NULL))
    {
#ifdef _DEBUG_WK2XXX1
      pr_info("push buffer tty flip port = :0x%lx, count = :%d\n",s->port.iobase,rx_count);
#endif
      tty_flip_buffer_push(&(s->port.state->port));
      rx_count = 0;
    }

  }//if()else

#if 0
  pr_info(" rx_num = :%d\n",s->port.icount.rx);
#endif

#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_rx_chars()---------out---\n");
#endif

  return;
#ifdef SUPPORT_SYSRQ
  s->port.state->sysrq = 0;
#endif
  goto error_return;

#ifdef _DEBUG_WK2XXX
  pr_info("--wk2xxx_rx_chars---exit---\n");
#endif

}

static void wk2xxx_tx_chars(struct uart_port *port)//
{


  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
  uint8_t fsr,tfcnt,dat[1],txbuf[255]={0};
  int count,tx_count,i;
#ifdef _DEBUG_WK2XXX
  pr_info("--wk2xxx_tx_chars---in---\n");
#endif

  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
  if (s->port.x_char) 
  {   
#ifdef _DEBUG_WK2XXX
    pr_info("wk2xxx_tx_chars   s->port.x_char:0x%x,port = %lx\n",s->port.x_char,s->port.iobase);
#endif
    wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FDAT,s->port.x_char);
    s->port.icount.tx++;
    s->port.x_char = 0;
    goto out;
  }

  if(uart_circ_empty(&s->port.state->xmit) || uart_tx_stopped(&s->port))
  {			
    goto out;

  }

  /*
   * Tried using FIFO (not checking TNF) for fifo fill:
   * still had the '1 bytes repeated' problem.
   */
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,dat);
  fsr = dat[0];

  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_TFCNT,dat); 
  tfcnt= dat[0];
#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_tx_chars   fsr:0x%x,rfcnt:0x%x,port = 0x%lx\n",fsr,tfcnt,s->port.iobase);
#endif
#if 1
  if(tfcnt==0)
  {
    if(fsr & WK2XXX_TFULL)
    {
      tfcnt=255;
      tx_count=0;
    }
    else 
    {
      tfcnt=0;
      tx_count=255;
    }
  }
  else
  {
    tx_count=255-tfcnt;
#ifdef _DEBUG_WK2XXX
    pr_info("wk2xxx_tx_chars2   tx_count:%x,port = %lx\n",tx_count,s->port.iobase);
#endif 
  }

#endif

#ifdef _DEBUG_WK2XXX
  pr_info("fsr:%x\n",fsr);
#endif


  count = tx_count;
  i=0;
  do
  {
    if(uart_circ_empty(&s->port.state->xmit))
      break;
    txbuf[i]=s->port.state->xmit.buf[s->port.state->xmit.tail];
    s->port.state->xmit.tail = (s->port.state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
    s->port.icount.tx++;
    i++;
#ifdef _DEBUG_WK2XXX

    pr_info("tx_chars:0x%x--\n",txbuf[i-1]);
#endif

  }while(--count>0);

#ifdef _DEBUG_WK2XXX5 

  pr_info("tx_chars I:0x%x--\n",i);
#endif

  wk2xxx_write_fifo(wk2xxx_i2c_client,s->port.iobase,i,txbuf);
out:wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,dat);
    fsr = dat[0];
    if(((fsr&WK2XXX_TDAT)==0)&&((fsr&WK2XXX_TBUSY)==0))
    {
      if (uart_circ_chars_pending(&s->port.state->xmit) < WAKEUP_CHARS)
        uart_write_wakeup(&s->port); 

      if (uart_circ_empty(&s->port.state->xmit))
      {
        wk2xxx_stop_tx(&s->port);
      }
    }
#ifdef _DEBUG_WK2XXX
    pr_info("--wk2xxx_tx_chars---exit---\n");
#endif


}

static irqreturn_t wk2xxx_irq_handler(int irq, void *dev_id)//
{
#ifdef _DEBUG_WK2XXX
  pr_info("--wk2xxx_irq---in---\n");
#endif

  struct wk2xxx_port *s = dev_id;
  disable_irq_nosync(s->port.irq);

  s->irq_flag = 1;

  if(wk2xxx_dowork(s))
  {

    //	s->irq_flag = 1;

  }
  else
  {	
    s->irq_flag = 0;
    s->irq_fail = 1;
  }	
#ifdef _DEBUG_WK2XXX
  pr_info("--wk2xxx_irq---exit---\n");
#endif

  return IRQ_HANDLED;
}

static void wk2xxxirq_app(struct uart_port *port)//
{

  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

  unsigned int  pass_counter = 0;
  uint8_t sifr,gifr,sier,dat[1];
#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxxirq_app()------port:%lx--------------\n",s->port.iobase);
#endif		
#ifdef _DEBUG_WK2XXX1

  uint8_t gier,sifr0,sifr1,sifr2,sifr3,sier1,sier0,sier2,sier3;
#endif

  wk2xxx_read_reg(wk2xxx_i2c_client,2,WK2XXX_GIFR ,dat);
  gifr = dat[0];		
#ifdef _DEBUG_WK2XXX1 
  wk2xxx_read_reg(wk2xxx_i2c_client,2,WK2XXX_GIER ,dat);
  gier = dat[0]; 
  wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
  wk2xxx_write_reg(wk2xxx_i2c_client,2,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
  wk2xxx_write_reg(wk2xxx_i2c_client,3,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
  wk2xxx_write_reg(wk2xxx_i2c_client,4,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0

  wk2xxx_read_reg(wk2xxx_i2c_client,1,WK2XXX_SIFR,&sifr0);
  wk2xxx_read_reg(wk2xxx_i2c_client,2,WK2XXX_SIFR,&sifr1);
  wk2xxx_read_reg(wk2xxx_i2c_client,3,WK2XXX_SIFR,&sifr2);
  wk2xxx_read_reg(wk2xxx_i2c_client,4,WK2XXX_SIFR,&sifr3);

  wk2xxx_read_reg(wk2xxx_i2c_client,1,WK2XXX_SIER,&sier0);
  wk2xxx_read_reg(wk2xxx_i2c_client,2,WK2XXX_SIER,&sier1);
  wk2xxx_read_reg(wk2xxx_i2c_client,3,WK2XXX_SIER,&sier2);
  wk2xxx_read_reg(wk2xxx_i2c_client,4,WK2XXX_SIER,&sier3);
#endif 		


#ifdef _DEBUG_WK2XXX1
  pr_info("irq_app....gifr:%x  gier:%x  sier1:%x  sier2:%x sier3:%x sier4:%x   sifr1:%x sifr2:%x sifr3:%x sifr4:%x \n",gifr,gier,sier0,sier1,sier2,sier3,sifr0,sifr1,sifr2,sifr3);
#endif
  switch(s->port.iobase)
  {
    case 1 :
      if(!(gifr & WK2XXX_UT1INT))
      {
        return;
      }
      break;
    case 2 :
      if(!(gifr & WK2XXX_UT2INT))
      { 			 
        return;
      } 									  
      break;
    case 3 :
      if(!(gifr & WK2XXX_UT3INT))
      { 			 
        return;
      }
      break;
    case 4 :
      if(!(gifr & WK2XXX_UT4INT))
      {				
        return;
      }
      break;
    default:
      break;

  }

  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,dat);
  sifr = dat[0];
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,dat);
  sier = dat[0];
#ifdef _DEBUG_WK2XXX1
  pr_info("irq_app..........sifr:%x sier:%x \n",sifr,sier);
#endif
  do {
    if ((sifr&WK2XXX_RFTRIG_INT)|(sifr&WK2XXX_RXOVT_INT))
    {
      wk2xxx_rx_chars(&s->port);
    }

    if ((sifr & WK2XXX_TFTRIG_INT)&&(sier & WK2XXX_TFTRIG_IEN ))
    {
      wk2xxx_tx_chars(&s->port);
      return;
    }
    if (pass_counter++ > WK2XXX_ISR_PASS_LIMIT)
      break;
    wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,dat);
    sifr = dat[0];				  
    wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,dat);
    sier = dat[0];
#ifdef _DEBUG_WK2XXX1
    pr_info("irq_app...........rx............tx  sifr:%x sier:%x port:%lx\n",sifr,sier,s->port.iobase);
#endif
  } while ((sifr &WK2XXX_RXOVT_INT)||(sifr & WK2XXX_RFTRIG_INT)||((sifr & WK2XXX_TFTRIG_INT)&&(sier & WK2XXX_TFTRIG_IEN)));
#ifdef _DEBUG_WK2XXX1
  pr_info("sifr:%d\n",sifr);
#endif
#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxxirq_app()---------exit---\n");
#endif

}


/*
 *   Return TIOCSER_TEMT when transmitter is not busy.
 */

static u_int wk2xxx_tx_empty(struct uart_port *port)// or query the tx fifo is not empty?
{
  uint8_t rx;

  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_tx_empty()---------in---\n");
#endif



  mutex_lock(&wk2xxxs_lock);

  if(!(s->tx_empty_flag || s->tx_empty_fail))
  {
    wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,&rx);

    while((rx & WK2XXX_TDAT)|(rx&WK2XXX_TBUSY))
    {
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,&rx);
    }
    s->tx_empty = ((rx & WK2XXX_TDAT)|(rx&WK2XXX_TBUSY))<=0;


    if(s->tx_empty)
    {
      s->tx_empty_flag =0;
      s->tx_empty_fail=0;
    }
    else
    {
      s->tx_empty_fail=0;
      s->tx_empty_flag =0;
    }
  }
#ifdef _DEBUG_WK2XXX1
  pr_info("s->tx_empty_fail----FSR:%d--s->tx_empty:%d--\n",rx,s->tx_empty);
#endif







#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_tx_empty----------exit---\n");
#endif
  mutex_unlock(&wk2xxxs_lock);
  return s->tx_empty;

}

static void wk2xxx_set_mctrl(struct uart_port *port, u_int mctrl)//nothing
{
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_set_mctrl---------exit---\n");
#endif

}
static u_int wk2xxx_get_mctrl(struct uart_port *port)// since no modem control line
{       
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_get_mctrl---------exit---\n");
#endif

  return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}


/*
 *  interrupts disabled on entry
 */

static void wk2xxx_stop_tx(struct uart_port *port)//
{

  uint8_t dat[1],sier,sifr;
  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_stop_tx------in---\n");
#endif
  mutex_lock(&wk2xxxs_lock);

  if(!(s->stop_tx_flag||s->stop_tx_fail))
  {
    wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,dat);
    sier=dat[0];
    s->stop_tx_fail=(sier&WK2XXX_TFTRIG_IEN)>0;
    if(s->stop_tx_fail)
    {

      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,dat);
      sier=dat[0];
      sier&=~WK2XXX_TFTRIG_IEN;
      wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,sier);
      wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,dat);
      sifr=dat[0];
      sifr&= ~WK2XXX_TFTRIG_INT;
      wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIFR,sifr);
      s->stop_tx_fail =0;
      s->stop_tx_flag=0;

    }
    else
    {
      s->stop_tx_fail =0;
      s->stop_tx_flag=0;


    }



  }
  mutex_unlock(&wk2xxxs_lock); 
#ifdef _DEBUG_WK2XXX4
  pr_info("-wk2xxx_stop_tx------exit---\n");
#endif

}

/*
 *  * interrupts may not be disabled on entry
 */
static void wk2xxx_start_tx(struct uart_port *port)
{	


  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);       
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_start_tx------in---\n");
#endif

  if(!(s->start_tx_flag||s->start_tx_fail))
  {    s->start_tx_flag = 1;
    if(wk2xxx_dowork(s))
    {
      ;
    }
    else
    {
      s->start_tx_fail = 1;
      s->start_tx_flag = 0;
    }
  }

#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_start_tx------exit---\n");
#endif

}

/*
 *  * Interrupts enabled
 */

static void wk2xxx_stop_rx(struct uart_port *port)
{


  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_stop_rx------in---\n");
#endif

  if(!(s->stop_rx_flag ||s->stop_rx_fail ))
  {
    s->stop_rx_flag = 1;
    if(wk2xxx_dowork(s))
    {

      ;
    }
    else
    {
      s->stop_rx_flag = 0;
      s->stop_rx_fail = 1;
    }
  }


#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_stop_rx------exit---\n");
#endif

}


/*
 *  * No modem control lines
 *   */
static void wk2xxx_enable_ms(struct uart_port *port)    //nothing
{
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_enable_ms------exit---\n");
#endif

}
/*
 *  * Interrupts always disabled.
 */   
static void wk2xxx_break_ctl(struct uart_port *port, int break_state)
{
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_break_ctl------exit---\n");
#endif

  //break operation, but there  seems no such operation in vk32  
}


static int wk2xxx_startup(struct uart_port *port)//i
{



  uint8_t gena,grst,gier,sier,scr,dat[1];
  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
  char b[12];
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_startup------in---\n");
#endif
  if (s->suspending)
    return 0;
  //gpio_set_value(s->pdata.power_gpio,1);
  //gpio_set_value(s->pdata.ls_gpio,1);
  //gpio_set_value(s->pdata.reset_gpio,1);
  s->force_end_work = 0;
  sprintf(b, "wk2xxx-%d", (uint8_t)s->port.iobase);
  s->workqueue = create_singlethread_workqueue(b);

  if (!s->workqueue) 
  {
    dev_warn(&wk2xxx_i2c_client->dev, "cannot create workqueue\n");
    return -EBUSY;
  }

  INIT_WORK(&s->work, wk2xxx_work);

  if (s->wk2xxx_hw_suspend)
    s->wk2xxx_hw_suspend(0);

  wk2xxx_read_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,dat);
  gena=dat[0];
  switch (s->port.iobase)
  {
    case 1:
      gena|=WK2XXX_UT1EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    case 2:
      gena|=WK2XXX_UT2EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    case 3:
      gena|=WK2XXX_UT3EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    case 4:
      gena|=WK2XXX_UT4EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    default:
      pr_info(":con_wk2xxx_subport bad iobase %d\n", (uint8_t)s->port.iobase);
  }

  wk2xxx_read_reg(wk2xxx_i2c_client,1,WK2XXX_GRST,dat);
  grst=dat[0];
  switch (s->port.iobase)
  {
    case 1:
      grst|=WK2XXX_UT1RST;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GRST,grst);
      break;
    case 2:
      grst|=WK2XXX_UT2RST;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GRST,grst);
      break;
    case 3:
      grst|=WK2XXX_UT3RST;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GRST,grst);
      break;
    case 4:
      grst|=WK2XXX_UT4RST;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GRST,grst);
      break;
    default:
      pr_info(":con_wk2xxx_subport bad iobase %x\n", (uint8_t)s->port.iobase);
  }
  //enable Receive Interrupt
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,dat);
  sier = dat[0];
  sier &= ~WK2XXX_TFTRIG_IEN;
  sier |= WK2XXX_RFTRIG_IEN;
  sier |= WK2XXX_RXOUT_IEN;
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER,sier);
  //enable uart TX and RX
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SCR,dat);
  scr = dat[0] | WK2XXX_TXEN|WK2XXX_RXEN;
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SCR,scr);

  //initiate the fifos
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FCR,0x0f);//initiate the fifos
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FCR,0x0c);
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SPAGE,1);  
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_RFTL,0X80);    
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_TFTL,0X20);    
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SPAGE,0);  
  //enable the sub port interrupt
  wk2xxx_read_reg(wk2xxx_i2c_client,2,WK2XXX_GIER,dat);
  gier = dat[0];

  switch (s->port.iobase){
    case 1:
      gier|=WK2XXX_UT1IE;
      wk2xxx_write_reg(wk2xxx_i2c_client,2,WK2XXX_GIER,gier);
      break;
    case 2:
      gier|=WK2XXX_UT2IE;
      wk2xxx_write_reg(wk2xxx_i2c_client,2,WK2XXX_GIER,gier);
      break;
    case 3:
      gier|=WK2XXX_UT3IE;
      wk2xxx_write_reg(wk2xxx_i2c_client,2,WK2XXX_GIER,gier);
      break;
    case 4:
      gier|=WK2XXX_UT4IE;
      wk2xxx_write_reg(wk2xxx_i2c_client,2,WK2XXX_GIER,gier);
      break;
    default:
      pr_info(": bad iobase %x\n", (uint8_t)s->port.iobase);
  }

  if (s->wk2xxx_hw_suspend)
    s->wk2xxx_hw_suspend(0);
  msleep(50);


  uart_circ_clear(&s->port.state->xmit);
  wk2xxx_enable_ms(&s->port);
  // request irq 
  if(request_irq(s->port.irq, wk2xxx_irq_handler,IRQF_SHARED|IRQF_TRIGGER_LOW,"wk2xxxi2c", s) < 0)
  {
    dev_warn(&wk2xxx_i2c_client->dev, "cannot allocate irq %d\n", s->irq);
    s->port.irq = 0;
    destroy_workqueue(s->workqueue);
    s->workqueue = NULL;
    return -EBUSY;
  }       udelay(100);
  udelay(100);
#ifdef _DEBUG_WK2XXX1
  wk2xxx_read_reg(wk2xxx_i2c_client,2,WK2XXX_GIER,dat);
  pr_info("GIER:0x%x\n",dat[0]);
#endif

#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_startup------exit---\n");
#endif

  return 0;
}
//* Power down all displays on reboot, poweroff or halt *

static void wk2xxx_shutdown(struct uart_port *port)//
{


  uint8_t gena,dat[1]; 
  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_shutdown------in---\n");
#endif

  if (s->suspending)
    return;
  s->force_end_work = 1;
  if (s->workqueue) 
  {
    flush_workqueue(s->workqueue);
    destroy_workqueue(s->workqueue);
    s->workqueue = NULL;
  }

  if (s->port.irq)
  {
    //disable_irq_nosync(s->port.irq);		
    free_irq(s->port.irq,s);
  }
  //disable port
  wk2xxx_read_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,dat);
  gena=dat[0];
  switch (s->port.iobase)
  {
    case 1:
      gena&=~WK2XXX_UT1EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    case 2:
      gena&=~WK2XXX_UT2EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    case 3:
      gena&=~WK2XXX_UT3EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    case 4:
      gena&=~WK2XXX_UT4EN;
      wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,gena);
      break;
    default:
      pr_info(":wk2xxx_shutdown   bad iobase %x\n", (uint8_t)s->port.iobase);
  }
#ifdef _DEBUG_WK2XXX1
  wk2xxx_read_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,dat);
  pr_info("wk2xxx_shutdown--------GENA:0x%x\n",dat[0]);
#endif


  //gpio_set_value(s->pdata.power_gpio,0);
  //gpio_set_value(s->pdata.ls_gpio,0);
  //gpio_set_value(s->pdata.reset_gpio,0);
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_shutdown-----exit---\n");
#endif

  return;

}

static void conf_wk2xxx_subport(struct uart_port *port)//i
{   


  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
  uint8_t old_sier,lcr,scr,scr_ss,dat[1],baud0_ss,baud1_ss,pres_ss;
#ifdef _DEBUG_WK2XXX
  pr_info("-conf_wk2xxx_subport------in---\n");
#endif


  lcr = s->new_lcr;
  scr_ss = s->new_scr;
  baud0_ss=s->new_baud0;
  baud1_ss=s->new_baud1;
  pres_ss=s->new_pres;
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER ,dat);
  old_sier = dat[0];
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER ,old_sier&(~(WK2XXX_TFTRIG_IEN | WK2XXX_RFTRIG_IEN|WK2XXX_RXOUT_IEN)));
  //local_irq_restore(flags);
  do{
    wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_FSR,dat);

  } while (dat[0] & WK2XXX_TBUSY);
  // then, disable everything 
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SCR,dat);
  scr = dat[0];
  scr &= 0x0f;
  scr |= scr_ss;

  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SCR ,scr&(~(WK2XXX_RXEN|WK2XXX_TXEN)));
  // set the parity, stop bits and data size //
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_LCR ,lcr);
  // set the baud rate //
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SIER ,old_sier);
  // set the baud rate //
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SPAGE ,1);
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_BAUD0 ,baud0_ss);
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_BAUD1 ,baud1_ss);
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_PRES ,pres_ss);
#ifdef _DEBUG_WK2XXX2
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_BAUD0,dat);
  pr_info(":WK2XXX_BAUD0=0x%X\n", dat[0]);
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_BAUD1,dat);
  pr_info(":WK2XXX_BAUD1=0x%X\n", dat[0]);
  wk2xxx_read_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_PRES,dat);
  pr_info(":WK2XXX_PRES=0x%X\n", dat[0]);
#endif
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SPAGE ,0);
  wk2xxx_write_reg(wk2xxx_i2c_client,s->port.iobase,WK2XXX_SCR ,scr|(WK2XXX_RXEN|WK2XXX_TXEN)     );


#ifdef _DEBUG_WK2XXX
  pr_info("-conf_wk2xxx_subport------exit---\n");
#endif

}


// change speed
static void wk2xxx_termios( struct uart_port *port, struct ktermios *termios,
    struct ktermios *old)
{


  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
  int baud = 0;
  uint8_t lcr,baud1,baud0,pres;
#ifdef _DEBUG_WK2XXX
  pr_info("-vk32xx_termios------in---\n");
#endif

  unsigned short cflag;
  unsigned short lflag;

  cflag = termios->c_cflag;
  lflag = termios->c_lflag;
#ifdef _DEBUG_WK2XXX1
  pr_info("cflag := 0x%X  lflag : = 0x%X\n",cflag,lflag);
#endif
  baud1=0;
  baud0=0;
  pres=0;
  baud = tty_termios_baud_rate(termios);

  //wk2xxx of clock= 11.0592M
  switch (baud) 
  {
    case 600:
      baud1=0x4;
      baud0=0x7f;
      pres=0;
      break;
    case 1200:
      baud1=0x2;
      baud0=0x3F;
      pres=0;
      break;
    case 2400:
      baud1=0x1;
      baud0=0x1f;
      pres=0;
      break;
    case 4800:
      baud1=0x00;
      baud0=0x8f;
      pres=0;
      break;
    case 9600:
      baud1=0x00;
      baud0=0x47;
      pres=0;
      break;
    case 19200:
      baud1=0x00;
      baud0=0x23;
      pres=0;
      break;
    case 38400:
      baud1=0x00;
      baud0=0x11;
      pres=0;
      break;
    case 76800:
      baud1=0x00;
      baud0=0x08;
      pres=0;
      break;	

    case 1800:
      baud1=0x01;
      baud0=0x7f;
      pres=0;
      break;
    case 3600:
      baud1=0x00;
      baud0=0xbf;
      pres=0;
      break;
    case 7200:
      baud1=0x00;
      baud0=0x5f;
      pres=0;
      break;
    case 14400:
      baud1=0x00;
      baud0=0x2f;
      pres=0;
      break;
    case 28800:
      baud1=0x00;
      baud0=0x17;
      pres=0;
      break;
    case 57600:
      baud1=0x00;
      baud0=0x0b;
      pres=0;
      break;
    case 115200:
      baud1=0x00;
      baud0=0x05;
      pres=0;
      break;
    case 230400:
      baud1=0x00;
      baud0=0x02;
      pres=0;
      break;
    default:
      baud1=0x00;
      baud0=0x00;
      pres=0;
  }
  tty_termios_encode_baud_rate(termios, baud, baud);

  /* we are sending char from a workqueue so enable */


#ifdef _DEBUG_WK2XXX

  pr_info("wk2xxx_termios()----port:%lx--lcr:0x%x- cflag:0x%x-CSTOPB:0x%x,PARENB:0x%x,PARODD:0x%x--\n",s->port.iobase,lcr,cflag,CSTOPB,PARENB,PARODD);
#endif

  lcr =0;
  if (cflag & CSTOPB)
    lcr|=WK2XXX_STPL;//two  stop_bits
  else
    lcr&=~WK2XXX_STPL;//one  stop_bits

  if (cflag & PARENB) 

  {
    lcr|=WK2XXX_PAEN;//enbale spa
    if (!(cflag & PARODD)){

      lcr |= WK2XXX_PAM1;
      lcr &= ~WK2XXX_PAM0;
    }
    else{
      lcr |= WK2XXX_PAM0;//PAM0=1
      lcr &= ~WK2XXX_PAM1;//PAM1=0
    }
  }
  else
  {
    lcr&=~WK2XXX_PAEN;
  }

#ifdef _DEBUG_WK2XXX

  pr_info("wk2xxx_termios()----port:%lx--lcr:0x%x- cflag:0x%x-CSTOPB:0x%x,PARENB:0x%x,PARODD:0x%x--\n",s->port.iobase,lcr,cflag,CSTOPB,PARENB,PARODD);
#endif


  s->new_baud1=baud1;
  s->new_baud0=baud0;
  s->new_pres=pres;


  s->new_lcr = lcr;


#if 1 // simon change
  conf_wk2xxx_subport(&s->port);

#else
  if(!(s->conf_flag|| s->conf_fail))
  {
    if(wk2xxx_dowork(s))
    {
      s->conf_flag =1;
    }
    else
    {
      s->conf_fail =1;
    }

  }
#endif
#ifdef _DEBUG_WK2XXX
  pr_info("-vk32xx_termios------exit---\n");
#endif


}


static const char *wk2xxx_type(struct uart_port *port)
{


#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_type-------------out-------- \n");
#endif
  return port->type == PORT_WK2XXX ? "wk2xxx" : NULL;//this is defined in serial_core.h
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void wk2xxx_release_port(struct uart_port *port)
{
  pr_info("wk2xxx_release_port\n");

}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int wk2xxx_request_port(struct uart_port *port)//no such memory region needed for vk32
{
  pr_info("wk2xxx_request_port\n");
  return 0;
}

/*
 * Configure/autoconfigure the port*/
static void wk2xxx_config_port(struct uart_port *port, int flags)
{
  struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_config_port \n");
#endif

  if (flags & UART_CONFIG_TYPE && wk2xxx_request_port(port) == 0)
    s->port.type = PORT_WK2XXX;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_vk32xx and PORT_UNKNOWN
 */
static int wk2xxx_verify_port(struct uart_port *port, struct serial_struct *ser)
{


  int ret = 0;
#ifdef _DEBUG_WK2XXX
  pr_info("wk2xxx_verify_port \n");
#endif

  if (ser->type != PORT_UNKNOWN && ser->type != PORT_WK2XXX)
    ret = -EINVAL;
  if (port->irq != ser->irq)
    ret = -EINVAL;
  if (ser->io_type != SERIAL_IO_PORT)
    ret = -EINVAL;
  if (port->iobase != ser->port)
    ret = -EINVAL;
  if (ser->hub6 != 0)
    ret = -EINVAL;
  return ret;
}




static struct uart_ops wk2xxx_pops = {
tx_empty:       wk2xxx_tx_empty,
                set_mctrl:      wk2xxx_set_mctrl,
                get_mctrl:      wk2xxx_get_mctrl,
                stop_tx:        wk2xxx_stop_tx,
                start_tx:       wk2xxx_start_tx,
                stop_rx:        wk2xxx_stop_rx,
                enable_ms:      wk2xxx_enable_ms,
                break_ctl:      wk2xxx_break_ctl,
                startup:        wk2xxx_startup,
                shutdown:       wk2xxx_shutdown,
                set_termios:    wk2xxx_termios,
                type:           wk2xxx_type,
                release_port:   wk2xxx_release_port,
                request_port:   wk2xxx_request_port,
                config_port:    wk2xxx_config_port,
                verify_port:    wk2xxx_verify_port,

};
static struct uart_driver wk2xxx_uart_driver = {
                 owner:                  THIS_MODULE,
                 major:        		 SERIAL_WK2XXX_MAJOR,
                 driver_name:            "ttySWK",
                 dev_name:               "ttysWK",
                 minor:                  MINOR_START,
                 nr:                     NR_PORTS,
                 cons:                   NULL//WK2Xxx_CONSOLE,
};

static int uart_driver_registered;

#ifdef CONFIG_OF
static int wk2xxx_parse_dt(struct device *dev, struct wk2xxx_i2c_pdata *pdata)
{
    int r = 0;
    struct device_node *np = dev->of_node;

    np = of_find_compatible_node(NULL, NULL, "mediatek,wk2xxx_i2c");

    if (np) {
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
        r = of_get_named_gpio(np, "gpio-rst-std", 0);
        if (r < 0)
            pr_err("%s: get wk2xxx RST GPIO failed (%d)", __FILE__, r);
        else
            pdata->reset_gpio = r;

        r = of_get_named_gpio(np, "gpio-irq-std", 0);
        if (r < 0)
            pr_err("%s: get wk2xxx IRQ GPIO failed (%d)", __FILE__, r);
        else
            pdata->irq_gpio = r;
        r = of_get_named_gpio(np, "gpio-power-std", 0);
        if (r < 0)
            pr_err("%s: get wk2xxx power GPIO failed (%d)", __FILE__, r);
        else
            pdata->power_gpio = r;
        r = of_get_named_gpio(np, "gpio-ls-std", 0);
        if (r < 0)
            pr_err("%s: get wk2xxx ls GPIO failed (%d)", __FILE__, r);
        else
            pdata->ls_gpio = r;

        r = 0;
#else
        of_property_read_u32_array(np, "gpio-rst", &(pdata->reset_gpio),1);
        of_property_read_u32_array(np, "gpio-irq", &(pdata->irq_gpio),1);
        of_property_read_u32_array(np, "gpio-power", &(pdata->power_gpio),1);
        of_property_read_u32_array(np, "gpio-ls", &(pdata->ls_gpio),1);
#endif
    } else {
            pr_info("%s : get gpio num err.\n", __func__);
        return -1;
    }

    pr_info(
        "[dsc]%s : get reset_gpio[%d], irq_gpio[%d],power_gpio[%d],ls_gpio[%d]\n",
        __func__, pdata->reset_gpio, pdata->irq_gpio,pdata->power_gpio,pdata->ls_gpio);
  return r;
}
#endif
static int wk2xxx_probe(struct i2c_client  *client,const struct i2c_device_id *dev_id)
{
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_probe()------in---\n");
#endif

  uint8_t i;
  int status;
  int ret;
  int dat[2];
  struct wk2xxx_i2c_pdata *pdata;
  struct device_node *node;
  struct wk2xxx_port *s = &wk2xxxs[i];//container_of(port,struct vk32xx_port,port); 	

  i2c_set_clientdata(client,pdata);
  wk2xxx_i2c_client=client;
  pr_info("wk2xxx_probe wk2xxx\n");
  if(client->dev.of_node) {
    pdata = devm_kzalloc(
        &client->dev, sizeof(struct wk2xxx_i2c_pdata),
        GFP_KERNEL);
    if (!pdata)
      return -ENOMEM;

    pr_info("%s : Parse wk2xxx DTS\n", __func__);
    ret = wk2xxx_parse_dt(&client->dev, pdata);
    if (ret) {
      pr_err("%s : ret =%d\n", __func__, ret);
      return ret;
    }
    pr_info("%s : Parsed wk2xxx DTS %d %d %d %d\n", __func__,
        pdata->reset_gpio, pdata->irq_gpio, pdata->power_gpio, pdata->ls_gpio);
  } else {
    pdata = client->dev.platform_data;
    pr_err("%s : No wk2xxx DTS\n", __func__);
  }
  if (!pdata)
    return -EINVAL;

  dev_dbg(&client->dev, "wk2xxx probe: %s, inside wk2xxx flags = %x\n",
      __func__, client->flags);

    node = of_find_compatible_node(NULL, NULL, "mediatek,irq_wk2xxx-eint");

    if (node) {
        wk2xxx_irq = irq_of_parse_and_map(node, 0);
        client->irq = wk2xxx_irq;
        pr_info("%s : MT IRQ GPIO = %d\n", __func__, client->irq);
        enable_irq_wake(client->irq);
    } else {
        pr_err("%s : can not find wk2xxx eint compatible node\n",
               __func__);
    }

  mutex_lock(&wk2xxxs_lock);
  if(!uart_driver_registered)
  {
    uart_driver_registered = 1;
    status=uart_register_driver(&wk2xxx_uart_driver);
    if (status)
    {
      pr_info("Couldn't register wk2xxx uart driver\n");
      mutex_unlock(&wk2xxxs_lock);
      return status;
    }
  }
  pr_info("wk2xxx_serial_init()\n");
  for(i =0;i<NR_PORTS;i++)
  {
    s->tx_done =0;
    s->client    = wk2xxx_i2c_client;
    s->port.line = i;
    s->port.ops = &wk2xxx_pops;
    s->port.uartclk = WK_CRASTAL_CLK;
    s->port.fifosize = 64;
    s->port.iobase = i+1;
    s->port.irq    = wk2xxx_irq;
    s->port.iotype = SERIAL_IO_PORT;
    s->port.flags  = ASYNC_BOOT_AUTOCONF;
    s->pdata.irq_gpio = pdata->irq_gpio;
    s->pdata.reset_gpio = pdata->reset_gpio;
    s->pdata.ls_gpio = pdata->ls_gpio;
    s->pdata.power_gpio = pdata->power_gpio;

    status = uart_add_one_port(&wk2xxx_uart_driver, &s->port);
    if(status<0)
    {
      //dev_warn(&spi->dev,"uart_add_one_port failed for line i:= %d with error %d\n",i,status);
      pr_info("uart_add_one_port failed for line i:= %d with error %d\n",i,status);
    }
  }
  pr_info("uart_add_one_port = 0x%d\n",status);
  mutex_unlock(&wk2xxxs_lock);
#ifdef _DEBUG_WK2XXX
  gpio_set_value(s->pdata.power_gpio,1);
  gpio_set_value(s->pdata.ls_gpio,1);
  gpio_set_value(s->pdata.reset_gpio,1);
  mdelay(5);
  for(i=1;i<17;i++)
  {
    wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA ,i);
    wk2xxx_read_reg(wk2xxx_i2c_client,1,WK2XXX_GENA,dat);
    pr_info(":WK2XXX_GENA=0x%X\n", dat[0]);

  }
  wk2xxx_write_reg(wk2xxx_i2c_client,1,WK2XXX_GENA ,0);
//  gpio_set_value(s->pdata.power_gpio,1);
 // gpio_set_value(s->pdata.ls_gpio,1);
  //gpio_set_value(s->pdata.reset_gpio,1);
  pr_info("-wk2xxx_probe()------exit---\n");
#endif
  return 0;

}


static int wk2xxx_remove(struct i2c_client  *i2c_client)
{

  int i;
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_remove()------in---\n");
#endif

  mutex_lock(&wk2xxxs_lock);
  for(i =0;i<NR_PORTS;i++)
  {
    struct wk2xxx_port *s = &wk2xxxs[i];
    uart_remove_one_port(&wk2xxx_uart_driver, &s->port);
  }
  pr_info("removing wk2xxx driver\n");
  uart_unregister_driver(&wk2xxx_uart_driver);
  mutex_unlock(&wk2xxxs_lock);
#ifdef _DEBUG_WK2XXX
  pr_info("-wk2xxx_remove()------exit---\n");
#endif

  return 0;
}

static int wk2xxx_resume( struct i2c_client  *i2c_client)
{
  pr_info("resume wk2xxx");
  return 0;
}
/*  platform driver */
#ifdef CONFIG_OF
static const struct of_device_id wk2xxx_dev_of_match[] = {
  {
    .compatible = "mediatek,wk2xxx_i2c",
  },
  {},
};
#endif
static const struct i2c_device_id wk2xxx_i2c_id_table[]={{"wk2xxx_i2c",0}, {} };

static struct i2c_driver wk2xxx_i2c_driver = {
  .id_table       = wk2xxx_i2c_id_table,
  .probe          = wk2xxx_probe,
  .remove         = wk2xxx_remove,
  .driver = {
    .name           = "wk2xxx_i2c",
    .owner          = THIS_MODULE,
#ifdef CONFIG_OF
    .of_match_table = wk2xxx_dev_of_match,
#endif
  },

};

MODULE_DEVICE_TABLE(i2c,wk2xxx_i2c_id_table);
static int __init wk2xxx_init(void)
{

  int retval;
  retval = i2c_add_driver(&wk2xxx_i2c_driver);
  return retval;
}



static void __exit wk2xxx_exit(void)
{
  i2c_del_driver(&wk2xxx_i2c_driver);
}


module_init(wk2xxx_init);
module_exit(wk2xxx_exit);

MODULE_AUTHOR("FE zhangqingzhan Ltd");
MODULE_DESCRIPTION("FE wk2xxx generic serial port driver");
MODULE_LICENSE("GPL");

