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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <m4u.h>

#include <linux/io.h> /*for mb();*/

#include "ccu_inc.h"
#include "ccu_hw.h"
#include "ccu_reg.h"
#include "ccu_cmn.h"
#include "ccu_kd_mailbox.h"
#include "ccu_i2c.h"

static uint64_t camsys_base;
static uint64_t bin_base;
static uint64_t dmem_base;

static ccu_device_t *ccu_dev;
static struct task_struct *enque_task;
static m4u_client_t *m4u_client;
static struct mutex cmd_mutex;
static wait_queue_head_t cmd_wait;
static volatile bool cmd_done;
static int32_t g_ccu_sensor_current_fps = -1;

#define SENSOR_NAME_MAX_LEN 32
static struct ccu_sensor_info g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {{0} };
static char g_ccu_sensor_name[IMGSENSOR_SENSOR_IDX_MAX_NUM][SENSOR_NAME_MAX_LEN];

volatile ccu_mailbox_t *pMailBox[MAX_MAILBOX_NUM];
static ccu_msg_t receivedCcuCmd;
static ccu_msg_t CcuAckCmd;
static uint32_t i2c_buffer_mva;

/*isr work management*/
struct ap_task_manage_t {
		struct workqueue_struct *ApTaskWorkQueue;
		struct mutex ApTaskMutex;
		struct list_head ApTskWorkList;
};

struct ap_task_manage_t ap_task_manage;


static CCU_INFO_STRUCT ccuInfo;
static volatile bool bWaitCond;
static unsigned int g_LogBufIdx = 1;

static int _ccu_powerdown(void);

static inline unsigned int CCU_MsToJiffies(unsigned int Ms)
{
	return ((Ms * HZ + 512) >> 10);
}


static inline void lock_command(void)
{
	mutex_lock(&cmd_mutex);
	cmd_done = false;
}

static inline int wait_command(void)
{
	return (wait_event_interruptible_timeout(cmd_wait, cmd_done,
						    msecs_to_jiffies(1 * 1000)) > 0) ? 0 : -ETIMEDOUT;
}

static inline void unlock_command(void)
{
	mutex_unlock(&cmd_mutex);
}


static void isr_sp_task(void)
{
	MUINT32 sp_task = ccu_read_reg(ccu_base, CCU_STA_REG_SP_ISR_TASK);
	MUINT32 i2c_transac_len;
	MBOOL i2c_do_dma_en;
	unsigned long flags;

	switch (sp_task) {
	case APISR_SP_TASK_TRIGGER_I2C:
		{
			LOG_DBG("isr_sp_task: APISR_SP_TASK_TRIGGER_I2C +++++\n");

			i2c_transac_len = ccu_read_reg(ccu_base, CCU_STA_REG_I2C_TRANSAC_LEN);
			i2c_do_dma_en = ccu_read_reg(ccu_base, CCU_STA_REG_I2C_DO_DMA_EN);

			/*LOG_DBG("i2c_transac_len: %d\n", i2c_transac_len);*/
			/*LOG_DBG("i2c_do_dma_en: %d\n", i2c_do_dma_en);*/

			/*Use spinlock to avoid trigger i2c after i2c cg turned off*/
			spin_lock_irqsave(&ccuInfo.SpinLockI2cPower, flags);
			if (ccuInfo.IsI2cPoweredOn == 1 && ccuInfo.IsI2cPowerDisabling == 0)
				ccu_trigger_i2c(i2c_transac_len, i2c_do_dma_en);
			spin_unlock_irqrestore(&ccuInfo.SpinLockI2cPower, flags);

			ccu_write_reg(ccu_base, CCU_STA_REG_SP_ISR_TASK, 0);

			LOG_DBG("isr_sp_task: APISR_SP_TASK_TRIGGER_I2C -----\n");
			break;
		}

	case APISR_SP_TASK_RESET_I2C:
		{
			LOG_DBG("isr_sp_task: APISR_SP_TASK_RESET_I2C +++++\n");

			ccu_i2c_frame_reset();

			ccu_write_reg(ccu_base, CCU_STA_REG_SP_ISR_TASK, 0);

			LOG_DBG("isr_sp_task: APISR_SP_TASK_RESET_I2C -----\n");
			break;
		}

	default:
		{
			LOG_DBG("no isr_sp_task: %x\n", sp_task);
			break;
		}
	}
}

#define CCU_ISR_WORK_BUFFER_SZIE 16
/*
* static atomic_t isr_work_idx_front = ATOMIC_INIT(0);
* static atomic_t isr_work_idx_rear = ATOMIC_INIT(0);
* static struct work_struct isr_works[CCU_ISR_WORK_BUFFER_SZIE];
*/

#if 0
static void isr_worker(struct work_struct *isr_work)
{
	LOG_DBG("+++++++");

	LOG_DBG("isr_worker, handling work with data:%ld\n", atomic_read(&isr_work->data));
	bWaitCond = true;
	g_LogBufIdx = (MUINT32) atomic_read(&isr_work->data);

	LOG_DBG("isr_worker, accuiring APTaskMutex\n");
	mutex_lock(&ap_task_manage.ApTaskMutex);
	LOG_DBG("isr_worker, got APTaskMutex, wakeup WaitQueueHead & unlock mutex\n");
	wake_up_interruptible(&ccuInfo.WaitQueueHead);
	mutex_unlock(&ap_task_manage.ApTaskMutex);

	/*vlist_node_of(ccuInfo.ApTskWorkList.next, work_struct_t);*/
	LOG_DBG("isr_worker, free isrWork:%p\n", isr_work);
	/*list_del_init(vlist_link(isr_work, work_struct_t));*/
	kfree(isr_work);
	LOG_DBG("-------");
}

void schedule_isr_worker(int data)
{
	work_struct_t *isrWork;

	LOG_DBG("+++++++schedule_isr_worker\n");

	LOG_DBG("allocate isrWork..\n");
	isrWork = kmalloc(sizeof(vlist_type(work_struct_t)), GFP_KERNEL);
	INIT_WORK(isrWork, isr_worker);
	atomic_set(&isrWork->data, data);
	/*isrWork->data = data;*/
	/*ATOMIC_LONG_SET_OP()*/
	/*list_add_tail(vlist_link(isrWork, work_struct_t), &ccuInfo.ApTskWorkList);*/

	LOG_DBG("schedule_work:%p\n", isrWork);
	LOG_DBG("ApTaskWorkQueue:%p\n", ap_task_manage.ApTaskWorkQueue);
	queue_work(ap_task_manage.ApTaskWorkQueue, isrWork);

	LOG_DBG("-------schedule_isr_worker\n");
}
#endif

irqreturn_t ccu_isr_handler(int irq, void *dev_id)
{
	mb_result mailboxRet;

	LOG_DBG("+++:%s\n", __func__);

	/*write clear mode*/
	LOG_DBG("write clear mode\n");
	ccu_write_reg(ccu_base, EINTC_CLR, 0xFF);
	LOG_DBG("read clear mode\n");
	ccu_read_reg(ccu_base, EINTC_ST);
	/**/

	isr_sp_task();

	while (1) {
		mailboxRet = mailbox_receive_cmd(&receivedCcuCmd);

		if (mailboxRet == MAILBOX_QUEUE_EMPTY)
			goto ISR_EXIT;

		switch (receivedCcuCmd.msg_id) {
#if 0
		case MSG_TO_APMCU_FLUSH_LOG:
			{
				LOG_DBG
						("got MSG_TO_APMCU_FLUSH_LOG:%d , wakeup ccuInfo.WaitQueueHead\n",
						receivedCcuCmd.in_data_ptr);
				schedule_isr_worker(receivedCcuCmd.msg_id);
				break;
			}
		case MSG_TO_APMCU_CCU_ASSERT:
			{
				LOG_DBG
						("got MSG_TO_APMCU_CCU_ASSERT:%d, wakeup ccuInfo.WaitQueueHead\n",
						receivedCcuCmd.in_data_ptr);
				schedule_isr_worker(receivedCcuCmd.msg_id);
				break;
			}
#else
		case MSG_TO_APMCU_FLUSH_LOG:
			{
				/*for ccu_waitirq();*/
				LOG_DBG
				    ("got MSG_TO_APMCU_FLUSH_LOG:%d , wakeup ccuInfo.WaitQueueHead\n",
				     receivedCcuCmd.in_data_ptr);
				bWaitCond = true;
				g_LogBufIdx = receivedCcuCmd.in_data_ptr;

				wake_up_interruptible(&ccuInfo.WaitQueueHead);
				LOG_DBG("wakeup ccuInfo.WaitQueueHead done\n");
				break;
			}

		case MSG_TO_APMCU_CCU_ASSERT:
			{
				LOG_ERR
				    ("got MSG_TO_APMCU_CCU_ASSERT:%d, wakeup ccuInfo.WaitQueueHead\n",
				     receivedCcuCmd.in_data_ptr);
				LOG_ERR
				    ("================== AP_ISR_CCU_ASSERT ===================\n");
				bWaitCond = true;
				g_LogBufIdx = 0xFFFFFFFF;	/* -1*/

				wake_up_interruptible(&ccuInfo.WaitQueueHead);
				LOG_ERR("wakeup ccuInfo.WaitQueueHead done\n");
				break;
			}

		case MSG_TO_APMCU_CCU_WARNING:
			{
				LOG_ERR
				    ("got MSG_TO_APMCU_CCU_WARNING:%d, wakeup ccuInfo.WaitQueueHead\n",
				     receivedCcuCmd.in_data_ptr);
				LOG_ERR
				    ("================== AP_ISR_CCU_WARNING ===================\n");
				bWaitCond = true;
				g_LogBufIdx = -2;

				wake_up_interruptible(&ccuInfo.WaitQueueHead);
				LOG_ERR("wakeup ccuInfo.WaitQueueHead done\n");
				break;
			}
#endif
		default:
			{
				LOG_DBG("got msgId: %d, cmd_wait\n", receivedCcuCmd.msg_id);
				ccu_memcpy(&CcuAckCmd, &receivedCcuCmd, sizeof(ccu_msg_t));

				cmd_done = true;
				wake_up_interruptible(&cmd_wait);
				break;
			}

		}
	}

ISR_EXIT:

	LOG_DBG("---:%s\n", __func__);

	/**/
	return IRQ_HANDLED;
}

static bool users_queue_is_empty(void)
{
	struct list_head *head;
	ccu_user_t *user;

	ccu_lock_user_mutex();

	list_for_each(head, &ccu_dev->user_list) {
		user = vlist_node_of(head, ccu_user_t);
		mutex_lock(&user->data_mutex);
		if (!list_empty(&user->enque_ccu_cmd_list)) {
			mutex_unlock(&user->data_mutex);
			ccu_unlock_user_mutex();
			return false;
		}
		mutex_unlock(&user->data_mutex);
	}

	ccu_unlock_user_mutex();

	return true;
}

static int ccu_enque_cmd_loop(void *arg)
{
	struct list_head *head;
	ccu_user_t *user;
	ccu_cmd_st *cmd;

	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	/*set_current_state(TASK_INTERRUPTIBLE);*/
	for (; !kthread_should_stop();) {
		LOG_DBG("+:%s\n", __func__);

		/* wait commands if there is no one in user's queue */
		LOG_DBG("wait for ccu_dev->cmd_wait\n");
		add_wait_queue(&ccu_dev->cmd_wait, &wait);
		while (1) {
			if (!users_queue_is_empty()) {
				LOG_DBG("awake & condition pass\n");
				break;
			}

			wait_woken(&wait, TASK_INTERRUPTIBLE, MAX_SCHEDULE_TIMEOUT);
			LOG_DBG("awake for ccu_dev->cmd_wait\n");
		}
		remove_wait_queue(&ccu_dev->cmd_wait, &wait);

		ccu_lock_user_mutex();

		/* consume the user's queue */
		list_for_each(head, &ccu_dev->user_list) {
			user = vlist_node_of(head, ccu_user_t);
			mutex_lock(&user->data_mutex);
			/* flush thread will handle the remaining queue if flush */
			if (user->flush || list_empty(&user->enque_ccu_cmd_list)) {
				mutex_unlock(&user->data_mutex);
				continue;
			}

			/* get first node from enque list */
			cmd = vlist_node_of(user->enque_ccu_cmd_list.next, ccu_cmd_st);

			list_del_init(vlist_link(cmd, ccu_cmd_st));
			user->running = true;
			mutex_unlock(&user->data_mutex);

			LOG_DBG("%s +:new command\n", __func__);
			ccu_send_command(cmd);

			mutex_lock(&user->data_mutex);
			list_add_tail(vlist_link(cmd, ccu_cmd_st), &user->deque_ccu_cmd_list);
			user->running = false;

			LOG_DBG("list_empty(%d)\n", (int)list_empty(&user->deque_ccu_cmd_list));

			mutex_unlock(&user->data_mutex);

			wake_up_interruptible_all(&user->deque_wait);

			LOG_DBG("wake_up user->deque_wait done\n");
			LOG_DBG("%s -:new command\n", __func__);
		}

		ccu_unlock_user_mutex();

		/* release cpu for another operations */
		usleep_range(1, 10);
	}

	LOG_DBG("-:%s\n", __func__);
	return 0;
}

int ccu_config_m4u_port(void)
{

	M4U_PORT_STRUCT port;

	port.ePortID = CCUG_OF_M4U_PORT;
	port.Virtuality = 1;
	port.Security = 0;
	port.domain = 3;
	port.Distance = 1;
	port.Direction = 0;

	return m4u_config_port(&port);
}

static void ccu_ap_task_mgr_init(void)
{
	mutex_init(&ap_task_manage.ApTaskMutex);
	/*
	*if (!(ap_task_manage.ApTaskWorkQueue))
	*{
	*	LOG_ERR("creating ApTaskWorkQueue !!\n");
	*	ap_task_manage.ApTaskWorkQueue = create_singlethread_workqueue("CCU_AP_WorkQueue");
	*}
	*if (!(ap_task_manage.ApTaskWorkQueue))
	*{
	*	LOG_ERR("create ApTaskWorkQueue error!!\n");
	*}
	*/
	/*INIT_LIST_HEAD(&ap_task_manage.ApTskWorkList);*/

}

int ccu_init_hw(ccu_device_t *device)
{
	int ret, n;

	m4u_client = m4u_create_client();
	/* init mutex */
	mutex_init(&cmd_mutex);
	/* init waitqueue */
	init_waitqueue_head(&cmd_wait);
	init_waitqueue_head(&ccuInfo.WaitQueueHead);
	/* init atomic task counter */
	/*ccuInfo.taskCount = ATOMIC_INIT(0);*/

	/* Init spinlocks */
	spin_lock_init(&(ccuInfo.SpinLockCcuRef));
	spin_lock_init(&(ccuInfo.SpinLockCcu));
	for (n = 0; n < CCU_IRQ_TYPE_AMOUNT; n++) {
		spin_lock_init(&(ccuInfo.SpinLockIrq[n]));
		spin_lock_init(&(ccuInfo.SpinLockIrqCnt[n]));
	}
	spin_lock_init(&(ccuInfo.SpinLockRTBC));
	spin_lock_init(&(ccuInfo.SpinLockClock));
	spin_lock_init(&(ccuInfo.SpinLockI2cPower));
	ccuInfo.IsI2cPoweredOn = 0;
	ccuInfo.IsI2cPowerDisabling = 0;
	/**/
	ccu_ap_task_mgr_init();

	ccu_base = device->ccu_base;
	camsys_base = device->camsys_base;
	bin_base = device->bin_base;
	dmem_base = device->dmem_base;

	ccu_dev = device;

	LOG_DBG("(0x%llx),(0x%llx),(0x%llx)\n", ccu_base, camsys_base, bin_base);

	ret = ccu_config_m4u_port();
	if (ret) {
		LOG_ERR("fail to config m4u port!\n");
		goto out;
	}

	if (request_irq(device->irq_num, ccu_isr_handler, IRQF_TRIGGER_NONE, "ccu", NULL)) {
		LOG_ERR("fail to request ccu irq!\n");
		ret = -ENODEV;
		goto out;
	}

	LOG_DBG("create ccu_enque_cmd_loop\n");
	enque_task = kthread_create(ccu_enque_cmd_loop, NULL, "ccu-enque");
	if (IS_ERR(enque_task)) {
		ret = PTR_ERR(enque_task);
		enque_task = NULL;
		goto out;
	}
	wake_up_process(enque_task);

out:
	return ret;
}

int ccu_uninit_hw(ccu_device_t *device)
{
	if (m4u_client != NULL)
		m4u_destroy_client(m4u_client);

	if (enque_task) {
		kthread_stop(enque_task);
		enque_task = NULL;
	}

	flush_workqueue(ap_task_manage.ApTaskWorkQueue);
	destroy_workqueue(ap_task_manage.ApTaskWorkQueue);

	return 0;
}

int ccu_mmap_hw(struct file *filp, struct vm_area_struct *vma)
{
#if 0
	int ret;
	unsigned long length = 0;
	void *va = NULL;

	length = (vma->vm_end - vma->vm_start);

	/* map the whole physically contiguous area in one piece */
	LOG_DBG("Vma->vm_pgoff(0x%lx),Vma->vm_start(0x%lx),Vma->vm_end(0x%lx),length(0x%lx)",
		vma->vm_pgoff, vma->vm_start, vma->vm_end, length);

	/* check length - do not allow larger mappings than the number of pages allocated */
	if (length > SIZE_1MB) {
		LOG_ERR("mmap range error! : length(%ld),CCU_RTBUF_REG_RANGE(%d)!", length,
			SIZE_1MB);
		return -EIO;
	}
	/**/
	switch (vma->vm_pgoff << PAGE_SHIFT) {
	case CCU_MMAP_LOG0:
		va = (void *)pLogBuf[0];
		break;
	case CCU_MMAP_LOG1:
		va = (void *)pLogBuf[1];
		break;
	case CCU_MMAP_LOG2:
		va = (void *)pLogBuf[2];
		break;
	default:
		LOG_ERR("0x%lx\n", vma->vm_pgoff << PAGE_SHIFT);
		return -1;
	}
	/**/
	ret = remap_pfn_range(vma,
				vma->vm_start,
				virt_to_phys((void *)va) >> PAGE_SHIFT,
				length, vma->vm_page_prot | PAGE_SHARED);

	if (ret < 0) {
		LOG_ERR("remap_pfn_range fail:\n");
		return ret;
	}
#endif
	return 0;
}

int ccu_get_i2c_dma_buf_addr(uint32_t *mva)
{
	int ret = 0;
	void *va;

	ret = i2c_get_dma_buffer_addr(&va);

	if (ret != 0)
		return ret;

	/*i2c dma buffer is PAGE_SIZE(4096B)*/
	/*--todo: need to dealloc i2c dma buf mva*/
	*mva = 0x10000000;
	ret = m4u_alloc_mva(m4u_client, CCUG_OF_M4U_PORT, (unsigned long)va, 0,
			4096, M4U_PROT_READ | M4U_PROT_WRITE,
			M4U_FLAGS_START_FROM, mva);

	if (ret)
		LOG_ERR("fail to allocate mva for the pointer to i2c_dma_buf_addr, ret=%d\n", ret);

		i2c_buffer_mva = *mva;

	return ret;
}

int ccu_memcpy(volatile void *dest, volatile void *src, int length)
{
	int i = 0;

	volatile char *destPtr = (volatile char *)dest;
	volatile char *srcPtr = (volatile char *)src;

	for (i = 0; i < length; i++)
		destPtr[i] = srcPtr[i];

	return length;
}

int ccu_memclr(volatile void *dest, int length)
{
	int i = 0;

	volatile char *destPtr = (volatile char *)dest;

	for (i = 0; i < length; i++)
		destPtr[i] = 0;

	return length;
}


int ccu_send_command(ccu_cmd_st *pCmd)
{
	int ret;
	/*unsigned int mva_buffers = 0;*/

	LOG_DBG("+:%s\n", __func__);

	lock_command();
	LOG_DBG("call ccu to do enque buffers\n");

	/* 1. push to mailbox_send */
	LOG_DBG("send command: id(%d), in(%x), out(%x)\n",
		pCmd->task.msg_id, pCmd->task.in_data_ptr, pCmd->task.out_data_ptr);
	mailbox_send_cmd(&(pCmd->task));

	/* 2. wait until done */
	LOG_DBG("wait ack command...\n");
	ret = wait_command();
	if (ret) {
		pCmd->status = CCU_ENG_STATUS_TIMEOUT;
		LOG_ERR("timeout to wait ack command\n");
		goto out;
	}

	pCmd->status = CCU_ENG_STATUS_SUCCESS;

	/* 3. fill pCmd with received command */
	ccu_memcpy(&pCmd->task, &CcuAckCmd, sizeof(ccu_msg_t));

	LOG_DBG("got ack command: id(%d), in(%x), out(%x)\n",
		pCmd->task.msg_id, pCmd->task.in_data_ptr, pCmd->task.out_data_ptr);

out:

	unlock_command();

	LOG_DBG("-:%s\n", __func__);

	return ret;

}

void ccu_set_current_fps(int32_t current_fps)
{
	g_ccu_sensor_current_fps = current_fps;
	LOG_DBG_MUST("ccu catch current fps :%d\n", current_fps);
}

int32_t ccu_get_current_fps(void)
{
	return g_ccu_sensor_current_fps;
}


int ccu_power(ccu_power_t *power)
{
	int ret = 0;
/*  unsigned int mva_buffers = 0;*/

	LOG_DBG("+:%s,(0x%llx)(0x%llx)\n", __func__, ccu_base, camsys_base);
	LOG_DBG("power->bON: %d\n", power->bON);

	if (power->bON == 1) {
		/*CCU power on sequence*/

		/*0. Set CCU_A_RESET. CCU_HW_RST=1*/
		ccu_write_reg(ccu_base, RESET, 0xFF3FFCFF);	/*TSF be affected.*/
		ccu_write_reg(ccu_base, RESET, 0x00010000);	/*CCU_HW_RST.*/
		LOG_DBG("reset wrote\n");
		/*ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 1);*/

		/*1. Enable CCU CAMSYS_CG_CON bit12 CCU_CGPDN=0*/
		/*CCU_CLR_BIT(camsys_base, 12);*/
		ccu_clock_enable();
		LOG_DBG("CG released\n");
		/*mdelay(1);*/
		/**/
		/*ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 0);*/

#if 0
		/*CAMSYS_SW_RST,CCU_RST*/
		CCU_SET_BIT(camsys_base+0x0c, 24);
		CCU_SET_BIT(camsys_base+0x0c, 25);
		mdelay(1);
		CCU_CLR_BIT(camsys_base+0x0c, 24);
		CCU_CLR_BIT(camsys_base+0x0c, 25);
#endif

		/*use user space buffer*/
		ccu_write_reg(ccu_base, CCU_DATA_REG_LOG_BUF0, power->workBuf.mva_log[0]);
		ccu_write_reg(ccu_base, CCU_DATA_REG_LOG_BUF1, power->workBuf.mva_log[1]);

		LOG_DBG("LogBuf_mva[0](0x%x)\n", power->workBuf.mva_log[0]);
		LOG_DBG("LogBuf_mva[1](0x%x)\n", power->workBuf.mva_log[1]);

		ccuInfo.IsI2cPoweredOn = 1;
		ccuInfo.IsCcuPoweredOn = 1;
	} else {
		/*CCU Power off*/
		ret = _ccu_powerdown();
	}

	LOG_DBG("-:%s\n", __func__);
	return ret;
}

int ccu_force_powerdown(void)
{
	int ret = 0;

	if (ccuInfo.IsCcuPoweredOn == 1) {
		LOG_WRN("CCU kernel drv released on CCU running, try to force shutdown\n");
		/*Set special isr task to MSG_TO_CCU_SHUTDOWN*/
		ccu_write_reg(ccu_base, CCU_INFO29, MSG_TO_CCU_SHUTDOWN);
		/*Interrupt to CCU*/
		ccu_write_reg(ccu_base, CCU_INT, 1);

		ret = _ccu_powerdown();

		if (ret < 0)
			return ret;

		LOG_WRN("CCU force shutdown success\n");
	}

	return 0;
}

static int _ccu_powerdown(void)
{
	int32_t timeout = 10;
	unsigned long flags;

	g_ccu_sensor_current_fps = -1;

	while (ccu_read_reg_bit(ccu_base, DONE_ST, CCU_HALT) == 0) {
		mdelay(1);
		LOG_DBG("wait ccu shutdown done\n");
		LOG_DBG("ccu shutdown stat: %x\n", ccu_read_reg_bit(ccu_base, DONE_ST, CCU_HALT));
		timeout = timeout - 1;
	}

	if (timeout <= 0) {
		LOG_ERR("_ccu_powerdown timeout\n");
		return -ETIMEDOUT;
	}

	/*Set CCU_A_RESET. CCU_HW_RST=1*/
	ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 1);
	/*CCF*/
	ccu_clock_disable();

	spin_lock_irqsave(&ccuInfo.SpinLockI2cPower, flags);
	ccuInfo.IsI2cPowerDisabling = 1;
	spin_unlock_irqrestore(&ccuInfo.SpinLockI2cPower, flags);

	ccu_i2c_buf_mode_en(0);
	ccuInfo.IsI2cPoweredOn = 0;
	ccuInfo.IsI2cPowerDisabling = 0;
	ccuInfo.IsCcuPoweredOn = 0;

	m4u_dealloc_mva(m4u_client, CCUG_OF_M4U_PORT, i2c_buffer_mva);

	return 0;
}

int ccu_run(void)
{
	int32_t timeout = 10;
	ccu_mailbox_t *ccuMbPtr = NULL;
	ccu_mailbox_t *apMbPtr = NULL;

	LOG_DBG("+:%s\n", __func__);

	/*smp_inner_dcache_flush_all();*/
	/*LOG_DBG("cache flushed 2\n");*/
	/*3. Set CCU_A_RESET. CCU_HW_RST=0*/
	ccu_write_reg_bit(ccu_base, RESET, CCU_HW_RST, 0);

	LOG_DBG("released CCU reset, wait for initial done\n");

	/*4. Pulling CCU init done spare register*/
	while ((ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE) != CCU_STATUS_INIT_DONE) && (timeout >= 0)) {
		mdelay(1);
		LOG_DBG("wait ccu initial done\n");
		LOG_DBG("ccu initial stat: %x\n", ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE));
		timeout = timeout - 1;
	}

	if (timeout <= 0) {
		LOG_ERR("CCU init timeout\n");
		return -ETIMEDOUT;
	}

	LOG_DBG("ccu initial done\n");
	LOG_DBG("ccu initial stat: %x\n", ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE));
	LOG_DBG("ccu initial debug info: %x\n", ccu_read_reg(ccu_base, CCU_INFO29));
	LOG_DBG("ccu initial debug info00: %x\n", ccu_read_reg(ccu_base, CCU_INFO00));
	LOG_DBG("ccu initial debug info01: %x\n", ccu_read_reg(ccu_base, CCU_INFO01));

	/*
	* 20160930
	* Due to AHB2GMC HW Bug, mailbox use SRAM
	* Driver wait CCU main initialize done and query INFO00 & INFO01 as mailbox address
	*/
	pMailBox[MAILBOX_SEND] =
	    (ccu_mailbox_t *)(uintptr_t)(dmem_base +
				       ccu_read_reg(ccu_base, CCU_DATA_REG_MAILBOX_CCU));
	pMailBox[MAILBOX_GET] =
	    (ccu_mailbox_t *)(uintptr_t)(dmem_base +
				       ccu_read_reg(ccu_base, CCU_DATA_REG_MAILBOX_APMCU));


	ccuMbPtr = (ccu_mailbox_t *) pMailBox[MAILBOX_SEND];
	apMbPtr = (ccu_mailbox_t *) pMailBox[MAILBOX_GET];

	mailbox_init(apMbPtr, ccuMbPtr);

	/*tell ccu that driver has initialized mailbox*/
	ccu_write_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE, 0);

	timeout = 10;
	while (ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE) != CCU_STATUS_INIT_DONE_2) {
		mdelay(1);
		LOG_DBG("wait ccu log test\n");
		timeout = timeout - 1;
	}

	if (timeout <= 0) {
		LOG_ERR("CCU init timeout 2\n");
		return -ETIMEDOUT;
	}

	LOG_DBG("ccu log test done\n");
	LOG_DBG("ccu log test stat: %x\n",
		ccu_read_reg(ccu_base, CCU_STA_REG_SW_INIT_DONE));
	LOG_DBG("ccu log test debug info: %x\n", ccu_read_reg(ccu_base, CCU_INFO29));

	LOG_DBG("-:%s\n", __func__);

	return 0;
}


int ccu_waitirq(CCU_WAIT_IRQ_STRUCT *WaitIrq)
{
	signed int ret = 0, Timeout = WaitIrq->EventInfo.Timeout;

	LOG_DBG("Clear(%d),bWaitCond(%d),Timeout(%d)\n", WaitIrq->EventInfo.Clear, bWaitCond, Timeout);
	LOG_DBG("arg is CCU_WAIT_IRQ_STRUCT, size:%zu\n", sizeof(CCU_WAIT_IRQ_STRUCT));

	if (Timeout != 0) {
		/* 2. start to wait signal */
		LOG_DBG("+:wait_event_interruptible_timeout\n");
		Timeout = wait_event_interruptible_timeout(ccuInfo.WaitQueueHead,
							   bWaitCond,
							   CCU_MsToJiffies(WaitIrq->EventInfo.
									   Timeout));
		bWaitCond = false;
		LOG_DBG("-:wait_event_interruptible_timeout\n");
	} else {
		LOG_DBG("+:ccu wait_event_interruptible\n");
		/*task_count_temp = atomic_read(&(ccuInfo.taskCount))*/
		/*if(task_count_temp == 0)*/
		/*{*/

		mutex_unlock(&ap_task_manage.ApTaskMutex);
		LOG_DBG("unlock ApTaskMutex\n");
		wait_event_interruptible(ccuInfo.WaitQueueHead, bWaitCond);
		LOG_DBG("accuiring ApTaskMutex\n");
		mutex_lock(&ap_task_manage.ApTaskMutex);
		LOG_DBG("got ApTaskMutex\n");
		/*}*/
		/*else*/
		/*{*/
		/*LOG_DBG("ccuInfo.taskCount is not zero: %d\n", task_count_temp);*/
		/*}*/
		bWaitCond = false;
		LOG_DBG("-:ccu wait_event_interruptible\n");
	}

	if (Timeout > 0) {
		LOG_DBG("remain timeout:%d, task: %d\n", Timeout, g_LogBufIdx);
		/*send to user if not timeout*/
		WaitIrq->EventInfo.TimeInfo.passedbySigcnt = (int)g_LogBufIdx;
	}
/*EXIT:*/

	return ret;
}

int ccu_flushLog(int argc, int *argv)
{
	LOG_DBG("bWaitCond(%d)\n", bWaitCond);

	bWaitCond = true;

	LOG_DBG("accuiring APTaskMutex, wakeup WaitQueueHead & unlock mutex\n");
	mutex_lock(&ap_task_manage.ApTaskMutex);
	LOG_DBG("got APTaskMutex, wakeup WaitQueueHead & unlock mutex\n");
	wake_up_interruptible(&ccuInfo.WaitQueueHead);
	mutex_unlock(&ap_task_manage.ApTaskMutex);
	LOG_DBG("unlock ApTaskMutex\n");

	LOG_DBG("bWaitCond(%d)\n", bWaitCond);
	return 0;
}

int ccu_i2c_ctrl(unsigned char i2c_write_id, int transfer_len)
{

	LOG_DBG("+:%s\n", __func__);

	/**/
	if (ccu_i2c_buf_mode_en(1) == -1) {
		LOG_DBG("i2c_buf_mode_en fail\n");
		return 0;
	}
	/*to set i2c buffer mode configuration*/
	{
		/*Transfer a dummy data, must>8 to let i2c drv go to dma mode*/
		ccu_init_i2c_buf_mode(i2c_write_id);
		ccu_config_i2c_buf_mode(transfer_len);
	}

	LOG_DBG("-:%s\n", __func__);

	return 0;
}

int ccu_read_info_reg(int regNo)
{
	int *offset = (int *)(uintptr_t)(ccu_base + 0x60 + regNo * 4);

	LOG_DBG("ccu_read_info_reg: %x\n", (unsigned int)(*offset));

	return *offset;
}

void ccu_set_sensor_info(int32_t sensorType, struct ccu_sensor_info *info)
{
	if (sensorType == IMGSENSOR_SENSOR_IDX_NONE) {
		/*Non-sensor*/
		LOG_ERR("No sensor been detected.\n");
	} else if ((sensorType >= IMGSENSOR_SENSOR_IDX_MIN_NUM) &&
		(sensorType < IMGSENSOR_SENSOR_IDX_MAX_NUM)) {
		g_ccu_sensor_info[sensorType].slave_addr  = info->slave_addr;
		g_ccu_sensor_info[sensorType].i2c_id  = info->i2c_id;
		if (info->sensor_name_string != NULL) {
			memcpy(g_ccu_sensor_name[sensorType],
			info->sensor_name_string, strlen(info->sensor_name_string)+1);
			g_ccu_sensor_info[sensorType].sensor_name_string = g_ccu_sensor_name[sensorType];
		}
		LOG_DBG_MUST("ccu catch sensor %d i2c slave address : 0x%x\n", sensorType, info->slave_addr);
		LOG_DBG_MUST("ccu catch sensor %d name : %s\n",
			sensorType, g_ccu_sensor_info[sensorType].sensor_name_string);
		LOG_DBG_MUST("ccu catch sensor %d i2c_id : %d\n", sensorType, g_ccu_sensor_info[sensorType].i2c_id);
	} else {
		LOG_DBG_MUST("ccu catch sensor i2c slave address fail!\n");
	}
}

void ccu_get_sensor_i2c_slave_addr(int32_t *sensorI2cSlaveAddr)
{
	sensorI2cSlaveAddr[0] = g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN].slave_addr;
	sensorI2cSlaveAddr[1] = g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_SUB].slave_addr;
	sensorI2cSlaveAddr[2] = g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN2].slave_addr;
}

void ccu_get_sensor_name(char **sensor_name)
{
	sensor_name[0] = g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN].sensor_name_string;
	sensor_name[1] = g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_SUB].sensor_name_string;
	sensor_name[2] = g_ccu_sensor_info[IMGSENSOR_SENSOR_IDX_MAIN2].sensor_name_string;
}

int ccu_query_power_status(void)
{
	return ccuInfo.IsCcuPoweredOn;
}
