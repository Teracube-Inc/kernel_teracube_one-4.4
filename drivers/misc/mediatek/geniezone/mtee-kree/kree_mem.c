/*
 * Copyright (C) 2015 MediaTek Inc.
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


#include <kree/mem.h>
#include <kree/system.h>
#include <tz_cross/ta_mem.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#define EACH_MAP_ENTRY_SIZE sizeof(struct KREE_SHM_RUNLENGTH_ENTRY)

#define MAX_PA_ENTRY (256/EACH_MAP_ENTRY_SIZE)
#define MAX_MARY_SIZE MAX_PA_ENTRY
#define MAX_NUM_OF_PARAM 3

#define DBG_KREE_MEM
#ifdef DBG_KREE_MEM
#define KREE_DEBUG(fmt...) pr_debug("[KREE_MEM]"fmt)
#define KREE_ERR(fmt...) pr_debug("[KREE_MEM][ERR]"fmt)
#else
#define KREE_DEBUG(fmt...)
#define KREE_ERR(fmt...) pr_debug("[KREE_MEM][ERR]"fmt)
#endif

DEFINE_MUTEX(shared_mem_mutex);
DEFINE_MUTEX(chunk_mem_mutex);

/*Translate mem_handle to ION_handle*/
/*If this is optimized. set IONHandle_Transfer:0*/
/*IONHandle_Transfer=1, as usual*/
#define IONHandle_Transfer 1	/*1:ION_handle opt not impl*/

#if IONHandle_Transfer

#define MAX_TEST_CHM_SIZE (320*1024*1024)	/*320MB*/
#define MIN_TEST_ALLOC_CHM_SIZE 4096	/*4KB*/
#define t_size (MAX_TEST_CHM_SIZE/MIN_TEST_ALLOC_CHM_SIZE)
struct ION_MEM_TABLE {
	KREE_SECUREMEM_HANDLE memHandle;
	KREE_ION_HANDLE ionHandle;
};

struct ION_MEM_TABLE ion_mem_handleAry[t_size];	/*~640KB*/

#endif

/* notiec: handle type is the same */
static inline TZ_RESULT _allocFunc(uint32_t cmd, KREE_SESSION_HANDLE session,
				KREE_SECUREMEM_HANDLE *mem_handle, uint32_t alignment,
				uint32_t size, char *dbg)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if ((mem_handle == NULL) || (size == 0)) {
		KREE_ERR("_allocFunc: invalid parameters\n");
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	p[0].value.a = alignment;
	p[1].value.a = size;
	ret = KREE_TeeServiceCall(session, cmd,
					TZ_ParamTypes3(TZPT_VALUE_INPUT,
							TZPT_VALUE_INPUT,
							TZPT_VALUE_OUTPUT),
					p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("%s Error: 0x%x\n", dbg, ret);
		return ret;
	}

	*mem_handle = (KREE_SECUREMEM_HANDLE)p[2].value.a;

	return TZ_RESULT_SUCCESS;
}

static inline TZ_RESULT _handleOpFunc(uint32_t cmd, KREE_SESSION_HANDLE session,
					  KREE_SECUREMEM_HANDLE mem_handle, char *dbg)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if (mem_handle == 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	p[0].value.a = (uint32_t) mem_handle;
	ret = KREE_TeeServiceCall(session, cmd,
					TZ_ParamTypes1(TZPT_VALUE_INPUT), p);
	if (ret < 0) {
		KREE_ERR("%s Error: 0x%x\n", dbg, ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}

static inline TZ_RESULT _handleOpFunc_1(uint32_t cmd,
					KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE mem_handle, uint32_t *count,
					char *dbg)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if ((mem_handle == 0) || (count == NULL))
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	p[0].value.a = (uint32_t) mem_handle;
	ret = KREE_TeeServiceCall(session, cmd,
					TZ_ParamTypes2(TZPT_VALUE_INPUT,
							TZPT_VALUE_OUTPUT),
					p);
	if (ret < 0) {
		KREE_ERR("%s Error: 0x%x\n", dbg, ret);
		*count = 0;
		return ret;
	}

	*count = p[1].value.a;

	return TZ_RESULT_SUCCESS;
}

static void add_shm_list_node(struct KREE_SHM_RUNLENGTH_LIST **tail, uint64_t start, uint32_t size)
{
	struct KREE_SHM_RUNLENGTH_LIST *mylist;

	if (!tail)
		return;

	/* add a record */
	mylist = kmalloc(sizeof(*mylist), GFP_KERNEL);
	mylist->entry.low = (uint32_t) (start & (0x00000000ffffffff));
	mylist->entry.high = (uint32_t) (start >> 32);
	mylist->entry.size = size;
	mylist->next = NULL;
	(*tail)->next = mylist;
	(*tail) = mylist;
}

struct KREE_SHM_RUNLENGTH_ENTRY *shmem_param_run_length_encoding(
		int numOfPA, int *runLeng_arySize, uint64_t *ary)
{
	int arySize = numOfPA + 1;
	int i = 0;
	uint64_t start;
	uint64_t now, next;
	uint32_t size = 1;
	int xx = 0;
	int idx = 0;

	struct KREE_SHM_RUNLENGTH_LIST *head, *tail;
	struct KREE_SHM_RUNLENGTH_LIST *curr;
	struct KREE_SHM_RUNLENGTH_ENTRY *runLengAry = NULL;

	/* compress by run length coding */
	KREE_DEBUG("[%s] ====> shmem_param_run_length_encoding. numOfPA=%d, runLeng_arySize=%d\n",
			__func__, numOfPA, *runLeng_arySize);

	head = kmalloc(sizeof(*head), GFP_KERNEL);
	head->next = NULL;
	tail = head;

	for (i = 1; i < arySize; i++) {
		now  = ary[i];

		if (size == 1)
			start = now;
		next = ary[i+1];

		if ((i+1) < arySize) {

			next = ary[i+1];
			if ((next-now) == (1*PAGE_SIZE)) {
				size++;
			} else {
				add_shm_list_node(&tail, start, size);
				size = 1; /* reset */
				xx++;
			}

		} else if (i == (arySize-1)) {	/* process the last entry */

			if ((ary[i] - start + (1*PAGE_SIZE)) == (size * PAGE_SIZE)) {

				add_shm_list_node(&tail, start, size);
				size = 1; /* reset */
				xx++;
			}
		}
	}

	*runLeng_arySize = xx;
	KREE_DEBUG("=====> runLeng_arySize = %d\n", *runLeng_arySize);

	runLengAry = kmalloc((*runLeng_arySize+1) * sizeof(struct KREE_SHM_RUNLENGTH_ENTRY), GFP_KERNEL);

	runLengAry[0].high = numOfPA;
	runLengAry[0].low  = *runLeng_arySize;
	runLengAry[0].size = 0x0;

	idx = 1;

	if (head->next != NULL) {
		curr = head->next;
		while (curr != NULL) {
			runLengAry[idx].high = curr->entry.high;
			runLengAry[idx].low = curr->entry.low;
			runLengAry[idx].size = curr->entry.size;

			idx++;
			tail = curr;
			curr = curr->next;

			kfree(tail);
		}
	}
	kfree(head);

#ifdef DBG_KREE_SHM
	for (idx = 0; idx <= xx; idx++)
		KREE_DEBUG("=====> runLengAry[%d]. high = 0x%x, low=0x%x, size = 0x%x\n",
				idx, runLengAry[idx].high, runLengAry[idx].low, runLengAry[idx].size);
#endif
	KREE_DEBUG("========> end of run length encoding =================================\n");

	return runLengAry;
}

static TZ_RESULT send_shm_cmd(int round, union MTEEC_PARAM *p, KREE_SESSION_HANDLE session
	, uint32_t numOfPA, uint32_t cmd)
{
	uint32_t paramTypes;
	TZ_RESULT ret = 0;

	p[3].value.a = round;
	p[3].value.b = numOfPA;
	paramTypes = TZ_ParamTypes4(TZPT_MEM_INPUT, TZPT_MEM_INPUT, TZPT_MEM_INPUT, TZPT_VALUE_INOUT);
	ret = KREE_TeeServiceCall(session, cmd, paramTypes, p);

	return ret;
}

static TZ_RESULT send_shm_ending_cmd(union MTEEC_PARAM *p, KREE_SESSION_HANDLE session
	, uint32_t numOfPA, uint32_t cmd)
{
	int i;
	uint32_t paramTypes;
	TZ_RESULT ret = 0;

	/* send ending command */
	for (i = 0; i < MAX_NUM_OF_PARAM; i++) {
		p[i].mem.buffer = NULL;
		p[i].mem.size = 0;
	}
	p[3].value.a = -99;
	p[3].value.b = numOfPA;
	paramTypes = TZ_ParamTypes4(TZPT_MEM_INPUT, TZPT_MEM_INPUT, TZPT_MEM_INPUT, TZPT_VALUE_INOUT);
	ret = KREE_TeeServiceCall(session, cmd, paramTypes, p);

	return ret;
}

#ifdef DBG_KREE_SHM
static void print_runlength_arr(union MTEEC_PARAM *p, int *prt_id)
{
	int i, j;
	int entry_num;
	struct KREE_SHM_RUNLENGTH_ENTRY *pp2;

	if (!p)
		return;

	for (i = 0; i < MAX_NUM_OF_PARAM; i++) {
		if (!p[i].mem.size)
			continue;

		pp2 = (struct KREE_SHM_RUNLENGTH_ENTRY *) p[i].mem.buffer;
		entry_num = (int)(p[i].mem.size/EACH_MAP_ENTRY_SIZE);

		KREE_DEBUG("[%s] p[%d].mem.buffer done --> size =0x%x [%d entries]\n",
			__func__, i, p[i].mem.size, entry_num);

		for (j = 0; j < entry_num; j++) {
			if ((j == 0) || j == (entry_num - 1)) {
				KREE_DEBUG("[%s][loop][%d].pp2[%d] high=0x%x,low=0x%x,size=0x%x\n",
					__func__, (*prt_id)++, j,
					(uint32_t) pp2[j].high, (uint32_t) pp2[j].low,
					(uint32_t) pp2[j].size);
			}
		}

	}
}
#endif

static void init_shm_params(union MTEEC_PARAM *p, int *arr)
{
	int i;

	for (i = 0; i < MAX_NUM_OF_PARAM; i++) {
		p[i].mem.buffer = NULL;
		p[i].mem.size = 0;
		if (arr)
			arr[i] = 0;
	}
}

static TZ_RESULT kree_register_cont_shm(union MTEEC_PARAM *p,
					KREE_SESSION_HANDLE session,
					void *start, uint32_t size, uint32_t cmd)
{
	TZ_RESULT ret = 0;
	int numOfPA;
#ifdef DBG_KREE_SHM
	int prt_id;
#endif
	struct KREE_SHM_RUNLENGTH_ENTRY *tmpAry = NULL;

	/* input data: ex: start = 0xaa000000; size = 8192(8KB) */
	KREE_DEBUG("[%s] Map Continuous Pages\n", __func__);

	/* init mtee param & other buffer */
	init_shm_params(p, NULL);

	numOfPA =  size / PAGE_SIZE;
	if ((size % PAGE_SIZE) != 0)
		numOfPA++;
	KREE_DEBUG("[%s] numOfPA= 0x%x\n", __func__, numOfPA);

	tmpAry = kmalloc((MAX_MARY_SIZE) * sizeof(struct KREE_SHM_RUNLENGTH_ENTRY), GFP_KERNEL);
	tmpAry[0].high = (uint32_t) ((uint64_t) start >> 32);
	tmpAry[0].low = (uint32_t) ((uint64_t) start & (0x00000000ffffffff));
	tmpAry[0].size = numOfPA;

	p[0].mem.buffer = tmpAry;
	p[0].mem.size = EACH_MAP_ENTRY_SIZE;

	KREE_DEBUG("[%s] [Case1]====> tmpAry[0] high= 0x%x, low= 0x%x, size= 0x%x\n",
			__func__,
			(uint32_t)tmpAry[0].high,
			(uint32_t)tmpAry[0].low,
			(uint32_t)tmpAry[0].size);

#ifdef DBG_KREE_SHM
	print_runlength_arr(p, &prt_id);
#endif
	ret = send_shm_cmd(0, p, session, numOfPA, cmd);
	ret = send_shm_ending_cmd(p, session, numOfPA, cmd);
	/* ------------------------------------------------------------------ */
	kfree(tmpAry);

	return ret;
}

static TZ_RESULT kree_register_desc_shm(union MTEEC_PARAM *p, KREE_SESSION_HANDLE session,
					void *start, uint32_t size, void *mapAry, uint32_t cmd)
{
	TZ_RESULT ret = 0;
	int numOfPA;
	int i, idx = 0, p_idx = 0, offset = 0;
	uint64_t *ary;
	int round = 0;
	int runLeng_arySize = 0;
#ifdef DBG_KREE_SHM
	int prt_id;
#endif
	int tmp_ary_entryNum[MAX_NUM_OF_PARAM];
	struct KREE_SHM_RUNLENGTH_ENTRY tmp_ary[MAX_NUM_OF_PARAM][MAX_MARY_SIZE];
	struct KREE_SHM_RUNLENGTH_ENTRY *runLengAry = NULL;

	KREE_DEBUG("[%s] Map Discontinuous Pages\n", __func__);

	/* init mtee param & other buffer */
	init_shm_params(p, tmp_ary_entryNum);

	ary = (uint64_t *) mapAry;
	numOfPA = ary[0];
	KREE_DEBUG("[%s] numOfPA = %d, MAX_MARY_SIZE = %lu\n", __func__, numOfPA, MAX_MARY_SIZE);

	/* encode page tables */
	runLengAry = shmem_param_run_length_encoding(numOfPA, &runLeng_arySize, ary);

#ifdef DBG_KREE_SHM
	for (i = 0; i <= numOfPA; i++)
		KREE_DEBUG("[%s] ====> mapAry[%d]= 0x%llx\n", __func__, i, (uint64_t) ary[i]);
	for (idx = 0; idx <= runLeng_arySize; idx++)
		KREE_DEBUG("=====> runLengAry[%d]. high = 0x%x, low=0x%x, size = 0x%x\n",
				idx, runLengAry[idx].high, runLengAry[idx].low, runLengAry[idx].size);
#endif

	/* start sending page tables... */
	idx = 1;
	do {
#ifdef DBG_KREE_SHM
		KREE_DEBUG("[%s]=====> idx [%d] runs.....\n", __func__, idx);
#endif
		if (idx % (MAX_NUM_OF_PARAM * MAX_MARY_SIZE) == 1) { /* each round restarts */

			if (tmp_ary_entryNum[0] > 0) {
				for (i = 0; i < MAX_NUM_OF_PARAM; i++) {
					p[i].mem.buffer = tmp_ary[i];
					/* #ofEntry * 12 Bytes */
					p[i].mem.size = tmp_ary_entryNum[i] * EACH_MAP_ENTRY_SIZE;
#ifdef DBG_KREE_SHM
					KREE_DEBUG("[%s] [loop] p[i].mem.size = %d [entryNum =%d]\n",
						__func__, p[i].mem.size, tmp_ary_entryNum[i]);
#endif
				}

#ifdef DBG_KREE_SHM
				prt_id = 1;
				print_runlength_arr(p, &prt_id);
#endif
				/* send a command */
				ret = send_shm_cmd(round, p, session, numOfPA, cmd);
				KREE_DEBUG("[%s]====> round %d done, restart\n", __func__, round);
				round++;
			}

			init_shm_params(p, tmp_ary_entryNum);
		}

		round = (int) ((idx-1) / (MAX_MARY_SIZE * MAX_NUM_OF_PARAM));
		p_idx = (int) ((idx-1) / MAX_MARY_SIZE) % MAX_NUM_OF_PARAM;
		offset = (int) (((idx-1) - (round * MAX_MARY_SIZE * MAX_NUM_OF_PARAM)) % MAX_MARY_SIZE);
		tmp_ary[p_idx][offset] = runLengAry[idx];
#ifdef DBG_KREE_SHM
		KREE_DEBUG("[%s] tmp_ary[%d][%d] high= 0x%x, low= 0x%x, size= 0x%x\n",
			__func__, p_idx, offset,
			tmp_ary[p_idx][offset].high, tmp_ary[p_idx][offset].low,
			tmp_ary[p_idx][offset].size);
#endif
		tmp_ary_entryNum[p_idx]++;
		idx++;

	} while (idx <= runLeng_arySize);

	KREE_DEBUG("[%s] do the last print (send command).\n", __func__);
	if (tmp_ary_entryNum[0] > 0) {
		for (i = 0; i < MAX_NUM_OF_PARAM; i++) {
			p[i].mem.buffer = tmp_ary[i];
			p[i].mem.size = tmp_ary_entryNum[i] * EACH_MAP_ENTRY_SIZE;
#ifdef DBG_KREE_SHM
			KREE_DEBUG("[%s] [last time] p[i].mem.size = %d [tmp_ary_entryNum =%d]\n",
				__func__, p[i].mem.size, tmp_ary_entryNum[i]);
#endif
		}

#ifdef DBG_KREE_SHM
		print_runlength_arr(p, &prt_id);
#endif
		/* send commnad. */
		KREE_DEBUG("===>  round = %d\n", round);
		ret = send_shm_cmd(round, p, session, numOfPA, cmd);
	}

	ret = send_shm_ending_cmd(p, session, numOfPA, cmd);
	kfree(runLengAry);

	return ret;
}

TZ_RESULT kree_register_sharedmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE *mem_handle,
					void *start, uint32_t size, void *mapAry, uint32_t cmd)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret = 0;
	int locktry;

	KREE_DEBUG("[%s] kree_register_sharedmem is calling.\n", __func__);

	/** FIXME: mutex should be removed after re-implement sending procedure **/
	do {
		locktry = mutex_lock_interruptible(&shared_mem_mutex);
		if (locktry && locktry != -EINTR) {
			KREE_DEBUG("mutex lock error: %d", locktry);
			return TZ_RESULT_ERROR_GENERIC;
		}
	} while (locktry);

	if (mapAry == NULL)
		ret = kree_register_cont_shm(p, session, start, size, cmd);
	else
		ret = kree_register_desc_shm(p, session, start, size, mapAry, cmd);

	mutex_unlock(&shared_mem_mutex); /* FIXME: should be removed */

	if (ret != TZ_RESULT_SUCCESS) {
		*mem_handle = 0;
		return ret;
	}
	*mem_handle = p[3].value.a;

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT kree_unregister_sharedmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE mem_handle)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	p[0].value.a = (uint32_t) mem_handle;
	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SHAREDMEM_UNREG,
				  TZ_ParamTypes1(TZPT_VALUE_INPUT), p);
	return ret;
}

TZ_RESULT kree_release_chunkmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE chm_handle)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	p[0].value.a = (uint32_t) chm_handle;
	ret = KREE_TeeServiceCall(session, TZCMD_MEM_RELEASE_CHUNKMEM_ION,
				  TZ_ParamTypes1(TZPT_VALUE_INPUT), p);
	return ret;
}
/* APIs
*/
TZ_RESULT KREE_RegisterSharedmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE *shm_handle,
					KREE_SHAREDMEM_PARAM *param)
{
	TZ_RESULT ret;

	if (shm_handle == NULL)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = kree_register_sharedmem(session, shm_handle,
						param->buffer, param->size,
						param->mapAry, TZCMD_MEM_SHAREDMEM_REG);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_RegisterSharedmem Error: 0x%x\n", ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT KREE_UnregisterSharedmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE shm_handle)
{
	TZ_RESULT ret;

	if (shm_handle == 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = kree_unregister_sharedmem(session, shm_handle);
	if (ret < 0) {
		KREE_ERR("KREE_UnregisterSharedmem Error: 0x%x\n", ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}

/*to-be-deleted: change to KREE_AppendSecurechunkmem()*/
#if 0
TZ_RESULT KREE_UnregisterChunkmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE cm_handle)
{
	TZ_RESULT ret;

	if (cm_handle == 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = kree_release_chunkmem(session, cm_handle);
	if (ret < 0) {
		KREE_ERR("KREE_UnregisterChunkmem Error: 0x%x\n", ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}
#endif

/*to-be-deleted: change to KREE_AppendSecurechunkmem()*/
#if 0
TZ_RESULT KREE_AllocChunkmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE *cm_handle,
					KREE_SHAREDMEM_PARAM *param)
{
	TZ_RESULT ret;

	if (cm_handle == NULL)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = kree_register_sharedmem(session, cm_handle,
						param->buffer, param->size,
						param->mapAry, TZCMD_MEM_APPEND_MULTI_CHUNKMEM);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_RegisterSharedmem Error: 0x%x\n", ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;

}
#endif

TZ_RESULT KREE_AllocSecuremem(KREE_SESSION_HANDLE session,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size)
{
	TZ_RESULT ret;

	ret =
		_allocFunc(TZCMD_MEM_SECUREMEM_ALLOC, session, mem_handle,
			alignment, size, "KREE_AllocSecuremem");

	return ret;
}

/*fix mtee sync*/
TZ_RESULT KREE_AllocSecurememWithTag(KREE_SESSION_HANDLE session,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size,
				const char *tag)
{
#if 0
	TZ_RESULT ret;

	ret =
	    _allocFunc(TZCMD_MEM_SECUREMEM_ALLOC_WITH_TAG, session, mem_handle,
			alignment, size, "KREE_AllocSecuremem", tag);

	return ret;
#endif
	KREE_DEBUG(" ===> %s: not support!\n", __func__);
	return -1;
}

TZ_RESULT KREE_ZallocSecuremem(KREE_SESSION_HANDLE session,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size)
{
	TZ_RESULT ret;

	ret =
		_allocFunc(TZCMD_MEM_SECUREMEM_ZALLOC, session, mem_handle,
			alignment, size, "KREE_ZallocSecuremem");

	return ret;
}

TZ_RESULT KREE_ZallocSecurememWithTag(KREE_SESSION_HANDLE session,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size)
{
	TZ_RESULT ret;

	ret =
		_allocFunc(TZCMD_MEM_SECUREMEM_ZALLOC, session, mem_handle,
			alignment, size, "KREE_ZallocSecurememWithTag");

	return ret;
}

TZ_RESULT KREE_ReferenceSecuremem(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE mem_handle)
{
	TZ_RESULT ret;

	ret =
		_handleOpFunc(TZCMD_MEM_SECUREMEM_REF, session, mem_handle,
				"KREE_ReferenceSecuremem");

	return ret;
}

TZ_RESULT KREE_UnreferenceSecuremem(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE mem_handle)
{
	TZ_RESULT ret;
	uint32_t count = 0;

	ret =
		_handleOpFunc_1(TZCMD_MEM_SECUREMEM_UNREF, session, mem_handle,
				&count, "KREE_UnreferenceSecuremem");
#ifdef DBG_KREE_MEM
	KREE_DEBUG("KREE_UnreferenceSecuremem: count = 0x%x\n", count);
#endif

	return ret;
}

TZ_RESULT KREE_ReleaseSecurechunkmem(KREE_SESSION_HANDLE session,
										uint32_t *size)
{
#if 0
	MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if (session == 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = UREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_RELEASE, TZ_ParamTypes1(TZPT_VALUE_OUTPUT), p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_DEBUG("[uree] UREE_ReleaseSecurechunkmem Error: 0x%x\n", ret);
		return ret;
	}

	*size = p[0].value.a;

	return TZ_RESULT_SUCCESS;
#endif
	KREE_DEBUG("%s is not supported.\n", __func__);
	return -1;
}

TZ_RESULT KREE_AppendSecurechunkmem(KREE_SESSION_HANDLE session)
{
#if 0
	MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if (session == 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = UREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_APPEND, 0, p);
	if (ret != TZ_RESULT_SUCCESS) {
#ifdef DBG_UREE_MEM
		printf("[uree] UREE_ReleaseSecurechunkmem Error: 0x%x\n", ret);
#endif
		return ret;
	}

	return TZ_RESULT_SUCCESS;
#endif
	KREE_DEBUG("%s is not supported.\n", __func__);
	return -1;
}

#if 1

TZ_RESULT KREE_AppendSecureMultichunkmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE *cm_handle,
					KREE_SHAREDMEM_PARAM *param)
{
	TZ_RESULT ret;

	if (cm_handle == NULL)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = kree_register_sharedmem(session, cm_handle,
						param->buffer, param->size,
						param->mapAry, TZCMD_MEM_APPEND_MULTI_CHUNKMEM_ION);
	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("KREE_RegisterSharedmem (chunk memory) Error: 0x%x\n", ret);

	return ret;
}

static inline TZ_RESULT _allocchmFunc(uint32_t cmd,
				KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size, char *dbg)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if ((chm_handle == 0) || (mem_handle == NULL) || (size == 0)) {
		KREE_ERR("_allocchmFunc: invalid parameters\n");
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	p[0].value.a = alignment;
	p[1].value.a = size;
	p[1].value.b = chm_handle;
	ret = KREE_TeeServiceCall(session, cmd,
					TZ_ParamTypes3(TZPT_VALUE_INPUT,
							TZPT_VALUE_INPUT,
							TZPT_VALUE_OUTPUT),
					p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("%s Error: 0x%x\n", dbg, ret);
		return ret;
	}

	*mem_handle = (KREE_SECUREMEM_HANDLE) p[2].value.a;
	KREE_DEBUG("Alloc ret mem_handle=0x%x\n", *mem_handle);

	if (*mem_handle == 0) {
		KREE_ERR("[%d]Alloc chmem=NULL: mem_handle=0x%x\n",
								__LINE__, *mem_handle);
		return TZ_RESULT_ERROR_GENERIC;
	}

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT KREE_AllocSecureMultichunkmem(KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size)
{
	TZ_RESULT ret;

	ret =
		_allocchmFunc(TZCMD_MEM_SECUREMULTICHUNKMEM_ALLOC,
			session, chm_handle, mem_handle,
			alignment, size, "KREE_Mem_AllocSecureMultichunkmem");

	KREE_DEBUG("[%d]after _allocchmFunc mem_handle=0x%x\n",
							__LINE__, *mem_handle);

	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("KREE_Mem_AllocSecureMultichunkmem Error: 0x%x\n", ret);

	return ret;
}

TZ_RESULT KREE_ZallocSecureMultichunkmem(KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_SECUREMEM_HANDLE *mem_handle,
				uint32_t alignment, uint32_t size)
{
	TZ_RESULT ret;

	ret =
		_allocchmFunc(TZCMD_MEM_SECUREMULTICHUNKMEM_ZALLOC,
			session, chm_handle, mem_handle,
			alignment, size, "KREE_Mem_ZallocSecureMultichunkmem");

	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("KREE_Mem_ZallocSecureMultichunkmem Error: 0x%x\n", ret);

	return ret;
}

TZ_RESULT KREE_ReferenceSecureMultichunkmem(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE mem_handle)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if (mem_handle == 0) {
		KREE_ERR("%s: invalid parameters\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	p[0].value.a = mem_handle;
	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECUREMULTICHUNKMEM_REF,
					TZ_ParamTypes1(TZPT_VALUE_INPUT), p);
	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("%s Error: 0x%x\n", __func__, ret);

	return ret;
}

TZ_RESULT KREE_UnreferenceSecureMultichunkmem(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE mem_handle)
{
	TZ_RESULT ret;
	uint32_t count = 0;
	union MTEEC_PARAM p[4];

	if (mem_handle == 0) {
		KREE_ERR("%s: invalid parameters\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	p[0].value.a = mem_handle;

	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECUREMULTICHUNKMEM_UNREF,
					TZ_ParamTypes2(TZPT_VALUE_INPUT, TZPT_VALUE_OUTPUT), p);

	count = p[1].value.a;

#ifdef DBG_KREE_MEM
	KREE_DEBUG("KREE_Mem_UnreferenceSecureMultichunkmem: count = 0x%x\n",
										count);
#endif

	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("KREE_Mem_UnreferenceSecureMultichunkmem Error: 0x%x\n", ret);

	return ret;
}

TZ_RESULT KREE_ReleaseSecureMultichunkmem(KREE_SESSION_HANDLE session,
					KREE_SHAREDMEM_HANDLE cm_handle)
{
	TZ_RESULT ret;

	if (cm_handle == 0)
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	ret = kree_release_chunkmem(session, cm_handle);
	if (ret < 0) {
		KREE_ERR("KREE_ReleaseSecureMultichunkmem Error: 0x%x\n", ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}

#if IONHandle_Transfer

TZ_RESULT _add_HandleMapping_ION_MEM(KREE_SECUREMEM_HANDLE mem_handle,
									KREE_ION_HANDLE IONHandle)
{
	TZ_RESULT ret = TZ_RESULT_ERROR_SHORT_BUFFER;
	int i;

	for (i = 0; i < (uint32_t) t_size; i++) {
		if ((ion_mem_handleAry[i].memHandle == 0)
			&& (ion_mem_handleAry[i].ionHandle == 0)) {
			ion_mem_handleAry[i].memHandle = mem_handle;
			ion_mem_handleAry[i].ionHandle = IONHandle;
			ret = TZ_RESULT_SUCCESS;
			break;
		}
	}
	return ret;
}

TZ_RESULT _del_HandleMapping_ION_MEM(KREE_SECUREMEM_HANDLE mem_handle,
								KREE_ION_HANDLE IONHandle)
{
	TZ_RESULT ret = TZ_RESULT_ERROR_ITEM_NOT_FOUND;
	int i;

	for (i = 0; i < (uint32_t) t_size; i++) {
		if ((ion_mem_handleAry[i].memHandle == mem_handle)
			&& (ion_mem_handleAry[i].ionHandle == IONHandle)) {
			ion_mem_handleAry[i].memHandle = 0;
			ion_mem_handleAry[i].ionHandle = 0;
			ret = TZ_RESULT_SUCCESS;
			break;
		}
	}
	return ret;
}

/*input mem_handle to get ION_Handle*/
TZ_RESULT KREE_ION_QueryIONHandle(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE mem_handle,
					KREE_ION_HANDLE *IONHandle)
{
	TZ_RESULT ret;
	union MTEEC_PARAM p[4];
	uint32_t ION_Handle = 0;

	if (mem_handle == 0) {
		KREE_ERR("%s: invalid parameters\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	p[0].value.a = mem_handle;

	ret = KREE_TeeServiceCall(session, TZCMD_MEM_Query_IONHandle,
					TZ_ParamTypes2(TZPT_VALUE_INPUT, TZPT_VALUE_OUTPUT), p);

	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_ION_QueryHandle Error: 0x%x\n", ret);
		return ret;
	}

	*IONHandle = p[1].value.a;
	KREE_DEBUG("==> KREE_ION_QueryHandle(): ret ION_Handle=0x%x\n",
										ION_Handle);

	return ret;
}
#endif	/*#if IONHandle_Transfer*/

uint32_t _IONHandle2MemHandle(KREE_ION_HANDLE IONHandle,
							KREE_SECUREMEM_HANDLE *mem_handle)
{
#if IONHandle_Transfer
	TZ_RESULT ret = TZ_RESULT_ERROR_ITEM_NOT_FOUND;
	int i;

	for (i = 0; i < (uint32_t) t_size; i++) {
		if (ion_mem_handleAry[i].ionHandle == IONHandle) {
			*mem_handle = ion_mem_handleAry[i].memHandle;
			ret = TZ_RESULT_SUCCESS;
			break;
		}
	}
	return ret;
#else
	*mem_handle = IONHandle;
	return TZ_RESULT_SUCCESS;
#endif
}

#endif

#if 1

TZ_RESULT _allocIONchmFunc(KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_ION_HANDLE *IONHandle,
				uint32_t alignment, uint32_t size, int isZalloc)
{
	TZ_RESULT ret;

	KREE_SECUREMEM_HANDLE mem_handle;

	*IONHandle = 0;

	/*allocate secure memory from a chunk memomry*/
	if (isZalloc == 1)
		ret = KREE_ZallocSecureMultichunkmem(session,
					chm_handle, &mem_handle, alignment, size);
	else
		ret = KREE_AllocSecureMultichunkmem(session,
					chm_handle, &mem_handle, alignment, size);

	KREE_DEBUG("[%d]after alloc mem_handle=0x%x\n",
							__LINE__, mem_handle);

	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_AllocSecureMultichunkmem Error: 0x%x\n", ret);
		return ret;
	}

	if (mem_handle == 0) {
		KREE_ERR("Alloc chmem=NULL: mem_handle=0x%x\n", mem_handle);
		return TZ_RESULT_ERROR_GENERIC;
	}

	*IONHandle = mem_handle;	/*init value*/

#if IONHandle_Transfer
	/*use mem_handle to get ION_Handle*/
	ret = KREE_ION_QueryIONHandle(session, mem_handle, IONHandle);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_ION_QueryIONHandle Error: 0x%x\n", ret);
		return ret;
	}
	KREE_DEBUG("get IONHandle:0x%x\n", (uint32_t) (*IONHandle));

	/*add (mem_handle, IONHandle) mapping into table*/
	ret = _add_HandleMapping_ION_MEM(mem_handle, (*IONHandle));
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("_add_HandleMapping_ION_MEM Error: 0x%x\n", ret);
		return ret;
	}
	KREE_DEBUG("add mapping table done\n");
#endif

	return TZ_RESULT_SUCCESS;

}

TZ_RESULT KREE_ION_AllocChunkmem(KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_ION_HANDLE *IONHandle,
				uint32_t alignment, uint32_t size)
{
	TZ_RESULT ret;

	ret = _allocIONchmFunc(session, chm_handle, IONHandle,
									alignment, size, 0);


	if ((ret != TZ_RESULT_SUCCESS) || (*IONHandle == 0)) {
		KREE_ERR("[%d]_allocIONchmFunc Error: ret=0x%x, IONHandle=0x%x\n",
						__LINE__, ret, *IONHandle);
		return TZ_RESULT_ERROR_GENERIC;
	}

	return ret;
}

TZ_RESULT KREE_ION_ZallocChunkmem(KREE_SESSION_HANDLE session,
				KREE_SHAREDMEM_HANDLE chm_handle,
				KREE_ION_HANDLE *IONHandle,
				uint32_t alignment, uint32_t size)
{
	TZ_RESULT ret;

	ret = _allocIONchmFunc(session, chm_handle, IONHandle,
									alignment, size, 1);	/*1:zalloc*/
	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("_allocIONchmFunc Error: 0x%x\n", ret);

	return ret;
}

TZ_RESULT KREE_ION_ReferenceChunkmem(KREE_SESSION_HANDLE session,
					KREE_ION_HANDLE IONHandle)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;
	KREE_SECUREMEM_HANDLE mem_handle;

	if (IONHandle == 0) {
		KREE_ERR("%s: invalid parameters\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	mem_handle = IONHandle;	/*init value*/

#if IONHandle_Transfer

	ret = _IONHandle2MemHandle(IONHandle, &mem_handle);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("_IONHandle2MemHandle Error: 0x%x\n", ret);
		return ret;
	}
#endif

	KREE_DEBUG("input IONHandle=0x%x, get mem_handle:0x%x\n",
		(uint32_t) IONHandle, (uint32_t) mem_handle);

	p[0].value.a = mem_handle;
	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECUREMULTICHUNKMEM_REF,
					TZ_ParamTypes1(TZPT_VALUE_INPUT), p);
	if (ret != TZ_RESULT_SUCCESS)
		KREE_ERR("%s Error: 0x%x\n", __func__, ret);

	return ret;
}

TZ_RESULT KREE_ION_UnreferenceChunkmem(KREE_SESSION_HANDLE session,
					KREE_ION_HANDLE IONHandle)
{
	TZ_RESULT ret;
	uint32_t count = 0;
	union MTEEC_PARAM p[4];
	KREE_SECUREMEM_HANDLE mem_handle;

	if (IONHandle == 0) {
		KREE_ERR("%s: invalid parameters\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	mem_handle = IONHandle; /*init value*/

#if IONHandle_Transfer

	ret = _IONHandle2MemHandle(IONHandle, &mem_handle);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("_IONHandle2MemHandle Error: 0x%x\n", ret);
		return ret;
	}
#endif

	KREE_DEBUG("input IONHandle=0x%x, get mem_handle:0x%x\n",
		(uint32_t) IONHandle, (uint32_t) mem_handle);

	p[0].value.a = mem_handle;

	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECUREMULTICHUNKMEM_UNREF,
					TZ_ParamTypes2(TZPT_VALUE_INPUT, TZPT_VALUE_OUTPUT), p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_Mem_UnreferenceSecureMultichunkmem Error: 0x%x\n", ret);
		return ret;
	}

	count = p[1].value.a;
	KREE_DEBUG("TZCMD_MEM_SECUREMULTICHUNKMEM_UNREF: count = 0x%x\n",
						count);

#if IONHandle_Transfer

	if (count == 0) {
		_del_HandleMapping_ION_MEM(mem_handle, IONHandle);

		if (ret != TZ_RESULT_SUCCESS) {
			KREE_ERR("_del_HandleMapping_ION_MEM Error: 0x%x\n", ret);
			return ret;
		}
	}
#endif

	return ret;
}

TZ_RESULT KREE_ION_QueryChunkmem_TEST(KREE_SESSION_HANDLE session,
					KREE_ION_HANDLE IONHandle, uint32_t cmd)
{
	union MTEEC_PARAM p[4];
	uint32_t paramTypes;

	TZ_RESULT ret;
	KREE_SECUREMEM_HANDLE mem_handle;

	if (IONHandle == 0) {
		KREE_ERR("%s: invalid parameters\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	mem_handle = IONHandle; /*init value*/

#if IONHandle_Transfer

	ret = _IONHandle2MemHandle(IONHandle, &mem_handle);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("_IONHandle2MemHandle Error: 0x%x\n", ret);
		return ret;
	}
#endif

	KREE_DEBUG("input IONHandle=0x%x, get mem_handle:0x%x\n",
		(uint32_t) IONHandle, (uint32_t) mem_handle);

	p[0].value.a = mem_handle;
	paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT, TZPT_VALUE_OUTPUT);

	ret = KREE_TeeServiceCall(session, cmd,	paramTypes, p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("%s Error: 0x%x\n", __func__, ret);
		return ret;
	}

	if (p[1].value.a != 0) {
		KREE_DEBUG("KREE_ION_QueryChunkmem_TEST Fail: mem_handle=0x%x\n",
											mem_handle);
		return TZ_RESULT_ERROR_GENERIC;	/*Query Fail*/
	}

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT KREE_ION_AccessChunkmem(KREE_SESSION_HANDLE session,
						union MTEEC_PARAM param[4], uint32_t cmd)
{
	uint32_t paramTypes;

	TZ_RESULT ret;
	KREE_SECUREMEM_HANDLE mem_handle;
	KREE_ION_HANDLE IONHandle;

	IONHandle = param[0].value.a;

	if (IONHandle == 0) {
		KREE_ERR("%s: invalid parameters\n", __func__);
		return TZ_RESULT_ERROR_BAD_PARAMETERS;
	}

	mem_handle = IONHandle; /*init value*/

#if IONHandle_Transfer

	ret = _IONHandle2MemHandle(IONHandle, &mem_handle);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("_IONHandle2MemHandle Error: 0x%x\n", ret);
		return ret;
	}
#endif

	KREE_DEBUG("input IONHandle=0x%x, get mem_handle:0x%x\n",
		(uint32_t) IONHandle, (uint32_t) mem_handle);

	param[0].value.a = mem_handle;	/*update param[0] to mem_handle*/

	paramTypes = TZ_ParamTypes4(TZPT_VALUE_INPUT, TZPT_VALUE_INPUT,
							TZPT_VALUE_INPUT, TZPT_VALUE_INOUT);

	ret = KREE_TeeServiceCall(session, cmd,	paramTypes, param);

	/*
	 * KREE_DEBUG("[%s] param[3].value.a =0x%x\n", __func__, param[3].value.a);
	*/

	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("%s Error: 0x%x\n", __func__, ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}

/*only for debug*/
TZ_RESULT KREE_ION_CP_Chm2Shm(KREE_SESSION_HANDLE session,
						KREE_ION_HANDLE ION_Handle,
						KREE_SECUREMEM_HANDLE shm_handle,
						uint32_t size)
{
	TZ_RESULT ret;
	/*copy chm data to shm.*/
	union MTEEC_PARAM param[4];

	param[0].value.a = ION_Handle;	/*need to transform to mem_handle*/
	param[0].value.b = shm_handle;
	param[1].value.a = size;		/*alloc size*/

	ret = KREE_ION_AccessChunkmem(session, param, TZCMD_MEM_CopyChmtoShm);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("%s Error: 0x%x\n", __func__, ret);
		return ret;
	}

	KREE_DEBUG("[%s] ret param[3].value.a =0x%x\n", __func__, param[3].value.a);
	return param[3].value.a;
}

#endif



TZ_RESULT KREE_AllocSecurechunkmem(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE *cm_handle,
					uint32_t alignment,
					uint32_t size)
{
	TZ_RESULT ret;

	ret =
		_allocFunc(TZCMD_MEM_SECURECM_ALLOC, session, cm_handle,
			alignment, size, "KREE_AllocSecurechunkmem");

	return ret;
}

/*fix mtee sync*/
TZ_RESULT KREE_AllocSecurechunkmemWithTag(KREE_SESSION_HANDLE session,
					KREE_SECUREMEM_HANDLE *cm_handle,
					uint32_t alignment,
					uint32_t size, const char *tag)
{
#if 0
	TZ_RESULT ret;

	ret =
	    _allocFunc(TZCMD_MEM_SECURECM_ALLOC_WITH_TAG, session, cm_handle,
			alignment, size, "KREE_AllocSecurechunkmem", tag);

	return ret;
#endif
	KREE_DEBUG(" ===> %s: not support!\n", __func__);
	return -1;
}

TZ_RESULT KREE_ReferenceSecurechunkmem(KREE_SESSION_HANDLE session,
					KREE_SECURECM_HANDLE cm_handle)
{
	TZ_RESULT ret;

	ret =
		_handleOpFunc(TZCMD_MEM_SECURECM_REF, session, cm_handle,
			  "KREE_ReferenceSecurechunkmem");

	return ret;
}

TZ_RESULT KREE_UnreferenceSecurechunkmem(KREE_SESSION_HANDLE session,
					 KREE_SECURECM_HANDLE cm_handle)
{
	TZ_RESULT ret;
	uint32_t count = 0;

	ret =
		_handleOpFunc_1(TZCMD_MEM_SECURECM_UNREF, session, cm_handle,
				&count, "KREE_UnreferenceSecurechunkmem");
#ifdef DBG_KREE_MEM
	KREE_DEBUG("KREE_UnreferenceSecurechunkmem: count = 0x%x\n", count);
#endif

	return ret;
}

TZ_RESULT KREE_ReadSecurechunkmem(KREE_SESSION_HANDLE session,
					uint32_t offset, uint32_t size, void *buffer)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if ((session == 0) || (size == 0))
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	p[0].value.a = offset;
	p[1].value.a = size;
	p[2].mem.buffer = buffer;
	p[2].mem.size = size;	/* fix me!!!! */
	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_READ,
					TZ_ParamTypes3(TZPT_VALUE_INPUT,
							TZPT_VALUE_INPUT,
							TZPT_MEM_OUTPUT),
					p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_ReadSecurechunkmem Error: 0x%x\n", ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT KREE_WriteSecurechunkmem(KREE_SESSION_HANDLE session, uint32_t offset,
					uint32_t size, void *buffer)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	if ((session == 0) || (size == 0))
		return TZ_RESULT_ERROR_BAD_PARAMETERS;

	p[0].value.a = offset;
	p[1].value.a = size;
	p[2].mem.buffer = buffer;
	p[2].mem.size = size;	/* fix me!!!! */
	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_WRITE,
					TZ_ParamTypes3(TZPT_VALUE_INPUT,
							TZPT_VALUE_INPUT,
							TZPT_MEM_INPUT),
					p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_WriteSecurechunkmem Error: 0x%x\n", ret);
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}


TZ_RESULT KREE_GetSecurechunkReleaseSize(KREE_SESSION_HANDLE session,
						uint32_t *size)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	ret =
		KREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_RSIZE,
				TZ_ParamTypes1(TZPT_VALUE_OUTPUT), p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_GetSecurechunkReleaseSize Error: 0x%x\n",
			ret);
		return ret;
	}

	*size = p[0].value.a;

	return TZ_RESULT_SUCCESS;
}

#if (GZ_API_MAIN_VERSION > 2)
TZ_RESULT KREE_StartSecurechunkmemSvc(KREE_SESSION_HANDLE session,
					unsigned long start_pa, uint32_t size)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	p[0].value.a = start_pa;
	p[1].value.a = size;
	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_START,
					TZ_ParamTypes2(TZPT_VALUE_INPUT,
							TZPT_VALUE_INPUT),
					p);
	if (ret != TZ_RESULT_SUCCESS) {
#ifdef DBG_KREE_MEM
		pr_debug("[kree] KREE_StartSecurechunkmemSvc Error: 0x%x\n", ret);
#endif
		return ret;
	}

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT KREE_StopSecurechunkmemSvc(KREE_SESSION_HANDLE session,
					unsigned long *cm_pa, uint32_t *size)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_STOP,
					TZ_ParamTypes2(TZPT_VALUE_OUTPUT,
							TZPT_VALUE_OUTPUT),
					p);
	if (ret != TZ_RESULT_SUCCESS) {
#ifdef DBG_KREE_MEM
		pr_debug("[kree] KREE_StopSecurechunkmemSvc Error: 0x%x\n", ret);
#endif
		return ret;
	}

	if (cm_pa != NULL)
		*cm_pa = (unsigned long)p[0].value.a;
	if (size != NULL)
		*size = p[1].value.a;

	return TZ_RESULT_SUCCESS;
}

TZ_RESULT KREE_QuerySecurechunkmem(KREE_SESSION_HANDLE session,
					unsigned long *cm_pa, uint32_t *size)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	ret = KREE_TeeServiceCall(session, TZCMD_MEM_SECURECM_QUERY,
					TZ_ParamTypes2(TZPT_VALUE_OUTPUT,
							TZPT_VALUE_OUTPUT),
					p);
	if (ret != TZ_RESULT_SUCCESS) {
#ifdef DBG_KREE_MEM
		pr_debug("[kree] KREE_QuerySecurechunkmem Error: 0x%x\n", ret);
#endif
		return ret;
	}

	if (cm_pa != NULL)
		*cm_pa = (unsigned long)p[0].value.a;
	if (size != NULL)
		*size = p[1].value.a;

	return TZ_RESULT_SUCCESS;
}
#endif /* GZ_API_MAIN_VERSION */

TZ_RESULT KREE_GetTEETotalSize(KREE_SESSION_HANDLE session, uint32_t *size)
{
	union MTEEC_PARAM p[4];
	TZ_RESULT ret;

	ret = KREE_TeeServiceCall(session, TZCMD_MEM_TOTAL_SIZE,
					TZ_ParamTypes1(TZPT_VALUE_OUTPUT), p);
	if (ret != TZ_RESULT_SUCCESS) {
		KREE_ERR("KREE_GetTEETotalSize Error: 0x%x\n", ret);
		return ret;
	}

	*size = p[0].value.a;

	return TZ_RESULT_SUCCESS;
}


