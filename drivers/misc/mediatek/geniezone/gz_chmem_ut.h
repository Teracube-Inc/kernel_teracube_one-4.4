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

#ifndef __GZ_CHMEM_UT_H__
#define __GZ_CHMEM_UT_H__

int chunk_memory_test(void *args);
int chunk_memory_test_ION_Multiple(void *args);
int chunk_memory_test_ION_simple(void *args);
int chunk_memory_test_by_MTEE_TA(void *args);

#endif
