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

struct _test_chmem_regions {
	uint64_t pa;
	uint32_t size;
	uint32_t region_id;
};

/*limit: PA & Size needs 2MB   alignment (p80)*/
/*limit: PA & Size needs 64KB alignment (p60)*/
/*test data will be replaced with SSMR setting*/
struct _test_chmem_regions _test_CM_ary[] = {
	{0x000000000, 0x000000000, 0x0}, /*320MB: (ION)secure shared mem*/
	{0x000000000, 0x000000000, 0x0}, /*32MB: secure camera*/
	{0x000000000, 0x000000000, 0x0}, /*2MB : TA elf*/
	{0x000000000, 0x000000000, 0x0}, /*4KB : TEE/MTEE shmem*/
	{0x000000000, 0x000000000, 0x0}  /*50MB: heap/stack*/
};
struct AllocParameters {
	int size;
	int alignment;
	int flags;
};

#define ION_FLAG_MM_HEAP_INIT_ZERO 1
struct AllocParameters alloc_test_params[] = {
	{1024,      0,       0},	/*SIZE_1K*/
	{2048,      0,       0},	/*SIZE_2K*/
	{4096,      0,       0},	/*SIZE_4K*/
	{8192,      0,       0},	/*SIZE_8K*/
	{16384,     0,       0},	/*SIZE_16K*/
	{32768,     0,       0},	/*SIZE_32K*/
	{65536,     0,       0},	/*SIZE_64K*/
	{131072,    0,       0},	/*SIZE_128K*/
	{262144,    0,       0},	/*SIZE_256K*/
	{524288,    0,       0},	/*SIZE_512K*/
	{1048576,   0,       0},	/*SIZE_1M*/
	{2097152,   0,       0},	/*SIZE_2M*/
	{4194304,   0,       0},	/*SIZE_4M*/
	{8388608,   0,       0},	/*SIZE_8M*/
	{16777216,  0,       0},	/*SIZE_16M*/
	{33554432,  0,       0},	/*SIZE_32M*/
	{67108864,  0,       0}		/*SIZE_64M*/
};

struct AllocParameters alloc_zero_test_params[] = {
	{1024,      0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_1K*/
	{2048,      0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_2K*/
	{4096,      0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_4K*/
	{8192,      0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_8K*/
	{16384,     0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_16K*/
	{32768,     0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_32K*/
	{65536,     0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_64K*/
	{131072,    0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_128K*/
	{262144,    0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_256K*/
	{524288,    0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_512K*/
	{1048576,   0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_1M*/
	{2097152,   0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_2M*/
	{4194304,   0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_4M*/
	{8388608,   0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_8M*/
	{16777216,  0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_16M*/
	{33554432,  0, ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_32M*/
	{67108864,  0, ION_FLAG_MM_HEAP_INIT_ZERO}	/*SIZE_64M*/
};

struct AllocParameters alloc_aligned_test_params[] = {
	{1024,      1024,           0},	/*SIZE_1K*/
	{2048,      2048,           0},	/*SIZE_2K*/
	{4096,      4096,           0},	/*SIZE_4K*/
	{8192,      8192,           0},	/*SIZE_8K*/
	{16384,     16384,          0},	/*SIZE_16K*/
	{32768,     32768,          0},	/*SIZE_32K*/
	{65536,     65536,          0},	/*SIZE_64K*/
	{131072,    131072,         0},	/*SIZE_128K*/
	{262144,    262144,         0},	/*SIZE_256K*/
	{524288,    524288,         0},	/*SIZE_512K*/
	{1048576,   1048576,        0},	/*SIZE_1M*/
	{2097152,   2097152,        0},	/*SIZE_2M*/
	{4194304,   4194304,        0},	/*SIZE_4M*/
	{8388608,   8388608,        0},	/*SIZE_8M*/
	{16777216,  16777216,       0},	/*SIZE_16M*/
	{33554432,  33554432,       0},	/*SIZE_32M*/
	{67108864,  67108864,       0}	/*SIZE_64M*/
};

struct AllocParameters alloc_zero_aligned_test_params[] = {
	{1024,      1024,           ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_1K*/
	{2048,      2048,           ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_2K*/
	{4096,      4096,           ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_4K*/
	{8192,      8192,           ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_8K*/
	{16384,     16384,          ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_16K*/
	{32768,     32768,          ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_32K*/
	{65536,     65536,          ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_64K*/
	{131072,    131072,         ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_128K*/
	{262144,    262144,         ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_256K*/
	{524288,    524288,         ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_512K*/
	{1048576,   1048576,        ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_1M*/
	{2097152,   2097152,        ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_2M*/
	{4194304,   4194304,        ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_4M*/
	{8388608,   8388608,        ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_8M*/
	{16777216,  16777216,       ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_16M*/
	{33554432,  33554432,       ION_FLAG_MM_HEAP_INIT_ZERO},	/*SIZE_32M*/
	{67108864,  67108864,       ION_FLAG_MM_HEAP_INIT_ZERO}	/*SIZE_64M*/
};
