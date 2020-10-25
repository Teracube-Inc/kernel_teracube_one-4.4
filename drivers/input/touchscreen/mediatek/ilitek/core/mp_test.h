/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __MP_TEST_H
#define __MP_TEST_H

struct mp_test_items
{
    char *name;
    /* The description must be the same as ini's section name */
    char *desp;
    char *result;
    int catalog;
    uint8_t cmd;
    bool run;
    int max;
    int min;
    int frame_count;
    int32_t* buf;
    int32_t* max_buf;
    int32_t* min_buf;
	int (*do_test)(int index);
};

struct core_mp_test_data
{
    /* A flag shows a test run in particular */
    bool m_signal;
    bool m_dac;
	bool s_signal;
	bool s_dac;
	bool key_dac;
    bool st_dac;
    bool p_no_bk;
    bool p_has_bk;
    bool open_integ;
    bool open_cap;

    int xch_len;
    int ych_len;
    int stx_len;
    int srx_len;
    int key_len;
    int st_len;
    int frame_len;
    int mp_items;
    bool final_result;

    /* Tx/Rx threshold & buffer */
    int TxDeltaMax;
    int TxDeltaMin;
    int RxDeltaMax;
    int RxDeltaMin;
    int32_t *tx_delta_buf;
    int32_t *rx_delta_buf;
    int32_t *tx_max_buf;
    int32_t *tx_min_buf;
    int32_t *rx_max_buf;
    int32_t *rx_min_buf;
};

extern struct core_mp_test_data *core_mp;
extern struct mp_test_items tItems[];

extern void core_mp_test_free(void);
extern void core_mp_show_result(void);
extern int core_mp_run_test(void);
extern int core_mp_move_code(void);
extern int core_mp_init(void);
extern void core_mp_remove(void);

#endif
