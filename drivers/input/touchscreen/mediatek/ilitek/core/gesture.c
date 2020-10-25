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

#include "../common.h"
#include "finger_report.h"
#include "gesture.h"

/* The example for the gesture virtual keys */
#define GESTURE_DOUBLECLICK			    0x0
#define GESTURE_UP						0x1
#define GESTURE_DOWN					0x2
#define GESTURE_LEFT					0x3
#define GESTURE_RIGHT					0x4
#define GESTURE_M						0x5
#define GESTURE_W						0x6
#define GESTURE_C						0x7
#define GESTURE_E						0x8
#define GESTURE_V						0x9
#define GESTURE_O						0xA
#define GESTURE_S						0xB
#define GESTURE_Z						0xC

#define KEY_GESTURE_D					KEY_D
#define KEY_GESTURE_UP					KEY_UP
#define KEY_GESTURE_DOWN				KEY_DOWN
#define KEY_GESTURE_LEFT				KEY_LEFT
#define KEY_GESTURE_RIGHT				KEY_RIGHT
#define KEY_GESTURE_O					KEY_O
#define KEY_GESTURE_E					KEY_E
#define KEY_GESTURE_M					KEY_M
#define KEY_GESTURE_W					KEY_W
#define KEY_GESTURE_S					KEY_S
#define KEY_GESTURE_V					KEY_V
#define KEY_GESTURE_C					KEY_C
#define KEY_GESTURE_Z					KEY_Z

int core_gesture_key(uint8_t gdata)
{
    int gcode;

	switch (gdata)
    {
        case GESTURE_LEFT:
            gcode = KEY_GESTURE_LEFT;
            break;
        case GESTURE_RIGHT:
            gcode = KEY_GESTURE_RIGHT;
            break;
        case GESTURE_UP:
            gcode = KEY_GESTURE_UP;
            break;
        case GESTURE_DOWN:
            gcode = KEY_GESTURE_DOWN;
            break;
        case GESTURE_DOUBLECLICK:
            gcode = KEY_GESTURE_D;
            break;
        case GESTURE_O:
            gcode = KEY_GESTURE_O;
            break;
        case GESTURE_W:
            gcode = KEY_GESTURE_W;
            break;
        case GESTURE_M:
            gcode = KEY_GESTURE_M;
            break;
        case GESTURE_E:
            gcode = KEY_GESTURE_E;
            break;
        case GESTURE_S:
            gcode = KEY_GESTURE_S;
            break;
        case GESTURE_V:
            gcode = KEY_GESTURE_V;
            break;
        case GESTURE_Z:
            gcode = KEY_GESTURE_Z;
            break;
        case  GESTURE_C:
            gcode = KEY_GESTURE_C;
            break;
        default:
            gcode = -1;
            break;
    }

    DBG(DEBUG_GESTURE,"gcode = %d\n", gcode);
    return gcode;
}
EXPORT_SYMBOL(core_gesture_key);

void core_gesture_init(struct core_fr_data *fr_data)
{
    struct input_dev *input_dev = fr_data->input_device;

    if(input_dev != NULL)
    {
        input_set_capability(input_dev, EV_KEY, KEY_POWER);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
        input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

        __set_bit(KEY_POWER, input_dev->keybit);
        __set_bit(KEY_GESTURE_UP, input_dev->keybit);
        __set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
        __set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
        __set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
        __set_bit(KEY_GESTURE_O, input_dev->keybit);
        __set_bit(KEY_GESTURE_E, input_dev->keybit);
        __set_bit(KEY_GESTURE_M, input_dev->keybit);
        __set_bit(KEY_GESTURE_W, input_dev->keybit);
        __set_bit(KEY_GESTURE_S, input_dev->keybit);
        __set_bit(KEY_GESTURE_V, input_dev->keybit);
        __set_bit(KEY_GESTURE_Z, input_dev->keybit);
        __set_bit(KEY_GESTURE_C, input_dev->keybit);
        return;
    }

    DBG_ERR("GESTURE: input dev is NULL \n");
}
EXPORT_SYMBOL(core_gesture_init);