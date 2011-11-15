/*
 * cm11.h
 * Definitions for manipulating CM11
 *
 * Copyright (C) 2011 Ramax Lo <ramaxlo@gmail.com> 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef  CM11_H
#define  CM11_H
#include "x10proto.h"
#include <termios.h>
#include <unistd.h>

#define STATE(h, house, device)	(h->status[house][device])

#define TYPE_APPLIANCE		0x01
#define TYPE_LAMP		0x02
#define TYPE_MASK		0x07
#define TYPE(x)			((x) & TYPE_MASK)

#define BRIGHT_MASK		0xf8
#define BRIGHT_VALUE(x)		(((x) & 0x1f) << 3)
#define BRIGHT(x)		(((x) & BRIGHT_MASK) >> 3)

#define ON_MASK			0x100
#define ON_VALUE(x)		(((x) & 0x01) << 8)
#define ON(x)			(((x) & ON_MASK) >> 8)

#define DIR_MASK		0x200
#define DIR_VALUE(x)		(((x) & 0x01) << 9)
#define BIDIR(x)		(((x) & DIR_MASK) >> 9)

#define CM11_STATUS_REQ		0x8b

typedef struct cm11_handle
{
	s32 fd;
	s32 house;
	s32 device;
	u32 status[16][16];
	struct termios old_termio;
} cm11_handle;

typedef struct cm11_clock
{
	u8 cmd;	
	u8 seconds;
	u8 minutes;
	u8 hours;
	u8 yearday;
	u8 daymask;
	u8 house;
} cm11_clock;

typedef struct cm11_status
{
	u16 batt_timer;
	u8 seconds;
	u8 minutes;
	u8 hours;
	u8 yearday;
	u8 daymask;
	u8 revision:4;
	u8 house:4;
	u16 cur_device;
	u16 cur_on;
	u16 cur_dim;
} cm11_status;

typedef void (*cm11_notify)(cm11_handle *handle, s32 house, s32 device, u8 func, u8 value);

cm11_handle *cm11_init(s8 *device);
void cm11_close(cm11_handle *handle);
int cm11_set_clock(cm11_handle *handle);
int cm11_get_status(cm11_handle *handle, int dev, u32 *status);
int cm11_detect_poll(cm11_handle *handle, u8 *code);
int cm11_device_on(cm11_handle *handle, int id);
int cm11_device_off(cm11_handle *handle, int id);
int cm11_device_brightness(cm11_handle *handle, int id, int bright);
int cm11_set_device_type(cm11_handle *handle, int id, int type);
int cm11_ifce_status(cm11_handle *handle, cm11_status *status);
int cm11_set_device_dir(struct cm11_handle *handle, int dev, int two_way);
int cm11_receive_cmd(cm11_handle *handle);
int cm11_receive_notify(cm11_handle *handle, cm11_notify notify_fxn);

#endif   // CM11_H

