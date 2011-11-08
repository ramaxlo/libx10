/*
 * cm11.c
 * Functions for dealing with CM11 PC interface
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h>
#include <fcntl.h>
#include <time.h>
#include <x10/x10proto.h>
#include <x10/cm11.h>
#include "debug.h"

static int serial_init(cm11_handle *handle, s8 *device)
{
	int fd;
	char *dev = "/dev/ttyS0";
	struct termios new_termio;

	if(device)
		dev = device;
	fd = open(dev, O_RDWR);
	if(fd < 0)
	{
		perror(dev);	
		return -1;
	}

	tcgetattr(fd, &handle->old_termio);

	bzero(&new_termio, sizeof(new_termio));
	/* 4800bps, 8-n-1 */
	new_termio.c_cflag = CS8 | CLOCAL | CREAD;
	new_termio.c_iflag = 0;
	new_termio.c_oflag = 0;
//	new_termio.c_lflag = ICANON;
	new_termio.c_lflag = 0;
	new_termio.c_cc[VMIN] = 1;
	new_termio.c_cc[VTIME] = 0;
	cfsetspeed(&new_termio, B4800);

	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &new_termio);

	handle->fd = fd;

	return 0;
}

int cm11_clock_init(cm11_handle *handle)
{
	int rc;
	u8 code;

	rc = cm11_detect_poll(handle, &code);
	if(rc == 0)
	{
		INFO("CM11 has been initialized\n");
		rc = 0;
		goto out;
	}
	else if(rc < 0)
	{
		ERROR("Something wrong\n");
		rc = -1;
		goto out;
	}
	else if(code != 0xa5)
	{
		ERROR("Wrong poll cmd: 0x%x\n", code);	
		rc = -1;
		goto out;
	}

	rc = cm11_set_clock(handle);
out:
	return rc;
}

int cm11_detect_poll(cm11_handle *handle, u8 *code)
{
	return x10_detect_poll(handle->fd, code);
}

cm11_handle *cm11_init(s8 *device)
{
	cm11_handle *handle = NULL;

	handle = (cm11_handle *)malloc(sizeof(cm11_handle));
	if(handle == NULL)
	{
		perror(NULL);	
		return NULL;
	}
	bzero(handle, sizeof(cm11_handle));
	handle->device = DEV_NONE;

	if(!device)
		device = getenv("CM11_DEV");

	if(serial_init(handle, device))
		goto err_serial_init;
	if(cm11_clock_init(handle))
		goto err_clock;
			
	handle->house = HOUSE_A;

	return handle;

err_clock:
	cm11_close(handle);
	handle = NULL;
err_serial_init:
	if(handle)
		free(handle);
	return NULL;
}

void cm11_close(cm11_handle *handle)
{
	/* Recover old termio settings */
	tcflush(handle->fd, TCIOFLUSH);
	tcsetattr(handle->fd, TCSANOW, &handle->old_termio);
	close(handle->fd);
	free(handle);
}

int cm11_set_clock(cm11_handle *handle)
{
	int rc = 0;
	time_t tv;
	struct tm *tm;
	cm11_clock clk;
	u8 checksum = 0;
	int i;

	time(&tv);
	tm = localtime(&tv);
	clk.cmd = 0x9b;
	clk.seconds = tm->tm_sec;
	clk.minutes = tm->tm_min + 60 * (tm->tm_hour & 1);
	clk.hours = tm->tm_hour >> 1;
	clk.yearday = tm->tm_yday;
	clk.daymask = 1 << tm->tm_wday;
	if(tm->tm_yday & 0x100)
		clk.daymask |= 0x80;
	clk.house = 0x60;

	for(i = 1; i < sizeof(clk); i++)
		checksum += ((unsigned char *)&clk)[i];

	write(handle->fd, &clk, sizeof(clk));

	if(wait_for_ack(handle->fd, checksum))
	{
		ERROR("Checksum error\n");	
		rc = -1;
		goto out;
	}

	i = 0;
	write(handle->fd, &i, 1);

	if(wait_for_ack(handle->fd, 0x55))
	{
		ERROR("Ack error\n");	
		rc = -1;
		goto out;
	}

	INFO("Set clock done\n");
out:
	return rc;
}

int cm11_device_on(cm11_handle *handle, int id)
{
	u32 *p_stat;

	if(CHECK_ID(id))
	{
		ERROR("Wrong device number\n");
		return -1;
	}

	x10_send(handle->fd, handle->house, id, FUN_ON, 0);

	p_stat = &STATE(handle, handle->house, id);
	if(TYPE(*p_stat) == TYPE_LAMP)
	{
		*p_stat &= ~BRIGHT_MASK;
		*p_stat |= BRIGHT_VALUE(22);
	}
	*p_stat &= ~ON_MASK;
	*p_stat |= ON_VALUE(1);
	handle->device = id;
	
	return 0;
}

int cm11_device_off(cm11_handle *handle, int id)
{
	u32 *p_stat;

	if(CHECK_ID(id))
	{
		ERROR("Wrong device number\n");
		return -1;
	}

	x10_send(handle->fd, handle->house, id, FUN_OFF, 0);

	p_stat = &STATE(handle, handle->house, id);
	*p_stat &= ~ON_MASK;
	handle->device = id;

	return 0;
}

int cm11_set_house(cm11_handle *handle, int house)
{
	if(CHECK_ID(house))
	{
		ERROR("Wrong house number\n");
		return -1;
	}

	handle->house = house;

	return 0;
}

int cm11_set_device_type(cm11_handle *handle, int id, int type)
{
	u32 *p_stat;

	if(CHECK_ID(id))
	{
		ERROR("Wrong device number\n");
		return -1;
	}
	else if(type != TYPE_APPLIANCE && type != TYPE_LAMP)
	{
		ERROR("Wrong type\n");
		return -1;	
	}

	p_stat = &STATE(handle, handle->house, id);
	*p_stat &= ~TYPE_MASK;
	*p_stat |= TYPE(type);

	return 0;
}

int cm11_device_brightness(cm11_handle *handle, int id, int bright)
{
	int cur_bright;
	u32 *p_stat;
	int offset;

	if(CHECK_ID(id))
	{
		ERROR("Wrong device number\n");
		return -1;
	}
	else if(bright < 1) 
	{
		bright = 1;
		DBG("Too small brightness\n");
	}
	else if(bright > 22)
	{
		bright = 22;
		DBG("Too large brightness\n");
	}

	p_stat = &handle->status[handle->house][id];
	cur_bright = BRIGHT(*p_stat);
	offset = bright - cur_bright;
	if(offset > 0)
		x10_send(handle->fd, handle->house, id, FUN_BRIGHT, offset);
	else if(offset < 0)
		x10_send(handle->fd, handle->house, id, FUN_DIM, -offset);

	*p_stat &= ~BRIGHT_MASK;
	*p_stat |= BRIGHT_VALUE(bright);
	handle->device = id;

	return 0;	
}

static inline int parse_addr(u8 addr, u8 *house, u8 *dev)
{
	*house = code2idx((addr & 0xf0) >> 4);
	*dev = code2idx(addr & 0x0f);

	return 0;	
}

static inline int parse_func(u8 code, u8 *func)
{
	*func = (code & 0x0f);

	return 0;	
}

static void __update_state2(u32 **p, int n, u32 mask, u32 value)
{
	int i;

	for(i = 0; i < n; i++)
	{
		u32 tmp = *(p[i]);

		/*
		 * If the state has been set before, we won't do it again.
		 */
		if((tmp & mask) == value)
			continue;
		tmp &= ~mask;	
		tmp |= value;	

		/*
		 * If the unit is a lamp, reset its brightness to the highest.
		 */
		if((value == ON_VALUE(1)) && (TYPE(tmp) == TYPE_LAMP))
		{
			tmp &= ~BRIGHT_MASK;
			tmp |= BRIGHT_VALUE(22);
		}

		*(p[i]) = tmp;
	}
}

static void __update_brightness(u32 **p, int n, int value)
{
	int i;
	int tmp;

	for(i = 0; i < n; i++)
	{
		tmp = (int)BRIGHT(*(p[i]));
		tmp += value;
		if(tmp < 1)
			tmp = 1;
		else if(tmp > 22)
			tmp = 22;

		*(p[i]) &= ~BRIGHT_MASK;
		*(p[i]) |= BRIGHT_VALUE((u32)tmp);	
		DBG("Brightness: %d\n", tmp);
	}
}

static int __update_state(u32 **p, int n, u8 **buf)
{
	u8 func;
	int shift = 1;
	parse_func(**buf, &func);

	switch(func)
	{
		u32 tmp;
		case FUN_DIM:
			tmp = *(*buf + 1) * 22 / 210;
			__update_brightness(p, n, -tmp);
			(*buf) += 2;
			shift = 2;
			break;
		case FUN_BRIGHT:
			tmp = *(*buf + 1) * 22 / 210;
			__update_brightness(p, n, tmp);
			(*buf) += 2;
			shift = 2;
			break;	
		case FUN_STAT_ON:
			__update_state2(p, n, ON_MASK, ON_VALUE(1));
			(*buf)++;
			break;
		case FUN_STAT_OFF:
			__update_state2(p, n, ON_MASK, ON_VALUE(0));
			(*buf)++;
			break;
		case FUN_ON:
			__update_state2(p, n, ON_MASK, ON_VALUE(1));
			(*buf)++;
			break;
		case FUN_OFF:
			__update_state2(p, n, ON_MASK, ON_VALUE(0));
			(*buf)++;
			break;
		default:
			break;
	}

	return shift;
}

static void __parse_update(cm11_handle *handle, int dev, struct read_buf *buf)
{
	u32 mask;
	u32 *ptrs[16] = {NULL};
	int idx = 0;
	u8 *p;

	mask = buf->mask;
	p = &buf->data0;

//	if(dev == DEV_NONE && handle->device != DEV_NONE)
//	{
//		dev = handle->device;
//		ptrs[idx] = &STATE(handle, handle->house, dev);
//		idx++;
//	}
//	else if(dev != DEV_NONE)
//	{
//		ptrs[idx] = &STATE(handle, handle->house, dev);
//		idx++;
//	}

	while(mask)
	{
		switch(mask & 0x01)
		{
			u8 h, d, shift;
			case 0:
				parse_addr(*p, &h, &d);
				ptrs[idx] = &STATE(handle, h, d);
				handle->device = d;
				idx++;
				p++;
				mask >>= 1;
				DBG("shift 1\n");
				break;	
			case 1:
				/*
				 * If there is no unit referenced in the command,
				 * use the recorded one.
				 */
				if(idx < 1 && handle->device != DEV_NONE)
				{
					ptrs[idx] = &STATE(handle, handle->house, handle->device);
					idx++;
				}

				/*
				 * We need to skip corresponding mask bits of
				 * processed data bytes.
				 */
				shift = __update_state(ptrs, idx, &p);
				mask >>= shift;
				DBG("shift %d\n", shift);
				break;
			default:
				ERROR("bug");
				break;
		}

	}
}

int cm11_receive_cmd(cm11_handle *handle)
{
	struct read_buf buf;
	int fd = handle->fd;
	int rc = 0;

	if((rc = x10_read(fd, &buf)))
		goto out;
	__parse_update(handle, DEV_NONE, &buf);

out:
	return rc;
}

int cm11_update_status(cm11_handle *handle, int dev)
{
	struct read_buf buf;
	int fd = handle->fd;
	int house = handle->house;
//	u32 mask;
//	u32 *ptrs[16] = {NULL};
//	int idx = 0;
//	u8 *p;

	x10_send(fd, house, dev, FUN_STAT_REQ, 0);
	bzero(&buf, sizeof(struct read_buf));
	x10_read(fd, &buf);

	handle->device = dev;
	__parse_update(handle, dev, &buf);

	return 0;
}

int cm11_get_status(cm11_handle *handle, int dev, u32 *status)
{
	if(BIDIR(STATE(handle, handle->house, dev)))
	{
		if(cm11_update_status(handle, dev))
		{
			printf("Status update failed\n");	
			return -1;
		}
	}

	*status = STATE(handle, handle->house, dev);

	return 0;
}

int cm11_ifce_status(cm11_handle *handle, cm11_status *status)
{
	int fd = handle->fd;
	u8 cmd;
	int rc;

	cmd = CM11_STATUS_REQ;
	write(fd, &cmd, 1);

	rc = read(fd, status, sizeof(cm11_status));
	if(rc < 0)
	{
		ERROR("Something wrong\n");
		rc = -1;
		goto out;
	}

	DBG("house: 0x%x\n", status->house);
	DBG("Firmware version: %d\n", status->revision);
	DBG("Monitored device: 0x%x\n", status->cur_device);
	DBG("device state: 0x%x\n", status->cur_on);
	rc = 0;
out:
	return rc;
}

int cm11_set_device_dir(struct cm11_handle *handle, int dev, int two_way)
{
	int rc = 0;
	u32 *status;

	if(CHECK_ID(dev))
	{
		ERROR("Invalid device id");
		rc = -1;
		goto out;
	}
	two_way = two_way ? 1 : 0;

	status = &STATE(handle, handle->house, dev);
	__update_state2(&status, 1, DIR_MASK, DIR_VALUE(two_way));
out:
	return rc;
}
