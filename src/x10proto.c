/*
 * x10proto.c
 * Functions dealing with X10 protocol
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
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include "x10proto.h"
#include "debug.h"

int wait_for_ack(int fd, unsigned char value)
{
	unsigned char ack;

	read(fd, &ack, 1);
	if(ack != value)
	{
		ERROR("Need: 0x%x Read: 0x%x\n", value, ack);
		return -1;
	}
	return 0;
}

int x10_detect_poll(int fd, u8 *code)
{
	fd_set rset;
	struct timeval tv;
	int rc;

	FD_ZERO(&rset);
	FD_SET(fd, &rset);

	/* 1.5 secs may be enough for detecting the poll cmd */
	tv.tv_sec = 1;
	tv.tv_usec = 5 * 100000;

	rc = select(fd + 1, &rset, NULL, NULL, &tv);
	if(rc > 0)
		read(fd, code, 1);

	return rc;
}

int x10_read(int fd, struct read_buf *buf)
{
	unsigned char tmp;
	unsigned char *p;
	int i;
	int trash = 0;

	bzero(buf, sizeof(struct read_buf));
	DBG("Now reading cmd...\n");
	if(x10_detect_poll(fd, &tmp) <= 0)
	{
		ERROR("Poll timedout");
		return -1;
	}
	if(tmp != SIG_POLL)
	{
		ERROR("Not poll code: 0x%x", tmp);
		return -1;
	}

	tmp = SIG_ACK;
	write(fd, &tmp, 1);

	read(fd, &buf->sz, 1);
	if(buf->sz > 9)
	{
		trash = buf->sz - 9;
		buf->sz = 9;
	}
	if(buf->sz != 0)
		read(fd, &buf->mask, buf->sz);

	/* Drop remaining bytes */
	while(trash)
	{
		read(fd, (char *)&i, 1);
		trash--;
	}

	DBG("data read:\n");
	DBG("\tsz: 0x%02x\n", buf->sz);
	DBG("\tmask: 0x%02x\n", buf->mask);
	p = &buf->data0;
	for(i = 0; i < buf->sz - 1; i++)
	{
		DBG("\tdata%d: 0x%02x\n", i, *p);
		p++;
	}
	return 0;
}

int x10_send(int fd, unsigned char house, unsigned char dev, 
unsigned char fun, unsigned char value)
{
	unsigned char cmd[2];	
	unsigned char ack;
	unsigned char checksum;

	cmd[0] = MODE_ADDR;
	cmd[1] = ((codes[house] << 4) | codes[dev]);
	checksum = (cmd[0] + cmd[1]) & 0xff;

	DBG("cmd: 0x%02x, 0x%02x\n", cmd[0], cmd[1]);
	write(fd, cmd, 2);

	if(wait_for_ack(fd, checksum))
	{
		ERROR("Checksum error");	
		return -1;
	}
	
	ack = 0;
	write(fd, &ack, 1);

	if(wait_for_ack(fd, 0x55))
	{
		ERROR("ACK error");	
		return -1;
	}

	cmd[0] = MODE_FUNC;
	if(fun == FUN_DIM || fun == FUN_BRIGHT)
		cmd[0] |= (value << 3);
	cmd[1] = ((codes[house] << 4) | fun);
	checksum = (cmd[0] + cmd[1]) & 0xff;

	DBG("cmd: 0x%02x, 0x%02x\n", cmd[0], cmd[1]);
	write(fd, cmd, 2);

	if(wait_for_ack(fd, checksum))
	{
		ERROR("Checksum error");	
		return -1;
	}

	ack = 0;
	write(fd, &ack, 1);

	if(wait_for_ack(fd, 0x55))
	{
		ERROR("ACK error");	
		return -1;
	}

	return 0;
}