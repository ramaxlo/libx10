/*
 * x10proto.h
 * The definitions used for X10 protocol
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

#ifndef  X10PROTO_H
#define  X10PROTO_H

typedef unsigned int 	u32;
typedef unsigned char 	u8;
typedef unsigned short 	u16;
typedef int 		s32;
typedef char 		s8;
typedef short 		s16;

extern u8 codes[];

enum houses
{
	HOUSE_A = 0,
	HOUSE_B,
	HOUSE_C,
	HOUSE_D,
	HOUSE_E,
	HOUSE_F,
	HOUSE_G,
	HOUSE_H,
	HOUSE_I,
	HOUSE_J,
	HOUSE_K,
	HOUSE_L,
	HOUSE_M,
	HOUSE_N,
	HOUSE_O,
	HOUSE_P,
};

enum devices
{
	DEV_NONE = -1,
	DEV_1 = 0,
	DEV_2,
	DEV_3,
	DEV_4,
	DEV_5,
	DEV_6,
	DEV_7,
	DEV_8,
	DEV_9,
	DEV_10,
	DEV_11,
	DEV_12,
	DEV_13,
	DEV_14,
	DEV_15,
	DEV_16,
};

#define CHECK_ID(x)	((x) < DEV_NONE && (x) > DEV_16)
/* 
 * Function codes
 */
#define FUN_ALL_OFF		0x00
#define FUN_ALL_LIGHT_ON	0x01
#define FUN_ON			0x02
#define FUN_OFF			0x03
#define FUN_DIM			0x04
#define FUN_BRIGHT		0x05
#define FUN_ALL_LIGHT_OFF	0x06
#define FUN_EXT			0x07
#define FUN_HAIL_REQ		0x08
#define FUN_HAIL_ACK		0x09
#define FUN_PREDIM_1		0x0a
#define FUN_PREDIM_2		0x0b
#define FUN_EXT_DATA		0x0c
#define FUN_STAT_ON		0x0d
#define FUN_STAT_OFF		0x0e
#define FUN_STAT_REQ		0x0f

#define MODE_ADDR		0x04
#define MODE_FUNC		0x06

#define SIG_POLL		0x5a
#define SIG_ACK			0xc3

struct read_buf
{
	unsigned char sz;
	unsigned char mask;
	unsigned char data0;
	unsigned char data1;
	unsigned char data2;
	unsigned char data3;
	unsigned char data4;
	unsigned char data5;
	unsigned char data6;
	unsigned char data7;
};

int wait_for_ack(int fd, unsigned char value);
int x10_read(int fd, struct read_buf *buf);
int x10_send(int fd, unsigned char house, unsigned char dev,
	unsigned char fun, unsigned char value);
int x10_detect_poll(int fd, u8 *code);

// x10codes.c
u8 code2idx(u8 code);

#endif   // X10PROTO_H

