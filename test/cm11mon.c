/*
 * cm11mon.c
 * X-10 bus monitor program, used for receiving commands sent by other
 * transmitters in the bus.
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
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <x10/x10proto.h>
#include <x10/cm11.h>

typedef void (*sighandler_t)(int);
sighandler_t old_handler;
cm11_handle *handle;

void sig_handler(int num)
{
	cm11_close(handle);
	signal(SIGINT, old_handler);
	exit(0);
}

char get_house(s32 house)
{
	switch(house)
	{
		case HOUSE_A:
			return 'A';
		case HOUSE_B:
			return 'B';
		case HOUSE_C:
			return 'C';
		case HOUSE_D:
			return 'D';
		case HOUSE_E:
			return 'E';
		case HOUSE_F:
			return 'F';
		case HOUSE_G:
			return 'G';
		case HOUSE_H:
			return 'H';
		case HOUSE_I:
			return 'I';
		case HOUSE_J:
			return 'J';
		case HOUSE_K:
			return 'K';
		case HOUSE_L:
			return 'L';
		case HOUSE_M:
			return 'M';
		case HOUSE_O:
			return 'O';
		case HOUSE_P:
			return 'P';
		default:
			return -1;
	}
}

char *parse_func(u8 func, u8 value)
{
	char *buf;

	buf = malloc(32);
	if(!buf)
	{
		printf("Memory alloc fail");
		return NULL;
	}

	switch(func)
	{
		case FUN_DIM:
			sprintf(buf, "dim %d", value);
			break;
		case FUN_BRIGHT:
			sprintf(buf, "bright %d", value);
			break;
		case FUN_ON:
			sprintf(buf, "on");
			break;
		case FUN_OFF:
			sprintf(buf, "off");
			break;
		case FUN_ALL_LIGHT_ON:
			sprintf(buf, "all light on");
			break;
		case FUN_ALL_LIGHT_OFF:
			sprintf(buf, "all light off");
			break;
		case FUN_ALL_OFF:
			sprintf(buf, "all off");
			break;
		default:
			sprintf(buf, "none");
	}

	return buf;
}

void notify(cm11_handle *handle, s32 house, s32 device, u8 func, u8 value)
{
	char house_code = get_house(house);
	char *str = parse_func(func, value);

	printf("%c%d, ", house_code, device);
	printf("%s", str);
	printf("\n");

	free(str);
}

int main()
{
	handle = cm11_init("/dev/ttyUSB0");
	if(handle == NULL)
	{
		printf("Can't init CM11\n"); 
		return 1;
	}

	old_handler = signal(SIGINT, sig_handler);

	while(1)
		cm11_receive_notify(handle, notify);

	cm11_close(handle);
	signal(SIGINT, old_handler);
	return 0;
}
