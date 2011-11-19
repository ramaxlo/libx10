/*
 * x10test.c
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
	u32 status;

	cm11_get_status(handle, DEV_2, &status);
	if(ON(status))
		printf("Device 2 on\n");
	else
		printf("Device 2 off\n");
	cm11_device_off(handle, DEV_2);
	cm11_close(handle);
	signal(SIGINT, old_handler);
	exit(0);
}

int main()
{
	u32 status;
	//cm11_status stat;

	handle = cm11_init("/dev/ttySAC1");
	if(handle == NULL)
	{
		printf("Can't init CM11\n"); 
		return 1;
	}

	old_handler = signal(SIGINT, sig_handler);

	cm11_set_device_type(handle, DEV_1, TYPE_APPLIANCE);
	cm11_set_device_dir(handle, DEV_1, 1);
	cm11_get_status(handle, DEV_1, &status);
	if(ON(status))
	{
		printf("Device 1 on\n");
		cm11_device_off(handle, DEV_1);
	}
	else
	{
		printf("Device 1 off\n");
		cm11_device_on(handle, DEV_1);
	}

	cm11_set_device_type(handle, DEV_2, TYPE_LAMP);
	cm11_set_device_dir(handle, DEV_2, 0);

	while(1)
		cm11_receive(handle);
//	cm11_device_on(handle, DEV_2);
//	sleep(10);
//	cm11_device_off(handle, DEV_2);
//	cm11_ifce_status(handle, &stat);

//	while(1)
//	{
//		cm11_device_brightness(handle, DEV_2, 10);
//		sleep(10);
//		cm11_device_brightness(handle, DEV_2, 22);
//		sleep(10);
//	}

	cm11_close(handle);
	signal(SIGINT, old_handler);
	return 0;
}
