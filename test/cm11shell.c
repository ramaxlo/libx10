/*
 * cm11shell.c
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
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <x10/x10proto.h>
#include <x10/cm11.h>

#define HELP_STR \
	"cm11shell: The interactive program for sending X-10 commands\n\n" \
	"Usage: cm11shell -d <serial device>\n" 

//typedef void (*sighandler_t)(int);
//sighandler_t old_handler;
cm11_handle *handle;
int stop = 0;

/*
void sig_handler(int num)
{
	stop = 1;
	printf("Exit...\n");
}
*/

int get_house(char house_code)
{
	switch(house_code)
	{
		case 'A':
			return HOUSE_A;
		case 'B':
			return HOUSE_B;
		case 'C':
			return HOUSE_C;
		case 'D':
			return HOUSE_D;
		case 'E':
			return HOUSE_E;
		case 'F':
			return HOUSE_F;
		case 'G':
			return HOUSE_G;
		case 'H':
			return HOUSE_H;
		case 'I':
			return HOUSE_I;
		case 'J':
			return HOUSE_J;
		case 'K':
			return HOUSE_K;
		case 'L':
			return HOUSE_L;
		case 'M':
			return HOUSE_M;
		case 'N':
			return HOUSE_N;
		case 'O':
			return HOUSE_O;
		case 'P':
			return HOUSE_P;
		default:
			return -1;
	}
}

void usage()
{
	printf(HELP_STR);
}

int parse_addr(char *addr, int *house, int *dev)
{
	int found = 0;

	if (addr[0] >= 'a' && addr[0] <= 'p')
		found = 1;
	else if (addr[0] >= 'A' && addr[0] <= 'P')
		found = 1;

	if (!found)
		return -1;

	*house = get_house(toupper(addr[0]));
	*dev = atoi(addr + 1);

	return 0;
}

int parse_value(char *value_str, int *value)
{
	*value = atoi(value_str);

	return 0;
}

int send_cmd(int house, int dev, char *func, char *value)
{
	int rc;
	int ivalue;

	cm11_set_house(handle, house);
	
	if (!strncmp(func, "on", 2))
		rc = cm11_device_on(handle, dev);
	else if (!strncmp(func, "off", 3))
		rc = cm11_device_off(handle, dev);
	else if (!strncmp(func, "bright", 6))
	{
		rc = parse_value(value, &ivalue);
		if (rc)
			goto out;
		rc = cm11_device_brightness(handle, dev, ivalue);
	}
	else if (!strncpy(func, "type", 6))
	{
		int type;

		if (!strncmp(value, "app", 3))
			type = TYPE_APPLIANCE;
		else if (!strncmp(value, "lamp", 4))
			type = TYPE_LAMP;
		else
		{
			rc = -1;
			goto out;
		}

		rc = cm11_set_device_type(handle, dev, type);
	}
	else
		rc = -1;

out:
	return rc;
}

void *monitor_thread(void *data)
{
	int rc = 0;

	while (!stop)
	{
		rc = cm11_receive(handle);
		if (rc < 0)
		{
			printf("Error on receiving\n");
			break;
		}
	}

	return (void *)rc;
}

int start_shell()
{
	int rc;
	char addr[8];
	char func[8];
	char value[8];

	while (1) 
	{
		printf("cm11>> ");
		fsync(0);

		rc = scanf("%s %s %s", addr, func, value);
		if (rc > 0)
		{
			int house, dev;

			if (!strncmp(addr, "quit", 4))
			{
				rc = 0;
				break;
			}

			rc = parse_addr(addr, &house, &dev);
			if (rc)
			{
				printf("Invalid address\n");
				continue;
			}

			rc = send_cmd(house, dev, func, value);
			if (rc)
			{
				printf("Invalid command\n");
				break;
			}
		}
	}

	return rc;
}

int main(int argc, char *argv[])
{
	char *dev = "/dev/ttyUSB0";
	char opt;
	int rc;
	pthread_t thread;

	while ((opt = getopt(argc, argv, "d:h")) != -1)
	{
		switch (opt)
		{
			case 'd':
				dev = optarg;
				break;
			case 'h':
				usage();
				return 0;
			default:
				usage();
				return 1;
		}
	}

	handle = cm11_init(dev);
	if(handle == NULL)
	{
		printf("Can't init CM11\n"); 
		return 1;
	}

//	old_handler = signal(SIGINT, sig_handler);

	rc = pthread_create(&thread, NULL, monitor_thread, NULL);
	if (rc)
	{
		printf("Thread creation fail\n");
		goto err_thread;
	}

	rc = start_shell();

	stop = 1;
	pthread_join(thread, NULL);

err_thread:
	cm11_close(handle);
//	signal(SIGINT, old_handler);

	return rc;
}
