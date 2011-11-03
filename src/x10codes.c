/*
 * x10codecs.c
 * Definitions of X10 transmission codes
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

#include "x10proto.h"

u8 codes[] = {
	0x06, 0x0e, 0x02, 0x0a, 
	0x01, 0x09, 0x05, 0x0d, 
	0x07, 0x0f, 0x03, 0x0b, 
	0x00, 0x08, 0x04, 0x0c,
};

u8 code2idx(u8 code)
{
	int sz = sizeof(codes);
	int i;
	int rc = 0xff;

	for(i = 0; i < sz; i++)
	{
		if(code == codes[i])
			rc = i;
	}

	return rc;
}
