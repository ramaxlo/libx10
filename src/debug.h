/*
 * debug.h
 * Debugging utilities
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

#ifndef  DEBUG_H
#define  DEBUG_H

#ifdef DEBUG
#define DBG(x, ...)		printf("%s:%d: " x, __FUNCTION__, __LINE__, ## __VA_ARGS__)
#else
#define DBG(x, ...)
#endif
#define ERROR(x, ...)		printf("%s:%d: Error: " x "\n", __FUNCTION__, __LINE__, ## __VA_ARGS__)
#define INFO(x, ...)		printf(x, ## __VA_ARGS__)

#endif   // DEBUG_H

