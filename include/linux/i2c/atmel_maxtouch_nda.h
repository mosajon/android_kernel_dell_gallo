/*
 *  Atmel maXTouch header file
 *
 *  Copyright (c) 2010 Atmel Corporation
 *  Copyright (c) 2011 Lon Lu <Lon_Lu@pegatroncorp.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 or 3 as
 *  published by the Free Software Foundation.
 *  See the file "COPYING" in the main directory of this archive
 *  for more details.
 *
 */

#ifndef __ATMEL_MAXTOUCH_FW_H
#define __ATMEL_MAXTOUCH_FW_H

#define FW_WAITING_BOOTLOAD_COMMAND 0xC0
#define FW_WAITING_FRAME_DATA       0x80
#define FW_FRAME_CRC_CHECK          0x02
#define FW_FRAME_CRC_PASS           0x04
#define FW_FRAME_CRC_FAIL           0x03

#define I2C_M_WRITE	0x00

extern const struct file_operations pass_fops;

extern u8 self_test_result;
extern int g_object, g_offset, g_length;
#endif

