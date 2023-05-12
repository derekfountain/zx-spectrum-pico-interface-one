/*
 * ZX Pico IF1 Firmware, a Raspberry Pi Pico based ZX Interface One emulator
 * Copyright (C) 2023 Derek Fountain
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __WORK_QUEUE_H
#define __WORK_QUEUE_H

#include "stdint.h"

typedef enum
{
  WORK_NULL            = 0,
  WORK_INIT_IO_LINK,
  WORK_INSERT_MDR,
  WORK_REQUEST_STATUS,
} work_queue_type_t;


typedef struct _work_init_io_link_t
{
  uint8_t  dummy;
}
work_init_io_link_t;

typedef struct _work_insert_mdr_t
{
  uint8_t  microdrive_index;
  uint8_t *filename;
}
work_insert_mdr_t;

typedef struct _work_request_status_t
{
  uint8_t  dummy;
}
work_request_status_t;

void work_queue_init( void );
void insert_work( work_queue_type_t type, void *data );
bool remove_work( work_queue_type_t *type, void **data  );
bool work_queue_is_empty( void );

#endif
