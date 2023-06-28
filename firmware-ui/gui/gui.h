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

#ifndef __GUI_H
#define __GUI_H

#include "pico/stdlib.h"
#include "microdrive.h"

typedef struct _status_screen_t
{
  bool               requesting_status;
  microdrive_index_t requesting_data_from_microdrive;

  bool               md_inserted[8];
  int8_t             selected;

  bool               inserting;
  uint8_t           *filename;
  uint8_t            num_blocks;
  bool               write_protected;

  uint8_t           *cartridge_error_str;

  uint8_t            test_value;
}
status_screen_t;

void draw_status_screen( status_screen_t *status );
void draw_insert_screen( uint32_t index, uint8_t *filename_ptr[] );
void draw_eject_screen( status_screen_t *status );
void clear_insert_screen( void );

#endif
