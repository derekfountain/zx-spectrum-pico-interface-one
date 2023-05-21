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

#ifndef __OLED_DISPLAY_H
#define __OLED_DISPLAY_H

#include "pico/stdlib.h"

#include "ui_io_comms.h"
#include "microdrive.h"

void oled_display_init( void );
void oled_update( void );
void oled_draw_status_menu( uint8_t selected );
void oled_display_clear_inserted_details_area( void );
void oled_draw_status_microdrive( microdrive_index_t microdrive_index, bool inserted, bool selected );

void oled_display_status_bytes( io_to_ui_status_response_t *status_struct );
void oled_display_done( void );
void oled_display_show_progress( uint8_t which, uint32_t i );
void oled_display_inserted_filename( uint8_t *filename );
void oled_display_inserted_num_blocks( uint8_t num_blocks );
void oled_display_inserted_write_protected( int8_t write_protected );
void oled_display_msg_requesting_status( void );
void oled_display_msg_saving_mdr_data( microdrive_index_t microdrive_index );
void oled_display_clear_msg( void );

#endif
