/*
 * ZX Pico IF1 Firmware, a Raspberry Pi Pico based ZX Interface One emulator
 * Copyright (C) 2023 Derek Fountain
 * Derived from the Fuse code:
 *    Copyright (c) 2004-2016 Gergely Szasz, Philip Kendall
 *    Copyright (c) 2015 Stuart Brady
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

#ifndef __SD_CARD_H
#define __SD_CARD_H

#include <stdint.h>

uint8_t query_sd_card_mounted( void );
uint8_t mount_sd_card( void );
uint8_t unmount_sd_card( void );
uint8_t read_mdr_file( uint8_t *filename, uint8_t *buffer, uint32_t max_length, uint32_t *bytes_read_ptr );
uint8_t write_mdr_file( uint8_t *filename, uint8_t *buffer, uint32_t length, uint32_t *bytes_written_ptr );
uint32_t read_directory_files( uint8_t **addr_ptr, uint32_t max_num_filenames );

uint8_t *open_config_file( void );
uint8_t *next_config_entry( void );

#endif
