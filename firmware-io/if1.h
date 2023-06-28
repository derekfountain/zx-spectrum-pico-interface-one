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

#ifndef __IF1_H
#define __IF1_H

#include "cartridge.h"
#include "microdrive.h"

/*
 * Very basic trace table, at least allows me to see the bringup sequence has completed
 *
 * (gdb) set print repeats 0
 * (gdb) set print elements unlimited
 * (gdb) set pagination off
 * (gdb) set max-value-size unlimited
 * (gdb) p trace_table
 */
typedef enum
{
  TRC_NONE = 0,

  TRC_TRC_ON,
  TRC_TRC_OFF,

  TRC_SPI_INIT,
  TRC_LINK_INIT,
  TRC_DATA_CONV,
  TRC_GPIOS_INIT,
  TRC_PIOS_INIT,
  TRC_CORE1_INIT,
  TRC_INTS_OFF,
  TRC_IF1_INIT,
  TRC_IMAGE_INIT,
  TRC_IMAGES_INIT,
  TRC_LOAD_IMAGE,
  TRC_UNLOAD_IMAGE,

  TRC_RCV_CMD,

  TRC_RCV_INSERT_MDR_STRUCT,

  TRC_READ_EF_STATUS,
  TRC_READ_E7_DATA,
  TRC_WRITE_EF_CONTROL,
  TRC_WRITE_E7_DATA,

  TRC_PORT_CTR_OUT,
  TRC_FALLING_EDGE,
  TRC_MOTORS_ON,

}
TRACE_CODE;

typedef struct _trace_type
{
  uint32_t   i;
  TRACE_CODE code;
  uint32_t   data;
}
TRACE_TYPE;

#define NUM_TRACE_ENTRIES   1000

void trace( TRACE_CODE code, uint32_t data );

/* Offset into external RAM device to find something */
typedef uint32_t psram_offset_t;

void if1_init( void );
cartridge_error_t if1_mdr_insert( const microdrive_index_t which, const uint32_t psram_offset, const uint32_t length_in_bytes,
				  const write_protect_t write_protected );

uint8_t port_ctr_in( void );
void port_ctr_out( uint8_t val );

uint8_t port_mdr_in( void );
void port_mdr_out( uint8_t val );

bool is_cartridge_inserted( microdrive_index_t which );
void if1_mdr_eject( const microdrive_index_t which );
bool is_cartridge_modified( microdrive_index_t which );
void set_cartridge_modified( microdrive_index_t which, bool modified );
bool is_cartridge_ejected_pending_save( microdrive_index_t which );
void set_cartridge_ejected_to_sd( microdrive_index_t which );
bool is_mdr_motor_on( microdrive_index_t which );

#endif
