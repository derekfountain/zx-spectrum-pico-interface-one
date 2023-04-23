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

#include "libspectrum.h"

/* Probably won't need this in the long term */
typedef struct utils_file {

  unsigned char *buffer;
  size_t length;

} utils_file;


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
  uint8_t    data;
}
TRACE_TYPE;

#define NUM_TRACE_ENTRIES   30000

void trace( TRACE_CODE code, uint8_t data );
void trace_on( uint8_t data );
void trace_off( uint8_t data );

/* This is used in the preamble */
enum
{
  SYNC_NO = 0,
  SYNC_OK = 0xff
};

/*
 * Microdrive structure, represents the drive ifself.
 */
typedef struct _microdrive_t
{
  utils_file file;
  char *filename;		/* old filename */
  int inserted;
  int modified;  // This isn't used, but might be needed for the save code
  int motor_on;
  int head_pos;
  int transfered;
  int max_bytes;
  libspectrum_byte pream[512];	/* preamble/sync area written. 256 header blocks and 256 data blocks */
  libspectrum_byte last;
  libspectrum_byte gap;
  libspectrum_byte sync;

  /*
   * Offset in PSRAM to the byte array representing the tape, approx 97KB for a 180 sector
   * tape. About 135KB for a 254 sector tape.
   *
   * The following is obsolete because the external RAM can't be examined
   * directly using the debugger. Might be useful though:
   *
   * Copy and paste out to a text file with:
   *
   * (gdb) set print repeats 0
   * (gdb) set print elements unlimited
   * (gdb) set pagination off
   * (gdb) p/x cartridge_data
   * (gdb) set max-value-size unlimited
   *
   * Convert to MDR image with:
   *
   * > perl -ne 'map { print chr(hex($_)) } split(/, /, $_)' < mm_reformated_in_zx.txt > mm_reformated_in_zx.mdr
   *
   * mm_reformated_in_zx.mdr will then load into FUSE as a normal
   * MDR file.
   *
   */
  uint32_t         cartridge_data_psram_offset;

  uint8_t          cartridge_data_modified;
  libspectrum_byte cartridge_write_protect;
  libspectrum_byte cartridge_len_in_blocks;

} microdrive_t;


/* Index into microdrives array, etc. 0 to 7, maybe -1 */
typedef int32_t microdrive_index_t;

/* A byte on tape, the cartridge has an array of these */
typedef uint8_t tape_byte_t;

/* Index of MDR image into flash array */
typedef int32_t flash_mdr_image_index_t;


int32_t if1_init( void );
int32_t if1_mdr_insert( const microdrive_index_t which );

libspectrum_byte port_ctr_in( void );
void port_ctr_out( libspectrum_byte val );

libspectrum_byte port_mdr_in( void );
void port_mdr_out( libspectrum_byte val );


#endif
