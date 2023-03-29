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

/*
 * Tape code. This isn't an instantiable object, there's only one of it.
 * It's sort of a singleton, only it doesn't need to be that formal.
 * I've pulled it out into it's own file and interface for code readability.
 */

#include <string.h>

#include "tape.h"
#include "libspectrum.h"
#include "if1.h"
#include "flash_images.h"

/*
 * This contains the tape image. Nothing more, not the cartridge,
 * not the write protect flag, just the data on the cartridge's
 * tape.
 *
 * There's only one of these. Pico doesn't have enough RAM to
 * store 8, so this one is filled with active data as required.
 */
static uint8_t tape_loaded = 0;
static uint8_t tape[LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH];


/* Initialisation. Don't stress on this, it'll be populated from SD card at some point */
static tape_image tape_images[2] = 
{
  { md1_image, md1_image_len },
  { md2_image, md2_image_len },
//  { md3_image, md3_image_len },
//  { md4_image, md4_image_len },
//  { md5_image, md5_image_len },
//  { md6_image, md6_image_len },
//  { md7_image, md7_image_len },
//  { md8_image, md8_image_len },
};


uint8_t load_tape( uint8_t microdrive_index )
{
  if( (microdrive_index < 0) || (microdrive_index > NUM_MICRODRIVES-1) )
    return -1;

  memcpy( tape, tape_images[microdrive_index].address, tape_images[microdrive_index].length );

  tape_loaded = 1;
  return 0;
}


uint8_t unload_tape( void )
{
  if( tape_loaded == 0 )
    return 0;

  memset( tape, 0, LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH );

  tape_loaded = 0;
  return 0;
}


uint8_t query_tape_byte( uint32_t head_pos, uint8_t *value )
{
  if( tape_loaded == 0 )
    return -1;

  if( head_pos >= LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH )
    return -1;

  *value = tape[head_pos];

  return 0;
}


uint8_t write_tape_byte( uint32_t head_pos, uint8_t value )
{
  if( tape_loaded == 0 )
    return -1;

  if( head_pos >= LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH )
    return -1;

  tape[head_pos] = value;

  return 0;
}
