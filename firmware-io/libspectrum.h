/* libspectrum.h: the library for dealing with ZX Spectrum emulator files
   Copyright (c) 2001-2018 Philip Kendall, Darren Salt, Fredrick Meunier

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along
   with this program; if not, write to the Free Software Foundation, Inc.,
   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

   Author contact information:

   E-mail: philip-fuse@shadowmagic.org.uk

*/

#ifndef LIBSPECTRUM_LIBSPECTRUM_H
#define LIBSPECTRUM_LIBSPECTRUM_H

#include <stdlib.h>
#include <stdint.h>

typedef  uint8_t libspectrum_byte;
typedef   int8_t libspectrum_signed_byte;
typedef uint16_t libspectrum_word;
typedef  int16_t libspectrum_signed_word;
typedef uint32_t libspectrum_dword;
typedef  int32_t libspectrum_signed_dword;
typedef uint64_t libspectrum_qword;
typedef  int64_t libspectrum_signed_qword;

/* The various errors which can occur */
typedef enum libspectrum_error {

  LIBSPECTRUM_ERROR_NONE = 0,

  LIBSPECTRUM_ERROR_WARNING,

  LIBSPECTRUM_ERROR_MEMORY,
  LIBSPECTRUM_ERROR_UNKNOWN,
  LIBSPECTRUM_ERROR_CORRUPT,
  LIBSPECTRUM_ERROR_SIGNATURE,
  LIBSPECTRUM_ERROR_SLT,        /* .slt data found at end of a .z80 file */
  LIBSPECTRUM_ERROR_INVALID,    /* Invalid parameter supplied */

  LIBSPECTRUM_ERROR_LOGIC = -1,

} libspectrum_error;


/* Pico doesn't have enough RAM to hold a 254 block cartridge image. */
/* #define LIBSPECTRUM_MICRODRIVE_BLOCK_MAX 254 */

#define LIBSPECTRUM_MICRODRIVE_BLOCK_MAX 180
#define LIBSPECTRUM_MICRODRIVE_HEAD_LEN 15
#define LIBSPECTRUM_MICRODRIVE_DATA_LEN 512
#define LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ( LIBSPECTRUM_MICRODRIVE_HEAD_LEN + \
                                           LIBSPECTRUM_MICRODRIVE_HEAD_LEN +  \
                                           LIBSPECTRUM_MICRODRIVE_DATA_LEN + 1 )
#define LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH \
        ( LIBSPECTRUM_MICRODRIVE_BLOCK_MAX * LIBSPECTRUM_MICRODRIVE_BLOCK_LEN )

/* Constructor/destructor */



#endif                          /* #ifndef LIBSPECTRUM_LIBSPECTRUM_H */
