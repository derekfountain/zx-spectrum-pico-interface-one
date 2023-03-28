/* microdrive.c: Routines for handling microdrive images
   Copyright (c) 2004-2015 Philip Kendall

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

#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "internals.h"

#include "if1.h"

static const size_t MDR_LENGTH = LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH + 1;

/* Constructor/destructor */

/* Allocate a microdrive image */
libspectrum_microdrive*
libspectrum_microdrive_alloc( void )
{
  return (libspectrum_microdrive*)malloc( sizeof(libspectrum_microdrive) );
#if 0
  return libspectrum_new( libspectrum_microdrive, 1 );
#endif
}

/* Free a microdrive image */
libspectrum_error
libspectrum_microdrive_free( libspectrum_microdrive *microdrive )
{
#if 0
  libspectrum_free( microdrive );
#endif

  return LIBSPECTRUM_ERROR_NONE;
}

/* Accessors */

inline
libspectrum_byte
libspectrum_microdrive_data( const libspectrum_microdrive *microdrive,
			     size_t which )
{
  return microdrive->data[ which ];
}

inline
void
libspectrum_microdrive_set_data( libspectrum_microdrive *microdrive,
				 size_t which, libspectrum_byte data )
{
  microdrive->data[ which ] = data;
}

inline
int
libspectrum_microdrive_write_protect( const libspectrum_microdrive *microdrive )
{
  return microdrive->write_protect;
}

inline
void
libspectrum_microdrive_set_write_protect( libspectrum_microdrive *microdrive,
					  int write_protect )
{
  microdrive->write_protect = write_protect;
}

inline
libspectrum_byte
libspectrum_microdrive_cartridge_len( const libspectrum_microdrive *microdrive )
{
  return microdrive->cartridge_len;
}

inline
void
libspectrum_microdrive_set_cartridge_len( libspectrum_microdrive *microdrive,
			     libspectrum_byte len )
{
  microdrive->cartridge_len = len;
}



/*
 * Read a buffer of data in MDR format into the microdrive cartridge
 * data structure. This is called from the "insert" code. In the FUSE
 * code the MDR data is read from the disk file and this is used to 
 * load that data into the cartridge image buffer.
 *
 * When I've got data on SD card, this is going to need to work the
 * same way as it does in FUSE.
 */
libspectrum_error
libspectrum_microdrive_mdr_read( libspectrum_microdrive *microdrive,
				 const libspectrum_byte *buffer, size_t length )
{
  size_t data_length;

  if( length < LIBSPECTRUM_MICRODRIVE_BLOCK_LEN * 10 ||
     ( length % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) > 1 ||
       length > MDR_LENGTH ) {

#if 0
    libspectrum_print_error(
      LIBSPECTRUM_ERROR_CORRUPT,
      "libspectrum_microdrive_mdr_read: not enough data in buffer"
    );
#endif

    return LIBSPECTRUM_ERROR_CORRUPT;
  }

  data_length = length - ( length % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN );

  memcpy( microdrive->data, buffer, data_length ); buffer += data_length;

  if( ( length % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) == 1 )
    libspectrum_microdrive_set_write_protect( microdrive, *buffer );
  else
    libspectrum_microdrive_set_write_protect( microdrive, 0 );

  libspectrum_microdrive_set_cartridge_len( microdrive,
			      data_length / LIBSPECTRUM_MICRODRIVE_BLOCK_LEN );
 
  return LIBSPECTRUM_ERROR_NONE;
}


/*
 * Write a cartridge image into a byte buffer.
 *
 * In the FUSE code the byte buffer is written out to a file on disk.
 * I'm going to need this when the SD card bit is done.
 */
void
libspectrum_microdrive_mdr_write( const libspectrum_microdrive *microdrive,
				  libspectrum_byte **buffer, size_t *length )
{
  *length = microdrive->cartridge_len * LIBSPECTRUM_MICRODRIVE_BLOCK_LEN;
  *buffer = libspectrum_new( libspectrum_byte, *length + 1 );

  memcpy( *buffer, microdrive->data, *length );

  (*buffer)[ *length ] = microdrive->write_protect;

  (*length)++;
}
