/* if1.h: Interface 1 handling routines
   Copyright (c) 2004-2016 Gergely Szasz, Philip Kendall
   Copyright (c) 2015 Stuart Brady
   Copyright (c) 2023 Derek Fountain

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

*/

#ifndef IF1_H
#define IF1_H

#include "libspectrum.h"


/* Probably won't need this in the long term */
typedef struct utils_file {

  unsigned char *buffer;
  size_t length;

} utils_file;


/* This is used in the preamble */
enum {
  SYNC_NO = 0,
  SYNC_OK = 0xff
};


/*
 * Microdrive structure, represents the tape drive ifself.
 */
typedef struct microdrive_t {
  utils_file file;
  char *filename;		/* old filename */
  int inserted;
  int modified;
  int motor_on;
  int head_pos;
  int transfered;
  int max_bytes;
  libspectrum_byte pream[512];	/* preamble/sync area written */
  libspectrum_byte last;
  libspectrum_byte gap;
  libspectrum_byte sync;

  libspectrum_microdrive *cartridge;	/* write protect, len, blocks */

} microdrive_t;


/*
 * IF1 ULA structure. I removed pretty much everything in this.
 */
typedef struct if1_ula_t {
  int comms_clk;	/* the previous data comms state */
} if1_ula_t;


int if1_init( void *context );

libspectrum_byte port_ctr_in( void );
void port_ctr_out( libspectrum_byte val );

libspectrum_byte port_mdr_in( void );
void port_mdr_out( libspectrum_byte val );

int if1_mdr_insert( int drive, const char *filename );
int if1_mdr_write( int drive, const char *filename );
int if1_mdr_eject( int drive );
int if1_mdr_save( int drive, int saveas );
void if1_mdr_writeprotect( int drive, int wrprot );

#endif
