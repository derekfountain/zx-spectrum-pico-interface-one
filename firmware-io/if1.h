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


/* This is used in the preamble */
enum
{
  SYNC_NO = 0,
  SYNC_OK = 0xff
};


/*
 * This represents a microdrive cartridge. It used to store the tape
 * image, but that got taken out for Pico reasons.
 */
typedef struct _cartridge_t
{
  /* Whether this cartridge has its w/p tab removed */
  int write_protect;
  
  /* Length in 543-byte blocks */
  libspectrum_byte cartridge_len;    /* Cartridge length in blocks */
}
cartridge_t;


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

  cartridge_t *cartridge;      /* write protect, len, blocks */

} microdrive_t;



int if1_init( void );
int if1_mdr_insert( int drive, const char *filename );

libspectrum_byte port_ctr_in( void );
void port_ctr_out( libspectrum_byte val );

libspectrum_byte port_mdr_in( void );
void port_mdr_out( libspectrum_byte val );


#endif
