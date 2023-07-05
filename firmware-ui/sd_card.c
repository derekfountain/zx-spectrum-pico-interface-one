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
 * This is a really basic interface to the the SD card. Minimal error
 * checking, no support for directory structure, etc. Massive
 * improvements required here, but it works for now.
 */

/* These are the FAT filesystem library headers */
#include "f_util.h"
#include "ff.h"
#include "rtc.h"
#include "sd_card.h"
#include "hw_config.h"

#include "work_queue.h"    /* For MAX_INSERT_FILENAME_LEN */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "sd_card.h"

/* There's only one SD card reader on the device, and this is its file object */
static FIL fil;
static sd_card_t *pSD;

static uint8_t sd_card_mounted = 0;

uint8_t query_sd_card_mounted( void )
{
  return sd_card_mounted;
}


uint8_t mount_sd_card( void )
{
  sd_card_mounted = 0;

  /* FatFs, on the SD card. Set up via hw_config.c */
  pSD = sd_get_by_num( 0 );

  /*
   * This appears buggy. The 1 says force mount the device and fail if
   * you can't, but the function always returns OK even if there's no
   * card in the slot
   */
  if( f_mount( &pSD->fatfs, pSD->pcName, 1 ) != FR_OK )
    return 1;

  sd_card_mounted = 1;
  return 0;
}


uint8_t unmount_sd_card( void )
{
  if( sd_card_mounted )
    f_unmount(pSD->pcName);

  sd_card_mounted = 0;
  return 0;
}


/*
 * The config file is simply a list of filenames, one per line.
 * Each line is expected to be the name of a file on the SD
 * card. They will be loaded into the microdrives, starting at
 * drive 0, in the order they're given. That's it, nothing
 * clever, no comments, nothing else.
 *
 * This routine opens the file and returns the first filename
 * from it, or NULL.
 */
static uint8_t config_mdr_filename[MAX_INSERT_FILENAME_LEN+1];
static FIL     config_file_handle;
static bool    config_file_open;
uint8_t *open_config_file( void )
{
  config_file_open = false;

  FRESULT fr = f_open( &config_file_handle, "zxes_config.txt", FA_READ );
  if( fr )
    return NULL;
  
  if( f_gets( config_mdr_filename, MAX_INSERT_FILENAME_LEN, &config_file_handle ) == NULL )
  {
    f_close( &config_file_handle );
    return NULL;
  }

  if( config_mdr_filename[ strlen(config_mdr_filename)-1 ] == '\n' )
    config_mdr_filename[ strlen(config_mdr_filename)-1 ] = 0;

  config_file_open = true;

  return config_mdr_filename;
}


/*
 * Read another line from the config file. Returns NULL when the
 * file is exhausted.
 */
uint8_t *next_config_entry( void )
{
  if( config_file_open )
  {
    if( f_gets( config_mdr_filename, MAX_INSERT_FILENAME_LEN, &config_file_handle ) == NULL )
    {
      f_close( &config_file_handle );
      config_file_open = false;
      return NULL;
    }
    else
    {
      return config_mdr_filename;
    }
  }
  else
  {
    return NULL;
  }
}


/*
 * Read the filenames from the SD card's root directory into the array given.
 * Each new entry is just the filename, and the memory for the string is
 * malloced. The memory is freed when the file for insertion is selected or
 * the insert operation is cancelled.
 *
 * Returns the number of filenames read into the array.
 */
uint32_t read_directory_files( uint8_t *addr_ptr[], uint32_t max_num_filenames )
{
  DIR     dir;
  FRESULT fr = f_opendir( &dir, "/" );

  uint32_t filenames_read = 0;

  if( fr == 0 )
  {
    while( filenames_read < max_num_filenames )
    {
      static FILINFO fno;

      /* Read a filename entry, break when there's no more */
      fr = f_readdir( &dir, &fno );
      if( (fr != FR_OK) || (fno.fname[0] == 0) )
	break;

      /* Ignore directories */
      if( fno.fattrib & AM_DIR )
	continue;
      
      /* Must be at least <something>.mdr */
      uint8_t filename_len = strlen( fno.fname );
      if( filename_len < 5 )
	continue;

      /* Ignore anything which doesn't end in ".mdr" */
      if( strncmp( fno.fname+filename_len-4, ".mdr", 4 ) != 0 )
	continue;

      /* OK, it's a file we can use, claim some memory for it */
      void *filename_ptr = malloc( filename_len + 1 );
      if( filename_ptr == NULL )
      {
	/* OOM, won't end well but not much I can do here */
	break;
      }
      /* Copy filename from SD card entry into new memory */
      strncpy( filename_ptr, fno.fname, filename_len+1 );

      /* Find the slot in the pointers array where this new filename goes */
      uint32_t i;
      for( i=0; i<filenames_read; i++ )
      {
	/*
	 * Make the list alphabetical.
	 * 
	 * This is a crude insertion sort. Walk down to find the slot where
	 * the new entry fits, then shuffle everything down to make room for
	 * it. Unless someone has an SD card with hundreds of entries I'd
	 * expect this to be adequate.
	 */
	if( strcmp( fno.fname, addr_ptr[i]  ) < 0 )
	{
	  /*
	   * i is the index of the slot this filename needs to go in at,
	   * shuffle the rest of the array down one
	   */

	  for( uint32_t walk_down=max_num_filenames-2; walk_down>i; walk_down-- )
	  {
	    addr_ptr[walk_down] = addr_ptr[walk_down-1];
	  }
	  break;
	}
      }

      addr_ptr[i] = filename_ptr;

      filenames_read++;
    }
  }

  return filenames_read;
}


uint8_t read_mdr_file( uint8_t *filename, uint8_t *buffer, uint32_t max_length, uint32_t *bytes_read_ptr )
{
  FIL fsrc;
  UINT bytes_read = 0;

  if( sd_card_mounted )
  {
    FRESULT fr = f_open( &fsrc, filename, FA_READ );
    if( fr )
      return (uint8_t)fr;

    *bytes_read_ptr = 0;
    f_read( &fsrc, buffer, max_length, &bytes_read );

    f_close(&fsrc);
  }
  *bytes_read_ptr = bytes_read;

  return 0;
}


uint8_t write_mdr_file( uint8_t *filename, uint8_t *buffer, uint32_t length, uint32_t *bytes_written_ptr )
{
  FIL fsrc;
  UINT bytes_written = 0;

  if( sd_card_mounted )
  {
    FRESULT fr = f_open( &fsrc, filename, FA_WRITE | FA_OPEN_ALWAYS );
    if( fr )
      return (uint8_t)fr;

    *bytes_written_ptr = 0;
    f_write( &fsrc, buffer, length, &bytes_written );

    f_close(&fsrc);
  }
  *bytes_written_ptr = bytes_written;

  return 0;
}

