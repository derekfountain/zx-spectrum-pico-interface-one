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

/* These are the FAT filesystem library headers */
#include "f_util.h"
#include "ff.h"
#include "rtc.h"
#include "sd_card.h"
#include "hw_config.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "sd_card.h"

/* There's only one SD card reader on the device, and this is it's file object */
static FIL fil;
static sd_card_t *pSD;

static uint8_t sd_card_mounted = 0;

uint8_t query_ds_card_mounted( void )
{
  return sd_card_mounted;
}


uint8_t mount_sd_card( void )
{
  sd_card_mounted = 0;

  /* FatFs, on the SD card. Set up via hw_config.c */
  pSD = sd_get_by_num( 0 );
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


#define MAX_MDR_FILENAME_LENGTH 32
static uint8_t config_mdr_filename[MAX_MDR_FILENAME_LENGTH+1];
static FIL     config_file_handle;
static bool    config_file_open;
uint8_t *open_config_file( void )
{
  config_file_open = false;

  FRESULT fr = f_open( &config_file_handle, "zxes_config.txt", FA_READ );
  if( fr )
    return NULL;
  
  if( f_gets( config_mdr_filename, MAX_MDR_FILENAME_LENGTH, &config_file_handle ) == NULL )
  {
    f_close( &config_file_handle );
    return NULL;
  }

  if( config_mdr_filename[ strlen(config_mdr_filename)-1 ] == '\n' )
    config_mdr_filename[ strlen(config_mdr_filename)-1 ] = 0;

  config_file_open = true;

  return config_mdr_filename;
}


uint8_t *next_config_entry( void )
{
  if( config_file_open )
  {
    if( f_gets( config_mdr_filename, MAX_MDR_FILENAME_LENGTH, &config_file_handle ) == NULL )
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


uint32_t read_directory_files( uint8_t **addr_ptr, uint32_t max_num_filenames )
{
  DIR     dir;
  FRESULT fr = f_opendir( &dir, "/" );

  uint32_t filenames_read = 0;

  if( fr == 0 )
  {
    while( filenames_read < max_num_filenames )
    {
      static FILINFO fno;

      fr = f_readdir( &dir, &fno );
      if( (fr != FR_OK) || (fno.fname[0] == 0) )
	break;

      if( fno.fattrib & AM_DIR )
	continue;
      
      uint8_t filename_len = strlen( fno.fname );
      if( filename_len < 5 )
	continue;

      if( strncmp( fno.fname+filename_len-4, ".mdr", 4 ) != 0 )
	continue;

      *addr_ptr = malloc( filename_len + 1 );
      strncpy( *addr_ptr, fno.fname, filename_len );

      *addr_ptr++;
      
      filenames_read++;
    }
  }

  return filenames_read;
}


uint8_t read_mdr_file( uint8_t *filename, uint8_t *buffer, uint32_t max_length, uint32_t *bytes_read_ptr )
{
  FIL fsrc;
  UINT bytes_read;

  FRESULT fr = f_open( &fsrc, filename, FA_READ );
  if( fr )
    return (uint8_t)fr;

  *bytes_read_ptr = 0;
  f_read( &fsrc, buffer, max_length, &bytes_read );

  f_close(&fsrc);

  *bytes_read_ptr = bytes_read;

  return 0;
}


uint8_t write_mdr_file( uint8_t *filename, uint8_t *buffer, uint32_t length, uint32_t *bytes_written_ptr )
{
  FIL fsrc;
  UINT bytes_written;

  FRESULT fr = f_open( &fsrc, filename, FA_WRITE | FA_OPEN_ALWAYS );
  if( fr )
    return (uint8_t)fr;

  *bytes_written_ptr = 0;
  f_write( &fsrc, buffer, length, &bytes_written );

  f_close(&fsrc);

  *bytes_written_ptr = bytes_written;

  return 0;
}

