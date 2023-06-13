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

#include "gui.h"
#include "microdrive.h"

#include "ssd1306.h"
#include "oled_display.h"

/*
FIXME:

Instead of the hardcoded list of MDR images to load, have an optional
file of mdr filenames. If the file exists, load each file named in it
into a MD at startup.

When selection cursor is on an empty microdrive, clicking needs to
bring up a file selector.

Need some sort of error handling. Invalid file, etc.

Indication of SD card not inserted. Won't be able to save data if
SD card is ejected.

*/

void draw_status_screen( status_screen_t *status )
{
  for( uint8_t microdrive_index=0; microdrive_index< NUM_MICRODRIVES; microdrive_index++ )
  {
    oled_draw_status_microdrive( microdrive_index, status->md_inserted[microdrive_index], (microdrive_index == status->selected) );
  }

  oled_display_clear_inserted_details_area();

  if( status->inserting )
  {
    oled_display_inserted_filename( "<Inserting...>" );
  }
  else
  {
    oled_display_inserted_filename( status->filename == NULL ? (uint8_t*)"<No cartridge>" : status->filename );
    oled_display_inserted_num_blocks( status->num_blocks );
    oled_display_inserted_write_protected( (status->filename == NULL) ? -1 : status->write_protected );
  }

  oled_display_clear_msg();
  if( status->requesting_data_from_microdrive != -1 )
  {
    oled_display_msg_saving_mdr_data( status->requesting_data_from_microdrive );
  }
  
  if( status->requesting_status )
  {
    oled_display_msg_requesting_status();
  }

  // Sometimes useful for printing a debug value
  oled_display_test_value( status->test_value );

  oled_update();
}


void draw_eject_screen( status_screen_t *status )
{
  for( uint8_t microdrive_index=0; microdrive_index< NUM_MICRODRIVES; microdrive_index++ )
  {
    oled_draw_status_microdrive( microdrive_index, status->md_inserted[microdrive_index], (microdrive_index == status->selected) );
  }

  oled_display_clear_inserted_details_area();

  oled_display_eject_option();

  oled_update();  
}


void draw_insert_screen( uint8_t **filename_ptr )
{
  oled_clear();

  uint8_t line = 0;
  for( uint8_t i = 0; i < 7; i++ )
  {
    if( *filename_ptr != NULL )
    {
      oled_display_selectable_filename( *filename_ptr, line );
      line += 9;
    }
    *filename_ptr++;
  }

  oled_update();  
}


void clear_insert_screen( void )
{
  oled_clear();
  oled_update();  
}
