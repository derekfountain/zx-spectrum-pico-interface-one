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

Make the FSM queue up stimulus, if currently drops them

Instead of the hardcoded list of MDR images to load, have an optional
file of mdr filenames. If the file exists, load each file named in it
into a MD at startup.

GUI needs indication of MDR saving back to SD card. Cue not to eject
the SD card.

When selection cursor is on an empty microdrive, clicking needs to
bring up a file selector.

Need some sort of error handling. Invalid file, etc.

Indication of SD card not inserted. Won't be able to save data if
SD card is ejected.

*/

void draw_status_screen( status_screen_t *status )
{
  oled_draw_status_menu( status->selected );

  for( uint8_t microdrive_index=0; microdrive_index< NUM_MICRODRIVES; microdrive_index++ )
  {
    oled_draw_status_microdrive( microdrive_index, status->md_inserted[microdrive_index], (microdrive_index == status->selected) );
  }

  /* Not sure about this, it's going to flicker when the timer is up and running */
  oled_display_clear_inserted_details_area();

  if( status->inserting )
  {
    oled_display_inserted_filename( "Inserting..." );
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

  oled_update();
}




#if 0
#include "gui.fsm.c"
#endif
