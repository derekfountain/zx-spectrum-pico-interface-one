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

void draw_status_screen( status_screen_t *status )
{
  oled_draw_status_menu( 0 );

  for( uint8_t microdrive_index=0; microdrive_index< NUM_MICRODRIVES; microdrive_index++ )
  {
    oled_draw_status_microdrive( microdrive_index, status->md_inserted[microdrive_index], (microdrive_index == status->selected) );
  }
  
  oled_update();
}




#if 0
#include "gui.fsm.c"
#endif
