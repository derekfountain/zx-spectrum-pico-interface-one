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

#include "ssd1306.h"

#include "ui_io_comms.h"     // Temp
#include <stdio.h>

ssd1306_t display;

#define OLED_I2C    (i2c0)
#define OLED_FREQ   400000
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_ADDR   0x3C
#define OLED_SCK    4
#define OLED_SDA    5

void oled_display_init( void )
{
  i2c_init(OLED_I2C, OLED_FREQ);
  gpio_set_function( OLED_SCK, GPIO_FUNC_I2C ); gpio_pull_up( OLED_SCK );
  gpio_set_function( OLED_SDA, GPIO_FUNC_I2C ); gpio_pull_up( OLED_SDA );
  display.external_vcc=false;

  ssd1306_init( &display, OLED_WIDTH, OLED_HEIGHT, OLED_ADDR, OLED_I2C );
  ssd1306_clear( &display );

  return;
}


void oled_update( void )
{
  ssd1306_show(&display);
}


/*
 * Not sure about this, the screen is too small to have a menu
 * Particularly one which happens to need long words!
 */
void oled_draw_status_menu( uint8_t selected )
{
  /* Plan is to underline one of these and have the screen toggle when it's selected */
  uint8_t *entries[2] = { "Microdrives", "Cartridges" };

  const uint8_t menu_y_pos = 0;

  ssd1306_draw_string(&display, 0,      menu_y_pos, 1, entries[0], 0);
  ssd1306_draw_string(&display, 11*6+3, menu_y_pos, 1, entries[1], 0);


  // Test
  ssd1306_draw_filled_square(&display, 0, 48, 128, 16);
  ssd1306_clear_pixel(&display, 0, 48);
  ssd1306_draw_string(&display, 0, 50, 2, "Hello", 1);
  
}

void oled_draw_status_microdrive( microdrive_index_t microdrive_index, bool inserted )
{
  const uint8_t md_y_pos = 10;

  uint8_t x_offset = 125 - ((microdrive_index+1)*15);
  if( inserted )
    ssd1306_draw_filled_square(&display, x_offset, md_y_pos, 12, 11 );
  else
    ssd1306_draw_empty_square(&display, x_offset, md_y_pos, 12, 11 );

  uint8_t num_str[2];
  snprintf( num_str, 2, "%01d", microdrive_index+1 );
  ssd1306_draw_string(&display, x_offset+4, md_y_pos+2, 1, num_str, inserted);
  
  return;
}



/* Hardcoded nonense to print some values which are currently useful */
void oled_display_status_bytes( io_to_ui_status_response_t *status_struct )
{
#if 0
  uint8_t line = 16;
  for( uint8_t i=0; i<8; i+=2 )
  {
    uint8_t value_str[32];
    snprintf( value_str, 16, "0x%02X 0x%02X", status_struct->status[i], status_struct->status[i+1] );

    ssd1306_draw_string(&display, 0, line+=8, 1, value_str);
    ssd1306_show(&display);
  }
#endif
  return;
}

void oled_display_done( void )
{
#if 0
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 0, 0, 1, "Done");
  ssd1306_show(&display);
#endif
}

void oled_display_filename( uint8_t *filename )
{
#if 0
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 10, 10, 1, filename);
  ssd1306_show(&display);
#endif
}

void oled_display_show_progress( uint8_t which, uint32_t i )
{
#if 0
  uint8_t msg[32];
  snprintf( msg, 16, "Drive %d, %d", which, i );
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 0, 0, 1, msg);
  ssd1306_show(&display);
#endif
}
