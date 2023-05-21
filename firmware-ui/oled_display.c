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


void oled_draw_status_menu( uint8_t selected )
{
#if 0
/*
 * Not sure about this, the screen is too small to have a menu
 * Particularly one which happens to need long words!
 */
  /* Plan is to underline one of these and have the screen toggle when it's selected */
  uint8_t *entries[2] = { "Microdrives", "Cartridges" };

  const uint8_t menu_y_pos = 0;

  ssd1306_draw_string(&display, 0,      menu_y_pos, 1, entries[0], 0);
  ssd1306_draw_string(&display, 11*6+3, menu_y_pos, 1, entries[1], 0);

  // Test
  uint8_t buffer[20];
  snprintf(buffer,20,"%d",selected);
  ssd1306_draw_string(&display, 0, 50, 1, buffer, 0);
#endif  
}


/*
 * This draws a box with a number in it, that being the current
 * depiction of a microdrive. There's 8 of them across the 
 * screen. Inserted means there's a cartridge in the drive, and
 * it's drawn as an inverted box. Selected means the UI selection
 * device is active on it, currently drawn as an underline.
 */
void oled_draw_status_microdrive( microdrive_index_t microdrive_index, bool inserted, bool selected )
{
  const uint8_t md_y_pos = 0;

  /* Draw the square */
  uint8_t x_offset = 125 - ((microdrive_index+1)*15);
  if( inserted )
    ssd1306_draw_filled_square(&display, x_offset, md_y_pos, 13, 11 );
  else
    ssd1306_draw_empty_square(&display, x_offset, md_y_pos, 13, 11 );

  /* Knock out the top corners - ooh, arty! */
  ssd1306_clear_pixel(&display, x_offset, md_y_pos);
  ssd1306_clear_pixel(&display, x_offset+12, md_y_pos);
  
  /* Number */
  uint8_t num_str[2];
  snprintf( num_str, 2, "%01d", microdrive_index+1 );
  ssd1306_draw_string(&display, x_offset+4, md_y_pos+2, 1, num_str, inserted);
  
  /* Add or clear selection device */
  if( selected )
  {
    /* Draw normally */
    ssd1306_draw_line(&display, x_offset, md_y_pos+13, x_offset+12, md_y_pos+13, 0);
  }
  else
  {
    /* Draw with "clear" to remove any previous line */
    ssd1306_draw_line(&display, x_offset, md_y_pos+13, x_offset+12, md_y_pos+13, 1);
  }
  return;
}


void oled_display_clear_inserted_details_area( void )
{
  ssd1306_clear_square( &display, 0, 16, 127, 32+8 );  
}


void oled_display_inserted_filename( uint8_t *filename )
{
  ssd1306_clear_square( &display, 0, 16, 127, 16+8 );  

  if( filename != NULL )
  {
    ssd1306_draw_string( &display, 0, 16, 1, filename, 0 );
  }
}


void oled_display_inserted_num_blocks( uint8_t num_blocks )
{
  ssd1306_clear_square( &display, 0, 24, 127, 24+8 );  

  if( num_blocks != 0 )
  {
    uint8_t num_blocks_str[32];
    snprintf( num_blocks_str, 32, "%d blocks", num_blocks );
    ssd1306_draw_string( &display, 0, 24, 1, num_blocks_str, 0 );
  }
}


void oled_display_inserted_write_protected( int8_t write_protected )
{
  ssd1306_clear_square( &display, 0, 32, 127, 32+8 );  

  if( write_protected != -1 )
  {
    ssd1306_draw_string( &display, 0, 32, 1, write_protected ? "Write protected" : "Not write protected", 0 );
  }
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
