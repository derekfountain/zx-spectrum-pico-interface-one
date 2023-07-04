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

#include "oled_display.h"
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

#if 0
  // Display tests can go here

#if 0
  // Fill, knock out corners
  ssd1306_draw_filled_square(&display, 0, 0, 128, 64);
  ssd1306_clear_pixel(&display,   0,  0);
  ssd1306_clear_pixel(&display, 127,  0);
  ssd1306_clear_pixel(&display,   0, 63);
  ssd1306_clear_pixel(&display, 127, 63);

  ssd1306_clear_square(&display, 1,1, 2,2);
#endif

#if 0
  // Check inverted text is OK
  ssd1306_draw_char(&display, 0,0, 1, '8', false);
  ssd1306_draw_char(&display, 0,8, 1, '8', true);

  ssd1306_draw_string(&display, 0,16, 1, "Hello world", true );
#endif

#if 0
  // Filled squares side by side, should see a solid block
  ssd1306_draw_filled_square(&display, 0,  0, 5, 8);
  ssd1306_draw_filled_square(&display, 5,  0, 5, 8);
  ssd1306_draw_filled_square(&display, 10, 0, 5, 8);
  ssd1306_draw_filled_square(&display, 15, 0, 5, 8);
#endif

#if 0
  // Another inverted text check
  ssd1306_draw_char(&display,  0,0, 1, 'H', false);
  ssd1306_draw_char(&display,  5,0, 1, 'e', false);
  ssd1306_draw_char(&display, 10,0, 1, 'l', false);
  ssd1306_draw_char(&display, 15,0, 1, 'l', false);
  ssd1306_draw_char(&display, 20,0, 1, 'o', false);

  ssd1306_draw_char(&display,  0,8, 1, 'H', true);
  ssd1306_draw_char(&display,  5,8, 1, 'e', true);
  ssd1306_draw_char(&display, 10,8, 1, 'l', true);
  ssd1306_draw_char(&display, 15,8, 1, 'l', true);
  ssd1306_draw_char(&display, 20,8, 1, 'o', true);

  ssd1306_draw_string(&display, 0,24, 1, "Hello", true );
#endif

  ssd1306_show(&display);
  while(1);
#endif

  return;
}

void oled_update( void )
{
  ssd1306_show(&display);
}

void oled_clear( void )
{
  ssd1306_clear( &display );
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
  ssd1306_clear_square( &display, x_offset, md_y_pos, 13, 11 );  
  if( inserted )
    ssd1306_draw_filled_square(&display, x_offset, md_y_pos, 13, 11 );
  else
    ssd1306_draw_empty_square(&display, x_offset, md_y_pos, 13, 11 );

  /* Knock out the top corners - ooh, arty! */
  ssd1306_clear_pixel(&display, x_offset, md_y_pos);
  ssd1306_clear_pixel(&display, x_offset+12, md_y_pos);
  
  /* Number, normal for not inserted, knock out the filled area for inserted */
  if( inserted )
    ssd1306_draw_inverted_char(&display, x_offset+4, md_y_pos+2, 1, '1'+microdrive_index);
  else
    ssd1306_draw_char(&display, x_offset+4, md_y_pos+2, 1, '1'+microdrive_index, false);
  
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
  ssd1306_clear_square( &display, 0, 16, 128, 32+8 );  
}


void oled_display_eject_option( void )
{
  ssd1306_draw_string( &display, 28, 24, 2, "Eject?", 0 );
  ssd1306_draw_line(&display, 28, 24+16, 28+72, 24+16, 0);
}


void oled_display_no_files( void )
{
  ssd1306_draw_string( &display, 22, 24, 2, "No files", 0 );
  ssd1306_draw_line(&display, 22, 24+16, 22+90, 24+16, 0);
}


void oled_display_inserted_filename( uint8_t *filename )
{
  ssd1306_clear_square( &display, 0, 16, 128, 8 );  

  if( filename != NULL )
  {
    ssd1306_draw_string( &display, 0, 16, 1, filename, 0 );
  }
}


void oled_display_inserted_num_blocks( uint8_t num_blocks )
{
  ssd1306_clear_square( &display, 0, 24, 128, 8 );  

  if( num_blocks != 0 )
  {
    uint8_t num_blocks_str[32];
    snprintf( num_blocks_str, 32, "%d blocks", num_blocks );
    ssd1306_draw_string( &display, 0, 24, 1, num_blocks_str, 0 );
  }
}


void oled_display_inserted_write_protected( int8_t write_protected )
{
  ssd1306_clear_square( &display, 0, 32, 128, 8 );  

  if( write_protected != -1 )
  {
    ssd1306_draw_string( &display, 0, 32, 1, write_protected ? "Write protected" : "Not write protected", 0 );
  }
}


void oled_display_cartridge_error( uint8_t *str )
{
  ssd1306_clear_square( &display, 0, 40, 128, 8 );

  if( str )
  {
    ssd1306_draw_string( &display, 0, 40, 1, str, 0 );
  }
}


void oled_display_msg_requesting_status( void )
{
  oled_display_clear_msg();

  ssd1306_draw_filled_square(&display, 125, 61, 2, 2 );
}


void oled_display_msg_saving_mdr_data( microdrive_index_t microdrive_index )
{
  oled_display_clear_msg();

  uint8_t saving_msg[32];
  snprintf( saving_msg, 32, "Saving MD%d to SD card", microdrive_index+1 );
  ssd1306_draw_string( &display, 0, 56, 1, saving_msg, 0 );
}


void oled_display_clear_msg( void )
{
  ssd1306_clear_square( &display, 0, 56, 128, 8 );  
}


void oled_display_test_value( uint32_t val )
{
  uint8_t value_str[32];
  snprintf( value_str, 32, "%d", val );
  ssd1306_clear_square( &display, 0, 48, 128, 8 );  
  ssd1306_draw_string( &display, 0, 48, 1, value_str, 0 );
}


void oled_display_selectable_filename( uint8_t *filename, uint32_t ypos, bool invert )
{
  if( filename != NULL )
  {
    ssd1306_draw_string( &display, 0, ypos, 1, filename, invert );
  }
}
