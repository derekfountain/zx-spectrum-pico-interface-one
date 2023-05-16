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

  ssd1306_draw_string(&display, 10, 10, 2, "ZX Pico");
  ssd1306_show(&display);

  return;
}



/* Hardcoded nonense to print some values which are currently useful */
void oled_display_status_bytes( io_to_ui_status_response_t *status_struct )
{
  uint8_t line = 16;
  for( uint8_t i=0; i<8; i+=2 )
  {
    uint8_t value_str[32];
    snprintf( value_str, 16, "0x%02X 0x%02X", status_struct->status[i], status_struct->status[i+1] );

    ssd1306_draw_string(&display, 0, line+=8, 1, value_str);
    ssd1306_show(&display);
  }

  return;
}

void oled_display_done( void )
{
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 0, 0, 1, "Done");
  ssd1306_show(&display);
}

void oled_display_filename( uint8_t *filename )
{
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 10, 10, 1, filename);
  ssd1306_show(&display);
}

void oled_display_show_progress( uint8_t which, uint32_t i )
{
  uint8_t msg[32];
  snprintf( msg, 16, "Drive %d, %d", which, i );
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 0, 0, 1, msg);
  ssd1306_show(&display);
}
