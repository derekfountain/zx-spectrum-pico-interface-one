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
 * cmake -DCMAKE_BUILD_TYPE=Debug ..
 * make -j10
 * sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -c "program ./zx_pico_if1_ui_fw.elf verify reset exit"
 * sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg
 * gdb-multiarch ./zx_pico_if1_ui_fw.elf
 *  target remote localhost:3333
 *  load
 *  monitor reset init
 *  continue
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "hardware/timer.h"
#include "hardware/spi.h"

#include "f_util.h"
#include "ff.h"
#include "rtc.h"
#include "hw_config.h"

#include "spi.h"

#include "ssd1306.h"

#define OLED_I2C    (i2c0)
#define OLED_FREQ   400000
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_ADDR   0x3C
#define OLED_SCK    4
#define OLED_SDA    5

/* 1 instruction on the 133MHz microprocessor is 7.5ns */
/* 1 instruction on the 140MHz microprocessor is 7.1ns */
/* 1 instruction on the 150MHz microprocessor is 6.6ns */
/* 1 instruction on the 200MHz microprocessor is 5.0ns */

//#define OVERCLOCK 150000
//#define OVERCLOCK 270000

//const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;

/* https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/ is useful */

#define ENC_SW	 6     // Marked SW or Switch on some devices
#define ENC_B	 7     // Marked DT on some devices
#define ENC_A	 8     // Marked CLK on some devices

/* Something to show on the screen */
uint8_t previous_value = 255;
uint8_t value          = 0;

void encoder_callback( uint gpio, uint32_t events ) 
{
  uint32_t gpio_state = (gpio_get_all() >> ENC_B) & 0x0003;
  uint8_t  enc_value  = (gpio_state & 0x03);
	
  static bool counterclockwise_fall = 0;
  static bool clockwise_fall        = 0;
	
  if( gpio == ENC_A ) 
  {
    if( (!clockwise_fall) && (enc_value == 0x01) )
      clockwise_fall = 1; 

    if( (counterclockwise_fall) && (enc_value == 0x00) )
    {
      /* Counter clockwise event */
      clockwise_fall        = 0;
      counterclockwise_fall = 0;

      /* Do application action here */
      value--;
      gpio_put( LED_PIN, 1 );
    }
  }	
  else if( gpio == ENC_B )
  {
    if( (!counterclockwise_fall) && (enc_value == 0x02) )
      counterclockwise_fall = 1;

    if( (clockwise_fall) && (enc_value == 0x00) )
    {
      /* Clockwise event */
      clockwise_fall        = 0;
      counterclockwise_fall = 0;

      /* Do application action here */
      value++;
      gpio_put( LED_PIN, 0 );
    }    
  }
  else if( gpio == ENC_SW )
  {
    /*
     * Switch event, set to interrupt on falling edge, so this is a click down.
     * Debounce is left as an exercise for the reader :)
     */
    value = 0;
  }
}


int main( void )
{
  bi_decl(bi_program_description("ZX Spectrum Pico IF1 board binary."));

#ifdef OVERCLOCK
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  /*
   * First, set up the screen. It's an I2C device.
   */
  i2c_init(OLED_I2C, OLED_FREQ);
  gpio_set_function( OLED_SCK, GPIO_FUNC_I2C ); gpio_pull_up( OLED_SCK );
  gpio_set_function( OLED_SDA, GPIO_FUNC_I2C ); gpio_pull_up( OLED_SDA );

  ssd1306_t display;
  display.external_vcc=false;

  ssd1306_init( &display, OLED_WIDTH, OLED_HEIGHT, OLED_ADDR, OLED_I2C );
  ssd1306_clear( &display );

  ssd1306_draw_string(&display, 10, 10, 2, "ZX Pico");
  ssd1306_show(&display);

  /* Rotary encoder, 3 GPIOs */
  gpio_init( ENC_SW ); gpio_set_dir( ENC_SW, GPIO_IN ); // gpio_disable_pulls(ENC_SW);
  gpio_init( ENC_A );  gpio_set_dir( ENC_A, GPIO_IN );  // gpio_disable_pulls(ENC_A);
  gpio_init( ENC_B );  gpio_set_dir( ENC_B, GPIO_IN );  // gpio_disable_pulls(ENC_B);

  /* Set the handler for all 3 GPIOs */
  gpio_set_irq_enabled_with_callback( ENC_SW, GPIO_IRQ_EDGE_FALL, true, &encoder_callback );
  gpio_set_irq_enabled( ENC_A, GPIO_IRQ_EDGE_FALL, true );
  gpio_set_irq_enabled( ENC_B, GPIO_IRQ_EDGE_FALL, true );

    // See FatFs - Generic FAT Filesystem Module, "Application Interface",
    // http://elm-chan.org/fsw/ff/00index_e.html
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    FIL fil;
    const char* const filename = "filen_df.txt";
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&fil, "Hello, world!\n") < 0) {
        printf("f_printf failed\n");
    }
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
    f_unmount(pSD->pcName);

  /* Loop, not doing much for this example. Just update the display if value changes */
  while( true ) 
  {
    if( value != previous_value )
    {
      uint8_t value_str[4];
      snprintf( value_str, 4, "%d", value );

      ssd1306_clear(&display);
      ssd1306_draw_string(&display, 10, 10, 2, value_str);
      ssd1306_show(&display);

      previous_value = value;
    }
    sleep_ms(5);
  }

  /*
   * With reference to this thread:
   *  https://forums.raspberrypi.com//viewtopic.php?f=145&t=300589
   * setting up the slave-select as controlled by SPI hardware means
   * the line is pulsed after each byte. That's the way the Pico
   * hardware does it. The slave side, which is the IO Pico in this
   * case, needs to recognise that as the standard being used, which
   * it does as long as the IO Pico code also sets up with the 
   * slave-select line as SPI hardware controlled.
   */
  spi_init(UI_TO_IO_SPI, UI_TO_IO_SPI_SPEED);
  gpio_set_function(UI_TO_IO_SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(UI_TO_IO_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(UI_TO_IO_SPI_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(UI_TO_IO_SPI_CSN_PIN, GPIO_FUNC_SPI);

  gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put( LED_PIN, 0 );

  uint8_t test_data[] = { 0,1,2,3,4,5,6,7,8,9,
			  10,11,12,13,14,15,16,17,18,19,
			  20,21,22,23,24,25,26,27,28,29 };
  while( 1 )
  {
//    gpio_put( UI_TO_IO_SPI_CSN_PIN, 0 );
    UI_TO_IO_CMD led_on = UI_TO_IO_TEST_LED_ON;
    spi_write_blocking(UI_TO_IO_SPI, test_data, sizeof(test_data));
///    spi_write_blocking(UI_TO_IO_SPI, &led_on, sizeof(UI_TO_IO_CMD));
//    gpio_put( UI_TO_IO_SPI_CSN_PIN, 1 );

    gpio_put( LED_PIN, 1 );
    sleep_ms(1000);



//    gpio_put( UI_TO_IO_SPI_CSN_PIN, 0 );
//    UI_TO_IO_CMD led_off = UI_TO_IO_TEST_LED_OFF;
//    spi_write_blocking(UI_TO_IO_SPI, &led_off, sizeof(UI_TO_IO_CMD));
//    gpio_put( UI_TO_IO_SPI_CSN_PIN, 1 );

    gpio_put( LED_PIN, 0 );
    sleep_ms(1000);

  } /* Infinite loop */

}



#if 0
/* Blip the result pin, shows on scope */
gpio_put( TEST_OUTPUT_GP, 1 );
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
gpio_put( TEST_OUTPUT_GP, 0 );
busy_wait_us_32(1);
#endif
