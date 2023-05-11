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
#include "hardware/uart.h"

/* These are the FAT filesystem library headers */
#include "f_util.h"
#include "ff.h"
#include "rtc.h"
#include "hw_config.h"

#include "uart.h"
#include "microdrive.h"
#include "ui_io_comms.h"

/* This file is generated from test data, don't change what's in it manually */
#include "flash_images.h"

/*
 * These are the test images in flash. They can be memcpy'ed from flash memory,
 * but writing them back is more elaborate.
 *
 * Examine, for example, 98304 bytes of flash memory with:
 *
 * (gdb) x/98304xb flash_mdr_image[0].flash_address
 *
 * Remove any cruft from the bottom of the output (left side is OK) then convert
 * to binary with
 *
 * > perl -ne 'while( m/ (0x..)/g) {print chr(hex($1))}' < ram.txt > ram.bin
 */

/*
 * Details of an MDR image in flash. At the moment the address points to
 * the tape bytes. This will eventually be the complete MDR image sent
 * from SD card
 */
typedef struct _flash_mdr_image
{
  void     *flash_address;
  uint32_t  length;
}
flash_mdr_image_t;

static flash_mdr_image_t flash_mdr_image[NUM_MICRODRIVES] =
{
  { tape1_image, tape1_image_len },
  { tape2_image, tape2_image_len },
  { tape3_image, tape3_image_len },
  { tape4_image, tape4_image_len },
  { tape5_image, tape5_image_len },
  { tape6_image, tape6_image_len },
  { tape7_image, tape7_image_len },
  { tape8_image, tape8_image_len },
};

#include "ssd1306.h"

ssd1306_t display;

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


/*
 * This implementation loads the MDR image out of flash, where the test
 * images are compiled in. It'll have to work with the SD card in due course.
 */
void insert_mdr_image( uint8_t which, uint8_t *src, uint32_t length )
{
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 10, 10, 1, "Send cmd");
  ssd1306_show(&display);

  uart_putc_raw(UI_PICO_UART_ID, UI_TO_IO_INSERT_MDR);
  UI_TO_IO_CMD ack = uart_getc(UI_PICO_UART_ID);
  if( ack != UI_TO_IO_ACK )
    gpio_put( LED_PIN, 0 );   // Just break the pattern so I can see it's wrong

  /* Write the data which describes the command */
  ui_to_io_insert_mdr_t cmd_struct =
    {
      .microdrive_index   = which,
      .data_size          = length,
      .write_protected    = WRITE_PROTECT_OFF,    // Needs to come from final byte of the MDR file
      .checksum           = 0
    };
  uart_write_blocking(UI_PICO_UART_ID, (uint8_t*)&cmd_struct, sizeof(cmd_struct)); 	
  ack = uart_getc(UI_PICO_UART_ID);
  if( ack != UI_TO_IO_ACK )
    gpio_put( LED_PIN, 1 );   // Just break the pattern so I can see it's wrong

  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 10, 10, 1, "Sending data");
  ssd1306_show(&display);

  for( uint32_t i=0; i < length; i++ )
  {
    /* Feedback on screen, probably redundant when I get a proper GUI */
    if( (i % 16384) == 0 )
    {
      uint8_t msg[32];
      snprintf( msg, 16, "Drive %d, %d", which, i );
      ssd1306_clear(&display);
      ssd1306_draw_string(&display, 0, 0, 1, msg);
      ssd1306_show(&display);
    }

    uart_putc_raw(UI_PICO_UART_ID, *(src+i));
  }

  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 0, 0, 1, "Done");
  ssd1306_show(&display);

  return;
}

void insert_mdr_file( uint8_t which, uint8_t *filename )
{
  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 10, 10, 1, filename);
  ssd1306_show(&display);

  uart_putc_raw(UI_PICO_UART_ID, UI_TO_IO_INSERT_MDR);
  UI_TO_IO_CMD ack = uart_getc(UI_PICO_UART_ID);
  if( ack != UI_TO_IO_ACK )
    gpio_put( LED_PIN, 0 );   // Just break the pattern so I can see it's wrong


  /* Write the data which describes the command */
  ui_to_io_insert_mdr_t cmd_struct =
    {
      .microdrive_index   = which,
      .data_size          = 97197,
      .write_protected    = WRITE_PROTECT_ON,    // Needs to come from final byte of the MDR file
      .checksum           = 0
    };
  uart_write_blocking(UI_PICO_UART_ID, (uint8_t*)&cmd_struct, sizeof(cmd_struct)); 	
  ack = uart_getc(UI_PICO_UART_ID);
  if( ack != UI_TO_IO_ACK )
    gpio_put( LED_PIN, 1 );   // Just break the pattern so I can see it's wrong

  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 10, 10, 1, "Sending data");
  ssd1306_show(&display);


  FIL fil;
  FRESULT fr = f_open(&fil, filename, FA_READ);
  if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);

  for( uint32_t i=0; i < 97197; i++ )
  {
    /* Feedback on screen, probably redundant when I get a proper GUI */
    if( (i % 16384) == 0 )
    {
      uint8_t msg[32];
      snprintf( msg, 16, "Drive %d, %d", which, i );
      ssd1306_clear(&display);
      ssd1306_draw_string(&display, 0, 0, 1, msg);
      ssd1306_show(&display);
    }

    uint8_t byte;
    UINT    br;
    fr = f_read(&fil, &byte, 1, &br);
    if (br == 0) break; /* error or eof */

    uart_putc_raw(UI_PICO_UART_ID, byte);
  }

  fr = f_close(&fil);

  ssd1306_clear(&display);
  ssd1306_draw_string(&display, 0, 0, 1, "Done");
  ssd1306_show(&display);

  return;
}


int main( void )
{
  bi_decl(bi_program_description("ZX Spectrum Pico IF1 board binary."));

#ifdef OVERCLOCK
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put( LED_PIN, 0 );

  /*
   * First, set up the screen. It's an I2C device.
   */
  i2c_init(OLED_I2C, OLED_FREQ);
  gpio_set_function( OLED_SCK, GPIO_FUNC_I2C ); gpio_pull_up( OLED_SCK );
  gpio_set_function( OLED_SDA, GPIO_FUNC_I2C ); gpio_pull_up( OLED_SDA );
  display.external_vcc=false;

  ssd1306_init( &display, OLED_WIDTH, OLED_HEIGHT, OLED_ADDR, OLED_I2C );
  ssd1306_clear( &display );

  ssd1306_draw_string(&display, 10, 10, 2, "ZX Pico");
  ssd1306_show(&display);

  /*
   * Rotary encoder, 3 GPIOs
   */
  gpio_init( ENC_SW ); gpio_set_dir( ENC_SW, GPIO_IN ); // gpio_disable_pulls(ENC_SW);
  gpio_init( ENC_A );  gpio_set_dir( ENC_A, GPIO_IN );  // gpio_disable_pulls(ENC_A);
  gpio_init( ENC_B );  gpio_set_dir( ENC_B, GPIO_IN );  // gpio_disable_pulls(ENC_B);

  /* Set the handler for all 3 GPIOs */
  gpio_set_irq_enabled_with_callback( ENC_SW, GPIO_IRQ_EDGE_FALL, true, &encoder_callback );
  gpio_set_irq_enabled( ENC_A, GPIO_IRQ_EDGE_FALL, true );
  gpio_set_irq_enabled( ENC_B, GPIO_IRQ_EDGE_FALL, true );

  /* hw_config, i think? */
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
//    f_unmount(pSD->pcName);


  /*
   * Set up our UART to talk to the IO Pico
   */

  /*
   * I've soldered this Pico's UART0 to the IO Pico. The link was
   * originally SPI, so the IO Pico's UART pins are connected to
   * this Pico's SPI1 device pins. I'll cut the tracks when I've
   * finally decided what to do, but for now set this Pico's SPI
   * pins to inputs so they don't interfere with the UART link.
   */
  gpio_init(12); gpio_set_dir(12, GPIO_IN);
  gpio_init(13); gpio_set_dir(13, GPIO_IN);
  gpio_init(14); gpio_set_dir(14, GPIO_IN);
  gpio_init(15); gpio_set_dir(15, GPIO_IN);
  
  uart_init(UI_PICO_UART_ID, PICOS_BAUD_RATE);
  gpio_set_function(UI_PICO_UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UI_PICO_UART_RX_PIN, GPIO_FUNC_UART);

  /* Set UART flow control CTS/RTS */
  uart_set_hw_flow(UI_PICO_UART_ID, true, true);

  /* Set our data format, 8N1 */
  uart_set_format(UI_PICO_UART_ID, PICOS_DATA_BITS, PICOS_STOP_BITS, PICOS_PARITY);
  uart_set_translate_crlf(UI_PICO_UART_ID, false);

#if 0
  /* Send test flash images over to the IO Pico */
  for( microdrive_index_t microdrive_index=0; microdrive_index < NUM_MICRODRIVES; microdrive_index++ )
  {
    insert_mdr_image( microdrive_index, flash_mdr_image[microdrive_index].flash_address, flash_mdr_image[microdrive_index].length );
  }
#else

  /* Read files from SD card and insert each one */
  /* h8_254.mdr  mm.mdr  test_image_32blk.mdr  test_image.mdr */

  uint8_t *mdr_files[] = {"mm.mdr"};//, "h8_254.mdr", "test_image_32blk.mdr", "test_image.mdr" };

//  for( uint8_t mdr_file_index=0; mdr_file_index < sizeof(mdr_files); mdr_file_index++ )
//  {
    insert_mdr_file( 0, mdr_files[0] );
//  }
#endif

  while( 1 )
  {
    uart_putc_raw(UI_PICO_UART_ID, UI_TO_IO_TEST_LED_ON);
    gpio_put( LED_PIN, 1 );

    UI_TO_IO_CMD ack = uart_getc(UI_PICO_UART_ID);
    if( ack != UI_TO_IO_ACK )
      gpio_put( LED_PIN, 0 );   // Just break the pattern so I can see it's wrong
      
    sleep_ms(1000);




    uart_putc_raw(UI_PICO_UART_ID, UI_TO_IO_TEST_LED_OFF);
    gpio_put( LED_PIN, 0 );

    ack = uart_getc(UI_PICO_UART_ID);
    if( ack != UI_TO_IO_ACK )
      gpio_put( LED_PIN, 1 );   // Just break the pattern so I can see it's wrong

    sleep_ms(1000);

#if 0
    gpio_put( LED_PIN, 0 );
    sleep_ms(1000);

    if( value != previous_value )
    {
      uint8_t value_str[4];
      snprintf( value_str, 4, "%d", value );

      ssd1306_clear(&display);
      ssd1306_draw_string(&display, 10, 10, 2, value_str);
      ssd1306_show(&display);

      previous_value = value;
    }

#endif

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
