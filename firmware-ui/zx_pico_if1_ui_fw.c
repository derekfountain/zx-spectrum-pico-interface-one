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
#include "pico/time.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "hardware/timer.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

#include "fsm.h"
#include "gui_fsm.h"

#include "uart.h"
#include "microdrive.h"
#include "ui_io_comms.h"
#include "work_queue.h"

#include "sd_card.h"

#include "oled_display.h"
#include "ssd1306.h"
#include "gui.h"

/* Something to show on the screen */
uint8_t previous_value = 255;
uint8_t value          = 0;

/* 1 instruction on the 133MHz microprocessor is 7.5ns */
/* 1 instruction on the 140MHz microprocessor is 7.1ns */
/* 1 instruction on the 150MHz microprocessor is 6.6ns */
/* 1 instruction on the 200MHz microprocessor is 5.0ns */

//#define OVERCLOCK 150000
//#define OVERCLOCK 270000

const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;

/* https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/ is useful */

#define ENC_SW	 6     // Marked SW or Switch on some devices
#define ENC_B	 7     // Marked DT on some devices
#define ENC_A	 8     // Marked CLK on some devices

/*
 * This describes the data which has been loaded from the SD card and
 * sent to the IO pico for use by the Spectrum. I need to keep this in
 * order to be able to save data back to SD card, etc
 */
typedef struct _live_microdrive_data_t
{
  char           *filename;                // Name of SD card file loaded
  uint32_t        cartridge_data_length;   // Number of bytes in the cartridge image
  write_protect_t write_protected;         // Whether the cartridge is write protected in the IO Pico
}
live_microdrive_data_t;

static live_microdrive_data_t live_microdrive_data[NUM_MICRODRIVES];

/* Room for one full MDR image to work with. Includes w/p byte */
static uint8_t working_image_buffer[MICRODRIVE_MDR_MAX_LENGTH];

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
 * Send the given command to the IO Pico. The preamble is sent first so
 * the IO Pico can sync with the command byte which follows, then the
 * command byte itself, then it waits for the ACK to come back saying
 * the IO Pico has received it
 */
static bool send_cmd( UI_TO_IO_CMD cmd )
{
  uint8_t preamble[] = UI_TO_IO_CMD_PREAMBLE;

  /* Send the preamble */
  for( uint8_t preamble_index=0; preamble_index < sizeof(preamble); preamble_index++ )
    uart_putc_raw(UI_PICO_UART_ID, preamble[preamble_index]);

  /* Send the command, this is just one byte */
  uart_putc_raw(UI_PICO_UART_ID, cmd);

  /* Read the ACK from the IO Pico */
  UI_TO_IO_CMD ack = uart_getc(UI_PICO_UART_ID);

  if( ack != UI_TO_IO_ACK )
  {
    return false;
  }

  return true;
}


static void insert_mdr_file( uint8_t which, uint8_t *filename )
{
  /* Load full image, inc w/p byte, into the working buffer */
  uint32_t bytes_read;
  if( read_mdr_file( filename, working_image_buffer, MICRODRIVE_MDR_MAX_LENGTH, &bytes_read ) != 0 )
    return;
  
  /* Empty file? Not good... */
  if( bytes_read == 0 )
    return;

  /* Must be a round number of blocks otherwise it's probably not an MDR file */
  if( ((bytes_read-1) / MICRODRIVE_BLOCK_LEN) * MICRODRIVE_BLOCK_LEN != (bytes_read-1) )
    return;

  /* Minimum size in blocks, arbitrary for now */
  if( (bytes_read-1) / MICRODRIVE_BLOCK_LEN < 10 )
    return;

  oled_display_filename( filename );

  (void)send_cmd( UI_TO_IO_INSERT_MDR );

  /* Calculate the correct checksum */
  uint8_t checksum = 0;
  for( uint32_t i=0; i < bytes_read-1; i++ )
  {
    checksum += working_image_buffer[i];
  }

  /* Write the data which describes the command */
  write_protect_t write_protected = working_image_buffer[bytes_read-1] ? WRITE_PROTECT_ON : WRITE_PROTECT_OFF;
  ui_to_io_insert_mdr_t cmd_struct =
    {
      .microdrive_index   = which,
      .data_size          = bytes_read-1,
      .write_protected    = write_protected,
      .checksum           = checksum
    };
  uart_write_blocking(UI_PICO_UART_ID, (uint8_t*)&cmd_struct, sizeof(cmd_struct)); 	
  UI_TO_IO_CMD ack = uart_getc(UI_PICO_UART_ID);
  if( ack != UI_TO_IO_ACK )
    gpio_put( LED_PIN, 1 );

  for( uint32_t i=0; i < cmd_struct.data_size; i++ )
  {
    /* Feedback on screen, probably redundant when I get a proper GUI */
    if( (i % 16384) == 0 )
    {
      oled_display_show_progress( which, i );
    }

    uart_putc_raw( UI_PICO_UART_ID, working_image_buffer[i] );
  }

  /* Store away what the IO Pico is using */
  live_microdrive_data[which].filename              = filename;
  live_microdrive_data[which].cartridge_data_length = bytes_read-1;
  live_microdrive_data[which].write_protected       = write_protected;

  oled_display_done();

  return;
}


/* Timer function, called repeatedly to set up a request of status from the IO Pico */
static bool add_work_request_status( repeating_timer_t *rt )
{
  work_request_status_t *request_status_ptr = malloc( sizeof(work_request_status_t) );
  request_status_ptr->dummy = 0;
  insert_work( WORK_REQUEST_STATUS, request_status_ptr );

  return true;
}


/*
 * When status indicates the IO Pico has a microdrive with a cartridge
 * which has changed data, add a work item to request the data so it
 * can be saved back to SD card
 */
static bool add_work_request_mdr_data( microdrive_index_t microdrive_index )
{
  work_request_mdr_data_t *request_data_ptr = malloc( sizeof(work_request_mdr_data_t) );
  request_data_ptr->microdrive_index = microdrive_index;
  insert_work( WORK_REQUEST_MDR_DATA, request_data_ptr );

  return true;
}


static void init_io_link( void )
{
  /*
   * All this does is repeatedly try to get a preamble/cmd/ack sequence
   * from the IO Pico. If it fails, it tries again. It's hard to see
   * why this wouldn't work (once the Picos are booted and running)
   * but if it doesn't we can't really proceed so we might as well
   * just spin trying.
   */
  while( send_cmd( UI_TO_IO_INIALISE ) == false );

  return;
}


static void request_status( void )
{
  (void)send_cmd( UI_TO_IO_REQUEST_STATUS );
    
  /* Write the data which describes the command */
  ui_to_io_request_status_t cmd_struct =
    {
      .dummy              = 0xFF,        // Eyecatcher, nothing actually required here as yet
    };
  uart_write_blocking(UI_PICO_UART_ID, (uint8_t*)&cmd_struct, sizeof(ui_to_io_request_status_t)); 	

  io_to_ui_status_response_t status_struct;
  uart_read_blocking(UI_PICO_UART_ID, (uint8_t*)&status_struct, sizeof(io_to_ui_status_response_t)); 	

  /* I need to emit the values into the GUI somehow. For now, just print it */
  oled_display_status_bytes( &status_struct );
  
  /* Look for microdrives which need their cartridge saving */
  for( microdrive_index_t microdrive_index = 0; microdrive_index < NUM_MICRODRIVES; microdrive_index++ )
  {
    if( status_struct.status[microdrive_index] == MD_STATUS_MDR_LOADED_NEEDS_SAVING )
    {
      add_work_request_mdr_data( microdrive_index );
    } 
  }
}


static void request_mdr_data_to_save( microdrive_index_t microdrive_index )
{
  (void)send_cmd( UI_TO_IO_REQUEST_MDR_TO_SAVE );

  /* Write the data which describes the command */
  ui_to_io_request_mdr_data_t cmd_struct =
    {
      .microdrive_index = microdrive_index,
      .bytes_expected   = live_microdrive_data[microdrive_index].cartridge_data_length,
    };
  uart_write_blocking(UI_PICO_UART_ID, (uint8_t*)&cmd_struct, sizeof(ui_to_io_request_mdr_data_t)); 

  /* Read the cartridge contents back. This is sent by the IO Pico in pages, but arrives all in one go */
  uart_read_blocking(UI_PICO_UART_ID, working_image_buffer, live_microdrive_data[microdrive_index].cartridge_data_length);

  working_image_buffer[live_microdrive_data[microdrive_index].cartridge_data_length] =
                                                                   live_microdrive_data[microdrive_index].write_protected;
  
  uint32_t bytes_written;
  write_mdr_file( live_microdrive_data[microdrive_index].filename,
		  working_image_buffer,
		  live_microdrive_data[microdrive_index].cartridge_data_length+1, &bytes_written );
}


/*
 * Core1 does what is termed "the work". This is the meaty stuff - comms with the IO Pico,
 * copying cartridge image data around, updating status, etc. The work here can block for
 * a while, or take some time to complete. (Copying an MDR image to or from a Pico via the
 * UART can take significant time.) So this work happens in the background and doesn't
 * interfere with the GUI.
 */
static void __time_critical_func(core1_main)( void )
{
  while( 1 )
  {
    work_queue_type_t type;
    void             *data;
    if( remove_work( &type, &data  ) )
    {
      /* There's work to be done - what is it? */
      switch( type )
      {
      case WORK_INIT_IO_LINK:
      {
	/* Work required is to link to the IO Pico, can't proceed without this */
	work_init_io_link_t *init_io_link_data = (work_init_io_link_t*)data;

	init_io_link();
	free(init_io_link_data);
      }
      break;

      case WORK_INSERT_MDR:
      {
	/* Work required is the insertion of a cartridge. The mdr/filename is in the data structure */
	work_insert_mdr_t *insert_mdr_data = (work_insert_mdr_t*)data;

	insert_mdr_file( insert_mdr_data->microdrive_index, insert_mdr_data->filename );
	free(insert_mdr_data);
      }
      break;

      case WORK_REQUEST_STATUS:
      {
	/* Work required is to ask the IO Pico to send its status */
	work_request_status_t *request_status_data = (work_request_status_t*)data;

	request_status();
	free(request_status_data);
      }
      break;

      case WORK_REQUEST_MDR_DATA:
      {
	/* Work required is to ask the IO Pico to send the data for one of the drives */
	work_request_mdr_data_t *request_mdr_data = (work_request_mdr_data_t*)data;

	request_mdr_data_to_save( request_mdr_data->microdrive_index );
	free(request_mdr_data);
      }
      break;

      default:
	break;
      }
    }

  } /* Infinite loop */
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
  oled_display_init();

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

  /* Mount the SD card, if it's ready */
  mount_sd_card();

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

  /* Initialise the never ending list of things that need doing */
  work_queue_init();

  /*
   * For some reason the second core code doesn't get started after SWD programming
   * unless I pause for a moment here
   */
  busy_wait_us_32(100000);

  /* Init complete, run 2nd core code */
  multicore_launch_core1( core1_main ); 

  /* First piece of work is to establish the link to the IO Pico */
  work_init_io_link_t *init_io_work_ptr = malloc( sizeof(work_init_io_link_t) );
  insert_work( WORK_INIT_IO_LINK, init_io_work_ptr );

  /* Set requests for microdrive status running. Don't move the var anywhere it might get lost */
  repeating_timer_t repeating_status_timer;
  add_repeating_timer_ms( 500, add_work_request_status, NULL, &repeating_status_timer );

#define TEST_SETUP 1
#if TEST_SETUP
  /* Read named files from SD card and insert each one that exists */
  uint8_t *mdr_files[] = {"1.mdr", 
			  "2.mdr", 
			  "3.mdr", 
			  "4.mdr", 
			  "5.mdr", 
			  "6.mdr", 
			  "7.mdr", 
			  "8.mdr"};

  for( uint8_t mdr_file_index=0; mdr_file_index < (sizeof(mdr_files)/sizeof(uint8_t*)); mdr_file_index++ )
  {
    /* Insert MDR files into some drives. This is test code, to be removed */
    work_insert_mdr_t *work_ptr = malloc( sizeof(work_insert_mdr_t) );

    work_ptr->microdrive_index = mdr_file_index;
    work_ptr->filename         = mdr_files[mdr_file_index];

    insert_work( WORK_INSERT_MDR, work_ptr );
  }
#endif

  /* Create the finite state machine which operates the GUI */
  fsm_t *gui_fsm;
  if( (gui_fsm=create_fsm( query_gui_fsm_map(), query_gui_fsm_initial_state() )) == NULL )
    panic("Unable to create GUI FSM");

  /*
   * This loop looks after the user interface. It's a state machine which monitors devices
   * driven by interrupts, etc. Anything which takes more than trivial time is considered
   * "work" and will be added by this to the work queue, from where the other core will
   * do it in due course. Nothing here should block or take any significant time, otherwise
   * the GUI will freeze up.
   *
   * Start by kicking the state machine.
   */
  generate_stimulus( gui_fsm, FSM_STIMULUS_YES );
  while( 1 )
  {
    process_fsms();

#if 0
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
