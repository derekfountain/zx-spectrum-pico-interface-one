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

#include "fsm.h"
#include "gui_fsm.h"

#include "microdrive.h"
#include "ui_io_comms.h"
#include "work_queue.h"
#include "live_microdrive_data.h"

#include "sd_card.h"

#include "oled_display.h"
#include "ssd1306.h"
#include "gui.h"

/* Link to IO Pico is done with transputer based PIO code */
#include "picoputer.pio.h"
#include "link_common.h"

const uint8_t LINKOUT_PIN     = 0;
const uint8_t LINKIN_PIN      = 1;

static int linkout_sm;
static int linkin_sm;

/* 1 instruction on the 133MHz microprocessor is 7.5ns */
/* 1 instruction on the 140MHz microprocessor is 7.1ns */
/* 1 instruction on the 150MHz microprocessor is 6.6ns */
/* 1 instruction on the 200MHz microprocessor is 5.0ns */

//#define OVERCLOCK 150000
//#define OVERCLOCK 270000

const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;

/* https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/ is useful */

#define ENCODER_SW_GP	 6     // Marked SW or Switch on some devices
#define ENCODER_B_GP	 7     // Marked DT on some devices
#define ENCODER_A_GP	 8     // Marked CLK on some devices

#define ACTION_SW_GP  26
#define CANCEL_SW_GP  22

static fsm_t *gui_fsm;

/* Keep tabs on what's happening so the GUI can offer the correct options */
static live_microdrive_data_t live_microdrive_data;
auto_init_mutex( live_microdrive_data_mutex );

/* Timer used to fetch status from IO Pico every so often */
#define STATUS_TIMER_PERIOD_MS 500
static repeating_timer_t repeating_status_timer;

/* Room for one full MDR image to work with. Includes w/p byte */
static uint8_t working_image_buffer[MICRODRIVE_MDR_MAX_LENGTH];


/* Rotary encoder callback. Based on one I found on the internet, no licence attached */
static void encoder_callback( uint gpio, uint32_t events ) 
{
  uint32_t gpio_state = (gpio_get_all() >> ENCODER_B_GP) & 0x0003;
  uint8_t  enc_value  = (gpio_state & 0x03);
	
  static bool counterclockwise_fall = 0;
  static bool clockwise_fall        = 0;
	
  if( gpio == ENCODER_A_GP ) 
  {
    if( (!clockwise_fall) && (enc_value == 0x01) )
      clockwise_fall = 1; 

    if( (counterclockwise_fall) && (enc_value == 0x00) )
    {
      /* Counter clockwise event */
      clockwise_fall        = 0;
      counterclockwise_fall = 0;

      /* Do application action here */
      generate_stimulus( gui_fsm, ST_ROTATE_CCW );  
    }
  }	
  else if( gpio == ENCODER_B_GP )
  {
    if( (!counterclockwise_fall) && (enc_value == 0x02) )
      counterclockwise_fall = 1;

    if( (clockwise_fall) && (enc_value == 0x00) )
    {
      /* Clockwise event */
      clockwise_fall        = 0;
      counterclockwise_fall = 0;

      /* Do application action here */
      generate_stimulus( gui_fsm, ST_ROTATE_CW );  
    }    
  }
  else if( gpio == ENCODER_SW_GP )
  {
    /*
     * Switch event, set to interrupt on falling edge, so this is a click down.
     * Debounce is left as an exercise for the reader :)
     */
  }
}


/*
 * Handler for GPIOs. At present this marshalls the user interface input switches.
 */
void gpios_callback( uint gpio, uint32_t events ) 
{
  if( gpio == ENCODER_A_GP || gpio == ENCODER_B_GP || gpio == ENCODER_SW_GP )
  {
    /* Rotary encoder has its own handler */
    return encoder_callback( gpio, events );
  }
  else
  {
    if( gpio == ACTION_SW_GP )
    {
      generate_stimulus( gui_fsm, ST_ACTION_BUTTON_PRESS );
    }
    else if( gpio == CANCEL_SW_GP )
    {
      generate_stimulus( gui_fsm, ST_CANCEL_BUTTON_PRESS );
    }
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
    send_byte( pio0, linkout_sm, linkin_sm, preamble[preamble_index] );

  /* Send the command, this is just one byte */
  send_byte( pio0, linkout_sm, linkin_sm, cmd );

  /* Read the ACK from the IO Pico */
  UI_TO_IO_CMD ack;
  while( receive_acked_byte( pio0, linkin_sm, linkout_sm, &ack ) != LINK_BYTE_DATA );

  if( ack != UI_TO_IO_ACK )
  {
    return false;
  }

  return true;
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


static void work_insert_mdr_file( uint8_t which, uint8_t *filename )
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

  mutex_enter_blocking( &live_microdrive_data_mutex );
  live_microdrive_data.currently_inserted[which].status   = LIVE_STATUS_INSERTING;
  live_microdrive_data.currently_inserted[which].filename = filename;
  mutex_exit( &live_microdrive_data_mutex );

  /* Turn off the timer which sets up a status fetch, this routine takes a while */
  cancel_repeating_timer( &repeating_status_timer );

  generate_stimulus( gui_fsm, ST_MDR_INSERTING );

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
  send_buffer( pio0, linkout_sm, linkin_sm, (uint8_t*)&cmd_struct, sizeof(cmd_struct) );

  UI_TO_IO_CMD ack;
  while( receive_acked_byte( pio0, linkin_sm, linkout_sm, &ack ) != LINK_BYTE_DATA );

  if( ack != UI_TO_IO_ACK )
    gpio_put( LED_PIN, 1 );

  for( uint32_t i=0; i < cmd_struct.data_size; i++ )
  {
    /* Feedback on screen, probably redundant when I get a proper GUI */
    if( (i % 16384) == 0 )
    {
      // FIXME Do i need feedback here?
    }

    send_byte( pio0, linkout_sm, linkin_sm, working_image_buffer[i] );
  }

  /*
   * Store away what the IO Pico is using. The other core is continuously reading
   * this structure so mutex is required
   */
  mutex_enter_blocking( &live_microdrive_data_mutex );
  live_microdrive_data.currently_inserted[which].status                = LIVE_STATUS_INSERTED;
  live_microdrive_data.currently_inserted[which].filename              = filename;
  live_microdrive_data.currently_inserted[which].cartridge_data_length = bytes_read-1;
  live_microdrive_data.currently_inserted[which].write_protected       = write_protected;
  mutex_exit( &live_microdrive_data_mutex );

  /* Poke state machine to update GUI */
  generate_stimulus( gui_fsm, ST_MDR_INSERTED );

  /* Restart the status timer */
  add_repeating_timer_ms( STATUS_TIMER_PERIOD_MS, add_work_request_status, NULL, &repeating_status_timer );

  return;
}


static void work_init_io_link( void )
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


static void work_request_status( void )
{
  generate_stimulus( gui_fsm, ST_REQUEST_STATUS );
  
  (void)send_cmd( UI_TO_IO_REQUEST_STATUS );
    
  /* Write the data which describes the command */
  ui_to_io_request_status_t cmd_struct =
    {
      .dummy              = 0xFF,        // Eyecatcher, nothing actually required here as yet
    };
  send_buffer( pio0, linkout_sm, linkin_sm, (uint8_t*)&cmd_struct, sizeof(ui_to_io_request_status_t) );

  io_to_ui_status_response_t status_struct;
  receive_buffer( pio0, linkin_sm, linkout_sm, (uint8_t*)&status_struct, sizeof(io_to_ui_status_response_t) );

  /* Look for microdrives which need their cartridge saving */
  for( microdrive_index_t microdrive_index = 0; microdrive_index < NUM_MICRODRIVES; microdrive_index++ )
  {
    if( (status_struct.status[microdrive_index] == MD_STATUS_MDR_LOADED_NEEDS_SAVING)
	||
        (status_struct.status[microdrive_index] == MD_STATUS_MDR_EJECTED_NEEDS_SAVING) )
    {
      /* If this one needs its data saving back to SD card, request the data for the IO Pico */
      add_work_request_mdr_data( microdrive_index );
    } 

    /*
     * The eject works by this UI Pico sending an eject request message to
     * the IO Pico, which will result in a status of NO_CARTRIDGE once the
     * IO Pico has completed it. So this is the point to update the live
     * status once the cartridge is reported as not there.
     * Otherwise the cartridge is either inserted or being inserted, and
     * the insertion code will update the live status as required.
     */
    switch( status_struct.status[microdrive_index] )
    {
    case MD_STATUS_EMPTY:
      /* If there's been an eject, update the live status here... */
      if( live_microdrive_data.currently_inserted[microdrive_index].status != LIVE_STATUS_NO_CARTRIDGE )
      {
	mutex_enter_blocking( &live_microdrive_data_mutex );
	live_microdrive_data.currently_inserted[microdrive_index].status = LIVE_STATUS_NO_CARTRIDGE;
	mutex_exit( &live_microdrive_data_mutex );
      }
      break;

    default:
      /* ...otherwise just leave things as they are */
      break;
    }

  }

  generate_stimulus( gui_fsm, ST_REQUEST_STATUS_DONE );
}


static void work_request_mdr_data_to_save( microdrive_index_t microdrive_index )
{
  live_microdrive_data.microdrive_saving_to_sd = microdrive_index;
  generate_stimulus( gui_fsm, ST_REQUEST_DATA_TO_SAVE );

  /* Turn off the timer which sets up a status fetch, this routine takes a while */
  cancel_repeating_timer( &repeating_status_timer );

  (void)send_cmd( UI_TO_IO_REQUEST_MDR_TO_SAVE );

  /*
   * Take a copy of what's currently in the live data. I'm not sure if this mutex is
   * required, but until the GUI is finished I'll keep it in to be safe
   */
  mutex_enter_blocking( &live_microdrive_data_mutex );

  uint8_t         filename[strlen(live_microdrive_data.currently_inserted[microdrive_index].filename)+1];
  strncpy(filename, live_microdrive_data.currently_inserted[microdrive_index].filename, sizeof(filename) );
  filename[sizeof(filename)-1] = 0;

  uint32_t        bytes_expected  = live_microdrive_data.currently_inserted[microdrive_index].cartridge_data_length;
  write_protect_t write_protected = live_microdrive_data.currently_inserted[microdrive_index].write_protected;

  mutex_exit( &live_microdrive_data_mutex );


  /* Write the data which describes the command */
  ui_to_io_request_mdr_data_t cmd_struct =
    {
      .microdrive_index = microdrive_index,
      .bytes_expected   = bytes_expected,
    };
  send_buffer( pio0, linkout_sm, linkin_sm, (uint8_t*)&cmd_struct, sizeof(ui_to_io_request_mdr_data_t) );

  /* Read the cartridge contents back. This is sent by the IO Pico in pages, but arrives all in one go */
  receive_buffer( pio0, linkin_sm, linkout_sm, working_image_buffer, bytes_expected );

  working_image_buffer[bytes_expected] = write_protected;
  
  /* Write the MDR file back out to SD card */
  uint32_t bytes_written;
  write_mdr_file( filename, working_image_buffer, bytes_expected+1, &bytes_written );

  live_microdrive_data.microdrive_saving_to_sd = -1;
  generate_stimulus( gui_fsm, ST_DATA_SAVED );

  /* Restart the status timer */
  add_repeating_timer_ms( STATUS_TIMER_PERIOD_MS, add_work_request_status, NULL, &repeating_status_timer );

  return;
}


static void work_eject_mdr( microdrive_index_t microdrive_index )
{
  (void)send_cmd( UI_TO_IO_REQUEST_EJECT_MDR );

  /* Write the data which describes the command */
  ui_to_io_request_eject_mdr_t cmd_struct =
    {
      .microdrive_index   = microdrive_index,
    };
  send_buffer( pio0, linkout_sm, linkin_sm, (uint8_t*)&cmd_struct, sizeof(cmd_struct) );

  /* Status will be fetched again very shortly, no need to force anything */
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

	work_init_io_link();
	free(init_io_link_data);
      }
      break;

      case WORK_INSERT_MDR:
      {
	/* Work required is the insertion of a cartridge. The mdr/filename is in the data structure */
	work_insert_mdr_t *insert_mdr_data = (work_insert_mdr_t*)data;

	work_insert_mdr_file( insert_mdr_data->microdrive_index, insert_mdr_data->filename );
	free(insert_mdr_data);
      }
      break;

      case WORK_REQUEST_STATUS:
      {
	/* Work required is to ask the IO Pico to send its status */
	work_request_status_t *request_status_data = (work_request_status_t*)data;

	work_request_status();
	free(request_status_data);
      }
      break;

      case WORK_REQUEST_MDR_DATA:
      {
	/* Work required is to ask the IO Pico to send the data for one of the drives */
	work_request_mdr_data_t *request_mdr_data = (work_request_mdr_data_t*)data;

	work_request_mdr_data_to_save( request_mdr_data->microdrive_index );
	free(request_mdr_data);
      }
      break;

      case WORK_EJECT_MDR:
      {
	/* Work required is to eject the cartridge in the microdrive given in the data */
	work_eject_mdr_data_t *eject_mdr_data = (work_eject_mdr_data_t*)data;

	work_eject_mdr( eject_mdr_data->microdrive_index );
	free(eject_mdr_data);
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
  gpio_init( ENCODER_SW_GP ); gpio_set_dir( ENCODER_SW_GP, GPIO_IN ); // gpio_disable_pulls(ENCODER_SW_GP);
  gpio_init( ENCODER_A_GP );  gpio_set_dir( ENCODER_A_GP, GPIO_IN );  // gpio_disable_pulls(ENCODER_A_GP);
  gpio_init( ENCODER_B_GP );  gpio_set_dir( ENCODER_B_GP, GPIO_IN );  // gpio_disable_pulls(ENCODER_B_GP);

  gpio_init( ACTION_SW_GP ); gpio_set_dir( ACTION_SW_GP, GPIO_IN ); gpio_pull_up( ACTION_SW_GP );
  gpio_init( CANCEL_SW_GP ); gpio_set_dir( CANCEL_SW_GP, GPIO_IN ); gpio_pull_up( CANCEL_SW_GP );

  /* Set the handler for all 3 GPIOs */
  gpio_set_irq_enabled_with_callback( ENCODER_SW_GP, GPIO_IRQ_EDGE_FALL, true, &gpios_callback );
  gpio_set_irq_enabled( ENCODER_A_GP, GPIO_IRQ_EDGE_FALL, true );
  gpio_set_irq_enabled( ENCODER_B_GP, GPIO_IRQ_EDGE_FALL, true );

  gpio_set_irq_enabled( ACTION_SW_GP, GPIO_IRQ_EDGE_FALL, true );
  gpio_set_irq_enabled( CANCEL_SW_GP, GPIO_IRQ_EDGE_FALL, true );

  /* Mount the SD card, if it's ready */
  mount_sd_card();

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

  /* Set up the link to the other Pico */
  gpio_init(LINKOUT_PIN); gpio_set_dir(LINKOUT_PIN,GPIO_OUT); gpio_put(LINKOUT_PIN, 1);
  gpio_set_function(LINKOUT_PIN, GPIO_FUNC_PIO0);

  gpio_init(LINKIN_PIN); gpio_set_dir(LINKIN_PIN,GPIO_IN);
  gpio_set_function(LINKIN_PIN, GPIO_FUNC_PIO0);
    
  /* Outgoing side of the link */
  linkout_sm      = pio_claim_unused_sm(pio0, true);
  uint offset     = pio_add_program(pio0, &picoputerlinkout_program);
  picoputerlinkout_program_init(pio0, linkout_sm, offset, LINKOUT_PIN);

  /* Incoming side of the link */
  linkin_sm       = pio_claim_unused_sm(pio0, true);
  offset          = pio_add_program(pio0, &picoputerlinkin_program);
  picoputerlinkin_program_init(pio0, linkin_sm, offset, LINKIN_PIN);

  send_init_sequence( pio0, linkout_sm, linkin_sm );

  /* Initialise the live data */
  live_microdrive_data.microdrive_saving_to_sd = -1;
  for( microdrive_index_t microdrive_index = 0; microdrive_index < NUM_MICRODRIVES; microdrive_index++ )
  {
    live_microdrive_data.currently_inserted[microdrive_index].status                = LIVE_STATUS_NO_CARTRIDGE;
    live_microdrive_data.currently_inserted[microdrive_index].filename              = NULL;
    live_microdrive_data.currently_inserted[microdrive_index].cartridge_data_length = 0;
    live_microdrive_data.currently_inserted[microdrive_index].write_protected       = WRITE_PROTECT_OFF;
  }

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

  /* Set requests for microdrive status running */
  add_repeating_timer_ms( STATUS_TIMER_PERIOD_MS, add_work_request_status, NULL, &repeating_status_timer );

#define TEST_SETUP 1
#if TEST_SETUP
  /* Read named files from SD card and insert each one that exists */
  uint8_t *mdr_files[] = {"1.mdr", 
			  "2.mdr", 
			  "3x.mdr", 
			  "4x.mdr", 
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
  if( (gui_fsm=create_fsm( query_gui_fsm_map(),
			   query_gui_fsm_binding(),
			   query_gui_fsm_initial_state(),
			   &live_microdrive_data )) == NULL )
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
  generate_stimulus( gui_fsm, ST_BUILTIN_YES );
  while( 1 )
  {
    process_fsms();
  }

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


  for( uint8_t blink=0; blink<5; blink++)
  {
    gpio_put( LED_PIN, 1 );
    busy_wait_us_32(500000);
    gpio_put( LED_PIN, 0 );
    busy_wait_us_32(500000);
  }
  busy_wait_us_32(5000000);

#endif
