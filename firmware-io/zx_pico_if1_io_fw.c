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
 * sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -c "program ./zx_pico_if1_io_fw.elf verify reset exit"
 * sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg
 * gdb-multiarch ./zx_pico_if1_io_fw.elf
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

#include "if1.h"
#include "spi.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "mreq_dir.pio.h"

/* 1 instruction on the 133MHz microprocessor is 7.5ns */
/* 1 instruction on the 140MHz microprocessor is 7.1ns */
/* 1 instruction on the 150MHz microprocessor is 6.6ns */
/* 1 instruction on the 200MHz microprocessor is 5.0ns */

#define OVERCLOCK 150000
//#define OVERCLOCK 270000

const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;

/*
 * The IF1 port signal comes from the ROM Pico which has can see the
 * address lines. When that's active (low) the Z80 has 0xEF or 0xE7
 * on the lower address lines. The extra bit is A3 which this Pico
 * still needs to be able to see to differentiate them
 */
const uint8_t  IF1_PORT_SIGNAL_GP         = 8;
const uint8_t  A3_GP                      = 9;

const uint32_t IF1_PORT_SIGNAL_BIT_MASK = ((uint32_t)1 <<  IF1_PORT_SIGNAL_GP);
const uint32_t A3_BIT_MASK              = ((uint32_t)1 <<  A3_GP);

const uint8_t  D0_GP        = 0;
const uint8_t  D1_GP        = 1;
const uint8_t  D2_GP        = 2;
const uint8_t  D3_GP        = 3; 
const uint8_t  D4_GP        = 4;
const uint8_t  D5_GP        = 5;
const uint8_t  D6_GP        = 6;
const uint8_t  D7_GP        = 7;

const uint32_t D0_BIT_MASK  = ((uint32_t)1 <<  D0_GP);
const uint32_t D1_BIT_MASK  = ((uint32_t)1 <<  D1_GP);
const uint32_t D2_BIT_MASK  = ((uint32_t)1 <<  D2_GP);
const uint32_t D3_BIT_MASK  = ((uint32_t)1 <<  D3_GP);
const uint32_t D4_BIT_MASK  = ((uint32_t)1 <<  D4_GP);
const uint32_t D5_BIT_MASK  = ((uint32_t)1 <<  D5_GP);
const uint32_t D6_BIT_MASK  = ((uint32_t)1 <<  D6_GP);
const uint32_t D7_BIT_MASK  = ((uint32_t)1 <<  D7_GP);

const uint32_t DBUS_MASK    = ((uint32_t)1 << D0_GP) |
                              ((uint32_t)1 << D1_GP) |
                              ((uint32_t)1 << D2_GP) |
                              ((uint32_t)1 << D3_GP) |
                              ((uint32_t)1 << D4_GP) |
                              ((uint32_t)1 << D5_GP) |
                              ((uint32_t)1 << D6_GP) |
                              ((uint32_t)1 << D7_GP);

/* Input from Z80 indicating an I/O request is happening */
const uint8_t  IORQ_GP                  = 20;
const uint32_t IORQ_BIT_MASK            = ((uint32_t)1 << IORQ_GP);

const uint8_t  RD_GP                    = 21;
const uint32_t RD_BIT_MASK              = ((uint32_t)1 << RD_GP);

const uint8_t  WR_GP                    = 22;
const uint32_t WR_BIT_MASK              = ((uint32_t)1 << WR_GP);

/*
 * These are the bits which allow us to decide whether what is
 * on the Z80 buses is IF1 related
 */
const uint32_t IF1_IOPORT_ACCESS_BIT_MASK = IORQ_BIT_MASK |
                                            RD_BIT_MASK |
                                            WR_BIT_MASK |
                                            IF1_PORT_SIGNAL_BIT_MASK |
                                            A3_BIT_MASK;

const uint32_t PORT_E7_READ =  ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)0 << RD_GP)   |
                               ((uint32_t)1 << WR_GP)   |
                               ((uint32_t)0 << IF1_PORT_SIGNAL_GP)   |
                               ((uint32_t)0 << A3_GP);

const uint32_t PORT_EF_READ =  ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)0 << RD_GP)   |
                               ((uint32_t)1 << WR_GP)   |
                               ((uint32_t)0 << IF1_PORT_SIGNAL_GP)   |
                               ((uint32_t)1 << A3_GP);

const uint32_t PORT_E7_WRITE = ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)1 << RD_GP)   |
                               ((uint32_t)0 << WR_GP)   |
                               ((uint32_t)0 << IF1_PORT_SIGNAL_GP)   |
                               ((uint32_t)0 << A3_GP);

const uint32_t PORT_EF_WRITE = ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)1 << RD_GP)   |
                               ((uint32_t)0 << WR_GP)   |
                               ((uint32_t)0 << IF1_PORT_SIGNAL_GP)   |
                               ((uint32_t)1 << A3_GP);

/*
 * ROM read logic input, goes 0 when the MREQ to ROM is happening.
 * This is configured as input from the PIO code which is where it's
 * used. It's not used in this C.
 */
const uint8_t  ROM_READ_GP              = 17;
const uint32_t ROM_READ_BIT_MASK        = ((uint32_t)1 << ROM_READ_GP);

/*
 * Data bus level shifter direction pin, 1 is zx->pico, which is the normal
 * position, 0 means pico->zx which is temporarily switched to when the
 * Pico wants to send a data byte back to the Spectrum. This is now unused
 * in this code because the PIO does it, see mreq_dir.pio
 */
const uint8_t  DIR_OUTPUT_GP            = 28;

/*
 * This one's attached to the Z80's /WAIT line, open collector,
 * pull low to set the Z80 waiting
 */
const uint8_t  WAIT_GP                  = 27;

/* Test pin */
const uint8_t  TEST_OUTPUT_GP           = 10;

static void __time_critical_func(blip_test_pin)( void )
{
  gpio_put( TEST_OUTPUT_GP, 1 );
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
  gpio_put( TEST_OUTPUT_GP, 0 );
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
}

/*
 * Rudimentary trace table for whole program
 *
 * (gdb) set print repeats 0
 * (gdb) set print elements unlimited
 * (gdb) set pagination off
 * (gdb) set max-value-size unlimited
 * (gdb) p trace_table
 */
TRACE_TYPE trace_table[NUM_TRACE_ENTRIES];
uint8_t  trace_active=1;
uint32_t trace_index=0;

void trace( TRACE_CODE code, uint8_t data )
{
  if( trace_active )
  {
    trace_table[trace_index].i=trace_index;
    trace_table[trace_index].code=code;
    trace_table[trace_index].data=data;

    if( ++trace_index == NUM_TRACE_ENTRIES )
      trace_index=0;
  }
}

/* Not sure these are much use */
void trace_on( uint8_t data )
{
  trace_active = 1;
  trace(TRC_TRC_ON, data);
}
void trace_off( uint8_t data )
{
  trace(TRC_TRC_OFF, data);
  trace_active = 0;
}



/*
 * This PIO state machine is used to change the data bus level
 * shifter direction when either the ROM handling Pico says it
 * wants to send a byte back to the ZX, or when this code wants
 * to send a byte from the IF1 back to the ZX.
 */
PIO pio;
uint sm_mreq;


/*
 * I/O port handling runs in the second core.
 */
void __time_critical_func(core1_main)( void )
{
  /* All interrupts off in this core, the IO emulation won't work with interrupts enabled */
  irq_set_mask_enabled( 0xFFFFFFFF, 0 );  

  trace(TRC_INTS_OFF, 0);

  if( if1_init() == -1 )
  {
    /* Slow blink means not enough memory for the in-RAM microdrive image */
    while(1)
    {
      gpio_put(LED_PIN, 1);
      busy_wait_us_32(250000);
      gpio_put(LED_PIN, 0);
      busy_wait_us_32(250000);
    }
  }

  trace(TRC_IF1_INIT, 0);

  /* Insert the test images into the Microdrives */
  if( (if1_mdr_insert( 0 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 2 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 3 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 4 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 5 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 6 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 7 ) != LIBSPECTRUM_ERROR_NONE) )
  {
    while(1)
    {
      gpio_put(LED_PIN, 1);
      busy_wait_us_32(25000);
      gpio_put(LED_PIN, 0);
      busy_wait_us_32(25000);
    }
  }

  trace(TRC_IMAGES_INIT, 0);

  /* Tracing off, I'll turn it back on if I get a bug where it's needed */
  trace_active=0;

  while( 1 )
  {
    register uint32_t gpios_state = gpio_get_all();

    if( (gpios_state & IF1_IOPORT_ACCESS_BIT_MASK) == PORT_E7_WRITE )
    {
      /* Z80 write (OUT instruction) to port 0xE7 (231), microdrive data */

      /* Pick up the pattern of bits from the data bus GPIOs */
      register uint32_t z80_written_byte = (gpios_state & DBUS_MASK);

      /* Set Z80 waiting */
      gpio_set_dir(WAIT_GP, GPIO_OUT);
      gpio_put(WAIT_GP, 0);

      // trace(TRC_WRITE_E7_DATA, z80_written_byte);

      /* Write the byte out */
      port_mdr_out( z80_written_byte );

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );
    }

    else if( (gpios_state & IF1_IOPORT_ACCESS_BIT_MASK) == PORT_EF_WRITE )
    {
      /* Z80 write (OUT instruction) to port 0xEF (239), microdrive control */
      /* This turns on the motor */

      /* Pick up the pattern of bits from the data bus GPIOs */
      register uint32_t control_byte = (gpios_state & DBUS_MASK);
    
      /* Set Z80 waiting */
      gpio_set_dir(WAIT_GP, GPIO_OUT);
      gpio_put(WAIT_GP, 0);

      // trace(TRC_WRITE_EF_CONTROL, control_byte);

      /* Microdrive control, motor switch */
      port_ctr_out( control_byte );

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );
    }

    else if( (gpios_state & IF1_IOPORT_ACCESS_BIT_MASK) == PORT_E7_READ )
    {
      /* Z80 read from port 0xE7 (231), Z80 wants a microdrive data byte */

      /* Set Z80 waiting */
      gpio_set_dir(WAIT_GP, GPIO_OUT);
      gpio_put(WAIT_GP, 0);

      /* Direction needs to be Pico->ZX */
      pio_sm_put( pio, sm_mreq, 1 );

      /* Make data bus GPIOs outputs, pointed at the ZX */
      gpio_set_dir_out_masked( DBUS_MASK );

      /* Port handling function returns the data from the microdrive */
      register uint32_t md_data = port_mdr_in();
      gpio_put_masked( DBUS_MASK, md_data );

      // trace(TRC_READ_E7_DATA, md_data);

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

      /* Make the GPIOs inputs again */
      gpio_set_dir_in_masked( DBUS_MASK );
	  
      /* Put level shifter direction back to ZX->Pico */
      pio_sm_put( pio, sm_mreq, 0 );
    }

    else if( (gpios_state & IF1_IOPORT_ACCESS_BIT_MASK) == PORT_EF_READ )
    {
      /* Z80 read from port 0xEF (239), microdrive status */

      /* Set Z80 waiting */
      gpio_set_dir(WAIT_GP, GPIO_OUT);
      gpio_put(WAIT_GP, 0);

      /* Direction needs to be Pico->ZX */
      pio_sm_put( pio, sm_mreq, 1 );

      /* Make data bus GPIOs outputs, pointed at the ZX */
      gpio_set_dir_out_masked( DBUS_MASK );

      /* Port handling function returns the status */
      register uint32_t md_status = port_ctr_in();
      gpio_put_masked( DBUS_MASK, md_status );

      // trace(TRC_READ_EF_STATUS, md_status);

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

      /* Make the GPIOs inputs again */
      gpio_set_dir_in_masked( DBUS_MASK );
	  
      /* Put level shifter direction back to ZX->Pico */
      pio_sm_put( pio, sm_mreq, 0 );
    }
  } 
}


int __time_critical_func(main)( void )
{
  bi_decl(bi_program_description("ZX Spectrum Pico IF1 board binary."));

  /* Clean out the trace table */
  memset( trace_table, 0, sizeof(trace_table) );

  /* All interrupts off */
  irq_set_mask_enabled( 0xFFFFFFFF, 0 );  

#ifdef OVERCLOCK
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  trace(TRC_DATA_CONV,0 );

  /* Pull the buses to zeroes. Most of the address bus info comes from the ROM Pico */
  gpio_init( IF1_PORT_SIGNAL_GP  ); gpio_set_dir( IF1_PORT_SIGNAL_GP,  GPIO_IN );
  gpio_init( A3_GP  ); gpio_set_dir( A3_GP,  GPIO_IN );  gpio_pull_down( A3_GP  );

  gpio_init( D0_GP  ); gpio_set_dir( D0_GP,  GPIO_IN );  gpio_pull_down( D0_GP  );
  gpio_init( D1_GP  ); gpio_set_dir( D1_GP,  GPIO_IN );  gpio_pull_down( D1_GP  );
  gpio_init( D2_GP  ); gpio_set_dir( D2_GP,  GPIO_IN );  gpio_pull_down( D2_GP  );
  gpio_init( D3_GP  ); gpio_set_dir( D3_GP,  GPIO_IN );  gpio_pull_down( D3_GP  );
  gpio_init( D4_GP  ); gpio_set_dir( D4_GP,  GPIO_IN );  gpio_pull_down( D4_GP  );
  gpio_init( D5_GP  ); gpio_set_dir( D5_GP,  GPIO_IN );  gpio_pull_down( D5_GP  );
  gpio_init( D6_GP  ); gpio_set_dir( D6_GP,  GPIO_IN );  gpio_pull_down( D6_GP  );
  gpio_init( D7_GP  ); gpio_set_dir( D7_GP,  GPIO_IN );  gpio_pull_down( D7_GP  );

  gpio_init( IORQ_GP ); gpio_set_dir( IORQ_GP, GPIO_IN );
  gpio_pull_up( IORQ_GP );

  gpio_init( RD_GP ); gpio_set_dir( RD_GP, GPIO_IN );
  gpio_pull_up( RD_GP );

  gpio_init( WR_GP ); gpio_set_dir( WR_GP, GPIO_IN );
  gpio_pull_up( WR_GP );

  /*
   * Wait is an output from here to the Z80. Set as an input to sink the +5V
   * that's on it when it's not being used.
   */
  gpio_init( WAIT_GP ); gpio_set_dir( WAIT_GP, GPIO_IN );

  gpio_init( LED_PIN ); gpio_set_dir( LED_PIN, GPIO_OUT );

  trace(TRC_GPIOS_INIT ,0);

  /* Use PIO to switch the level shifter's DIRection */
  pio              = pio0;
  sm_mreq          = pio_claim_unused_sm( pio, true );
  uint offset_mreq = pio_add_program( pio, &mreq_dir_program );
  mreq_dir_program_init( pio, sm_mreq, offset_mreq, ROM_READ_GP, DIR_OUTPUT_GP );
  pio_sm_set_enabled(pio, sm_mreq, true);

  trace(TRC_PIOS_INIT, 0);

  /*
   * Enable SPI 0 at <n> MHz and connect to GPIOs. Second param is a baudrate, giving it
   * a frequency like this seems rather silly. You get what the hardware can give you.
   * Might as well ask for the theoretical maximum though.
   */
  spi_init(PSRAM_SPI, 62 * 1000 * 1000);
  gpio_set_function(PSRAM_SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PSRAM_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PSRAM_SPI_TX_PIN, GPIO_FUNC_SPI);

  /*
   * With reference to this thread:
   *  https://forums.raspberrypi.com//viewtopic.php?f=145&t=300589
   * this PSRAM device likes its slave-select line low across the
   * whole transaction (not high and low, byte to byte). It's
   * therefore controlled with software (the SPI hardware doing
   * things differently).
   */
  gpio_init(PSRAM_SPI_CSN_PIN);
  gpio_set_dir(PSRAM_SPI_CSN_PIN, GPIO_OUT);
  gpio_put(PSRAM_SPI_CSN_PIN, 1);

  /* Datasheet says 150uS between power up and the reset command */
  busy_wait_us_32(200);

  /* All examples I've seen don't bother with this reset, might be optional */
  uint8_t reset_cmd[] = { PSRAM_CMD_RESET_ENABLE, 
			  PSRAM_CMD_RESET };
  gpio_put(PSRAM_SPI_CSN_PIN, 0); 
  spi_write_blocking(PSRAM_SPI, reset_cmd, 2);
  gpio_put(PSRAM_SPI_CSN_PIN, 1);   

  /* Test SPI RAM is present */
  gpio_put(PSRAM_SPI_CSN_PIN, 0);

  /* Read ID, on the chip I'm using takes 0x9F as the command followed by 3 "don't care"s */
  uint8_t read_cmd[] = { PSRAM_CMD_READ_ID,
			 0, 0, 0 };
  spi_write_blocking(PSRAM_SPI, read_cmd, sizeof(read_cmd)); 
			
  /* Chip I'm using returns 0x0D, 0x5D according to the datasheet */
  uint8_t id1, id2;
  spi_read_blocking(PSRAM_SPI, 0, &id1, 1 ); 
  spi_read_blocking(PSRAM_SPI, 0, &id2, 1 ); 
  gpio_put(PSRAM_SPI_CSN_PIN, 1);

  if( (id1 != 0x0D) || (id2 != 0x5D) )
  {
    while(1)
    {
      gpio_put(LED_PIN, 1);
      busy_wait_us_32(1000000);
      gpio_put(LED_PIN, 0);
      busy_wait_us_32(100000);
    }
  }

  trace(TRC_SPI_INIT, 0);

  /* Enable SPI1 as slave from the UI Pico */
  spi_init(IO_FROM_UI_SPI, UI_TO_IO_SPI_SPEED);
  spi_set_slave(IO_FROM_UI_SPI, true);
  gpio_set_function(IO_FROM_UI_SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(IO_FROM_UI_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(IO_FROM_UI_SPI_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(IO_FROM_UI_SPI_CSN_PIN, GPIO_FUNC_SPI);

  gpio_init(TEST_OUTPUT_GP); gpio_set_dir(TEST_OUTPUT_GP, GPIO_OUT);
  gpio_put(TEST_OUTPUT_GP, 0);

  /*
   * For some reason the second core code doesn't get started after SWD programming
   * unless I pause for a moment here
   */
  busy_wait_us_32(100000);

  /* Init complete, run 2nd core code */
  multicore_launch_core1( core1_main ); 

  trace(TRC_CORE1_INIT, 0);

  /* This core responds to commands from the User Interface Pico */
  while( 1 )
  {
    /* Wait for IU Pico to say something */
    while( !spi_is_readable( IO_FROM_UI_SPI ) );

    UI_TO_IO_CMD cmd;

    /* Read the command byte from the UI Pico */
    spi_read_blocking( IO_FROM_UI_SPI, 0, &cmd, sizeof(UI_TO_IO_CMD) );

    switch( cmd )
    {
    case 0:
gpio_put( TEST_OUTPUT_GP, 1 );
    /* Read the command byte from the UI Pico */
uint8_t data[29];
    spi_read_blocking( IO_FROM_UI_SPI, 0, data, 29 );
gpio_put( TEST_OUTPUT_GP, 0 );
for(int i=1; i<=29; i++ )
{
  if( data[i-1] != i )
    gpio_put(LED_PIN, 1);
  
}
      break;

    case UI_TO_IO_TEST_LED_ON:
      gpio_put(LED_PIN, 1);
      break;
	
    case UI_TO_IO_TEST_LED_OFF:
      gpio_put(LED_PIN, 0);
      break;

    default:
      break;
    }

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
