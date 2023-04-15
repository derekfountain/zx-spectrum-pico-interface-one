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
 * These pin values are the GPxx ones in green background on the pinout diagram.
 * See schematic for how the signals are fed into the Pico's GPIOs
 */
const uint8_t A0_GP          = 11;
const uint8_t A1_GP          = 12;
const uint8_t A2_GP          = 13;
const uint8_t A3_GP          = 14;
const uint8_t A4_GP          = 20;
const uint8_t A5_GP          = 21;
const uint8_t A6_GP          = 22;
const uint8_t A7_GP          = 26;

const uint32_t  A0_BIT_MASK  = ((uint32_t)1 <<  A0_GP);
const uint32_t  A1_BIT_MASK  = ((uint32_t)1 <<  A1_GP);
const uint32_t  A2_BIT_MASK  = ((uint32_t)1 <<  A2_GP);
const uint32_t  A3_BIT_MASK  = ((uint32_t)1 <<  A3_GP);
const uint32_t  A4_BIT_MASK  = ((uint32_t)1 <<  A4_GP);
const uint32_t  A5_BIT_MASK  = ((uint32_t)1 <<  A5_GP);
const uint32_t  A6_BIT_MASK  = ((uint32_t)1 <<  A6_GP);
const uint32_t  A7_BIT_MASK  = ((uint32_t)1 <<  A7_GP);

/*
 * Given value i, this calculates the pattern of the GPIOs if
 * value i were to appear on the Z80 address bus.
 */
uint32_t create_gpios_for_address( uint32_t i )
{
  uint32_t gpio_pattern = 
    ( ((i & 0x0001) >>  0) <<  A0_GP ) |
    ( ((i & 0x0002) >>  1) <<  A1_GP ) |
    ( ((i & 0x0004) >>  2) <<  A2_GP ) |
    ( ((i & 0x0008) >>  3) <<  A3_GP ) |
    ( ((i & 0x0010) >>  4) <<  A4_GP ) |
    ( ((i & 0x0020) >>  5) <<  A5_GP ) |
    ( ((i & 0x0040) >>  6) <<  A6_GP ) |
    ( ((i & 0x0080) >>  7) <<  A7_GP );

  return gpio_pattern;
}


const uint8_t  D0_GP          = 0;
const uint8_t  D1_GP          = 1;
const uint8_t  D2_GP          = 2;
const uint8_t  D3_GP          = 4; 
const uint8_t  D4_GP          = 6;
const uint8_t  D5_GP          = 3;
const uint8_t  D6_GP          = 5;
const uint8_t  D7_GP          = 7;

const uint32_t  D0_BIT_MASK  = ((uint32_t)1 <<  D0_GP);
const uint32_t  D1_BIT_MASK  = ((uint32_t)1 <<  D1_GP);
const uint32_t  D2_BIT_MASK  = ((uint32_t)1 <<  D2_GP);
const uint32_t  D3_BIT_MASK  = ((uint32_t)1 <<  D3_GP);
const uint32_t  D4_BIT_MASK  = ((uint32_t)1 <<  D4_GP);
const uint32_t  D5_BIT_MASK  = ((uint32_t)1 <<  D5_GP);
const uint32_t  D6_BIT_MASK  = ((uint32_t)1 <<  D6_GP);
const uint32_t  D7_BIT_MASK  = ((uint32_t)1 <<  D7_GP);

const uint32_t DBUS_MASK     = ((uint32_t)1 << D0_GP) |
                               ((uint32_t)1 << D1_GP) |
                               ((uint32_t)1 << D2_GP) |
                               ((uint32_t)1 << D3_GP) |
                               ((uint32_t)1 << D4_GP) |
                               ((uint32_t)1 << D5_GP) |
                               ((uint32_t)1 << D6_GP) |
                               ((uint32_t)1 << D7_GP);

/* Input from Z80 indicating an I/O request is happening */
const uint8_t  IORQ_GP                  = 8;
const uint32_t IORQ_BIT_MASK            = ((uint32_t)1 << IORQ_GP);

const uint8_t  RD_GP                    = 9;
const uint32_t RD_BIT_MASK              = ((uint32_t)1 << RD_GP);

const uint8_t  WR_GP                    = 10;
const uint32_t WR_BIT_MASK              = ((uint32_t)1 << WR_GP);

const uint32_t IF1_IOPORT_ACCESS_BIT_MASK = IORQ_BIT_MASK |
                                            RD_BIT_MASK |
                                            WR_BIT_MASK |
                                            A0_BIT_MASK |
                                            A1_BIT_MASK |
                                            A2_BIT_MASK |
                                            A3_BIT_MASK |
                                            A4_BIT_MASK |
                                            A5_BIT_MASK |
                                            A6_BIT_MASK |
                                            A7_BIT_MASK;

const uint32_t PORT_E7_READ =  ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)0 << RD_GP)   |
                               ((uint32_t)1 << WR_GP)   |
                               ((uint32_t)1 << A0_GP)   |
                               ((uint32_t)1 << A1_GP)   |
                               ((uint32_t)1 << A2_GP)   |
                               ((uint32_t)0 << A3_GP)   |
                               ((uint32_t)0 << A4_GP)   |
                               ((uint32_t)1 << A5_GP)   |
                               ((uint32_t)1 << A6_GP)   |
                               ((uint32_t)1 << A7_GP);

const uint32_t PORT_EF_READ =  ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)0 << RD_GP)   |
                               ((uint32_t)1 << WR_GP)   |
                               ((uint32_t)1 << A0_GP)   |
                               ((uint32_t)1 << A1_GP)   |
                               ((uint32_t)1 << A2_GP)   |
                               ((uint32_t)1 << A3_GP)   |
                               ((uint32_t)0 << A4_GP)   |
                               ((uint32_t)1 << A5_GP)   |
                               ((uint32_t)1 << A6_GP)   |
                               ((uint32_t)1 << A7_GP);

const uint32_t PORT_E7_WRITE = ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)1 << RD_GP)   |
                               ((uint32_t)0 << WR_GP)   |
                               ((uint32_t)1 << A0_GP)   |
                               ((uint32_t)1 << A1_GP)   |
                               ((uint32_t)1 << A2_GP)   |
                               ((uint32_t)0 << A3_GP)   |
                               ((uint32_t)0 << A4_GP)   |
                               ((uint32_t)1 << A5_GP)   |
                               ((uint32_t)1 << A6_GP)   |
                               ((uint32_t)1 << A7_GP);

const uint32_t PORT_EF_WRITE = ((uint32_t)0 << IORQ_GP) |
                               ((uint32_t)1 << RD_GP)   |
                               ((uint32_t)0 << WR_GP)   |
                               ((uint32_t)1 << A0_GP)   |
                               ((uint32_t)1 << A1_GP)   |
                               ((uint32_t)1 << A2_GP)   |
                               ((uint32_t)1 << A3_GP)   |
                               ((uint32_t)0 << A4_GP)   |
                               ((uint32_t)1 << A5_GP)   |
                               ((uint32_t)1 << A6_GP)   |
                               ((uint32_t)1 << A7_GP);

/*
 * ROM read logic input, goes 0 when the MREQ to ROM is happening.
 * This is configured as input from the PIO code which is where it's
 * used. It's not used in this C.
 */
const uint8_t  ROM_READ_GP              = 27;
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
const uint8_t  WAIT_GP                  = 15;

/* Test pin is one of the UART pins for now because there's a test point */
// No GPIOs left const uint8_t  TEST_OUTPUT_GP           = 17;

/* Rudimentary trace table for whole program */
TRACE_TYPE trace[256];
uint8_t trace_index=0;     /* 8 bit index, auto-wraps */

/*
 * The 8 address bus bits arrive on the GPIOs in a weird pattern which is
 * defined by the edge connector layout and the board design. See the
 * ROM code for the description of this.
 *
 * Here it's an 8 bit IO address, so a smaller table than in the ROM code.
 */
uint16_t address_indirection_table[ 256 ];

/*
 * Given the GPIOs with an address bus value on them, this packs the
 * 14 address bits down into the least significant 14 bits. Only the
 * lower 8 are used in this code.
 */
inline uint16_t pack_address_gpios( uint32_t gpios )
{
  /*     Bits 0,1,2,3,4,5       Bits 6,7,8,9,10,11,12     Bit 13                   */
  return ((gpios>>9) & 0x03F) | ((gpios>>10) & 0x1FC0) | ((gpios & 0x4000000) >> 13);
}

/*
 * Populate the address bus indirection table.
 */
void create_indirection_table( void )
{
  uint32_t i;

  for( i=0; i<256; i++ )
  {
    uint32_t raw_bit_pattern = create_gpios_for_address( i );

    uint32_t packed_bit_pattern = pack_address_gpios( raw_bit_pattern );

    address_indirection_table[packed_bit_pattern] = i;
  }

  return;
}


/*
 * Instead of shuffling the bits of an output value around at the point
 * they need to be sent back to the Z80, preconvert them and put them
 * in a table. The table is indexed 0-255, and each entry holds the
 * index value shuffled to match the GPIOs of the data bus.
 */
uint8_t preconverted_data[256];
void preconvert_data( void )
{
  uint16_t conv_index;
  for( conv_index=0; conv_index < 256; conv_index++ )
  {
    /* Convert the number to the pattern the GPIOs show when that number is on them */
    preconverted_data[conv_index] = 
                                   (conv_index & 0x87)       |        /* bxxx xbbb */
                                  ((conv_index & 0x08) << 1) |        /* xxxb xxxx */
                                  ((conv_index & 0x10) << 2) |        /* xbxx xxxx */
                                  ((conv_index & 0x20) >> 2) |        /* xxxx bxxx */
                                  ((conv_index & 0x40) >> 1);         /* xxbx xxxx */
  }
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
 * The software doesn't work with interrupts on. stdio doesn't work
 * with interrupts off. This is only of occasional use.
 */
#define STDIO_ENABLED 0

#define CORE1_IN_USE 1
#if CORE1_IN_USE

#define MESSAGE_LEN   32
uint8_t message[MESSAGE_LEN];

void __time_critical_func(core1_main)( void )
{
  /* All interrupts off in this core, the IO emulation won't work with interrupts enabled */
  irq_set_mask_enabled( 0xFFFFFFFF, 0 );  

  TRACE(TRC_INTS_OFF);

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

  TRACE(TRC_IF1_INIT);

  /* Insert the test images into the Microdrives */
  if( (if1_mdr_insert( 0, 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 1, 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 2, 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 3, 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 4, 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 5, 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 6, 1 ) != LIBSPECTRUM_ERROR_NONE) ||
      (if1_mdr_insert( 7, 1 ) != LIBSPECTRUM_ERROR_NONE) )
  {
    while(1)
    {
      gpio_put(LED_PIN, 1);
      busy_wait_us_32(25000);
      gpio_put(LED_PIN, 0);
      busy_wait_us_32(25000);
    }
  }

  TRACE(TRC_IMAGES_INIT);

  while( 1 )
  {
    register uint32_t gpios_state = gpio_get_all();

    if( (gpios_state & IF1_IOPORT_ACCESS_BIT_MASK) == PORT_E7_WRITE )
    {
      /* Z80 write (OUT instruction) to port 0xE7 (231), microdrive data */

      /* Pick up the pattern of bits from the jumbled data bus GPIOs */
      register uint32_t raw_pattern = (gpios_state & DBUS_MASK);

      /* Set Z80 waiting */
      gpio_set_dir(WAIT_GP, GPIO_OUT);
      gpio_put(WAIT_GP, 0);

      /* Sort those bits out into the value which the Z80 originally wrote */
      register uint32_t z80_written_byte =  (raw_pattern & 0x87)       |        /* bxxx xbbb */
                                           ((raw_pattern & 0x08) << 2) |        /* xxbx xxxx */
                                           ((raw_pattern & 0x10) >> 1) |        /* xxxx bxxx */
                                           ((raw_pattern & 0x20) << 1) |        /* xbxx xxxx */
                                           ((raw_pattern & 0x40) >> 2);         /* xxxb xxxx */

      /* Write the byte out */
      port_mdr_out( z80_written_byte );

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

      TRACE_DATA(TRC_WRITE_E7_DATA, z80_written_byte);
    }

    else if( (gpios_state & IF1_IOPORT_ACCESS_BIT_MASK) == PORT_EF_WRITE )
    {
      /* Z80 write (OUT instruction) to port 0xEF (239), microdrive control */
      /* This turns on the motor */

      /* Pick up the pattern of bits from the jumbled data bus GPIOs */
      register uint32_t raw_pattern = (gpios_state & DBUS_MASK);

      /* Set Z80 waiting */
      gpio_set_dir(WAIT_GP, GPIO_OUT);
      gpio_put(WAIT_GP, 0);

      /* Sort those bits out into the value which the Z80 originally wrote */
      register uint32_t control_byte =  (raw_pattern & 0x87)       |        /* bxxx xbbb */
	                               ((raw_pattern & 0x08) << 2) |        /* xxbx xxxx */
                                       ((raw_pattern & 0x10) >> 1) |        /* xxxx bxxx */
		                       ((raw_pattern & 0x20) << 1) |        /* xbxx xxxx */
		                       ((raw_pattern & 0x40) >> 2);         /* xxxb xxxx */
      port_ctr_out( control_byte );

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

      TRACE_DATA(TRC_WRITE_EF_CONTROL, control_byte);
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

      /* Port handling function returns the data */
      register uint32_t md_data = preconverted_data[port_mdr_in()];
      gpio_put_masked( DBUS_MASK, md_data );

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

      /* Make the GPIOs inputs again */
      gpio_set_dir_in_masked( DBUS_MASK );
	  
      /* Put level shifter direction back to ZX->Pico */
      pio_sm_put( pio, sm_mreq, 0 );

      TRACE_DATA(TRC_READ_E7_DATA, md_data);
    }

    else if( (gpios_state & IF1_IOPORT_ACCESS_BIT_MASK) == PORT_EF_READ )
    {
      /* Z80 read from port 0xEF (239), microdrive status */

      /* Set Z80 waiting */
      gpio_set_dir(WAIT_GP, GPIO_OUT);
      gpio_put(WAIT_GP, 0);

#if 0
      strcpy( message, "PORT_EF_READ running\n" );
#endif

      /* Direction needs to be Pico->ZX */
      pio_sm_put( pio, sm_mreq, 1 );

      /* Make data bus GPIOs outputs, pointed at the ZX */
      gpio_set_dir_out_masked( DBUS_MASK );

      /* Port handling function returns the status */
      register uint32_t md_status = preconverted_data[port_ctr_in()];
      gpio_put_masked( DBUS_MASK, md_status );

      /* Done waiting */
      gpio_set_dir(WAIT_GP, GPIO_IN);

      /* Wait for the IO request to complete */
      while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

      /* Make the GPIOs inputs again */
      gpio_set_dir_in_masked( DBUS_MASK );
	  
      /* Put level shifter direction back to ZX->Pico */
      pio_sm_put( pio, sm_mreq, 0 );

      TRACE_DATA(TRC_READ_EF_STATUS, md_status);

#if 0
      strcpy( message, "PORT_EF_READ complete\n\n" );
#endif
    }
  } 
}
#endif


int __time_critical_func(main)( void )
{
  bi_decl(bi_program_description("ZX Spectrum Pico IF1 board binary."));

  TRACE(TRC_INIT);

#if STDIO_ENABLED
  stdio_init_all();
  printf("ZX Spectrum Pico IF1 board binary."); fflush(stdout);
#else  
  /* All interrupts off */
  irq_set_mask_enabled( 0xFFFFFFFF, 0 );  
#endif  

#ifdef OVERCLOCK
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  /* Build the preconverted data table */
  preconvert_data();

  /* Create address indirection table, this is the address bus optimisation  */
  create_indirection_table();

  TRACE(TRC_DATA_CONV);

  /* Pull the buses to zeroes */
  gpio_init( A0_GP  ); gpio_set_dir( A0_GP,  GPIO_IN );  gpio_pull_down( A0_GP  );
  gpio_init( A1_GP  ); gpio_set_dir( A1_GP,  GPIO_IN );  gpio_pull_down( A1_GP  );
  gpio_init( A2_GP  ); gpio_set_dir( A2_GP,  GPIO_IN );  gpio_pull_down( A2_GP  );
  gpio_init( A3_GP  ); gpio_set_dir( A3_GP,  GPIO_IN );  gpio_pull_down( A3_GP  );
  gpio_init( A4_GP  ); gpio_set_dir( A4_GP,  GPIO_IN );  gpio_pull_down( A4_GP  );
  gpio_init( A5_GP  ); gpio_set_dir( A5_GP,  GPIO_IN );  gpio_pull_down( A5_GP  );
  gpio_init( A6_GP  ); gpio_set_dir( A6_GP,  GPIO_IN );  gpio_pull_down( A6_GP  );
  gpio_init( A7_GP  ); gpio_set_dir( A7_GP,  GPIO_IN );  gpio_pull_down( A7_GP  );

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
   * Wait an output from here to the Z80. Set as an input to sink the +5V
   * that's on it when it's not being used.
   */
  gpio_init( WAIT_GP ); gpio_set_dir( WAIT_GP, GPIO_IN );

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  TRACE(TRC_GPIOS_INIT);

  /*
   * Enable SPI 0 at <n> MHz and connect to GPIOs. Second param is a baudrate, giving it
   * a frequency like this seems rather silly. You get what the hardware can give you.
   * Might as well ask for the theoretical maximum though.
   */
  spi_init(PICO_SPI, 62 * 1000 * 1000);
  gpio_set_function(PICO_SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_SPI_TX_PIN, GPIO_FUNC_SPI);

  /* Output for chip select on slave, starts high (unselected) */
  gpio_init(PICO_SPI_CSN_PIN);
  gpio_set_dir(PICO_SPI_CSN_PIN, GPIO_OUT);
  gpio_put(PICO_SPI_CSN_PIN, 1);

  /* Datasheet says 150uS between power up and the reset command */
  busy_wait_us_32(200);

  /* All examples I've seen don't bother with this reset, might be optional */
  uint8_t reset_cmd[] = { PRAM_CMD_RESET_ENABLE, 
			  PRAM_CMD_RESET };
  gpio_put(PICO_SPI_CSN_PIN, 0); 
  spi_write_blocking(PICO_SPI, reset_cmd, 2);
  gpio_put(PICO_SPI_CSN_PIN, 1);   

  /* Test SPI RAM is present */
  gpio_put(PICO_SPI_CSN_PIN, 0);

  /* Read ID, on the chip I'm using takes 0x9F as the command followed by 3 "don't care"s */
  uint8_t read_cmd[] = { PRAM_CMD_READ_ID,
			 0, 0, 0 };
  spi_write_blocking(PICO_SPI, read_cmd, sizeof(read_cmd)); 
			
  /* Chip I'm using returns 0x0D, 0x5D according to the datasheet */
  uint8_t id1, id2;
  spi_read_blocking(PICO_SPI, 0, &id1, 1 ); 
  spi_read_blocking(PICO_SPI, 0, &id2, 1 ); 
  gpio_put(PICO_SPI_CSN_PIN, 1);

  if( (id1 != 0x0D) || (id2 != 0x5D) )
  {
    while(1)
    {
      gpio_put(LED_PIN, 1);
      busy_wait_us_32(1000000);
      gpio_put(LED_PIN, 0);
      busy_wait_us_32(500000);
    }
  }

//  gpio_init(TEST_OUTPUT_GP); gpio_set_dir(TEST_OUTPUT_GP, GPIO_OUT);
//  gpio_put(TEST_OUTPUT_GP, 0);

  TRACE(TRC_SPI_INIT);

  /* Use PIO to switch the level shifter's DIRection */
  pio              = pio0;
  sm_mreq          = pio_claim_unused_sm( pio, true );
  uint offset_mreq = pio_add_program( pio, &mreq_dir_program );
  mreq_dir_program_init( pio, sm_mreq, offset_mreq, ROM_READ_GP, DIR_OUTPUT_GP );
  pio_sm_set_enabled(pio, sm_mreq, true);

  TRACE(TRC_PIOS_INIT);

#if CORE1_IN_USE
  /*
   * For some reason the second core code doesn't get started after SWD programming
   * unless I pause for a moment here
   */
  busy_wait_us_32(100000);

  /* Init complete, run 2nd core code */
  multicore_launch_core1( core1_main ); 

  TRACE(TRC_CORE1_INIT);
#endif

  memset( message, 0, MESSAGE_LEN );

  while( 1 )
  {
#if STDIO_ENABLED
    if( strchr( message, '\n' ) != NULL )
    {
      busy_wait_us_32(1000000);

      printf(message); fflush(stdout);
      while(1);
      memset( message, 0, MESSAGE_LEN );
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
