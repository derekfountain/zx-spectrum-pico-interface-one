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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/timer.h"


/* 1 instruction on the 133MHz microprocessor is 7.5ns */
/* 1 instruction on the 140MHz microprocessor is 7.1ns */
/* 1 instruction on the 150MHz microprocessor is 6.6ns */
/* 1 instruction on the 200MHz microprocessor is 5.0ns */

//#define OVERCLOCK 150000
#define OVERCLOCK 200000

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

const uint8_t  IORQ_GP                  = 8;
const uint32_t IORQ_BIT_MASK            = ((uint32_t)1 << IORQ_GP);

const uint8_t  RD_GP                    = 9;
const uint32_t RD_BIT_MASK              = ((uint32_t)1 << RD_GP);

const uint8_t  WR_GP                    = 10;
const uint32_t WR_BIT_MASK              = ((uint32_t)1 << WR_GP);


/* ROM read logic input, goes 0 when the MREQ to ROM is happening */
const uint8_t  ROM_READ_GP              = 27;
const uint32_t ROM_READ_BIT_MASK        = ((uint32_t)1 << ROM_READ_GP);

/*
 * Data bus leve shifter direction pin, 1 is zx->pico, which is the normal
 * position, 0 means pico->zx which is temporarily switched to when the
 * Pico wants to send a data byte back to the Spectrum
 */
const uint8_t  DIR_OUTPUT_GP            = 28;


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
 * 14 address bits down into the least significant 14 bits
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


void core1_main( void )
{
  while( 1 )
  {
    register uint32_t gpios_state = gpio_get_all();

    if( (gpios_state & ROM_READ_BIT_MASK) == 0 )
    {
      /* ROM memory read, the other Pico handles the details of this */

      /*
       * This Pico's responsibility is to switch the data bus level
       * switcher direction to Pico->ZX
       */
      gpio_put( DIR_OUTPUT_GP, 0 );

      /* Wait for the ROM request to complete */
      while( (gpio_get_all() & ROM_READ_BIT_MASK) == 0 );

      /* Put level shifter direction back to ZX->Pico */
      gpio_put( DIR_OUTPUT_GP, 1 );
    }

    else if( (gpios_state & IORQ_BIT_MASK) == 0 )
    {
      /* IO, this Pico does this for the IF1 ports */

      /* Sort out the port from the address bus */
      uint16_t io_address = ((gpios_state >>  A0_GP   ) & 0x01) |
                            ((gpios_state >> (A1_GP-1)) & 0x02) |
                            ((gpios_state >> (A2_GP-2)) & 0x04) |
                            ((gpios_state >> (A3_GP-3)) & 0x08) |
                            ((gpios_state >> (A4_GP-4)) & 0x10) |
                            ((gpios_state >> (A5_GP-5)) & 0x20) |
                            ((gpios_state >> (A6_GP-6)) & 0x40) |
                            ((gpios_state >> (A7_GP-7)) & 0x80);

      if( io_address == 223 )
      {

	/* It's one for us to worry about */

	if( (gpios_state & RD_BIT_MASK) == 0 )
	{
	  /* A Z80 read, an IN instruction. Load a byte onto the data bus */

	  /* Direction needs to be Pico->ZX */
	  gpio_put( DIR_OUTPUT_GP, 0 );

	  /* Make data bus GPIOs outputs, pointed at the ZX */
	  gpio_set_dir_out_masked( DBUS_MASK );

	  /*
	   * The other core will write out the data byte here
	   */

	  /* Wait for the IO request to complete */
	  while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

	  /* Make the GPIOs inputs again */
	  gpio_set_dir_in_masked( DBUS_MASK );

	  /* Put level shifter direction back to ZX->Pico */
	  gpio_put( DIR_OUTPUT_GP, 1 );
	}

      }
    }

  } /* Infinite loop */
}



int main()
{
  bi_decl(bi_program_description("ZX Spectrum Pico IF1 board binary."));

  /* All interrupts off */
  irq_set_mask_enabled( 0xFFFFFFFF, 0 );

#ifdef OVERCLOCK
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  /* Build the preconverted data table */
  preconvert_data();

  /* Create address indirection table, this is the address bus optimisation  */
  create_indirection_table();

  /* Pull the buses to zeroes */
  gpio_init( A0_GP  ); gpio_set_dir( A0_GP,  GPIO_IN );  gpio_pull_down( A0_GP  );
  gpio_init( A1_GP  ); gpio_set_dir( A1_GP,  GPIO_IN );  gpio_pull_down( A1_GP  );
  gpio_init( A2_GP  ); gpio_set_dir( A2_GP,  GPIO_IN );  gpio_pull_down( A2_GP  );
  gpio_init( A3_GP  ); gpio_set_dir( A3_GP,  GPIO_IN );  gpio_pull_down( A3_GP  );
  gpio_init( A4_GP  ); gpio_set_dir( A4_GP,  GPIO_IN );  gpio_pull_down( A4_GP  );
  gpio_init( A5_GP  ); gpio_set_dir( A5_GP,  GPIO_IN );  gpio_pull_down( A5_GP  );
  gpio_init( A6_GP  ); gpio_set_dir( A6_GP,  GPIO_IN );  gpio_pull_down( A6_GP  );
  gpio_init( A7_GP  ); gpio_set_dir( A7_GP,  GPIO_IN );  gpio_pull_down( A7_GP  );

  gpio_init( D0_GP  ); gpio_set_dir( D0_GP,  GPIO_IN );
  gpio_init( D1_GP  ); gpio_set_dir( D1_GP,  GPIO_IN );
  gpio_init( D2_GP  ); gpio_set_dir( D2_GP,  GPIO_IN );
  gpio_init( D3_GP  ); gpio_set_dir( D3_GP,  GPIO_IN );
  gpio_init( D4_GP  ); gpio_set_dir( D4_GP,  GPIO_IN );
  gpio_init( D5_GP  ); gpio_set_dir( D5_GP,  GPIO_IN );
  gpio_init( D6_GP  ); gpio_set_dir( D6_GP,  GPIO_IN );
  gpio_init( D7_GP  ); gpio_set_dir( D7_GP,  GPIO_IN );

  gpio_init( IORQ_GP ); gpio_set_dir( IORQ_GP, GPIO_IN );
  gpio_pull_up( IORQ_GP );

  gpio_init( RD_GP ); gpio_set_dir( RD_GP, GPIO_IN );
  gpio_pull_up( RD_GP );

  /* Input from ROM logic, 1 when MREQ isn't happening, 0 when it is */
  gpio_init( ROM_READ_GP ); gpio_set_dir( ROM_READ_GP, GPIO_IN );
  gpio_pull_up( ROM_READ_GP );

  /*
   * Output to databus level shifter DIRection pin. Normally 1 meaning
   * zx->pico, we assert 0 to switch it pico->zx to send back a response
   * to the Z80
   */
  gpio_init(DIR_OUTPUT_GP); gpio_set_dir(DIR_OUTPUT_GP, GPIO_OUT);
  gpio_put(DIR_OUTPUT_GP, 1);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  int signal;
  for( signal=0; signal<2; signal++ )
  {
    gpio_put(LED_PIN, 1);
    busy_wait_us_32(250000);
    gpio_put(LED_PIN, 0);
    busy_wait_us_32(250000);
  }

  /* 2nd core code */
  multicore_launch_core1( core1_main ); 

  uint8_t response_byte = 0;

  gpio_set_dir_in_masked( DBUS_MASK );
  while( 1 )
  {
    register uint32_t gpios_state = gpio_get_all();

    if( (gpios_state & IORQ_BIT_MASK) == 0 )
    {
      /* IO, this Pico does this for the IF1 ports */

      uint16_t io_address = ((gpios_state >>  A0_GP   ) & 0x01) |
	                    ((gpios_state >> (A1_GP-1)) & 0x02) |
                            ((gpios_state >> (A2_GP-2)) & 0x04) |
                            ((gpios_state >> (A3_GP-3)) & 0x08) |
                            ((gpios_state >> (A4_GP-4)) & 0x10) |
                            ((gpios_state >> (A5_GP-5)) & 0x20) |
                            ((gpios_state >> (A6_GP-6)) & 0x40) |
                            ((gpios_state >> (A7_GP-7)) & 0x80);

      if( io_address == 223 )
      {

	/* It's one for us to worry about */

	if( (gpios_state & RD_BIT_MASK) == 0 )
	{
	  /* A Z80 read, an IN instruction. Load a byte onto the data bus */

	  gpio_put_masked( DBUS_MASK, preconverted_data[response_byte] );

	}
	else if( (gpios_state & WR_BIT_MASK) == 0 )
	{
	  /* A Z80 write, an OUT instruction. Fetch the byte from the data bus */

	  /* Pick up the pattern of bits from the jumbled data bus GPIOs */
	  uint8_t raw_pattern = (gpios_state & DBUS_MASK) & 0xFF;

	  /* Sort those bits out into the value which the Z80 originally wrote */
	  uint8_t z80_written_byte =  (raw_pattern & 0x87)       |        /* bxxx xbbb */
                                     ((raw_pattern & 0x08) << 2) |        /* xxbx xxxx */
                                     ((raw_pattern & 0x10) >> 1) |        /* xxxx bxxx */
                                     ((raw_pattern & 0x20) << 1) |        /* xbxx xxxx */
                                     ((raw_pattern & 0x40) >> 2);         /* xxxb xxxx */

	  response_byte = z80_written_byte;

	}

	/* Wait for the IO request to complete */
	while( (gpio_get_all() & IORQ_BIT_MASK) == 0 );

      }
    }

  } /* Infinite loop */

}


#if 0



  while(1)
  {

    /*
     * Spin while the hardware is saying at least one of A14, A15 and MREQ is 1.
     * ROM_READ is active low - if it's 1 then the ROM is not being read.
     */
    while( (gpios_state=gpio_get_all()) & ROM_READ_BIT_MASK );

    register uint16_t raw_bit_pattern = pack_address_gpios( gpios_state );

    register uint16_t rom_address = address_indirection_table[raw_bit_pattern];

    register uint8_t rom_value = *(rom_image_ptr+rom_address);

    /*
     * The level shifter for the data bus lines is switched to Pico->ZX,
     * the Pico GPIOs are set to outputs and the value asserted for the
     * Z80 to collect
     */
    gpio_put(DIR_OUTPUT_GP, 0);
    gpio_set_dir_out_masked( DBUS_MASK );
    gpio_put_masked( DBUS_MASK, rom_value );

    /*
     * Spin until the Z80 releases MREQ indicating the read is complete.
     * ROM_READ is active low - if it's 0 then the ROM is still being read.
     */
    while( (gpio_get_all() & ROM_READ_BIT_MASK) == 0 );

    /*
     * Set the data bus GPIOs back to inputs, then switch the level shifter
     * for the data bus lines back to ZX->Pico
     */
    gpio_set_dir_in_masked( DBUS_MASK );
    gpio_put(DIR_OUTPUT_GP, 1);



  }

}



#if 0
/* Blip the result pin, shows on scope */
gpio_put( TEST_OUTPUT_GP, 1 ); busy_wait_us_32(5);
gpio_put( TEST_OUTPUT_GP, 0 ); busy_wait_us_32(5);

gpio_put( TEST_OUTPUT_GP, 1 ); busy_wait_us_32(1000);
gpio_put( TEST_OUTPUT_GP, 0 ); busy_wait_us_32(1000);
__asm volatile ("nop");
#endif

#endif
