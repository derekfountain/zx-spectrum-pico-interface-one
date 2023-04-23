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
 * sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -c "program ./zx_pico_if1_fw.elf verify reset exit"
 * sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg
 * gdb-multiarch ./zx_pico_if1_fw.elf
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
#include "hardware/timer.h"
#include "pico/multicore.h"


/* 1 instruction on the 133MHz microprocessor is 7.5ns */
/* 1 instruction on the 140MHz microprocessor is 7.1ns */
/* 1 instruction on the 150MHz microprocessor is 6.6ns */
/* 1 instruction on the 200MHz microprocessor is 5.0ns */

//#define OVERCLOCK 170000
#define OVERCLOCK 270000

#include "roms.h"

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
const uint8_t A8_GP          = 19;
const uint8_t A9_GP          = 18;
const uint8_t A10_GP         = 17;
const uint8_t A11_GP         = 16;
const uint8_t A12_GP         = 10;
const uint8_t A13_GP         = 9;

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
    ( ((i & 0x0080) >>  7) <<  A7_GP ) |
    ( ((i & 0x0100) >>  8) <<  A8_GP ) |
    ( ((i & 0x0200) >>  9) <<  A9_GP ) |
    ( ((i & 0x0400) >> 10) << A10_GP ) |
    ( ((i & 0x0800) >> 11) << A11_GP ) |
    ( ((i & 0x1000) >> 12) << A12_GP ) |
    ( ((i & 0x2000) >> 13) << A13_GP );

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

/* Input from the logic which merges A14, A15 and MREQ, goes active on a ROM read */
const uint8_t  ROM_READ_GP              = 8;
const uint32_t ROM_READ_BIT_MASK        = ((uint32_t)1 << ROM_READ_GP);

/* Z80's M1 signal, needs to be merged into the IF1 paging logic */
const uint8_t  M1_GP                    = 15;
const uint32_t M1_INPUT_BIT_MASK        = ((uint32_t)1 << M1_GP);

const uint8_t  IF1_PORT_ACTIVE_OUTPUT_GP = 27;

/* This pin triggers a transistor which shorts the Z80's /RESET to ground */
const uint8_t  PICO_RESET_Z80_GP        = 28;

const uint32_t DBUS_MASK     = ((uint32_t)1 << D0_GP) |
                               ((uint32_t)1 << D1_GP) |
                               ((uint32_t)1 << D2_GP) |
                               ((uint32_t)1 << D3_GP) |
                               ((uint32_t)1 << D4_GP) |
                               ((uint32_t)1 << D5_GP) |
                               ((uint32_t)1 << D6_GP) |
                               ((uint32_t)1 << D7_GP);

/*
 * The 14 address bus bits arrive on the GPIOs in a weird pattern which is
 * defined by the edge connector layout and the board design. Shifting all
 * 14 bits into place works, but it's slow, it needs all 14 masks and shifts
 * done each byte read from ROM. The Pico needs a significant overclock to
 * manage that.
 *
 * This is an optimisation. Take the 14 address bus bits, which are scattered
 * in the 32bit GPIO value, and shift them down into the lowest 14 bits of
 * a 16 bit word. They're not in order A0 to A13, they're in some weird order.
 * So this is a conversion table. The index into this is the weird 14 bit
 * value, the value at that offset into this table is the actual address bus
 * value the weird value represents.
 *
 * e.g. you might read the GPIOs, shuffle the 14 bits down and end with,
 * say, 0x032A. Look up entry 0x032A in this table and find, say, 0x0001.
 * That means when 0x32A appears on the mixed up address bus, the actual
 * address bus value the Z80 has passed in is 0x0001. It wants that byte
 * from the ROM.
 *
 * This table is filled in at the start; a lookup is done here for each
 * ROM byte read.
 */
uint16_t address_indirection_table[ 16384 ];


/*
 * The bits of the bytes in the ROM need shuffling around to match the
 * ordering of the D0-D7 bits on the output GPIOs. See the schematic.
 * Do this now so the pre-converted bytes can be put straight onto
 * the GPIOs at runtime.
 */
void preconvert_rom( uint8_t *image_ptr, uint32_t length )
{
  uint16_t conv_index;
  for( conv_index=0; conv_index < length; conv_index++ )
  {
    uint8_t rom_byte = *(image_ptr+conv_index);
    *(image_ptr+conv_index) =  (rom_byte & 0x87)       |        /* bxxx xbbb */
                              ((rom_byte & 0x08) << 1) |        /* xxxb xxxx */
                              ((rom_byte & 0x10) << 2) |        /* xbxx xxxx */
                              ((rom_byte & 0x20) >> 2) |        /* xxxx bxxx */
                              ((rom_byte & 0x40) >> 1);         /* xxbx xxxx */
  }
}

void preconvert_rom_image( uint8_t rom_index )
{
  preconvert_rom( cycle_roms[rom_index].rom_data, cycle_roms[rom_index].rom_size ); 
}

/*
 * Loop over all the ROM images in the header file and convert their bit
 * patterns to match the order of bits of the data bus. It's quicker to
 * preconvert these at the start than to fiddle the bits each read cycle.
 */
void preconvert_roms( void )
{
  uint8_t rom_index;
  for( rom_index = 0; rom_index < num_cycle_roms; rom_index++ )
  {
    preconvert_rom_image( rom_index );
  }  
}

/*
 * Populate the address bus indirection table.
 */
void create_indirection_table( void )
{
  uint32_t i;

  /*
   * Loop over all 16384 address the Z80 might ask for. For each one calculate
   * the pattern of the GPIOs when that value is on the address bus. Then pack
   * that pattern down into the lowest 14 bits. That's the value which is
   * found for each read byte, so the value at that offset into the table is
   * the original value which will match what the Z80's after.
   */
  for( i=0; i<16384; i++ )
  {
    uint32_t raw_bit_pattern = create_gpios_for_address( i );

    uint32_t packed_bit_pattern = pack_address_gpios( raw_bit_pattern );

    address_indirection_table[packed_bit_pattern] = i;
  }

  return;
}

uint8_t *rom_image_ptr = __ROMs_48_original_rom;

/*
 * This is called by an alarm function. It lets the Z80 run by pulling the
 * Pico's controlling GPIO low
 */
int64_t start_z80_alarm_func( alarm_id_t id, void *user_data )
{
  /*
   * At power up the Z80 starts running, and with the Pico not yet booted
   * and able to get the Z80 into reset, the Z80 will cheerfully run what
   * are random instructions found in the noise on the data bus. If that
   * takes it to one of the "page IF1 in" addresses, the IF1 will be paged
   * in. So just to be absolutely sure, when the Pico is ready and the code
   * comes through here to start the Z80, ensure that the 48K ROM is in
   * place.
   */
  rom_image_ptr = __ROMs_48_original_rom;
  gpio_put(LED_PIN, 0);

  /* Don't need the timers anymore, no interrupts now */
  irq_set_mask_enabled( 0xFFFFFFFF, 0 );

  gpio_put( PICO_RESET_Z80_GP, 0 );
  return 0;
}

/* Crank up the overclock to 270MHz if this is turned on */
#define RECORDING 0
#if RECORDING

uint8_t unconverted_byte[ 256 ];

/*
 * ROM bytes are stored converted into data bus GPIO order. To log
 * the byte being returned I need to convert it back to the value
 * in the ROM. This is fast path for the ROM emulation so it needs
 * to be quick. So store this table of pre-unconverted 8 bit bytes.
 */
void create_unconvert_table( void )
{
  uint32_t i;
  
  for( i=0; i<256; i++ )
  {
    unconverted_byte[i] = (i & 0x87)       |        /* bxxx xbbb */
                         ((i & 0x08) << 2) |        /* xxbx xxxx */
                         ((i & 0x10) >> 1) |        /* xxxx bxxx */
                         ((i & 0x20) << 1) |        /* xbxx xxxx */
                         ((i & 0x40) >> 2);         /* xxxb xxxx */
  }
}

typedef struct _recorder
{
  uint32_t gpios;
  uint16_t address;
  uint8_t  response;
}
RECORDER;

#define RECORDER_SIZE 20000
#define RECORDER_RESET ((uint16_t)65535)

int32_t recorder_index=0;

/*
 * Print in debugger with
 *
 * (gdb) set max-value-size unlimited
 * (gdb) print -array on -array-indexes -elements 16384 -- /x address_recorder
 *
 * where 16384 is recorder_index
 */
RECORDER address_recorder[ RECORDER_SIZE ];

#endif

void __time_critical_func(core1_main)( void )
{
  const uint32_t ABUS_MASK     =
    ((uint32_t)1 << A0_GP) |
    ((uint32_t)1 << A1_GP) |
    ((uint32_t)1 << A2_GP) |

    ((uint32_t)1 << A4_GP) |
    ((uint32_t)1 << A5_GP) |
    ((uint32_t)1 << A6_GP) |
    ((uint32_t)1 << A7_GP);

  const uint32_t IF1_PORT     =
    ((uint32_t)1 << A0_GP)   |
    ((uint32_t)1 << A1_GP)   |
    ((uint32_t)1 << A2_GP)   |

    ((uint32_t)0 << A4_GP)   |
    ((uint32_t)1 << A5_GP)   |
    ((uint32_t)1 << A6_GP)   |
    ((uint32_t)1 << A7_GP);

  /*
   * This is a basic loop which sets a GPIO low if the pattern
   * 1110x111 is on the lower address bus. That signals to the
   * IO Pico that the Z80 is addressing something that might
   * be one of the IF1's IO ports. The other Pico worries about
   * that extra bit, and the IORQ line, etc.
   * This is crude, but fast enough at 150MHz. It'd be good if I
   * could replace this with a PIO program. That would free up
   * this core again.
   */
  while(1)
  {
    register uint32_t gpios_state = gpio_get_all();

    /* If we're currently dealing with a ROM read then it's not an IO */
    if( (gpios_state & ROM_READ_BIT_MASK) == 0 )
    {
      gpio_put( IF1_PORT_ACTIVE_OUTPUT_GP, 1 );
      continue;
    }


    if( (gpios_state & ABUS_MASK) == IF1_PORT )
    {
      /* It's one of ours, signal to the other Pico */
      gpio_put( IF1_PORT_ACTIVE_OUTPUT_GP, 0 );
    }
    else
    {
      /* It's not one of ours, turn the signal off */
      gpio_put( IF1_PORT_ACTIVE_OUTPUT_GP, 1 );
    }
  }
}

static int _1c05_visits = 0;
static int _13e3_visits = 0;
static int _1c0a_visits = 0;
static int _1c16_visits = 0;
static int _169d_visits = 0;
static int _16a4_visits = 0;
static int _16a5_visits = 0;
static int _16ac_visits = 0;
int main()
{
  bi_decl(bi_program_description("ZX Spectrum Pico IF1 board binary."));

#ifdef OVERCLOCK
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  /*
   * Set up Pico's Z80 reset pin, hold this at 0 to let Z80 run.
   * Set and hold 1 here to hold Spectrum in reset at startup until we're good
   * to provide its ROM
   */
  gpio_init( PICO_RESET_Z80_GP );  gpio_set_dir( PICO_RESET_Z80_GP, GPIO_OUT );
  gpio_put( PICO_RESET_Z80_GP, 1 );

  /* All interrupts off except the timers */
  irq_set_mask_enabled( 0xFFFFFFFF, 0 );
  irq_set_mask_enabled( 0x0000000F, 1 );

  /* Create address indirection table, this is the address bus optimisation  */
  create_indirection_table();

  /* Switch the bits in the ROM bytes around, this is the data bus optimisation */
  preconvert_roms();

#if RECORDING
  create_unconvert_table();
#endif

  /* Pull the buses to zeroes */
  gpio_init( A0_GP  ); gpio_set_dir( A0_GP,  GPIO_IN );  gpio_pull_down( A0_GP  );
  gpio_init( A1_GP  ); gpio_set_dir( A1_GP,  GPIO_IN );  gpio_pull_down( A1_GP  );
  gpio_init( A2_GP  ); gpio_set_dir( A2_GP,  GPIO_IN );  gpio_pull_down( A2_GP  );
  gpio_init( A3_GP  ); gpio_set_dir( A3_GP,  GPIO_IN );  gpio_pull_down( A3_GP  );
  gpio_init( A4_GP  ); gpio_set_dir( A4_GP,  GPIO_IN );  gpio_pull_down( A4_GP  );
  gpio_init( A5_GP  ); gpio_set_dir( A5_GP,  GPIO_IN );  gpio_pull_down( A5_GP  );
  gpio_init( A6_GP  ); gpio_set_dir( A6_GP,  GPIO_IN );  gpio_pull_down( A6_GP  );
  gpio_init( A7_GP  ); gpio_set_dir( A7_GP,  GPIO_IN );  gpio_pull_down( A7_GP  );
  gpio_init( A8_GP  ); gpio_set_dir( A8_GP,  GPIO_IN );  gpio_pull_down( A8_GP  );
  gpio_init( A9_GP  ); gpio_set_dir( A9_GP,  GPIO_IN );  gpio_pull_down( A9_GP  );
  gpio_init( A10_GP ); gpio_set_dir( A10_GP, GPIO_IN );  gpio_pull_down( A10_GP );
  gpio_init( A11_GP ); gpio_set_dir( A11_GP, GPIO_IN );  gpio_pull_down( A11_GP );
  gpio_init( A12_GP ); gpio_set_dir( A12_GP, GPIO_IN );  gpio_pull_down( A12_GP );
  gpio_init( A13_GP ); gpio_set_dir( A13_GP, GPIO_IN );  gpio_pull_down( A13_GP );

  gpio_init( D0_GP  ); gpio_set_dir( D0_GP,  GPIO_IN );
  gpio_init( D1_GP  ); gpio_set_dir( D1_GP,  GPIO_IN );
  gpio_init( D2_GP  ); gpio_set_dir( D2_GP,  GPIO_IN );
  gpio_init( D3_GP  ); gpio_set_dir( D3_GP,  GPIO_IN );
  gpio_init( D4_GP  ); gpio_set_dir( D4_GP,  GPIO_IN );
  gpio_init( D5_GP  ); gpio_set_dir( D5_GP,  GPIO_IN );
  gpio_init( D6_GP  ); gpio_set_dir( D6_GP,  GPIO_IN );
  gpio_init( D7_GP  ); gpio_set_dir( D7_GP,  GPIO_IN );

  /* Input from logic hardware, indicates the ROM is being read by the Z80 */
  gpio_init( ROM_READ_GP ); gpio_set_dir( ROM_READ_GP, GPIO_IN );
  gpio_pull_down( ROM_READ_GP );

  /* M1 signal input */
  gpio_init( M1_GP ); gpio_set_dir( M1_GP, GPIO_IN );
  gpio_pull_up( M1_GP );

  /* Blip LED to show we're running */
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
  gpio_put(LED_PIN, 1);

  /*
   * Ready to go, give it a few milliseconds for this Pico code to get into
   * its main loop, then let the Z80 start
   */
  add_alarm_in_ms( 5, start_z80_alarm_func, NULL, 0 );

  /* Signal to the IO Pico that an IF1 port is on A0 to A7 (active low) */
  gpio_init( IF1_PORT_ACTIVE_OUTPUT_GP ); gpio_set_dir( IF1_PORT_ACTIVE_OUTPUT_GP, GPIO_OUT );
  gpio_put( IF1_PORT_ACTIVE_OUTPUT_GP, 1 );

  multicore_launch_core1( core1_main );

  gpio_set_dir_in_masked( DBUS_MASK );
  while(1)
  {
    register uint32_t gpios_state;

    /*
     * Spin while the hardware is saying at least one of A14, A15 and MREQ is 1.
     * ROM_READ is active low - if it's 1 then the ROM is not being read.
     */
    while( (gpios_state=gpio_get_all()) & ROM_READ_BIT_MASK );

    register uint16_t raw_bit_pattern = pack_address_gpios( gpios_state );

    register uint16_t rom_address = address_indirection_table[raw_bit_pattern];

    register uint8_t rom_value = *(rom_image_ptr+rom_address);

//if( (rom_image_ptr == __ROMs_if1_rom) && ((rom_address == 0x16AA) || (rom_address == 0x16AB)) )
//{
//  rom_value = 0;
//}
//if( (rom_image_ptr == __ROMs_if1_rom) && (rom_address == 0x1693) )
//{
// This fixes the format problem for all microdrives except #1
//  rom_value = 1;   // 1 byte is max, 2 fails
//}

#if 0
if( (rom_image_ptr == __ROMs_if1_rom) && (rom_address == 0x16B6) )
{
  rom_value = 0xF2;   // LD      A,$E2 ;    OUT     ($EF),A         ; enable writing
}
if( (rom_image_ptr == __ROMs_if1_rom) && (rom_address == 0x16D2) )
{
  rom_value = 0xF6;   // LD      A,$E6 ;    OUT     ($EF),A
}
if( (rom_image_ptr == __ROMs_if1_rom) && (rom_address == 0x1B7A) )
{
  rom_value = 0x0E; // 15 secotrs
}
if( (rom_image_ptr == __ROMs_if1_rom) && (rom_address == 0x1693) )
{
  rom_value = 0x1; // Read one more FC, not 10015 secotrs
}
#endif

    /*
     * This Pico is about to put a value on the data bus. The "ROM read" logic
     * hardware signal goes to Pico2 (the IO handling Pico) as well as this one.
     * The other Pico uses that (plus its own IORQ logic) to set the level
     * shifter for the data bus lines to Pico->ZX. This Pico doesn't need to
     * worry about the level shifter direction. So here, the Pico GPIOs are set
     * to outputs and the value asserted for the Z80 to collect
     */
    gpio_set_dir_out_masked( DBUS_MASK );
    gpio_put_masked( DBUS_MASK, rom_value );

    /*
     * Spin until the Z80 releases MREQ indicating the read is complete.
     * ROM_READ is active low - if it's 0 then the ROM is still being read.
     */
    while( (gpio_get_all() & ROM_READ_BIT_MASK) == 0 );

    /*
     * Set the data bus GPIOs back to inputs. The level shifter for the data
     * bus lines is switched back to ZX->Pico by the IO handling Pico
     */
    gpio_set_dir_in_masked( DBUS_MASK );

    /*
     * M1 active means that that was an instruction fetch the Z80 just did.
     * If it's one of the magic addresses, page the IF1 ROM
     */
    if( (gpios_state & M1_INPUT_BIT_MASK) == 0 )
    {
      if( rom_image_ptr == __ROMs_if1_rom )
      {
#if 0
	if( rom_address == 0x1c05 ) _1c05_visits++;
	if( rom_address == 0x13e3 ) _13e3_visits++;
	if( rom_address == 0x1c0a ) _1c0a_visits++;
	if( rom_address == 0x1c16 ) _1c16_visits++;
	if( rom_address == 0x169d ) _169d_visits++;
	if( rom_address == 0x16a4 ) _16a4_visits++;
	if( rom_address == 0x16a5 ) _16a5_visits++;
	if( rom_address == 0x16ac ) _16ac_visits++;
#endif
      }
      if( (rom_address == 0x0008) || (rom_address == 0x1708) )
      {
	gpio_put(LED_PIN, 1);
	rom_image_ptr = __ROMs_if1_rom;
      }
      else if( rom_address == 0x0700 )
      {
	rom_image_ptr = __ROMs_48_original_rom;
	gpio_put(LED_PIN, 0);
      }
#if RECORDING
    /* Log to the recorder buffer the ROM address requested */
    if( (rom_image_ptr == __ROMs_if1_rom) && (recorder_index >=0) )
    {
      address_recorder[ recorder_index ].gpios    = gpios_state;
      address_recorder[ recorder_index ].address  = rom_address;
      address_recorder[ recorder_index ].response = unconverted_byte[ (uint8_t)rom_value ];
      if( ++recorder_index == RECORDER_SIZE )
      {
	recorder_index = 0;
      }
    }
#endif

    }


  } /* Infinite loop */

}



#if 0
/* Blip the result pin, shows on scope */
gpio_put( TEST_OUTPUT_GP, 1 ); busy_wait_us_32(5);
gpio_put( TEST_OUTPUT_GP, 0 ); busy_wait_us_32(5);

gpio_put( TEST_OUTPUT_GP, 1 ); busy_wait_us_32(1000);
gpio_put( TEST_OUTPUT_GP, 0 ); busy_wait_us_32(1000);
__asm volatile ("nop");
#endif

