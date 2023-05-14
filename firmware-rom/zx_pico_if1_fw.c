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

//#define OVERCLOCK 150000
//#define OVERCLOCK 270000

#include "roms.h"

const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;

/*
 * These pin values are the GPxx ones in green background on the pinout diagram.
 * See schematic for how the signals are fed into the Pico's GPIOs
 */
const uint8_t A0_GP          = 8;
const uint8_t A1_GP          = 9;
const uint8_t A2_GP          = 10;
const uint8_t A3_GP          = 11;
const uint8_t A4_GP          = 12;
const uint8_t A5_GP          = 13;
const uint8_t A6_GP          = 14;
const uint8_t A7_GP          = 15;
const uint8_t A8_GP          = 16;
const uint8_t A9_GP          = 17;
const uint8_t A10_GP         = 18;
const uint8_t A11_GP         = 19;
const uint8_t A12_GP         = 20;
const uint8_t A13_GP         = 21;

const uint8_t  D0_GP         = 0;
const uint8_t  D1_GP         = 1;
const uint8_t  D2_GP         = 2;
const uint8_t  D3_GP         = 3; 
const uint8_t  D4_GP         = 4;
const uint8_t  D5_GP         = 5;
const uint8_t  D6_GP         = 6;
const uint8_t  D7_GP         = 7;

const uint32_t  D0_BIT_MASK  = ((uint32_t)1 <<  D0_GP);
const uint32_t  D1_BIT_MASK  = ((uint32_t)1 <<  D1_GP);
const uint32_t  D2_BIT_MASK  = ((uint32_t)1 <<  D2_GP);
const uint32_t  D3_BIT_MASK  = ((uint32_t)1 <<  D3_GP);
const uint32_t  D4_BIT_MASK  = ((uint32_t)1 <<  D4_GP);
const uint32_t  D5_BIT_MASK  = ((uint32_t)1 <<  D5_GP);
const uint32_t  D6_BIT_MASK  = ((uint32_t)1 <<  D6_GP);
const uint32_t  D7_BIT_MASK  = ((uint32_t)1 <<  D7_GP);

/* Input from the logic which merges A14, A15 and MREQ, goes active on a ROM read */
const uint8_t  ROM_READ_GP              = 26;
const uint32_t ROM_READ_BIT_MASK        = ((uint32_t)1 << ROM_READ_GP);

/* Z80's M1 signal, needs to be merged into the IF1 paging logic */
const uint8_t  M1_GP                    = 22;
const uint32_t M1_INPUT_BIT_MASK        = ((uint32_t)1 << M1_GP);

/*
 * Output signal, goes to Pico2 (IO Pico) to tell it that one of the
 * IO ports it's interested in is on the address bus
 */
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

/* Start with the 1982 Sinclair Research ROM */
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

/* You'll need to crank up the overclock to 270MHz if this is turned on */
#define RECORDING 0
#if RECORDING

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


/*
 * Second core code, spins, watching the address bus. If one of the port values
 * which the IF1 is interested in appears on the lower address lines, set the
 * flag GPIO which tells the IO Pico it needs to pay attention.
 *
 * This can probably be moved to PIO if I need the core.
 */
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

    /*
     * Assume the address bus GPIOs are in sequence, so just shift
     * down and mask everything except the lowest 14 bits
     */
    register uint16_t rom_address = (gpios_state>>A0_GP) & 0x3FFF;

    register uint8_t rom_value = *(rom_image_ptr+rom_address);

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

