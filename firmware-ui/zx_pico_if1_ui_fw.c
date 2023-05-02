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

#include "spi.h"

/* 1 instruction on the 133MHz microprocessor is 7.5ns */
/* 1 instruction on the 140MHz microprocessor is 7.1ns */
/* 1 instruction on the 150MHz microprocessor is 6.6ns */
/* 1 instruction on the 200MHz microprocessor is 5.0ns */

//#define OVERCLOCK 150000
//#define OVERCLOCK 270000

const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;


int main( void )
{
  bi_decl(bi_program_description("ZX Spectrum Pico IF1 board binary."));

#ifdef OVERCLOCK
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  spi_init(UI_TO_IO_SPI, 1 * 1000 * 1000);
  gpio_set_function(UI_TO_IO_SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(UI_TO_IO_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(UI_TO_IO_SPI_TX_PIN, GPIO_FUNC_SPI);

  /* Output for chip select on slave, starts high (unselected) */
  gpio_init(UI_TO_IO_SPI_CSN_PIN);
  gpio_set_dir(UI_TO_IO_SPI_CSN_PIN, GPIO_OUT);
  gpio_put(UI_TO_IO_SPI_CSN_PIN, 1);

  gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put( LED_PIN, 0 );

  while( 1 )
  {
    gpio_put( UI_TO_IO_SPI_CSN_PIN, 0 );
    uint8_t write_cmd[] = { 1 };
    spi_write_blocking(UI_TO_IO_SPI, write_cmd, 1);
    gpio_put( UI_TO_IO_SPI_CSN_PIN, 1 );

    gpio_put( LED_PIN, 1 );
    sleep_ms(250);



    gpio_put( UI_TO_IO_SPI_CSN_PIN, 0 );
    write_cmd[0] = 0;
    spi_write_blocking(UI_TO_IO_SPI, write_cmd, 1);
    gpio_put( UI_TO_IO_SPI_CSN_PIN, 1 );

    gpio_put( LED_PIN, 0 );
    sleep_ms(250);

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
