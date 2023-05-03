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
