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

#ifndef __SPI_H
#define __SPI_H

/* Pseudo SRAM is on SPI0 */
#define PSRAM_SPI               spi0

#define PSRAM_SPI_RX_PIN        16
#define PSRAM_SPI_TX_PIN        19
#define PSRAM_SPI_SCK_PIN       18
#define PSRAM_SPI_CSN_PIN       26

/* Commands for the memory chip I'm using */
#define PSRAM_CMD_WRITE         0x02
#define PSRAM_CMD_READ          0x03
#define PSRAM_CMD_FAST_READ     0x0B
#define PSRAM_CMD_RESET_ENABLE  0x66
#define PSRAM_CMD_RESET         0x99
#define PSRAM_CMD_READ_ID       0x9F


/* UI Pico is master to UI Pico on SPI1 */
#define UI_TO_IO_SPI           spi1

/*
 * Empirically, a receiving Pico at 133MHz can handle 5MHz. It can't do 10MHz.
 * That's with a dedicated core which isn't doing much else.
 */
#define UI_TO_IO_SPI_SPEED     (5*1000*1000)

#define UI_TO_IO_SPI_RX_PIN    12
#define UI_TO_IO_SPI_TX_PIN    15
#define UI_TO_IO_SPI_SCK_PIN   14
#define UI_TO_IO_SPI_CSN_PIN   13

/* IO Pico is slave to UI Pico on SPI1 */
#define IO_FROM_UI_SPI          spi1

#define IO_FROM_UI_SPI_RX_PIN   12
#define IO_FROM_UI_SPI_TX_PIN   15
#define IO_FROM_UI_SPI_SCK_PIN  14
#define IO_FROM_UI_SPI_CSN_PIN  13

/*
 * Commands from UI to IO Pico. UI is always master, so it needs to 
 * poll the IO Pico to receive requests for action from the IO Pico
 */
typedef enum
{
  UI_TO_IO_TEST_LED_ON        = 0x01,        // Tell IO Pico to turn its LED on
  UI_TO_IO_TEST_LED_OFF,                     // Tell IO Pico to turn its LED off
  UI_TO_IO_INSERT_MDR,                       // Tell IO Pico to insert the attached MDR image
  UI_TO_IO_EJECT_MDR_REQUEST,                // Tell IO Pico to eject the given MD
  UI_TO_IO_REQUEST_STATUS,                   // Request the IO Pico returns status
  UI_TO_IO_REQUEST_MDR_TO_SAVE,              // Request the IO Pico returns data from MD to save to SD card

  UI_TO_IO_ACK,
}
UI_TO_IO_CMD;

#endif
