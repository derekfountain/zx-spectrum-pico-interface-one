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

#ifndef __UI_IO_COMMS_H
#define __UI_IO_COMMS_H

#include "cartridge.h"
#include "microdrive.h"

/*
 * This is the page size for inter-Pico transfers. 137922 divides into 181 762 byte pages. This will need
 * revisiting if the max cartridge length changes, which, since it was defined in 1983, is quite unlikely.
 */
#define MICRODRIVE_CARTRIDGE_PAGE_SIZE (MICRODRIVE_CARTRIDGE_LENGTH/181)

/*
 * My commands from UI to IO Pico. UI is always "master", so it needs to 
 * poll the IO Pico to receive requests for action from the IO Pico
 */
typedef enum
{
  UI_TO_IO_REQUEST_TO_SEND    = 0x01,
  UI_TO_IO_ACK,
  UI_TO_IO_TEST_LED_ON,                                // Tell IO Pico to turn its LED on
  UI_TO_IO_TEST_LED_OFF,                               // Tell IO Pico to turn its LED off
  UI_TO_IO_INSERT_MDR,                                 // Tell IO Pico to insert the attached MDR image
  UI_TO_IO_REQUEST_EJECT_MDR,                          // Tell IO Pico to eject the given MD
  UI_TO_IO_REQUEST_STATUS,                             // Request the IO Pico returns status
  UI_TO_IO_REQUEST_MDR_TO_SAVE,                        // Request the IO Pico returns data from MD to save to SD card
}
UI_TO_IO_CMD;

/* Not sure where this is going yet */
typedef struct _io_status_t
{
  uint8_t error;
}
io_status_t;

/* Data structure for MDR insert command */
typedef struct _ui_to_io_insert_mdr_t
{
  microdrive_index_t microdrive_index;                 // Which microdrive
  uint8_t            is_write_protected;               // Whether the cartridge is write protected
  uint32_t           data_size;                        // Size of cartridge data in bytes
  uint32_t           page_size;                        // A page size suitable for sending the data across to the IO Pico
  uint32_t           checksum;                         // Checksum, not sure if this is needed
}
ui_to_io_insert_mdr_t;

#endif


