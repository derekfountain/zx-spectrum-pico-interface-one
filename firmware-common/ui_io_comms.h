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

/* Sequence of bytes to preceed a UI to IO command. Arbitrary. */
#define UI_TO_IO_CMD_PREAMBLE { 0xDF, 0xAA, 0x55 }

/*
 * My commands from UI to IO Pico. UI is always "master", so it needs to 
 * poll the IO Pico to receive requests for action from the IO Pico
 */
typedef enum
{
  UI_TO_IO_ACK                = 0xAC,

  UI_TO_IO_INIALISE           = 0xF1, 
  UI_TO_IO_REQUEST_TO_SEND,
  UI_TO_IO_TEST_LED_ON,                                // Tell IO Pico to turn its LED on
  UI_TO_IO_TEST_LED_OFF,                               // Tell IO Pico to turn its LED off
  UI_TO_IO_INSERT_MDR,                                 // Tell IO Pico to insert the attached MDR image
  UI_TO_IO_REQUEST_EJECT_MDR,                          // Tell IO Pico to eject the given MD
  UI_TO_IO_REQUEST_STATUS,                             // Request the IO Pico returns status
  UI_TO_IO_REQUEST_MDR_TO_SAVE,                        // Request the IO Pico returns data from MD to save to SD card
}
UI_TO_IO_CMD;


/*
 * Status the drive can be in. Whether it has a cartridge inserted,
 * whether that cartridge has been modified, etc.
 */
typedef enum
{
  MD_STATUS_EMPTY,
  MD_STATUS_MDR_LOADED_UNCHANGED,
  MD_STATUS_MDR_LOADED_NEEDS_SAVING,
  MD_STATUS_MDR_EJECTED_NEEDS_SAVING,
}
microdrive_status_t;


/* Data structure for MDR insert command */
typedef struct _ui_to_io_insert_mdr_t
{
  microdrive_index_t microdrive_index;                 // Which microdrive
  write_protect_t    write_protected;                  // Whether the cartridge is write protected
  uint32_t           data_size;                        // Size of cartridge data in bytes
  uint8_t            checksum;                         // Very basic 8 bit addition checksum, need to improve this
}
ui_to_io_insert_mdr_t;


/* Data structure to request IO Pico reports status of the microdrives */
typedef struct _ui_to_io_request_status_t
{
  uint8_t dummy;                                       // Not sure anything is needed here?
}
ui_to_io_request_status_t;


/* Response from IO Pico informing status of each microdrive */
typedef struct _io_to_ui_status_response_t
{
  microdrive_status_t status[NUM_MICRODRIVES];
  bool                motor_on[NUM_MICRODRIVES];
  cartridge_error_t   cartridge_error[NUM_MICRODRIVES];
}
io_to_ui_status_response_t;


/* Data structure to request IO Pico reports returns cartridge data from one of the microdrives */
typedef struct _ui_to_io_request_mdr_data_t
{
  microdrive_index_t microdrive_index;
  uint32_t           bytes_expected;
}
ui_to_io_request_mdr_data_t;


/* Data structure to request IO Pico ejects a cartridge */
typedef struct _ui_to_io_request_eject_mdr_t
{
  microdrive_index_t microdrive_index;
}
ui_to_io_request_eject_mdr_t;

#endif


