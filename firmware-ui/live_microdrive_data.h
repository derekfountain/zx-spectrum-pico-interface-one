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

#ifndef __LIVE_MICRODRIVE_DATA_H
#define __LIVE_MICRODRIVE_DATA_H

#include "microdrive.h"
#include "cartridge.h"
#include "gui/gui.h"

typedef enum
{
  LIVE_STATUS_NO_CARTRIDGE,
  LIVE_STATUS_INSERTING,
  LIVE_STATUS_INSERTED,
}
live_microdrive_status_t;

/*
 * This describes the data which has been loaded from the SD card and
 * sent to the IO pico for use by the Spectrum. I need to keep this in
 * order to be able to save data back to SD card, etc
 *
 * The live status structure contains an array of these, one for each
 * microdrive.
 */
typedef struct _microdrive_inserted_data_t
{
  live_microdrive_status_t status;                  // Status of one microdrive (inserted, no cartridge, etc)
  char                    *filename;                // Name of SD card file loaded
  uint32_t                 cartridge_data_length;   // Number of bytes in the cartridge image
  write_protect_t          write_protected;         // Whether the cartridge is write protected in the IO Pico
  cartridge_error_t        cartridge_error;         // Cartridge error status
  gui_error_t              gui_error;               // GUI error status
}
microdrive_inserted_data_t;


typedef struct _live_microdrive_data_t
{
  microdrive_index_t         microdrive_saving_to_sd;              // Which MD, if any, needs saving data out to SD
  microdrive_inserted_data_t currently_inserted[NUM_MICRODRIVES];
}
live_microdrive_data_t;

#endif
