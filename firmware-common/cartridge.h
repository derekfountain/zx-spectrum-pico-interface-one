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

#ifndef __CARTRIDGE_H
#define __CARTRIDGE_H

/* A byte on tape, the cartridge has an array of these */
typedef uint8_t tape_byte_t;

/*
 * Errors associated with cartridge insertion, etc, reported back to
 * the GUI to give the user a clue why a cartridge hasn't inserted.
 */
typedef enum
{
  CARTRIDGE_ERR_OK = 0,
  CARTRIDGE_ERR_CHECKSUM_INCORRECT,
  CARTRIDGE_ERR_NEED_EJECT_BEFORE_INSERT,
  CARTRIDGE_ERR_NEED_SAVE_BEFORE_INSERT,
}
cartridge_error_t;

/*
 * Enum just to make the code more readable. "Non zero if write protected" is defined in the MDR
 * fornmat so it's a binary flag
 */
typedef enum
{
  WRITE_PROTECT_OFF = 0,
  WRITE_PROTECT_ON  = 1
}
write_protect_t;

#define MICRODRIVE_BLOCK_MAX 254
#define MICRODRIVE_HEAD_LEN 15
#define MICRODRIVE_DATA_LEN 512
#define MICRODRIVE_BLOCK_LEN ( MICRODRIVE_HEAD_LEN + \
                               MICRODRIVE_HEAD_LEN +  \
                               MICRODRIVE_DATA_LEN + 1 )   /* 543 */
#define MICRODRIVE_CARTRIDGE_LENGTH \
        ( MICRODRIVE_BLOCK_MAX * MICRODRIVE_BLOCK_LEN )    /* 97740 for 180 block cartridge 137922 for 254 blocks */

/* MDR image is data size plus one more for the write protect flag */
#define MICRODRIVE_MDR_MAX_LENGTH ( MICRODRIVE_CARTRIDGE_LENGTH + 1)

#endif


