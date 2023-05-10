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

#define MICRODRIVE_BLOCK_MAX 254
#define MICRODRIVE_HEAD_LEN 15
#define MICRODRIVE_DATA_LEN 512
#define MICRODRIVE_BLOCK_LEN ( MICRODRIVE_HEAD_LEN + \
                               MICRODRIVE_HEAD_LEN +  \
                               MICRODRIVE_DATA_LEN + 1 )   /* 543 */
#define MICRODRIVE_CARTRIDGE_LENGTH \
        ( MICRODRIVE_BLOCK_MAX * MICRODRIVE_BLOCK_LEN )    /* 97740 for 180 block cartridge 137922 for 254 blocks */

#endif


