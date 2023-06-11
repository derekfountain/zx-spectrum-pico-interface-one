/*
 * Pico PIO Connect, a utility to connect Raspberry Pi Picos together
 * Copyright (C) 2023 Andrew Menadue
 * Copyright (C) 2023 Derek Fountain
 * 
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __LINK_COMMON_H
#define __LINK_COMMON_H

typedef enum
{
  LINK_BYTE_NONE,
  LINK_BYTE_ACK,
  LINK_BYTE_DATA
}
link_received_t;

/* This is in the PIO source code */
bool picoputerlinkin_get( PIO pio, uint sm, uint32_t *value );

link_received_t receive_acked_byte( PIO pio, int linkin_sm, int linkout_sm, uint8_t *received_value );
void receive_buffer( PIO pio, int linkin_sm, int linkout_sm, uint8_t *data, uint32_t count );
void send_ack_to_link( PIO pio, int linkout_sm );
void send_byte( PIO pio, int linkout_sm, int linkin_sm, uint8_t data );
void send_buffer( PIO pio, int linkout_sm, int linkin_sm, const uint8_t *data, uint32_t count );

void send_init_sequence( PIO pio, int linkout_sm, int linkin_sm );
void wait_for_init_sequence( PIO pio, int linkin_sm, int linkout_sm );

uint16_t fletcher16( uint8_t *data, int count );

#endif
