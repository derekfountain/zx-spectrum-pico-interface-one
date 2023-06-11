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

/*
 * This is a reasonably generic, high speed Pico to Pico link application
 * discussed here:
 *
 * https://github.com/derekfountain/pico-pio-connect
 *
 * It's used here to link the UI Pico to the IO Pico. I renamed the functions
 * so it's a little more obvious from the UI source code what the functons do.
 *
 * This code is compiled and linked into both the UI and IO Pico applications.
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "link_common.h"

/*
 * Receive a byte from the link. The received value goes into the given location,
 * and the routine returns one of the status values indicating no data received,
 * an acknowledgement received, or data received.
 *
 * The function name is a bit of a misnomer; if you're expecting an ACK then you
 * don't actually receive a byte. In this case you should pass received_value as
 * NULL.
 */
static link_received_t receive_byte( PIO pio, int linkin_sm, uint8_t *received_value )
{
  /* Read from PIO input FIFO */
  uint32_t data;
  if( picoputerlinkin_get( pio, linkin_sm, &data ) == false )
  {
    return LINK_BYTE_NONE;
  }

  /* Invert what's been received */
  data = data ^ 0xFFFFFFFF;

  /* It arrives from the PIO at the top end of the word, so shift down */
  data >>= 22;

  /* Magic value indicates a ACK on the wire */
  if( data == 0x100 )
  {
    return LINK_BYTE_ACK;
  }
  else
  {
    /* Remove stop bit in LSB */
    data >>= 1;
    
    /* Mask out data, just to be sure */
    data &= 0xff;

    if( received_value != NULL )
      *received_value = (uint8_t)data;

    return LINK_BYTE_DATA;
  }
}


/*
 * Receive a byte and ACK it back to the sender
 */
link_received_t ui_link_receive_acked_byte( PIO pio, int linkin_sm, int linkout_sm, uint8_t *received_value )
{
  if( receive_byte( pio, linkin_sm, received_value ) == LINK_BYTE_NONE )
    return LINK_BYTE_NONE;

  ui_link_send_ack_to_link( pio, linkout_sm );

  return LINK_BYTE_DATA;
}


/*
 * Receive a number of bytes into the given buffer. Bytes are acknowledged.
 */
void ui_link_receive_buffer( PIO pio, int linkin_sm, int linkout_sm, uint8_t *data, uint32_t count )
{
  while( count )
  {
    while( ui_link_receive_acked_byte( pio, linkin_sm, linkout_sm, data ) == LINK_BYTE_NONE );
    data++;
    count--;
  }
}


/*
 * Send an ACK.
 */
void ui_link_send_ack_to_link( PIO pio, int linkout_sm )
{
  /* Sends the magic value */
  pio_sm_put_blocking( pio, linkout_sm, (uint32_t)0x2ff );
}


/*
 * Send a byte and wait for the receiver to acknowledge it.
 */
void ui_link_send_byte( PIO pio, int linkout_sm, int linkin_sm, uint8_t data )
{
  pio_sm_put_blocking(pio, linkout_sm, 0x200 | (((uint32_t)data ^ 0xff)<<1));

  while( receive_byte( pio, linkin_sm, NULL ) != LINK_BYTE_ACK );
}


/*
 * Send a buffer of bytes. All bytes are acknowledged.
 */
void ui_link_send_buffer( PIO pio, int linkout_sm, int linkin_sm, const uint8_t *data, uint32_t count )
{
  while( count )
  {
    ui_link_send_byte( pio, linkout_sm, linkin_sm, *data );
    data++;
    count--;
  }
}


/*
 * Send a magic sequence of known bytes. This marries up with the
 * wait_for_init_sequence() function and is used to initialise the
 * link. It gets the two sides in sync.
 */
void ui_link_send_init_sequence( PIO pio, int linkout_sm, int linkin_sm )
{
  const uint8_t init_msg[] = { 0x02, 0x04, 0x08, 0 };
  ui_link_send_buffer( pio, linkout_sm, linkin_sm, init_msg, sizeof(init_msg) );

  while( receive_byte( pio, linkin_sm, NULL ) != LINK_BYTE_ACK );
}


/*
 * Wait for the initialisation sequence sent by the send_init_sequence()
 * function.
 */
void ui_link_wait_for_init_sequence( PIO pio, int linkin_sm, int linkout_sm )
{
  uint8_t init_msg[] = { 0x02, 0x04, 0x08, 0 };
  uint8_t *init_msg_ptr = init_msg;
  while(1)
  {
    uint8_t chr;
    while( ui_link_receive_acked_byte( pio, linkin_sm, linkout_sm, &chr ) == LINK_BYTE_NONE );
    if( chr == *init_msg_ptr )
    {
      init_msg_ptr++;
      if( chr == '\0' )
	break;
    }
    else
    {
      init_msg_ptr = init_msg;
    }
  }
  ui_link_send_ack_to_link( pio, linkout_sm );
}


/*
 * Standard 16 bit checksum, nicked from the Wikipedia entry.
 * Not part of the protocol as such, but might be useful.
 */
uint16_t fletcher16( uint8_t *data, int count )
{
  uint32_t c0, c1;

  for (c0 = c1 = 0; count > 0; )
  {
    size_t blocklen = count;
    if (blocklen > 5802) {
      blocklen = 5802;
    }
    count -= blocklen;
    do {
      c0 = c0 + *data++;
      c1 = c1 + c0;
    } while (--blocklen);
    c0 = c0 % 255;
    c1 = c1 % 255;
  }
  return (c1 << 8 | c0);
}
