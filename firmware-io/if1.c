/*
 * ZX Pico IF1 Firmware, a Raspberry Pi Pico based ZX Interface One emulator
 * Copyright (C) 2023 Derek Fountain
 * Derived from the Fuse code:
 *    Copyright (c) 2004-2016 Gergely Szasz, Philip Kendall
 *    Copyright (c) 2015 Stuart Brady
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

#include <string.h>

#include "hardware/gpio.h"
#include "pico/platform.h"
#include "libspectrum.h"
#include "if1.h"

#include "flash_images.h"

static microdrive_t microdrive;
static if1_ula_t    if1_ula;

extern const uint8_t LED_PIN;

int if1_init( void )
{
  int m, i;

  if1_ula.comms_clk = 0;

  /* 
   * There's only one microdrive image in RAM. I don't have enough RAM to
   * malloc more than one. The "not currently being used" images are held
   * in flash and "paged" in.
   */
  if( (microdrive.cartridge = malloc( sizeof(struct libspectrum_microdrive) )) == NULL )
    return -1;

  microdrive.inserted = 0;
  microdrive.modified = 0;

  return 0;
}

int if1_mdr_insert( int which, const char *filename )
{
  int i;

  /*
   * This currently loads the test image from RAM into the RAM buffer.
   * I need to move things around so the test image is in flash, which
   * will be closer to what is required.
   */
  if( libspectrum_microdrive_mdr_read( microdrive.cartridge,
				       md1_image,
				       md1_image_len ) != LIBSPECTRUM_ERROR_NONE )
  {
    return -1;
  }
  libspectrum_microdrive_set_write_protect( microdrive.cartridge, 0 );

  microdrive.inserted = 1;
  microdrive.modified = 0;

  /*
   * pream is 512 bytes in the microdrive_t structure, the one in this
   * file which represents the microdrive.
   * This is filling 2 areas of the microdrive area's premable with SYNC_OK.
   * The loop is over the number of blocks on the cartridge.
   * Not quite sure what it's doing, maybe marking some sort of sector map?
   */

  /* we assume formatted cartridges */
  for( i = libspectrum_microdrive_cartridge_len( microdrive.cartridge ); i > 0; i-- )
    microdrive.pream[255 + i] = microdrive.pream[i-1] = SYNC_OK;

  return 0;
}

void microdrives_reset( void )
{
  microdrive.head_pos   = 0;
  microdrive.motor_on   = 0;
  microdrive.gap        = 15;
  microdrive.sync       = 15;
  microdrive.transfered = 0;

  if1_ula.comms_clk     = 0;
}

static
inline
void
__time_critical_func(increment_head)( void )
{
  microdrive.head_pos++;
  if( microdrive.head_pos >= (libspectrum_microdrive_cartridge_len( microdrive.cartridge ) * LIBSPECTRUM_MICRODRIVE_BLOCK_LEN) )
  {
    microdrive.head_pos = 0;
  }
}

/*
 * I think the idea here is that whenever the Z80 asks for the microdrive
 * status that can be seen as an indication that the IF1 is going to want
 * to read the tape. On the real device this action will be a poll, "is
 * the data ready? is the data ready? is the data ready?..." For this
 * emulation we can immediately mkae it ready and respond "yes, ready".
 */
void
__time_critical_func(microdrives_restart)( void )
{
  /* FIXME Surely it's possible to calculate where the head needs to move to? */
  while( ( microdrive.head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != 0  &&
	 ( microdrive.head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != LIBSPECTRUM_MICRODRIVE_HEAD_LEN )
  {
    increment_head(); /* put head in the start of a block */
  }
	
  microdrive.transfered = 0; /* reset current number of bytes written */

  if( ( microdrive.head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) == 0 )
  {
    microdrive.max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN; /* up to 15 bytes for header blocks */
  }
  else
  {
    microdrive.max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN + LIBSPECTRUM_MICRODRIVE_DATA_LEN + 1; /* up to 528 bytes for data blocks */
  }
}

inline
libspectrum_byte
__time_critical_func(port_ctr_in)( void )
{
  libspectrum_byte ret = 0xff;

  int block;

  if( microdrive.motor_on && microdrive.inserted )
  {
    /* Calculate the block under the head */
    /* max_bytes is the number of bytes which can be read from the current block */
    block = microdrive.head_pos / 543 + ( microdrive.max_bytes == 15 ? 0 : 256 );

    /* pream might be an array of flags, one for each block, indicating something... */
    /* Original comment suggests formatted? Of the block? */
    if( microdrive.pream[block] == SYNC_OK )  	/* if formatted */
    {
      /* This is the only place the gap is used. It counts down from 15 to 0.
       * While it's non-zero the GAP bit is set in the status byte returned
       * to the IF1. When it gets to zero the sync value is counted down.
       * That's treated in the exact same way, counting down from 15. When
       * that gets to 0 both gap and sync are reset to 15.
       * The effect is output of
       *
       *  GAP=1, SYNC=1 15 times
       *  GAP=0, SYNC=0 15 times
       *
       * From the IF1 disassembly:
       * 
       * ;; REPTEST
       * L154C:  LD      B,$06           ; six consecutive reads required to register
       *                                 ; as a gap.
       *
       * Gap is looked for in the FORMAT command, and the TURN-ON routine
       * which establishes if a formatted cartridge is in the drive. Also
       * GET-M-BUF which is the microdrive block read routine.
       *
       * The sync counter is also only used here. That's accessed in the ROM
       *
       * LD      B,$3C           ; set count 60 decimal.
       *
       * ;; DR-READY
       * L1620:  IN      A,($EF)         ;
       * AND     $02             ; isolate sync bit.
       * JR      Z,L162A         ; forward to READY-RE
       *
       * DJNZ    L1620           ; back to DR-READY
       *
       * So it just asks "is sync set yet?" 60 times and breaks out when it sees
       * a yes. It's just waiting for the tape to get into position, I think.
       * A loop like that is pretty tight, there's 38Ts between INs, which on the
       * Spectrum's 3.5MHz Z80 is around 10uS.
       */
      if( microdrive.gap )
      {
	microdrive.gap--;
      }
      else
      {
	ret &= 0xf9; /* GAP and SYNC low (both are active low) */

	if( microdrive.sync )
	{
	  microdrive.sync--;
	}
	else
	{
	  microdrive.gap = 15;
	  microdrive.sync = 15;
	}
      }
    }
    else
    {
      /* pream[block] is not SYNC_OK, we'll return GAP=1 and SYNC=1 indefinitely */
    }
    
    /*
     * The IF1 ROM code reads the status byte and checks the bit0 result
     * for FORMAT, WRITE and the other write operations. It doesn't keep
     * the value cached, it reads it fresh each time.
     */
    if( libspectrum_microdrive_write_protect( microdrive.cartridge ) )
    {
       /* If write protected flag is true, pull the bit in the status byte low */
      ret &= 0xfe;
    }
  }
  else
  {
    /* motor isn't running, we'll return GAP=1 and SYNC=1 */
  }

  /*
   * Position the microdrives at the start of the next block.
   * If the Z80 likes the response from this function it'll
   * start its next read. This makes the microdrive ready
   * for that next read
   */
  microdrives_restart();

  return ret;
}



/*
 * Control output, which means the IF1 is setting the active drive.
 * If the CLK line is going low set microdrive0 (the only one, now)
 * to motor status as per bit0 (COMMS DATA).
 */
inline
void
__time_critical_func(port_ctr_out)( libspectrum_byte val )
{
  /* Look for a falling edge on the CLK line */
  if( !( val & 0x02 ) && ( if1_ula.comms_clk ) ) {	/* ~~\__ */

    microdrive.motor_on = (val & 0x01) ? 0 : 1;
    gpio_put( LED_PIN, microdrive.motor_on );
  }

  /* Note the level of the CLK line so we can see what it's done next time */
  if1_ula.comms_clk = ( val & 0x02 ) ? 1 : 0;

  microdrives_restart();
}



/*
 * Microdrive in, as seen from the IF1's perspective. This emulates a
 * byte arriving from the microdrive into the IF1.
 *
 * Byte is taken from the data block of the running microdrive
 * cartridge
 */
inline
libspectrum_byte
__time_critical_func(port_mdr_in)( void )
{
  libspectrum_byte ret = 0xff;

  if( microdrive.motor_on && microdrive.inserted )
  {
    /*
     * max_bytes is the number of bytes in the block under
     * the head, and transfered is the number of bytes transferred out of
     * it so far
     */
    if( microdrive.transfered < microdrive.max_bytes )
    {
      ret = libspectrum_microdrive_data( microdrive.cartridge,
					 microdrive.head_pos );
      /* Move tape on, with wrap */
      increment_head();
    }

    /*
     * transfered is a count of the number of bytes transferred from
     * the block to the IF1
     */
    microdrive.transfered++;
  }

  return ret;
}


/*
 * Microdrive output.
 *
 * This is going to write the given byte to the current microdrive position.
 *
 * Recall this from the top of this file:
 *
 Microdrive cartridge
   GAP      PREAMBLE      15 byte      GAP      PREAMBLE      15 byte    512     1
 [-----][00 00 ... ff ff][BLOCK HEAD][-----][00 00 ... ff ff][REC HEAD][ DATA ][CHK]
 Preamble = 10 * 0x00 + 2 * 0xff (12 byte) 
 *
 * The preamble is written to tape by the IF1 code, but isn't actually stored in the
 * MDR format. So this code watches for 10 0x00 bytes, then 2 0xff bytes - that's
 * the preamble. Once that arrives the SYNC_OK flag is set.
 *
 */
inline
void
__time_critical_func(port_mdr_out)( libspectrum_byte val )
{
  int block;

  if( microdrive.motor_on && microdrive.inserted )
  {
    /* Calculate the block under the head */
    /* max_bytes is the number of bytes which can be read from the current block */
    block = microdrive.head_pos / 543 + ( microdrive.max_bytes == 15 ? 0 : 256 );

    /*
     * Preamble handling
     */
    if( microdrive.transfered == 0 && val == 0x00 )
    {
      /*
       * This tracks the writing of 10 0x00 bytes...
       */
      microdrive.pream[block] = 1;
    }
    else if( microdrive.transfered > 0 && microdrive.transfered < 10 && val == 0x00 )
    {
      microdrive.pream[block]++;
    }
    else if( microdrive.transfered > 9 && microdrive.transfered < 12 && val == 0xff )
    {
      /*
       * ...followed by 2 0xFF bytes...
       */
      microdrive.pream[block]++;
    }
    else if( microdrive.transfered == 12 && microdrive.pream[block] == 12 )
    {
      /*
       * ...and when those 12 have arrived the preamble is complete.
       * Not exactly robust, but good enough.
       */
      microdrive.pream[block] = SYNC_OK;
    }

    /*
     * max_bytes is the number of bytes in the block the head is on.
     * The preamble isn't counted, so only write when the head is
     * outside that range
     */
    if( microdrive.transfered > 11 && microdrive.transfered < microdrive.max_bytes + 12 )
    {
      libspectrum_microdrive_set_data( microdrive.cartridge,
				       microdrive.head_pos,
				       val );
      increment_head();
      // microdrive.modified = 1; // Unused for now
    }

    /* transfered does include the preamble */
    microdrive.transfered++;
  }
}
