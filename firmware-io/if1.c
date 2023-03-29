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

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "libspectrum.h"
#include "if1.h"
#include "tape.h"

#include "flash_images.h"

#define NO_ACTIVE_MICRODRIVE   -1

static microdrive_t microdrive[NUM_MICRODRIVES];
static int32_t      active_microdrive;

//static uint8_t     *cartridge_data;

//static tape_image tape_images[2] = 
//{
//  { md1_image, md1_image_len },
//  { md2_image, md2_image_len },
//  { md3_image, md3_image_len },
//  { md4_image, md4_image_len },
//  { md5_image, md5_image_len },
//  { md6_image, md6_image_len },
//  { md7_image, md7_image_len },
//  { md8_image, md8_image_len },
//};

static if1_ula_t    if1_ula;

extern const uint8_t LED_PIN;

/*
 * Initialise Interface One structure. Create the 8 microdrive images
 * and allocate a buffer for the currently used cartridge.
 */
int if1_init( void )
{
  int m;

  if1_ula.comms_clk = 0;

  /* 
   * There's only one microdrive image in RAM. I don't have enough RAM to
   * malloc more than one. The "not currently being used" images are held
   * in flash and "paged" in.
   */
//  if( (cartridge_data = malloc( LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH )) == NULL )
//    return -1;

  for( m=0; m<NUM_MICRODRIVES; m++ )
  {
    if( (microdrive[m].cartridge = malloc( sizeof(struct libspectrum_microdrive) )) == NULL )
      return -1;    
    
//    microdrive[m].cartridge->data = cartridge_data;
    microdrive[m].inserted        = 0;
    microdrive[m].modified        = 0;
  }

  active_microdrive = NO_ACTIVE_MICRODRIVE;

  return 0;
}


int if1_mdr_insert( int which, const char *filename )
{
  int i;

  if( which == -1 )
    return -1;

  /*
   * This currently loads the test image from RAM into the RAM buffer.
   * I need to move things around so the test image is in flash, which
   * will be closer to what is required.
   */

  if( load_tape( which ) == -1 )
    return -1;

  if( libspectrum_microdrive_mdr_read( microdrive[which].cartridge,
				       NULL,
				       0,
				       1 ) != LIBSPECTRUM_ERROR_NONE )
    return -1;

  libspectrum_microdrive_set_write_protect( microdrive[which].cartridge, 0 );

  /* Inserted, but not active */
  microdrive[which].inserted = 1;
  microdrive[which].modified = 0;

  /*
   * pream is 512 bytes in the microdrive_t structure.
   * This is filling 2 areas of the microdrive area's premable with SYNC_OK.
   * The loop is over the number of blocks on the cartridge.
   * Not quite sure what it's doing, maybe marking some sort of sector map?
   */

  /* we assume formatted cartridges */
  for( i = libspectrum_microdrive_cartridge_len( microdrive[which].cartridge ); i > 0; i-- )
    microdrive[which].pream[255 + i] = microdrive[which].pream[i-1] = SYNC_OK;

  return 0;
}


void microdrives_reset( void )
{
  int m;

  for( m=0; m<NUM_MICRODRIVES; m++ )
  {
    microdrive[m].head_pos   = 0;
    microdrive[m].motor_on   = 0;
    microdrive[m].gap        = 15;
    microdrive[m].sync       = 15;
    microdrive[m].transfered = 0;
  }

  active_microdrive = NO_ACTIVE_MICRODRIVE;

  if1_ula.comms_clk = 0;
}


static inline void __time_critical_func(increment_head)( int which )
{
  if( which == NO_ACTIVE_MICRODRIVE )
    return;

  microdrive[which].head_pos++;
  if( microdrive[which].head_pos >=
                        (libspectrum_microdrive_cartridge_len( microdrive[which].cartridge ) * LIBSPECTRUM_MICRODRIVE_BLOCK_LEN) )
  {
    microdrive[which].head_pos = 0;
  }
}


/*
 * I think the idea here is that whenever the Z80 asks for the microdrive
 * status that can be seen as an indication that the IF1 is going to want
 * to read the tape. On the real device this action will be a poll, "is
 * the data ready? is the data ready? is the data ready?..." For this
 * emulation we can immediately mkae it ready and respond "yes, ready".
 */
void __time_critical_func(microdrives_restart)( void )
{
  int m;

  for( m=0; m<NUM_MICRODRIVES; m++ )
  {
    /* FIXME Surely it's possible to calculate where the head needs to move to? */
    while( ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != 0  &&
	   ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != LIBSPECTRUM_MICRODRIVE_HEAD_LEN )
    {
      increment_head( m ); /* put head in the start of a block */
    }
	
    microdrive[m].transfered = 0; /* reset current number of bytes written */

    if( ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) == 0 )
    {
      /* up to 15 bytes for header blocks */
      microdrive[m].max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN;
    }
    else
    {
      /* up to 528 bytes for data blocks */
      microdrive[m].max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN + LIBSPECTRUM_MICRODRIVE_DATA_LEN + 1;
    }
  }
}


inline libspectrum_byte __time_critical_func(port_ctr_in)( void )
{
  libspectrum_byte ret = 0xff;

  int block;

  if( (active_microdrive != NO_ACTIVE_MICRODRIVE) && microdrive[active_microdrive].motor_on && microdrive[active_microdrive].inserted )
  {
    /* Calculate the block under the head */
    /* max_bytes is the number of bytes which can be read from the current block */
    block = microdrive[active_microdrive].head_pos / 543 + ( microdrive[active_microdrive].max_bytes == 15 ? 0 : 256 );

    /* pream might be an array of flags, one for each block, indicating something... */
    /* Original comment suggests formatted? Of the block? */
    if( microdrive[active_microdrive].pream[block] == SYNC_OK )  	/* if formatted */
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
      if( microdrive[active_microdrive].gap )
      {
	microdrive[active_microdrive].gap--;
      }
      else
      {
	ret &= 0xf9; /* GAP and SYNC low (both are active low) */

	if( microdrive[active_microdrive].sync )
	{
	  microdrive[active_microdrive].sync--;
	}
	else
	{
	  microdrive[active_microdrive].gap = 15;
	  microdrive[active_microdrive].sync = 15;
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
    if( libspectrum_microdrive_write_protect( microdrive[active_microdrive].cartridge ) )
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
inline void __time_critical_func(port_ctr_out)( libspectrum_byte val )
{
  /* Look for a falling edge on the CLK line */
  if( ((val & 0x02) == 0) && (if1_ula.comms_clk == 1) )
  {
    /* Falling edge of the clock on bit 0x02  ~~\__ */

    uint32_t previously_active_microdrive = active_microdrive;

    /*
     * A pulse has arrived on bit 0x01. There will be 8 of these in total,
     * sent in a 1ms sequence by the IF1's ROM routine SEL-DRIVE.
     * https://www.tablix.org/~avian/spectrum/rom/if1_2.htm#L1532
     * This pulse just received will be used to switch the motor of
     * microdrive 0. All the current motor control values are shuffled
     * along to the left and the new one is dropped into the microdrive
     * furthest right
     */
    int m;
    for( m = 7; m > 0; m-- )
    {
      microdrive[m].motor_on = microdrive[m - 1].motor_on;
    }
    uint8_t action = (val & 0x01) ? 0 : 1;
    microdrive[0].motor_on = action;

    /* Work out which microdrive that leaves active, if any */
    uint32_t active_microdrive = NO_ACTIVE_MICRODRIVE;
    for( m = 0; m < NUM_MICRODRIVES; m++ )
    {
      if( microdrive[m].motor_on )
      {
	active_microdrive = m;
	break;
      }
    }
active_microdrive = 0;

#if 0
    if( (previously_active_microdrive != NO_ACTIVE_MICRODRIVE) && (active_microdrive != previously_active_microdrive) )
    {
      /* Previously spinning drive has just had its motor turned off  - we need to copy the data buffer out to flash */

      if( microdrive[active_microdrive].modified == 1 )
      {
	long unsigned int offset = (long unsigned int)((uint8_t*)cartridge_images[previously_active_microdrive].address - XIP_BASE);
	size_t            bytes  = (size_t)(((cartridge_images[previously_active_microdrive].length+
					                                 FLASH_SECTOR_SIZE) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE);

	flash_range_erase( offset, bytes );

	bytes  = (size_t)(((cartridge_images[previously_active_microdrive].length+FLASH_PAGE_SIZE) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE);
	flash_range_program( offset, microdrive[previously_active_microdrive].cartridge->data, bytes );
	microdrive[active_microdrive].modified = 0;
      }
      
      microdrive[previously_active_microdrive].cartridge->data = NULL;

//      gpio_put( LED_PIN, 0 );
    }

    if( active_microdrive != NO_ACTIVE_MICRODRIVE )
    {
      /* Turn on motor - we need to bring in the data buffer from flash */

      microdrive[active_microdrive].cartridge->data = cartridge_data;
      libspectrum_microdrive_mdr_read( microdrive[active_microdrive].cartridge,
				       cartridge_images[active_microdrive].address,
				       cartridge_images[active_microdrive].length,
				       1 );
//      gpio_put( LED_PIN, 1 );
    }
#endif

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
inline libspectrum_byte __time_critical_func(port_mdr_in)( void )
{
  libspectrum_byte ret = 0xff;

  if( active_microdrive == NO_ACTIVE_MICRODRIVE )
    return ret;

  if( microdrive[active_microdrive].motor_on && microdrive[active_microdrive].inserted )
  {
    /*
     * max_bytes is the number of bytes in the block under
     * the head, and transfered is the number of bytes transferred out of
     * it so far
     */
    if( microdrive[active_microdrive].transfered < microdrive[active_microdrive].max_bytes )
    {
      query_tape_byte( microdrive[active_microdrive].head_pos, &ret );

      /* Move tape on, with wrap */
      increment_head( active_microdrive );
    }

    /*
     * transfered is a count of the number of bytes transferred from
     * the block to the IF1
     */
    microdrive[active_microdrive].transfered++;
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
inline void __time_critical_func(port_mdr_out)( libspectrum_byte val )
{
  int block;

  if( active_microdrive == NO_ACTIVE_MICRODRIVE )
    return;

  if( microdrive[active_microdrive].motor_on && microdrive[active_microdrive].inserted )
  {
    /* Calculate the block under the head */
    /* max_bytes is the number of bytes which can be read from the current block */
    block = microdrive[active_microdrive].head_pos / 543 + ( microdrive[active_microdrive].max_bytes == 15 ? 0 : 256 );

    /*
     * Preamble handling
     */
    if( microdrive[active_microdrive].transfered == 0 && val == 0x00 )
    {
      /*
       * This tracks the writing of 10 0x00 bytes...
       */
      microdrive[active_microdrive].pream[block] = 1;
    }
    else if( microdrive[active_microdrive].transfered > 0 && microdrive[active_microdrive].transfered < 10 && val == 0x00 )
    {
      microdrive[active_microdrive].pream[block]++;
    }
    else if( microdrive[active_microdrive].transfered > 9 && microdrive[active_microdrive].transfered < 12 && val == 0xff )
    {
      /*
       * ...followed by 2 0xFF bytes...
       */
      microdrive[active_microdrive].pream[block]++;
    }
    else if( microdrive[active_microdrive].transfered == 12 && microdrive[active_microdrive].pream[block] == 12 )
    {
      /*
       * ...and when those 12 have arrived the preamble is complete.
       * Not exactly robust, but good enough.
       */
      microdrive[active_microdrive].pream[block] = SYNC_OK;
    }

    /*
     * max_bytes is the number of bytes in the block the head is on.
     * The preamble isn't counted, so only write when the head is
     * outside that range
     */
    if( (microdrive[active_microdrive].transfered > 11)
	&&
	(microdrive[active_microdrive].transfered < (microdrive[active_microdrive].max_bytes + 12)) )
    {
      write_tape_byte( microdrive[active_microdrive].head_pos, val );
      increment_head( active_microdrive );
      microdrive[active_microdrive].modified = 1;
    }

    /* transfered does include the preamble */
    microdrive[active_microdrive].transfered++;
  }
}
