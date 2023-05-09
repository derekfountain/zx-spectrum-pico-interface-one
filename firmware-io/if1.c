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
#include "hardware/flash.h"
#include "hardware/dma.h"
#include "hardware/spi.h"

#include "libspectrum.h"
#include "if1.h"
#include "spi.h"

/* This file is generated from test data, don't change what's in it manually */
#include "flash_images.h"

#define NO_ACTIVE_MICRODRIVE  ((microdrive_index_t)(-1))

/*
 * These are the test images in flash. They can be memcpy'ed from flash memory,
 * but writing them back is more elaborate.
 *
 * Examine, for example, 98304 bytes of flash memory with:
 *
 * (gdb) x/98304xb flash_mdr_image[0].flash_address
 *
 * Remove any cruft from the bottom of the output (left side is OK) then convert
 * to binary with
 *
 * > perl -ne 'while( m/ (0x..)/g) {print chr(hex($1))}' < ram.txt > ram.bin
 */
static flash_mdr_image_t flash_mdr_image[NUM_MICRODRIVES] =
{
  { tape1_image, tape1_image_len },
  { tape2_image, tape2_image_len },
  { tape3_image, tape3_image_len },
  { tape4_image, tape4_image_len },
  { tape5_image, tape5_image_len },
  { tape6_image, tape6_image_len },
  { tape7_image, tape7_image_len },
  { tape8_image, tape8_image_len },
};


/*
 * These are the microdrives, typically 8 of them. This structure
 * keeps track of the data (in external PSRAM), motor, the head
 * position, gaps, preambles, things like that.
 */
static microdrive_t microdrive[NUM_MICRODRIVES];

/* Clock bit in the ULA */
static uint8_t if1_ula_comms_clk;

extern const uint8_t LED_PIN;

static void __time_critical_func(microdrives_restart)( void );

static void __time_critical_func(blip_test_pin)( void )
{
// No GPIOs left
#if 0
  extern const uint8_t TEST_OUTPUT_GP;
  gpio_put( TEST_OUTPUT_GP, 1 );
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
  gpio_put( TEST_OUTPUT_GP, 0 );
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
  __asm volatile ("nop");
#endif
}


/*
 * Initialise Interface One structure. Create the 8 microdrive images.
 * All start off with no cartridge inserted.
 */
int32_t if1_init( void )
{
  if1_ula_comms_clk = 0;

  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    microdrive[m].inserted   = 0;
    microdrive[m].modified   = 0;
    microdrive[m].head_pos   = 0;
    microdrive[m].motor_on   = 0;
    microdrive[m].gap        = 15;
    microdrive[m].sync       = 15;
    microdrive[m].transfered = 0;

    microdrive[m].cartridge_data_psram_offset = 0;
    microdrive[m].cartridge_data_modified     = 0;
    microdrive[m].cartridge_write_protect     = 0;
    microdrive[m].cartridge_len_in_blocks     = 0;
  }

  return 0;
}


/*
 * Load one of the tape images from flash into the PSRAM buffer.
 * The UI Pico will do this job at some point but I don't yet
 * know how
 */
/* Lopping this lot off the stack will cause a problem if the program grows. Keep static. Might need to page */
static tape_byte_t  __attribute__((aligned(4))) cartridge_data[LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH];

static int32_t load_flash_tape_image( flash_mdr_image_index_t which )
{
  /* Check requested image exists */
  if( (which < 0) || (which > LAST_MICRODRIVE_INDEX) )
    return -1;

  /* Check requested image will fit in buffer */
  if( flash_mdr_image[which].length > LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH )
    return -1;

  /*
   * Copy cartridge image data from flash into the SPI PSRAM which is where it's
   * normally used from. I can't DMA (or memcpy()) into the SPI device, so this
   * goes in 2 steps: flash into onboard RAM, onboard RAM into PSRAM
   */

  /*
   * Step 1, pull from flash into onboard RAM
   *
   * memcpy( cartridge_data, (tape_byte_t*)flash_mdr_image[which].flash_address, flash_mdr_image[which].length );
   *
   * Using DMA, but in practise this is only about 1ms faster than the memcpy()
   * It still takes about 8ms.
   */
  int chan             = dma_claim_unused_channel( true );
  dma_channel_config c = dma_channel_get_default_config( chan );
  channel_config_set_transfer_data_size( &c, DMA_SIZE_32 );
  channel_config_set_read_increment( &c, true );
  channel_config_set_write_increment( &c, true );

  dma_channel_configure(
    chan,                                  // Channel to be configured
    &c,                                    // The configuration we just created
    cartridge_data,                        // The initial write address
    flash_mdr_image[which].flash_address,  // The initial read address
    flash_mdr_image[which].length/4,       // Number of transfers; in this case each is 4 bytes.
    true                                   // Start immediately.
  );

  dma_channel_wait_for_finish_blocking( chan );
  dma_channel_unclaim( chan );

  /* OK, cartridge data is DMA'ed from flash in onboard RAM, step 2 is to copy from RAM into PSRAM */
  gpio_put( PSRAM_SPI_CSN_PIN, 0 );

  /*
   * Write cartridge data bytes into SPI PSRAM. Don't worry about exact size, there's
   * plenty of room in the PSRAM. Each cartridge takes up the maximum possible number
   * of bytes
   */
  uint32_t psram_offset = which * LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH;

  uint8_t write_cmd[] = { PSRAM_CMD_WRITE,
			  psram_offset >> 16, psram_offset >> 8 , psram_offset
			};
  spi_write_blocking(PSRAM_SPI, write_cmd, sizeof(write_cmd));
  spi_write_blocking(PSRAM_SPI, cartridge_data, LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH);

  gpio_put( PSRAM_SPI_CSN_PIN, 1 );

  /* Note where the cartridge image data has been put in the PSRAM */
  microdrive[which].cartridge_data_psram_offset = psram_offset;

  /* Note how many blocks it is */
  size_t length_in_bytes = flash_mdr_image[which].length - ( flash_mdr_image[which].length % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN );
  microdrive[which].cartridge_len_in_blocks = (length_in_bytes / LIBSPECTRUM_MICRODRIVE_BLOCK_LEN);
 
  trace(TRC_LOAD_IMAGE, which);

  return which;
}


int32_t if1_mdr_insert( const microdrive_index_t which )
{
  /*
   * This loads the test cartridge data from flash. It'll come via
   * the UI Pico at some point
   */
  if( load_flash_tape_image( which ) == -1 )
    return -1;

  microdrive[which].inserted = 1;
  microdrive[which].modified = 0;

  /*
   * pream is 512 bytes in the microdrive_t structure.
   * This is filling 2 areas of the microdrive area's premable with SYNC_OK.
   * The loop is over the number of blocks on the cartridge.
   * Not quite sure what it's doing, maybe marking some sort of sector map?
   */
  for( size_t i = microdrive[which].cartridge_len_in_blocks; i > 0; i-- )
    microdrive[which].pream[255 + i] = microdrive[which].pream[i-1] = SYNC_OK;

  /*
   * Position ready to read a block. Probably redundant because the motor-on
   * code does it
   */
  microdrives_restart();

  trace(TRC_IMAGE_INIT, which);

  return 0;
}


/*
 * Advance the given microdrive's head position on the tape.
 * Wrap at the "end" of the tape.
 */
static inline void __time_critical_func(increment_head)( microdrive_index_t which )
{
  microdrive[which].head_pos++;
  if( microdrive[which].head_pos >= (microdrive[which].cartridge_len_in_blocks * LIBSPECTRUM_MICRODRIVE_BLOCK_LEN) )
  {
    microdrive[which].head_pos = 0;
  }
}


/*
 * Find and return the index of the microdrive with the motor on.
 * Returns NO_ACTIVE_MICRODRIVE if they're all off.
 */
static microdrive_index_t __time_critical_func(query_active_microdrive)( void )
{
  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    if( microdrive[m].motor_on )
    {
      return m;
    }
  }
  return NO_ACTIVE_MICRODRIVE;
}


/*
 * I think the idea here is that whenever the Z80 asks for the microdrive
 * status that can be seen as an indication that the IF1 is going to want
 * to read the tape. On the real device this action will be a poll, "is
 * the data ready? is the data ready? is the data ready?..." For this
 * emulation we can immediately make it ready and respond "yes, ready".
 */
static void __time_critical_func(microdrives_restart)( void )
{
  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    while( ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != 0  &&
	   ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != LIBSPECTRUM_MICRODRIVE_HEAD_LEN )
    {
      increment_head(m); /* put head in the start of a block */
    }
	
    microdrive[m].transfered = 0; /* reset current number of bytes written */

    if( ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) == 0 )
    {
      microdrive[m].max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN; /* up to 15 bytes for header blocks */
    }
    else
    {
      microdrive[m].max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN + LIBSPECTRUM_MICRODRIVE_DATA_LEN + 1; /* up to 528 bytes for data blocks */
    }
  }

}


/*
 * Control port in - Z80 is reading IF1/microdrives status
 */
inline libspectrum_byte __time_critical_func(port_ctr_in)( void )
{
  libspectrum_byte ret = 0xff;

  microdrive_index_t active_microdrive_index;
  if( (active_microdrive_index=query_active_microdrive()) == NO_ACTIVE_MICRODRIVE )
  {
    blip_test_pin(); /* "Can't happen" with IF1 ROM code */
  }
  else 
  {
    if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].inserted )
    {
      /* Calculate the block under the head */
      /* max_bytes is the number of bytes which can be read from the current block */
      int block = microdrive[active_microdrive_index].head_pos / 543 + ( microdrive[active_microdrive_index].max_bytes == 15 ? 0 : 256 );

      /* pream might be an array of flags, one for each block, indicating something... */
      /* Original comment suggests formatted? Of the block? */
      if( microdrive[active_microdrive_index].pream[block] == SYNC_OK )  	/* if formatted */
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
	 * Gap is looked for in the FORMAT command, and the TURN-ON routine
	 * which establishes if a formatted cartridge is in the drive. Also
	 * GET-M-BUF which is the microdrive block read routine. These bits
	 * of code look for a non-gap followed by a gap. In the receive-block
	 * code https://www.tablix.org/~avian/spectrum/rom/if1_2.htm#L15EB
	 * it looks for 8 non-gap statuses, followed by 6 gap statuses
	 * followed by one SYNC status. (Setting .gap to 8 and .sync to 6
	 * works here, but I'll keep with the original code's 15s.)
	 *
	 * If the gap isn't found the IF1 ROM code drops out at NOPRES:
	 * https://www.tablix.org/~avian/spectrum/rom/if1_2.htm#L153D
	 * If the sync isn't found then it goes back to looking for
	 * non-gap followed by gap. When looking for a data block it
	 * loops 500 times before giving up; when looking for a header
	 * block it loops 65535 times before giving up.
	 */
	if( microdrive[active_microdrive_index].gap )
	{
	  /* Send back a "no-gap status", the IF1 looks for 8 consecutive of these */
	  microdrive[active_microdrive_index].gap--;
	}
	else
	{
	  /*
	   * Now start sending back "gap status", the IF1 looks for 6 consecutive of these
	   * followed by one "sync status"
	   */
	  ret &= 0xf9; /* GAP and SYNC low (both are active low) */

	  if( microdrive[active_microdrive_index].sync )
	  {
	    microdrive[active_microdrive_index].sync--;
	  }
	  else
	  {
	    /* 15 is overkill to keep the IF1 ROM code happy, but that's the original FUSE code */
	    microdrive[active_microdrive_index].gap  = 15;
	    microdrive[active_microdrive_index].sync = 15;
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
      if( microdrive[active_microdrive_index].cartridge_write_protect )
      {
	/* If write protected flag is true, pull the bit in the status byte low */
	ret &= 0xfe;
      }
    }
    else
    {
      /* motor isn't running, we'll return GAP=1 and SYNC=1 */
    }
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
  if( ((val & 0x02) == 0) && (if1_ula_comms_clk == 1) )
  {
    /* Falling edge of the clock on bit 0x02  ~~\__ */
    // trace(TRC_FALLING_EDGE, if1_ula_comms_clk);

    /*
     * A falling edge has arrived on the clock bit 0x02. There will
     * be 8 of these in total, sent in a 1ms sequence by the IF1's ROM
     * routine SEL-DRIVE:
     * https://www.tablix.org/~avian/spectrum/rom/if1_2.htm#L1532
     * One of these will have an accompanying 0 pulse on the data bit
     * 0x01 (it's active low, that bit). All the pulses are shifted
     * along the line of microdrives being applied to the motor of each.
     * When the sequence is complete the data bit pulse of the first
     * clock will be in the left-most microdrive's motor status, and
     * the last data pulse will be in the right-most microdrive's
     * motor status.
     * Note the iteration in this mechanism. This code gets called
     * 8 times by the SEL-DRIVE routine, which ensures it turns off
     * all the drives except the one it wants on. So CAT 8 will
     * momentarily turn on MD0, then the shift will turn on MD1 and
     * MD0 will go off, then the next shift will turn on MD2 and
     * MD1 will go off, etc. Each microdrive will briefly go motor-on
     * as the shifting happens.
     */

    /*
     * We have a new pulse, shift all the previous ones along and
     * put this new one (inverted) in the right-most microdrive's motor
     */
    uint8_t any_motor_on = 0;
    for( microdrive_index_t m = LAST_MICRODRIVE_INDEX; m > 0; m-- )
    {
      microdrive[m].motor_on = microdrive[m - 1].motor_on;
      any_motor_on |= microdrive[m].motor_on;
    }
    microdrive[0].motor_on = (val & 0x01) ? 0 : 1;
    any_motor_on |= microdrive[0].motor_on;

//    gpio_put( LED_PIN, any_motor_on );

#if 0
    trace(TRC_MOTORS_ON, (microdrive[7].motor_on << 7) |
	                 (microdrive[6].motor_on << 6) |
	                 (microdrive[5].motor_on << 5) |
	                 (microdrive[4].motor_on << 4) |
                         (microdrive[3].motor_on << 3) |
	                 (microdrive[2].motor_on << 2) |
	                 (microdrive[1].motor_on << 1) |
                         (microdrive[0].motor_on) );
#endif
  }

  /* Note the level of the CLK line so we can see what it's done next time */
  if1_ula_comms_clk = ( val & 0x02 ) ? 1 : 0;

  /*
   * Position all drives at the start of their next block. This is
   * redundant because after starting the motor the next thing the
   * IF1 will do is read status, which does this step. Leave it in
   * though because FUSE does it.
   */
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

  microdrive_index_t active_microdrive_index;
  if( (active_microdrive_index=query_active_microdrive()) == NO_ACTIVE_MICRODRIVE )
  {
    blip_test_pin(); blip_test_pin();
    return ret;  /* "Can't happen" with IF1 ROM code */
  }

  if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].inserted )
  {
    /*
     * max_bytes is the number of bytes in the block under
     * the head, and transfered is the number of bytes transferred out of
     * it so far
     */
    if( microdrive[active_microdrive_index].transfered < microdrive[active_microdrive_index].max_bytes )
    {
      /* Fetch the required byte from the PSRAM device on the SPI bus */

      gpio_put( PSRAM_SPI_CSN_PIN, 0 );

      /* Work out where the byte under the active microdrive's head is stored in the PSRAM */
      uint32_t psram_offset = microdrive[active_microdrive_index].cartridge_data_psram_offset
	                      +
	                      microdrive[active_microdrive_index].head_pos;

      uint8_t read_cmd[] = { PSRAM_CMD_READ,
			     psram_offset >> 16, psram_offset >> 8, psram_offset };
      spi_write_blocking(PSRAM_SPI, read_cmd, sizeof(read_cmd));

      /*
       * Read the byte at that address, that's the one the IF1 wants.
       * This goes into the microdrive's "last" value. There's a corner case during
       * FORMAT. The FORMAT code writes about 650 test bytes (0xFC), then reads
       * them back. If it can't read them then it assumed it's at the tape splice
       * point and marks the sector as unusable. But that 650 bytes is a weird
       * number, it's outside the normal block range. The write code doesn't
       * actually write it to tape beyond the normal block size, so here, where
       * it's read back, it's not available to read from the tape. So under this
       * odd circumstance we just return the last byte again in the "ret &=" line
       * below (which is outside this max_bytes conditional block).
       */
      spi_read_blocking(PSRAM_SPI, 0, (uint8_t*)&(microdrive[active_microdrive_index].last), 1 ); 
      
      gpio_put( PSRAM_SPI_CSN_PIN, 1 );

      /* Move tape on, with wrap */
      increment_head(active_microdrive_index);
    }

    /*
     * transfered is a count of the number of bytes transferred from
     * the block to the IF1
     */
    microdrive[active_microdrive_index].transfered++;
    ret &= microdrive[active_microdrive_index].last;
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
  microdrive_index_t active_microdrive_index;
  if( (active_microdrive_index=query_active_microdrive()) == NO_ACTIVE_MICRODRIVE )
  {
    blip_test_pin(); blip_test_pin(); blip_test_pin(); blip_test_pin();
    return;  /* "Can't happen" with IF1 ROM code */
  }

  if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].inserted )
  {
    /* Calculate the block under the head */
    /* max_bytes is the number of bytes which can be read from the current block */
    int block = microdrive[active_microdrive_index].head_pos / 543 + ( microdrive[active_microdrive_index].max_bytes == 15 ? 0 : 256 );

    /*
     * Preamble handling
     */
    if( microdrive[active_microdrive_index].transfered == 0 && val == 0x00 )
    {
      /*
       * This tracks the writing of 10 0x00 bytes...
       */
      microdrive[active_microdrive_index].pream[block] = 1;
    }
    else if( microdrive[active_microdrive_index].transfered > 0 && microdrive[active_microdrive_index].transfered < 10 && val == 0x00 )
    {
      microdrive[active_microdrive_index].pream[block]++;
    }
    else if( microdrive[active_microdrive_index].transfered > 9 && microdrive[active_microdrive_index].transfered < 12 && val == 0xff )
    {
      /*
       * ...followed by 2 0xFF bytes...
       */
      microdrive[active_microdrive_index].pream[block]++;
    }
    else if( microdrive[active_microdrive_index].transfered == 12 && microdrive[active_microdrive_index].pream[block] == 12 )
    {
      /*
       * ...and when those 12 have arrived the preamble is complete.
       * Not exactly robust, but good enough.
       */
      microdrive[active_microdrive_index].pream[block] = SYNC_OK;
    }

    /*
     * max_bytes is the number of bytes in the block the head is on.
     * The preamble isn't counted, so only write when the head is
     * outside that range
     */
    if( (microdrive[active_microdrive_index].transfered > 11)
	&&
	(microdrive[active_microdrive_index].transfered < (microdrive[active_microdrive_index].max_bytes + 12)) )
    {
      /* OK, write the byte to "tape" which is really the PSRAM on the SPI bus */
      
      gpio_put( PSRAM_SPI_CSN_PIN, 0 );

      /* Work out where the byte under the active microdrive's head is stored in the PSRAM */
      uint32_t psram_offset = microdrive[active_microdrive_index].cartridge_data_psram_offset
	                      +
	                      microdrive[active_microdrive_index].head_pos;
      uint8_t write_cmd[] = { PSRAM_CMD_WRITE,
			      psram_offset >> 16, psram_offset >> 8, psram_offset,
                              val };

      /* Write the IF1's byte to that location in PSRAM */
      spi_write_blocking(PSRAM_SPI, write_cmd, sizeof(write_cmd));

      gpio_put( PSRAM_SPI_CSN_PIN, 1 );

      increment_head(active_microdrive_index);

      microdrive[active_microdrive_index].cartridge_data_modified = 1;
    }

    /* transfered does include the preamble */
    microdrive[active_microdrive_index].transfered++;
  }
}


#if 0
gpio_put( TEST_OUTPUT_GP, 1 );
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
__asm volatile ("nop");
gpio_put( TEST_OUTPUT_GP, 0 );
#endif
