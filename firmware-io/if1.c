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
#include "hardware/spi.h"

#include "cartridge.h"
#include "microdrive.h"
#include "if1.h"
#include "spi.h"

#define NO_ACTIVE_MICRODRIVE  ((microdrive_index_t)(-1))

/* This is used in the preamble */
enum
{
  SYNC_NO = 0,
  SYNC_OK = 0xff
};

/*
 * Microdrive structure, represents the drive ifself.
 */
typedef struct _microdrive_t
{
  bool    motor_on;
  int     head_pos;
  int     transfered;
  int     max_bytes;
  uint8_t pream[512];   /* preamble/sync area written. 256 header blocks and 256 data blocks */
  uint8_t last;
  uint8_t gap;
  uint8_t sync;

  /*
   * Offset in PSRAM to the byte array representing the tape, approx 97KB for a 180 sector
   * tape. About 135KB for a 254 sector tape.
   *
   * The following is obsolete because the external RAM can't be examined
   * directly using the debugger. Might be useful though:
   *
   * Copy and paste out to a text file with:
   *
   * (gdb) set print repeats 0
   * (gdb) set print elements unlimited
   * (gdb) set pagination off
   * (gdb) p/x cartridge_data
   * (gdb) set max-value-size unlimited
   *
   * Convert to MDR image with:
   *
   * > perl -ne 'map { print chr(hex($_)) } split(/, /, $_)' < mm_reformated_in_zx.txt > mm_reformated_in_zx.mdr
   *
   * mm_reformated_in_zx.mdr will then load into FUSE as a normal
   * MDR file.
   *
   */
  uint32_t cartridge_data_psram_offset;
  
  bool     cartridge_inserted;
  bool     cartridge_ejected_pending_save;
  bool     cartridge_data_modified;
  bool     cartridge_write_protect;
  uint8_t  cartridge_len_in_blocks;

} microdrive_t;


/*
 * These are the microdrives, typically 8 of them. This structure
 * keeps track of the data (in external PSRAM), motor, the head
 * position, gaps, preambles, things like that.
 */
static microdrive_t microdrive[NUM_MICRODRIVES];

/* Clock bit in the ULA */
static uint8_t if1_ula_comms_clk;

static void __time_critical_func(microdrives_restart)( void );


/* This can be used to blip a pin on the scope, sometimes gives a clue what's happening */
static void __time_critical_func(blip_test_pin)( void )
{
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
}


/*
 * Initialise Interface One structure. Create the 8 microdrive images.
 * All start off with no cartridge inserted.
 */
void if1_init( void )
{
  if1_ula_comms_clk = 0;

  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    microdrive[m].cartridge_inserted             = false;
    microdrive[m].cartridge_ejected_pending_save = false;
    microdrive[m].cartridge_data_modified        = false;
    microdrive[m].head_pos                       = 0;
    microdrive[m].motor_on                       = false;
    microdrive[m].gap                            = 15;
    microdrive[m].sync                           = 15;
    microdrive[m].transfered                     = 0;

    microdrive[m].cartridge_data_psram_offset    = 0;
    microdrive[m].cartridge_write_protect        = false;
    microdrive[m].cartridge_len_in_blocks        = 0;
  }

  return;
}


/*
 * "Insert" one of the tape images into the PSRAM buffer. The PSRAM buffer
 * is where the image will be read from and written to.
 */
void if1_mdr_insert( const microdrive_index_t which, const uint32_t psram_offset, const uint32_t length_in_bytes,
                     const write_protect_t write_protected )
{
  /*
   * We can't insert a new image if there's already one there, or the ejected
   * one hasn't yet been written out to SD card. In theory the UI won't request
   * a new insert until those conditions are sorted out. I just return
   * quietly.
   */
  if( microdrive[which].cartridge_inserted )
  {
    return;
  }

  if( microdrive[which].cartridge_ejected_pending_save )
  {
    return;
  }

  microdrive[which].cartridge_data_psram_offset = psram_offset;
  microdrive[which].cartridge_len_in_blocks     = (length_in_bytes / MICRODRIVE_BLOCK_LEN);
  microdrive[which].cartridge_write_protect     = write_protected;
  microdrive[which].cartridge_inserted          = true;

  /*
   * pream is 512 bytes in the microdrive_t structure. It's actually 2 256 byte
   * buffers, one to indicate the 256 header blocks are formatted, the other
   * to indicate the 256 data blocks are formatted.
   *
   * This is filling the 2 areas of the microdrive area's premable with SYNC_OK.
   * (We assume formatted a cartridge.)
   */
  for( size_t i = microdrive[which].cartridge_len_in_blocks; i > 0; i-- )
    microdrive[which].pream[255 + i] = microdrive[which].pream[i-1] = SYNC_OK;

  /*
   * Position ready to read a block. Probably redundant because the motor-on
   * code does it
   */
  microdrives_restart();

  trace(TRC_IMAGE_INIT, which);

  return;
}


void if1_mdr_eject( const microdrive_index_t which )
{
  /* If no cartridge is inserted there's nothing to do */
  if( microdrive[which].cartridge_inserted )
  {
    /* Immediately mark as not inserted, no further writes will take place */
    microdrive[which].cartridge_inserted = false;

    /*
     * If the data has been modified the PSRAM image for this cartridge is currently
     * unique. It needs to be sent back to the UI Pico for saving on the SD card
     */
    if( microdrive[which].cartridge_data_modified )
    {
      microdrive[which].cartridge_ejected_pending_save = true;
    }
  }
}


/* Answers true if the drive has a cartridge inserted */
bool is_cartridge_inserted( microdrive_index_t which )
{
  return microdrive[which].cartridge_inserted;
}


/* Answers true if the drive has a modified cartridge */
bool is_cartridge_modified( microdrive_index_t which )
{
  return microdrive[which].cartridge_data_modified;
}


/* Sets/resets the cartridge modification flag */
void set_cartridge_modified( microdrive_index_t which, bool modified )
{
  microdrive[which].cartridge_data_modified = modified;
}


/*
 * Answers true if the cartridge has been ejected but the image in
 * the PSRAM still needs to writing back to SD card
 */
bool is_cartridge_ejected_pending_save( microdrive_index_t which )
{
  return microdrive[which].cartridge_ejected_pending_save;
}


/* Resets the pending-save flag, called when the cartridge is safely saved to SD card */
void set_cartridge_ejected_to_sd( microdrive_index_t which )
{
  microdrive[which].cartridge_ejected_pending_save = false;
}


/*
 * Advance the given microdrive's head position on the tape.
 * Wrap at the "end" of the tape.
 */
static inline void __time_critical_func(increment_head)( microdrive_index_t which )
{
  microdrive[which].head_pos++;
  if( microdrive[which].head_pos >= (microdrive[which].cartridge_len_in_blocks * MICRODRIVE_BLOCK_LEN) )
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
 * Whenever the Z80 asks for the microdrive status that can be seen as
 * an indication that the IF1 is going to want to read the tape. On the
 * real device this action will be a poll, "is the data ready? is the
 * data ready? is the data ready?..." For this emulation we can immediately
 * make it ready and respond "yes, ready".
 */
static void __time_critical_func(microdrives_restart)( void )
{
  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    /* Move the to the start of a block */
    while( ( microdrive[m].head_pos % MICRODRIVE_BLOCK_LEN ) != 0  &&
           ( microdrive[m].head_pos % MICRODRIVE_BLOCK_LEN ) != MICRODRIVE_HEAD_LEN )
    {
      increment_head(m);
    }
        
    /* Reset current number of bytes written */
    microdrive[m].transfered = 0;

    /* Set max byte for the block we're now at */
    if( ( microdrive[m].head_pos % MICRODRIVE_BLOCK_LEN ) == 0 )
    {
      /* up to 15 bytes for header blocks */
      microdrive[m].max_bytes = MICRODRIVE_HEAD_LEN;
    }
    else
    {
      /* up to 528 bytes for data blocks */
      microdrive[m].max_bytes = MICRODRIVE_HEAD_LEN + MICRODRIVE_DATA_LEN + 1;
    }
  }

}


/*
 * Control port in - Z80 is reading IF1/microdrives status
 */
inline uint8_t __time_critical_func(port_ctr_in)( void )
{
  uint8_t ret = 0xff;

  microdrive_index_t active_microdrive_index;
  if( (active_microdrive_index=query_active_microdrive()) == NO_ACTIVE_MICRODRIVE )
  {
    blip_test_pin(); /* "Can't happen" with IF1 ROM code */
  }
  else 
  {
    if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].cartridge_inserted )
    {
      /* Calculate the block under the head */
      /* max_bytes is the number of bytes which can be read from the current block */
      int block = microdrive[active_microdrive_index].head_pos / 543 + ( microdrive[active_microdrive_index].max_bytes == 15 ? 0 : 256 );

      /* pream might be an array of flags, one for each block, indicating something... */
      /* Original comment suggests formatted? Of the block? */
      if( microdrive[active_microdrive_index].pream[block] == SYNC_OK )         /* if formatted */
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
      if( microdrive[active_microdrive_index].cartridge_write_protect == true )
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
inline void __time_critical_func(port_ctr_out)( uint8_t val )
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
    uint8_t any_motor_on = false;
    for( microdrive_index_t m = LAST_MICRODRIVE_INDEX; m > 0; m-- )
    {
      microdrive[m].motor_on = microdrive[m - 1].motor_on;
      any_motor_on |= microdrive[m].motor_on;
    }
    microdrive[0].motor_on = (val & 0x01) ? false : true;
    any_motor_on |= microdrive[0].motor_on;

#if 0
    extern const uint8_t LED_PIN;
    gpio_put( LED_PIN, any_motor_on );
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
inline uint8_t __time_critical_func(port_mdr_in)( void )
{
  uint8_t ret = 0xff;

  microdrive_index_t active_microdrive_index;
  if( (active_microdrive_index=query_active_microdrive()) == NO_ACTIVE_MICRODRIVE )
  {
    blip_test_pin(); blip_test_pin();
    return ret;  /* "Can't happen" with IF1 ROM code */
  }

  if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].cartridge_inserted )
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
inline void __time_critical_func(port_mdr_out)( uint8_t val )
{
  microdrive_index_t active_microdrive_index;
  if( (active_microdrive_index=query_active_microdrive()) == NO_ACTIVE_MICRODRIVE )
  {
    blip_test_pin(); blip_test_pin(); blip_test_pin();
    return;  /* "Can't happen" with IF1 ROM code */
  }

  if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].cartridge_inserted )
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

      microdrive[active_microdrive_index].cartridge_data_modified = true;
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
