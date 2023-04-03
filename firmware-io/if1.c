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

#include "libspectrum.h"
#include "if1.h"

/* This file is generated from test data, don't change what's in it manually */
#include "flash_images.h"

#define NO_ACTIVE_MICRODRIVE  ((microdrive_index_t)(-1))
#define NUM_MICRODRIVES       ((microdrive_index_t)(8))
#define LAST_MICRODRIVE_INDEX (NUM_MICRODRIVES-1)

/*
 * Details of an MDR image in flash. At the moment the address points to
 * the tape bytes. This will eventually be the complete MDR image sent
 * from SD card
 */
typedef struct _flash_mdr_image
{
  void     *flash_address;
  uint32_t  length;
}
flash_mdr_image_t;

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
flash_mdr_image_t flash_mdr_image[NUM_MICRODRIVES] =
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
 * There are 8 cartridge data images in flash. Only one of them can
 * be in RAM at once. This holds the index of the one that's in RAM.
 */
flash_mdr_image_index_t flash_image_loaded;

#define NO_FLASH_IMAGE  ((flash_mdr_image_index_t)(-1))

/*
 * These are the microdrives, typically 8 of them. This structure
 * keeps track of the motor, the head position, gaps, preambles,
 * things like that.
 */
static microdrive_t microdrive[NUM_MICRODRIVES];

/*
 * Byte array representing the tape, approx 97KB for a 180 sector
 * tape. About 135KB for a 254 sector tape.
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
static tape_byte_t cartridge_data[LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH];
static uint8_t     cartridge_data_modified;

/* Clock bit in the ULA */
static uint8_t if1_ula_comms_clk;

extern const uint8_t LED_PIN;
extern const uint8_t TEST_OUTPUT_GP;


void __time_critical_func(microdrives_restart)( void );


static void __time_critical_func(blib_test_pin)( void )
{
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
 * Reset the microdrive structures.
 */
static void microdrives_reset( void )
{
  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    microdrive[m].inserted   = 0;
    microdrive[m].modified   = 0;
    microdrive[m].head_pos   = 0;
    microdrive[m].motor_on   = 0;
    microdrive[m].gap        = 15;
    microdrive[m].sync       = 15;
    microdrive[m].transfered = 0;
  }

  if1_ula_comms_clk     = 0;
}


/*
 * Initialise Interface One structure. Create the 8 microdrive images.
 * All start off with no cartridge inserted.
 */
int32_t if1_init( void )
{
  int m, i;

  if1_ula_comms_clk = 0;

  /* 
   * There's only one microdrive image in RAM. I don't have enough RAM to
   * malloc more than one. The "not currently being used" images are held
   * in flash and "paged" in.
   */
  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    if( (microdrive[m].cartridge = malloc( sizeof(cartridge_t) )) == NULL )
      return -1;

    microdrive[m].cartridge->write_protect           = 0;
    microdrive[m].cartridge->cartridge_len_in_blocks = 0;
  }

  microdrives_reset();

  flash_image_loaded = NO_FLASH_IMAGE;

  return 0;
}


typedef struct
{
  uint32_t     flash_offset;       /* Area of flash from XIP_BASE which gets erased */
  size_t       erase_length;
  tape_byte_t *src_address;        /* Area from RAM which gets programmed into flash */
  size_t       program_length;
}
erase_trace_entry_t;

static erase_trace_entry_t erase_trace[10];
static uint32_t erase_trace_index = 0;

static int32_t __time_critical_func(unload_flash_mdr_image)( void )
{
  if( cartridge_data_modified == 0 )
    return 0;

  if( flash_image_loaded == NO_FLASH_IMAGE )
    return 0;

  /* Write tape image back out to flash - erase flash first */
  uint32_t offset = (uint32_t)(((uint8_t*)(flash_mdr_image[flash_image_loaded].flash_address)) - XIP_BASE);
  size_t   bytes  = (size_t)(((flash_mdr_image[flash_image_loaded].length +
					                              FLASH_SECTOR_SIZE) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE);

  /*
   * With just the erase, /IORQ lasts about 450ms
   *  Pre-erase starts about 2us from /IORQ low
   *  Post-erase comes in about 8.25ms from /IORQ high
   * With just the program, /IORQ lasts about 65ms
   *  Pre-program comes in about 2.5us from /IORQ low
   *  Post-program comes in about 9ms from /IORQ high
   * With both, /IORQ lasts about 625ms
   *  Pre-erase starts about 2us from /IORQ low
   *  Post-program comes in about 9ms from /IORQ high
   *
   * All timings at 150MHz. The erase is the slow bit, programming is much faster.
   * The "bytes" recalculation explains the difference between the two singular
   * data points and the whole lot in one go.
   * 
   * Running flash_range_erase() and flash_range_program() a second time without
   * reseting the Pico doesn't change anything, so they must be running from RAM.
   */
#define DISABLED_FLASH_WRITE_BACK 1
/* Disabled, it takes far too long and crashes the Z80 which is in /WAIT */
#if !DISABLED_FLASH_WRITE_BACK
  flash_range_erase( offset, bytes );

  erase_trace[erase_trace_index].flash_offset = offset;
  erase_trace[erase_trace_index].erase_length = bytes;

  /* Now write out the current tape contents */
  bytes  = (size_t)(((flash_mdr_image[flash_image_loaded].length+FLASH_PAGE_SIZE) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE);
  flash_range_program( offset, cartridge_data, bytes );

  erase_trace[erase_trace_index].src_address    = cartridge_data;
  erase_trace[erase_trace_index].program_length = bytes;
  erase_trace_index++;
#endif

  /* I don't actually clear the cartridge RAM memory, just the metadata */
  flash_image_loaded      = NO_FLASH_IMAGE;
  cartridge_data_modified = 0;

  return 0;
}


/*
 * Load one of the tape images from flash into the RAM buffer.
 * This updates the 'flash_image_loaded' static variable.
 */
static int32_t __time_critical_func(load_flash_tape_image)( flash_mdr_image_index_t which )
{
  /* Check requested image exists */
  if( (which < 0) || (which > LAST_MICRODRIVE_INDEX) )
    return -1;

  /* Check requested image will fit in buffer */
  if( flash_mdr_image[which].length > LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH )
    return -1;

  gpio_put(LED_PIN, 1);

  /* If change of tape image in memory, write current tape data back out to flash */
  if( (flash_image_loaded != NO_FLASH_IMAGE) && (flash_image_loaded != which) )
  {
    if( unload_flash_mdr_image() == -1 )
      return -1;
  }

  /*
   * Copy data in from flash. What's currently at the address is the tape data, but
   * that might change as what's held in flash develops
   */
  memcpy( cartridge_data, (tape_byte_t*)flash_mdr_image[which].flash_address, flash_mdr_image[which].length );
  flash_image_loaded      = which;
  cartridge_data_modified = 0;

  gpio_put(LED_PIN, 0);

  return which;
}




int32_t if1_mdr_insert( const microdrive_index_t which, const uint8_t load_data )
{
  size_t length_in_bytes = flash_mdr_image[which].length - ( flash_mdr_image[which].length % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN );

  microdrive[which].cartridge->cartridge_len_in_blocks = (length_in_bytes / LIBSPECTRUM_MICRODRIVE_BLOCK_LEN);
  microdrive[which].inserted = 1;
  microdrive[which].modified = 0;

  if( load_data )
  {
    if( load_flash_tape_image( which ) == -1 )
      return -1;
  }

  /*
   * pream is 512 bytes in the microdrive_t structure.
   * This is filling 2 areas of the microdrive area's premable with SYNC_OK.
   * The loop is over the number of blocks on the cartridge.
   * Not quite sure what it's doing, maybe marking some sort of sector map?
   */
  for( size_t i = microdrive[which].cartridge->cartridge_len_in_blocks; i > 0; i-- )
    microdrive[which].pream[255 + i] = microdrive[which].pream[i-1] = SYNC_OK;

  microdrives_restart();

  return 0;
}


/*
 * Advance the given microdrive's head position on the tape.
 * Wrap at the "end" of the tape.
 */
static inline void __time_critical_func(increment_head)( microdrive_index_t which )
{
  microdrive[which].head_pos++;
  if( microdrive[which].head_pos >= (microdrive[which].cartridge->cartridge_len_in_blocks * LIBSPECTRUM_MICRODRIVE_BLOCK_LEN) )
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
 * emulation we can immediately mkae it ready and respond "yes, ready".
 */
void __time_critical_func(microdrives_restart)( void )
{
  for( microdrive_index_t m=0; m<NUM_MICRODRIVES; m++ )
  {
    /* FIXME Surely it's possible to calculate where the head needs to move to? */
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
    blib_test_pin();
    return ret;  /* "Can't happen" */
  }

  /* This routine doesn't access the tape data, no need to page that */

  int block;

  if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].inserted )
  {
    /* Calculate the block under the head */
    /* max_bytes is the number of bytes which can be read from the current block */
    block = microdrive[active_microdrive_index].head_pos / 543 + ( microdrive[active_microdrive_index].max_bytes == 15 ? 0 : 256 );

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
      if( microdrive[active_microdrive_index].gap )
      {
	microdrive[active_microdrive_index].gap--;
      }
      else
      {
	ret &= 0xf9; /* GAP and SYNC low (both are active low) */

	if( microdrive[active_microdrive_index].sync )
	{
	  microdrive[active_microdrive_index].sync--;
	}
	else
	{
	  microdrive[active_microdrive_index].gap = 15;
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
    if( microdrive[active_microdrive_index].cartridge->write_protect )
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
  if( ((val & 0x02) == 0) && (if1_ula_comms_clk == 1) )
  {
    /* Falling edge of the clock on bit 0x02  ~~\__ */

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
    for( microdrive_index_t m = LAST_MICRODRIVE_INDEX; m > 0; m-- )
    {
      microdrive[m].motor_on = microdrive[m - 1].motor_on;
    }
    microdrive[0].motor_on =(val & 0x01) ? 0 : 1;
  }

  /* Note the level of the CLK line so we can see what it's done next time */
  if1_ula_comms_clk = ( val & 0x02 ) ? 1 : 0;

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
    blib_test_pin(); blib_test_pin();
    return ret;  /* "Can't happen" */
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
      if( flash_image_loaded != active_microdrive_index )
      {
	if( load_flash_tape_image( active_microdrive_index ) == -1 )
	{
	  blib_test_pin(); blib_test_pin(); blib_test_pin();
	  return 0xFF;     /* "Can't happen" and there really aren't any good options here */
	}
      }

      ret = cartridge_data[microdrive[active_microdrive_index].head_pos];

      /* Move tape on, with wrap */
      increment_head(active_microdrive_index);
    }
    else
    {
      // What happens here and why? I think it means we've gone off the end of the block, so gap tape
    }

    /*
     * transfered is a count of the number of bytes transferred from
     * the block to the IF1
     */
    microdrive[active_microdrive_index].transfered++;
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

  microdrive_index_t active_microdrive_index;
  if( (active_microdrive_index=query_active_microdrive()) == NO_ACTIVE_MICRODRIVE )
  {
    blib_test_pin(); blib_test_pin(); blib_test_pin(); blib_test_pin();
    return;  /* "Can't happen" */
  }

  if( microdrive[active_microdrive_index].motor_on && microdrive[active_microdrive_index].inserted )
  {
    /* Calculate the block under the head */
    /* max_bytes is the number of bytes which can be read from the current block */
    block = microdrive[active_microdrive_index].head_pos / 543 + ( microdrive[active_microdrive_index].max_bytes == 15 ? 0 : 256 );

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
    if( microdrive[active_microdrive_index].transfered > 11 && microdrive[active_microdrive_index].transfered < microdrive[active_microdrive_index].max_bytes + 12 )
    {

      if( flash_image_loaded != active_microdrive_index )
      {
	if( load_flash_tape_image( active_microdrive_index ) == -1 )
	{
	  blib_test_pin(); blib_test_pin(); blib_test_pin(); blib_test_pin(); blib_test_pin();
	  return;     /* "Can't happen" */
	}
      }

      cartridge_data[microdrive[active_microdrive_index].head_pos] = val;
      cartridge_data_modified = 1;
      increment_head(active_microdrive_index);

      microdrive[active_microdrive_index].modified = 1; // Unused for now
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
