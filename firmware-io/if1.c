#include "hardware/gpio.h"

#include "libspectrum.h"
#include "if1.h"

#include "test_image.h"

extern const uint8_t LED_PIN;

typedef struct utils_file {

  unsigned char *buffer;
  size_t length;

} utils_file;

enum {
  SYNC_NO = 0,
  SYNC_OK = 0xff
};

typedef struct microdrive_t {
  utils_file file;
  char *filename;		/* old filename */
  int inserted;
  int modified;
  int motor_on;
  int head_pos;
  int transfered;
  int max_bytes;
  libspectrum_byte pream[512];	/* preamble/sync area written */
  libspectrum_byte last;
  libspectrum_byte gap;
  libspectrum_byte sync;

  libspectrum_microdrive *cartridge;	/* write protect, len, blocks */

} microdrive_t;

typedef struct if1_ula_t {
  int fd_r;	/* file descriptor for reading bytes or bits RS232 */
  int fd_t;	/* file descriptor for writing bytes or bits RS232 */
  int fd_net;	/* file descriptor for rw bytes or bits SinclairNET */
  int rs232_buffer;	/* read buffer */
  int s_net_mode;
  int status;	/* if1_ula/SinclairNET */
  int comms_data;	/* the previous data comms state */
  int comms_clk;	/* the previous data comms state */
  int cts;	/* CTS of peripheral */
  int dtr;	/* DTR of peripheral */
  int tx;	/* TxD the name is very kind, because this is the read end of
                   the TxD wire of DATA machine (really RxD the view of
		   spectrum */
  int rx;	/* RxD the name is very kind, because this is the write end of
                   the RxD wire of DATA machine (really TxD the view of
		   spectrum */
  int data_in;	/* interpreted incoming data */
  int count_in;
  int data_out; /* interpreted outgoing data */
  int count_out;
  int esc_in;	/* if we compose an escape seq */
  
  int net;	/* Network in/out (really 1 wire bus :-) */
  int net_data;	/* Interpreted network data */
  int net_state;	/* Interpreted network data */
  int wait;	/* Wait state */
  int busy;	/* Indicate busy; if1 software never poll it ... */
} if1_ula_t;

/* Pico only has memory for 1 microdrive, the structure is 135KB. I can use a smaller image */
#define NUM_MICRODRIVES 1

static microdrive_t microdrive;
static if1_ula_t if1_ula;


int
if1_init( void *context )
{
  int m, i;

  if1_ula.comms_clk = 0;
  if1_ula.comms_data = 0; /* really? */

  microdrive.cartridge = libspectrum_microdrive_alloc();
  microdrive.inserted = 0;
  microdrive.modified = 0;

  return 0;
}

int
if1_mdr_insert( int which, const char *filename )
{
  microdrive_t *mdr;
  int m, i;

  mdr = &microdrive;

  mdr->file.buffer = test_image_32blk_mdr;
  mdr->file.length = test_image_32blk_mdr_len;
  libspectrum_microdrive_mdr_read( mdr->cartridge,
				   mdr->file.buffer,
				   mdr->file.length );

  mdr->inserted = 1;
  mdr->modified = 0;

  /*
   * pream is 512 bytes in the microdrive_t structure, the one in this
   * file which represents the microdrive.
   * This is filling 2 areas of the microdrive area's premable with SYNC_OK.
   * The loop is over the number of blocks on the cartridge.
   * Not quite sure what it's doing, maybe marking some sort of sector map?
   */

  /* we assume formatted cartridges */
  for( i = libspectrum_microdrive_cartridge_len( mdr->cartridge );
	i > 0; i-- )
    mdr->pream[255 + i] = mdr->pream[i-1] = SYNC_OK;

  return 0;
}

static void
increment_head( void )
{
  microdrive.head_pos++;
  if( microdrive.head_pos >=
      libspectrum_microdrive_cartridge_len( microdrive.cartridge ) *
      LIBSPECTRUM_MICRODRIVE_BLOCK_LEN )
    microdrive.head_pos = 0;
}

/*
 * I think the idea here is that whenever the Z80 asks for the microdrive
 * status that can be seen as an indication that the IF1 is going to want
 * to read the tape. On the real device this action will be a poll, "is
 * the data ready? is the data ready? is the data ready?..." For this
 * emulation we can immediately mkae it ready and respond "yes, ready".
 */
void
microdrives_restart( void )
{
  while( ( microdrive.head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != 0  &&
	 ( microdrive.head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != LIBSPECTRUM_MICRODRIVE_HEAD_LEN )
    increment_head(); /* put head in the start of a block */
	
    microdrive.transfered = 0; /* reset current number of bytes written */

    if( ( microdrive.head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) == 0 ) {
      microdrive.max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN; /* up to 15 bytes for header blocks */
    } else {
      microdrive.max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN + LIBSPECTRUM_MICRODRIVE_DATA_LEN + 1; /* up to 528 bytes for data blocks */
    }
}


/*
 * Work out the status byte value ready for next time
 * the Z80 asks for it. Pre-calculation seems OK so far.
 */
static libspectrum_byte precalculated_status = 0xff;

void
precalculate_port_ctr_in( void )
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
	ret &= 0xf9; /* GAP and SYNC low */

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
    
    /* if write protected */
    if( libspectrum_microdrive_write_protect( microdrive.cartridge) )
      ret &= 0xfe; /* active bit */
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

  /* Return precalculated value */
  precalculated_status = ret;

  return;
}


/*
 * Control register in, this is the Z80 reading status.
 * Fast path, just return the pre-calculated byte
 */
libspectrum_byte
port_ctr_in( void )
{
  return precalculated_status;
}



/*
 * Control output, which means the IF1 is setting the active drive.
 * If the CLK line is going low set microdrive0 (the only one, now)
 * to motor status as per bit0 (COMMS DATA).
 */
void
port_ctr_out( libspectrum_byte val )
{
  /* Look for a falling edge on the CLK line */
  if( !( val & 0x02 ) && ( if1_ula.comms_clk ) ) {	/* ~~\__ */

    microdrive.motor_on = (val & 0x01) ? 0 : 1;
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
/* This is the tricky one. I can hand back the byte as soon as ret is populated, might need to */
libspectrum_byte
port_mdr_in( void )
{
  libspectrum_byte ret = 0xff;

  if( microdrive.motor_on )
  {
    /*
     * This looks like max_bytes is the number of bytes in the block under
     * the head, and transfered is the number of bytes transferred out of
     * it so far
     */
    if( microdrive.transfered < microdrive.max_bytes )
    {
      /* last is the last byte read from the tape. It's not used anywhere but here */
      ret = libspectrum_microdrive_data( microdrive.cartridge,
					 microdrive.head_pos );
      /* Move tape on, with wrap */
      increment_head();
    }

    /*
     * transfered is a count of the number of bytes transferred from
     * the cartridge to the IF1
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
 * I probably don't need this to work for read-only support. It's not going to work
 * inline, there's too much of it. But it looks like a delayed implementation wll
 * be ok. As long as the OUT instruction completes quickly what happens at the
 * microdrive - this - can take a few ms.
 */
void
port_mdr_out( libspectrum_byte val )
{
#define SUPPORTED_YET 0
#if SUPPORTED_YET
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
       * ...followed by 2 0x00 bytes...
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
      increment_head( 0 );
      microdrive.modified = 1;
    }

    /* transfered does include the preamble */
    microdrive.transfered++;
  }
#endif

}
