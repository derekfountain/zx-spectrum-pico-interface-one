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

static microdrive_t microdrive[NUM_MICRODRIVES];		/* We have 8 microdrive */
static if1_ula_t if1_ula;

#define MDR_IN(m) microdrive[m - 1].inserted
#define MDR_WP(m) libspectrum_microdrive_write_protect( microdrive[m - 1].cartridge )



int
if1_init( void *context )
{
  int m, i;

#if 0
  if1_ula.fd_r = -1;
  if1_ula.fd_t = -1;
  if1_ula.dtr = 0;		/* No data terminal yet */
  if1_ula.cts = 2;		/* force to emit first cts status */
#endif
  if1_ula.comms_clk = 0;
  if1_ula.comms_data = 0; /* really? */
#if 0
  if1_ula.fd_net = -1;
  if1_ula.s_net_mode = 1;
  if1_ula.net = 0;
  if1_ula.esc_in = 0; /* empty */
#endif

  for( m = 0; m < NUM_MICRODRIVES; m++ ) {
    microdrive[m].cartridge = libspectrum_microdrive_alloc();
    microdrive[m].inserted = 0;
    microdrive[m].modified = 0;
  }

#if 0  
  if( settings_current.rs232_rx ) {
    if1_plug( settings_current.rs232_rx, 1 );
    libspectrum_free( settings_current.rs232_rx );
    settings_current.rs232_rx = NULL;
  }

  if( settings_current.rs232_tx ) {
    if1_plug( settings_current.rs232_tx, 2 );
    libspectrum_free( settings_current.rs232_tx );
    settings_current.rs232_tx = NULL;
  }

  if( settings_current.snet ) {
    if1_plug( settings_current.snet, 3 );
    libspectrum_free( settings_current.snet );
    settings_current.snet = NULL;
  }

  module_register( &if1_module_info );

  if1_memory_source = memory_source_register( "If1" );
  for( i = 0; i < MEMORY_PAGES_IN_8K; i++ )
    if1_memory_map_romcs[i].source = if1_memory_source;

  periph_register( PERIPH_TYPE_INTERFACE1, &if1_periph );
  periph_register_paging_events( event_type_string, &page_event,
				 &unpage_event );
#endif

  return 0;
}

int
if1_mdr_insert( int which, const char *filename )
{
  microdrive_t *mdr;
  int m, i;

  if( which == -1 ) {	/* find an empty one */
    for( m = 0; m < NUM_MICRODRIVES; m++ ) {
      if( !microdrive[m].inserted ) {
        which = m;
	break;
      }
    }
  }

  if( which == -1 ) {
#if 0
    ui_error( UI_ERROR_ERROR,
	      "Cannot insert cartridge '%s', all Microdrives in use",
	      filename );
#endif
    return 1;
  }

  if( which >= NUM_MICRODRIVES ) {
#if 0
    ui_error( UI_ERROR_ERROR, "if1_mdr_insert: unknown drive %d", which );
#endif
    return 1;
  }

  mdr = &microdrive[ which ];

#if 0
  /* Eject any cartridge already in the drive */
  if( mdr->inserted ) {
    /* Abort the insert if we want to keep the current cartridge */
    if( if1_mdr_eject( which ) ) return 0;
  }
#endif

#if 0
  if( filename == NULL ) {	/* insert new unformatted cartridge */
    if1_mdr_new( mdr );
    update_menu( UMENU_MDRV1 + which );
    return 0;
  }
#endif

#if 0
  if( utils_read_file( filename, &mdr->file ) ) {
    ui_error( UI_ERROR_ERROR, "Failed to open cartridge image" );
    return 1;
  }

  if( libspectrum_microdrive_mdr_read( mdr->cartridge, mdr->file.buffer,
				       mdr->file.length ) ) {
    utils_close_file( &mdr->file );
    ui_error( UI_ERROR_ERROR, "Failed to open cartridge image" );
    return 1;
  }

  utils_close_file( &mdr->file );
#endif

  mdr->file.buffer = test_image_32blk_mdr;
  mdr->file.length = test_image_32blk_mdr_len;
  libspectrum_microdrive_mdr_read( mdr->cartridge,
				   mdr->file.buffer,
				   mdr->file.length );

  mdr->inserted = 1;
  mdr->modified = 0;
#if 0
  mdr->filename = utils_safe_strdup( filename );
#endif
  /* we assume formatted cartridges */
  for( i = libspectrum_microdrive_cartridge_len( mdr->cartridge );
	i > 0; i-- )
    mdr->pream[255 + i] = mdr->pream[i-1] = SYNC_OK;

#if 0
  update_menu( UMENU_MDRV1 + which );
#endif

  return 0;
}

static void
increment_head( int m )
{
  microdrive[m].head_pos++;
  if( microdrive[m].head_pos >=
      libspectrum_microdrive_cartridge_len( microdrive[m].cartridge ) *
      LIBSPECTRUM_MICRODRIVE_BLOCK_LEN )
    microdrive[m].head_pos = 0;
}

static void
microdrives_restart( void )
{
  int m;

  for( m = 0; m < NUM_MICRODRIVES; m++ ) {
    while( ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != 0  &&
           ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) != LIBSPECTRUM_MICRODRIVE_HEAD_LEN )
      increment_head( m ); /* put head in the start of a block */
	
    microdrive[m].transfered = 0; /* reset current number of bytes written */

    if( ( microdrive[m].head_pos % LIBSPECTRUM_MICRODRIVE_BLOCK_LEN ) == 0 ) {
      microdrive[m].max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN; /* up to 15 bytes for header blocks */
    } else {
      microdrive[m].max_bytes = LIBSPECTRUM_MICRODRIVE_HEAD_LEN + LIBSPECTRUM_MICRODRIVE_DATA_LEN + 1; /* up to 528 bytes for data blocks */
    }
  }	
}


/*
 * Handler for control byte input from microdrives.
 *
 * This produces the status byte for reading port 0xEF (239) - the gap, sync and write protect flags.
 */
libspectrum_byte
port_ctr_in( void )
{
  libspectrum_byte ret = 0xff;
  int m, block;

  for( m = 0; m < NUM_MICRODRIVES; m++ ) {

    microdrive_t *mdr = &microdrive[ m ];

    if( mdr->motor_on && mdr->inserted ) {
      block = mdr->head_pos / 543 + ( mdr->max_bytes == 15 ? 0 : 256 );
      if( mdr->pream[block] == SYNC_OK ) {	/* if formatted */
	if( mdr->gap ) {
	/* ret &= 0xff;  GAP and SYNC high ? */
	  mdr->gap--;
        } else {
	  ret &= 0xf9; /* GAP and SYNC low */
	  if( mdr->sync ) {
	    mdr->sync--;
	  } else {
	    mdr->gap = 15;
	    mdr->sync = 15;
	  }
        }
      }
      /* if write protected */
      if( libspectrum_microdrive_write_protect( mdr->cartridge) )
	ret &= 0xfe; /* active bit */
    }
  }

// Still don't understand what this is doing here, but I can't reset
// everything now that this routine is called constantly
// Not sure this is going to work out...
//
//  microdrives_restart();

  return ret;
}


/*
 * Handler for control byte output from Z80 to IF1, a write to port 0xEF (239).
 * Carries CLK, ERASE and R/W.
 */
void
port_ctr_out( libspectrum_byte val )
{
  int m;

  if( !( val & 0x02 ) && ( if1_ula.comms_clk ) ) {	/* ~~\__ */

    for( m = NUM_MICRODRIVES-1; m > 0; m-- ) {
      /* Rotate one drive */
      microdrive[m].motor_on = microdrive[m - 1].motor_on;
    }
    microdrive[0].motor_on = (val & 0x01) ? 0 : 1;

    if( microdrive[0].motor_on )
    {
      gpio_put(LED_PIN, 1);
    }
    else
    {
      gpio_put(LED_PIN, 0);
    }


#if 0
    if( microdrive[0].motor_on || microdrive[1].motor_on || 
	microdrive[2].motor_on || microdrive[3].motor_on ||
	microdrive[4].motor_on || microdrive[5].motor_on ||
	microdrive[6].motor_on || microdrive[7].motor_on ) {
      if( !if1_mdr_status ) {
	ui_statusbar_update( UI_STATUSBAR_ITEM_MICRODRIVE,
	                     UI_STATUSBAR_STATE_ACTIVE );
	if1_mdr_status = 1;
      }
    } else if ( if1_mdr_status ) {
      ui_statusbar_update( UI_STATUSBAR_ITEM_MICRODRIVE,
			   UI_STATUSBAR_STATE_INACTIVE );
      if1_mdr_status = 0;
    }
#endif
  }

#if 0
  if( val & 0x01 ) {	/* comms_data == 1 */
    /* Interface 1 service manual p.:1.4 par.: 1.5.1
       The same pin on IC1 (if1 ULA), pin 33, is used for the network
       transmit data and for the RS232 transmit data. In order to select
       the required function IC1 uses its COMMS_OUT (pin 30) signal,
       borrowed from the microdrive control when the microdrive is not
       being used (I do not know what it is exactly meaning. It is a
       hardware not being used, or a software should not use..?) This signal
       is routed from pin 30 to the emitter of transistor Q3 (RX DATA) and
       via resistor R4, to the base of transistor Q1 (NET). When COMMS_OUT
       is high Q3 is enabled this selecting RS232, and when it is low
       Q1 is enabled selecting the network.
	 
       OK, the schematics offer a different interpretation, because if
       COMMS_OUT pin level high (>+3V) then Q3 is off (the basis cannot
       be more higher potential then emitter (NPN transistor), so whatever
       is on the IC1 RX DATA (pin 33) the basis of Q4 is always on +12V,
       so the collector of Q4 (PNP transistor) always on -12V.
       If COMMS_OUT pin level goes low (~0V), then Q3 basis (connected
       to IC1 RX DATA pin) can be higher level (>+3V) than emitter, so
       the basis potential of Q4 depend on IC1 RX DATA.
	 
       OK, Summa summarum I assume that, the COMMS OUT pin is a
       negated output of the if1 ULA CTR register's COMMS DATA bit. 
    */
    /* C_DATA = 1 */
    if( if1_ula.comms_data == 0 ) {
      if1_ula.count_out = 0;
      if1_ula.data_out = 0;
      if1_ula.count_in = 0;
      if1_ula.data_in = 0;
    }
  }
  if1_ula.wait = ( val & 0x20 ) ? 1 : 0;
  if1_ula.comms_data = ( val & 0x01 ) ? 1 : 0;
#endif
  if1_ula.comms_clk = ( val & 0x02 ) ? 1 : 0;
#if 0
  val = ( val & 0x10 ) ? 1 : 0;
  if( settings_current.rs232_handshake && 
      if1_ula.fd_t != -1 && if1_ula.cts != val ) {
    char data = val ? 0x03 : 0x02;
    do {} while( write( if1_ula.fd_t, "", 1 ) != 1 );
    do {} while( write( if1_ula.fd_t, &data, 1 ) != 1 );
  }
  if1_ula.cts = val;
    
#ifdef IF1_DEBUG_NET
  fprintf( stderr, "Set CTS to %d, set WAIT to %d and COMMS_DATA to %d\n",
	   if1_ula.cts, if1_ula.wait, if1_ula.comms_data );
#endif
#endif

  microdrives_restart();
}



libspectrum_byte
port_mdr_in( void )
{
  libspectrum_byte ret = 0xff;
  int m;

  for( m = 0; m < NUM_MICRODRIVES; m++ ) {

    microdrive_t *mdr = &microdrive[ m ];

    if( mdr->motor_on && mdr->inserted ) {

      if( mdr->transfered < mdr->max_bytes ) {
	mdr->last = libspectrum_microdrive_data( mdr->cartridge,
						   mdr->head_pos );
	increment_head( m );
      }

      mdr->transfered++;
      ret &= mdr->last;  /* I assume negative logic, but how know? */
    }

  }

  return ret;
}



void
port_mdr_out( libspectrum_byte val )
{
  int m, block;

  /* allow access to the port only if motor 1 is ON and there's a file open */
  for( m = 0; m < NUM_MICRODRIVES; m++ ) {

    microdrive_t *mdr = &microdrive[ m ];
 
    if( mdr->motor_on && mdr->inserted ) {
#ifdef IF1_DEBUG_MDR
      fprintf(stderr, "#%05d  %03d(%03d): 0x%02x\n",
    			mdr->head_pos, mdr->transfered, mdr->max_bytes, val );
#endif
      block = mdr->head_pos / 543 + ( mdr->max_bytes == 15 ? 0 : 256 );
      if( mdr->transfered == 0 && val == 0x00 ) {	/* start pream */
        mdr->pream[block] = 1;
      } else if( mdr->transfered > 0 && mdr->transfered < 10 && val == 0x00 ) {
        mdr->pream[block]++;
      } else if( mdr->transfered > 9 && mdr->transfered < 12 && val == 0xff ) {
        mdr->pream[block]++;
      } else if( mdr->transfered == 12 && mdr->pream[block] == 12 ) {
        mdr->pream[block] = SYNC_OK;
      }
      if( mdr->transfered > 11 &&
	  mdr->transfered < mdr->max_bytes + 12 ) {
 
	libspectrum_microdrive_set_data( mdr->cartridge, mdr->head_pos,
 					 val );
 	increment_head( m );
	mdr->modified = 1;
      }
      mdr->transfered++;
    }
  }
}
