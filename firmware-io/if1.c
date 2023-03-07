#include "libspectrum.h"
#include "if1.h"

#include "test_image.h"

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

static microdrive_t microdrive[8];		/* We have 8 microdrive */
static if1_ula_t if1_ula;

#define MDR_IN(m) microdrive[m - 1].inserted
#define MDR_WP(m) libspectrum_microdrive_write_protect( microdrive[m - 1].cartridge )


int
if1_mdr_insert( int which, const char *filename )
{
  microdrive_t *mdr;
  int m, i;

  if( which == -1 ) {	/* find an empty one */
    for( m = 0; m < 8; m++ ) {
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

  if( which >= 8 ) {
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

  mdr->file.buffer = ___resources_test_image_mdr;
  mdr->file.length = ___resources_test_image_mdr_len;
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

  for( m = 0; m < 8; m++ ) {
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

libspectrum_byte
port_ctr_in( void )
{
  libspectrum_byte ret = 0xff;
  int m, block;

  for( m = 0; m < 8; m++ ) {

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
  microdrives_restart();

  return ret;
}
