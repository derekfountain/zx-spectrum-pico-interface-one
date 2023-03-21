#ifndef MICRODRIVE_H
#define MICRODRIVE_H

/*
 * This represents a microdrive cartridge. I'm going to rename it.
 */
struct libspectrum_microdrive
{
  /* Byte array representing the tape, approx 135KB */
  libspectrum_byte data[ LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH ];

  /* Whether this cartridge has its w/p tab removed */
  int write_protect;
  
  /* Length in 543-byte blocks */
  libspectrum_byte cartridge_len;    /* Cartridge length in blocks */
};

/* Not sure if this is needed */
typedef struct libspectrum_microdrive_block {

  libspectrum_byte hdflag;		/* bit0 = 1-head, ( 0 - data ) */
  libspectrum_byte hdbnum;		/* block num 1 -- 254 */
  libspectrum_word hdblen;		/* not used */
  libspectrum_byte hdbnam[11];		/* cartridge label + \0 */
  libspectrum_byte hdchks;		/* header checksum */
 
  libspectrum_byte recflg;		/* bit0 = 0-data, bit1, bit2 */
  libspectrum_byte recnum;		/* data block num  */
  libspectrum_word reclen;		/* block length 0 -- 512 */
  libspectrum_byte recnam[11];		/* record (file) name + \0 */
  libspectrum_byte rechks;		/* descriptor checksum */

  libspectrum_byte data[512];		/* data bytes */
  libspectrum_byte datchk;		/* data checksum */

} libspectrum_microdrive_block;

#endif
