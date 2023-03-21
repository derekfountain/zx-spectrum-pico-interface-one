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


#endif
