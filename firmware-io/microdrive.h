#ifndef MICRODRIVE_H
#define MICRODRIVE_H

/*
 * This represents a microdrive cartridge. I'm going to rename it.
 */
struct libspectrum_microdrive
{
  /*
   * Byte array representing the tape, approx 135KB
   *
   * Copy and paste out to a text file with:
   *
   * (gdb) set print repeats 0
   * (gdb) set print elements unlimited
   * (gdb) set pagination off
   * (gdb) p/x microdrive->cartridge.data
   *
   * Convert to MDR image with:
   *
   * > perl -ne 'map { print chr(hex($_)) } split(/, /, $_)' < mm_reformated_in_zx.txt > mm_reformated_in_zx.mdr
   *
   * mm_reformated_in_zx.mdr will then load into FUSE as a normal
   * MDR file.
   */
  libspectrum_byte data[ LIBSPECTRUM_MICRODRIVE_CARTRIDGE_LENGTH ];

  /* Whether this cartridge has its w/p tab removed */
  int write_protect;
  
  /* Length in 543-byte blocks */
  libspectrum_byte cartridge_len;    /* Cartridge length in blocks */
};


#endif
