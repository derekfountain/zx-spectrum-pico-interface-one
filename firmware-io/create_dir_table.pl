#!/usr/bin/perl -w
use strict;

open( H, ">./dir_table.inc" ) or die("Can't open ./dir_table.inc\n");

foreach my $rom_read (0,1)
{
  foreach my $iorq (0,1)
  {
    foreach my $rd (0,1)
    {
      print( H, "0b0000".
	     ($rom_read ? "1" : "0").                           # GPIO 27
	     "00000000000000000".
	     ($rd       ? "1" : "0").                           # GPIO 9
	     ($iorq     ? "1" : "0").                           # GPIO 8
	     "00000000" );
    }
  }
}

close( H );
exit( 0 );
