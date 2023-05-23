#!/usr/bin/perl -w

# ZX Pico IF1 Firmware, a Raspberry Pi Pico based ZX Interface One emulator
# Copyright (C) 2023 Derek Fountain
# 
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

use strict;

use File::Basename;

if( @ARGV != 1 )
{
  print("Usage process_fsm.pl <fsm.def>\n");
  exit(-1);
}

my $fsm_file = $ARGV[0];

open( FSM_HANDLE, "<$fsm_file" ) or die("Unable to open file $fsm_file\n");
my $fsm_def;
while( my $line = <FSM_HANDLE> )
{
  if( $line !~ /^\s*\#/ )
  {
    $fsm_def.= $line;
  }
}
close( FSM_HANDLE );

# Create the names
#
my $states_enum_name   = basename($fsm_file); $states_enum_name   =~ s/\.def$/_state_t/;       $states_enum_name =~ s/\./_/g;
my $stimulus_enum_name = basename($fsm_file); $stimulus_enum_name =~ s/\.def$/_stimulus_t/;    $stimulus_enum_name =~ s/\./_/g;
my $fsm_h_inc_filename = $fsm_file;           $fsm_h_inc_filename =~ s/\.def$/\.gen\.h/;
my $fsm_c_inc_filename = $fsm_file;           $fsm_c_inc_filename =~ s/\.def$/\.gen.c/;


open( FSM_H_INC_HANDLE, ">$fsm_h_inc_filename" ) or die("Unable to write to $fsm_h_inc_filename\n");

print FSM_H_INC_HANDLE <<STATES_ENUM_HDR;
typedef enum
{
STATES_ENUM_HDR

# States enum
#
my $number = 100;
while( $fsm_def =~ /STATE\s+(\w+)/gs )
{
  my $state = $1;

  my $suffix = "            = $number";
  $number++;
  print FSM_H_INC_HANDLE ( sprintf("  %-50s%s,\n", $state, $suffix ) );
}

print FSM_H_INC_HANDLE <<STATES_ENUM_FTR;
}
$states_enum_name;
STATES_ENUM_FTR




open( FSM_C_INC_HANDLE, ">$fsm_c_inc_filename" ) or die("Unable to write to $fsm_c_inc_filename\n");

# Bindings array (the entry functions)
#
print FSM_C_INC_HANDLE <<BINDINGS_HDR;
/* State to entry function bindings */
static fsm_state_entry_fn_binding_t binding[] = 
{
BINDINGS_HDR

while( $fsm_def =~ /STATE\s+(\w+)\s+ENTRY\s+(.+?)TRANSITIONS/gs )
{
  my $state = $1;
  my @entry_fns = split( /\s+/, $2 );
  die("Multiple entry functions not supported yet\n") if( @entry_fns > 1 );

  print FSM_C_INC_HANDLE ( sprintf("  { %-50s, %-50s },\n", $state, $entry_fns[0] ) );
}

print FSM_C_INC_HANDLE <<BINDINGS_FTR;

  { FSM_STATE_NONE, NULL },
};
BINDINGS_FTR


# Transitions: find the state, then produce a line for state,stimulus,destinationstate
#
print FSM_C_INC_HANDLE <<TRANSITIONS_HDR;
/* Map of states, stimulus, and destination */
static fsm_map_t gui_fsm_map[] =
{
TRANSITIONS_HDR

my %stimulus_hash = ();
while( $fsm_def =~ /STATE\s+(\w+)\s+ENTRY\s+(.+?)\s+TRANSITIONS\s+(.*?)END_TRANSITIONS/gs )
{
  my $state = $1;
  my $transitions_list = $3;

  while( $transitions_list =~ /\s*(\w+)\s+(\w+)\s*/gs )
  {
    my $stimulus          = $1;
    my $destination_state = $2;

    $stimulus_hash{$stimulus}  = 1;
    print FSM_C_INC_HANDLE ( sprintf("  { %-50s, %-50s, %-50s },\n", $state, $stimulus, $destination_state) );
  }
}

print FSM_C_INC_HANDLE <<TRANSITIONS_FTR;

  {FSM_STATE_NONE, ST_BUILTIN_NONE, FSM_STATE_NONE }
};
TRANSITIONS_FTR


# Stimulus
#
print FSM_H_INC_HANDLE <<STIMULUS_ENUM_HDR;
typedef enum
{
STIMULUS_ENUM_HDR

$number = 100;
foreach my $stimulus (keys(%stimulus_hash))
{
  if( $stimulus !~ /^ST_BUILTIN_/ )
  {
    my $suffix = "            = $number";
    $number++;
    print FSM_H_INC_HANDLE ( sprintf("  %-50s%s,\n", $stimulus, $suffix ) );
  }
}


print FSM_H_INC_HANDLE <<STIMULUS_ENUM_FTR;
}
$stimulus_enum_name;
STIMULUS_ENUM_FTR


close( FSM_H_INC_HANDLE );
close( FSM_C_INC_HANDLE );
