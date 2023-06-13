/*
 * ZX Pico IF1 Firmware, a Raspberry Pi Pico based ZX Interface One emulator
 * Copyright (C) 2023 Derek Fountain
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

#include <stdlib.h>

#include "fsm.h"
#include "gui_fsm.h"
#include "gui.h"
#include "microdrive.h"
#include "cartridge.h"
#include "live_microdrive_data.h"
#include "work_queue.h"
#include "sd_card.h"

status_screen_t status;


/*
 * Initialise the GUI. The OLED is cleared as part of its configuration
 * so that doesn't need doing here.
 */
void gui_sm_init( fsm_t *fsm )
{
  status.selected = 0;
  status.requesting_data_from_microdrive = -1;

  generate_stimulus( fsm, ST_BUILTIN_YES );
}


/*
 * Pick up live data showing what's going on with the drives and draw it
 * on OLED screen. For the time being I'm not protecting this with the
 * mutex on the assumption that it's display-only and if things are
 * changing they'll update again momentarily.
 */
void gui_sm_show_status( fsm_t *fsm )
{
  /*
   * Pick up the live data which tells us what the current state is, and convert it to
   * a status structure which can be presented on the screen 
   */

  live_microdrive_data_t *live_microdrive_data = (live_microdrive_data_t*)fsm->fsm_data;

  for( microdrive_index_t microdrive_index = 0; microdrive_index < NUM_MICRODRIVES; microdrive_index++ )
  {
    status.md_inserted[microdrive_index] = (live_microdrive_data->currently_inserted[microdrive_index].status == LIVE_STATUS_INSERTED);
  }

  if( live_microdrive_data->currently_inserted[status.selected].status == LIVE_STATUS_NO_CARTRIDGE )
  {
    status.filename        = NULL;
    status.num_blocks      = 0;
    status.write_protected = false;
    status.inserting       = false;
  }
  else if( live_microdrive_data->currently_inserted[status.selected].status == LIVE_STATUS_INSERTING )
  {
    status.inserting       = true;
  }
  else if( live_microdrive_data->currently_inserted[status.selected].status == LIVE_STATUS_INSERTED )
  {
    status.filename        = live_microdrive_data->currently_inserted[status.selected].filename;
    status.num_blocks      = live_microdrive_data->currently_inserted[status.selected].cartridge_data_length / MICRODRIVE_BLOCK_LEN;
    status.write_protected = live_microdrive_data->currently_inserted[status.selected].write_protected;
    status.inserting       = false;
  }

  draw_status_screen( &status );
}


void gui_sm_show_eject_screen( fsm_t *fsm )
{
  draw_eject_screen( &status );
  
  /* No transition from here, we wait until the user clicks a button */
}


#define MAX_NUM_FILENAMES 25
static uint8_t *filenames[MAX_NUM_FILENAMES+1] = { "0.mdr", 
						   "1.mdr", 
						   "2.mdr", 
						   "3.mdr", 
						   "4.mdr", 
						   "5.mdr", 
						   "6.mdr", 
						   "7.mdr", 
						   "8.mdr", 
						   "9.mdr", 
						   "10.mdr", 
						   "11.mdr", 
						   "12.mdr", 
						   "13.mdr", 
						   "14.mdr",
						   NULL};

static uint32_t selected_filename_index = 0;
static uint32_t num_filenames_read;
void gui_sm_show_insert_screen( fsm_t *fsm )
{
  num_filenames_read = 15; //read_directory_files( &filenames[selected_filename_index],
  //		      MAX_NUM_FILENAMES );

  /* Fill in bottom of the array so it doesn't print rubbish off the bottom of the list */
//  filenames[num_filenames_read] = NULL;

  draw_insert_screen( selected_filename_index, &filenames[0] );
}


void gui_sm_selecting_next_file( fsm_t *fsm )
{
  if( selected_filename_index < num_filenames_read-1 )
  {
    selected_filename_index++;
    draw_insert_screen( selected_filename_index, &filenames[0] );
  }
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


void gui_sm_selecting_previous_file( fsm_t *fsm )
{
  if( selected_filename_index > 0 )
  {
    selected_filename_index--;
    draw_insert_screen( selected_filename_index, &filenames[0] );
  }
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


void gui_sm_action_insert( fsm_t *fsm )
{
  
}


void gui_sm_cancel_insert( fsm_t *fsm )
{
  clear_insert_screen();  
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


/*
 * Cartridge is being inserted. Data is being copied across to the IO Pico.
 */
void gui_sm_inserting_mdr( fsm_t *fsm )
{
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


/*
 * Microdrive has been inserted, update the GUI. This doesn't need to do
 * anything at the moment, it can just drop through and the FSM will
 * arrive back at the show status state which will update the screen.
 */
void gui_sm_inserted_mdr( fsm_t *fsm )
{
  /* Insertion routine will have updated live microdrive data, just advance */

  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


/*
 * The UI needs to move the microdrive selection icon to the next one.
 */
void gui_sm_selecting_next_md( fsm_t *fsm )
{
  if( ++status.selected == NUM_MICRODRIVES )
    status.selected = 0;

  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


/*
 * The UI needs to move the microdrive selection icon to the previous one.
 */
void gui_sm_selecting_previous_md( fsm_t *fsm )
{
  if( status.selected-- == 0 )
    status.selected = NUM_MICRODRIVES-1;

  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


void gui_sm_requesting_status( fsm_t *fsm )
{
  status.requesting_status = true;
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


void gui_sm_requesting_status_done( fsm_t *fsm )
{
  status.requesting_status = false;
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


void gui_sm_requesting_data_to_save( fsm_t *fsm )
{
  live_microdrive_data_t *live_microdrive_data = (live_microdrive_data_t*)fsm->fsm_data;

  status.requesting_data_from_microdrive = live_microdrive_data->microdrive_saving_to_sd;
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


void gui_sm_data_saved( fsm_t *fsm )
{
  live_microdrive_data_t *live_microdrive_data = (live_microdrive_data_t*)fsm->fsm_data;

  /*
   * This would be expected to assign -1 since the live data should show that
   * nothing is being saved to SD card now
   */
  status.requesting_data_from_microdrive = -1;

  generate_stimulus( fsm, ST_BUILTIN_YES );  
}


void gui_sm_insert_sd_card( fsm_t *fsm )
{
}
void gui_sm_eject_sd_card( fsm_t *fsm )
{
}
void gui_sm_scroll_inserted_filename( fsm_t *fsm )
{
}

/*
 * One of the microdrives has been selected. i.e. button pressed when the
 * active microdrive cursor is on one of the icons.
 */
void gui_sm_md_selected( fsm_t *fsm )
{
  live_microdrive_data_t *live_microdrive_data = (live_microdrive_data_t*)fsm->fsm_data;

  if( live_microdrive_data->currently_inserted[status.selected].status == LIVE_STATUS_NO_CARTRIDGE )
  {
    /* No cartridge inserted, let's offer to insert one */
    generate_stimulus( fsm, ST_ACTION_INSERT );  
  }
  else if( live_microdrive_data->currently_inserted[status.selected].status == LIVE_STATUS_INSERTED )
  {
    /* Cartridge is inserted, let's offer to eject it */
    generate_stimulus( fsm, ST_ACTION_EJECT );  
  }
  else
  {
    /* Cartridge is currently inserting, nothing makes much sense so ignore */
    generate_stimulus( fsm, ST_BUILTIN_INVALID );  
  }
}


/*
 * User has confirmed they want to eject
 */
void gui_sm_action_eject( fsm_t *fsm )
{
  live_microdrive_data_t *live_microdrive_data = (live_microdrive_data_t*)fsm->fsm_data;

  work_eject_mdr_data_t *eject_data_ptr = malloc( sizeof(work_eject_mdr_data_t) );
  eject_data_ptr->microdrive_index = status.selected;
  insert_work( WORK_EJECT_MDR, eject_data_ptr );
  
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}

void gui_sm_action_test( fsm_t *fsm )
{
  status.test_value = 100;
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}

void gui_sm_cancel_test( fsm_t *fsm )
{
  status.test_value = 200;
  generate_stimulus( fsm, ST_BUILTIN_YES );  
}

      

#include "gui_fsm.gen.c"


fsm_state_entry_fn_binding_t *query_gui_fsm_binding( void )
{
  return binding;
}

fsm_map_t *query_gui_fsm_map( void )
{
  return gui_fsm_map;
}


gui_fsm_state_t query_gui_fsm_initial_state( void )
{
  return STATE_GUI_INIT;
}



