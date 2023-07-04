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
#include <string.h>

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

  switch( live_microdrive_data->currently_inserted[status.selected].cartridge_error )
  {
  case CARTRIDGE_ERR_CHECKSUM_INCORRECT:
    status.error_str = "E_C_CHECKSUM";
    break;

  case CARTRIDGE_ERR_NEED_EJECT_BEFORE_INSERT:
    status.error_str = "E_EJECT_FIRST";
    break;

  case CARTRIDGE_ERR_NEED_SAVE_BEFORE_INSERT:
    status.error_str = "E_SAVE_FIRST";
    break;

  default:
    status.error_str = NULL;
  }

  if( status.error_str == NULL )
  {
    switch( live_microdrive_data->currently_inserted[status.selected].gui_error )
    {
    case GUI_ERR_CHECKSUM_INCORRECT:
      status.error_str = "E_S_CHECKSUM";
      break;

    case GUI_ERR_FILE_NOT_FOUND:
      status.error_str = "E_NO_FILE";
      break;
      
    case GUI_ERR_FILE_EMPTY:
      status.error_str = "E_FILE_EMPTY";
      break;

    case GUI_ERR_NOT_EVEN_BLOCKS:
      status.error_str = "E_BAD_BLOCKS";
      break;

    case GUI_ERR_FILE_TOO_SMALL:
      status.error_str = "E_FILE_TOO_SMALL";
      break;

    case GUI_ERR_FILE_TOO_LARGE:
      status.error_str = "E_FILE_TOO_LARGE";
      break;

    default:
      status.error_str = NULL;
    }
  }

  status.sd_card_inserted = live_microdrive_data->sd_card_inserted;

  draw_status_screen( &status );
}


void gui_sm_show_eject_screen( fsm_t *fsm )
{
  draw_eject_screen( &status );
  
  /* No transition from here, we wait until the user clicks a button */
}


/*
 * Keep an array of pointers to filenames, as fetched by the SD card
 * directory-reading library code. The array is filled when the insert
 * screen is loaded, and cleared out when the insert screen is cleared
 * (either because a file was chosen or the screen was cancelled).
 *
 * Each pointer holds a malloc'ed filename buffer. Not ideal, but the
 * option is either a large static array or to somehow dynamically
 * present the contents of the SD card directory on demand. This is
 * much easier and since it's the only place in the whole program
 * which uses malloc/free I wouldn't expect it to fragment too much.
 *
 * FIXME What if the card is ejected while the file list is displayed?
 * That could realistically happen if the user decides the SD card
 * doesn't contain the file they want.
 */
#define MAX_NUM_FILENAMES 1024           /* This many pointers */
static uint8_t *filenames[MAX_NUM_FILENAMES+1];

static uint32_t selected_filename_index = 0;
static uint32_t num_filenames_read;
void gui_sm_show_insert_screen( fsm_t *fsm )
{
  for( uint32_t i=0; i<MAX_NUM_FILENAMES; i++ )
    filenames[i] = NULL;

  num_filenames_read = read_directory_files( &filenames[0],
					     MAX_NUM_FILENAMES );

  if( num_filenames_read == 0 )
  {
    generate_stimulus( fsm, ST_BUILTIN_NO );  
  }
  else
  {
    /* Fill in bottom of the array so it doesn't print rubbish off the bottom of the list */
    filenames[num_filenames_read] = NULL;

    draw_insert_screen( selected_filename_index, &filenames[0] );
  }
}


void gui_sm_no_files( fsm_t *fsm )
{
  draw_no_files_screen();

  generate_stimulus( fsm, ST_BUILTIN_YES );  
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


/*
 * This array of reusable structures duplicates the one in zx_pico_if1_ui_fw.c.
 * It would better to keep just the one array somewhere where both bits of
 * code can access it. But it's a trivial issue.
 */
static work_insert_mdr_t insert_data[NUM_MICRODRIVES];
void gui_sm_action_insert( fsm_t *fsm )
{
  work_insert_mdr_t *work_ptr = &insert_data[status.selected];

  work_ptr->microdrive_index = status.selected;
  strncpy( work_ptr->filename, filenames[selected_filename_index], MAX_INSERT_FILENAME_LEN );

  insert_work( WORK_INSERT_MDR, work_ptr );

  clear_insert_screen();  

  for( uint32_t i=0; i<MAX_NUM_FILENAMES; i++ )
  {
    if( filenames[i] )
    {
      free( filenames[i] );
      filenames[i] = NULL;
    }
  }

  generate_stimulus( fsm, ST_BUILTIN_YES );    
}


void gui_sm_cancel_insert( fsm_t *fsm )
{
  clear_insert_screen();  

  for( uint32_t i=0; i<MAX_NUM_FILENAMES; i++ )
  {
    if( filenames[i] )
    {
      free( filenames[i] );
      filenames[i] = NULL;
    }
  }

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
static work_eject_mdr_data_t eject_data[NUM_MICRODRIVES];
void gui_sm_action_eject( fsm_t *fsm )
{
  live_microdrive_data_t *live_microdrive_data = (live_microdrive_data_t*)fsm->fsm_data;

  work_eject_mdr_data_t *eject_data_ptr = &eject_data[status.selected];
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



