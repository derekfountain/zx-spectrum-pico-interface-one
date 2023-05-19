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

#include "gui_fsm.h"
#include "gui.h"
#include "microdrive.h"
#include "live_microdrive_data.h"
#include "oled_display.h"

status_screen_t status;


/*
 * Initialise the GUI. The OLED is cleared as part of its configuration
 * so that doesn't need doing here.
 */
void gui_sm_init( fsm_t *fsm )
{
  status.selected = 0;

  generate_stimulus( fsm, FSM_STIMULUS_YES );
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
    status.md_inserted[microdrive_index] = (live_microdrive_data->currently_inserted[microdrive_index].filename != NULL);
  }

  draw_status_screen( &status );
}


/*
 * Microdrive has been inserted, update the GUI. This doesn't need to do
 * anything at the moment, it can just drop through and the FSM will
 * arrive back at the show status state which will update the screen.
 */
void gui_sm_insert_mdr( fsm_t *fsm )
{
  /* Insertion routine will have updated live microdrive data, just advance */

  generate_stimulus( fsm, FSM_STIMULUS_YES );  
}


void gui_sm_selecting_next_md( fsm_t *fsm )
{
  if( ++status.selected == NUM_MICRODRIVES )
    status.selected = 0;

  generate_stimulus( fsm, FSM_STIMULUS_YES );  
}


void gui_sm_selecting_previous_md( fsm_t *fsm )
{
  if( status.selected-- == 0 )
    status.selected = NUM_MICRODRIVES-1;

  generate_stimulus( fsm, FSM_STIMULUS_YES );  
}



static fsm_map_t gui_fsm_map[] =
{
  /* This is wrong, I need a table of entry functions per state, not have them hardcoded here */
  {STATE_GUI_INIT,                  FSM_STIMULUS_YES,  STATE_GUI_SHOW_STATUS,           gui_sm_show_status},
  {STATE_GUI_SHOW_STATUS,           ST_MDR_INSERTED,   STATE_GUI_INSERTING_MDR,         gui_sm_insert_mdr },
  {STATE_GUI_INSERTING_MDR,         FSM_STIMULUS_YES,  STATE_GUI_SHOW_STATUS,           gui_sm_show_status},

  {STATE_GUI_SHOW_STATUS,           ST_ROTATE_CCW,     STATE_GUI_SELECTING_NEXT_MD,     gui_sm_selecting_next_md },
  {STATE_GUI_SHOW_STATUS,           ST_ROTATE_CW,      STATE_GUI_SELECTING_PREVIOUS_MD, gui_sm_selecting_previous_md },

  {STATE_GUI_SELECTING_NEXT_MD,     FSM_STIMULUS_YES,  STATE_GUI_SHOW_STATUS,           gui_sm_show_status },
  {STATE_GUI_SELECTING_PREVIOUS_MD, FSM_STIMULUS_YES,  STATE_GUI_SHOW_STATUS,           gui_sm_show_status },


  {FSM_STATE_NONE,          FSM_STIMULUS_NONE, FSM_STATE_NONE,                  NULL}
};


fsm_map_t *query_gui_fsm_map( void )
{
  return gui_fsm_map;
}


gui_fsm_state_t query_gui_fsm_initial_state( void )
{
  return STATE_GUI_INIT;
}

