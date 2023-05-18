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
#include "oled_display.h"

void gui_sm_show_status( void *fsm_data )
{
  status_screen_t status;

  status.md_inserted[0] = 1;
  status.md_inserted[1] = 1;
  status.md_inserted[2] = 1;
  status.md_inserted[3] = 1;
  status.md_inserted[4] = 0;
  status.md_inserted[5] = 0;
  status.md_inserted[6] = 0;
  status.md_inserted[7] = 0;
  status.selected       = 7;
  draw_status_screen( &status );
}

static fsm_map_t gui_fsm_map[] =
{
  {STATE_GUI_INIT, FSM_STIMULUS_YES, STATE_GUI_SHOW_STATUS, gui_sm_show_status},

  {FSM_STATE_NONE, FSM_STIMULUS_YES, FSM_STATE_NONE, NULL}
};

fsm_map_t *query_gui_fsm_map( void )
{
  return gui_fsm_map;
}


gui_fsm_state_t query_gui_fsm_initial_state( void )
{
  return STATE_GUI_INIT;
}

