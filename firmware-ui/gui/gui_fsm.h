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

#ifndef __GUI_FSM_H
#define __GUI_FSM_H

#include "fsm.h"


typedef enum
{
  STATE_GUI_INIT            = FSM_STATE_LAST,
  STATE_GUI_SHOW_STATUS,
  STATE_GUI_INSERTING_MDR,
  STATE_GUI_INSERTED_MDR,
  STATE_GUI_SELECTING_NEXT_MD,
  STATE_GUI_SELECTING_PREVIOUS_MD,
}
gui_fsm_state_t;


typedef enum
{
  ST_MDR_INSERTING          = FSM_STIMULUS_LAST,
  ST_MDR_INSERTED,
  ST_ROTATE_CCW,
  ST_ROTATE_CW,
}
gui_fsm_stimulus_t;


fsm_map_t                    *query_gui_fsm_map( void );
gui_fsm_state_t               query_gui_fsm_initial_state( void );
fsm_state_entry_fn_binding_t *query_gui_fsm_binding( void );

#endif
