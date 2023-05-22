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

#ifndef __FSM_H
#define __FSM_H

#include <stdint.h>
#include "pico/sync.h"

struct _fsm;
typedef void (*fsm_state_entry_fn_t)( struct _fsm * );


typedef enum _fsm_state
{
  FSM_STATE_NONE,

  FSM_STATE_LAST
}
fsm_state_t;


typedef enum _fsm_stimulus
{
  FSM_STIMULUS_NONE,
  FSM_STIMULUS_YES,
  FSM_STIMULUS_TIMEOUT,

  FSM_STIMULUS_LAST
}
fsm_stimulus_t;


/*
 * This binds a state to a piece of code which runs when that state is
 * entered. One function per state at the moment.
 */
typedef struct _fsm_state_entry_fn_binding_t
{
  fsm_state_t          state;
  fsm_state_entry_fn_t entry_fn;  
}
fsm_state_entry_fn_binding_t;


/*
 * This maps the state transitions. When in 'state', if 'stimulus'
 * appears, transit to 'dest_state'.
 */
typedef struct _fsm_map
{
  fsm_state_t                  state;
  fsm_stimulus_t               stimulus;
  fsm_state_t                  dest_state;
}
fsm_map_t;


#define STIMULUS_QUEUE_EMPTY -1
#define STIMULUS_QUEUE_DEPTH 10
typedef struct _fsm
{
  uint16_t                      id;
  fsm_state_t                   current_state;
  fsm_map_t                    *map;
  fsm_state_entry_fn_binding_t *binding;
  void                         *fsm_data;

  critical_section_t           *stimulus_critical_section;
  int8_t                        stimulus_queue_head;
  fsm_stimulus_t                pending_stimulus[STIMULUS_QUEUE_DEPTH];
}
fsm_t;


fsm_t *create_fsm( fsm_map_t*, fsm_state_entry_fn_binding_t*, fsm_state_t, void *fsm_data );
void   process_fsms( void );
void   generate_stimulus( fsm_t*, fsm_stimulus_t );

#endif
