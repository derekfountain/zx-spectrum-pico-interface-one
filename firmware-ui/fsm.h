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


typedef struct _fsm_map
{
  fsm_state_t          state;
  fsm_stimulus_t       stimulus;
  fsm_state_t          dest_state;
  fsm_state_entry_fn_t entry_fn;
}
fsm_map_t;


typedef struct _fsm
{
  uint16_t     id;
  fsm_state_t  current_state;
  fsm_map_t   *map;
  void        *fsm_data;

  fsm_stimulus_t pending_stimulus;
}
fsm_t;


fsm_t *create_fsm( fsm_map_t*, fsm_state_t, void *fsm_data );
void   process_fsms( void );
void   generate_stimulus( fsm_t*, fsm_stimulus_t );

#endif
