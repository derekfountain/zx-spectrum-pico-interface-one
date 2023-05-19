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

#include <stdint.h>
#include <malloc.h>
#include "pico/platform.h"
#include "fsm.h"

/* I only need one so far.. */
#define MAX_NUM_FSMS 1

static fsm_t *fsm_list[MAX_NUM_FSMS];

static uint16_t next_fsm_id     = 0;
static uint16_t num_active_fsms = 0;

fsm_t *create_fsm( fsm_map_t *map, fsm_state_entry_fn_binding_t *binding, fsm_state_t initial_state, void *fsm_data )
{
  if( num_active_fsms == MAX_NUM_FSMS )
    panic("Out of FSMs");

  fsm_t *fsm = malloc( sizeof(fsm_t) );

  fsm->id               = next_fsm_id;
  fsm->current_state    = initial_state;
  fsm->map              = map;
  fsm->binding          = binding;
  fsm->fsm_data         = fsm_data;
  fsm->pending_stimulus = FSM_STIMULUS_NONE;

  /* Use ID as index, ewww */
  fsm_list[next_fsm_id++] = fsm;
  
  num_active_fsms++;

  return fsm;
}


void process_fsms( void )
{
  for( uint32_t fsm_index = 0; fsm_index < num_active_fsms; fsm_index++ )
  {
    fsm_t *fsm = fsm_list[fsm_index];

    if( fsm->pending_stimulus != FSM_STIMULUS_NONE )
    {
      fsm_map_t *map = fsm->map;
      while( map && map->state != FSM_STATE_NONE )
      {
	if( (fsm->current_state == map->state) && (fsm->pending_stimulus == map->stimulus) )
	{
	  /* This map entry represents the transition we want to make */
	  fsm->pending_stimulus = FSM_STIMULUS_NONE;

	  /* Find and call the entry function for the destination state if there is one */
	  uint32_t binding_index = 0;
	  do
	  {
	    if( fsm->binding[binding_index].state == map->dest_state )
	    {
	      if( fsm->binding[binding_index].entry_fn != NULL )
	      {
		(fsm->binding[binding_index].entry_fn)(fsm);
	      }
	      break;
	    }
	  } while( fsm->binding[++binding_index].state != FSM_STATE_NONE );

	  /* New state is achieved */
	  fsm->current_state = map->dest_state;
	}
	map++;
      }
    }
  }

}

void generate_stimulus( fsm_t *fsm, fsm_stimulus_t stim )
{
  if( fsm->pending_stimulus != FSM_STIMULUS_NONE )
  {
    /* I think this is OK, stimulus will get lost, not sure what else I can do */
  }

// Going to need to lock something here? What if a new stim comes in from interrupt?
  fsm->pending_stimulus = stim;
}
