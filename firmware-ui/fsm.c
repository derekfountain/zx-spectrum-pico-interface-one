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
#include "pico/sync.h"
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
  if( fsm == NULL )
    panic("Out of memory for FSM");    

  fsm->id                        = next_fsm_id;
  fsm->current_state             = initial_state;
  fsm->map                       = map;
  fsm->binding                   = binding;
  fsm->fsm_data                  = fsm_data;
  fsm->stimulus_critical_section = malloc( sizeof(critical_section_t) );
  fsm->stimulus_queue_head       = STIMULUS_QUEUE_EMPTY;
  fsm->stimulus_queue_high_water = fsm->stimulus_queue_head;

  /* Use ID as index */
  fsm_list[next_fsm_id++] = fsm;

  /*
   * Critical section protects the FSM from incoming stimulus clashing with
   * processing of existing stimulus
   */
  if( fsm->stimulus_critical_section == NULL )
  {
    panic("Out of memory for FSM critical section");    
  }
  critical_section_init( fsm->stimulus_critical_section );
  
  num_active_fsms++;

  /* Find and call the entry function for the intial state if there is one */
  uint32_t binding_index = 0;
  do
  {
    if( fsm->binding[binding_index].state == initial_state )
    {
      if( fsm->binding[binding_index].entry_fn != NULL )
      {
	(fsm->binding[binding_index].entry_fn)(fsm);
      }
      break;
    }
  } while( fsm->binding[++binding_index].state != FSM_STATE_NONE );

  return fsm;
}


void process_fsms( void )
{
  for( uint32_t fsm_index = 0; fsm_index < num_active_fsms; fsm_index++ )
  {
    fsm_t *fsm = fsm_list[fsm_index];

    if( fsm->stimulus_queue_head != STIMULUS_QUEUE_EMPTY )
    {
      fsm_map_t *map = fsm->map;

      /*
       * Pick up the stimulus. It's always claimed and consumed, if it turns out there's
       * no transition to be made due to this stimulus arriving while we're in the
       * current state, the stimulus is dropped on the floor.
       *
       * Critical section is to protect against an ISR adding a new stimulus at exactly
       * this moment and interfering with the queue head counter.
       */
      critical_section_enter_blocking( fsm->stimulus_critical_section );
      fsm_stimulus_t incoming_stimulus = fsm->pending_stimulus[fsm->stimulus_queue_head];
      fsm->stimulus_queue_head--;
      critical_section_exit( fsm->stimulus_critical_section );

      /* Work down the transitions map looking for one which says a state transition is required */
      while( map && map->state != FSM_STATE_NONE )
      {
	/*
	 * Is this map entry describing the required transition? i.e. it describes the
	 * state we're in, and the stimulus received?
	 */
	bool required_transition = (fsm->current_state == map->state) && (incoming_stimulus == map->stimulus);

	if( required_transition )
	{
	  /* This map entry represents the transition we want to make, new state is achieved */
	  fsm->current_state = map->dest_state;

	  /* Find and call the entry function for the new state if there is one */
	  uint32_t binding_index = 0;
	  do
	  {
	    if( fsm->binding[binding_index].state == map->dest_state )
	    {
	      if( fsm->binding[binding_index].entry_fn != NULL )
	      {
		(fsm->binding[binding_index].entry_fn)(fsm);
	      }
	      /* Don't break here, there might be multiple entry functions */
	    }
	  } while( fsm->binding[++binding_index].state != FSM_STATE_NONE );

	  /*
	   * Break? Or not? With a large number of FSMs, not breaking here
	   * would cause this FSM to hog the processor if a sequence of
	   * stimuli come in. Unlikely, and breaking here adds overhead in
	   * the path of just getting back to this point. But break I do.
	   */
	  break;

	}
	map++;
      }
    }
  }

}

void generate_stimulus( fsm_t *fsm, fsm_stimulus_t stim )
{
  if( fsm->stimulus_queue_head == STIMULUS_QUEUE_DEPTH )
  {
    /* Stimulus would get lost, not sure what else I can do? */
    panic("Stimulus queue full");    
  }
  else
  {
    /*
     * This routine can be called by either core or an ISR. I
     * need to protect the stimulus queue from concurrent
     * modification
     */
    critical_section_enter_blocking( fsm->stimulus_critical_section );
    fsm->pending_stimulus[++fsm->stimulus_queue_head] = stim;
    if( fsm->stimulus_queue_high_water < fsm->stimulus_queue_head )
    {
      fsm->stimulus_queue_high_water = fsm->stimulus_queue_head;
    }
    critical_section_exit( fsm->stimulus_critical_section );
  }
}
