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

/*
 * The work queue is a structure representing work which has to be done.
 * This means large work items, not button presses and stuff like that.
 * Examples include inserting a cartridge, saving a microdrive image
 * out to SD card, things like that. These things take time so core1's
 * sole function is to deal with them. It gets its work from this queue.
 */

#include "pico/util/queue.h"
#include "work_queue.h"

#define QUEUE_DEPTH     25

static queue_t q;

/*
 * Command pattern, a work queue entry is a type of work and a
 * pointer to a structure which holds the data which describes
 * the work to do.
 */
typedef struct _work_queue_entry
{
  work_queue_type_t work_type;
  void             *work_data;
}
work_queue_entry_t;


/*
 * This implementation uses the Pico SDK's queue structure,
 * which is multi-core safe.
 */
void work_queue_init( void )
{
  queue_init( &q, sizeof(work_queue_entry_t), QUEUE_DEPTH );
}


bool work_queue_is_empty( void )
{
  return queue_is_empty( &q );
}


void insert_work( work_queue_type_t type, void *data )
{
  work_queue_entry_t entry;

  entry.work_type = type;
  entry.work_data = data;

  queue_add_blocking( &q, &entry );

  return;
}


/*
 * Remove work, returns true if there was work on the queue.
 * Details will be at the pointers provided, and the work
 * will have been removed from the queue. Quietly returns
 * false if there's nothing to do.
 */
bool remove_work( work_queue_type_t *type, void **data  )
{
  work_queue_entry_t entry;

  if( queue_try_remove( &q, &entry ) )
  {
    *type = entry.work_type;
    *data = entry.work_data;

    return true;
  }
  else
  {
    return false;
  }
}
