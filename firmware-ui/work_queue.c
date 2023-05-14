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

#include "pico/util/queue.h"
#include "work_queue.h"

#define QUEUE_DEPTH     250

static queue_t q;

typedef struct _work_queue_entry
{
  work_queue_type_t work_type;
  void             *work_data;
}
work_queue_entry_t;



void work_queue_init( void )
{
  queue_init( &q, sizeof(work_queue_entry_t), QUEUE_DEPTH );
}


bool work_queue_is_empty( void )
{
  return queue_is_empty( &q );
}


/*
 * I'm not quite sure how to handle this... If a piece of work takes
 * a long time, such as sending an MDR image, a number of request
 * status items will be added via the timer - 2 per second. There's
 * no point collecting all these up and running them all at once
 * when the MDR send completes. So I'm specifically checking here
 * to see if the last work item was a request status and if it was
 * I'm not adding the new one to the queue if that's also a request
 * status. I need to watch for the queue emptying. If it does, and
 * the last item was a request status, no more request statuses will
 * be added. To guard against that the last added type is reset when
 * the queue empties.
 *
 * This seems like a hack, but I'm not sure how else to handle it.
 */
static work_queue_type_t last_added_type = WORK_NULL;

void insert_work( work_queue_type_t type, void *data )
{
  work_queue_entry_t entry;

  if( (type == WORK_REQUEST_STATUS) && (last_added_type == WORK_REQUEST_STATUS) )
    return;

  entry.work_type = type;
  entry.work_data = data;

  queue_add_blocking( &q, &entry );

  last_added_type = type;

  return;
}


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
    last_added_type = WORK_NULL;
    return false;
  }
}
