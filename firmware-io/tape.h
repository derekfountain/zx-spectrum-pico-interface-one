#ifndef __TAPE_H
#define __TAPE_H

#include <pico/stdlib.h>

uint8_t load_tape( uint8_t microdrive_index );
uint8_t unload_tape( void );
uint8_t query_tape_byte( uint32_t head_pos, uint8_t *value );
uint8_t write_tape_byte( uint32_t head_pos, uint8_t value );

#endif
