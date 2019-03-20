#ifndef RING_BUFFER_OBJECT_H
#define RING_BUFFER_OBJECT_H

#include "main.h"

#define RING_BUFFER_LENGTH 3000
#define RING_BUFFER_PRE_ACTION_LENGTH 1000

static uint16_t ring_buffer_array[RING_BUFFER_LENGTH];
static uint32_t ring_buffer_write_index;
static uint32_t ring_buffer_read_index;
static uint32_t ring_buffer_start_marker;

static uint8_t 	ring_buffer_registration_flag = 0;
#endif
