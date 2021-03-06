#ifndef RING_BUFFER_INTERFACE_H
#define RING_BUFFER_INTERFACE_H

#include "main.h"

uint8_t ring_buffer_add_sample(uint16_t pressure);
uint16_t ring_buffer_read_sample();

uint8_t ring_buffer_set_start_marker();
uint8_t ring_buffer_set_registration_flag();
uint8_t ring_buffer_get_registration_flag();

uint8_t	ring_buffer_dump(uint32_t start_marker);


#endif
