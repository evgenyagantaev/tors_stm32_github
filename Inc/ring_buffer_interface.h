#ifndef RING_BUFFER_INTERFACE_H
#define RING_BUFFER_INTERFACE_H

#include "main.h"
#include "ring_buffer_object.h"

uint8_t ring_buffer_add_sample(uint16_t pressure);
uint16_t ring_buffer_read_sample();





#endif
