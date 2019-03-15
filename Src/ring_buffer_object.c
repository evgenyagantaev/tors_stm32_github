#include "main.h"
#include "ring_buffer_object.h"
#include "ring_buffer_interface.h"

uint8_t ring_buffer_add_sample(uint16_t pressure)
{
	ring_buffer_array[ring_buffer_write_index % RING_BUFFER_LENGTH] = pressure;
	ring_buffer_write_index++;

	return 0;
}

uint16_t ring_buffer_read_sample()
{
	return ring_buffer_array[0];
}
