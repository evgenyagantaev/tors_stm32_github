#include "main.h"
#include "ring_buffer_object.h"
#include "ring_buffer_interface.h"

uint8_t ring_buffer_add_sample(uint16_t pressure)
{
	ring_buffer_array[ring_buffer_write_index % RING_BUFFER_LENGTH] = pressure;
	ring_buffer_write_index++;

	if(ring_buffer_registration_flag)  // we are registering impulse data
	{
		if((ring_buffer_write_index % RING_BUFFER_LENGTH) == (ring_buffer_start_marker % RING_BUFFER_LENGTH))  // we completed full loop af data writing
			return 1;
		else
			return 0;
	}
	else
		return 0;


}

uint16_t ring_buffer_read_sample()
{
	return ring_buffer_array[ring_buffer_write_index % RING_BUFFER_LENGTH - 1];
}


uint8_t ring_buffer_set_start_marker()
{
	ring_buffer_start_marker = ring_buffer_write_index - RING_BUFFER_PRE_ACTION_LENGTH;
}


uint8_t ring_buffer_set_registration_flag()
{
	ring_buffer_registration_flag = 1;
}

uint8_t ring_buffer_get_registration_flag()
{
	return ring_buffer_registration_flag;
}

uint8_t	ring_buffer_dump()
{
	char message[64];
	int i;

	for(i=0; i<RING_BUFFER_LENGTH; i++)
	{

		sprintf(message, "%d\r\n", ring_buffer_array[(ring_buffer_start_marker+i)%RING_BUFFER_LENGTH]);
		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	}
}


