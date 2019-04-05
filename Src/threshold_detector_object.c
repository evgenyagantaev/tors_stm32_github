#include "threshold_detector_object.h"
#include "threshold_detector_interface.h"
#include "ring_buffer_interface.h"
#include "pressure_sensor_interface.h"


uint8_t threshold_detector_initialization()
{


	int counter = 0;

	while(counter < 3000)
	{
		uint16_t pressure = pressure_sensor_get_sample();
		if(pressure != 0)
		{
			ring_buffer_add_sample(pressure);
			shift_average_window();
			threshold_detector_averaging_window[THRESHOLD_AVERAGING_WINDOW_LENGTH-1] =  pressure;
			counter++;
		}
	}


	average_pressure = calculate_average_pressure();
}



uint8_t threshold_detector_action()
{
	uint16_t last_pressure_sample = ring_buffer_read_sample();

	average_pressure = calculate_average_pressure();


	if(last_pressure_sample != 0)
	{
		if(abs((int)((int)average_pressure - (int)last_pressure_sample)) > THRESHOLD_DETECTOR_VALUE)
		{
			ring_buffer_set_start_marker();
			ring_buffer_set_registration_flag();
		}
	}



	shift_average_window();
	threshold_detector_averaging_window[THRESHOLD_AVERAGING_WINDOW_LENGTH-1] =  last_pressure_sample;


}



uint8_t shift_average_window()
{
	int i;
	for(i=0; i<(THRESHOLD_AVERAGING_WINDOW_LENGTH-1); i++)
	{
		threshold_detector_averaging_window[i] = threshold_detector_averaging_window[i+1];
	}

}


uint16_t calculate_average_pressure()
{
	uint32_t aver = 0;
	int i;
	for(i=0; i<THRESHOLD_AVERAGING_WINDOW_LENGTH; i++)
	{
		aver += threshold_detector_averaging_window[i];
	}

	 aver /= THRESHOLD_AVERAGING_WINDOW_LENGTH;
	 return (uint16_t)aver;

}



