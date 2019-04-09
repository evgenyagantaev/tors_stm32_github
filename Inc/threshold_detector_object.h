#ifndef THRESHOLD_DETECTOR_OBJECT
#define THRESHOLD_DETECTOR_OBJECT
#include "main.h"
#include "threshold_detector_interface.h"

#define THRESHOLD_AVERAGING_WINDOW_LENGTH 150
#define THRESHOLD_DETECTOR_VALUE 2000

static uint16_t threshold_detector_averaging_window[THRESHOLD_AVERAGING_WINDOW_LENGTH]; 

static uint16_t average_pressure;


uint8_t shift_average_window();
uint16_t calculate_average_pressure();





#endif
