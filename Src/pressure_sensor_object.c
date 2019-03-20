#include "pressure_sensor_object.h"
#include "pressure_sensor_interface.h"

//****************************************************************************
uint8_t write_byte(uint8_t data)
{

	uint8_t data_out;
    uint8_t read_data;

	// wait for spi transmitter readiness
	while ((SPI1->SR & SPI_SR_TXE) == RESET );
	data_out = data;
    SPI1->DR = data_out;
    // wait while a transmission complete
	while ((SPI1->SR & SPI_SR_RXNE) == RESET );
    read_data = SPI1->DR;
	
	return read_data;

	
}

//****************************************************************************
uint16_t pressure_sensor_get_sample()
{

    uint8_t least_byte, med_byte, most_byte;
	uint32_t sample;

	// chipselect low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	// read most significant byte ***********

	// wait for spi transmitter readiness
	while ((SPI1->SR & SPI_SR_TXE) == RESET );
    SPI1->DR = 0x55;
    // wait while a transmission complete
	while ((SPI1->SR & SPI_SR_RXNE) == RESET );
    most_byte = SPI1->DR;
	

	// read medium significant byte ***********
	
	// wait for spi transmitter readiness
	while ((SPI1->SR & SPI_SR_TXE) == RESET );
    SPI1->DR = 0x55;
    // wait while a transmission complete
	while ((SPI1->SR & SPI_SR_RXNE) == RESET );
    med_byte = SPI1->DR;
	

	// read least significant byte ***********
	
	// wait for spi transmitter readiness
	while ((SPI1->SR & SPI_SR_TXE) == RESET );
    SPI1->DR = 0x55;
    // wait while a transmission complete
	while ((SPI1->SR & SPI_SR_RXNE) == RESET );
    least_byte = SPI1->DR;
	
	// chipselect high 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);



	// save most significant byte ***********
	sample = ((uint32_t)(most_byte & 0x07))<<16;
	// save medium significant byte ***********
	sample += ((uint32_t)med_byte)<<8;
	// save least significant byte ***********
	sample += least_byte;


	current_pressure = (uint16_t)(sample>>3);
	return (uint16_t)(sample>>3);

	
}






double pressure_sensor_get_pressure()
{
	return current_pressure;
}

