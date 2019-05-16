/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "ring_buffer_interface.h"
//#include "ring_buffer_object.h"
#include "pressure_sensor_interface.h"
//#include "threshold_detector_interface.h"
//#include "threshold_detector_object.h"

/* Private variables ---------------------------------------------------------*/

#define RING_BUFFER_LENGTH 9000
uint16_t ring_buffer_array[RING_BUFFER_LENGTH];
uint16_t ring_buffer_write_index = 0;
#define RING_BUFFER_PRE_ACTION_LENGTH 1000
#define THRESHOLD_DETECTOR_VALUE 2000
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

uint8_t	ring_buffer_dump(uint32_t start_marker)
{

//#define USART_OUT
#define SPI_OUT

#ifdef USART_OUT
	char message[64];
	int i;

	uint32_t average = 0;

	for(i=0; i<RING_BUFFER_PRE_ACTION_LENGTH; i++)
		average += ring_buffer_array[(start_marker+i)%RING_BUFFER_LENGTH];
	average /= RING_BUFFER_PRE_ACTION_LENGTH;

	double coeffitient_kilopaskales = 10480 / 32768.0;

	for(i=0; i<RING_BUFFER_LENGTH; i++)
	{

		sprintf(message, "%6d %5d\r\n", i*11, (int)(((int)(ring_buffer_array[(start_marker+i)%RING_BUFFER_LENGTH] - average))*coeffitient_kilopaskales));
		//sprintf(message, "%6d %5d\r\n", i*21, ring_buffer_array[(start_marker+i)%RING_BUFFER_LENGTH]);
		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
	}
#endif

#ifdef SPI_OUT
	int i;

	uint32_t average = 0;

	for(i=0; i<RING_BUFFER_PRE_ACTION_LENGTH; i++)
		average += ring_buffer_array[(start_marker+i)%RING_BUFFER_LENGTH];
	average /= RING_BUFFER_PRE_ACTION_LENGTH;

	double coeffitient_kilopaskales = 10480 / 32768.0;

	for(i=0; i<RING_BUFFER_LENGTH; i++)
	{
    	SPI2->DR = (int16_t)(((int)(ring_buffer_array[(start_marker+i)%RING_BUFFER_LENGTH] - average))*coeffitient_kilopaskales)                                                                                      
	    // wait while a transmission complete                                                                  
        while ((SPI2->SR & SPI_SR_RXNE) == RESET );                                                            

	}
#endif

}

uint8_t primitive_delay()
{
	uint32_t volatile i;
	for(i=0; i<300000; i++);

	return 0;
}
/* Private function prototypes -----------------------------------------------*/




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int main(void)
{

	int i,j,k;

	char message[256];


  	/* MCU Configuration----------------------------------------------------------*/

  	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  	HAL_Init();

  	/* Configure the system clock */
  	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_SPI1_Init();
	// enable spi1
	SPI1->CR1 |= SPI_CR1_SPE;
	MX_SPI2_Init();
	SPI2->CR2 &= ~SPI_CR2_TXEIE;   // disable txe interrupt
	SPI2->CR2 &= ~SPI_CR2_ERRIE;   // disable error interrupt
	SPI2->CR2 |= SPI_CR2_RXNEIE;   // enable rxne interrupt
	// enable spi2
	SPI2->CR1 |= SPI_CR1_SPE;

  	MX_USART1_UART_Init();
  	
	MX_TIM6_Init();

  
  	HAL_GPIO_WritePin(led0_GPIO_Port, led0_Pin, GPIO_PIN_RESET);

	//threshold_detector_initialization();
	while(ring_buffer_write_index < RING_BUFFER_LENGTH)
	{
		uint16_t pressure = pressure_sensor_get_sample();
		if((pressure >31000) && (pressure < 35000))
		{
			ring_buffer_array[ring_buffer_write_index] = pressure;
			//sprintf(message, "%5d %6d\r\n", ring_buffer_write_index, ring_buffer_array[ring_buffer_write_index]);
			//HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
			ring_buffer_write_index++;
		}
	}

	/*
	//DEBUGDEBUG***************************************************
		// output saved data
		int i;
		for(i=0; i<RING_BUFFER_LENGTH; i++)
		{
			sprintf(message, "%d\r\n", ring_buffer_array[i]);
			sprintf(message, "%5d %6d\r\n", i, ring_buffer_array[i]);
			HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
		}


		    // stop
		    while(1)
		    {
	  	        HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	        primitive_delay();
				HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	        primitive_delay();
		    }
	//DEBUGDEBUG***************************************************
	*/
	
	uint8_t least_byte, med_byte, most_byte;
    uint32_t sample;
    uint16_t pressure;

	// long delay
	for(i=0; i<25; i++)
		primitive_delay();
	/*
  	while (1)
  	{
	    
	    // chipselect low
    	GPIOA->BRR = GPIO_PIN_8 ;
	                                                                      
        // read most significant byte ***********
	                                                                      
	    // wait for spi transmitter readiness
        //while ((SPI1->SR & SPI_SR_TXE) == RESET );
        SPI1->DR = 0x55;
	    // wait while a transmission complete
        while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	    most_byte = SPI1->DR;
        
	                                                                      
	    // read medium significant byte ***********
	    
	    // wait for spi transmitter readiness
        //while ((SPI1->SR & SPI_SR_TXE) == RESET );
        SPI1->DR = 0x55;
	    // wait while a transmission complete
        while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	    med_byte = SPI1->DR;
        
	                                                                      
	    // read least significant byte ***********
	    
	    // wait for spi transmitter readiness
        //while ((SPI1->SR & SPI_SR_TXE) == RESET );
        SPI1->DR = 0x55;
	    // wait while a transmission complete
        while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	    least_byte = SPI1->DR;
	    
	    // chipselect high 
    	GPIOA->BSRR = GPIO_PIN_8 ;
                                                                          
                                                                          
	                                                                      
	    // save most significant byte ***********
	    sample = ((uint32_t)(most_byte & 0x07))<<16;
	    // save medium significant byte ***********
	    sample += ((uint32_t)med_byte)<<8;
	    // save least significant byte ***********
        sample += least_byte;
                                                                          
	                                                                      
        pressure = (uint16_t)(sample>>2);

		ring_buffer_array[ring_buffer_write_index % RING_BUFFER_LENGTH] = pressure;
		ring_buffer_write_index++;

		if(abs((int)((int)ring_buffer_array[(ring_buffer_write_index-250) % RING_BUFFER_LENGTH] - (int)pressure)) > THRESHOLD_DETECTOR_VALUE)
		{


			int i;
	
			uint32_t start_marker = ring_buffer_write_index - RING_BUFFER_PRE_ACTION_LENGTH;

			for(i=0; i<(RING_BUFFER_LENGTH - RING_BUFFER_PRE_ACTION_LENGTH); i++)
			{
	   			// chipselect low                                     
    	        GPIOA->BRR = GPIO_PIN_8 ;
	                                                                      
                // read most significant byte ***********
	                                                                      
	            // wait for spi transmitter readiness
                //while ((SPI1->SR & SPI_SR_TXE) == RESET );
                SPI1->DR = 0x55;
	            // wait while a transmission complete
                while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	            most_byte = SPI1->DR;
                
	                                                                      
	            // read medium significant byte ***********
	            
	            // wait for spi transmitter readiness
                //while ((SPI1->SR & SPI_SR_TXE) == RESET );
                SPI1->DR = 0x55;
	            // wait while a transmission complete
                while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	            med_byte = SPI1->DR;
                
	                                                                      
	            // read least significant byte ***********
	            
	            // wait for spi transmitter readiness
                //while ((SPI1->SR & SPI_SR_TXE) == RESET );
                SPI1->DR = 0x55;
	            // wait while a transmission complete
                while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	            least_byte = SPI1->DR;
	            
	            // chipselect high 
    	        GPIOA->BSRR = GPIO_PIN_8 ;
                                                                          
                                                                          
	                                                                      
	            // save most significant byte ***********
	            sample = ((uint32_t)(most_byte & 0x07))<<16;
	            // save medium significant byte ***********
	            sample += ((uint32_t)med_byte)<<8;
	            // save least significant byte ***********
                sample += least_byte;
                                                                          
	                                                                      
                pressure = (uint16_t)(sample>>2);                    
				
				ring_buffer_array[ring_buffer_write_index % RING_BUFFER_LENGTH] = pressure;
				ring_buffer_write_index++;
			}
		        	
					
	  	    // blink****************************************
			HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    HAL_Delay(500);
	  	    HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    HAL_Delay(500);
			HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    HAL_Delay(500);
	  	    HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    HAL_Delay(500);
			// **********************************************
			// output saved data
		    ring_buffer_dump(start_marker);


		    // stop
		    while(1)
		    {
	  	        HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	        HAL_Delay(500);
	  	        HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	        HAL_Delay(500);
		    }
		}

		//sprintf(message, "%d\r\n", pressure);
		//HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);

		
  	}
	//*/



	//*                                                                    	
	// DEBUGDEBUGDEBUG  full speed ADC reading                             	
	while(1)                              	                             	
	{                                                                      	                            	
		
		
		// check comparator
		if((GPIOA->IDR & GPIO_PIN_2) != (uint32_t)GPIO_PIN_RESET)
		{ 
		
		
			// chipselect low                                                     	                                      
            GPIOA->BRR = GPIO_PIN_8 ;                                          	                                
	                                                                                                               
            // read most significant byte ***********                                                              
	                                                                                                               
	        // wait for spi transmitter readiness                                                                  
            //while ((SPI1->SR & SPI_SR_TXE) == RESET );                                                           
            SPI1->DR = 0x55;                                                                                       
	        // wait while a transmission complete                                                                  
            while ((SPI1->SR & SPI_SR_RXNE) == RESET );                                                            
	        most_byte = SPI1->DR;                                                                                  
                                                                                                                   
	                                                                                                               
	        // read medium significant byte ***********                                                            
	                                                                                                               
	        // wait for spi transmitter readiness                                                                  
            //while ((SPI1->SR & SPI_SR_TXE) == RESET );                                                           
            SPI1->DR = 0x55;                                                                                       
	        // wait while a transmission complete                                                                  
            while ((SPI1->SR & SPI_SR_RXNE) == RESET );                                                            
	        med_byte = SPI1->DR;                                                                                   
                                                                                                                   
	                                                                                                               
	        // read least significant byte ***********                                                             
	                                                                                                               
	        // wait for spi transmitter readiness                                                                  
            //while ((SPI1->SR & SPI_SR_TXE) == RESET );                                                           
            SPI1->DR = 0x55;                                                                                       
	        // wait while a transmission complete                                                                  
            while ((SPI1->SR & SPI_SR_RXNE) == RESET );                                                            
	        least_byte = SPI1->DR;                                                                                 
	                                                                                                               
	        // chipselect high                                                                                     
            GPIOA->BSRR = GPIO_PIN_8 ;                                         	                                
                                                                                                                   
                                                                                                                   
	                                                                                                               
	        // save most significant byte ***********                                                              
	        sample = ((uint32_t)(most_byte & 0x07))<<16;                                                           
	        // save medium significant byte ***********                                                            
	        sample += ((uint32_t)med_byte)<<8;                                                                     
	        // save least significant byte ***********                                                             
            sample += least_byte;                                                                                  
                                                                                                                   
	                                                                                                               
            pressure = (uint16_t)(sample>>2);                                                                      
		    if(ring_buffer_write_index >= RING_BUFFER_LENGTH)
		    	ring_buffer_write_index = 0;
		    ring_buffer_array[ring_buffer_write_index] = pressure;                    	
		    ring_buffer_write_index++;                                         	                            	
		    
		    //sprintf(message, "%d\r\n", pressure);
		    //HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
		}
		else
		{

			uint32_t start_marker;
			if(ring_buffer_write_index < RING_BUFFER_PRE_ACTION_LENGTH)
				 start_marker = RING_BUFFER_LENGTH  - RING_BUFFER_PRE_ACTION_LENGTH + ring_buffer_write_index;
			else
				 start_marker = ring_buffer_write_index - RING_BUFFER_PRE_ACTION_LENGTH;


			uint32_t i;
	
			for(i=0; i<(RING_BUFFER_LENGTH - RING_BUFFER_PRE_ACTION_LENGTH); i++)
			//for(i=0; i<0xffffffff; i++)
			{
	   			// chipselect low                                     
    	        GPIOA->BRR = GPIO_PIN_8 ;
	                                                                      
                // read most significant byte ***********
	                                                                      
	            // wait for spi transmitter readiness
                //while ((SPI1->SR & SPI_SR_TXE) == RESET );
                SPI1->DR = 0x55;
	            // wait while a transmission complete
                while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	            most_byte = SPI1->DR;
                
	                                                                      
	            // read medium significant byte ***********
	            
	            // wait for spi transmitter readiness
                //while ((SPI1->SR & SPI_SR_TXE) == RESET );
                SPI1->DR = 0x55;
	            // wait while a transmission complete
                while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	            med_byte = SPI1->DR;
                
	                                                                      
	            // read least significant byte ***********
	            
	            // wait for spi transmitter readiness
                //while ((SPI1->SR & SPI_SR_TXE) == RESET );
                SPI1->DR = 0x55;
	            // wait while a transmission complete
                while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	            least_byte = SPI1->DR;
	            
	            // chipselect high 
    	        GPIOA->BSRR = GPIO_PIN_8 ;
                                                                          
                                                                          
	                                                                      
	            // save most significant byte ***********
	            sample = ((uint32_t)(most_byte & 0x07))<<16;
	            // save medium significant byte ***********
	            sample += ((uint32_t)med_byte)<<8;
	            // save least significant byte ***********
                sample += least_byte;
                                                                          
	                                                                      
                pressure = (uint16_t)(sample>>2);                    
				
		    	if(ring_buffer_write_index >= RING_BUFFER_LENGTH)
		    		ring_buffer_write_index = 0;
		    	ring_buffer_array[ring_buffer_write_index] = pressure;                    	
		    	ring_buffer_write_index++;                                         	                            	
			}
		        	
					
	  	    // blink****************************************
			HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    //HAL_Delay(500);
			primitive_delay();
	  	    HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    //HAL_Delay(500);
			primitive_delay();
			HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    //HAL_Delay(500);
			primitive_delay();
	  	    HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	    //HAL_Delay(500);
			primitive_delay();
			// **********************************************
			// output saved data
		    ring_buffer_dump(start_marker);


		    // stop
		    while(1)
		    {
	  	        HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	        //HAL_Delay(500);
				primitive_delay();
	  	        HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
	  	        //HAL_Delay(500);
				primitive_delay();
		    }
		}
	}                                                                      	
	//*/                                                                   	



}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
















/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SysTick->CTRL = 0;    //Disable Systick
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
