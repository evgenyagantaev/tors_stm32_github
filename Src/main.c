/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ring_buffer_interface.h"
#include "ring_buffer_object.h"
#include "pressure_sensor_interface.h"
#include "threshold_detector_interface.h"
#include "threshold_detector_object.h"

/* Private variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

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

	//int i,j,k;

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

	HAL_Delay(1000);
	threshold_detector_initialization();

  	while (1)
  	{
		/*
	  	HAL_Delay(1500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led off\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led on\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led off\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led on\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(1500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led off\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led on\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led off\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led on\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led off\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin); //
		sprintf(message, "led on\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
		*/


		//uint16_t pressure = pressure_sensor_get_sample();
	
   		//*                                                                
	    uint8_t least_byte, med_byte, most_byte;
        uint32_t sample;
	                                                                      
	    
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
                                                                          
	                                                                      
        uint16_t pressure = (uint16_t)(sample>>2);
        //*/

		ring_buffer_array[ring_buffer_write_index % RING_BUFFER_LENGTH] = pressure;
		ring_buffer_write_index++;

		//*
		//if(threshold_detector_action())
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



		//*/

		/*
		sprintf(message, "%d\r\n", pressure);
		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen((const char *)message), 500);
		//*/		



  	}

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
  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
