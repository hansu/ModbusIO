// To be included after line
// /* USER CODE BEGIN 2 */
  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  // receive buffer not empty interrupt enable (USART_CR1_RXNEIE)

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RTO);  // receive timeout interrupt enable (USART_CR1_RTOIE)

  HAL_UART_EnableReceiverTimeout(&huart1);    // receive timeout enable (USART_CR2_RTOEN)

  HAL_UART_ReceiverTimeout_Config(&huart1, 22);


#if defined(STM32L432xx)
	HAL_TIM_Encoder_Start(&htim2, 1);     //TIM1->CR1 |= TIM_CR1_CEN;

  hdma_adc1.Instance = DMA1_Channel1;
//  hdma_adc1.Init.Request = DMA_REQUEST_0;
//  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
//  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
//  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_adc1);

#elif defined(STM32L476xx)

    HAL_TIM_Encoder_Start(&htim2, 1);     //TIM1->CR1 |= TIM_CR1_CEN;

  hdma_adc1.Instance = DMA1_Channel1;
//  hdma_adc1.Init.Request = DMA_REQUEST_0;
//  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
//  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
//  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_adc1);


//  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint16_t*)&anModbus_HoldingRegister[0], 4);

#elif defined(STM32F303xE)

	HAL_TIM_Encoder_Start(&htim2, 1);     //TIM1->CR1 |= TIM_CR1_CEN;

	hdma_adc1.Instance = DMA1_Channel1;
	//  hdma_adc1.Init.Request = DMA_REQUEST_0;
	//  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	//  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	//  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	//  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	//  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc1.Init.Mode = DMA_CIRCULAR;
	//  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&hdma_adc1);


//  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint16_t*)&anModbus_HoldingRegister[0], 4);

#endif
