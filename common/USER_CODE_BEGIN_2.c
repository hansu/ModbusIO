// To be included after line
// /* USER CODE BEGIN 2 */
  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  // receive buffer not empty interrupt enable (USART_CR1_RXNEIE)

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RTO);  // receive timeout interrupt enable (USART_CR1_RTOIE)

  HAL_UART_EnableReceiverTimeout(&huart1);    // receive timeout enable (USART_CR2_RTOEN)

  HAL_UART_ReceiverTimeout_Config(&huart1, 22);

  HAL_TIM_Encoder_Start(&htim1, 1);     //TIM1->CR1 |= TIM_CR1_CEN;