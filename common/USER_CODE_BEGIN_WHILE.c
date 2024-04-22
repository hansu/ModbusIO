// To be included after line
// /* USER CODE BEGIN WHILE */
  
  while (1)
  {
//  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  anModbus_HoldingRegister[0] = (uint8_t)TIM1->CNT;
  anModbus_HoldingRegister[1] = (uint8_t)(((TIM1->CNT) >> 8) & 0xFF);
    HAL_Delay(100);
  }