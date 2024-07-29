// To be included after line
// /* USER CODE BEGIN WHILE */
  
while (1)
{
//  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#if defined(STM32L476xx)
//  anModbus_HoldingRegister[0] = (uint8_t)TIM2->CNT;
//  anModbus_HoldingRegister[1] = (uint8_t)(((TIM2->CNT) >> 8) & 0xFF);
  anModbus_HoldingRegister[4] = (uint8_t)TIM2->CNT;
#elif defined(STM32L432xx)
  anModbus_HoldingRegister[0] = (uint8_t)TIM1->CNT;
  anModbus_HoldingRegister[1] = (uint8_t)(((TIM1->CNT) >> 8) & 0xFF);
#elif defined(STM32F303xE)
//  anModbus_HoldingRegister[0] = (uint8_t)TIM1->CNT;
//  anModbus_HoldingRegister[1] = (uint8_t)(((TIM1->CNT) >> 8) & 0xFF);
#endif
  HAL_Delay(100);
}
