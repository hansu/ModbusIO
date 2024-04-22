// To be included after line
// /* USER CODE BEGIN 0 */

#define UART_BUFFERSIZE 200
uint16_t nUARTIter;
uint8_t anUARTRxBuf[UART_BUFFERSIZE];
uint8_t anUARTTxBuf[UART_BUFFERSIZE];
extern uint16_t anModbus_HoldingRegister[MDB_NUM_HOLDINGREG];
extern uint8_t bModbus_Coils[MDB_NUM_COILS/8];
uint16_t ADC1Values[4];

uint8_t ReadUSART ()
{
  return USART1->RDR;
}

void UartTransmit (uint8_t *data, uint8_t len)
{
  HAL_UART_Transmit(&huart1, (unsigned char*)data, (uint16_t)len, 100);
}

void UART1_RX_IRQ(UART_HandleTypeDef *huart)
{

  if (USART1->ISR & USART_ISR_RXNE) {
    if(nUARTIter < UART_BUFFERSIZE) {
      anUARTRxBuf[nUARTIter++] = ReadUSART();
    } else {
//      ReadUSART();
      USART1->RQR |= USART_RQR_RXFRQ; // clear RXNE flag
    }

  }
  // Overrun error
  if (USART1->ISR & USART_ISR_ORE) {
    USART1->ICR |= USART_ICR_ORECF; // Reset overrun error
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  }

  if (USART1->ISR & USART_ISR_EOBF) {
    USART1->ICR |= USART_ICR_EOBCF; // clear end of block flag
  }

  if (USART1->ISR & USART_ISR_RTOF) {
    USART1->ICR |= USART_ICR_RTOCF; // clear receive timeout flag
      nUARTIter = 0;                // Get ready for new data
      Modbus_Parse((uint8_t*)anUARTRxBuf, anUARTTxBuf, UartTransmit);
  }

}

 uint8_t GetCoil(uint16_t nCoilAddress){
   if(nCoilAddress <= 8)
       return 1;
   else
       return 0;
 }
