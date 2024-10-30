#include "main.h"
#include "stdio.h"
#include <string.h>
#include "../../../common/modbus.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern uint16_t anModbus_HoldingRegister[];

#define UART_BUFFERSIZE 200
uint16_t nUARTIter;
uint8_t anUARTRxBuf[UART_BUFFERSIZE];
uint8_t anUARTTxBuf[UART_BUFFERSIZE];
extern uint16_t anModbus_HoldingRegister[MDB_NUM_HOLDINGREG];
uint16_t ADC1Values[4];
#define FIRST_OUTPUT_GPIOC 4 // first 4 pins are used as analoh input

/*
 * Interface function for modbus
 */
void UartTransmit(uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(&huart1, (unsigned char *)data, (uint16_t)len, 100);
}

/*
 * Interface function for modbus
 */
uint8_t GetCoil(uint16_t nCoilAddress)
{
    if (nCoilAddress >= MDB_ADDR_FIRST_INPUT_COIL &&
        nCoilAddress < (MDB_ADDR_FIRST_INPUT_COIL+MDB_NUM_INPUT_COIL) ) {
        return HAL_GPIO_ReadPin(GPIOB, 1 << nCoilAddress);
    } else if (nCoilAddress >= MDB_ADDR_FIRST_OUTPUT_COIL &&
        nCoilAddress < (MDB_ADDR_FIRST_OUTPUT_COIL+MDB_NUM_OUTPUT_COIL) ) {
        return HAL_GPIO_ReadPin(GPIOC, 1 << (nCoilAddress+FIRST_OUTPUT_GPIOC));
    }
}

/*
 * Interface function for modbus
 */
int8_t SetCoil(uint16_t nCoilAddress, uint8_t value)
{
    if (nCoilAddress < 10) { // TODO insert variable limit
        HAL_GPIO_WritePin(GPIOC, 1 << (nCoilAddress+FIRST_OUTPUT_GPIOC), value);
        return 0;
    } else {
        return -1;
    }

}

/*
 * Interface function for modbus
 */
void SetMultipleCoils(uint16_t bitMask, uint16_t data)
{
    uint32_t temp = GPIOC->IDR;
    temp &= ~(bitMask<<FIRST_OUTPUT_GPIOC);
    GPIOC->ODR = temp | (data<<FIRST_OUTPUT_GPIOC);

}

uint8_t ReadUSART()
{
    return USART1->RDR;
}

void UART1_RX_IRQ(UART_HandleTypeDef *huart)
{
    if (USART1->ISR & USART_ISR_RXNE) {
        if (nUARTIter < UART_BUFFERSIZE) {
            anUARTRxBuf[nUARTIter++] = ReadUSART();
        } else
        {
            USART1->RQR |= USART_RQR_RXFRQ; // clear RXNE flag
        }
    }
    // Overrun error
    if (USART1->ISR & USART_ISR_ORE) {
        USART1->ICR |= USART_ICR_ORECF; // Reset overrun error
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        nUARTIter = 0;
    }
    if (USART1->ISR & USART_ISR_EOBF) {
        USART1->ICR |= USART_ICR_EOBCF; // clear end of block flag
    }
    if (USART1->ISR & USART_ISR_RTOF) {
        USART1->ICR |= USART_ICR_RTOCF; // clear receive timeout flag
        nUARTIter = 0;                  // Get ready for new data
        Modbus_Parse((uint8_t*)anUARTRxBuf, anUARTTxBuf, UartTransmit);
    }
}

void StartPeripherals(void)
{
	// Interrupt for single character reception
	//    UART_Start_Receive_IT(&huart1, anUARTRxBuf, ?);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // receive buffer not empty interrupt enable (USART_CR1_RXNEIE)

    // Interrupt for end of block
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RTO); // receive timeout interrupt enable (USART_CR1_RTOIE)

	HAL_UART_ReceiverTimeout_Config(&huart1, 22);
	HAL_UART_EnableReceiverTimeout(&huart1); // receive timeout enable (USART_CR2_RTOEN)

    HAL_TIM_Encoder_Start(&htim2, 1); // TIM1->CR1 |= TIM_CR1_CEN;
    //  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    // anModbus_HoldingRegister[0..1] are reserved for encoder
    HAL_ADC_Start_DMA(&hadc1, (uint16_t *)&anModbus_HoldingRegister[2], 4);
}

void MainLoop(void)
{
    while (1)
    {
//  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#if defined(STM32L476xx) || defined(STM32L432xx) || defined(STM32F303xE)
        anModbus_HoldingRegister[0] = (uint8_t)TIM2->CNT;
        anModbus_HoldingRegister[1] = (uint8_t)(((TIM2->CNT) >> 8) & 0xFF);
#endif
        HAL_Delay(10);
    }
}

/*
 * Attention:
 * @param: GPIO_Pin is a bit mask,
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin & (uint16_t)(1 << 0)){
		anModbus_HoldingRegister[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
	}
    // TODO: debouncing for buttons


}
