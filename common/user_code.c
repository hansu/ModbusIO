#include "main.h"
#include "stdio.h"
#include <string.h>
#include "modbus.h"
#include "user.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

#define UART_BUFFERSIZE 200
uint16_t nUARTIter;
uint8_t anUARTRxBuf[UART_BUFFERSIZE];
uint8_t anUARTTxBuf[UART_BUFFERSIZE];
uint16_t anModbus_HoldingRegister[MDB_NUM_HOLDINGREG];
uint16_t anADC1Values[4];
#define FIRST_OUTPUT_GPIOC 4 // first 4 pins are used as analoh input
uint16_t anADC1AVG[4];
uint16_t anADC1AVG_temp[4][MAX_NUM_AVG_VALUES];
uint16_t NUM_AVG_VALUES = 100;

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
    if (nCoilAddress >= MDB_ADDR_OUTPUT_COIL &&
        nCoilAddress < (MDB_ADDR_OUTPUT_COIL+MDB_NUM_OUTPUT_COIL) ) {
        return HAL_GPIO_ReadPin(GPIOC, 1 << (nCoilAddress-MDB_ADDR_OUTPUT_COIL+FIRST_OUTPUT_GPIOC));

    } else if (nCoilAddress >= MDB_ADDR_INPUT_COIL &&
        nCoilAddress < (MDB_ADDR_INPUT_COIL+MDB_NUM_INPUT_COIL) ) {
        return HAL_GPIO_ReadPin(GPIOB, 1 << (nCoilAddress-MDB_ADDR_INPUT_COIL));
    } else if (nCoilAddress >= MDB_ADDR_INPUT_COIL_INVERTED &&
        nCoilAddress < (MDB_ADDR_INPUT_COIL_INVERTED+MDB_NUM_INPUT_COIL) ) {
        return !HAL_GPIO_ReadPin(GPIOB, 1 << (nCoilAddress-MDB_ADDR_INPUT_COIL_INVERTED));
    } else {
    	return 0;
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

void GetHolding(uint8_t *highByte, uint8_t *lowByte, uint16_t address){

    uint32_t temp;
    static uint32_t tim2CNT_snapshot;
    switch(address){
    	case 0:
            tim2CNT_snapshot = TIM2->CNT;
            *highByte = (uint8_t)(tim2CNT_snapshot>>24);
            *lowByte = (uint8_t)(tim2CNT_snapshot>>16);

    	case 1:
            *highByte = (uint8_t)(tim2CNT_snapshot>>8);
            *lowByte = (uint8_t)tim2CNT_snapshot;
        case 2:
        case 3:
        case 4:
        case 5:
            temp = 0;
            for(int i=0; i<NUM_AVG_VALUES; i++){
                temp += anADC1AVG_temp[address-2][i];
            }
            anADC1AVG[address-2] = temp/NUM_AVG_VALUES;
            *highByte = (uint8_t)(anADC1AVG[address-2]>>8);
            *lowByte = (uint8_t)(anADC1AVG[address-2]&0xFF);
            break;
        default:
            *highByte = 0;
            *lowByte = 0;
            break;
    }
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
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)anADC1Values, 4);
}

void MainLoop(void)
{
    while (1)
    {
//  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#if defined(STM32L476xx) || defined(STM32L432xx) || defined(STM32F303xE)

#endif
        HAL_Delay(1);
    }
}

/*
 * Attention:
 * @param: GPIO_Pin is a bit mask,
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin & (uint16_t)(1 << 0)){
//		 = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
	}
    // TODO: debouncing for buttons


}
