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
uint16_t anButtonState[16] = {0};
uint8_t anButtonOut[16] = {0};
uint16_t nButtonPulse_ms = 100;

/*
 * Interface function for modbus
 */
void UartTransmit(uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(&huart1, (unsigned char *)data, (uint16_t)len, 100);
}

int CoilInRange(uint16_t coil, uint16_t start, uint16_t count)
{
    if (coil >= start && coil < (start+count)) return 1;
    else return 0;
}

/*
 * Interface function for modbus
 */
uint8_t GetCoil(uint16_t nCoilAddress)
{
    if (CoilInRange(nCoilAddress, MDB_ADDR_OUTPUT_COIL, MDB_NUM_OUTPUT_COIL)){
        return HAL_GPIO_ReadPin(GPIOC, 1 << (nCoilAddress-MDB_ADDR_OUTPUT_COIL+FIRST_OUTPUT_GPIOC));
    } else if (CoilInRange(nCoilAddress, MDB_ADDR_INPUT_COIL, MDB_NUM_INPUT_COIL) ) {
        return HAL_GPIO_ReadPin(GPIOB, 1 << (nCoilAddress-MDB_ADDR_INPUT_COIL));
    } else if (CoilInRange(nCoilAddress, MDB_ADDR_INPUT_COIL_INV, MDB_NUM_INPUT_COIL) ) {
        return !HAL_GPIO_ReadPin(GPIOB, 1 << (nCoilAddress-MDB_ADDR_INPUT_COIL_INV));
    } else if (CoilInRange(nCoilAddress, MDB_ADDR_INPUT_COIL_BUTTONS, MDB_NUM_INPUT_COIL) ) {
        return anButtonOut[nCoilAddress-MDB_ADDR_INPUT_COIL_BUTTONS];
    } else {
    	return 0;
    }
}

/*
 * Interface function for modbus
 */
int8_t SetCoil(uint16_t nCoilAddress, uint8_t value)
{
    if (CoilInRange(nCoilAddress, MDB_ADDR_OUTPUT_COIL, 10)) {
        HAL_GPIO_WritePin(GPIOC, 1 << (nCoilAddress-MDB_ADDR_OUTPUT_COIL+FIRST_OUTPUT_GPIOC), value);
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
        case 1:
        case 2:
        case 3:
            temp = 0;
            for(int i=0; i<NUM_AVG_VALUES; i++){
                temp += anADC1AVG_temp[address][i];
            }
            anADC1AVG[address] = temp/NUM_AVG_VALUES;
            *highByte = (uint8_t)(anADC1AVG[address]>>8);
            *lowByte = (uint8_t)(anADC1AVG[address]&0xFF);
            break;
    	case 4:
            tim2CNT_snapshot = TIM2->CNT;
            *highByte = (uint8_t)(tim2CNT_snapshot>>24);
            *lowByte = (uint8_t)(tim2CNT_snapshot>>16);
            break;
    	case 5:
            *highByte = (uint8_t)(tim2CNT_snapshot>>8);
            *lowByte = (uint8_t)tim2CNT_snapshot;
            break;
        case 10:
            *highByte = (uint8_t)(NUM_AVG_VALUES>>8);
            *lowByte = (uint8_t)NUM_AVG_VALUES;
            break;
        case 11:
            *highByte = (uint8_t)(nButtonPulse_ms>>8);
            *lowByte = (uint8_t)nButtonPulse_ms;
            break;
        default:
            *highByte = 0;
            *lowByte = 0;
            break;
    }
}
        
int8_t SetHolding(uint16_t nAddress, uint16_t data){
    switch(nAddress) {
        case 10:
            NUM_AVG_VALUES = data;
            return 0;
        case 11:
            nButtonPulse_ms = data;
            return 0;
        default:
            return -1;
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
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == 0) {
		uint8_t bit = __builtin_ctz(GPIO_Pin);
		anButtonState[bit] = nButtonPulse_ms + DEBOUNCE_TIME_MS;
	}
}
