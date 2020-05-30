/*
* This file is part of the ModbusIO project.
*
* Copyright (C) 2020 Hans Unzner <hansunzner@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <p18f2620.h>
#include <delays.h>
#include <usart.h>
#include <pwm.h>
#include <timers.h>
#include <adc.h>
#include "delay_user.h"
#include <stdio.h>
#include "modbus.h"

 /** Configuration ********************************************************/
#pragma		config	OSC = INTIO67
#pragma		config	PWRT = ON    	// power up timer on
#pragma		config	BOREN = OFF   	// brown out detect
#pragma		config	WDT = OFF    	// watchdog off
#pragma		config	LVP = OFF    	// lvp off
#pragma		config	FCMEN = OFF
#pragma		config	IESO = OFF
#pragma		config 	MCLRE = OFF
#pragma		config	PBADEN = OFF
#pragma		config  DEBUG = OFF

#define N_ADC_CHANNELS	4
#define BAUD19200

unsigned int anADCValues_RAW[N_ADC_CHANNELS];


#pragma code
void main(void)
 {
	const char anWelcomeString[] ={"ModbusIO starting...\r\n"};
	OSCTUNEbits.PLLEN=1;	// PLL enable
	OSCCONbits.IRCF0=1;		// 8 MHz clock
	OSCCONbits.IRCF1=1;
	OSCCONbits.IRCF2=1;

	TRISA = 0xFF;			// Set to input
	TRISC = 0b10000000;	    // Set all to output except UART RX
	TRISB = 0xFF;			// Set to input

//	ADCON2 = 0b10110101;	// right, 16 TAD, FOSC/16
//	ADCON1 = 0b00001011;	// PortA AD I/O
//	ADCON0 = 0b00001101;

	OpenPWM1(200);
	OpenTimer2(T2_PS_1_4 & TIMER_INT_OFF);
	SetDCPWM1(30);
	RCONbits.IPEN = 0;			// Disable priority levels on interrupts
	Nop();
	INTCON = 0;
	INTCONbits.GIE = 1;			// global interrupts enable
	INTCONbits.PEIE = 1;		// global interrupts enable
//	PIE2 = 0;
//	PIE1bits.RCIE = 1;			// Enables the EUSART receive interrupt

	// T0_PS_1_32 = 1.05 s

	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32);
	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_2 & T1_OSC1EN_OFF);	

	Delay1KTCYx(100);		    // 50ms

#ifdef BAUD19200
	// Configure USART @19200 Baud 8N1 (spbrg = (f_osc/(16*Baudrate))-1)
	OpenUSART( USART_TX_INT_OFF & USART_RX_INT_ON & USART_BRGH_HIGH & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX,	25);
#endif
 	putsUSART ((char*)anWelcomeString);

	while(1)
	{
		OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_16_TAD,
				ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS,
				ADC_4ANA);
		ConvertADC();	        // Start conversion
		while(BusyADC());
		anModbus_HoldingRegister[0] = ReadADC();
		Delayms(10);
		Delayx100ms(10);
	}

}
