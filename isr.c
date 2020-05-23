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

#include "isr.h"
#include <usart.h>
//#include <timers.h>
#include <adc.h>
#include <stdio.h>

extern unsigned int anADCValues_RAW[];

#pragma code isr=0x08
#pragma interrupt isr
void isr(void)
{
  	// Timer 0
	if(INTCONbits.TMR0IF)
	{

	//	ConvertADC();	        // Start conversion
	//	while(BusyADC());
	//	U_adc = ReadADC();

		INTCONbits.TMR0IF = 0;  // clear Interrupt flag
	}

	// RS232 reception
	if(PIR1bits.RCIF)
	{
		char anTxBuf[20];
		ReadUSART();

		anADCValues_RAW[0] = ReadADC();
		sprintf(anTxBuf, "ADC0=%04d %X\r\n", anADCValues_RAW[0], PORTB & 0x01);
	    putsUSART(anTxBuf);
	
		PIR1bits.RCIF=0;
	}
//		PIR1 = 0;
}
