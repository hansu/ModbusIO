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
#include <p18f2620.h>
#include <usart.h>
#include <timers.h>
#include <adc.h>
#include <stdio.h>
#include <GenericTypeDefs.h>
#include "modbus.h"

extern unsigned int anADCValues_RAW[];

UINT8 nUARTIter = 0xFF;
#pragma udata my_memory_section_1
UINT8 anUARTBuf[UART_BUFFERSIZE];
#pragma udata my_memory_section_2
UINT8 anTxBuf[UART_BUFFERSIZE];


void StartTimer1(void)
{
	T1CONbits.TMR1ON = 1;
}
void StopTimer1(void)
{
	T1CONbits.TMR1ON = 0;
}

void UART_Send(UINT8 *pData, UINT8 nLen)
{
	UINT8 ni;
	for(ni=0; ni<nLen; ni++)
	{
		putcUSART(*(pData++));
		while(BusyUSART());
	}
}

#pragma code isr=0x08
#pragma interrupt isr
void isr(void)
{
  	// Timer 0
	if(INTCONbits.TMR0IF)
	{
	//	char anTxBuf[20];

	//	ConvertADC();	        // Start conversion
	//	while(BusyADC());
	//	U_adc = ReadADC();

//		anADCValues_RAW[0] = ReadADC();
//		sprintf(anTxBuf, "ADC0=%04d %X\r\n", anADCValues_RAW[0], PORTB & 0x01);
//	    putsUSART(anTxBuf);

		INTCONbits.TMR0IF = 0; //clear Interrupt flag
	}

  	//Timer 1 - Timeout -> Anfrage beantworten
	if(PIR1bits.TMR1IF)
	{
		UINT16 nCRC16;
		//CloseTimer1();
		StopTimer1();
		nUARTIter = 0; // Bereit machen fuer neue Daten
		
		anTxBuf[0] = 240;  	// Adresse
		anTxBuf[1] = MODBUS_READ_HOLDING;
		anTxBuf[2] = 2;  	// Laenge in Bytes
		anTxBuf[3] = 0x15;  // Wert
		anTxBuf[4] = 0xCA;	// Wert
		nCRC16 = CRC16(&anTxBuf[0], 5);
		anTxBuf[5] = (UINT8)(nCRC16&0xFF);
		anTxBuf[6] = (UINT8)(nCRC16>>8);
    
		UART_Send(anTxBuf, 7);

		PIR1bits.TMR1IF = 0; // clear Interrupt flag
	}

	// RS232 reception
	if(PIR1bits.RCIF)
	{
		if(nUARTIter < UART_BUFFERSIZE)
			anUARTBuf[nUARTIter++] = ReadUSART();
		else
			ReadUSART();

		// Restart time-out counter at every reception
		
		WriteTimer1(0xFFFF-1700);
		StartTimer1();

		if(RCSTAbits.OERR)
		{
			RCSTAbits.CREN = 0;
			Nop();
			RCSTAbits.CREN = 1;
		}

		if(RCSTAbits.FERR)
		{
			ReadUSART();
		}

		PIR1bits.RCIF=0;
	}
//		PIR1 = 0;
}
