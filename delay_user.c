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

#include "delay_user.h"

#ifdef __18CXX
#include <usart.h>
#include <delays.h>
#elif __XC8

#else 
#warning "compiler not supported"
#endif
#include "GenericTypeDefs.h"


void DelayFor18TCY( void )
{
	Nop();	Nop();	Nop();	Nop();	Nop();	Nop();	Nop();	Nop();	Nop();	Nop();	Nop();	Nop();
}


// milliseconds for 8 MHz
void Delayms(unsigned char ms)
{
	char i;
	for(i=0;i<ms;i++)
	{
		Delay1KTCYx(2);
	}
}

// 1/10 s for 8 MHz
void Delayx100ms(unsigned char xs)
{
	unsigned char i;
	for(i=0;i<xs;i++)
	{
		Delay10KTCYx(20);
	}
}
