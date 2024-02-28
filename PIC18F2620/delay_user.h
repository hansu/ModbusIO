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

#ifndef __DELAY_USER_H
#define __DELAY_USER_H

extern unsigned int U_adc,I_adc;

void Delayms(unsigned char ms);
void Delayx100ms(unsigned char xs);

#endif
