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
  
#ifndef MODBUS_H_
#define MODBUS_H_

#include <GenericTypeDefs.h>

#define UART_BUFFERSIZE 200
extern  UINT8 nDeviceID_gl;
extern UINT8 anTxBuf[];
extern UINT16 anModbus_HoldingRegister[];


UINT16 CRC16(UINT8 *buffer, UINT8 count);
UINT8 Modbus_Parse(UINT8 *pModbusPacket);


#define MODBUS_READ_COIL                 0x01
#define MODBUS_READ_STATUS_INPUTS        0x02
#define MODBUS_READ_HOLDING              0x03
#define MODBUS_READ_INPUT_REGISTERS      0x04
#define MODBUS_SET_COIL                  0x05
#define MODBUS_SET_SINGLE_REGISTER       0x06
#define MODBUS_SET_MULTIPLE_COILS        0x0F
#define MODBUS_WRITE_MULTIPLE_HOLDING    0x10


// Modbus exception codes
#define MODBUS_ERR_NO_EXCEPTION           0
#define MODBUS_ERR_ILLEGAL_FUNCTION       1
#define MODBUS_ERR_ILLEGAL_DATA_ADDRESS   2
#define MODBUS_ERR_ILLEGAL_DATA_VALUE     3
#define MODBUS_ERR_SLAVE_DEVICE_FAILURE   4
#define MODBUS_ERR_ACKNOWLEDGE            5
#define MODBUS_ERR_SLAVE_DEVICE_BUSY      6
#define MODBUS_ERR_NEGATIVE_ACKNOWLEDGE   7
#define MODBUS_ERR_MEMORY_PARITY_ERROR    8


#endif // MODBUS_H_
