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

#include <stdint.h>

#define UART_BUFFERSIZE 200
extern  uint8_t nDeviceID_gl;
extern uint8_t anTxBuf[];
void Send(uint8_t *pData, uint8_t nLen);
uint8_t GetCoil(uint16_t nCoilAddress);
int8_t SetCoil(uint16_t nCoilAddress, uint8_t value);
void SetMultipleCoils(uint16_t bitMask, uint16_t data);
void GetHolding(uint8_t *highByte, uint8_t *lowByte, uint16_t address);
int8_t SetHolding(uint16_t nAddress, uint16_t data);

uint16_t CRC16(uint8_t *buffer, uint8_t count);
uint8_t Modbus_Parse(uint8_t *pRxPacket, uint8_t *pTxPacket, void (*Send)(uint8_t *, uint8_t));


#define MDB_DEFAULT_DEVICE_ID            1
#define MDB_NUM_HOLDINGREG               6
#define MDB_NUM_OUTPUT_COIL              10
#define MDB_NUM_INPUT_COIL               16
// The addresses below define the beginng address of that type
#define MDB_ADDR_HOLDINGREG              0
#define MDB_ADDR_INPUT_COIL              0
#define MDB_ADDR_INPUT_COIL_INV          (MDB_ADDR_INPUT_COIL + MDB_NUM_INPUT_COIL)
#define MDB_ADDR_INPUT_COIL_BUTTONS      (MDB_ADDR_INPUT_COIL_INV + MDB_NUM_INPUT_COIL)
#define MDB_ADDR_OUTPUT_COIL             100

// Modbus function codes
#define MODBUS_READ_COIL                 0x01
#define MODBUS_READ_STATUS_INPUTS        0x02
#define MODBUS_READ_HOLDING              0x03
#define MODBUS_READ_INPUT_REGISTERS      0x04
#define MODBUS_WRITE_SINGLE_COIL         0x05
#define MODBUS_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_WRITE_MULTIPLE_COILS      0x0F
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
