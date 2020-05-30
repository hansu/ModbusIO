
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

#include "modbus.h"
#include <usart.h>

UINT8 nDeviceID_gl = 240;
UINT16 anModbus_HoldingRegister[4];
UINT8 bModbus_Coils;
UINT8 nModbus_TxBuffer[20];

void UART_Send(UINT8 *pData, UINT8 nLen);

/*
 * Calculate CRC16 of a data buffer.
 *
 * @param buffer  Data buffer
 * @param count   Number of bytes to use for CRC calculation
 * @return        CRC16 value
 */
UINT16 CRC16(UINT8 *buffer, UINT8 count)
{
  UINT16 bCRC = 0xFFFF;
  UINT8 ni, nj;

  for (ni = 0; ni < count; ni++) {
    bCRC = bCRC ^ *buffer;
    buffer++;

    for (nj = 0; nj != 8; nj++) {
      if (bCRC & 0x01) {
        bCRC = (bCRC >> 1) ^ 0xA001;
      } else {
        bCRC = bCRC >> 1;
      }
    }
  }
  ni=bCRC;
  bCRC>>=8;
  bCRC|=(UINT16)ni<<8;
  return (bCRC);
}

UINT8 Modbus_Parse(UINT8 *pModbusPacket)
{
  UINT16 nHoldingReg, nLen, nCRC16, nCRC16_Rx;
  if(pModbusPacket[0] != nDeviceID_gl)
    return MODBUS_ERR_NO_EXCEPTION;
  switch(pModbusPacket[1])
  {
    case MODBUS_READ_HOLDING:
      anTxBuf[0] = nDeviceID_gl;
      anTxBuf[1] = MODBUS_READ_HOLDING;
      anTxBuf[2] = 2; //Length in Bytes
      nHoldingReg = ((UINT16)pModbusPacket[2])<<8;
      nHoldingReg += (UINT16)pModbusPacket[3];
      nLen = ((UINT16)pModbusPacket[4])<<8;
      nLen += (UINT16)pModbusPacket[5];
      anTxBuf[3] = (UINT8)(anModbus_HoldingRegister[nHoldingReg]>>8);
      anTxBuf[4] = (UINT8)(anModbus_HoldingRegister[nHoldingReg]&0xFF);
      nCRC16_Rx  = ((UINT16)pModbusPacket[6])<<8;
      nCRC16_Rx += (UINT16)pModbusPacket[7];
      nCRC16 = CRC16(pModbusPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return 1;
      nCRC16 = CRC16(&anTxBuf[0], 5);
      anTxBuf[5] = (UINT8)(nCRC16>>8);
      anTxBuf[6] = (UINT8)(nCRC16&0xFF);
      UART_Send(anTxBuf, 7);
      break;
    default:
      return MODBUS_ERR_ILLEGAL_FUNCTION;
  }

}
