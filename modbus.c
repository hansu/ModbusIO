
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
UINT16 anModbus_HoldingRegister[MDB_NUM_HOLDINGREG];
UINT8 bModbus_Coils[MDB_NUM_COILS/8];
extern UINT8 anUARTRxBuf[];
extern UINT8 anUARTTxBuf[];

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
/*
  ni=bCRC;
  bCRC>>=8;
  bCRC|=(UINT16)ni<<8;
*/
  return (bCRC);
}

UINT8 GetCoil(UINT16 nCoilAddress){
  if(nCoilAddress <= 8){
    if(PORTB & (1<<nCoilAddress)) //(0x80>>nCoilAddress))
      return 1;
    else
      return 0;
  } else
    return 0;
}

UINT8 Modbus_Parse(UINT8 *pRxPacket, UINT8 *pTxPacket, void (*Send)(UINT8 *, UINT8))
{
  UINT16 nHoldingReg, nLen, nCRC16, nCRC16_Rx;
  UINT16 nCoilAddr;
  UINT16 ni, nDataBytes;
  if(pRxPacket[0] != nDeviceID_gl)
    return MODBUS_ERR_NO_EXCEPTION;
  switch(pRxPacket[1])
  {
    case MODBUS_READ_HOLDING:
      nHoldingReg = ((UINT16)pRxPacket[2])<<8;
      nHoldingReg += (UINT16)pRxPacket[3];
      // Number of registers to read
      nLen = ((UINT16)pRxPacket[4])<<8;
      nLen += (UINT16)pRxPacket[5];

      if((nHoldingReg+nLen) > MDB_NUM_HOLDINGREG){
        return 1;
      }
      nCRC16_Rx  = ((UINT16)pRxPacket[7])<<8;
      nCRC16_Rx += (UINT16)pRxPacket[6];
      nCRC16 = CRC16(pRxPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return 1;

      // Response
      pTxPacket[0] = nDeviceID_gl;
      pTxPacket[1] = MODBUS_READ_HOLDING;
      // Length in Bytes
      pTxPacket[2] = (UINT8)(nLen<<1);
      nDataBytes=0;
      for(ni=nHoldingReg; ni<(nHoldingReg+nLen); ni++){
        pTxPacket[3+nDataBytes] = (UINT8)(anModbus_HoldingRegister[ni]>>8);
        pTxPacket[4+nDataBytes] = (UINT8)(anModbus_HoldingRegister[ni]&0xFF);
        nDataBytes+=2;
      }
      nCRC16 = CRC16(pTxPacket, 3+nDataBytes);
      pTxPacket[3+nDataBytes] = (UINT8)(nCRC16&0xFF);
      pTxPacket[4+nDataBytes] = (UINT8)(nCRC16>>8);
      Send(pTxPacket, 5+nDataBytes);
      break;

    case MODBUS_READ_COIL:
      nCoilAddr = ((UINT16)pRxPacket[2])<<8;
      nCoilAddr += (UINT16)pRxPacket[3];
      // Nuber of coils to read
      nLen = ((UINT16)pRxPacket[4])<<8;
      nLen += (UINT16)pRxPacket[5];

      if((nCoilAddr+nLen) > MDB_NUM_COILS){
        return 1;
      }
      nCRC16_Rx  = ((UINT16)pRxPacket[7])<<8;
      nCRC16_Rx += (UINT16)pRxPacket[6];
      nCRC16 = CRC16(pRxPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return 1;

      // Response
      pTxPacket[0] = nDeviceID_gl;
      pTxPacket[1] = MODBUS_READ_HOLDING;
      // Length in Bytes
      if(nLen<=8){
        nDataBytes=1;                // One byte data for 8 coils
        pTxPacket[2] = nDataBytes;
        pTxPacket[3] = 0;
        for(ni=0; ni<nLen; ni++){
          pTxPacket[3] |= GetCoil(nCoilAddr+ni) << ni;
        }
      }
      nCRC16 = CRC16(pTxPacket, 3+nDataBytes);
      pTxPacket[3+nDataBytes] = (UINT8)(nCRC16&0xFF);
      pTxPacket[4+nDataBytes] = (UINT8)(nCRC16>>8);
      Send(pTxPacket, 5+nDataBytes);

    default:
      return MODBUS_ERR_ILLEGAL_FUNCTION;
  }

}
