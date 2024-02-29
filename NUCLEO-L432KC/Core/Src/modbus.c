
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

uint8_t nDeviceID_gl = 240;
uint16_t anModbus_HoldingRegister[MDB_NUM_HOLDINGREG];
uint8_t bModbus_Coils[MDB_NUM_COILS/8];
extern uint8_t anUARTRxBuf[];
extern uint8_t anUARTTxBuf[];
uint8_t GetCoil(uint16_t nCoilAddress);


/*
 * Calculate CRC16 of a data buffer.
 *
 * @param buffer  Data buffer
 * @param count   Number of bytes to use for CRC calculation
 * @return        CRC16 value
 */
uint16_t CRC16(uint8_t *buffer, uint8_t count)
{
  uint16_t bCRC = 0xFFFF;
  uint8_t ni, nj;

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
  bCRC|=(uint16_t)ni<<8;
*/
  return (bCRC);
}

// uint8_t GetCoil(uint16_t nCoilAddress){
//   if(nCoilAddress <= 8){
//     if(PORTB & (1<<nCoilAddress)) //(0x80>>nCoilAddress))
//       return 1;
//     else
//       return 0;
//   } else
//     return 0;
// }

uint8_t Modbus_Parse(uint8_t *pRxPacket, uint8_t *pTxPacket, void (*Send)(uint8_t *, uint8_t))
{
  uint16_t nAddr, nLen, nCRC16, nCRC16_Rx;
  uint16_t ni, nDataBytes;
  if(pRxPacket[0] != nDeviceID_gl)
    return MODBUS_ERR_NO_EXCEPTION;

  switch(pRxPacket[1])
  {
    case MODBUS_READ_HOLDING:
      nAddr = ((uint16_t)pRxPacket[2])<<8;
      nAddr += (uint16_t)pRxPacket[3];
      // Number of registers to read
      nLen = ((uint16_t)pRxPacket[4])<<8;
      nLen += (uint16_t)pRxPacket[5];

      if((nAddr+nLen) > MDB_NUM_HOLDINGREG){
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }
      nCRC16_Rx  = ((uint16_t)pRxPacket[7])<<8;
      nCRC16_Rx += (uint16_t)pRxPacket[6];
      nCRC16 = CRC16(pRxPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return MODBUS_ERR_NEGATIVE_ACKNOWLEDGE;

      // Response
      pTxPacket[0] = nDeviceID_gl;
      pTxPacket[1] = MODBUS_READ_HOLDING;
      // Length in Bytes
      pTxPacket[2] = (uint8_t)(nLen<<1);
      nDataBytes=0;
      for(ni=nAddr; ni<(nAddr+nLen); ni++){
        pTxPacket[3+nDataBytes] = (uint8_t)(anModbus_HoldingRegister[ni]>>8);
        pTxPacket[4+nDataBytes] = (uint8_t)(anModbus_HoldingRegister[ni]&0xFF);
        nDataBytes+=2;
      }
      nCRC16 = CRC16(pTxPacket, 3+nDataBytes);
      pTxPacket[3+nDataBytes] = (uint8_t)(nCRC16&0xFF);
      pTxPacket[4+nDataBytes] = (uint8_t)(nCRC16>>8);
      Send(pTxPacket, 5+nDataBytes);
      return MODBUS_ERR_NO_EXCEPTION;

    case MODBUS_READ_COIL:
      nAddr = ((uint16_t)pRxPacket[2])<<8;
      nAddr += (uint16_t)pRxPacket[3];
      // Number of coils to read
      nLen = ((uint16_t)pRxPacket[4])<<8;
      nLen += (uint16_t)pRxPacket[5];

      if((nAddr+nLen) > MDB_NUM_COILS){
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }
      nCRC16_Rx  = ((uint16_t)pRxPacket[7])<<8;
      nCRC16_Rx += (uint16_t)pRxPacket[6];
      nCRC16 = CRC16(pRxPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return MODBUS_ERR_NEGATIVE_ACKNOWLEDGE;

      // Response
      pTxPacket[0] = nDeviceID_gl;
      pTxPacket[1] = MODBUS_READ_COIL;
      // Length in Bytes
      if(nLen<=8){
        nDataBytes=1;                // One byte data for 8 coils
        pTxPacket[2] = nDataBytes;
        pTxPacket[3] = 0;
        for(ni=0; ni<nLen; ni++){
          pTxPacket[3] |= GetCoil(nAddr+ni) << ni;
        }
      }
      nCRC16 = CRC16(pTxPacket, 3+nDataBytes);
      pTxPacket[3+nDataBytes] = (uint8_t)(nCRC16&0xFF);
      pTxPacket[4+nDataBytes] = (uint8_t)(nCRC16>>8);
      Send(pTxPacket, 5+nDataBytes);
      return MODBUS_ERR_NO_EXCEPTION;

    case MODBUS_READ_STATUS_INPUTS:
      nAddr = ((uint16_t)pRxPacket[2])<<8;
      nAddr += (uint16_t)pRxPacket[3];
      // Number of bits to read
      nLen = ((uint16_t)pRxPacket[4])<<8;
      nLen += (uint16_t)pRxPacket[5];

      if((nAddr+nLen) > MDB_NUM_COILS){
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }
      nCRC16_Rx  = ((uint16_t)pRxPacket[7])<<8;
      nCRC16_Rx += (uint16_t)pRxPacket[6];
      nCRC16 = CRC16(pRxPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return MODBUS_ERR_NEGATIVE_ACKNOWLEDGE;

      // Response
      pTxPacket[0] = nDeviceID_gl;
      pTxPacket[1] = MODBUS_READ_STATUS_INPUTS;
      // Length in Bytes
      if(nLen<=8){
        nDataBytes=1;                // One byte data for 8 coils
        pTxPacket[2] = nDataBytes;
        pTxPacket[3] = 0;
        for(ni=0; ni<nLen; ni++){
          pTxPacket[3] |= GetCoil(nAddr+ni) << ni;
        }
      }
      nCRC16 = CRC16(pTxPacket, 3+nDataBytes);
      pTxPacket[3+nDataBytes] = (uint8_t)(nCRC16&0xFF);
      pTxPacket[4+nDataBytes] = (uint8_t)(nCRC16>>8);
      Send(pTxPacket, 5+nDataBytes);
      return MODBUS_ERR_NO_EXCEPTION;

    default:
      return MODBUS_ERR_ILLEGAL_FUNCTION;
  }

}
