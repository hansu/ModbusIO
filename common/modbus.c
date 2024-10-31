
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

uint8_t nDeviceID_gl = MDB_DEFAULT_DEVICE_ID;
uint16_t anModbus_HoldingRegister[MDB_NUM_HOLDINGREG];
extern uint8_t anUARTRxBuf[];
extern uint8_t anUARTTxBuf[];


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

uint8_t Modbus_Parse(uint8_t *pRxPacket, uint8_t *pTxPacket, void (*Send)(uint8_t *, uint8_t))
{
  uint16_t nAddr; // Register address
  uint16_t nLen, nCRC16, nCRC16_Rx;
  uint16_t ni, nData;
  uint8_t nByteCount;
  if(pRxPacket[0] != nDeviceID_gl)
    return MODBUS_ERR_NO_EXCEPTION;

  switch(pRxPacket[1])
  {
    case MODBUS_READ_HOLDING:
    case MODBUS_READ_INPUT_REGISTERS:
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
      pTxPacket[1] = pRxPacket[1];
      // Length in Bytes
      pTxPacket[2] = (uint8_t)(nLen<<1);
      nByteCount=0;
      for(ni=nAddr; ni<(nAddr+nLen); ni++){
        pTxPacket[3+nByteCount] = (uint8_t)(anModbus_HoldingRegister[ni]>>8);
        pTxPacket[4+nByteCount] = (uint8_t)(anModbus_HoldingRegister[ni]&0xFF);
        nByteCount+=2;
      }
      nCRC16 = CRC16(pTxPacket, 3+nByteCount);
      pTxPacket[3+nByteCount] = (uint8_t)(nCRC16&0xFF);
      pTxPacket[4+nByteCount] = (uint8_t)(nCRC16>>8);
      Send(pTxPacket, 5+nByteCount);
      return MODBUS_ERR_NO_EXCEPTION;

    case MODBUS_READ_STATUS_INPUTS:
    case MODBUS_READ_COIL:
      nAddr = ((uint16_t)pRxPacket[2])<<8;
      nAddr += (uint16_t)pRxPacket[3];
      // Number of coils to read
      nLen = ((uint16_t)pRxPacket[4])<<8;
      nLen += (uint16_t)pRxPacket[5];

      // Check input coil range
      if(nAddr < MDB_ADDR_FIRST_INPUT_COIL && nAddr < MDB_ADDR_FIRST_OUTPUT_COIL){
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }
      if(nLen > (MDB_NUM_INPUT_COIL+MDB_NUM_OUTPUT_COIL)){
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }
      nCRC16_Rx  = ((uint16_t)pRxPacket[7])<<8;
      nCRC16_Rx += (uint16_t)pRxPacket[6];
      nCRC16 = CRC16(pRxPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return MODBUS_ERR_NEGATIVE_ACKNOWLEDGE;

      // Response
      pTxPacket[0] = nDeviceID_gl;
      pTxPacket[1] = pRxPacket[1];  // Function code

      uint8_t nBitPos=0;
      nByteCount=1;
      pTxPacket[3] = 0;
      for(uint16_t nCurrAddr = nAddr; nCurrAddr < (nAddr + nLen); nCurrAddr++){
        if(nBitPos > 7){
          nBitPos = 0;
          nByteCount++;
          pTxPacket[2+nByteCount] = 0;
        }
        pTxPacket[2+nByteCount] |= GetCoil(nCurrAddr) << nBitPos; // TODO: read whole GPIO Port at once
        nBitPos++;
      }
      pTxPacket[2] = nByteCount;
      nCRC16 = CRC16(pTxPacket, 3+nByteCount);
      pTxPacket[3+nByteCount] = (uint8_t)(nCRC16&0xFF);
      pTxPacket[4+nByteCount] = (uint8_t)(nCRC16>>8);
      Send(pTxPacket, 5+nByteCount);
      return MODBUS_ERR_NO_EXCEPTION;
  
    case MODBUS_WRITE_SINGLE_COIL:
      nAddr = ((uint16_t)pRxPacket[2])<<8;
      nAddr += (uint16_t)pRxPacket[3];

      if(nAddr < MDB_ADDR_FIRST_OUTPUT_COIL){
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }
      // Remove address offset
      nAddr -= MDB_ADDR_FIRST_OUTPUT_COIL;

      // Check CRC
      nCRC16_Rx  = ((uint16_t)pRxPacket[7])<<8;
      nCRC16_Rx += (uint16_t)pRxPacket[6];
      nCRC16 = CRC16(pRxPacket, 6);
      if(nCRC16 != nCRC16_Rx)
        return MODBUS_ERR_NEGATIVE_ACKNOWLEDGE;

      // Write data
      nData = ((uint16_t)pRxPacket[4])<<8;
      nData += (uint16_t)pRxPacket[5];

      if(SetCoil(nAddr, (nData==0xFF00)?1:0)) {
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }

      // Response : echo of the request
      Send(pRxPacket, 8);
      return MODBUS_ERR_NO_EXCEPTION;


    case MODBUS_WRITE_MULTIPLE_COILS:
      nAddr = ((uint16_t)pRxPacket[2])<<8;
      nAddr += (uint16_t)pRxPacket[3];

      if(nAddr < MDB_ADDR_FIRST_OUTPUT_COIL){
        return MODBUS_ERR_ILLEGAL_DATA_VALUE;
      }
      // Remove address offset
      nAddr -= MDB_ADDR_FIRST_OUTPUT_COIL;
      // nLen = Quantity of Outputs
      nLen = ((uint16_t)pRxPacket[4])<<8;
      nLen += (uint16_t)pRxPacket[5];
      // Byte count (Quantity of Outputs / 8)
      uint8_t nBytes = pRxPacket[6];
      // Output values, N bytes
      (void)pRxPacket[7];

      // Check CRC
      nCRC16_Rx  = ((uint16_t)pRxPacket[8+nBytes])<<8;
      nCRC16_Rx += (uint16_t)pRxPacket[7+nBytes];
      nCRC16 = CRC16(pRxPacket, 7+nBytes);
      if(nCRC16 != nCRC16_Rx)
        return MODBUS_ERR_NEGATIVE_ACKNOWLEDGE;
  
      // Set outputs at once - only possible if all bits are on one port
      if(0) {
//    if(nBytes <= 2){
        uint16_t nBitmask = nLen * 2 - 1;
        // for(int i = 0; i<nLen; i++){ nBitmask |= 1<<i;  }
        nBitmask = nBitmask << nAddr;

        if (nLen < 8){
          SetMultipleCoils(nBitmask, pRxPacket[7]);
        } else{
          SetMultipleCoils(nBitmask, (((uint16_t)pRxPacket[8])<<8) + pRxPacket[7]);
        }
      } else{
        // Set outputs separately
        uint8_t nBitPos=0;
        nByteCount=0;
        for(uint16_t nCurrAddr = nAddr; nCurrAddr < (nAddr + nLen); nCurrAddr++){
          if (SetCoil(nCurrAddr, pRxPacket[7+nByteCount] & (1<<nBitPos))){
            return MODBUS_ERR_ILLEGAL_DATA_VALUE;
          }
          if(nBitPos == 7){
            nBitPos = 0;
            nByteCount++;
          } else {
            nBitPos++;
          }
        }
      }
      
      // Response
      pTxPacket[0] = pRxPacket[0]; // Device ID
      pTxPacket[1] = pRxPacket[1]; // Function code
      pTxPacket[2] = pRxPacket[2]; // Starting address byte 1
      pTxPacket[3] = pRxPacket[3]; // Starting address byte 2
      pTxPacket[4] = pRxPacket[4]; // Quantity of outputs byte 1
      pTxPacket[5] = pRxPacket[5]; // Quantity of outputs byte 2
      nCRC16 = CRC16(pTxPacket, 6);
      pTxPacket[6] = (uint8_t)(nCRC16&0xFF);
      pTxPacket[7] = (uint8_t)(nCRC16>>8);
      Send(pTxPacket, 8);
      return MODBUS_ERR_NO_EXCEPTION;

    default:
      return MODBUS_ERR_ILLEGAL_FUNCTION;
  }

}
