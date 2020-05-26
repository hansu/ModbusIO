
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

  return (bCRC);
}
