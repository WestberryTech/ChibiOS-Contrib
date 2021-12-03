/*
    Copyright (C) 2021 Westberry Technology (ChangZhou) Corp., Ltd

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include <string.h>

void ee24c02_ByteWrite(uint8_t data_addr, uint8_t write_data)
{
  uint8_t data_bytes[2] ={data_addr, write_data};
  while(i2cMasterTransmitTimeout(&I2CD2, (0xA0>>1), data_bytes, 2, NULL, 0, TIME_INFINITE)!= MSG_OK);
}

void ee24c02_SequentialRead(uint8_t data_addr, uint8_t* pbuf, uint32_t read_len)
{
  while(i2cMasterTransmitTimeout(&I2CD2, (0xA0>>1), &data_addr, 1, pbuf, read_len, TIME_INFINITE)!= MSG_OK);
}

void ee24c02_PageWrite(uint8_t data_addr, uint8_t* pbuf, uint32_t write_len)
{
  uint8_t data_bytes[17] = {data_addr};
  write_len = write_len > 16? 16 : write_len;
  memcpy(&data_bytes[1], pbuf, write_len);
  while(i2cMasterTransmitTimeout(&I2CD2, (0xA0>>1), data_bytes, (write_len + 1), NULL, 0, TIME_INFINITE)!= MSG_OK);
}
