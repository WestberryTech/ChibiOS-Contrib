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
#include "board.h"
#include <stdio.h>

void SpiFlash_SendCmd(uint8_t cmd)
{
  spiSend(&SPIDQ, 1, &cmd);
}

/**
 * @brief  Read JEDEC ID (9Fh).
 * @param  None
 * @return 24bit JEDEC ID (Eg. the w25q32 id is 0x684016)
 */
uint32_t SpiFlash_ReadIdentification(void)
{
  uint8_t jedec_id[3];
  spiSelect(&SPIDQ);
  
  SpiFlash_SendCmd(0x9F);
  spiReceive(&SPIDQ, 3, jedec_id);
  
  spiUnselect(&SPIDQ);
  return ((jedec_id[0]<<16) | (jedec_id[1]<<8) |(jedec_id[2]));
}

/**
 * @brief  Read Status Register-1 (05h)
 * @param  None
 * @return Status Register-1
 */
uint8_t SpiFlash_ReadStatusReg1(void)
{
  uint8_t status = 0;

  /* Select FLASH: Chip Select pin low */
  spiSelect(&SPIDQ);

  SpiFlash_SendCmd(0x05);    // Read Status Register-1 (05h)
  spiReceive(&SPIDQ, 3, &status);

  /* Deselect FLASH: Chip Select pin high */
  spiUnselect(&SPIDQ);

  return status;
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write operation has completed.
 * @param  None
 * @return None
 */
void SpiFlash_WaitReady(void)
{
  while(SpiFlash_ReadStatusReg1() & 0x01);  // check the WIP bit
}

/**
 * @brief  Enables the write access to the FLASH.
 * @param  None
 * @return None
 */
void SpiFlash_WriteEnable(void)
{
  /* Select FLASH: Chip Select pin low */
  spiSelect(&SPIDQ);
  
  SpiFlash_SendCmd(0x06);    // Write Enable (06h)

  /* Deselect FLASH: Chip Select pin high */
  spiUnselect(&SPIDQ);
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 * @return None
 */
void SpiFlash_ChipErase(void)
{
  SpiFlash_WriteEnable();
  
  /* Select FLASH: Chip Select pin low */
  spiSelect(&SPIDQ);
  
  SpiFlash_SendCmd(0xC7);    // Chip Erase (C7h)

  /* Deselect FLASH: Chip Select pin high */
  spiUnselect(&SPIDQ);
  
  SpiFlash_WaitReady();
}

/**
 * @brief  Erases the specified FLASH sector.
 * @param  address: address of the sector to erase.
 * @return None
 */
void SpiFlash_SectorErase(uint32_t address)
{
  uint8_t addr_buf[3] = {(uint8_t)(address >> 16),(uint8_t)(address >> 8),(uint8_t)(address)};
  SpiFlash_WriteEnable();

  /* Select FLASH: Chip Select pin low */
  spiSelect(&SPIDQ);

  SpiFlash_SendCmd(0x20);    // Sector Erase (20h)
  spiSend(&SPIDQ, 3, addr_buf);

  /* Deselect FLASH: Chip Select pin high */
  spiUnselect(&SPIDQ);

  SpiFlash_WaitReady();
}

/**
 * @brief  Writes data to one page (256B).
 * @note   The number of byte can't exceed the FLASH page remain size.
 * @param  address: FLASH's internal address to write to.
 * @param  buffer: pointer to the buffer containing the data to be written
 *         to the FLASH.
 * @param  length: number of bytes to write to the FLASH.
 * @return None
 */
void SpiFlash_PageProgram(uint32_t address, const uint8_t* buffer, uint16_t length)
{
  uint8_t addr_buf[3] = {(uint8_t)(address >> 16),(uint8_t)(address >> 8),(uint8_t)(address)};
  SpiFlash_WriteEnable();

  /* Select FLASH: Chip Select pin low */
  spiSelect(&SPIDQ);

  SpiFlash_SendCmd(0x02);    // Page Program (02h)

  spiSend(&SPIDQ, 3, addr_buf);
  spiSend(&SPIDQ, length, buffer);

  /* Deselect FLASH: Chip Select pin high */
  spiUnselect(&SPIDQ);

  SpiFlash_WaitReady();
}

/**
 * @brief  Writes block of data to the FLASH.
 * @param  address: FLASH's internal address to write to.
 * @param  buffer: pointer to the buffer containing the data to be written
 *         to the FLASH.
 * @param  length: number of bytes to write to the FLASH.
 * @return None
 */
void SpiFlash_WriteBuffer(uint32_t address, const uint8_t* buffer, uint32_t length)
{
  uint32_t write_len;
  write_len = 256 - (address % 256);    /* calculate the start page remain size */
  if(length <= write_len) write_len = length;

  while(1)
  {
    SpiFlash_PageProgram(address, buffer, write_len);
    if(length == write_len) break;    /* Write complete */
    else {
      buffer += write_len;
      address += write_len;

      length -= write_len;
      if(length > 256) write_len = 256;
      else write_len = length;
    }
  }
}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param  address: FLASH's internal address to read from.
 * @param  buffer: pointer to the buffer that receives the data read from the FLASH.
 * @param  length: number of bytes to read from the FLASH.
 * @return None
 */
void SpiFlash_ReadData(uint32_t address, uint8_t* buffer, uint32_t length)
{
  uint8_t addr_buf[3] = {(uint8_t)(address >> 16),(uint8_t)(address >> 8),(uint8_t)(address)};
  
  /* Select FLASH: Chip Select pin low */
  spiSelect(&SPIDQ);
  
  SpiFlash_SendCmd(0x03);    // Read Data (03h)
  spiSend(&SPIDQ, 3, addr_buf);

  spiReceive(&SPIDQ, length, buffer);

  /* Deselect FLASH: Chip Select pin high */
  spiUnselect(&SPIDQ);
}

