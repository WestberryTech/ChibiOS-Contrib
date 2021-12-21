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
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "flash_spi.h"
#include "spi_master.h"
/*===========================================================================*/
/* Private variables.                                                        */
/*===========================================================================*/

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

#if PAL_USE_WAIT || defined(__DOXYGEN__)

uint8_t spi_master_tx_data[20] =
{
  0x97, 0x8D, 0x38, 0xDA, 0xDB, 0x54, 0xBF, 0x6D, 0x1C, 0xFF,
  0x54, 0xEF, 0x82, 0x5C, 0x40, 0xAC, 0x68, 0xA0, 0x91, 0xEC
};
uint8_t spi_master_rx_buf[20] = {0};

uint8_t watch_buffer[4][256];

#define SPI_FLASH_START_ADDR (SPI_FLASH_SECTOR_START_COUNT * SPI_FLASH_SECTOR_SIZE)

/*
 * Application entry point.
 */
int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  spi_init();

//  SPI_FLASH_Erase();
  
  flash_read_block((uint8_t *)watch_buffer, (void *)0, 1024);
  
  flash_erase_block((void *)0);
//  flash_erase_chip();
  
  flash_read_block((uint8_t *)watch_buffer, (void *)0, 1024);
  
  flash_write_block((uint8_t *)spi_master_tx_data, (void *)0, sizeof(spi_master_tx_data));
  
  flash_read_block((uint8_t *)watch_buffer, (void *)0, 1024);

  /*
   * Normal main() thread activity.
   */
  while (true)
  {

  }
}

#endif /* PAL_USE_WAIT */
