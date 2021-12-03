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
#include <string.h>
/*===========================================================================*/
/* Private variables.                                                        */
/*===========================================================================*/
SPIConfig spislaveconfig = { NULL,NULL,NULL,
                        SPI_IT_TXE | SPI_IT_RXF,
                        SPI_CR0_TMOD_TX_AND_RX,
                        SPI_CR0_DFS_8BITS,
                        SPI_CPOL_Low,
                        SPI_CPHA_1Edge,
                        0,
                        SPI_CR0_FRF_SPI,
                        SPI_NSS_0 };
SPIConfig spimasterconfig = { NULL,GPIOA,GPIOA_SPI1_NSS,
                        SPI_IT_TXE | SPI_IT_RXF,
                        SPI_CR0_TMOD_TX_AND_RX,
                        SPI_CR0_DFS_8BITS,
                        SPI_CPOL_Low,
                        SPI_CPHA_1Edge,
                        96,
                        SPI_CR0_FRF_SPI,
                        SPI_NSS_0 };
UARTConfig uartconfig = { NULL,NULL,NULL,NULL,NULL,NULL,
                          115200,
                          UART_WordLength_8b,
                          UART_StopBits_One,
                          UART_Parity_None,
                          UART_AutoFlowControl_None };
/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

#define TEST_CODE  TRUE
#define SPI_MASTER TRUE
#define FLASH_TEST TRUE

#define FLASH_WRITE_ADDRESS       0x1000
#define FLASH_READ_ADDRESS        FLASH_WRITE_ADDRESS
#define FLASH_SECTOR_TO_ERASE     FLASH_WRITE_ADDRESS
                          
#define PORTAB_LINE_LED1 PAL_LINE(GPIOB, 14U)
#define PORTAB_LINE_LED2 PAL_LINE(GPIOB, 13U)
#define PORTAB_LED_OFF PAL_HIGH
#define PORTAB_LED_ON PAL_LOW

#define PORTAB_LINE_BUTTON PAL_LINE(GPIOA, 0U)
#define PORTAB_BUTTON_PRESSED PAL_LOW
/* Exported functions --------------------------------------------------------*/
uint32_t SpiFlash_ReadIdentification(void);
void SpiFlash_SectorErase(uint32_t address);
void SpiFlash_WriteBuffer(uint32_t address, const uint8_t* buffer, uint32_t length);
void SpiFlash_ReadData(uint32_t address, uint8_t* buffer, uint32_t length);

#if defined(PORTAB_LINE_LED2)
/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  while (true)
  {
    systime_t time = palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED ? 250 : 500;
    palToggleLine(PORTAB_LINE_LED2);
    chThdSleepMilliseconds(time);
  }
}
#endif

#if PAL_USE_WAIT || defined(__DOXYGEN__)

/**
  * @brief  Configure PA0 in interrupt mode
  * @param  None
  * @return None
  */
void EXTI0_Config(void)
{
  // _pal_lld_setgroupmode(GPIOA, GPIO_Pin_0, GPIO_MODE_IN | GPIO_PUPD_DOWN);

  pal_lld_setgroupmode(GPIOA, 1, 0, PAL_WB32_MODE_INPUT | PAL_WB32_PUPDR_PULLDOWN);

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(PORTAB_LINE_BUTTON, PAL_EVENT_MODE_RISING_EDGE);

  nvicEnableVector(EXTI0_IRQn, WB32_IRQ_EXTI0_PRIORITY);
}

uint8_t const tx_buffer[]={ 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,
                      0x11,0x12,0x13,0x14,0xEC,0xBC,0x6C,0xF0,0x71,0xD8,0xE4,0x1D,0x6F,0x7F,0x08,0xD7,
                      0x92,0x61,0x36,0x36,0xE3,0x08,0x7E,0xED,0x44,0x22,0x72,0x36,0x84,0x27,0xF5,0x98,
                      0x95,0x7C,0xFE,0x4F,0x17,0x8A,0xD3,0x6C,0x43,0xB0,0xBC,0xEC,0x07,0x99,0x81,0x29,
                      0x28,0x32,0xFD,0x3B,0x8D,0xCD,0x02,0x97,0xAE,0x80,0xE6,0xC9,0xE5,0x3B,0x9A,0x24,
                      0x97,0x31,0xAF,0x57,0x00,0x5E,0xCC,0x1B,0xEB,0x5F,0x2C,0x07,0x68,0x21,0x72,0xFD,
                      0x8B,0xDE,0x66,0x74,0xB4,0x09,0xD5,0x33,0xD7,0x5E,0x4A,0x42,0x46,0xD9,0x6F,0x5D,
                      0x33,0xCF,0xAD,0xD6,0x62,0x5C,0xF8,0x62,0x92,0x4E,0xE0,0xF0,0x81,0x9A,0xCE,0x94,
                      0x5A,0xCC,0x05,0x67,0xD0,0x3E,0xFE,0xB1,0x51,0xFF,0x57,0x8D,0xB1,0x8D,0xAC,0x32,
                      0xB5,0xD5,0x7F,0xA8,0xB8,0x4A,0x0E,0xD3,0xEF,0x4A,0x15,0x0D,0x40,0x2D,0xA4,0xC6,
                      0x97,0x8D,0x38,0xDA,0xDB,0x54,0xBF,0x6D,0x1C,0xFF,0x54,0xEF,0x82,0x5C,0x40,0xAC,
                      0x68,0xA0,0x91,0xEC,0x1B,0x24,0x1D,0x86,0x77,0x5B,0xB6,0x85,0xAF,0xC1,0xAA,0xAD,
                      0xC2,0xA6,0xA5,0x76,0x4C,0x66,0xA5,0xFF,0x0A,0x99,0xCA,0x1F,0xC3,0x71,0x86,0xA1,
                      0x64,0x85,0xD6,0xA0,0xFB,0xE4,0x68,0x0D,0xDB,0x90,0xC6,0x4B,0x5B,0xB6,0x63,0xE3,
                      0x4B,0x8A,0xAA,0x99,0xDD,0x31,0xD2,0xC1,0x4D,0x4A,0xEA,0xC2,0x42,0x36,0xC7,0x0A,
                      0x28,0xC8,0xBD,0x46,0x52,0x70,0x94,0x23,0x44,0x39,0xD7,0x07,0xCC,0xCC,0x5C,0x81,
                      0x4B,0x2F,0x8C,0xA1,0xDA,0xF4,0x86,0x74,0xB0,0x5A,0x49,0x10,0x1F,0xD4,0x8B,0x7B,
                      0x9C,0x65,0x14,0x8E,0x81,0x28,0x6F,0x50,0xE2,0x4E,0x62,0x55,0xFD,0x27,0x7C,0x87,
                      0x0E,0xAB,0xD0,0xE9,0x09,0xC7,0x9A,0x77,0xD7,0xBD,0x45,0xEE,};
uint8_t rx_buffer[sizeof(tx_buffer)];
/*
 * Application entry point.
 */
int main(void)
{
#if TEST_CODE
  size_t sz = 20;
  __IO msg_t msg;
  uint32_t jedec_id = 0;
#endif /*TEST_CODE*/ 
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  /*
    PA4、PA5、PA6、PA7(QSPI_NSS0、QSPI_SCK、QSPI_MI_IO1、QSPI_MO_IO0)
  */
#if TEST_CODE
  pal_lld_setgroupmode(GPIOA, 0x03, GPIOA_USART1_TX, PAL_MODE_ALTERNATE(7) |PAL_WB32_OTYPE_PUSHPULL |PAL_WB32_PUPDR_PULLUP |PAL_WB32_OSPEED_HIGH);
  uartStart(&UARTD1 , &uartconfig);
  UART1->SFE = 0x01; /*Enable UART FIFO*/
#if (SPI_MASTER | FLASH_TEST) /*SPI_MASTER*/
  pal_lld_setgroupmode(GPIOA, 0x01,  GPIOA_SPI1_NSS, PAL_WB32_MODE_OUTPUT | PAL_WB32_OTYPE_PUSHPULL  | PAL_WB32_OSPEED_HIGH);
  pal_lld_setgroupmode(GPIOA, 0x07,  GPIOA_SPI1_SCK, PAL_MODE_ALTERNATE(5) | PAL_WB32_OTYPE_PUSHPULL  | PAL_WB32_OSPEED_HIGH);
  spiStart(&SPIDQ , &spimasterconfig); 
  spiUnselect(&SPIDQ);
  
#else /*SPI_SlAVE*/
   /*
    PA4 (SPIS1_NSS)    PA5 (SPIS1_SCK)    PA6 (SPIS1_SO)    PA7 (SPIS1_SI)
  */
  pal_lld_setgroupmode(GPIOA, 0x0F, GPIOA_SPI1_NSS, PAL_MODE_ALTERNATE(6) | PAL_WB32_OTYPE_PUSHPULL  | PAL_WB32_OSPEED_HIGH);
  spiStart(&SPIDS1,&spislaveconfig);
#endif 
#endif /*TEST_CODE*/


  /*
   * Configure PA0 in interrupt mode 
   */
  EXTI0_Config();

  /*
   * Configure MCO output 
   */
  RCC->MCOSEL = 0;
  RCC->MCOSEL = 1 << 1;
#if TEST_CODE

  printf("-----------this is SPI test-----------\r\n");

#endif /*TEST_CODE*/  

#if defined(PORTAB_LINE_LED2)
  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
#endif

  /*
   * Normal main() thread activity.
   */
  while (true)
  {
#if TEST_CODE
    printf("\r\nPlease press key1 to continue the test.\r\n");
#endif /*TEST_CODE*/
    /* Waiting for an edge on the button.*/
    palWaitLineTimeout(PORTAB_LINE_BUTTON, TIME_INFINITE);
    chThdSleep(1000);
    palToggleLine(PORTAB_LINE_LED1);

#if TEST_CODE /*  SPI TEST   */

#if !FLASH_TEST /*SPI master and slave communication test*/
    sz = sizeof(rx_buffer);
#if SPI_MASTER /*master*/
    printf("[SPI master device]\r\n");
    
    spiSelect(&SPIDQ);
    spiSend(&SPIDQ, sz, tx_buffer);
    spiUnselect(&SPIDQ);
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
    spiSelect(&SPIDQ);
    spiReceive(&SPIDQ, sz, rx_buffer);
    spiUnselect(&SPIDQ);
    
    if(memcmp(tx_buffer, rx_buffer, sz) == 0)
    {
      printf("SPI master and slave communication test is successful.\r\n");
    }
    else
    {
      printf("SPI master and slave communication test is failed!\r\n");
      printf("rx_buffer:\r\n");
      for(int i = 0;i < sz; i++){
        printf(" %02X ",rx_buffer[i]);
      }
    }
    
    printf("\r\nTest the functionality of the spiExchange()\r\n");
    spiSelect(&SPIDQ);
    memset(rx_buffer, 0, sizeof(rx_buffer));
    spiExchange(&SPIDQ, sizeof(tx_buffer), tx_buffer,rx_buffer);
    spiUnselect(&SPIDQ);
    if(memcmp(tx_buffer, rx_buffer, sz) == 0)
    {
      printf("SPI master and slave communication test is successful.\r\n");
    }
    else
    {
      printf("SPI master and slave communication test is failed!\r\n");
      printf("rx_buffer:\r\n");
      for(int i = 0;i < sz; i++){
        printf(" %02X ",rx_buffer[i]);
      }
    }

    
#else  /*slave*/
    memset(rx_buffer, 0, sizeof(rx_buffer));
    spiReceive(&SPIDS1, sz, rx_buffer);
    spiSend(&SPIDS1, sz, rx_buffer);
    
    spiExchange(&SPIDQ, sizeof(tx_buffer), tx_buffer,rx_buffer);
#endif 
#else /*SPI Flash test*/
    /*SPI Flash communication*/
    jedec_id = SpiFlash_ReadIdentification();
    printf("This flash chip JEDEC ID is 0x%06X\r\n", jedec_id);
    
    /* Erase SPI FLASH Sector to write on */
    SpiFlash_SectorErase(FLASH_SECTOR_TO_ERASE);
    
    /* Write WriteData data to SPI FLASH memory */
    SpiFlash_WriteBuffer(FLASH_WRITE_ADDRESS, tx_buffer, sizeof(tx_buffer));
    
    /* Read data from SPI FLASH memory */
    memset(rx_buffer, 0, sizeof(rx_buffer));
    SpiFlash_ReadData(FLASH_READ_ADDRESS, rx_buffer, sizeof(rx_buffer));

    /* Check the correctness of written data */
    if(memcmp(tx_buffer, rx_buffer, sizeof(tx_buffer)) == 0)
    {
      printf("Flash progam and read operation ok.\r\n");
    }
    else
    {
      printf("Flash progam and read operation failed!\r\n");
    }
#endif /*MS_TEST*/
#endif /*TEST_CODE*/

  }
}

#endif /* PAL_USE_WAIT */

#if !PAL_USE_WAIT && PAL_USE_CALLBACKS

static event_source_t button_pressed_event;
static event_source_t button_released_event;

static void button_cb(void *arg)
{

  (void)arg;

  chSysLockFromISR();
  if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED)
  {
    chEvtBroadcastI(&button_pressed_event);
  }
  else
  {
    chEvtBroadcastI(&button_released_event);
  }
  chSysUnlockFromISR();
}

/*
 * Application entry point.
 */
int main(void)
{
  event_listener_t el0, el1;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Events initialization and registration.*/
  chEvtObjectInit(&button_pressed_event);
  chEvtObjectInit(&button_released_event);
  chEvtRegister(&button_pressed_event, &el0, 0);
  chEvtRegister(&button_released_event, &el1, 1);

#if defined(PORTAB_LINE_LED2)
  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
#endif

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(PORTAB_LINE_BUTTON, PAL_EVENT_MODE_BOTH_EDGES);
  palSetLineCallback(PORTAB_LINE_BUTTON, button_cb, NULL);

  /*
   * Normal main() thread activity.
   */
  while (true)
  {
    eventmask_t events;

    events = chEvtWaitOne(EVENT_MASK(0) | EVENT_MASK(1));
    if (events & EVENT_MASK(0))
    {
      palWriteLine(PORTAB_LINE_LED1, PORTAB_LED_ON);
    }
    if (events & EVENT_MASK(1))
    {
      palWriteLine(PORTAB_LINE_LED1, PORTAB_LED_OFF);
    }
  }
}
#endif /* !PAL_USE_WAIT && PAL_USE_CALLBACKS */

#if !PAL_USE_WAIT && !PAL_USE_CALLBACKS
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

#if defined(PORTAB_LINE_LED2)
  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
#endif

  /*
   * Normal main() thread activity.
   */
  while (true)
  {
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
  }
}
#endif /* !PAL_USE_WAIT && !PAL_USE_CALLBACKS */
