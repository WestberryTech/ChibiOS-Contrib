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
/*===========================================================================*/
/* Private variables.                                                        */
/*===========================================================================*/


I2CConfig i2cconfig = { I2C_INTR_TX_EMPTY | I2C_INTR_TX_ABRT | I2C_INTR_STOP_DET | I2C_INTR_RX_FULL,
                        I2C_CON_SLAVE_DISABLE | I2C_CON_RESTART_EN | I2C_CON_SPEED_STANDARD | I2C_CON_MASTER_MODE,
                        0xA0 >> 1,  
                        0,  
                        0,  
                        373,  /* tHIGH = (373 + FS_SPKLEN + 7) / 96MHz = 4us */
                        451,  /* tLOW = (451 + 1) / 96MHz = 4.708us */
                        4,    /* tSP = 4 / 96MHz = 41.67ns */
                        24,   /* tSU;DAT = 24 / 96MHz = 250ns */
                        29 }; /* tHD;DAT = 29 / 96MHz = 302.08ns */ 
UARTConfig uartconfig = { NULL,NULL,NULL,NULL,NULL,NULL,
                          115200,
                          UART_WordLength_8b,
                          UART_StopBits_One,
                          UART_Parity_None,
                          UART_AutoFlowControl_None };
SPIConfig spiconfig = { NULL,GPIOA,4,
                        SPI_IT_TXE | SPI_IT_RXF,
                        SPI_CR0_TMOD_TX_AND_RX,
                        SPI_CR0_DFS_8BITS,
                        SPI_CPOL_Low,
                        SPI_CPHA_1Edge,
                        96,
                        SPI_CR0_FRF_SPI,
                        SPI_NSS_0 };
/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

#define PORTAB_LINE_LED1 PAL_LINE(GPIOB, 14U)
#define PORTAB_LINE_LED2 PAL_LINE(GPIOB, 13U)
#define PORTAB_LED_OFF PAL_HIGH
#define PORTAB_LED_ON PAL_LOW

#define PORTAB_LINE_BUTTON PAL_LINE(GPIOA, 0U)
#define PORTAB_BUTTON_PRESSED PAL_LOW

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
  /* Enable GPIOA clock */
  RCC->APB1ENR |= (1 << 15) | (1 << 5);

  // _pal_lld_setgroupmode(GPIOA, GPIO_Pin_0, PAL_WB32_MODE_INPUT | PAL_WB32_PUPDR_PULLDOWN);

  pal_lld_setgroupmode(GPIOA, 1, 0, PAL_WB32_MODE_INPUT | PAL_WB32_PUPDR_PULLDOWN);

  /* Enable AFIO clock */
  RCC->APB1ENR |= (1 << 15) | (1 << 9) | (1 << 10);

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(PORTAB_LINE_BUTTON, PAL_EVENT_MODE_RISING_EDGE);

  nvicEnableVector(EXTI0_IRQn, WB32_IRQ_EXTI0_PRIORITY);
}

uint8_t rx_buffer[256];
uint8_t tx_buffer[256]={0x06,0x07,0x08,0x09,0x0a};
uint8_t spi_master_tx_data[20] =
{
  0x97, 0x8D, 0x38, 0xDA, 0xDB, 0x54, 0xBF, 0x6D, 0x1C, 0xFF,
  0x54, 0xEF, 0x82, 0x5C, 0x40, 0xAC, 0x68, 0xA0, 0x91, 0xEC
};
uint8_t spi_master_rx_buf[20] = {0};

#define DUMMY_BYTE     0xFF

#define SPI_FLASH_CS_LOW()    palClearPort(GPIOA, GPIO_Pin_4)
/* Deselect FLASH: Chip Select pin high */
#define SPI_FLASH_CS_HIGH()   palSetPort(GPIOA, GPIO_Pin_4)

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte: byte to send.
 * @return The value of the received byte.
 */
//uint8_t SPI_FLASH_ReadWriteByte( uint8_t Data )
//{
//  uint8_t tx_buf = Data;
//  uint8_t rx_buf;
//  
//  spiExchange(&SPIDQ,1,&tx_buf,&rx_buf);
//  
//  while(SPIDQ.state != SPI_READY);

//  return rx_buf;
//}

/**
 * @brief  Read JEDEC ID (9Fh).
 * @param  None
 * @return 24bit JEDEC ID (Eg. the w25q32 id is 0xEF4016)
 */
//uint32_t SpiFlash_ReadIdentification( void )
//{
//  uint32_t jedec_id = 0;
//  
//  uint8_t tx_buf[4] = {0x9F,0xFF,0XFF,0XFF};
//  uint8_t rx_buf[4] = {0};

//  /* Select FLASH: Chip Select pin low */
////  SPI_FLASH_CS_LOW();
//  spiSelect(&SPIDQ);
////  chThdSleepMilliseconds(10);

////  SPI_FLASH_ReadWriteByte( 0x9F ); // Read JEDEC ID (9Fh)
////  jedec_id = SPI_FLASH_ReadWriteByte( DUMMY_BYTE );
////  jedec_id <<= 8;
////  jedec_id |= SPI_FLASH_ReadWriteByte( DUMMY_BYTE );
////  jedec_id <<= 8;
////  jedec_id |= SPI_FLASH_ReadWriteByte( DUMMY_BYTE );
//  
////  spiExchange(&SPIDQ,1,tx_buf,rx_buf);
//  spiSend(&SPIDQ,1,tx_buf);
//  spiReceive(&SPIDQ,3,rx_buf);
////  spiExchange(&SPIDQ,3,tx_buf+1,rx_buf+1);
//  
//  jedec_id = (uint32_t)(*(uint32_t*)rx_buf);

//  /* Deselect FLASH: Chip Select pin high */
////  SPI_FLASH_CS_HIGH();
//  spiUnselect(&SPIDQ);
////  chThdSleepMilliseconds(10);

//  return jedec_id;
//}

uint32_t jedec_id;
/*
 * Application entry point.
 */
int main(void)
{
  __IO msg_t msg;
  uint8_t show[100];

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  pal_lld_setgroupmode(GPIOB, 3, 10, PAL_MODE_ALTERNATE(4) |PAL_WB32_OTYPE_OPENDRAIN |PAL_WB32_PUPDR_PULLUP |PAL_WB32_OSPEED_HIGH);
  pal_lld_setgroupmode(GPIOA, 3,  9, PAL_MODE_ALTERNATE(7) |PAL_WB32_OTYPE_PUSHPULL |PAL_WB32_PUPDR_PULLUP |PAL_WB32_OSPEED_HIGH);
//  pal_lld_setgroupmode(GPIOA, 0x1,  4, GPIO_MODE_OUT | PAL_WB32_OTYPE_PUSHPULL  | PAL_WB32_OSPEED_HIGH );
//  SPI_FLASH_CS_HIGH();
//  pal_lld_setgroupmode(GPIOA, 0xf,  4, PAL_MODE_ALTERNATE(5) | PAL_WB32_OTYPE_PUSHPULL  | PAL_WB32_OSPEED_HIGH);
  pal_lld_setgroupmode(GPIOA, 0xf,  4, PAL_MODE_ALTERNATE(6) |PAL_WB32_OTYPE_PUSHPULL |PAL_WB32_OSPEED_HIGH);
  
//  i2cStart(&I2CD2 , &i2cconfig);
  uartStart(&UARTD1 , &uartconfig);
  spiStart(&SPIDS1 , &spiconfig);

  /*
   * Configure PA0 in interrupt mode 
   */
  EXTI0_Config();

  /*
   * Configure MCO output 
   */
  RCC->MCOSEL = 0;
  RCC->MCOSEL = 1 << 1;
  
//  spiReceive(&SPIDS1,20,spi_master_tx_data);
//  chThdSleepMilliseconds(10);
//  spiSend(&SPIDS1,20,spi_master_rx_buf);
spiExchange(&SPIDS1,20,spi_master_tx_data,spi_master_rx_buf);
spiExchange(&SPIDS1,20,spi_master_rx_buf,spi_master_tx_data);
  
//  jedec_id = SpiFlash_ReadIdentification();
  
//  sprintf((char *)show,"jedec_id = 0x%x\n",jedec_id);
//  
//  uartStartSend(&UARTD1 , strlen((const char*)show) , (const char*)show);
  
//  uartStartReceive(&UARTD1 , 10 ,rx_buffer);

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
    /* Waiting for an edge on the button.*/
    palWaitLineTimeout(PORTAB_LINE_BUTTON, TIME_INFINITE);
    
    palToggleLine(PORTAB_LINE_LED1);

//    /* Action depending on button state.*/
//    if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED)
//    {
//      palWriteLine(PORTAB_LINE_LED1, PORTAB_LED_ON);
//    }
//    else
//    {
//      palWriteLine(PORTAB_LINE_LED1, PORTAB_LED_OFF);
//    }
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
