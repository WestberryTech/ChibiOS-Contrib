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
I2CConfig  i2cconfig = {  I2C_INTR_TX_EMPTY | I2C_INTR_TX_ABRT | I2C_INTR_STOP_DET | I2C_INTR_RX_FULL,
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
/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

#define TEST_CODE  TRUE
#define PORTAB_LINE_LED1 PAL_LINE(GPIOB, 14U)
#define PORTAB_LINE_LED2 PAL_LINE(GPIOB, 13U)
#define PORTAB_LED_OFF PAL_HIGH
#define PORTAB_LED_ON PAL_LOW

#define PORTAB_LINE_BUTTON PAL_LINE(GPIOA, 0U)
#define PORTAB_BUTTON_PRESSED PAL_LOW

/* Exported functions --------------------------------------------------------*/
void ee24c02_ByteWrite(uint8_t data_addr, uint8_t write_data);
void ee24c02_PageWrite(uint8_t data_addr, uint8_t* pbuf, uint32_t write_len);
void ee24c02_SequentialRead(uint8_t data_addr, uint8_t* pbuf, uint32_t read_len);

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
  pal_lld_setgroupmode(GPIOA, 1, 0, PAL_WB32_MODE_INPUT | PAL_WB32_PUPDR_PULLDOWN);

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(PORTAB_LINE_BUTTON, PAL_EVENT_MODE_RISING_EDGE);

  nvicEnableVector(EXTI0_IRQn, WB32_IRQ_EXTI0_PRIORITY);
}
#if TEST_CODE
uint8_t ff_buffer[16];
uint8_t rx_buffer[256];
uint8_t tx_buffer[16] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46,};
#endif /*TEST_CODE*/
/*
 * Application entry point.
 */
int main(void)
{
#if TEST_CODE
  uint8_t data_addr;
  __IO msg_t msg;
  size_t ze;
  memset(ff_buffer,0xFF,sizeof(ff_buffer));
#endif  /*TEST_CODE*/
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
#if TEST_CODE
  /*
    PB10(I2C2_SCL)
    PB11(I2C2_SDA)
    PA9(UART1_TX)
    PA10(UART1_RX)
  
  */
  pal_lld_setgroupmode(GPIOB, 3, GPIOB_I2C2_SCL, PAL_MODE_ALTERNATE(4) |PAL_WB32_OTYPE_OPENDRAIN |PAL_WB32_PUPDR_PULLUP |PAL_WB32_OSPEED_HIGH);
    /* Activates the I2C driver 1.*/
  i2cStart(&I2CD2 , &i2cconfig);

  pal_lld_setgroupmode(GPIOA, 3, GPIOA_USART1_TX, PAL_MODE_ALTERNATE(7) |PAL_WB32_OTYPE_PUSHPULL |PAL_WB32_PUPDR_PULLUP |PAL_WB32_OSPEED_HIGH);
  /* Activates the UART driver 2.*/
  uartStart(&UARTD1 , &uartconfig);
  /*Enable UART FIFO*/
  UART1->SFE = 0x01;
  
  printf("----------This is i2c test----------\r\n");
#endif /*TEST_CODE*/  
    
  /*
   * Configure PA0 in interrupt mode 
   */
  EXTI0_Config();

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
    printf("\r\nPress key1 to begin the test...\r\n");
#endif /*TEST_CODE*/

    /* Waiting for an edge on the button.*/
    palWaitLineTimeout(PORTAB_LINE_BUTTON, TIME_INFINITE);
    chThdSleepMilliseconds(100);  
    palToggleLine(PORTAB_LINE_LED1);

#if TEST_CODE
    data_addr = 0x00;
    printf("Communicate with an eeprom_24c02 using synchronous API functions.\r\n");
    ee24c02_ByteWrite(data_addr, tx_buffer[15]);
    printf("\r\nbyte write to [0x%02X] is: %02X \r\n",data_addr, tx_buffer[15]);

    ee24c02_SequentialRead(data_addr, rx_buffer, 1);
    printf("read form [0x%02X] is: %02X \r\n",data_addr, rx_buffer[0]);
 
    printf("\r\nwrite 0xFF for 256 byte to EEPROM...\r\n");
    for(uint32_t iaddr = 0; iaddr < 255; iaddr += 16)
      ee24c02_PageWrite(iaddr, ff_buffer, 16);
    
    
    ee24c02_SequentialRead(0x00, rx_buffer, 256);
    printf("\r\nread 256 bytes form to [0x00] is:");
    for(int i = 0; i < 256; i++) {
      if(i % 16 == 0)
        printf("\r\n[0x%02X]:",i);

      printf(" %02X ",rx_buffer[i]);}

    ze = 16;
    ee24c02_PageWrite(data_addr,tx_buffer, ze);
    printf("\r\n\r\nPage write %d bytes to [0x%02X] is: ",ze, data_addr);
    for(int i = 0; i < ze && i < 16; i++){
      if(i % 16 == 0)
        printf("\r\n[0x%02X]:",i);
      
      printf(" %02X ",tx_buffer[i]);}

    ee24c02_SequentialRead(data_addr, rx_buffer, ze);
    printf("\r\n\r\nread %d bytes form [0x%02X] is: ",ze, data_addr);
    for(int i = 0; i < ze && i < 16; i++){
      if(i % 16 == 0)
        printf("\r\n[0x%02X]:",i);
      
      printf(" %02X ",rx_buffer[i]);}

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
