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

/*===========================================================================*/
/* Private variables.                                                        */
/*===========================================================================*/
UARTConfig uartconfig = { NULL,NULL,NULL,NULL,NULL,NULL,
                          115200,
                          UART_WordLength_8b,
                          UART_StopBits_One,
                          UART_Parity_None,
                          UART_AutoFlowControl_None };
/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

#define LAN_CODE  1
#define UART_SLAVE  0

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

  // _pal_lld_setgroupmode(GPIOA, GPIO_Pin_0, GPIO_MODE_IN | GPIO_PUPD_DOWN);

  pal_lld_setgroupmode(GPIOA, 1, 0, PAL_WB32_MODE_INPUT | PAL_WB32_PUPDR_PULLDOWN);

  /* Enable AFIO clock */
  RCC->APB1ENR |= (1 << 15) | (1 << 9) | (1 << 10);

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(PORTAB_LINE_BUTTON, PAL_EVENT_MODE_RISING_EDGE);

  nvicEnableVector(EXTI0_IRQn, WB32_IRQ_EXTI0_PRIORITY);
}

uint8_t rx_buffer[16];
uint8_t tx_buffer[16]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46,};/*0123456789ABCDEF*/
/*
 * Application entry point.
 */
int main(void)
{
  size_t sz;
  __IO msg_t msg;
  
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  pal_lld_setgroupmode(GPIOA, 3, GPIOA_USART1_TX, PAL_MODE_ALTERNATE(7) | PAL_WB32_OTYPE_PUSHPULL | PAL_WB32_PUPDR_PULLUP | PAL_WB32_OSPEED_HIGH);
  
    /*
   * Activates the UART driver 2.
   */
  uartStart(&UARTD1 , &uartconfig);
  
  /*Enable UART FIFO*/
  UART1->SFE = 0x01;

  /*
   * Configure PA0 in interrupt mode 
   */
  EXTI0_Config();

  /*
   * Configure MCO output 
   */
  RCC->MCOSEL = 0;
  RCC->MCOSEL = 1 << 1;
#if LAN_CODE
#if(UART_SLAVE == 0)
  /*Start a transmission and then stop it*/
  uartStartSend(&UARTD1 , 10 , tx_buffer);
  sz = uartStopSend(&UARTD1);
  printf("\r\nThe number of data frames not transmitted by the stopped transmit operation is %d\r\n",sz);
  
  /*Starts a receive and then stop it*/
  uartStartReceive(&UARTD1 , 10 , rx_buffer);
  sz = uartStopReceive(&UARTD1);
  printf("The number of data frames not received by the stopped receive operation is %d\r\n",sz);
#endif /*UART_SLAVE*/
#endif /*LAN_CODE*/  
  /**/

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

#if LAN_CODE
    /*
   * Blocking API test, send and restart send.
   */
   sz = 12;
#if UART_SLAVE
   msg = uartSendFullTimeout(&UARTD1, &sz, tx_buffer, TIME_INFINITE);
   if (msg != MSG_OK)
     chSysHalt("invalid return code");
#else
  printf("Now start to receive %d data, please use the slave to send data...\r\n",sz);
  msg = uartReceiveTimeout(&UARTD1,&sz,rx_buffer,TIME_INFINITE);

  printf("Send the received data using UartsEndFullTimeout():");
  msg = uartSendFullTimeout(&UARTD1, &sz, rx_buffer, TIME_INFINITE);
  if (msg != MSG_OK)
   chSysHalt("invalid return code");
  printf("\r\nSend the received data using uartSendTimeout():");
  msg = uartSendTimeout(&UARTD1, &sz, rx_buffer, TIME_INFINITE);
  if (msg != MSG_OK)
   chSysHalt("invalid return code");
   
#endif /*UART_SLAVE*/

#endif /*LAN_CODE*/

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
