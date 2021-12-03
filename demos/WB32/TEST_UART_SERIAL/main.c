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
/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

#define PORTAB_LINE_LED1 PAL_LINE(GPIOB, 14U)
#define PORTAB_LINE_LED2 PAL_LINE(GPIOB, 13U)
#define PORTAB_LED_OFF PAL_HIGH
#define PORTAB_LED_ON PAL_LOW

#define PORTAB_LINE_BUTTON PAL_LINE(GPIOA, 0U)
#define PORTAB_BUTTON_PRESSED PAL_LOW

#ifndef SERIAL_DRIVER
#    define SERIAL_DRIVER SD1
#endif

#define UART_MSG_LEN 20
#define UART_PREAMBLE 0x69


static SerialConfig serialConfig =
{
  SERIAL_DEFAULT_BITRATE,
  UART_WordLength_8b,
  UART_StopBits_One,
  UART_Parity_None,
  UART_AutoFlowControl_None
};

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

void uart_init(uint32_t baud) 
{
    static bool is_initialised = false;

    if (!is_initialised) 
    {
        is_initialised = true;

        serialConfig.speed = baud;

        palSetLineMode(PAL_LINE(GPIOA, 9U), PAL_MODE_ALTERNATE(7) | PAL_WB32_OTYPE_PUSHPULL | PAL_WB32_PUPDR_PULLUP | PAL_WB32_OSPEED_HIGH);
        palSetLineMode(PAL_LINE(GPIOA, 10U), PAL_MODE_ALTERNATE(7) | PAL_WB32_OTYPE_PUSHPULL | PAL_WB32_PUPDR_PULLUP | PAL_WB32_OSPEED_HIGH);

        sdStart(&SERIAL_DRIVER, &serialConfig);
    }
}

void uart_putchar(uint8_t c) 
{
  sdPut(&SERIAL_DRIVER, c); 
}

uint8_t uart_getchar(void) 
{
    msg_t res = sdGet(&SERIAL_DRIVER);

    return (uint8_t)res;
}

bool uart_available(void) 
{
  return !sdGetWouldBlock(&SERIAL_DRIVER); 
}
uint8_t msg[UART_MSG_LEN];
uint8_t msg_idx = 0;
static void get_msg(void) 
{
  while (uart_available()) 
  {
    msg[msg_idx] = uart_getchar();
//    printf("idx: %u, recv: %u\n", msg_idx, msg[msg_idx]);
    uart_putchar(msg[msg_idx]);
    if (msg_idx == 0 && (msg[msg_idx] != UART_PREAMBLE)) 
    {
      msg_idx = 0;
    } else if (msg_idx == (UART_MSG_LEN-1)) 
    {
      msg_idx = 0;
    } else 
    {
      msg_idx++;
    }
  }
}

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

/*
 * Application entry point.
 */
int main(void)
{
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
  
  uart_init(115200);
  
  /*
   * Configure PA0 in interrupt mode 
   */
  EXTI0_Config();  
  
  pal_lld_setgroupmode(GPIOB, 3, 13, PAL_WB32_MODE_OUTPUT | PAL_WB32_OTYPE_PUSHPULL);

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
    get_msg();
    chThdSleepMilliseconds(100);
//    /* Waiting for an edge on the button.*/
//    palWaitLineTimeout(PORTAB_LINE_BUTTON, TIME_INFINITE);
//    
//    palToggleLine(PORTAB_LINE_LED1);
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
