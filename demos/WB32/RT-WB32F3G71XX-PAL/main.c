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

#define PORTAB_LINE_BUTTON1 PAL_LINE(GPIOA, 0U)
#define PORTAB_LINE_BUTTON2 PAL_LINE(GPIOA, 1U)
#define PORTAB_BUTTON_PRESSED PAL_LOW

#define TEST_CODE   TRUE

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
    systime_t time = (palReadLine(PORTAB_LINE_BUTTON1)| !palReadLine(PORTAB_LINE_BUTTON2)) == PORTAB_BUTTON_PRESSED ? 250 : 500;
    palToggleLine(PORTAB_LINE_LED2);
    chThdSleepMilliseconds(time);
  }
}
#endif
#if defined(PORTAB_LINE_BUTTON2)
/*
 * button thread
 */
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg)
{
  (void)arg;
  chRegSetThreadName("wait button2");
  while (true)
  {
    palWaitLineTimeout(PORTAB_LINE_BUTTON2, TIME_INFINITE);
    palWriteLine(PORTAB_LINE_LED1, PORTAB_LED_OFF);
  }
}
#endif
void EXTI0_Config(void)
{
  pal_lld_setgroupmode(GPIOA, 1, 0, PAL_WB32_MODE_INPUT | PAL_WB32_PUPDR_PULLDOWN);

  // /* Enable AFIO clock */
  // RCC->APB1ENR |= (1 << 15) | (1 << 9) | (1 << 10);

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(PORTAB_LINE_BUTTON1, PAL_EVENT_MODE_RISING_EDGE);

   nvicEnableVector(EXTI0_IRQn, WB32_IRQ_EXTI0_PRIORITY);
}
void EXTI1_Config(void)
{
  pal_lld_setgroupmode(GPIOA, 1, 1, PAL_WB32_MODE_INPUT | PAL_WB32_PUPDR_PULLUP);

  // /* Enable AFIO clock */
  // RCC->APB1ENR |= (1 << 15) | (1 << 9) | (1 << 10);

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(PORTAB_LINE_BUTTON2, PAL_EVENT_MODE_FALLING_EDGE);

   nvicEnableVector(EXTI1_IRQn, WB32_IRQ_EXTI1_PRIORITY);
}
#if PAL_USE_WAIT || defined(__DOXYGEN__)

/**
  * @brief  Configure PA0 in interrupt mode
  * @param  None
  * @return None
  */


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

#if TEST_CODE
  /*
   * Configure PA0 PA1 in interrupt mode 
   */
   EXTI0_Config();
   EXTI1_Config();

   palSetLine(PORTAB_LINE_LED1);

#if defined(PORTAB_LINE_BUTTON2)
  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
#endif
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
    /* Waiting for an edge on the button.*/
    palWaitLineTimeout(PORTAB_LINE_BUTTON1, TIME_INFINITE);
    palWriteLine(PORTAB_LINE_LED1, PORTAB_LED_ON);
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
  if (palReadLine(PORTAB_LINE_BUTTON1) == PORTAB_BUTTON_PRESSED)
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

  EXTI0_Config();

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
  palEnableLineEvent(PORTAB_LINE_BUTTON1, PAL_EVENT_MODE_BOTH_EDGES);
  palSetLineCallback(PORTAB_LINE_BUTTON1, button_cb, NULL);

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
