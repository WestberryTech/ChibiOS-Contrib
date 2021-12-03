#pragma once
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

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup board.
 */

/*
 * Board identifier.
 */

#if !defined(WB32F3G71xx) 
  #define WB32F3G71xx  
#endif

/* on-board */
#define GPIOA_PWM1_CH1    8
#define GPIOA_PWM1_CH2    9
#define GPIOA_PWM1_CH3    10
#define GPIOA_PWM1_CH4    11

#define GPIOB_PWM1_CH1N    13
#define GPIOB_PWM1_CH2N    14
#define GPIOB_PWM1_CH3N    15

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
