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

#include <stdio.h>
#include "wb32f3g71xx.h"


#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;



int fputc(int ch, FILE *f) {
  while(!(UART1->LSR & UART_LSR_THRE));
  UART1->THR = ch;
  while(!(UART1->LSR & UART_LSR_TEMT));
  return ch;
}

int fgetc(FILE *f) {
  while(!(UART1->LSR & UART_LSR_DR));
  return (UART1->RBR);
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int ch) {
}

int __backspace()
{
  return 0;
}

void _sys_exit(int return_code) {
  while (1);    /* endless loop */
}
