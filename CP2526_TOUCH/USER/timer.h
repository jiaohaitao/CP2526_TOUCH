#ifndef _TIMER_H
#define _TIMER_H

#include "stm32f10x.h"

void Start_Timer(void);
void Stop_Timer(void);
void Timer_Nvic_Configuration(void);
void Timer_Configuration(void);
#endif
