#ifndef _REALTIME_H
#define _REALTIME_H
#include "system.h" 

void REALTIME_INIT(void);
void TIM3_IRQHandler(void);
uint64_t micros(void);
extern uint64_t timer3_overflow;

#endif
