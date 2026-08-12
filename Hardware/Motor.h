#ifndef __MOTOR_H
#define __MOTOR_H
#include "system.h"

void Motor_Init(void);
void Motor_SetPWM(uint8_t n, float Duty);

#endif
