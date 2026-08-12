#include "stm32f10x.h"                  // Device header
#include "PWM.h"

/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
	PWM_Init(999, 23);	//239,9//PWM_Init(u16 Period, u16 psc)
}

void Motor_SetPWM(uint8_t n, float Duty)
{
	uint16_t comparen = Duty*1000;
	if(n==1)
	{
		PWM_SetCompare1((uint16_t)comparen);
	}
	else if(n==2)
	{
		PWM_SetCompare2((uint16_t)comparen);
	}
	else if(n==3)
	{
		PWM_SetCompare3((uint16_t)comparen);
	}
}
