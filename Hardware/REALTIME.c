#include "stm32f10x.h"   
#include "REALTIME.h"

uint64_t timer3_overflow;

void REALTIME_INIT(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitStructure.TIM_Period=0XFFFF;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1; //分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //开启定时器中断
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3,ENABLE);
	
	timer3_overflow=0;
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		timer3_overflow++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

uint64_t micros(void)
{
	  uint64_t overflow_count;
    uint64_t timer_count;
    
    // 防止在读取过程中发生溢出
    do {
        overflow_count = timer3_overflow;
        timer_count = TIM_GetCounter(TIM3);
        
        // 检查是否在读取过程中发生了溢出
        if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
            // 如果检测到溢出标志，重新读取
            continue;
        }
    } while (overflow_count !=timer3_overflow);
    
    // 计算总微秒数 = 溢出次数 * 65536 + 当前计数值
    return (overflow_count * 65536) + timer_count;
}
