#ifndef   __TIMER_H
#define   __TIMER_H
#include "stm32f4xx.h"

#define TIM_X         TIM5
#define TIM_X_Clk     RCC_APB1Periph_TIM5
#define TIM_X_IRQn    TIM5_IRQn         

void TIM_Timer_Init(u32 arr,u16 psc);

#endif   /* __TIMER_H*/

