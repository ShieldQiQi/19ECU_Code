/*
Author:SHIELD_QI
Date:2018-06-30
************************************/

#include "timer.h"

void TIM_Timer_Init(u32 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(TIM_X_Clk,ENABLE);                     //使能TIM5时钟
	
	//------------------------------时基机构体初始化------------------------------/
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	                //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;                  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM_X,&TIM_TimeBaseInitStructure);           //初始化TIM5
	
	TIM_ITConfig(TIM_X,TIM_IT_Update,ENABLE);                     //允许定时器5更新中断
	TIM_Cmd(TIM_X,ENABLE);                                        //使能定时器5

}

/****************END*****************/
