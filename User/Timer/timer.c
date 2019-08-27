/*
Author:SHIELD_QI
Date:2018-06-30
************************************/

#include "timer.h"

void TIM_Timer_Init(u32 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(TIM_X_Clk,ENABLE);                     //ʹ��TIM5ʱ��
	
	//------------------------------ʱ���������ʼ��------------------------------/
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	                //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;                  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM_X,&TIM_TimeBaseInitStructure);           //��ʼ��TIM5
	
	TIM_ITConfig(TIM_X,TIM_IT_Update,ENABLE);                     //����ʱ��5�����ж�
	TIM_Cmd(TIM_X,ENABLE);                                        //ʹ�ܶ�ʱ��5

}

/****************END*****************/
