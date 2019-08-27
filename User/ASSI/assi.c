/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "assi.h"

//无人驾驶状态信号指示灯-输出

void ASSI_GPIO_Config(void)
{		
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd ( ASSI1_GPIO_CLK|ASSI2_GPIO_CLK|ASSI3_GPIO_CLK, ENABLE); 	
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
		GPIO_InitStructure.GPIO_Pin = ASSI1_PIN;
    GPIO_Init(ASSI1_GPIO_PORT, &GPIO_InitStructure);	

		GPIO_InitStructure.GPIO_Pin = ASSI2_PIN;
    GPIO_Init(ASSI2_GPIO_PORT, &GPIO_InitStructure);	

		GPIO_InitStructure.GPIO_Pin = ASSI3_PIN;
    GPIO_Init(ASSI3_GPIO_PORT, &GPIO_InitStructure);	
	
}
