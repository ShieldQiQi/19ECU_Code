/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "asms.h"

//无人驾驶系统主开关-输入模式

void ASMS_GPIO_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd ( ASMS_GPIO_CLK, ENABLE); 	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	GPIO_InitStructure.GPIO_Pin = ASMS_PIN;
	GPIO_Init(ASMS_GPIO_PORT, &GPIO_InitStructure);	
	
}
